"""The tape dock, closed-loop against a Go1 that sways, lags, coasts and stalls.

Every scenario must end -- ARRIVED or ABORTED, commanding zero -- inside the
dock's own timeout. That is the property the 2026-10-25 drive lacked: it
hunted until the operator killed it.
"""

import math
import random
from types import SimpleNamespace

import pytest

from omx_navigation.corridor_dock import CorridorFix, corridor_fix
from omx_navigation.tape_dock import DockPhase, TapeDock, TapeDockPolicy

DT = 0.1


class Go1:
    """Translation-only body with the driver's deadband in front of it.

    `min_speed` is the slowest command the gait actually follows (below it the
    robot treads in place), `delay` the command-to-motion latency, `tau` the
    velocity lag, and while walking the sensor sways `sway` metres at 2 Hz.
    """

    def __init__(self, x, y, yaw=0.0, *, min_speed=0.0, delay=0.15, tau=0.25, sway=0.012,
                 noise=0.003, dropout=0.05, seed=1):
        self.x, self.y, self.yaw = x, y, yaw          # tape midpoint relative to the robot, map-aligned
        self.vx = self.vy = 0.0
        self.min_speed, self.tau, self.sway, self.noise, self.dropout = min_speed, tau, sway, noise, dropout
        self.queue = [(0.0, 0.0)] * max(1, round(delay / DT))
        self.t = 0.0
        self.rng = random.Random(seed)
        self.window = []

    def step(self, cmd_vx, cmd_vy):
        if math.hypot(cmd_vx, cmd_vy) < 0.025:          # go1_driver: STAND
            cmd_vx = cmd_vy = 0.0
        self.queue.append((cmd_vx, cmd_vy))
        want_x, want_y = self.queue.pop(0)
        if math.hypot(want_x, want_y) < self.min_speed:
            want_x = want_y = 0.0
        alpha = DT / (self.tau + DT)
        self.vx += alpha * (want_x - self.vx)
        self.vy += alpha * (want_y - self.vy)
        # Body-frame velocity moves the robot, so the tape moves the other way.
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x -= (c * self.vx - s * self.vy) * DT
        self.y -= (s * self.vx + c * self.vy) * DT
        self.t += DT

    @property
    def walking(self):
        return math.hypot(self.vx, self.vy) > 0.005

    def true_error(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * self.x + s * self.y, -s * self.x + c * self.y

    def fix(self):
        ex, ey = self.true_error()
        if self.walking:
            phase = 2 * math.pi * 2.0 * self.t
            ex += 0.6 * self.sway * math.sin(phase)
            ey += self.sway * math.sin(phase + 1.0)
        ex += self.rng.gauss(0, self.noise)
        ey += self.rng.gauss(0, self.noise)
        # The node merges 0.2 s of frames, which lags and smooths.
        self.window = (self.window + [(ex, ey)])[-2:]
        if self.rng.random() < self.dropout:
            return None
        mx = sum(p[0] for p in self.window) / len(self.window)
        my = sum(p[1] for p in self.window) / len(self.window)
        return CorridorFix(mx, my, -self.yaw, 2.0, 80)


def run(robot, dock=None, *, max_time=None, inputs=lambda t: {}):
    dock = dock or TapeDock()
    dock.start(0.0)
    max_time = max_time or dock.policy.total_timeout + 5
    t, command = 0.0, None
    while t < max_time:
        t = round(t + DT, 6)
        command = dock.update(t, robot.fix(), **inputs(t))
        robot.step(command.vx, command.vy)
        if command.phase in (DockPhase.ARRIVED, DockPhase.ABORTED):
            break
    return dock, command, t


SCENARIOS = [
    dict(x=0.30, y=0.10),
    dict(x=-0.25, y=-0.20),
    dict(x=0.05, y=0.30),
    dict(x=0.34, y=-0.05, yaw=math.radians(15)),
    dict(x=-0.20, y=0.15, yaw=math.radians(-25)),
    dict(x=0.01, y=0.01),
]
GO1S = [
    dict(),                                   # follows anything above the deadband
    dict(min_speed=0.05),                     # ignores the first v_min: must step up
    dict(min_speed=0.07, delay=0.3),          # sluggish and late
    dict(sway=0.025, noise=0.006),            # a wobbly gait
    dict(dropout=0.3),                        # tape often missed
]


@pytest.mark.parametrize("start", SCENARIOS)
@pytest.mark.parametrize("go1", GO1S)
def test_the_dock_always_ends_on_the_line(start, go1):
    robot = Go1(start["x"], start["y"], start.get("yaw", 0.0), **go1)
    dock, command, t = run(robot)
    ex, ey = robot.true_error()
    assert command.phase is DockPhase.ARRIVED, (command.reason, dock.rounds)
    assert (command.vx, command.vy) == (0.0, 0.0)
    # Judged against the truth, not against what the dock measured.
    assert abs(ex) <= dock.policy.x_tolerance + 0.008, dock.rounds
    assert abs(ey) <= dock.policy.y_tolerance + 0.008, dock.rounds
    assert t < 60.0
    assert len(dock.rounds) <= dock.policy.max_rounds


def test_arrival_is_never_judged_while_walking():
    robot = Go1(0.30, 0.0, sway=0.03)
    dock = TapeDock()
    dock.start(0.0)
    t, last_phase = 0.0, None
    while t < 60:
        t = round(t + DT, 6)
        command = dock.update(t, robot.fix())
        if command.phase is DockPhase.ARRIVED:
            # The cycle before arrival was a standing measurement.
            assert last_phase is DockPhase.MEASURE
            assert not robot.walking
            break
        last_phase = command.phase
        robot.step(command.vx, command.vy)
    assert command.phase is DockPhase.ARRIVED


def test_a_robot_that_will_not_move_ends_aborted_not_hunting():
    robot = Go1(0.30, 0.0, min_speed=0.5)            # never walks at dock speeds
    dock, command, t = run(robot)
    assert command.phase is DockPhase.ABORTED
    assert (command.vx, command.vy) == (0.0, 0.0)
    assert t <= dock.policy.total_timeout + DT
    # It stepped its floor up before giving up.
    assert dock.rounds[-1]["v_min"] == pytest.approx(dock.policy.v_min_limit)


def test_losing_the_tape_aborts_with_zero():
    robot = Go1(0.20, 0.0, dropout=1.0)
    dock, command, t = run(robot)
    assert command.phase is DockPhase.ABORTED and "never seen" in command.reason
    assert t <= dock.policy.detection_timeout + 2 * DT


def test_a_ready_flicker_pauses_and_recovers():
    robot = Go1(0.25, 0.05)
    dock, command, _ = run(robot, inputs=lambda t: {"localization_ready": not (3.0 <= t < 5.0)})
    assert command.phase is DockPhase.ARRIVED


def test_ready_gone_for_good_aborts():
    robot = Go1(0.25, 0.05)
    dock, command, t = run(robot, inputs=lambda t: {"localization_ready": t < 2.0})
    assert command.phase is DockPhase.ABORTED and "READY" in command.reason
    assert t < 2.0 + dock.policy.ready_grace + 1.0


def test_manual_override_stops_at_once():
    robot = Go1(0.25, 0.05)
    dock, command, t = run(robot, inputs=lambda t: {"manual_override": t >= 2.5})
    assert command.phase is DockPhase.ABORTED and command.reason == "manual override"
    assert t == pytest.approx(2.5, abs=DT)


def test_no_command_is_ever_inside_the_driver_deadband():
    """A command the driver would zero is a stall the dock cannot see."""
    robot = Go1(0.30, 0.20)
    dock = TapeDock()
    dock.start(0.0)
    t = 0.0
    while t < 60:
        t = round(t + DT, 6)
        command = dock.update(t, robot.fix())
        speed = math.hypot(command.vx, command.vy)
        assert speed == 0.0 or speed >= dock.policy.v_min - 1e-9
        robot.step(command.vx, command.vy)
        if command.phase in (DockPhase.ARRIVED, DockPhase.ABORTED):
            break


def test_through_the_real_detector():
    """Same loop with the fix taken from a synthetic Livox cloud, not handed in."""
    robot = Go1(0.22, -0.12, dropout=0.0)

    def cloud():
        ex, ey = robot.true_error()
        points = []
        for wall in (1.0, -1.0):
            for i in range(40):
                points.append((ex + random.gauss(0, 0.004), ey + wall, -0.10 + 0.3 * i / 39, 250.0))
        return points

    dock = TapeDock()
    dock.start(0.0)
    t = 0.0
    while t < 60:
        t = round(t + DT, 6)
        command = dock.update(t, corridor_fix(cloud()))
        robot.step(command.vx, command.vy)
        if command.phase in (DockPhase.ARRIVED, DockPhase.ABORTED):
            break
    assert command.phase is DockPhase.ARRIVED
    assert abs(robot.true_error()[0]) <= 0.025


def test_policy_refuses_a_floor_inside_the_deadband():
    with pytest.raises(ValueError, match="deadband"):
        TapeDockPolicy(v_min=0.02, v_min_step=0.01, v_min_limit=0.05)
    with pytest.raises(ValueError):
        TapeDockPolicy(x_aim=0.03)
