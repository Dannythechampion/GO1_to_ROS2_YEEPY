import csv
import math
from pathlib import Path

import pytest

from go1_driver.command_filter import CommandFilter
from go1_driver.execution_monitor import (
    ExecutionGuard,
    ExecutionMonitor,
    ExecutionPolicy,
    ExecutionState,
    ExecutionVerdict,
)

FIELD = Path(__file__).parent / "data" / "execution_20260918.csv"
# Seconds since the first odometry sample of the 2026-09-18 armed run.
NAV2_FIRST_ABORT = 388.5


def drive(monitor, commands, poses, until, step=0.1):
    """Feed timed commands/poses and evaluate every `step` s; return verdicts."""
    events = sorted([(t, "c", v) for t, *v in commands] + [(t, "o", v) for t, *v in poses])
    verdicts = []
    now = events[0][0]
    for stamp, kind, values in events:
        if kind == "c":
            monitor.command(stamp, *values)
        else:
            monitor.pose(stamp, *values)
        while stamp >= now and now <= until:
            verdicts.append((now, monitor.evaluate(now)))
            now = round(now + step, 6)
    return verdicts


def still_robot(start, end, rate=10.0):
    return [(start + index / rate, 1.0, 2.0, 0.3) for index in range(int((end - start) * rate) + 1)]


def test_steady_turn_that_never_happens_is_a_refusal():
    verdicts = drive(ExecutionMonitor(), [(0.0, 0.0, 0.0, 0.4)], still_robot(0.0, 4.0), until=4.0)
    last = verdicts[-1][1]
    assert last.state is ExecutionState.NOT_EXECUTING
    assert math.degrees(last.commanded_turn) == pytest.approx(68.8, abs=1.0)
    assert last.ratio < 0.05


def test_followed_forward_motion_is_ok():
    poses = [(index / 10.0, 0.18 * index / 10.0, 0.0, 0.0) for index in range(41)]
    verdicts = drive(ExecutionMonitor(), [(0.0, 0.2, 0.0, 0.0)], poses, until=4.0)
    assert verdicts[-1][1].state is ExecutionState.OK
    assert verdicts[-1][1].ratio == pytest.approx(0.9, abs=0.05)


def test_motion_without_a_command_means_someone_else_is_driving():
    poses = [(index / 10.0, 0.5 * index / 10.0, 0.0, 0.0) for index in range(41)]
    verdicts = drive(ExecutionMonitor(), [(0.0, 0.0, 0.0, 0.0)], poses, until=4.0)
    assert verdicts[-1][1].state is ExecutionState.UNCOMMANDED_MOTION


def test_standing_noise_is_judged_by_net_not_path():
    """FAST-LIO wandered ~2 m of path per minute while the robot stood still."""
    poses = [(index / 10.0, 0.004 * (index % 2), 0.004 * ((index + 1) % 2), 0.0) for index in range(41)]
    verdicts = drive(ExecutionMonitor(), [(0.0, 0.0, 0.0, 0.0)], poses, until=4.0)
    assert verdicts[-1][1].state is ExecutionState.OK


def test_small_commands_cannot_be_judged_as_refusals():
    verdicts = drive(ExecutionMonitor(), [(0.0, 0.05, 0.0, 0.1)], still_robot(0.0, 4.0), until=4.0)
    assert verdicts[-1][1].state is ExecutionState.OK


def test_odometry_gap_makes_the_window_unknown():
    poses = still_robot(0.0, 1.0) + still_robot(2.0, 4.0)
    verdicts = drive(ExecutionMonitor(), [(0.0, 0.0, 0.0, 0.4)], poses, until=4.0)
    assert verdicts[-1][1].state is ExecutionState.UNKNOWN


def test_time_going_backwards_starts_over():
    monitor = ExecutionMonitor()
    monitor.pose(10.0, 0.0, 0.0, 0.0)
    monitor.command(10.0, 0.0, 0.0, 0.4)
    monitor.pose(5.0, 0.0, 0.0, 0.0)
    assert monitor.evaluate(5.0).state is ExecutionState.UNKNOWN


def test_non_finite_inputs_are_rejected():
    monitor = ExecutionMonitor()
    with pytest.raises(ValueError):
        monitor.command(0.0, float("nan"), 0.0, 0.0)
    with pytest.raises(ValueError):
        monitor.pose(0.0, 0.0, float("inf"), 0.0)


@pytest.mark.parametrize("override", ({"window": 0.0}, {"min_achieved_ratio": 1.0}, {"max_pose_gap": float("nan")}))
def test_policy_rejects_invalid_values(override):
    with pytest.raises(ValueError):
        ExecutionPolicy(**override)


def test_guard_waits_for_persistence_and_then_latches():
    guard = ExecutionGuard(refusal_hold=5.0, uncommanded_hold=1.0)
    refusal = ExecutionVerdict(ExecutionState.NOT_EXECUTING)
    assert guard.update(0.0, refusal) is None
    assert guard.update(4.9, refusal) is None
    assert guard.update(5.0, refusal) is ExecutionState.NOT_EXECUTING
    assert guard.update(6.0, ExecutionVerdict(ExecutionState.OK)) is ExecutionState.NOT_EXECUTING
    guard.clear()
    assert guard.fault is None


def test_guard_restarts_on_any_good_verdict():
    guard = ExecutionGuard(refusal_hold=5.0)
    refusal = ExecutionVerdict(ExecutionState.NOT_EXECUTING)
    guard.update(0.0, refusal)
    guard.update(4.0, ExecutionVerdict(ExecutionState.OK))
    assert guard.update(8.0, refusal) is None
    assert guard.update(13.0, refusal) is ExecutionState.NOT_EXECUTING


def test_guard_acts_on_uncommanded_motion_quickly():
    guard = ExecutionGuard(refusal_hold=5.0, uncommanded_hold=1.0)
    moving = ExecutionVerdict(ExecutionState.UNCOMMANDED_MOTION)
    guard.update(0.0, moving)
    assert guard.update(1.0, moving) is ExecutionState.UNCOMMANDED_MOTION


def field_run():
    commands, poses = [], []
    with FIELD.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            values = (float(row["t"]), float(row["a"]), float(row["b"]), float(row["c"]))
            (commands if row["kind"] == "c" else poses).append(values)
    command_filter = CommandFilter()
    applied = []
    for stamp, vx, vy, yaw in commands:
        filtered = command_filter.filter(vx, vy, yaw)
        applied.append((stamp, filtered.vx, filtered.vy, filtered.yaw))
    return applied, poses


def field_faults():
    """Driver-side decisions over the 09-18 run, one fault per episode."""
    commands, poses = field_run()
    monitor, guard = ExecutionMonitor(), ExecutionGuard()
    faults, quiet_until = [], -math.inf
    for now, verdict in drive(monitor, commands, poses, until=poses[-1][0]):
        if now < quiet_until:
            continue
        fault = guard.update(now, verdict)
        if fault is not None:
            faults.append((now, fault))
            guard.clear()
            monitor.reset()
            quiet_until = now + 10.0
    return faults


def test_field_run_flags_every_real_stall_and_nothing_else():
    faults = field_faults()
    stamps = [stamp for stamp, _fault in faults]
    kinds = [fault for _stamp, fault in faults]
    # 238-250 s: 12 s of commanded in-place rotation, 0 deg turned, mid-goal.
    assert 243.0 < stamps[0] < 248.0 and kinds[0] is ExecutionState.NOT_EXECUTING
    # The stall behind Nav2's first abort, flagged more than 20 s earlier.
    first_abort_stall = next(stamp for stamp in stamps if stamp > 300.0)
    assert first_abort_stall < NAV2_FIRST_ABORT - 20.0
    # The operator driving back by remote.
    assert any(fault is ExecutionState.UNCOMMANDED_MOTION and 400.0 < stamp < 420.0 for stamp, fault in faults)
    # Minutes of good following (176-236 s and 252-356 s) raise nothing.
    assert not any(176.0 <= stamp < 243.0 or 252.0 <= stamp < 356.0 for stamp in stamps)
