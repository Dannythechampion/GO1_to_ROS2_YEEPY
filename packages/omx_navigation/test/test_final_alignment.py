import math

import pytest

from omx_navigation.final_alignment import (
    AlignmentPhase,
    AlignmentPolicy,
    FinalAligner,
    ScanCheck,
)
from omx_navigation.scan_map_quality import Pose2D


GOAL = Pose2D(1.0, -2.5, 0.0)
DT = 0.1


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Robot:
    """A Go1 as the 2026-09-18 / 10-25 bags show it.

    It turns at `yaw_gain` of the command (65-73 % measured), ignores yaw
    commands below `yaw_floor` (about half of the ~0.1 rad/s commands did
    nothing), acts on each command one cycle late, and slides sideways while
    turning in place. `bias` is how far the tracked pose is from the truth:
    the scan sees `true = tracked + bias`.
    """

    def __init__(self, pose, *, yaw_gain=0.7, yaw_floor=0.12, linear_gain=0.9, turn_slide=0.05,
                 bias=Pose2D(0.0, 0.0, 0.0), frozen=False):
        self.true = pose
        self.yaw_gain = yaw_gain
        self.yaw_floor = yaw_floor
        self.linear_gain = linear_gain
        self.turn_slide = turn_slide
        self.bias = bias
        self.frozen = frozen
        self.pending = (0.0, 0.0, 0.0)
        self.commands = []

    def tracked(self):
        return Pose2D(self.true.x - self.bias.x, self.true.y - self.bias.y, wrap(self.true.yaw - self.bias.yaw))

    def act(self, command):
        self.commands.append(command)
        vx, vy, rate = self.pending
        self.pending = command
        if self.frozen:
            return
        turned = self.yaw_gain * rate * DT if abs(rate) >= self.yaw_floor else 0.0
        c, s = math.cos(self.true.yaw), math.sin(self.true.yaw)
        moved = self.linear_gain * DT
        self.true = Pose2D(
            self.true.x + (c * vx - s * vy) * moved + self.turn_slide * abs(turned),
            self.true.y + (s * vx + c * vy) * moved,
            wrap(self.true.yaw + turned),
        )


def run(robot, *, goal=GOAL, policy=None, seconds=60.0, overlap=0.9, check_period=2.0, offsets=None):
    """Drive until the aligner finishes; scan checks arrive every `check_period`."""
    aligner = FinalAligner(policy)
    now = 0.0
    step = aligner.start(goal, robot.tracked(), now)
    scan = None
    next_check = check_period
    count = 0
    while not step.done and now < seconds:
        robot.act(step.command)
        now = round(now + DT, 6)
        if now >= next_check:
            offset = offsets[count] if offsets is not None and count < len(offsets) else robot.bias
            scan = ScanCheck(now, overlap, offset)
            count += 1
            next_check += check_period
        step = aligner.step(robot.tracked(), now, scan)
    return step, aligner


def true_error(robot, goal=GOAL):
    return math.hypot(goal.x - robot.true.x, goal.y - robot.true.y), abs(wrap(goal.yaw - robot.true.yaw))


def test_turns_first_then_shifts_then_confirms_on_a_robot_that_slides_while_turning():
    """10-25 goal 1: 0.2 m off and 90 deg from the goal heading, where DWB
    turned back and forth for 27 s."""
    robot = Robot(Pose2D(1.19, -2.48, math.radians(-90.0)))

    step, _ = run(robot)

    assert step.phase is AlignmentPhase.SUCCEEDED, step.reason
    distance, yaw = true_error(robot)
    policy = AlignmentPolicy()
    assert distance <= policy.xy_tolerance
    assert yaw <= policy.yaw_tolerance + math.radians(1.0)
    first_shift = next(index for index, command in enumerate(robot.commands) if command[:2] != (0.0, 0.0))
    turning = [command for command in robot.commands[:first_shift] if command[2] != 0.0]
    assert turning, "the heading is corrected before any translation"
    assert all(command[2] == 0.0 for command in robot.commands if command[:2] != (0.0, 0.0))


def test_yaw_rate_is_never_below_the_rate_the_robot_actually_follows():
    aligner = FinalAligner()
    step = aligner.start(GOAL, Pose2D(GOAL.x, GOAL.y, math.radians(-6.0)), 0.0)

    assert step.phase is AlignmentPhase.TURN
    assert step.command == (0.0, 0.0, AlignmentPolicy().min_yaw_rate)


def test_shift_is_commanded_in_the_body_frame():
    aligner = FinalAligner()
    # Facing +y with the goal 0.2 m along +x map: the goal is to the robot's right.
    step = aligner.start(Pose2D(1.2, 0.0, math.pi / 2), Pose2D(1.0, 0.0, math.pi / 2), 0.0)

    assert step.phase is AlignmentPhase.SHIFT
    vx, vy, yaw = step.command
    assert abs(vx) < 1e-9 and vy < 0.0 and yaw == 0.0
    assert math.hypot(vx, vy) == pytest.approx(AlignmentPolicy().max_linear_speed)


def test_realigns_to_the_scan_matched_pose_when_the_tracked_pose_is_off():
    """10-25: the live scan disagreed with the tracked pose by up to 0.34 m."""
    robot = Robot(Pose2D(1.15, -2.45, math.radians(-10.0)), bias=Pose2D(0.08, -0.04, math.radians(1.5)))

    step, _ = run(robot)

    assert step.phase is AlignmentPhase.SUCCEEDED, step.reason
    distance, yaw = true_error(robot)
    assert distance <= AlignmentPolicy().xy_tolerance
    assert yaw <= AlignmentPolicy().yaw_tolerance + math.radians(1.0)


def test_a_robot_that_does_not_turn_fails_instead_of_hunting():
    """09-18: commanded -0.4 rad/s for up to 20 s while the heading stayed put."""
    robot = Robot(Pose2D(1.1, -2.5, math.radians(-60.0)), frozen=True)

    step, _ = run(robot)

    assert step.phase is AlignmentPhase.FAILED
    assert "did not turn" in step.reason


def test_arrival_needs_scan_checks_taken_after_the_robot_stopped():
    aligner = FinalAligner()
    stale = ScanCheck(0.5, 0.95, Pose2D(0.0, 0.0, 0.0))
    step = aligner.start(GOAL, GOAL, 1.0)
    assert step.phase is AlignmentPhase.VERIFY

    now = 1.0
    while not step.done:
        now = round(now + DT, 6)
        step = aligner.step(GOAL, now, stale)

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "no scan match arrived while stopped at the goal"


def test_low_scan_overlap_at_the_goal_is_not_an_arrival():
    robot = Robot(Pose2D(GOAL.x, GOAL.y, 0.0))

    step, _ = run(robot, overlap=0.62)

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "scan match 0.62 is below 0.85 at the goal"


def test_scan_checks_that_disagree_are_not_trusted():
    robot = Robot(Pose2D(GOAL.x, GOAL.y, 0.0))
    offsets = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.15, 0.0, 0.0)] * 10

    step, _ = run(robot, offsets=offsets)

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "scan matches at the goal disagree with each other"


def test_a_large_scan_disagreement_is_a_localization_problem_not_a_target():
    robot = Robot(Pose2D(GOAL.x, GOAL.y, 0.0), bias=Pose2D(0.34, 0.0, 0.0))

    step, _ = run(robot)

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason.startswith("scan disagrees with the tracked pose by 0.34 m")


def test_refuses_to_start_far_from_the_goal():
    step = FinalAligner().start(GOAL, Pose2D(GOAL.x + 0.8, GOAL.y, 0.0), 0.0)

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "started 0.80 m from the goal"
    assert step.command == (0.0, 0.0, 0.0)


def test_losing_the_pose_stops_and_then_fails():
    aligner = FinalAligner()
    step = aligner.start(GOAL, Pose2D(GOAL.x, GOAL.y, 0.5), 0.0)
    assert step.command[2] != 0.0

    step = aligner.step(None, 0.3)
    assert step.command == (0.0, 0.0, 0.0) and not step.done
    step = aligner.step(None, 0.6)
    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "robot pose is unavailable"


def test_a_robot_that_overshoots_every_turn_fails_and_says_so():
    # At 8x the commanded rate every cycle jumps past the 3 deg tolerance.
    robot = Robot(Pose2D(GOAL.x, GOAL.y, math.radians(20.0)), yaw_gain=8.0, yaw_floor=0.0)

    step, _ = run(robot)

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "robot moved but came no closer for 3 s"


def test_motion_budget_bounds_a_run_that_does_not_converge():
    # The turn slides the robot off the goal, so a shift is needed after it.
    robot = Robot(Pose2D(GOAL.x, GOAL.y, math.radians(30.0)), turn_slide=0.6)

    step, _ = run(robot, policy=AlignmentPolicy(max_motions=1))

    assert step.phase is AlignmentPhase.FAILED
    assert step.reason == "not converged after the budget of 1 motions"
    assert all(command[:2] == (0.0, 0.0) for command in robot.commands)


def test_finished_aligner_only_commands_stop():
    robot = Robot(Pose2D(GOAL.x, GOAL.y, 0.0))
    step, aligner = run(robot)
    assert step.phase is AlignmentPhase.SUCCEEDED

    again = aligner.step(Pose2D(GOAL.x + 0.2, GOAL.y, 1.0), 99.0)
    assert again.phase is AlignmentPhase.SUCCEEDED
    assert again.command == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("override", (
    {"xy_tolerance": 0.0},
    {"min_yaw_rate": 0.5, "max_yaw_rate": 0.3},
    {"min_linear_speed": 0.2, "max_linear_speed": 0.1},
    {"handoff_radius": 0.7},
    {"min_overlap": 1.2},
    {"max_motions": 2.5},
    {"verify_timeout": math.nan},
    {"settle_time": True},
))
def test_policy_rejects_unusable_values(override):
    with pytest.raises(ValueError):
        AlignmentPolicy(**override)
