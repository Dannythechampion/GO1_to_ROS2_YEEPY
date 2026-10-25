"""ROS-independent final alignment: from "Nav2 brought it near" to "on the goal".

Nav2 never finished an arrival on this robot. Recovered from the bags of
2026-09-18 (`posegraph_20260918T114201Z_154779`) and 2026-10-25
(`posegraph_20261025T102846Z_151370`):

* Within 0.2 m of a goal DWB alternated between RotateToGoal (turn to the goal
  heading) and its path critics (turn to face the goal point), so the robot
  turned back and forth in place: 27 s on 10-25, 86 s on 09-18. RotateToGoal's
  `stateful` latch does not survive Humble's 1 Hz replan, because every new
  plan resets the critics.
* Turning in place moves a legged base: 0.10 m (10-25) and up to 0.45 m (09-18)
  of displacement while only turning. Heading is therefore corrected first and
  position last, and position is corrected without turning.
* Yaw commands near 0.1 rad/s produced no turn about half the time; from
  0.2 rad/s the robot turned at 65-73 % of the command. Turns are closed on the
  measured heading and never slowed below `min_yaw_rate`.
* Standing still the tracked pose varies by about 1.5 cm, but the live scan
  disagreed with it by up to 0.34 m on 10-25. Arrival is confirmed against
  scan matches taken while the robot stands at the goal, and the robot is
  re-aligned to the scan-matched pose when the two disagree.

Every motion is bounded: a stalled robot, a run that does not converge, a
scan that does not confirm the goal, or a robot that ends up further than
`max_start_distance` from the goal fails the alignment instead of retrying.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from enum import Enum

from omx_navigation.scan_map_quality import Pose2D


class AlignmentPhase(str, Enum):
    TURN = "TURN"
    SHIFT = "SHIFT"
    SETTLE = "SETTLE"
    VERIFY = "VERIFY"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


_DONE = (AlignmentPhase.SUCCEEDED, AlignmentPhase.FAILED)


@dataclass(frozen=True)
class AlignmentPolicy:
    # Nav2 hands over once the tracked pose is this close. It must exceed
    # Nav2's xy_goal_tolerance (0.20 m) so DWB never reaches RotateToGoal.
    handoff_radius: float = 0.35
    # Alignment refuses to start, and gives up, beyond this distance.
    max_start_distance: float = 0.60
    xy_tolerance: float = 0.05
    yaw_tolerance: float = math.radians(3.0)
    min_yaw_rate: float = 0.20
    max_yaw_rate: float = 0.35
    yaw_gain: float = 1.0
    # Above the driver's 0.025 m/s linear deadband. Not yet measured on the
    # Go1 as it is loaded now; lateral response in particular is unmeasured.
    min_linear_speed: float = 0.05
    max_linear_speed: float = 0.10
    linear_gain: float = 0.8
    # While shifting, a heading error past this many yaw tolerances stops the
    # shift so the heading is corrected first.
    shift_yaw_limit: float = 2.0
    # Stand still this long before judging where the robot ended up.
    settle_time: float = 1.0
    # A motion that has not reduced its error by the progress amount within
    # stall_time is a robot not following commands.
    stall_time: float = 3.0
    stall_turn_progress: float = math.radians(3.0)
    stall_shift_progress: float = 0.02
    # Turn and shift motions in total, re-alignments after a scan check included.
    max_motions: int = 8
    # Scan confirmation. The supervisor checks every 2 s; two checks taken while
    # stopped must agree before either is trusted.
    min_overlap: float = 0.85
    verify_checks: int = 2
    check_agreement: float = 0.05
    check_agreement_yaw: float = math.radians(2.0)
    # A scan-matched pose this far from the tracked one is not a correction
    # to drive by; it is a localization problem.
    max_scan_offset: float = 0.30
    max_scan_offset_yaw: float = math.radians(10.0)
    verify_timeout: float = 10.0
    pose_timeout: float = 0.5
    timeout: float = 90.0

    def __post_init__(self) -> None:
        for item in fields(self):
            value = getattr(self, item.name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{item.name} must be finite and positive")
        for name in ("max_motions", "verify_checks"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be a positive integer")
        if self.min_overlap > 1.0:
            raise ValueError("min_overlap must be within (0.0, 1.0]")
        if self.min_yaw_rate > self.max_yaw_rate:
            raise ValueError("min_yaw_rate must not exceed max_yaw_rate")
        if self.min_linear_speed > self.max_linear_speed:
            raise ValueError("min_linear_speed must not exceed max_linear_speed")
        if not self.xy_tolerance < self.handoff_radius <= self.max_start_distance:
            raise ValueError("xy_tolerance < handoff_radius <= max_start_distance is required")


@dataclass(frozen=True)
class ScanCheck:
    """One supervisor scan match: `offset` is scan-matched minus tracked pose."""

    stamp: float
    overlap: float
    offset: Pose2D


@dataclass(frozen=True)
class AlignmentStep:
    phase: AlignmentPhase
    # Body-frame velocity command (vx, vy, yaw rate).
    command: tuple[float, float, float]
    error_xy: float
    error_yaw: float
    reason: str = ""

    @property
    def done(self) -> bool:
        return self.phase in _DONE


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _finite_pose(pose: Pose2D) -> bool:
    return all(isinstance(value, (int, float)) and math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))


class FinalAligner:
    """Turn, then shift, then confirm with the scan; one goal at a time."""

    def __init__(self, policy: AlignmentPolicy | None = None) -> None:
        self.policy = policy or AlignmentPolicy()
        self._phase = AlignmentPhase.FAILED
        self._reason = "not started"
        self._target: Pose2D | None = None

    @property
    def phase(self) -> AlignmentPhase:
        return self._phase

    @property
    def done(self) -> bool:
        return self._phase in _DONE

    def start(self, goal: Pose2D, pose: Pose2D, now: float) -> AlignmentStep:
        if not _finite_pose(goal) or not _finite_pose(pose) or not math.isfinite(now):
            raise ValueError("goal, pose and time must be finite")
        self._goal = goal
        self._target = goal
        self._started_at = now
        self._motions = 0
        self._pose_seen_at = now
        self._checks: list[ScanCheck] = []
        self._last_check_stamp = None
        self._reason = ""
        distance, _ = self._errors(pose)
        if distance > self.policy.max_start_distance:
            return self._fail(pose, f"started {distance:.2f} m from the goal")
        return self._decide(pose, now)

    def step(self, pose: Pose2D | None, now: float, scan: ScanCheck | None = None) -> AlignmentStep:
        """Advance with the latest tracked pose (None when unavailable)."""
        policy = self.policy
        if self.done:
            return self._result(pose)
        if now - self._started_at > policy.timeout:
            return self._fail(pose, f"did not finish within {policy.timeout:.0f} s")
        if pose is None or not _finite_pose(pose):
            if now - self._pose_seen_at > policy.pose_timeout:
                return self._fail(None, "robot pose is unavailable")
            return AlignmentStep(self._phase, (0.0, 0.0, 0.0), math.nan, math.nan, "waiting for the robot pose")
        self._pose_seen_at = now
        distance, yaw_error = self._errors(pose)
        if distance > policy.max_start_distance:
            return self._fail(pose, f"moved {distance:.2f} m away from the goal")

        if self._phase is AlignmentPhase.TURN:
            return self._turn(pose, now, distance, yaw_error)
        if self._phase is AlignmentPhase.SHIFT:
            return self._shift(pose, now, distance, yaw_error)
        if self._phase is AlignmentPhase.SETTLE:
            if now < self._settle_until:
                return self._stopped(distance, yaw_error)
            return self._decide(pose, now)
        return self._verify(pose, now, scan, distance, yaw_error)

    # -- phases ---------------------------------------------------------------

    def _turn(self, pose: Pose2D, now: float, distance: float, yaw_error: float) -> AlignmentStep:
        policy = self.policy
        magnitude = abs(yaw_error)
        if magnitude <= policy.yaw_tolerance:
            return self._settle(now, distance, yaw_error)
        stalled = self._stalled(pose, now, magnitude, policy.stall_turn_progress, "turn")
        if stalled:
            return self._fail(pose, stalled)
        rate = min(policy.max_yaw_rate, max(policy.min_yaw_rate, policy.yaw_gain * magnitude))
        return AlignmentStep(AlignmentPhase.TURN, (0.0, 0.0, math.copysign(rate, yaw_error)), distance, yaw_error)

    def _shift(self, pose: Pose2D, now: float, distance: float, yaw_error: float) -> AlignmentStep:
        policy = self.policy
        if distance <= policy.xy_tolerance or abs(yaw_error) > policy.shift_yaw_limit * policy.yaw_tolerance:
            return self._settle(now, distance, yaw_error)
        stalled = self._stalled(pose, now, distance, policy.stall_shift_progress, "move")
        if stalled:
            return self._fail(pose, stalled)
        dx, dy = self._target.x - pose.x, self._target.y - pose.y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        forward, left = c * dx + s * dy, -s * dx + c * dy
        speed = min(policy.max_linear_speed, max(policy.min_linear_speed, policy.linear_gain * distance))
        return AlignmentStep(
            AlignmentPhase.SHIFT, (forward / distance * speed, left / distance * speed, 0.0), distance, yaw_error
        )

    def _verify(self, pose: Pose2D, now: float, scan: ScanCheck | None,
                distance: float, yaw_error: float) -> AlignmentStep:
        policy = self.policy
        # A pose that moves while standing (a pushed localization correction,
        # or the robot nudged) invalidates the checks gathered so far.
        moved = math.hypot(pose.x - self._verify_pose.x, pose.y - self._verify_pose.y)
        if moved > policy.check_agreement or abs(_wrap(pose.yaw - self._verify_pose.yaw)) > policy.check_agreement_yaw:
            self._begin_verify(pose, now)
        if (
            scan is not None
            and scan.stamp >= self._verify_since
            and scan.stamp != self._last_check_stamp
            and math.isfinite(scan.overlap)
            and _finite_pose(scan.offset)
        ):
            self._last_check_stamp = scan.stamp
            self._checks = (self._checks + [scan])[-policy.verify_checks:]
        if len(self._checks) >= policy.verify_checks and self._checks_agree():
            return self._judge(pose, now)
        if now - self._verify_since > policy.verify_timeout:
            if len(self._checks) >= policy.verify_checks:
                return self._fail(pose, "scan matches at the goal disagree with each other")
            return self._fail(pose, "no scan match arrived while stopped at the goal")
        return self._stopped(distance, yaw_error)

    def _judge(self, pose: Pose2D, now: float) -> AlignmentStep:
        policy = self.policy
        count = len(self._checks)
        overlap = sum(check.overlap for check in self._checks) / count
        reference = self._checks[-1].offset.yaw
        offset = Pose2D(
            sum(check.offset.x for check in self._checks) / count,
            sum(check.offset.y for check in self._checks) / count,
            _wrap(reference + sum(_wrap(check.offset.yaw - reference) for check in self._checks) / count),
        )
        if overlap < policy.min_overlap:
            return self._fail(pose, f"scan match {overlap:.2f} is below {policy.min_overlap:.2f} at the goal")
        offset_distance = math.hypot(offset.x, offset.y)
        if offset_distance > policy.max_scan_offset or abs(offset.yaw) > policy.max_scan_offset_yaw:
            return self._fail(
                pose,
                f"scan disagrees with the tracked pose by {offset_distance:.2f} m, "
                f"{math.degrees(offset.yaw):+.1f} deg",
            )
        matched = Pose2D(pose.x + offset.x, pose.y + offset.y, _wrap(pose.yaw + offset.yaw))
        distance = math.hypot(self._goal.x - matched.x, self._goal.y - matched.y)
        yaw_error = _wrap(self._goal.yaw - matched.yaw)
        if distance <= policy.xy_tolerance and abs(yaw_error) <= policy.yaw_tolerance:
            self._phase = AlignmentPhase.SUCCEEDED
            self._reason = f"scan match {overlap:.2f}"
            return AlignmentStep(AlignmentPhase.SUCCEEDED, (0.0, 0.0, 0.0), distance, yaw_error, self._reason)
        # Drive the tracked pose to where it must be for the scan-matched pose
        # to land on the goal.
        self._target = Pose2D(self._goal.x - offset.x, self._goal.y - offset.y, _wrap(self._goal.yaw - offset.yaw))
        return self._decide(pose, now)

    # -- helpers --------------------------------------------------------------

    def _decide(self, pose: Pose2D, now: float) -> AlignmentStep:
        """Choose the next motion from a standing robot, or start confirming."""
        policy = self.policy
        distance, yaw_error = self._errors(pose)
        if abs(yaw_error) > policy.yaw_tolerance:
            return self._begin_motion(AlignmentPhase.TURN, pose, now, distance, yaw_error, abs(yaw_error))
        if distance > policy.xy_tolerance:
            return self._begin_motion(AlignmentPhase.SHIFT, pose, now, distance, yaw_error, distance)
        self._begin_verify(pose, now)
        return self._stopped(distance, yaw_error)

    def _begin_motion(self, phase, pose, now, distance, yaw_error, magnitude) -> AlignmentStep:
        if self._motions >= self.policy.max_motions:
            return self._fail(pose, f"not converged after the budget of {self.policy.max_motions} motions")
        self._motions += 1
        self._phase = phase
        self._progress = (now, magnitude)
        self._travel = 0.0
        self._last_pose = pose
        return self._turn(pose, now, distance, yaw_error) if phase is AlignmentPhase.TURN else self._shift(
            pose, now, distance, yaw_error
        )

    def _begin_verify(self, pose: Pose2D, now: float) -> None:
        self._phase = AlignmentPhase.VERIFY
        self._verify_since = now
        self._verify_pose = pose
        self._checks = []

    def _settle(self, now: float, distance: float, yaw_error: float) -> AlignmentStep:
        self._phase = AlignmentPhase.SETTLE
        self._settle_until = now + self.policy.settle_time
        return self._stopped(distance, yaw_error)

    def _stalled(self, pose: Pose2D, now: float, magnitude: float, progress: float, verb: str) -> str:
        """Why the current motion has stopped making progress, or "" while it has not."""
        if verb == "turn":
            self._travel += abs(_wrap(pose.yaw - self._last_pose.yaw))
        else:
            self._travel += math.hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y)
        self._last_pose = pose
        since, reference = self._progress
        if magnitude <= reference - progress:
            self._progress = (now, magnitude)
            self._travel = 0.0
            return ""
        if now - since <= self.policy.stall_time:
            return ""
        if self._travel > progress:
            # It moves, just not towards the goal: overshooting back and forth.
            return f"robot moved but came no closer for {self.policy.stall_time:.0f} s"
        return f"robot did not {verb} for {self.policy.stall_time:.0f} s"

    def _checks_agree(self) -> bool:
        first = self._checks[0].offset
        return all(
            math.hypot(check.offset.x - first.x, check.offset.y - first.y) <= self.policy.check_agreement
            and abs(_wrap(check.offset.yaw - first.yaw)) <= self.policy.check_agreement_yaw
            for check in self._checks[1:]
        )

    def _errors(self, pose: Pose2D) -> tuple[float, float]:
        return (
            math.hypot(self._target.x - pose.x, self._target.y - pose.y),
            _wrap(self._target.yaw - pose.yaw),
        )

    def _stopped(self, distance: float, yaw_error: float) -> AlignmentStep:
        return AlignmentStep(self._phase, (0.0, 0.0, 0.0), distance, yaw_error)

    def _fail(self, pose: Pose2D | None, reason: str) -> AlignmentStep:
        self._phase = AlignmentPhase.FAILED
        self._reason = reason
        return self._result(pose)

    def _result(self, pose: Pose2D | None) -> AlignmentStep:
        if pose is None or self._target is None or not _finite_pose(pose):
            distance, yaw_error = math.nan, math.nan
        else:
            distance, yaw_error = self._errors(pose)
        return AlignmentStep(self._phase, (0.0, 0.0, 0.0), distance, yaw_error, self._reason)
