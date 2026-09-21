"""Compare the motion the driver commanded with the motion odometry measured.

Measured 2026-09-18: while the operator held the handheld remote, Nav2 kept an
active goal and commanded steady, non-oscillating turns of -401, +339 and -75
degrees over 20 s; the robot turned 5.4, 11.0 and 0.1 degrees. Nav2 noticed
only through its progress checker, 20 s later, and reported it as its own
failure to make progress. Minutes earlier the same robot had followed 85-103%
of what it was told.

Both failure directions matter: a robot that ignores commands, and a robot
that moves a lot with no command at all (09-18: 9.3 m in 40 s at up to
0.92 m/s, 4.5 times the driver's speed limit, with 0.20 m commanded) -- the
second means someone else is driving.

Odometry is judged by net displacement and net rotation, never path length.
Standing still, FAST-LIO wandered about 2 m of path per minute on 09-18 while
moving about 1 cm net.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass
from enum import Enum


class ExecutionState(str, Enum):
    OK = "OK"
    NOT_EXECUTING = "NOT_EXECUTING"
    UNCOMMANDED_MOTION = "UNCOMMANDED_MOTION"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ExecutionPolicy:
    window: float = 3.0
    # Commands below these are too small to judge.
    min_commanded_turn: float = math.radians(25.0)
    min_commanded_distance: float = 0.20
    # Less than this share of the commanded motion showing up is a refusal.
    # Following was 85-103% on 09-18; the refusals were 0-3%.
    min_achieved_ratio: float = 0.25
    # "Nothing commanded" for the uncommanded-motion check.
    idle_command_distance: float = 0.05
    idle_command_turn: float = math.radians(10.0)
    uncommanded_distance: float = 0.60
    uncommanded_turn: float = math.radians(45.0)
    # Odometry gaps longer than this make a window unjudgeable.
    max_pose_gap: float = 0.5

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.min_achieved_ratio >= 1.0:
            raise ValueError("min_achieved_ratio must be below 1")


@dataclass(frozen=True)
class ExecutionVerdict:
    state: ExecutionState
    commanded_distance: float = 0.0
    achieved_distance: float = 0.0
    commanded_turn: float = 0.0
    achieved_turn: float = 0.0

    @property
    def ratio(self) -> float:
        """Achieved share of the larger commanded motion (1.0 when nothing was commanded)."""
        shares = []
        if self.commanded_distance > 0.0:
            shares.append((self.commanded_distance, self.achieved_distance / self.commanded_distance))
        if self.commanded_turn > 0.0:
            shares.append((self.commanded_turn, self.achieved_turn / self.commanded_turn))
        return min(shares)[1] if shares else 1.0


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ExecutionMonitor:
    def __init__(self, policy: ExecutionPolicy | None = None) -> None:
        self.policy = policy or ExecutionPolicy()
        self.reset()

    def reset(self) -> None:
        self._commands: list[tuple[float, float, float]] = []  # (t, linear speed, yaw rate)
        self._poses: list[tuple[float, float, float, float]] = []  # (t, x, y, unwrapped yaw)

    def command(self, stamp: float, vx: float, vy: float, yaw_rate: float) -> None:
        """Record the command in effect from `stamp` on (held until the next one)."""
        values = (stamp, vx, vy, yaw_rate)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("command values must be finite")
        if self._commands and stamp < self._commands[-1][0]:
            self.reset()
        self._commands.append((stamp, math.hypot(vx, vy), yaw_rate))
        self._trim(stamp)

    def pose(self, stamp: float, x: float, y: float, yaw: float) -> None:
        values = (stamp, x, y, yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose values must be finite")
        if self._poses and stamp < self._poses[-1][0]:
            self.reset()
        unwrapped = yaw if not self._poses else self._poses[-1][3] + _wrap(yaw - self._poses[-1][3])
        self._poses.append((stamp, x, y, unwrapped))
        self._trim(stamp)

    def evaluate(self, now: float) -> ExecutionVerdict:
        policy = self.policy
        start = now - policy.window
        poses = [pose for pose in self._poses if start - policy.max_pose_gap <= pose[0] <= now]
        if (
            len(poses) < 2
            or poses[0][0] > start + policy.max_pose_gap
            or now - poses[-1][0] > policy.max_pose_gap
            or any(later[0] - earlier[0] > policy.max_pose_gap for earlier, later in zip(poses, poses[1:]))
        ):
            return ExecutionVerdict(ExecutionState.UNKNOWN)
        # The last pose at or before the window start anchors the window.
        first = poses[bisect_right([pose[0] for pose in poses], start) - 1] if poses[0][0] <= start else poses[0]
        last = poses[-1]
        achieved_distance = math.hypot(last[1] - first[1], last[2] - first[2])
        achieved_turn = abs(last[3] - first[3])
        commanded_distance, commanded_turn = self._integrate(first[0], last[0])
        verdict_values = (commanded_distance, achieved_distance, commanded_turn, achieved_turn)

        refused = (
            commanded_turn >= policy.min_commanded_turn
            and achieved_turn < policy.min_achieved_ratio * commanded_turn
        ) or (
            commanded_distance >= policy.min_commanded_distance
            and achieved_distance < policy.min_achieved_ratio * commanded_distance
        )
        if refused:
            return ExecutionVerdict(ExecutionState.NOT_EXECUTING, *verdict_values)
        idle = commanded_distance <= policy.idle_command_distance and commanded_turn <= policy.idle_command_turn
        if idle and (achieved_distance >= policy.uncommanded_distance or achieved_turn >= policy.uncommanded_turn):
            return ExecutionVerdict(ExecutionState.UNCOMMANDED_MOTION, *verdict_values)
        return ExecutionVerdict(ExecutionState.OK, *verdict_values)

    def _integrate(self, start: float, end: float) -> tuple[float, float]:
        """Path length and net rotation commanded over [start, end]."""
        distance = 0.0
        turn = 0.0
        for index, (stamp, speed, yaw_rate) in enumerate(self._commands):
            until = self._commands[index + 1][0] if index + 1 < len(self._commands) else end
            overlap = min(until, end) - max(stamp, start)
            if overlap > 0.0:
                distance += speed * overlap
                turn += yaw_rate * overlap
        return distance, abs(turn)

    def _trim(self, now: float) -> None:
        horizon = now - self.policy.window - 2.0 * self.policy.max_pose_gap
        while len(self._commands) > 1 and self._commands[1][0] <= horizon:
            self._commands.pop(0)
        while len(self._poses) > 1 and self._poses[1][0] <= horizon:
            self._poses.pop(0)


class ExecutionGuard:
    """Latch a fault once a bad verdict has persisted long enough to act on.

    Short stalls are real but not worth a cancelled goal: on 09-18 Nav2 was
    following well in minute 4 and the robot still stood still for 2 s once
    while told to walk. A refusal is acted on after `refusal_hold` s -- well
    inside Nav2's 20 s progress limit, which is what used to report it.
    Uncommanded motion means someone else is moving the robot and is acted on
    almost at once.
    """

    def __init__(self, refusal_hold: float = 5.0, uncommanded_hold: float = 1.0) -> None:
        for name, value in (("refusal_hold", refusal_hold), ("uncommanded_hold", uncommanded_hold)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        self._holds = {
            ExecutionState.NOT_EXECUTING: refusal_hold,
            ExecutionState.UNCOMMANDED_MOTION: uncommanded_hold,
        }
        self.clear()

    @property
    def fault(self) -> ExecutionState | None:
        return self._fault

    def clear(self) -> None:
        self._fault: ExecutionState | None = None
        self._state: ExecutionState | None = None
        self._since: float | None = None

    def update(self, now: float, verdict: ExecutionVerdict) -> ExecutionState | None:
        """Return the latched fault, if any, after taking `verdict` into account."""
        if self._fault is not None:
            return self._fault
        if verdict.state not in self._holds:
            self._state = None
            self._since = None
            return None
        if verdict.state is not self._state:
            self._state = verdict.state
            self._since = now
        if now - self._since >= self._holds[verdict.state]:
            self._fault = verdict.state
        return self._fault
