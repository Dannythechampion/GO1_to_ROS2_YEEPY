"""Close the last stretch to a target without rotating.

Nav2 stops within `xy_goal_tolerance` (0.35 m), which is wider than the whole
OpenMANIPULATOR-X reach. This stage runs afterwards and drives the remaining
error out using the target's own detection, so grasp precision never depends
on map localization.

It never rotates. The driver is `holonomic: true`, Nav2's stateful goal
checker already leaves yaw inside `yaw_goal_tolerance`, and commanding yaw
here would walk straight back into the `yaw_deadband` 0.04 rad/s dead zone
that swallowed the final alignment on 2026-09-18 and 2026-10-25: DWB slowed to
roughly 0.05 rad/s near the goal, the driver zeroed anything under 0.04, and
the robot stood still while the stack believed it was turning.

The same trap exists in translation and is avoided by construction -- see
`ApproachPolicy.__post_init__`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class ApproachState(str, Enum):
    IDLE = "IDLE"
    APPROACHING = "APPROACHING"
    ARRIVED = "ARRIVED"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class Detection:
    """Where the target is, in `body_nav`, as of `stamp` seconds."""

    stamp: float
    x: float
    y: float


@dataclass(frozen=True)
class ApproachCommand:
    vx: float
    vy: float
    state: ApproachState
    reason: str = ""
    error: float = 0.0


@dataclass(frozen=True)
class ApproachPolicy:
    # What this stage promises to deliver. `approach_planner` budgets the
    # standoff distance against this number, so loosening it costs reach.
    approach_tolerance: float = 0.05
    # Slower than navigation: this is a blind-ish final move next to an object.
    v_approach: float = 0.08
    kp: float = 0.6
    # Must match go1_driver's linear_deadband. Commands under it become exactly
    # zero at the driver, and with zero yaw the driver switches to STAND.
    linear_deadband: float = 0.025
    detection_timeout: float = 0.5
    approach_timeout: float = 30.0
    # Consecutive in-tolerance updates before declaring arrival. One sample is
    # what let the goal checker flip in and out of tolerance on 2026-10-25.
    settle_checks: int = 3

    def __post_init__(self) -> None:
        for name in (
            "approach_tolerance", "v_approach", "kp", "linear_deadband",
            "detection_timeout", "approach_timeout",
        ):
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not isinstance(self.settle_checks, int) or isinstance(self.settle_checks, bool) or self.settle_checks < 1:
            raise ValueError("settle_checks must be a positive integer")
        # The controller slows as the error shrinks, so the command falls under
        # the driver's deadband once the error drops below linear_deadband/kp.
        # If that happens before the error reaches approach_tolerance, the
        # driver zeroes the command and the robot stops short, outside
        # tolerance, forever -- the translation copy of the yaw dead zone that
        # cost two field sessions. Refuse that combination at construction
        # rather than discover it next to an object.
        stall_error = self.linear_deadband / self.kp
        if self.approach_tolerance <= stall_error:
            raise ValueError(
                "approach_tolerance must exceed linear_deadband / kp "
                f"({stall_error:.4f} m), otherwise the driver zeroes the "
                "command before the target is reached"
            )


class ApproachController:
    """Drive the detected target onto `desired` without turning."""

    def __init__(self, policy: ApproachPolicy | None = None) -> None:
        self.policy = policy or ApproachPolicy()
        self.reset()

    def reset(self) -> None:
        self._state = ApproachState.IDLE
        self._started_at: float | None = None
        self._settled = 0
        self._desired: tuple[float, float] = (0.0, 0.0)

    @property
    def state(self) -> ApproachState:
        return self._state

    def start(self, now: float, desired_x: float, desired_y: float = 0.0) -> None:
        """Begin approaching, aiming to put the target at `desired` in body_nav.

        `desired_x` is the standoff distance from `approach_planner`.
        """
        if not all(_is_finite_number(value) for value in (now, desired_x, desired_y)):
            raise ValueError("start values must be finite numbers")
        self._state = ApproachState.APPROACHING
        self._started_at = now
        self._settled = 0
        self._desired = (float(desired_x), float(desired_y))

    def update(
        self,
        now: float,
        detection: Detection | None,
        *,
        localization_ready: bool = True,
        manual_override: bool = False,
        execution_fault: bool = False,
    ) -> ApproachCommand:
        """Advance one control cycle and return what to publish.

        Every return is a command to publish on `/cmd_vel_nav`, so the existing
        `cmd_vel_safety_gate` still gates it. Aborts return zero velocity; they
        do not simply stop publishing, because a silent publisher would leave
        the gate's last value to time out instead of commanding a stop.
        """
        if not _is_finite_number(now):
            raise ValueError("now must be a finite number")
        if self._state in (ApproachState.ARRIVED, ApproachState.ABORTED, ApproachState.IDLE):
            return ApproachCommand(0.0, 0.0, self._state)

        policy = self.policy
        if manual_override:
            return self._abort("manual override")
        if execution_fault:
            return self._abort("execution fault")
        if not localization_ready:
            return self._abort("localization not ready")
        if self._started_at is not None and now - self._started_at > policy.approach_timeout:
            return self._abort("approach timeout")
        if detection is None:
            return self._abort("no detection")
        if not all(_is_finite_number(value) for value in (detection.stamp, detection.x, detection.y)):
            raise ValueError("detection values must be finite numbers")
        age = now - detection.stamp
        if age < 0.0 or age > policy.detection_timeout:
            return self._abort("detection stale")

        error_x = detection.x - self._desired[0]
        error_y = detection.y - self._desired[1]
        error = math.hypot(error_x, error_y)

        if error <= policy.approach_tolerance:
            self._settled += 1
            if self._settled >= policy.settle_checks:
                self._state = ApproachState.ARRIVED
                return ApproachCommand(0.0, 0.0, ApproachState.ARRIVED, "within tolerance", error)
            # Hold still while confirming rather than nudging on a single good
            # sample; the error is already inside what the arm can absorb.
            return ApproachCommand(0.0, 0.0, ApproachState.APPROACHING, "settling", error)

        self._settled = 0
        speed = min(policy.kp * error, policy.v_approach)
        # Outside tolerance the command must stay commandable. The policy
        # guarantees this cannot clamp away real error, but a target that jumps
        # inward between cycles can still land here.
        speed = max(speed, policy.linear_deadband)
        scale = speed / error
        return ApproachCommand(
            error_x * scale, error_y * scale, ApproachState.APPROACHING, "approaching", error
        )

    def _abort(self, reason: str) -> ApproachCommand:
        self._state = ApproachState.ABORTED
        return ApproachCommand(0.0, 0.0, ApproachState.ABORTED, reason)


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
