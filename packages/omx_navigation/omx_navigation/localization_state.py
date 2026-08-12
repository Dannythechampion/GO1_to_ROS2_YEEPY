"""ROS-independent safety state machine for pose-graph localization."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class LocalizationState(str, Enum):
    WAITING_INPUT = "WAITING_INPUT"
    ALIGNING = "ALIGNING"
    VERIFYING = "VERIFYING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


class ErrorCode(str, Enum):
    NONE = "NONE"
    INPUT_MISSING = "INPUT_MISSING"
    POSE_OUTSIDE_MAP = "POSE_OUTSIDE_MAP"
    ALIGNMENT_TIMEOUT = "ALIGNMENT_TIMEOUT"
    LOW_OVERLAP = "LOW_OVERLAP"
    AMBIGUOUS = "AMBIGUOUS"
    ODOM_RESET = "ODOM_RESET"
    TF_CONFLICT = "TF_CONFLICT"
    EXTRINSIC_UNCALIBRATED = "EXTRINSIC_UNCALIBRATED"


@dataclass(frozen=True)
class LocalizationPolicy:
    alignment_timeout: float = 20.0
    max_attempts: int = 3
    verify_duration: float = 3.0
    degraded_timeout: float = 2.0
    min_overlap: float = 0.45
    min_ambiguity_margin: float = 0.05
    max_position_jump: float = 0.30
    max_yaw_jump: float = math.radians(10.0)

    def __post_init__(self) -> None:
        finite_positive = (
            "alignment_timeout",
            "verify_duration",
            "degraded_timeout",
            "max_position_jump",
            "max_yaw_jump",
        )
        for name in finite_positive:
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not isinstance(self.max_attempts, int) or isinstance(self.max_attempts, bool) or self.max_attempts < 1:
            raise ValueError("max_attempts must be a positive integer")
        for name in ("min_overlap", "min_ambiguity_margin"):
            value = getattr(self, name)
            if not _is_finite_number(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be within [0.0, 1.0]")
        if self.max_yaw_jump > math.pi:
            raise ValueError("max_yaw_jump must not exceed pi radians")


@dataclass(frozen=True)
class QualityObservation:
    now: float
    inputs_fresh: bool
    pose_available: bool
    overlap: float
    ambiguity_margin: float
    position_jump: float
    yaw_jump: float
    odom_reset: bool
    tf_conflict: bool

    def __post_init__(self) -> None:
        if not _is_finite_number(self.now):
            raise ValueError("now must be finite")
        for name in ("overlap", "ambiguity_margin"):
            value = getattr(self, name)
            if not _is_finite_number(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be within [0.0, 1.0]")
        for name in ("position_jump", "yaw_jump"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

    @classmethod
    def missing(cls, now: float) -> "QualityObservation":
        return cls(now, False, False, 0.0, 0.0, 0.0, 0.0, False, False)


@dataclass(frozen=True)
class Transition:
    state: LocalizationState
    error: ErrorCode
    republish_initial_pose: bool
    publish_stop: bool


class LocalizationStateMachine:
    """Owns policy-driven localization transitions without reading a system clock."""

    def __init__(self, policy: LocalizationPolicy | None = None) -> None:
        self.policy = policy or LocalizationPolicy()
        self.state = LocalizationState.WAITING_INPUT
        self.error = ErrorCode.NONE
        self._attempts = 0
        self._alignment_deadline: float | None = None
        self._alignment_started_at: float | None = None
        self._verification_started_at: float | None = None
        self._degraded_started_at: float | None = None

    def receive_initial_pose(self, now: float) -> Transition:
        self._validate_now(now)
        self._attempts = 1
        self._alignment_deadline = now + self.policy.alignment_timeout
        self._start_alignment(now)
        return self._transition()

    def retry(self, now: float) -> Transition:
        self._validate_now(now)
        if self.state in (LocalizationState.WAITING_INPUT, LocalizationState.LOST):
            self._attempts = 1
            self._alignment_deadline = now + self.policy.alignment_timeout
        elif self._deadline_reached(now):
            self._lose(ErrorCode.ALIGNMENT_TIMEOUT)
            return self._transition()
        elif self._attempts < self.policy.max_attempts:
            self._attempts += 1
        else:
            self._lose(ErrorCode.ALIGNMENT_TIMEOUT)
            return self._transition()
        self._start_alignment(now)
        return self._transition(republish_initial_pose=True)

    def observe(self, observation: QualityObservation) -> Transition:
        self._validate_now(observation.now)
        immediate_error = self._immediate_error(observation)
        if immediate_error is not None:
            self._lose(immediate_error)
            return self._transition()
        if self.state is LocalizationState.WAITING_INPUT:
            self.error = ErrorCode.INPUT_MISSING
            return self._transition()
        if self.state in (LocalizationState.ALIGNING, LocalizationState.VERIFYING) and self._deadline_reached(observation.now):
            self._lose(ErrorCode.ALIGNMENT_TIMEOUT)
            return self._transition()
        quality_error = self._quality_error(observation)

        if self.state is LocalizationState.ALIGNING:
            if quality_error is None:
                self.state = LocalizationState.VERIFYING
                self.error = ErrorCode.NONE
                self._verification_started_at = observation.now
                return self._transition()
            if self._timed_out(
                observation.now,
                self._alignment_started_at,
                self.policy.alignment_timeout / self.policy.max_attempts,
            ):
                return self._retry_or_lose(ErrorCode.ALIGNMENT_TIMEOUT, observation.now)
            self.error = quality_error
            return self._transition()

        if self.state is LocalizationState.VERIFYING:
            if quality_error is not None:
                self._start_alignment(observation.now, quality_error)
                return self._transition()
            if self._timed_out(observation.now, self._verification_started_at, self.policy.verify_duration):
                self.state = LocalizationState.READY
                self.error = ErrorCode.NONE
            return self._transition()

        if self.state is LocalizationState.READY:
            if quality_error is not None:
                self.state = LocalizationState.DEGRADED
                self.error = quality_error
                self._degraded_started_at = observation.now
            return self._transition()

        if self.state is LocalizationState.DEGRADED:
            if quality_error is None:
                self.state = LocalizationState.READY
                self.error = ErrorCode.NONE
                self._degraded_started_at = None
            elif self._timed_out(observation.now, self._degraded_started_at, self.policy.degraded_timeout):
                self._lose(quality_error)
            else:
                self.error = quality_error
            return self._transition()

        return self._transition()

    def _retry_or_lose(self, error: ErrorCode, now: float) -> Transition:
        if self._attempts < self.policy.max_attempts:
            self._attempts += 1
            self._start_alignment(now)
            return self._transition(republish_initial_pose=True)
        self._lose(error)
        return self._transition()

    def _start_alignment(self, now: float, error: ErrorCode = ErrorCode.NONE) -> None:
        self.state = LocalizationState.ALIGNING
        self.error = error
        self._alignment_started_at = now
        self._verification_started_at = None
        self._degraded_started_at = None

    def _lose(self, error: ErrorCode) -> None:
        self.state = LocalizationState.LOST
        self.error = error
        self._verification_started_at = None
        self._degraded_started_at = None

    def _deadline_reached(self, now: float) -> bool:
        return self._alignment_deadline is not None and now >= self._alignment_deadline

    def _transition(self, republish_initial_pose: bool = False) -> Transition:
        return Transition(
            state=self.state,
            error=self.error,
            republish_initial_pose=republish_initial_pose,
            publish_stop=self.state is not LocalizationState.READY,
        )

    def _immediate_error(self, observation: QualityObservation) -> ErrorCode | None:
        if observation.tf_conflict:
            return ErrorCode.TF_CONFLICT
        if observation.odom_reset:
            return ErrorCode.ODOM_RESET
        return None

    def _quality_error(self, observation: QualityObservation) -> ErrorCode | None:
        numbers = (
            observation.overlap,
            observation.ambiguity_margin,
            observation.position_jump,
            observation.yaw_jump,
        )
        if not observation.inputs_fresh:
            return ErrorCode.INPUT_MISSING
        if not observation.pose_available:
            return ErrorCode.POSE_OUTSIDE_MAP
        if not all(_is_finite_number(value) for value in numbers):
            return ErrorCode.INPUT_MISSING
        if observation.overlap < self.policy.min_overlap:
            return ErrorCode.LOW_OVERLAP
        if observation.ambiguity_margin < self.policy.min_ambiguity_margin:
            return ErrorCode.AMBIGUOUS
        if observation.position_jump > self.policy.max_position_jump:
            return ErrorCode.POSE_OUTSIDE_MAP
        if abs(observation.yaw_jump) > self.policy.max_yaw_jump:
            return ErrorCode.POSE_OUTSIDE_MAP
        return None

    @staticmethod
    def _timed_out(now: float, started_at: float | None, timeout: float) -> bool:
        return started_at is not None and now - started_at >= timeout

    @staticmethod
    def _validate_now(now: float) -> None:
        if not _is_finite_number(now):
            raise ValueError("now must be finite")


def _is_finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
