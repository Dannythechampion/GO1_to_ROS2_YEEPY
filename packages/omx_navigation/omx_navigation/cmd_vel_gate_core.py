"""ROS-independent localization readiness gate for velocity commands."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    yaw: float

    @classmethod
    def zero(cls) -> "VelocityCommand":
        return cls(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GateDecision:
    command: VelocityCommand
    publish: bool
    reason: str


class VelocityGate:
    """Allows commands only during a fresh, true localization heartbeat."""

    def __init__(self, ready_timeout: float = 0.30, command_timeout: float = 0.30) -> None:
        self._validate_timeout("ready_timeout", ready_timeout)
        self._validate_timeout("command_timeout", command_timeout)
        self.ready_timeout = ready_timeout
        self.command_timeout = command_timeout
        self._ready = False
        self._last_ready_at: float | None = None
        self._last_command_at: float | None = None

    def update_ready(self, ready: bool, now: float) -> GateDecision:
        self._validate_now(now)
        if not isinstance(ready, bool):
            raise ValueError("ready must be a boolean")
        if ready:
            self._ready = True
            self._last_ready_at = now
            return GateDecision(VelocityCommand.zero(), False, "localization_ready")
        self._ready = False
        self._last_command_at = None
        return self._stop("localization_not_ready")

    def filter(self, command: VelocityCommand, now: float) -> GateDecision:
        self._validate_now(now)
        readiness_stop = self._readiness_stop(now)
        if readiness_stop is not None:
            return readiness_stop
        if not isinstance(command, VelocityCommand) or not _is_finite_command(command):
            return self._stop("command_nonfinite")
        self._last_command_at = now
        return GateDecision(command, True, "command_passed")

    def watchdog(self, now: float) -> GateDecision:
        self._validate_now(now)
        readiness_stop = self._readiness_stop(now)
        if readiness_stop is not None:
            return readiness_stop
        if self._last_command_at is None or now - self._last_command_at >= self.command_timeout:
            return self._stop("command_stale")
        return GateDecision(VelocityCommand.zero(), False, "command_fresh")

    def _readiness_stop(self, now: float) -> GateDecision | None:
        if not self._ready:
            return self._stop("localization_not_ready")
        if self._last_ready_at is None or now - self._last_ready_at >= self.ready_timeout:
            return self._stop("ready_heartbeat_stale")
        return None

    @staticmethod
    def _stop(reason: str) -> GateDecision:
        return GateDecision(VelocityCommand.zero(), True, reason)

    @staticmethod
    def _validate_timeout(name: str, value: object) -> None:
        if not _is_finite_number(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")

    @staticmethod
    def _validate_now(now: object) -> None:
        if not _is_finite_number(now):
            raise ValueError("now must be finite")


def _is_finite_command(command: VelocityCommand) -> bool:
    return all(_is_finite_number(value) for value in (command.vx, command.vy, command.yaw))


def _is_finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
