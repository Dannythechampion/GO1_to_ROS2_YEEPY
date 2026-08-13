"""Fail-closed velocity permission and freshness gate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    yaw: float

    @classmethod
    def zero(cls) -> "VelocityCommand":
        return cls(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ArmResult:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class GateResult:
    command: VelocityCommand
    enabled: bool
    reason: str


class MotionGateCore:
    def __init__(
        self,
        *,
        localization_timeout: float = 0.30,
        estop_timeout: float = 0.30,
        command_timeout: float = 0.25,
        max_linear_speed: float = 0.20,
        max_angular_speed: float = 0.40,
    ) -> None:
        self.localization_timeout = localization_timeout
        self.estop_timeout = estop_timeout
        self.command_timeout = command_timeout
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self._localization_ready = False
        self._localization_stamp: Optional[float] = None
        self._estop_pressed = True
        self._estop_stamp: Optional[float] = None
        self._armed = False
        self._command = VelocityCommand.zero()
        self._command_stamp: Optional[float] = None

    def update_localization(self, ready: bool, stamp: float) -> None:
        self._localization_ready = bool(ready)
        self._localization_stamp = float(stamp)
        if not ready:
            self._armed = False

    def update_estop(self, pressed: bool, stamp: float) -> None:
        self._estop_pressed = bool(pressed)
        self._estop_stamp = float(stamp)
        if pressed:
            self._armed = False

    def update_command(self, command: VelocityCommand, stamp: float) -> None:
        self._command = command
        self._command_stamp = float(stamp)

    def arm(self, now: float) -> ArmResult:
        permission_reason = self._permission_failure(float(now), require_arm=False)
        if permission_reason is not None:
            return ArmResult(False, permission_reason)
        self._armed = True
        return ArmResult(True, "motion gate armed")

    def disarm(self) -> None:
        self._armed = False
        self._command = VelocityCommand.zero()
        self._command_stamp = None

    def evaluate(self, now: float) -> GateResult:
        now = float(now)
        permission_reason = self._permission_failure(now, require_arm=True)
        if permission_reason is not None:
            return GateResult(VelocityCommand.zero(), False, permission_reason)
        if self._command_stamp is None:
            return GateResult(
                VelocityCommand.zero(), True, "waiting for navigation command"
            )
        if now - self._command_stamp > self.command_timeout:
            return GateResult(VelocityCommand.zero(), True, "navigation command is stale")
        values = (self._command.vx, self._command.vy, self._command.yaw)
        if not all(math.isfinite(float(value)) for value in values):
            return GateResult(VelocityCommand.zero(), True, "non-finite command")
        if math.hypot(self._command.vx, self._command.vy) > self.max_linear_speed:
            return GateResult(VelocityCommand.zero(), True, "linear speed exceeds limit")
        if abs(self._command.yaw) > self.max_angular_speed:
            return GateResult(VelocityCommand.zero(), True, "angular speed exceeds limit")
        return GateResult(self._command, True, "command accepted")

    def _permission_failure(self, now: float, *, require_arm: bool) -> Optional[str]:
        if self._localization_stamp is None or not self._localization_ready:
            return "localization is not ready"
        if now - self._localization_stamp > self.localization_timeout:
            self._armed = False
            return "localization heartbeat is stale"
        if self._estop_stamp is None:
            return "emergency stop state is unknown"
        if now - self._estop_stamp > self.estop_timeout:
            self._armed = False
            return "emergency stop heartbeat is stale"
        if self._estop_pressed:
            self._armed = False
            return "emergency stop is pressed"
        if require_arm and not self._armed:
            return "motion gate is disarmed"
        return None
