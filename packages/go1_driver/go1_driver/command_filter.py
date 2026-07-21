"""ROS-independent command filtering for Unitree Go1 navigation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


STAND_MODE = 1
WALK_MODE = 2


@dataclass(frozen=True)
class MotionCommand:
    """A filtered command expressed in the Go1 body frame."""

    mode: int
    vx: float
    vy: float
    yaw: float
    reason: str

    @classmethod
    def stand(cls, reason: str = "zero command") -> "MotionCommand":
        return cls(STAND_MODE, 0.0, 0.0, 0.0, reason)


class CommandFilter:
    """Apply deadbands and direction-preserving limits to cmd_vel."""

    def __init__(
        self,
        max_linear_speed: float = 0.20,
        max_yaw_speed: float = 0.40,
        linear_deadband: float = 0.025,
        yaw_deadband: float = 0.04,
        holonomic: bool = True,
        invert_lateral: bool = False,
    ) -> None:
        if max_linear_speed <= 0.0:
            raise ValueError("max_linear_speed must be positive")
        if max_yaw_speed <= 0.0:
            raise ValueError("max_yaw_speed must be positive")
        if linear_deadband < 0.0 or yaw_deadband < 0.0:
            raise ValueError("deadbands cannot be negative")

        self.max_linear_speed = float(max_linear_speed)
        self.max_yaw_speed = float(max_yaw_speed)
        self.linear_deadband = float(linear_deadband)
        self.yaw_deadband = float(yaw_deadband)
        self.holonomic = bool(holonomic)
        self.invert_lateral = bool(invert_lateral)

    def filter(self, vx: float, vy: float, yaw: float) -> MotionCommand:
        values = (float(vx), float(vy), float(yaw))
        if not all(math.isfinite(value) for value in values):
            return MotionCommand.stand("non-finite command")

        vx, vy, yaw = values
        if not self.holonomic:
            vy = 0.0
        elif self.invert_lateral:
            vy = -vy

        linear_norm = math.hypot(vx, vy)
        if linear_norm < self.linear_deadband:
            vx = 0.0
            vy = 0.0
            linear_norm = 0.0
        elif linear_norm > self.max_linear_speed:
            scale = self.max_linear_speed / linear_norm
            vx *= scale
            vy *= scale

        if abs(yaw) < self.yaw_deadband:
            yaw = 0.0
        else:
            yaw = max(-self.max_yaw_speed, min(self.max_yaw_speed, yaw))

        if linear_norm == 0.0 and yaw == 0.0:
            return MotionCommand.stand()
        return MotionCommand(WALK_MODE, vx, vy, yaw, "active command")


def apply_watchdog(
    command: MotionCommand,
    last_command_time: Optional[float],
    now: float,
    timeout: float,
) -> MotionCommand:
    """Return stand when cmd_vel has never arrived or has gone stale."""

    if timeout <= 0.0:
        raise ValueError("timeout must be positive")
    if last_command_time is None:
        return MotionCommand.stand("waiting for cmd_vel")
    if now - last_command_time > timeout:
        return MotionCommand.stand("cmd_vel watchdog timeout")
    return command
