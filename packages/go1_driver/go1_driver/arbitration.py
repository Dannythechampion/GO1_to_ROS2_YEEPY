"""Decide what reaches the robot when someone else may be in control."""

from __future__ import annotations

import math
from typing import Optional

from .command_filter import STAND_MODE, MotionCommand


class CommandArbiter:
    """Hold the robot in stand while it is not ours, and do not resume on our own.

    The hazard is the moment after: Nav2's goal outlives a manual takeover, so
    when the operator lets go of the remote the stale goal's velocity would
    reach the robot again, and it walks off toward a target set before the
    takeover. After an override or an execution fault the incoming command
    stream must first go neutral -- a zero command or silence -- for
    `rearm_zero_s`. The navigation side cancels its goal on the same signals,
    which is what makes the stream go neutral.
    """

    def __init__(self, rearm_zero_s: float = 0.5) -> None:
        if not math.isfinite(rearm_zero_s) or rearm_zero_s <= 0.0:
            raise ValueError("rearm_zero_s must be finite and positive")
        self.rearm_zero_s = rearm_zero_s
        self._reason: Optional[str] = None
        self._neutral_since: Optional[float] = None

    @property
    def blocked_reason(self) -> Optional[str]:
        return self._reason

    def block(self, reason: str) -> None:
        """Hold stand until the command stream has been neutral for `rearm_zero_s`."""
        self._reason = reason
        self._neutral_since = None

    def decide(self, now: float, requested: MotionCommand, override_active: bool) -> MotionCommand:
        if override_active:
            self.block("manual override")
            return MotionCommand.stand("manual override: remote in use")
        if self._reason is None:
            return requested
        if requested.mode == STAND_MODE:
            if self._neutral_since is None:
                self._neutral_since = now
            if now - self._neutral_since >= self.rearm_zero_s:
                self._reason = None
                self._neutral_since = None
                return requested
        else:
            self._neutral_since = None
        return MotionCommand.stand(f"holding after {self._reason}: waiting for zero command")
