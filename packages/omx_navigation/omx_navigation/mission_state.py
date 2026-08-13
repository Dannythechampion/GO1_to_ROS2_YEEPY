"""Pure arbitration state for fixed and RViz navigation goals."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .pose_config import PlanarPose


class MissionState(Enum):
    BOOT = "BOOT"
    UNCONFIGURED_DESTINATION = "UNCONFIGURED_DESTINATION"
    IDLE = "IDLE"
    SENDING = "SENDING"
    ACTIVE = "ACTIVE"
    CANCELING = "CANCELING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class GoalSource(Enum):
    NONE = "NONE"
    FIXED = "FIXED"
    RVIZ = "RVIZ"


class MissionEffect(Enum):
    NONE = "NONE"
    SEND_GOAL = "SEND_GOAL"
    CANCEL_GOAL = "CANCEL_GOAL"
    REJECT = "REJECT"


@dataclass(frozen=True)
class MissionDecision:
    effect: MissionEffect
    reason: str
    goal: Optional[PlanarPose] = None


class MissionStateMachine:
    def __init__(self, destination: Optional[PlanarPose]) -> None:
        self.destination = destination
        self.state = (
            MissionState.IDLE
            if destination is not None
            else MissionState.UNCONFIGURED_DESTINATION
        )
        self.source = GoalSource.NONE
        self.reason = (
            "ready for a goal"
            if destination is not None
            else "registered destination is not configured"
        )
        self._localization_ready = False
        self._localization_stamp: Optional[float] = None
        self._gate_enabled = False
        self._gate_stamp: Optional[float] = None
        self._action_available = False

    @property
    def active(self) -> bool:
        return self.state in {
            MissionState.SENDING,
            MissionState.ACTIVE,
            MissionState.CANCELING,
        }

    def update_localization(self, ready: bool, stamp: float) -> MissionDecision:
        self._localization_ready = bool(ready)
        self._localization_stamp = float(stamp)
        if not ready and self.state in {MissionState.SENDING, MissionState.ACTIVE}:
            return self._begin_cancel("localization readiness lost")
        return MissionDecision(MissionEffect.NONE, "localization updated")

    def update_motion_gate(self, enabled: bool, stamp: float) -> MissionDecision:
        self._gate_enabled = bool(enabled)
        self._gate_stamp = float(stamp)
        if not enabled and self.state in {MissionState.SENDING, MissionState.ACTIVE}:
            return self._begin_cancel("motion gate permission lost")
        return MissionDecision(MissionEffect.NONE, "motion gate updated")

    def update_action_server(self, available: bool) -> None:
        self._action_available = bool(available)

    def request_fixed(self, *, now: float, goal_valid: bool) -> MissionDecision:
        if self.destination is None:
            self.state = MissionState.UNCONFIGURED_DESTINATION
            self.reason = "registered destination is not configured"
            return MissionDecision(MissionEffect.REJECT, self.reason)
        return self._request(
            self.destination, GoalSource.FIXED, now=float(now), goal_valid=goal_valid
        )

    def request_rviz(
        self, goal: PlanarPose, *, now: float, goal_valid: bool
    ) -> MissionDecision:
        return self._request(goal, GoalSource.RVIZ, now=float(now), goal_valid=goal_valid)

    def _request(
        self, goal: PlanarPose, source: GoalSource, *, now: float, goal_valid: bool
    ) -> MissionDecision:
        if self.active:
            return MissionDecision(MissionEffect.REJECT, "a mission is already active")
        failure = self._prerequisite_failure(now)
        if failure is not None:
            self.reason = failure
            return MissionDecision(MissionEffect.REJECT, failure)
        if not goal_valid:
            self.reason = "goal failed map validation"
            return MissionDecision(MissionEffect.REJECT, self.reason)
        self.state = MissionState.SENDING
        self.source = source
        self.reason = "sending goal to Nav2"
        return MissionDecision(MissionEffect.SEND_GOAL, self.reason, goal)

    def _prerequisite_failure(self, now: float) -> Optional[str]:
        if not self._localization_ready or self._localization_stamp is None:
            return "localization is not ready"
        if now - self._localization_stamp > 0.30:
            return "localization heartbeat is stale"
        if not self._gate_enabled or self._gate_stamp is None:
            return "motion gate is not enabled"
        if now - self._gate_stamp > 0.30:
            return "motion gate heartbeat is stale"
        if not self._action_available:
            return "NavigateToPose action server is unavailable"
        return None

    def goal_response(self, accepted: bool) -> None:
        if self.state is not MissionState.SENDING:
            return
        if accepted:
            self.state = MissionState.ACTIVE
            self.reason = "Nav2 accepted goal"
        else:
            self.state = MissionState.FAILED
            self.reason = "Nav2 rejected goal"

    def request_cancel(self) -> MissionDecision:
        if self.state not in {MissionState.SENDING, MissionState.ACTIVE}:
            return MissionDecision(MissionEffect.NONE, "no cancelable mission")
        return self._begin_cancel("explicit cancel requested")

    def _begin_cancel(self, reason: str) -> MissionDecision:
        if self.state is MissionState.CANCELING:
            return MissionDecision(MissionEffect.NONE, "cancel already in progress")
        self.state = MissionState.CANCELING
        self.reason = reason
        return MissionDecision(MissionEffect.CANCEL_GOAL, reason)

    def action_result(self, outcome: str) -> None:
        mapping = {
            "SUCCEEDED": MissionState.SUCCEEDED,
            "FAILED": MissionState.FAILED,
            "CANCELED": MissionState.CANCELED,
        }
        if outcome not in mapping:
            raise ValueError(f"unsupported action outcome: {outcome}")
        self.state = mapping[outcome]
        self.reason = f"mission {outcome.lower()}"
