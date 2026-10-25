"""Walk a recorded mission: navigate, dock, grasp, and come back.

This sits above the two controllers rather than replacing either. Nav2 covers
the distance to a recorded waypoint and stops within `xy_goal_tolerance`
(0.35 m), which is wider than the arm's whole reach; `ApproachController` then
closes that on the corridor markers. Keeping them in sequence is a safety
property, not just tidiness: both publish on `/cmd_vel_nav`, so only one may
be active at a time, and the runner is what guarantees it.

Every outcome is an `Action` the node carries out. Aborts carry a reason and
leave the runner stopped, because a mission that silently continues past a
failed step is how a robot ends up driving at furniture.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from omx_navigation.approach_controller import (
    ApproachController,
    ApproachPolicy,
    ApproachState,
    Detection,
)
from omx_navigation.corridor_dock import CorridorDockPolicy
from omx_navigation.mission import Mission


class MissionState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    DOCKING = "DOCKING"
    GRASPING = "GRASPING"
    DONE = "DONE"
    ABORTED = "ABORTED"


class ActionKind(str, Enum):
    NONE = "NONE"
    NAVIGATE = "NAVIGATE"
    DRIVE = "DRIVE"
    GRASP = "GRASP"
    DONE = "DONE"
    ABORT = "ABORT"


@dataclass(frozen=True)
class Action:
    kind: ActionKind
    waypoint: object = None
    vx: float = 0.0
    vy: float = 0.0
    reason: str = ""


class MissionRunner:
    def __init__(
        self,
        mission: Mission,
        dock_policy: CorridorDockPolicy | None = None,
        approach_policy: ApproachPolicy | None = None,
    ) -> None:
        self.mission = mission
        self.dock_policy = dock_policy or CorridorDockPolicy()
        self._approach = ApproachController(approach_policy)
        self.reset()

    def reset(self) -> None:
        self._state = MissionState.IDLE
        self._index = 0
        self._approach.reset()

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def step_index(self) -> int:
        return self._index

    def start(self, now: float) -> Action:
        self.reset()
        return self._enter(self._index, now)

    def on_navigation_result(self, succeeded: bool, now: float) -> Action:
        """Nav2 finished the current navigate step."""
        if self._state is not MissionState.NAVIGATING:
            return self._abort("navigation result arrived outside a navigate step")
        if not succeeded:
            return self._abort("navigation failed")
        return self._advance(now)

    def on_grasp_result(self, succeeded: bool, now: float) -> Action:
        if self._state is not MissionState.GRASPING:
            return self._abort("grasp result arrived outside a grasp step")
        if not succeeded:
            return self._abort("grasp failed")
        return self._advance(now)

    def update(
        self,
        now: float,
        fix=None,
        *,
        localization_ready: bool = True,
        manual_override: bool = False,
        execution_fault: bool = False,
        forward_clearance: float = float("inf"),
        minimum_clearance: float = 0.0,
    ) -> Action:
        """Advance a docking step by one control cycle.

        Called every cycle while `state` is DOCKING. `fix` is the latest
        `CorridorFix`, or None when the markers were not cleanly seen --
        `ApproachController` turns that into a stop.

        `forward_clearance` is the net under the loop. The keep-out that guards
        a facing dock cannot work here, because the corridor markers' midpoint
        sits roughly at the robot rather than ahead of it, so the chair is
        watched directly instead.
        """
        if self._state is not MissionState.DOCKING:
            return Action(ActionKind.NONE)
        if forward_clearance < minimum_clearance:
            self._approach.reset()
            return self._abort("forward clearance breached")
        detection = None if fix is None else Detection(now, fix.mid_x, fix.mid_y)
        command = self._approach.update(
            now,
            detection,
            localization_ready=localization_ready,
            manual_override=manual_override,
            execution_fault=execution_fault,
        )
        if command.state is ApproachState.ABORTED:
            return self._abort(f"docking aborted: {command.reason}")
        if command.state is ApproachState.ARRIVED:
            return self._advance(now)
        return Action(ActionKind.DRIVE, vx=command.vx, vy=command.vy)

    def _advance(self, now: float) -> Action:
        self._index += 1
        if self._index >= len(self.mission.sequence):
            self._state = MissionState.DONE
            return Action(ActionKind.DONE)
        return self._enter(self._index, now)

    def _enter(self, index: int, now: float) -> Action:
        step = self.mission.sequence[index]
        if step.kind == "navigate":
            self._state = MissionState.NAVIGATING
            return Action(ActionKind.NAVIGATE, waypoint=self.mission.waypoint(step.waypoint))
        if step.kind == "dock":
            self._state = MissionState.DOCKING
            self._approach.start(
                now, self.dock_policy.stop_offset_x, self.dock_policy.stop_offset_y
            )
            # No velocity yet: the first real command waits for a detection.
            return Action(ActionKind.DRIVE)
        self._state = MissionState.GRASPING
        return Action(ActionKind.GRASP)

    def _abort(self, reason: str) -> Action:
        self._state = MissionState.ABORTED
        return Action(ActionKind.ABORT, reason=reason)
