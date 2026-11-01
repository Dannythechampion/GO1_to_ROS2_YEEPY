"""Walk a recorded mission: navigate, dock, grasp, and come back.

This sits above the two controllers rather than replacing either. Nav2 covers
the distance to a recorded waypoint; `TapeDock` then stops on the corridor
markers. Keeping them in sequence is a safety property, not just tidiness:
both publish on `/cmd_vel_nav`, so only one may be active at a time, and the
runner is what guarantees it.

Nav2 does not have to finish for the dock to start. On 2026-10-25 it never
did: near the goal the pose wobbled across `xy_goal_tolerance` while the
controller turned for the final heading, and every crossing restarted the
approach until the run was killed. When the next step is a dock and the
tape is already in view close to the goal, `on_tape_handoff` ends navigation
there and the dock takes over -- the tape, not the goal checker, decides where
the robot stops. A navigation that fails with the tape in view is handed over
the same way rather than aborted.

Every outcome is an `Action` the node carries out. Aborts carry a reason and
leave the runner stopped, because a mission that silently continues past a
failed step is how a robot ends up driving at furniture.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from omx_navigation.corridor_dock import CorridorDockPolicy
from omx_navigation.mission import Mission
from omx_navigation.tape_dock import DockPhase, TapeDock, TapeDockPolicy


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
    # What the decision was made from, for the decision log.
    detail: dict = field(default_factory=dict)


class MissionRunner:
    def __init__(
        self,
        mission: Mission,
        dock_policy: CorridorDockPolicy | None = None,
        tape_dock_policy: TapeDockPolicy | None = None,
    ) -> None:
        self.mission = mission
        self.dock_policy = dock_policy or CorridorDockPolicy()
        self._dock = TapeDock(tape_dock_policy)
        self.reset()

    def reset(self) -> None:
        self._state = MissionState.IDLE
        self._index = 0
        self._active_dock_policy = self.dock_policy
        self._dock.reset()
        self.last_dock_rounds = []

    @property
    def active_dock_policy(self) -> CorridorDockPolicy:
        """The detector policy for the dock in progress -- or about to start.

        A waypoint recorded at its tape carries the measured wall-to-wall gap,
        and the detector is held to it; one without keeps the default. While
        navigating to a spot that a dock follows, this is already that dock's
        policy, so the node can watch for the hand-over with the same test.
        """
        return self._active_dock_policy

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def step_index(self) -> int:
        return self._index

    @property
    def dock_phase(self) -> DockPhase:
        return self._dock.phase

    @property
    def navigation_target(self):
        """The waypoint being navigated to, or None outside a navigate step."""
        if self._state is not MissionState.NAVIGATING:
            return None
        return self.mission.waypoint(self.mission.sequence[self._index].waypoint)

    @property
    def docks_next(self) -> bool:
        """True while navigating to a spot whose tape a dock step drives onto."""
        following = self._index + 1
        return (
            self._state is MissionState.NAVIGATING
            and following < len(self.mission.sequence)
            and self.mission.sequence[following].kind == "dock"
        )

    def start(self, now: float) -> Action:
        self.reset()
        return self._enter(self._index, now)

    def abort(self, reason: str) -> Action:
        """Stop the mission from outside: an operator cancel, or a dead link."""
        return self._abort(reason)

    def on_navigation_result(self, succeeded: bool, now: float, tape_in_view: bool = False) -> Action:
        """Nav2 finished the current navigate step.

        A failure with the spot's tape in view still docks: the point of the
        navigate step was only to bring the tape into range, and it has.
        """
        if self._state is not MissionState.NAVIGATING:
            return self._abort("navigation result arrived outside a navigate step")
        if not succeeded:
            if tape_in_view and self.docks_next:
                return self._advance(now, "navigation failed with the tape in view; docking")
            return self._abort("navigation failed")
        return self._advance(now, "navigation succeeded")

    def on_tape_handoff(self, now: float) -> Action:
        """The tape is in view near the goal: stop navigating, start docking.

        The caller cancels the Nav2 goal; the dock opens with a settle, which
        covers the controller and velocity smoother running down.
        """
        if not self.docks_next:
            return self._abort("tape handoff outside a navigate step that a dock follows")
        return self._advance(now, "tape in view near the goal; navigation handed to the dock")

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
        rear_clearance: float = float("inf"),
        minimum_clearance: float = 0.0,
    ) -> Action:
        """Advance a docking step by one control cycle.

        Called every cycle while `state` is DOCKING. `fix` is the latest
        `CorridorFix`, or None when the markers were not cleanly seen.

        The clearances are the net under the loop and are enforced in the
        direction the dock is about to move: something passing behind a robot
        that is standing still to measure is no reason to give up on the line.
        """
        if self._state is not MissionState.DOCKING:
            return Action(ActionKind.NONE)
        policy = self._active_dock_policy
        if fix is not None and fix.correction(policy) > policy.max_correction:
            return self._abort(
                f"markers {fix.correction(policy):.2f} m away, beyond "
                f"max_correction {policy.max_correction:.2f} m"
            )
        command = self._dock.update(
            now,
            None if fix is None else _Offset(fix.mid_x - policy.stop_offset_x, fix.mid_y - policy.stop_offset_y),
            localization_ready=localization_ready,
            manual_override=manual_override,
            execution_fault=execution_fault,
        )
        detail = dict(command.detail, phase=command.phase.value, why=command.reason)
        if command.phase is DockPhase.ABORTED:
            return self._abort(f"docking aborted: {command.reason}", detail)
        if command.phase is DockPhase.ARRIVED:
            self.last_dock_rounds = self._dock.rounds
            return self._advance(now, command.reason, detail)
        if command.vx > 0.0 and forward_clearance < minimum_clearance:
            return self._abort("forward clearance breached", detail)
        if command.vx < 0.0 and rear_clearance < minimum_clearance:
            return self._abort("rear clearance breached", detail)
        return Action(ActionKind.DRIVE, vx=command.vx, vy=command.vy, reason=command.reason, detail=detail)

    def _advance(self, now: float, reason: str = "", detail=None) -> Action:
        self._index += 1
        if self._index >= len(self.mission.sequence):
            self._state = MissionState.DONE
            return Action(ActionKind.DONE, reason=reason, detail=detail or {})
        return self._enter(self._index, now, reason, detail)

    def _enter(self, index: int, now: float, reason: str = "", detail=None) -> Action:
        step = self.mission.sequence[index]
        if step.kind == "navigate":
            self._state = MissionState.NAVIGATING
            following = index + 1
            if following < len(self.mission.sequence) and self.mission.sequence[following].kind == "dock":
                self._active_dock_policy = self._policy_for(following)
            return Action(ActionKind.NAVIGATE, waypoint=self.mission.waypoint(step.waypoint),
                          reason=reason, detail=detail or {})
        if step.kind == "dock":
            self._state = MissionState.DOCKING
            self._active_dock_policy = self._policy_for(index)
            self._dock.start(now)
            # No velocity yet: the dock opens with a settle.
            return Action(ActionKind.DRIVE, reason=reason, detail=detail or {})
        self._state = MissionState.GRASPING
        return Action(ActionKind.GRASP, reason=reason, detail=detail or {})

    def _policy_for(self, dock_index: int) -> CorridorDockPolicy:
        spot = self.mission.dock_waypoint(dock_index)
        if spot.recorded_at_tape:
            return self.dock_policy.with_separation(spot.marker_separation)
        return self.dock_policy

    def _abort(self, reason: str, detail=None) -> Action:
        self._state = MissionState.ABORTED
        self._dock.reset()
        return Action(ActionKind.ABORT, reason=reason, detail=detail or {})


@dataclass(frozen=True)
class _Offset:
    mid_x: float
    mid_y: float
