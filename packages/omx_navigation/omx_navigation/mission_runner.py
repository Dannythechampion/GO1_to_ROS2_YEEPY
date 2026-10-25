"""Walk a recorded mission: navigate, dock, grasp, and come back.

This sits above the controllers rather than replacing any. A navigate step is
Nav2 plus final alignment (`final_alignment`, run by `rviz_goal_bridge`), which
lands on the recorded waypoint by the map -- good to a few centimetres at
best, and the map hunted by decimetres on 2026-10-25. A dock step then closes
the rest against the chair itself:

1. **Latch.** Standing where navigation left it, the robot measures the
   chair's front edge over several scans and fixes it in odometry
   (`chair_front.FrontLatch`). Close in, the edge drops under the LiDAR's
   field, so it is measured here, from where it is still in view.
2. **Approach.** `ApproachController` drives onto a target: either the
   standoff point in front of the latched edge, or the corridor tape
   markers. Every command is then cut so that the measured stopping distance
   still ends before `min_clearance` from the edge.

Keeping these in sequence is a safety property, not just tidiness: final
alignment and docking both drive the base, so only one may be active at a
time, and the runner is what guarantees it.

Every outcome is an `Action` the node carries out. Aborts carry a reason and
leave the runner stopped, because a mission that silently continues past a
failed step is how a robot ends up driving at furniture.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum

from omx_navigation.approach_controller import (
    ApproachController,
    ApproachPolicy,
    ApproachState,
    Detection,
)
from omx_navigation.chair_front import (
    ChairFrontPolicy,
    FrontLatch,
    LatchedFront,
    OdomPose,
    StoppingModel,
    limit_toward,
)
from omx_navigation.corridor_dock import CorridorDockPolicy
from omx_navigation.mission import Mission


class MissionState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    LATCHING = "LATCHING"
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
    # On GRASP: what docking left over, (x, y, yaw) in body_nav -- the target
    # minus where the robot stopped, and the turn it would take to square up.
    # The base does not chase these; the arm takes them out.
    residual: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class ChairDockPolicy:
    """Docking on the chair's latched front edge.

    `stopping` and `standoff_gap` have no defaults. The first is a
    measurement nobody has made yet; the second is a choice between that
    measurement and the arm's reach, which on 2026-10-26 left a 0.035 m band.
    """

    stopping: StoppingModel
    # Nose-to-edge gap to stop at.
    standoff_gap: float
    front: ChairFrontPolicy = field(default_factory=ChairFrontPolicy)
    # What the approach drives onto: "chair" for the standoff point in front
    # of the latched edge, "tape" for the corridor markers. Either way the
    # latched edge bounds every command.
    target: str = "chair"
    # Slide the stop along the edge, for an object off the chair's centre.
    lateral_offset: float = 0.0
    # The robot's nose ahead of the LiDAR, and its half-width at the front.
    nose_x: float = 0.32
    half_width: float = 0.15
    latch_timeout: float = 5.0

    def __post_init__(self) -> None:
        if self.target not in ("chair", "tape"):
            raise ValueError("target must be 'chair' or 'tape'")
        for name in ("standoff_gap", "nose_x", "half_width", "latch_timeout"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not _is_finite_number(self.lateral_offset):
            raise ValueError("lateral_offset must be a finite number")


class MissionRunner:
    def __init__(
        self,
        mission: Mission,
        dock_policy: CorridorDockPolicy | None = None,
        approach_policy: ApproachPolicy | None = None,
        chair_policy: ChairDockPolicy | None = None,
    ) -> None:
        self.mission = mission
        self.dock_policy = dock_policy or CorridorDockPolicy()
        self.chair_policy = chair_policy
        self._approach = ApproachController(approach_policy)
        if chair_policy is not None:
            _check_stop_fits(chair_policy, self._approach.policy)
        self._latch = FrontLatch(chair_policy.front if chair_policy else None)
        self.reset()

    def reset(self) -> None:
        self._state = MissionState.IDLE
        self._index = 0
        self._approach.reset()
        self._latch.reset()
        self._latch_started = 0.0
        self._last_yaw: float | None = None

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def step_index(self) -> int:
        return self._index

    @property
    def latched_front(self) -> LatchedFront | None:
        return self._latch.latched

    def start(self, now: float) -> Action:
        self.reset()
        return self._enter(self._index, now)

    def on_navigation_result(self, succeeded: bool, now: float) -> Action:
        """Navigation, final alignment included, finished the current step."""
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
        fix_stamp: float | None = None,
        chair=None,
        pose: OdomPose | None = None,
        localization_ready: bool = True,
        manual_override: bool = False,
        execution_fault: bool = False,
        forward_clearance: float = float("inf"),
        minimum_clearance: float = 0.0,
    ) -> Action:
        """Advance a dock step by one control cycle.

        Called every cycle while `state` is LATCHING or DOCKING.

        - `fix`: the latest `CorridorFix`, or None when the tape pair was not
          cleanly seen; `fix_stamp` is the time of the scan it came from.
        - `chair`: the latest `FrontReading`, used while latching.
        - `pose`: `body_nav` in the odometry frame at this cycle. Required
          whenever a `chair_policy` is set.
        - `forward_clearance`: the live range straight ahead, a cross-check
          on top of the latched edge.
        """
        if self._state not in (MissionState.LATCHING, MissionState.DOCKING):
            return Action(ActionKind.NONE)
        if manual_override:
            return self._abort("manual override")
        if execution_fault:
            return self._abort("execution fault")
        if not localization_ready:
            return self._abort("localization not ready")
        if forward_clearance < minimum_clearance:
            return self._abort("forward clearance breached")
        if self._state is MissionState.LATCHING:
            return self._latching(now, chair, pose)

        chair_policy = self.chair_policy
        latched = self._latch.latched
        if chair_policy is not None:
            if pose is None:
                return self._abort("odometry pose missing")
            clearance = latched.nose_clearance(pose, chair_policy.nose_x, chair_policy.half_width)
            if clearance < chair_policy.stopping.min_clearance:
                return self._abort("chair clearance breached")

        if chair_policy is not None and chair_policy.target == "chair":
            tx, ty = pose.to_body(*latched.standoff(
                chair_policy.standoff_gap, chair_policy.nose_x, chair_policy.lateral_offset
            ))
            detection = Detection(now, tx, ty)
            self._last_yaw = _wrap(latched.normal_yaw - pose.yaw)
        elif fix is None:
            detection = None
        else:
            stamp = now if fix_stamp is None else fix_stamp
            detection = Detection(stamp, fix.mid_x, fix.mid_y)
            self._last_yaw = _wrap(fix.yaw_error - self.dock_policy.yaw_bias)

        command = self._approach.update(now, detection)
        if command.state is ApproachState.ABORTED:
            return self._abort(f"docking aborted: {command.reason}")
        if command.state is ApproachState.ARRIVED:
            error = self._approach.last_error or (0.0, 0.0)
            action = self._advance(now)
            if action.kind is ActionKind.GRASP:
                action = replace(action, residual=(error[0], error[1], self._last_yaw or 0.0))
            return action

        vx, vy, reason = command.vx, command.vy, command.reason
        if chair_policy is not None:
            vx, vy, cut = limit_toward(
                vx, vy, pose, latched, chair_policy.stopping,
                chair_policy.nose_x, chair_policy.half_width,
            )
            if cut:
                reason = "limited by stopping distance"
        return Action(ActionKind.DRIVE, vx=vx, vy=vy, reason=reason)

    def _latching(self, now: float, chair, pose: OdomPose | None) -> Action:
        policy = self.chair_policy
        latched = self._latch.add(chair, pose)
        if latched is None:
            if now - self._latch_started > policy.latch_timeout:
                return self._abort("chair front not latched")
            # Stand still while measuring: readings taken on the move smear.
            return Action(ActionKind.DRIVE, reason="latching chair front")
        clearance = latched.nose_clearance(pose, policy.nose_x, policy.half_width)
        if clearance < policy.stopping.min_clearance:
            return self._abort("chair clearance breached")
        self._state = MissionState.DOCKING
        self._start_approach(now)
        return Action(ActionKind.DRIVE, reason="chair front latched")

    def _start_approach(self, now: float) -> None:
        if self.chair_policy is not None and self.chair_policy.target == "chair":
            # The standoff point is handed over in body_nav, so it is reached
            # when it sits at the origin.
            self._approach.start(now, 0.0, 0.0)
        else:
            self._approach.start(
                now, self.dock_policy.stop_offset_x, self.dock_policy.stop_offset_y
            )

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
            self._last_yaw = None
            if self.chair_policy is not None:
                self._state = MissionState.LATCHING
                self._latch.reset()
                self._latch_started = now
                return Action(ActionKind.DRIVE, reason="latching chair front")
            self._state = MissionState.DOCKING
            self._start_approach(now)
            # No velocity yet: the first real command waits for a detection.
            return Action(ActionKind.DRIVE)
        self._state = MissionState.GRASPING
        return Action(ActionKind.GRASP)

    def _abort(self, reason: str) -> Action:
        self._state = MissionState.ABORTED
        self._approach.reset()
        return Action(ActionKind.ABORT, reason=reason)


def _check_stop_fits(chair: ChairDockPolicy, approach: ApproachPolicy) -> None:
    """Refuse a standoff the stopping distance cannot honour.

    The approach may arrive up to `approach_tolerance` short of the standoff,
    and at that point it may still be commanding the driver's slowest speed.
    From there the robot must stop before `min_clearance`; otherwise the
    stopping limit cuts the command under the deadband and the robot stalls
    outside tolerance -- or, without the limit, slides into the chair.
    """
    closest = chair.standoff_gap - approach.approach_tolerance
    needed = chair.stopping.min_clearance + chair.stopping.stopping_distance(approach.linear_deadband)
    if closest < needed:
        raise ValueError(
            f"standoff_gap {chair.standoff_gap:.3f} m leaves {closest:.3f} m at the "
            f"near edge of tolerance, but stopping from the driver's slowest speed "
            f"needs {needed:.3f} m; widen the gap or measure a shorter stop"
        )


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
