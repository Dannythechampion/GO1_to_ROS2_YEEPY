"""Place the base so the OpenMANIPULATOR-X can reach a target.

The base is deliberately placed coarsely: `xy_goal_tolerance` is 0.35 m,
because along a corridor the along-axis translation is not observable and a
narrower arrival window cannot be held (2026-10-25, see
migration/FIELD_SESSION_2026-10-25.md). The arm cannot work with that error --
its whole reach is 0.38 m -- so the precision comes from the target-relative
approach stage that runs after Nav2's goal completes, and the standoff
distance is budgeted against **that** stage's tolerance, never against
Nav2's.

Getting this the wrong way round is easy and silently impossible: budgeting
0.35 m out of a 0.38 m arm leaves a negative standoff.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class ArmReach:
    """OpenMANIPULATOR-X link geometry, read from the ROBOTIS URDF.

    Values come from `open_manipulator_description/urdf/open_manipulator_x/
    open_manipulator_x_arm.urdf.xacro` (ROBOTIS-GIT/open_manipulator), as the
    distance between consecutive joint origins:

        joint2 -> joint3   xyz 0.024 0 0.128   -> hypot = 0.13023
        joint3 -> joint4   xyz 0.124 0 0       -> 0.124
        joint4 -> ee       xyz 0.126 0 0       -> 0.126

    They sum to 0.38023 m, which is the 380 mm reach ROBOTIS publishes, so the
    chain is being read correctly. `shoulder_height` is joint1 -> joint2
    (xyz 0 0 0.0595), the height of the shoulder above the arm's own base.
    """

    shoulder_to_elbow: float = 0.13023
    elbow_to_wrist: float = 0.124
    wrist_to_tool: float = 0.126
    shoulder_height: float = 0.0595
    # Full extension is a singularity: no force, no payload, and no room to
    # correct. Grasps are budgeted inside a fraction of the envelope. 0.75 is a
    # starting value and has not been checked against this arm on the robot.
    usable_fraction: float = 0.75

    def __post_init__(self) -> None:
        for name in ("shoulder_to_elbow", "elbow_to_wrist", "wrist_to_tool", "shoulder_height"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0.0 < self.usable_fraction <= 1.0:
            raise ValueError("usable_fraction must be within (0.0, 1.0]")

    @property
    def max_reach(self) -> float:
        """Distance from the shoulder to the tool with the arm straight out."""
        return self.shoulder_to_elbow + self.elbow_to_wrist + self.wrist_to_tool

    @property
    def usable_reach(self) -> float:
        return self.max_reach * self.usable_fraction

    def horizontal_reach(self, height_above_shoulder: float) -> float:
        """How far forward the tool reaches for a target at this height.

        The envelope is a sphere around the shoulder, so a target above or
        below it costs horizontal reach. Returns 0.0 when the height alone
        already spends the budget, which means no standoff can work and the
        arm has to be raised or lowered instead.
        """
        if not _is_finite_number(height_above_shoulder):
            raise ValueError("height_above_shoulder must be a finite number")
        usable = self.usable_reach
        if abs(height_above_shoulder) >= usable:
            return 0.0
        return math.sqrt(usable * usable - height_above_shoulder * height_above_shoulder)


@dataclass(frozen=True)
class StandoffPolicy:
    """Where the base must stand for the arm to work.

    `arm_base_offset_x` and `arm_base_height` describe where the arm is bolted
    to the Go1, measured in `body_nav` -- that is, relative to the MID-360,
    because `body_nav` is the planarized FAST-LIO frame and its origin sits at
    the sensor. The defaults are the 2026-10-25 build: the arm is on the Go1's
    own mounting point, 0.15 m forward of the LiDAR and 0.19 m below it (the
    LiDAR having been raised 13 cm onto the printed riser). **Re-measure these
    if the mount changes**; nothing else in the stack models the arm's
    position.

    Note that `body_nav` has z forced to 0 at whatever height the sensor
    started at, so these are sensor-relative, not floor-relative.

    `approach_tolerance` is what the target-relative approach stage promises to
    deliver, not Nav2's `xy_goal_tolerance`. Nav2's job is only to get close
    enough that the target is in the detector's field of view.
    """

    arm_base_offset_x: float = 0.15
    arm_base_height: float = -0.19
    approach_tolerance: float = 0.05
    # Clearance between the tool and the target at the moment of the grasp.
    grasp_margin: float = 0.03
    arm: ArmReach = field(default_factory=ArmReach)

    def __post_init__(self) -> None:
        for name in ("arm_base_offset_x", "arm_base_height", "approach_tolerance", "grasp_margin"):
            value = getattr(self, name)
            if not _is_finite_number(value):
                raise ValueError(f"{name} must be a finite number")
        for name in ("approach_tolerance", "grasp_margin"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")

    def shoulder_height(self) -> float:
        """Height of the arm's shoulder in `body_nav`."""
        return self.arm_base_height + self.arm.shoulder_height

    def reach_budget(self, target_height: float) -> float:
        """Forward distance from the shoulder that is left for the standoff.

        Negative when the arm cannot cover its own uncertainty: the approach
        stage has to get tighter, the arm has to move, or the target height has
        to change. Callers must check the sign rather than use the number.
        """
        above = target_height - self.shoulder_height()
        return (
            self.arm.horizontal_reach(above)
            - self.approach_tolerance
            - self.grasp_margin
        )

    def reachable_height_range(self) -> tuple[float, float]:
        """Heights in `body_nav` the arm can work at, at any standoff.

        The envelope is a sphere around the shoulder, so a target far enough
        above or below it is out of reach however the base is placed. With the
        2026-10-25 mount the shoulder sits at -0.1305 m in `body_nav` and the
        usable radius is about 0.285 m, which puts the floor out of reach: the
        sensor stands well over 0.285 m above it. Targets have to be on a
        raised surface, or the arm has to move.
        """
        shoulder = self.shoulder_height()
        usable = self.arm.usable_reach
        return (shoulder - usable, shoulder + usable)

    def standoff_distance(self, target_height: float) -> float:
        """Distance from `body_nav` to the target the base should aim for."""
        budget = self.reach_budget(target_height)
        if budget <= 0.0:
            raise ValueError(
                "the arm cannot reach past its own approach tolerance at this "
                f"target height: budget {budget:.3f} m"
            )
        return self.arm_base_offset_x + budget


def standoff_pose(target_x: float, target_y: float, approach_yaw: float, distance: float):
    """Base pose at `distance` from the target, facing it along `approach_yaw`.

    `approach_yaw` is the heading the base will hold, so the base sits on the
    opposite side of the target from where it is pointing.
    """
    values = (target_x, target_y, approach_yaw, distance)
    if not all(_is_finite_number(value) for value in values):
        raise ValueError("standoff inputs must be finite numbers")
    if distance <= 0.0:
        raise ValueError("distance must be positive")
    return (
        target_x - distance * math.cos(approach_yaw),
        target_y - distance * math.sin(approach_yaw),
        _wrap(approach_yaw),
    )


def choose_approach_yaw(
    target_x: float,
    target_y: float,
    distance: float,
    is_free,
    candidates: int = 24,
    preferred_yaw: float | None = None,
):
    """Pick the heading whose standoff pose stands in free space.

    `is_free(x, y)` is supplied by the caller -- the costmap at runtime, a stub
    in tests -- so this stays a pure function. Among the free candidates the
    one closest to `preferred_yaw` wins, which keeps the choice stable between
    calls instead of flipping to the far side of the target whenever the
    costmap updates. Returns None when nothing is free, which the caller must
    report rather than drive into.
    """
    if not isinstance(candidates, int) or isinstance(candidates, bool) or candidates < 1:
        raise ValueError("candidates must be a positive integer")
    best = None
    best_cost = None
    for index in range(candidates):
        yaw = _wrap(-math.pi + 2.0 * math.pi * index / candidates)
        x, y, _ = standoff_pose(target_x, target_y, yaw, distance)
        if not is_free(x, y):
            continue
        cost = 0.0 if preferred_yaw is None else abs(_wrap(yaw - preferred_yaw))
        if best_cost is None or cost < best_cost:
            best = (x, y, yaw)
            best_cost = cost
    return best


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
