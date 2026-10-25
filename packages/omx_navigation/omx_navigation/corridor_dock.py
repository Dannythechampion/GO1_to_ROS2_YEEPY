"""Dock to two tape markers on opposite corridor walls.

The chair stands in the middle of a corridor, so the stopping spot can be
marked on the walls beside it instead of on the chair itself. That is better
than taping the backrest for four reasons, three of them measured on
2026-10-26 with the robot parked in front of the chair:

- **No occlusion.** The side sectors carried 9,600-10,800 points per 15 deg,
  against a few hundred forward where the arm and its basket sit.
- **The middle of the sensor's FOV.** Walls were 0.97-1.04 m away, so markers
  at the LiDAR's own height sit near 0 deg elevation. Tape on the backrest
  would be around +40 deg, close to the +52 deg top of the MID-360's field.
- **Collision stops dictating the geometry.** Tape on the backrest sits behind
  the thing the robot must not hit, so the seat's depth forced the standoff
  and left a 0.035 m feasible band. Markers beside the stopping spot put that
  distance back under our control.
- **The along-corridor direction becomes observable.** Matching wall surfaces
  cannot fix position along a corridor -- that degeneracy is what made the
  2026-10-25 drive hunt -- but two discrete markers at known places do.

The markers are placed so that the robot, parked correctly, sees them abeam.
Their midpoint is then what the approach controller drives onto, which needs
no change to that controller: the node passes the midpoint as the detection
and `stop_offset` as the desired position.

Yaw is reported but never acted on. The base does not rotate here -- that is
what keeps it clear of the driver's yaw deadband -- and Nav2's stateful goal
checker has already left the heading inside `yaw_goal_tolerance`. The residual
belongs to the arm, whose joint1 absorbs it directly; at 0.36 m, 8.6 deg of
base yaw is 0.054 m of lateral error at the object, which is the arm's to
remove and not the base's.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from omx_navigation.reflector_detector import ReflectorPolicy, find_marker_pair


def _corridor_markers() -> ReflectorPolicy:
    """Marker policy for strips facing each other across a corridor."""
    return ReflectorPolicy(
        # Walls measured at 0.97-1.04 m, so a marker at the LiDAR's height sits
        # near 0 deg elevation. The band stays narrow because nothing else
        # competes once the intensity threshold has run.
        band_lo=-0.10,
        band_hi=0.20,
        range_min=0.30,
        range_max=2.00,
        cluster_gap=0.15,
        min_points_per_strip=20,
        # Wall to wall rather than two strips on one surface.
        strip_separation=2.00,
        separation_tolerance=0.30,
        # Only the walls. Ahead is the arm, the basket and the chair; behind
        # is the rest of the corridor. Anything bright there would otherwise
        # be a third cluster and veto the pair on every scan. 30 deg is where
        # a marker on a wall 1 m away leaves the 2 m range anyway.
        bearing_min=math.radians(30.0),
        bearing_max=math.radians(150.0),
    )


@dataclass(frozen=True)
class CorridorDockPolicy:
    markers: ReflectorPolicy = field(default_factory=_corridor_markers)
    # Where the robot should sit relative to the markers' midpoint, in
    # body_nav. Zero means abeam and centred; the markers are placed so that
    # this is also where the arm can reach the object.
    stop_offset_x: float = 0.0
    stop_offset_y: float = 0.0
    # Largest yaw the arm is expected to absorb. Beyond it the detection is
    # reported but flagged, because the base will not turn to fix it.
    max_absorbable_yaw: float = math.radians(10.0)
    # Yaw the pair reads when the robot is parked right. Tape never goes up
    # exactly opposite; 5 cm of stagger across a 2 m corridor is 1.4 deg.
    yaw_bias: float = 0.0

    def __post_init__(self) -> None:
        for name in ("stop_offset_x", "stop_offset_y", "yaw_bias"):
            if not _is_finite_number(getattr(self, name)):
                raise ValueError(f"{name} must be a finite number")
        if not _is_finite_number(self.max_absorbable_yaw) or self.max_absorbable_yaw <= 0.0:
            raise ValueError("max_absorbable_yaw must be finite and positive")


@dataclass(frozen=True)
class CorridorFix:
    """The markers' midpoint in `body_nav`, plus what the pair implies."""

    mid_x: float
    mid_y: float
    yaw_error: float
    width: float
    points: int

    def yaw_within(self, policy: CorridorDockPolicy) -> bool:
        return abs(_wrap(self.yaw_error - policy.yaw_bias)) <= policy.max_absorbable_yaw


def taught_policy(policy: CorridorDockPolicy, fix: CorridorFix) -> CorridorDockPolicy:
    """The policy that drives back to where `fix` was read.

    Park the robot where the arm reaches the object, read the markers there,
    and that reading becomes the target. Assuming the midpoint sits exactly
    at (0, 0) would ask the tape to be placed to a centimetre; teaching it
    absorbs whatever stagger and offset the tape went up with.
    """
    return replace(
        policy, stop_offset_x=fix.mid_x, stop_offset_y=fix.mid_y, yaw_bias=fix.yaw_error
    )


def corridor_fix(points, policy: CorridorDockPolicy | None = None) -> CorridorFix | None:
    """Find the wall markers and report where their midpoint sits.

    `points` is any iterable of (x, y, z, reflectivity) in the LiDAR frame,
    which is `body_nav`'s origin, so nothing has to be transformed first.
    """
    policy = policy or CorridorDockPolicy()
    found = find_marker_pair(points, policy.markers)
    if found is None:
        return None
    first, second, count = found

    # Order left then right so the yaw sign does not depend on which marker the
    # bearing sort happened to reach first.
    left, right = (first, second) if first[1] >= second[1] else (second, first)
    width = math.hypot(left[0] - right[0], left[1] - right[1])
    mid_x = (left[0] + right[0]) / 2.0
    mid_y = (left[1] + right[1]) / 2.0
    # Parked square, the right-to-left vector points along +y. Whatever it is
    # short of that is the rotation the robot would have to apply.
    yaw_error = _wrap(math.atan2(left[1] - right[1], left[0] - right[0]) - math.pi / 2.0)
    return CorridorFix(mid_x, mid_y, yaw_error, width, count)


def forward_clearance(points, half_width: float = 0.20, band_lo: float = -0.15,
                      band_hi: float = 0.25, ignore_within: float = 0.32) -> float:
    """Distance to the nearest return in a straight channel ahead of the robot.

    The keep-out that guards the backrest dock cannot work here: it measures
    the range to the dock, and the corridor dock's midpoint sits roughly at the
    robot. Collision safety instead comes from the markers being placed
    correctly, with this as the net underneath -- the chair is in front, and
    the robot's nose is 0.32 m ahead of the LiDAR, so the caller aborts when
    this falls to the nose plus its margin.

    Returns infinity when the channel is clear. That is not proof of a clear
    path close in: at the 0.33 m the seat edge sits when parked, the -7 deg
    bottom of the field sees only 0.04 m under the sensor, so the seat can
    drop out of view. The latched front in `chair_front` is what guards that
    stretch; this is a live cross-check on top of it.

    Returns within `ignore_within` -- the nose, 0.32 m ahead of the LiDAR --
    are the robot's own arm and basket and are skipped; counting them would
    read as an obstacle on every scan.
    """
    if not _is_finite_number(half_width) or half_width <= 0.0:
        raise ValueError("half_width must be finite and positive")
    if band_lo >= band_hi:
        raise ValueError("band_lo must be below band_hi")
    if not _is_finite_number(ignore_within) or ignore_within < 0.0:
        raise ValueError("ignore_within must be finite and non-negative")
    nearest = math.inf
    for x, y, z, _reflectivity in points:
        if x <= ignore_within or abs(y) > half_width:
            continue
        if not (band_lo <= z <= band_hi):
            continue
        if x < nearest:
            nearest = x
    return nearest


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
