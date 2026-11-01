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

from omx_navigation.reflector_detector import (
    ReflectorPolicy,
    bright_clusters,
    find_marker_pair,
)


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
    # Largest correction the dock will drive. Nav2 stops within 0.35 m of the
    # recorded pose, so a midpoint further out than this is not the pair that
    # was recorded there -- another taped spot, or something shiny -- and
    # driving onto it would take the robot somewhere nobody chose.
    max_correction: float = 0.60

    def __post_init__(self) -> None:
        for name in ("stop_offset_x", "stop_offset_y"):
            if not _is_finite_number(getattr(self, name)):
                raise ValueError(f"{name} must be a finite number")
        for name in ("max_absorbable_yaw", "max_correction"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

    def with_separation(self, separation: float, tolerance: float = 0.10) -> "CorridorDockPolicy":
        """The same policy, expecting the wall-to-wall gap measured at recording.

        The 2.00 m default only has to admit a corridor nobody has measured.
        Once a spot is recorded its real gap is known, and holding the pair to
        it is a much sharper test that the right two strips are in view.
        """
        if not _is_finite_number(separation) or separation <= 0.0:
            raise ValueError("separation must be finite and positive")
        if not _is_finite_number(tolerance) or tolerance <= 0.0:
            raise ValueError("tolerance must be finite and positive")
        markers = replace(self.markers, strip_separation=float(separation),
                          separation_tolerance=float(tolerance))
        return replace(self, markers=markers)


def unmeasured_corridor() -> CorridorDockPolicy:
    """The policy for looking at a spot whose wall gap is not known yet.

    Checking the tape and recording a spot are how the gap gets measured, so
    they cannot hold the pair to the 2.00 +- 0.30 m default: a 1.6 m corridor
    would never be recordable. Anything from 1 to 3 m wall to wall is
    accepted; docking is then held to what was measured.
    """
    return CorridorDockPolicy().with_separation(2.0, tolerance=1.0)


@dataclass(frozen=True)
class CorridorFix:
    """The markers' midpoint in `body_nav`, plus what the pair implies."""

    mid_x: float
    mid_y: float
    yaw_error: float
    width: float
    points: int

    def yaw_within(self, policy: CorridorDockPolicy) -> bool:
        return abs(self.yaw_error) <= policy.max_absorbable_yaw

    def correction(self, policy: CorridorDockPolicy) -> float:
        """How far the base would have to move to put the midpoint on target."""
        return math.hypot(self.mid_x - policy.stop_offset_x, self.mid_y - policy.stop_offset_y)


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
                      band_hi: float = 0.25, ignore_within: float = 0.0,
                      behind: bool = False) -> float:
    """Distance to the nearest return in a straight channel ahead of the robot.

    The keep-out that guards the backrest dock cannot work here: it measures
    the range to the dock, and the corridor dock's midpoint sits roughly at the
    robot. Collision safety instead comes from the markers being placed
    correctly, with this as the net underneath -- the chair is in front, and
    the robot's nose is 0.32 m ahead of the LiDAR, so the caller aborts when
    this falls to the nose plus its margin.

    `ignore_within` drops returns closer than that, which are the robot's own
    body, arm or basket rather than anything it could drive into. `behind`
    watches the channel to the rear instead, since the dock may also back up.

    Returns infinity when the channel is clear, which is a safe reading: it
    cannot be mistaken for an obstacle.
    """
    if not _is_finite_number(half_width) or half_width <= 0.0:
        raise ValueError("half_width must be finite and positive")
    if band_lo >= band_hi:
        raise ValueError("band_lo must be below band_hi")
    if not _is_finite_number(ignore_within) or ignore_within < 0.0:
        raise ValueError("ignore_within must be finite and not negative")
    nearest = math.inf
    for x, y, z, _reflectivity in points:
        if behind:
            x = -x
        if x <= ignore_within or abs(y) > half_width:
            continue
        if not (band_lo <= z <= band_hi):
            continue
        if x < nearest:
            nearest = x
    return nearest


def median_fix(fixes) -> CorridorFix | None:
    """One fix from several, robust to a frame that caught a bad cluster.

    Recording a spot or seeding localization happens once, standing still, so
    there is time to look for a second or two and take the median of each
    field instead of trusting whichever frame came first.
    """
    fixes = [f for f in fixes if f is not None]
    if not fixes:
        return None
    return CorridorFix(
        _median([f.mid_x for f in fixes]),
        _median([f.mid_y for f in fixes]),
        _median([f.yaw_error for f in fixes]),
        _median([f.width for f in fixes]),
        min(f.points for f in fixes),
    )


def tape_pose(robot_x: float, robot_y: float, robot_yaw: float, fix: CorridorFix):
    """Where a taped spot sits in the map, seen from a robot at a known pose.

    Returns `(x, y, yaw)`: the markers' midpoint, and the robot's heading
    squared to the corridor. That is what gets recorded, rather than the
    robot's own pose, so a spot recorded with the robot standing a little off
    still names the tape -- the place the dock will drive onto -- and Nav2 is
    sent there.

    The heading keeps the robot's facing: the pair looks the same from either
    end of the corridor, so it cannot tell which way along it is "forward".
    """
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    x = robot_x + cos_yaw * fix.mid_x - sin_yaw * fix.mid_y
    y = robot_y + sin_yaw * fix.mid_x + cos_yaw * fix.mid_y
    return x, y, _wrap(robot_yaw + fix.yaw_error)


def robot_pose_at(tape_x: float, tape_y: float, tape_yaw: float, fix: CorridorFix):
    """The inverse of `tape_pose`: where the robot is, from a recorded spot.

    Lets the start tape replace the operator's 2D Pose Estimate click. It
    assumes the robot faces the way it did when the spot was recorded, for the
    same reason `tape_pose` keeps the facing.
    """
    robot_yaw = _wrap(tape_yaw - fix.yaw_error)
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    x = tape_x - (cos_yaw * fix.mid_x - sin_yaw * fix.mid_y)
    y = tape_y - (sin_yaw * fix.mid_x + cos_yaw * fix.mid_y)
    return x, y, robot_yaw


@dataclass(frozen=True)
class StripSurvey:
    """One bright cluster, measured over its whole visible height."""

    x: float
    y: float
    points: int
    in_band: int
    z_lo: float
    z_hi: float
    peak: float

    @property
    def range(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing(self) -> float:
        return math.atan2(self.y, self.x)


def survey_strips(points, policy: CorridorDockPolicy | None = None,
                  min_points: int = 3) -> list:
    """Every tape-bright cluster in range, for checking how the tape was hung.

    The detector only looks inside its height band, so a strip that is too
    short or hung too high simply disappears from it. Surveying without the
    band shows how much of each strip the sensor sees and whether that covers
    the band, which is what an operator standing at the wall needs to know.
    """
    policy = policy or CorridorDockPolicy()
    out = []
    for cluster in bright_clusters(points, policy.markers, band=(-math.inf, math.inf)):
        if len(cluster) < min_points:
            continue
        heights = [item[3] for item in cluster]
        in_band = sum(1 for z in heights if policy.markers.band_lo <= z <= policy.markers.band_hi)
        out.append(StripSurvey(
            sum(item[1] for item in cluster) / len(cluster),
            sum(item[2] for item in cluster) / len(cluster),
            len(cluster),
            in_band,
            min(heights),
            max(heights),
            max(item[4] for item in cluster),
        ))
    return out


def tape_span(lidar_height: float, wall_distance: float = 1.0,
              policy: CorridorDockPolicy | None = None,
              pitch: float = math.radians(3.0)):
    """Floor heights between which a wall strip has to run, as (bottom, top).

    The detector counts only returns inside its height band, so the strip must
    cover that band where it meets the wall, plus what the body's pitch moves
    it by while the Go1 trots through the dock: 3 deg tilts the band 5 cm up
    or down on a wall 1 m away. Extra length above or below is harmless -- it
    is filtered out -- which is why an unmeasured LiDAR height is handled by
    taping long rather than by widening the band.
    """
    policy = policy or CorridorDockPolicy()
    for name, value in (("lidar_height", lidar_height), ("wall_distance", wall_distance)):
        if not _is_finite_number(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if not _is_finite_number(pitch) or pitch < 0.0:
        raise ValueError("pitch must be finite and not negative")
    margin = wall_distance * math.tan(pitch)
    return (lidar_height + policy.markers.band_lo - margin,
            lidar_height + policy.markers.band_hi + margin)


def _median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
