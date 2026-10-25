"""Find two retroreflective strips in a Livox cloud and report the dock pose.

The chair itself is a poor dock: measured on 2026-10-26 with the robot parked
in front of it, the backrest came back as a 0.595 m wide blob with a 0.039 m
line residual and only 0.168 m of height, because the seat, the legs and the
backrest merge into one cluster. Retroreflective tape replaces all of that
geometry with an intensity threshold.

Livox reports reflectivity per point. In that same ordinary indoor scene only
0.018% of 322,032 points reached 180, and 0.015% reached 200, so a threshold
up there isolates tape with essentially no competition. Two strips a known
distance apart give position, bearing and yaw from one scan, and the known
separation doubles as the rejection test.

Everything here is in the LiDAR frame, which is also `body_nav`'s origin, so
no extrinsic calibration stands between a detection and the controller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ReflectorPolicy:
    # 0.018% of an ordinary scene reached this; tape saturates near 255.
    reflectivity_threshold: float = 180.0
    # Band around the LiDAR's own height where the strips are mounted. The
    # backrest spans roughly -0.04..+0.30 in this frame, so the strips sit
    # inside that and well clear of the -7 deg bottom of the sensor's FOV.
    band_lo: float = -0.02
    band_hi: float = 0.26
    range_min: float = 0.20
    range_max: float = 2.00
    # Points further apart than this belong to different strips.
    cluster_gap: float = 0.08
    min_points_per_strip: int = 15
    # Known horizontal gap between the two strips, and how far off it may be
    # before the pair is rejected as something else that happens to be shiny.
    strip_separation: float = 0.30
    separation_tolerance: float = 0.05

    def __post_init__(self) -> None:
        for name in ("reflectivity_threshold", "range_min", "range_max",
                     "cluster_gap", "strip_separation", "separation_tolerance"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.band_lo >= self.band_hi:
            raise ValueError("band_lo must be below band_hi")
        if self.range_min >= self.range_max:
            raise ValueError("range_min must be below range_max")
        if not isinstance(self.min_points_per_strip, int) or isinstance(self.min_points_per_strip, bool) or self.min_points_per_strip < 3:
            raise ValueError("min_points_per_strip must be an integer of at least 3")


@dataclass(frozen=True)
class DockDetection:
    """Midpoint between the strips, and the direction the dock faces."""

    x: float
    y: float
    yaw: float
    separation: float
    points: int


def detect_dock(points, policy: ReflectorPolicy | None = None) -> DockDetection | None:
    """Return the dock pose, or None when the pair is not cleanly visible.

    `points` is any iterable of (x, y, z, reflectivity). Returning None rather
    than a best guess matters: the controller turns a missing detection into a
    stop, and a wrong dock pose next to a chair is worse than no dock pose.
    """
    policy = policy or ReflectorPolicy()
    bright = []
    for x, y, z, reflectivity in points:
        if reflectivity < policy.reflectivity_threshold:
            continue
        if not (policy.band_lo <= z <= policy.band_hi):
            continue
        distance = math.hypot(x, y)
        if not (policy.range_min <= distance <= policy.range_max):
            continue
        bright.append((math.atan2(y, x), x, y))
    if len(bright) < 2 * policy.min_points_per_strip:
        return None

    bright.sort()
    clusters = []
    current = [bright[0]]
    for previous, item in zip(bright, bright[1:]):
        if math.hypot(item[1] - previous[1], item[2] - previous[2]) > policy.cluster_gap:
            clusters.append(current)
            current = []
        current.append(item)
    clusters.append(current)

    strips = [c for c in clusters if len(c) >= policy.min_points_per_strip]
    if len(strips) != 2:
        # One strip means half the dock is occluded; three or more means
        # something else in the scene is also returning at tape intensity.
        return None

    first = _centroid(strips[0])
    second = _centroid(strips[1])
    separation = math.hypot(second[0] - first[0], second[1] - first[1])
    if abs(separation - policy.strip_separation) > policy.separation_tolerance:
        return None

    mid_x = (first[0] + second[0]) / 2.0
    mid_y = (first[1] + second[1]) / 2.0
    # The dock's surface runs between the strips; it faces along the normal,
    # taken towards the sensor so the yaw is the heading to hold on approach.
    along = math.atan2(second[1] - first[1], second[0] - first[0])
    normal = along + math.pi / 2.0
    if math.cos(normal) * mid_x + math.sin(normal) * mid_y > 0.0:
        normal += math.pi
    return DockDetection(
        mid_x, mid_y, _wrap(normal), separation, len(strips[0]) + len(strips[1])
    )


def _centroid(cluster):
    return (
        sum(item[1] for item in cluster) / len(cluster),
        sum(item[2] for item in cluster) / len(cluster),
    )


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
