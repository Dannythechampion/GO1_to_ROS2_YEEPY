"""ROS message-free conversion helpers for localization supervision."""

from __future__ import annotations

import math
from typing import Sequence

from omx_navigation.scan_map_quality import GridMap, ScanPoint


def laser_ranges_to_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_points: int,
) -> tuple[ScanPoint, ...]:
    """Return finite in-range scan points, uniformly bounded to ``max_points``."""
    if not isinstance(max_points, int) or isinstance(max_points, bool) or max_points < 1:
        raise ValueError("max_points must be a positive integer")
    values = (angle_min, angle_increment, range_min, range_max)
    if not all(_finite(value) for value in values) or range_min < 0.0 or range_max <= range_min:
        raise ValueError("laser limits must be finite and ordered")
    points = tuple(
        ScanPoint(distance * math.cos(angle_min + index * angle_increment),
                  distance * math.sin(angle_min + index * angle_increment))
        for index, distance in enumerate(ranges)
        if _finite(distance) and range_min <= distance <= range_max
    )
    return _evenly_sample(points, max_points)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract a normalized planar yaw from a finite, non-zero quaternion."""
    values = (x, y, z, w)
    if not all(_finite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0.0:
        raise ValueError("quaternion must not be zero")
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def grid_map_from_values(
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    cells: Sequence[int],
) -> GridMap:
    """Validate raw occupancy-grid fields and construct the search grid."""
    return GridMap(width, height, resolution, origin_x, origin_y, origin_yaw, tuple(cells))


def _evenly_sample(points: Sequence[ScanPoint], maximum: int) -> tuple[ScanPoint, ...]:
    if len(points) <= maximum:
        return tuple(points)
    if maximum == 1:
        return (points[0],)
    return tuple(points[round(index * (len(points) - 1) / (maximum - 1))] for index in range(maximum))


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
