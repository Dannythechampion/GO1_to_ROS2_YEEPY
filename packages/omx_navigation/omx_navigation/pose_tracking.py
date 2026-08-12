"""Planar transform helpers used by localization supervision."""

from __future__ import annotations

import math

from omx_navigation.scan_map_quality import Pose2D


def compose_pose(parent_to_mid: Pose2D, mid_to_child: Pose2D) -> Pose2D:
    """Compose two finite planar transforms into ``parent -> child``."""
    values = (
        parent_to_mid.x,
        parent_to_mid.y,
        parent_to_mid.yaw,
        mid_to_child.x,
        mid_to_child.y,
        mid_to_child.yaw,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("planar transforms must be finite")
    c, s = math.cos(parent_to_mid.yaw), math.sin(parent_to_mid.yaw)
    return Pose2D(
        parent_to_mid.x + c * mid_to_child.x - s * mid_to_child.y,
        parent_to_mid.y + s * mid_to_child.x + c * mid_to_child.y,
        (parent_to_mid.yaw + mid_to_child.yaw + math.pi) % (2.0 * math.pi) - math.pi,
    )
