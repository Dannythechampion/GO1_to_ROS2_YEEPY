"""ROS-independent helpers for deriving a gravity-aligned base transform."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PlanarTransform:
    """A transform constrained to x, y, and yaw."""

    translation: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


def planarize_transform(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> PlanarTransform:
    """Keep x/y and yaw from a transform while removing vertical tilt and height."""
    values = (x, y, z, qx, qy, qz, qw)
    if not all(_is_finite_number(value) for value in values):
        raise ValueError("transform components must be finite numbers")

    norm = math.hypot(qx, qy, qz, qw)
    if norm == 0.0:
        raise ValueError("quaternion must have a non-zero norm")
    qx, qy, qz, qw = (component / norm for component in (qx, qy, qz, qw))

    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return PlanarTransform(
        translation=(float(x), float(y), 0.0),
        quaternion=(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
    )


def _is_finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
