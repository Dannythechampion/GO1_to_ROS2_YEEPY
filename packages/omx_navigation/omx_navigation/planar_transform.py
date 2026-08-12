"""ROS-independent helpers for deriving a gravity-aligned base transform."""

from __future__ import annotations

import math
from dataclasses import dataclass


MINIMUM_QUATERNION_NORM = 1e-12


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
    # Unit quaternions can be uniformly scaled by normal transport/serialization,
    # so normalize any practical non-zero value.  Values below this floor are
    # numerical noise, not a meaningful orientation, and must not be amplified.
    if norm <= MINIMUM_QUATERNION_NORM:
        raise ValueError("quaternion norm is too small to represent an orientation")
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
