import math

import pytest

from omx_navigation.planar_transform import planarize_transform


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    sr, cr = math.sin(roll / 2.0), math.cos(roll / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    sy, cy = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quaternion(quaternion: tuple[float, float, float, float]) -> float:
    qx, qy, qz, qw = quaternion
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def angle_distance(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


@pytest.mark.parametrize(
    ("roll", "pitch", "yaw"),
    ((0.3, -0.2, 0.7), (-0.4, 0.1, -1.2), (0.0, 0.0, math.pi)),
)
def test_planar_transform_keeps_translation_and_yaw_only(roll, pitch, yaw):
    result = planarize_transform(1.0, 2.0, 0.4, *quaternion_from_rpy(roll, pitch, yaw))

    assert result.translation == pytest.approx((1.0, 2.0, 0.0))
    assert result.quaternion[:2] == pytest.approx((0.0, 0.0), abs=1e-12)
    assert angle_distance(yaw_from_quaternion(result.quaternion), yaw) < 1e-9


def test_planar_transform_normalizes_a_scaled_input_quaternion():
    quaternion = quaternion_from_rpy(0.4, -0.3, -2.7)

    result = planarize_transform(3.0, -2.0, 1.0, *(value * 3.0 for value in quaternion))

    assert result.quaternion[2:] == pytest.approx(
        (math.sin(-2.7 / 2.0), math.cos(-2.7 / 2.0)), abs=1e-12
    )


def test_zero_norm_quaternion_is_rejected():
    with pytest.raises(ValueError, match="quaternion"):
        planarize_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_near_zero_norm_quaternion_is_rejected_before_normalization():
    with pytest.raises(ValueError, match="quaternion"):
        planarize_transform(0.0, 0.0, 0.0, 0.0, 0.0, 1e-13, 0.0)


@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
def test_nonfinite_transform_components_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        planarize_transform(value, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
