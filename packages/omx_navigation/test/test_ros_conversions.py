import math

import pytest

from omx_navigation.ros_conversions import (
    grid_map_from_values,
    laser_ranges_to_points,
    quaternion_to_yaw,
)


def test_laser_ranges_convert_to_bounded_evenly_sampled_points():
    points = laser_ranges_to_points(
        ranges=(1.0, float("inf"), 2.0, float("nan"), 3.0),
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.2,
        range_max=2.5,
        max_points=2,
    )

    assert len(points) == 2
    assert all(math.isfinite(point.x) and math.isfinite(point.y) for point in points)
    assert points[0].x == pytest.approx(1.0)
    assert points[1].x == pytest.approx(-2.0)


def test_quaternion_yaw_is_normalized_and_rejects_degenerate_values():
    assert quaternion_to_yaw(0.0, 0.0, 2.0, 2.0) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)


def test_grid_conversion_rejects_cell_count_mismatch():
    with pytest.raises(ValueError, match="cells"):
        grid_map_from_values(2, 2, 0.5, 0.0, 0.0, 0.0, (0, 1, 2))
