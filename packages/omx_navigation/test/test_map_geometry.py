import math

import pytest

from omx_navigation.map_geometry import (
    OccupancyMap,
    quaternion_to_yaw,
    score_scan_pose,
    validate_goal_pose,
)


def make_map(width=10, height=10, resolution=1.0, data=None, **origin):
    return OccupancyMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin.get("origin_x", 0.0),
        origin_y=origin.get("origin_y", 0.0),
        origin_yaw=origin.get("origin_yaw", 0.0),
        data=tuple(data if data is not None else [0] * (width * height)),
    )


def test_world_to_cell_applies_rotated_map_origin():
    grid = make_map(origin_x=10.0, origin_y=20.0, origin_yaw=math.pi / 2.0)
    assert grid.world_to_cell(9.5, 21.5) == (1, 0)
    assert grid.world_to_cell(50.0, 50.0) is None


def test_goal_rejects_occupied_unknown_and_insufficient_clearance():
    data = [0] * 25
    data[2 + 2 * 5] = 100
    data[4 + 4 * 5] = -1
    grid = make_map(width=5, height=5, data=data)

    assert not validate_goal_pose(grid, 2.5, 2.5, 0.0, 1.0, 0.35).accepted
    assert not validate_goal_pose(grid, 4.5, 4.5, 0.0, 1.0, 0.35).accepted
    assert not validate_goal_pose(grid, 1.5, 2.5, 0.0, 1.0, 1.1).accepted
    assert validate_goal_pose(grid, 0.5, 0.5, 0.0, 1.0, 0.35).accepted


def test_goal_clearance_treats_outside_map_as_unknown():
    grid = make_map(width=5, height=5, resolution=0.1)
    assert not validate_goal_pose(grid, 0.05, 0.25, 0.0, 1.0, 0.35).accepted
    assert validate_goal_pose(grid, 0.25, 0.25, 0.0, 1.0, 0.20).accepted


def test_goal_clearance_uses_continuous_distance_to_obstacle_cell_edge():
    data = [0] * 9
    data[0] = 100
    grid = make_map(width=3, height=3, resolution=1.0, data=data)
    assert not validate_goal_pose(grid, 1.01, 0.5, 0.0, 1.0, 0.35).accepted
    assert validate_goal_pose(grid, 1.36, 0.5, 0.0, 1.0, 0.35).accepted


@pytest.mark.parametrize(
    "quaternion",
    [
        (float("nan"), 1.0),
        (0.0, 0.0),
        (0.0, 2.0),
    ],
)
def test_quaternion_validation_rejects_invalid_planar_orientation(quaternion):
    with pytest.raises(ValueError):
        quaternion_to_yaw(0.0, 0.0, quaternion[0], quaternion[1])


def test_quaternion_to_yaw_accepts_normalized_planar_rotation():
    half = math.sin(math.pi / 4.0)
    assert quaternion_to_yaw(0.0, 0.0, half, half) == pytest.approx(math.pi / 2.0)


def test_scan_score_uses_map_obstacle_distance_and_minimum_beams():
    width = height = 100
    data = [0] * (width * height)
    for row in range(height):
        data[50 + row * width] = 100
    grid = make_map(width=width, height=height, resolution=0.1, data=data)
    ranges = [5.05] * 100

    score = score_scan_pose(
        grid,
        ranges=ranges,
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        sensor_x=0.0,
        sensor_y=5.05,
        sensor_yaw=0.0,
        minimum_beams=100,
    )
    assert score.valid_beams == 100
    assert score.median_residual == pytest.approx(0.0)
    assert score.p80_residual == pytest.approx(0.0)

    with pytest.raises(ValueError, match="valid beams"):
        score_scan_pose(
            grid,
            ranges=ranges[:99],
            angle_min=0.0,
            angle_increment=0.0,
            range_min=0.1,
            range_max=10.0,
            sensor_x=0.0,
            sensor_y=5.05,
            sensor_yaw=0.0,
            minimum_beams=100,
        )


def test_scan_score_does_not_treat_unknown_as_a_perfect_obstacle_match():
    data = [0] * 100
    data[55] = -1
    data[56] = 100
    grid = make_map(width=10, height=10, resolution=1.0, data=data)
    score = score_scan_pose(
        grid,
        ranges=[5.5] * 100,
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        sensor_x=0.0,
        sensor_y=5.5,
        sensor_yaw=0.0,
        minimum_beams=100,
    )
    assert score.median_residual == pytest.approx(1.0)
