import math

import pytest

from omx_navigation.scan_map_quality import (
    GridMap,
    Pose2D,
    ScanPoint,
    SearchWindow,
    angle_distance,
    build_distance_field,
    coarse_search,
    score_pose,
)


def corridor_map() -> GridMap:
    width = height = 21
    cells = [0] * (width * height)
    for y in range(2, 19):
        cells[y * width + 5] = 100
        cells[y * width + 15] = 100
    cells[18 * width + 5 : 18 * width + 16] = [100] * 11
    return GridMap(width, height, 0.5, 0.0, 0.0, 0.0, tuple(cells))


def asymmetric_room_map() -> GridMap:
    width = height = 21
    cells = [0] * (width * height)
    for x in range(2, 19):
        cells[2 * width + x] = 100
        cells[18 * width + x] = 100
    for y in range(2, 19):
        cells[y * width + 2] = 100
        cells[y * width + 18] = 100
    for x, y in ((5, 6), (12, 5), (14, 14), (7, 16)):
        cells[y * width + x] = 100
    return GridMap(width, height, 0.5, 0.0, 0.0, 0.0, tuple(cells))


def scan_points_for_pose(grid: GridMap, pose: Pose2D) -> tuple[ScanPoint, ...]:
    occupied = ((5, 6), (12, 5), (14, 14), (7, 16), (2, 10), (18, 8))
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return tuple(
        ScanPoint(c * (x * grid.resolution - pose.x) + s * (y * grid.resolution - pose.y),
                  -s * (x * grid.resolution - pose.x) + c * (y * grid.resolution - pose.y))
        for x, y in occupied
    )


def symmetric_corridor_scan() -> tuple[ScanPoint, ...]:
    return (ScanPoint(-2.5, 0.0), ScanPoint(2.5, 0.0))


def test_coarse_search_corrects_two_meter_and_ninety_degree_error():
    grid = asymmetric_room_map()
    scan = scan_points_for_pose(grid, Pose2D(5.0, 5.0, 0.0))
    result = coarse_search(
        grid,
        scan,
        Pose2D(3.0, 5.0, math.pi / 2),
        SearchWindow(3.0, 0.5, math.pi / 2, math.radians(15)),
    )
    assert result.best.pose.x == pytest.approx(5.0, abs=0.51)
    assert result.best.pose.y == pytest.approx(5.0, abs=0.51)
    assert angle_distance(result.best.pose.yaw, 0.0) <= math.radians(15)
    assert result.best.overlap >= 0.80
    assert result.ambiguous is False


def test_symmetric_corridor_is_ambiguous():
    result = coarse_search(
        corridor_map(), symmetric_corridor_scan(), Pose2D(5.0, 5.0, 0.0),
        SearchWindow(3.0, 0.5, math.pi / 2, math.radians(30)),
    )
    assert result.ambiguous is True


def test_empty_nonfinite_and_outside_scan_never_looks_good():
    result = coarse_search(
        corridor_map(),
        (ScanPoint(float("nan"), 0.0), ScanPoint(100.0, 100.0)),
        Pose2D(-20.0, -20.0, 0.0),
        SearchWindow(1.0, 0.5, math.radians(30), math.radians(15)),
    )
    assert result.best.overlap == 0.0
    assert math.isinf(result.best.mean_distance)


def test_coarse_search_supports_a_single_bounded_scan_point():
    result = coarse_search(
        corridor_map(), symmetric_corridor_scan(), Pose2D(5.0, 5.0, 0.0),
        SearchWindow(0.5, 0.5, math.radians(15), math.radians(15), max_scan_points=1),
    )
    assert result.best.points_used == 1


def test_pose_with_one_outside_endpoint_is_disqualified_even_with_ninety_nine_hits():
    grid = GridMap(3, 3, 1.0, 0.0, 0.0, 0.0, (100,) * 9)
    score = score_pose(
        grid,
        build_distance_field(grid),
        (ScanPoint(0.0, 0.0),) * 99 + (ScanPoint(100.0, 100.0),),
        Pose2D(0.5, 0.5, 0.0),
        hit_distance=0.25,
    )
    assert score.overlap == 0.0
    assert math.isinf(score.mean_distance)
    assert score.score == pytest.approx(-0.20)


def test_distance_field_has_exact_orthogonal_and_diagonal_costs():
    grid = GridMap(3, 3, 0.5, 0.0, 0.0, 0.0, (0, 0, 0, 0, 100, 0, 0, 0, 0))
    field = build_distance_field(grid)
    assert field[1] == pytest.approx(0.5)
    assert field[0] == pytest.approx(math.sqrt(2.0) * 0.5)


def test_world_to_cell_honors_a_rotated_map_origin():
    grid = GridMap(5, 5, 1.0, 10.0, 20.0, math.pi / 2, (0,) * 25)
    assert grid.world_to_cell(7.5, 21.5) == (1, 2)


def test_score_pose_uses_the_specified_overlap_distance_formula():
    grid = GridMap(3, 1, 1.0, 0.0, 0.0, 0.0, (100, 0, 0))
    score = score_pose(
        grid, build_distance_field(grid), (ScanPoint(1.1, 0.0),), Pose2D(0.0, 0.0, 0.0), 0.25
    )
    assert score.overlap == 0.0
    assert score.mean_distance == pytest.approx(1.0)
    assert score.score == pytest.approx(-0.20)


def test_runner_up_accepts_the_exact_point_seventy_five_meter_boundary():
    grid = GridMap(10, 10, 1.0, -5.0, -5.0, 0.0, (100,) * 100)
    result = coarse_search(
        grid, (ScanPoint(0.0, 0.0),), Pose2D(0.0, 0.0, 0.0),
        SearchWindow(0.75, 0.75, math.radians(1), math.radians(1)),
    )
    assert result.runner_up is not None
    assert math.hypot(
        result.runner_up.pose.x - result.best.pose.x,
        result.runner_up.pose.y - result.best.pose.y,
    ) == pytest.approx(0.75)


def test_runner_up_accepts_the_exact_twenty_degree_boundary():
    grid = GridMap(10, 10, 1.0, -5.0, -5.0, 0.0, (100,) * 100)
    result = coarse_search(
        grid, (ScanPoint(0.0, 0.0),), Pose2D(0.0, 0.0, 0.0),
        SearchWindow(0.1, 0.1, math.radians(20), math.radians(20)),
    )
    assert result.runner_up is not None
    assert angle_distance(result.runner_up.pose.yaw, result.best.pose.yaw) == pytest.approx(math.radians(20))


def test_coarse_search_includes_translation_and_yaw_window_endpoints():
    grid = GridMap(20, 20, 1.0, -10.0, -10.0, 0.0, (100,) * 400)
    result = coarse_search(
        grid, (ScanPoint(0.0, 0.0),), Pose2D(0.0, 0.0, 0.0),
        SearchWindow(3.0, 3.0, math.pi / 2, math.pi / 2),
    )
    assert result.best.pose == Pose2D(-3.0, -3.0, -math.pi / 2)
