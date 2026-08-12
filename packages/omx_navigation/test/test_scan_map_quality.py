import math

import pytest

from omx_navigation.scan_map_quality import (
    GridMap,
    Pose2D,
    ScanPoint,
    SearchWindow,
    angle_distance,
    coarse_search,
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
