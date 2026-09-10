import math
import random

import pytest

from omx_navigation.scan_map_quality import (
    _candidate_has_outside_endpoint,
    _candidate_poses,
    _is_distinct_pose,
    _rotated_point_bounds,
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


def test_coarse_search_reuses_a_valid_supplied_distance_field(monkeypatch):
    import omx_navigation.scan_map_quality as quality

    grid = corridor_map()
    field = build_distance_field(grid)
    monkeypatch.setattr(
        quality,
        "build_distance_field",
        lambda _grid: (_ for _ in ()).throw(AssertionError("field rebuilt")),
    )

    result = quality.coarse_search(
        grid,
        symmetric_corridor_scan(),
        Pose2D(5.0, 5.0, 0.0),
        SearchWindow(0.5, 0.5, math.radians(15), math.radians(15)),
        field=field,
    )

    assert result.best.points_used == 2


def test_coarse_search_rejects_a_supplied_field_with_wrong_size():
    with pytest.raises(ValueError, match="distance field"):
        coarse_search(
            corridor_map(),
            symmetric_corridor_scan(),
            Pose2D(5.0, 5.0, 0.0),
            SearchWindow(0.5, 0.5, math.radians(15), math.radians(15)),
            field=(0.0,),
        )


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
    assert _is_distinct_pose(Pose2D(0.0, 0.0, 0.0), Pose2D(0.75, 0.0, 0.0))
    grid = GridMap(10, 10, 1.0, -5.0, -5.0, 0.0, (100,) * 100)
    result = coarse_search(
        grid, (ScanPoint(0.0, 0.0),), Pose2D(0.0, 0.0, 0.0),
        SearchWindow(0.75, 0.75, math.radians(1), math.radians(1)),
    )
    assert result.runner_up is not None
    assert math.hypot(
        result.runner_up.pose.x - result.best.pose.x,
        result.runner_up.pose.y - result.best.pose.y,
    ) >= 0.75


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
    assert result.best.pose == Pose2D(-3.0, 0.0, -math.pi / 2)


def test_coarse_search_disqualifies_an_outside_endpoint_omitted_by_downsampling():
    grid = GridMap(3, 3, 1.0, 0.0, 0.0, 0.0, (100,) * 9)
    points = [ScanPoint(0.0, 0.0)] * 181
    points[90] = ScanPoint(100.0, 0.0)
    result = coarse_search(
        grid,
        tuple(points),
        Pose2D(1.0, 1.0, 0.0),
        SearchWindow(0.5, 0.5, math.radians(15), math.radians(15), max_scan_points=180),
    )
    assert result.best.overlap == 0.0
    assert math.isinf(result.best.mean_distance)
    assert result.best.score == pytest.approx(-0.20)


def test_coarse_search_checks_outside_endpoints_in_a_rotated_map_frame():
    grid = GridMap(3, 3, 1.0, 10.0, 20.0, math.pi / 2, (100,) * 9)
    result = coarse_search(
        grid,
        (ScanPoint(0.0, 0.0), ScanPoint(2.0, 0.0)),
        Pose2D(8.5, 21.5, math.pi / 2),
        SearchWindow(0.1, 0.1, math.radians(1), math.radians(1)),
    )
    assert result.best.score == pytest.approx(-0.20)


class CountingPoints:
    def __init__(self, points):
        self.points = tuple(points)
        self.iterations = 0

    def __len__(self):
        return len(self.points)

    def __getitem__(self, index):
        return self.points[index]

    def __iter__(self):
        self.iterations += 1
        return iter(self.points)


def test_original_scan_boundary_check_scales_with_yaw_count_not_candidate_count():
    points = CountingPoints(ScanPoint(0.0, 0.0) for _ in range(181))
    grid = GridMap(100, 100, 1.0, 0.0, 0.0, 0.0, (100,) * 10000)
    coarse_search(grid, points, Pose2D(50.0, 50.0, 0.0), SearchWindow())
    assert points.iterations <= 13


def _world_from_cell_coordinates(grid: GridMap, x: float, y: float) -> tuple[float, float]:
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    return grid.origin_x + c * x - s * y, grid.origin_y + s * x + c * y


def _pose_for_endpoint(grid: GridMap, endpoint_x: float, endpoint_y: float, point: ScanPoint, yaw: float) -> Pose2D:
    c, s = math.cos(yaw), math.sin(yaw)
    world_x, world_y = _world_from_cell_coordinates(
        grid, endpoint_x * grid.resolution, endpoint_y * grid.resolution
    )
    return Pose2D(world_x - c * point.x + s * point.y, world_y - s * point.x - c * point.y, yaw)


def test_rotated_grid_strict_edges_match_world_to_cell_and_aabb():
    grid = GridMap(7, 5, 0.3, 10.1, -3.7, 0.001, (100,) * 35)
    point = ScanPoint(1.1, -0.7)
    yaw = -3.14
    lower = _pose_for_endpoint(grid, 0.0, 1.5, point, yaw)
    upper = _pose_for_endpoint(grid, 7.0, 1.5, point, yaw)
    bounds = _rotated_point_bounds((point,), yaw - grid.origin_yaw)
    lower_endpoint = (lower.x + math.cos(yaw) * point.x - math.sin(yaw) * point.y,
                      lower.y + math.sin(yaw) * point.x + math.cos(yaw) * point.y)
    upper_endpoint = (upper.x + math.cos(yaw) * point.x - math.sin(yaw) * point.y,
                      upper.y + math.sin(yaw) * point.x + math.cos(yaw) * point.y)

    assert grid.world_to_cell(*lower_endpoint) == (0, 1)
    assert _candidate_has_outside_endpoint(grid, (point,), lower, bounds) is False
    assert grid.world_to_cell(*upper_endpoint) is None
    assert _candidate_has_outside_endpoint(grid, (point,), upper, bounds) is True


def test_rotated_upper_edge_omitted_by_downsampling_still_disqualifies_search():
    grid = GridMap(7, 5, 0.3, 10.1, -3.7, 0.001, (100,) * 35)
    point = ScanPoint(1.1, -0.7)
    yaw = -3.14
    points = [ScanPoint(0.0, 0.0)] * 181
    points[90] = point
    result = coarse_search(
        grid,
        tuple(points),
        _pose_for_endpoint(grid, 7.0, 1.5, point, yaw),
        SearchWindow(1e-16, 1e-16, 1e-16, 1e-16, max_scan_points=180),
    )
    assert result.best.score == pytest.approx(-0.20)


def test_aabb_boundary_check_agrees_with_direct_endpoint_checks():
    grid = GridMap(11, 9, 0.3, 10.1, -3.7, 0.73, (100,) * 99)
    generator = random.Random(7)
    for _ in range(100):
        pose = Pose2D(
            generator.uniform(8.0, 14.0),
            generator.uniform(-6.0, 0.0),
            generator.uniform(-math.pi, math.pi),
        )
        points = tuple(ScanPoint(generator.uniform(-2.0, 2.0), generator.uniform(-2.0, 2.0)) for _ in range(8))
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        direct_outside = any(
            grid.world_to_cell(pose.x + c * point.x - s * point.y, pose.y + s * point.x + c * point.y) is None
            for point in points
        )
        bounds = _rotated_point_bounds(points, pose.yaw - grid.origin_yaw)
        assert _candidate_has_outside_endpoint(grid, points, pose, bounds) is direct_outside


def test_downsampled_review_counterexample_still_disqualifies_search():
    grid = GridMap(7, 5, 1.0, 1.3271779832532098, 0.08697108026444411, -1.3745088108833117, (100,) * 35)
    points = [ScanPoint(0.0, 0.0)] * 181
    points[90] = ScanPoint(0.15911595166784753, 1.485347693771672)
    result = coarse_search(
        grid,
        tuple(points),
        Pose2D(2.0374671223870227, -3.3647327178525592, -2.8542972899048724),
        SearchWindow(1e-16, 1e-16, 1e-16, 1e-16, max_scan_points=180),
    )
    assert result.best.score == pytest.approx(-0.20)


def test_world_to_cell_rejects_nonfinite_intermediate_coordinates():
    grid = GridMap(7, 5, 1.0, -1e308, 0.0, 0.0, (100,) * 35)
    assert grid.world_to_cell(1e308, 0.0) is None


def test_coarse_search_handles_overflowing_relative_yaw_with_exact_fallback():
    grid = GridMap(7, 5, 1.0, 0.0, 0.0, -1e308, (100,) * 35)
    pose = Pose2D(3.0, 2.0, 1e308)
    point = ScanPoint(0.0, 0.0)
    direct = score_pose(grid, build_distance_field(grid), (point,), pose, 0.25)
    result = coarse_search(
        grid,
        (point,),
        pose,
        SearchWindow(1e-16, 1e-16, 1e-16, 1e-16),
    )
    assert result.best.score == pytest.approx(direct.score)


def test_translation_candidates_stay_inside_euclidean_radius():
    initial = Pose2D(10.0, 20.0, 0.3)
    window = SearchWindow(3.0, 0.5, math.pi / 2, math.radians(15))
    poses = _candidate_poses(initial, window)
    translations = {(pose.x - initial.x, pose.y - initial.y) for pose in poses}

    assert all(math.hypot(dx, dy) <= 3.0 + 1e-12 for dx, dy in translations)
    assert (3.0, 3.0) not in translations
    assert {(0.0, 0.0), (-3.0, 0.0), (3.0, 0.0), (0.0, -3.0), (0.0, 3.0)} <= translations
    assert poses == _candidate_poses(initial, window)


def test_continuous_monitoring_keeps_score_when_one_endpoint_leaves_the_map():
    """`disqualify_outside=False` must degrade gracefully, not collapse.

    Disqualification is correct while choosing a pose but wrong while watching
    a verified one: on the Hanyang 9F map a single beam of 180 crossed the
    boundary on 8.2% of scans, and zeroing overlap there cancelled the active
    navigation goal every few seconds.
    """
    grid = GridMap(3, 3, 1.0, 0.0, 0.0, 0.0, (100,) * 9)
    field = build_distance_field(grid)
    points = (ScanPoint(0.0, 0.0),) * 99 + (ScanPoint(100.0, 100.0),)
    pose = Pose2D(0.5, 0.5, 0.0)
    disqualified = score_pose(grid, field, points, pose, hit_distance=0.25)
    monitored = score_pose(
        grid, field, points, pose, hit_distance=0.25, disqualify_outside=False
    )
    assert disqualified.overlap == 0.0
    assert monitored.overlap == pytest.approx(0.99)
    assert not math.isinf(monitored.mean_distance)
    assert monitored.points_used == disqualified.points_used == 100
