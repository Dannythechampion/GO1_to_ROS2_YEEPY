"""ROS-independent bounded scan-to-map pose quality search."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ScanPoint:
    x: float
    y: float


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: tuple[int, ...]
    occupied_threshold: int = 50

    def __post_init__(self) -> None:
        if self.width < 1 or self.height < 1:
            raise ValueError("grid dimensions must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("resolution must be finite and positive")
        if not all(math.isfinite(value) for value in (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError("grid origin must be finite")
        if len(self.cells) != self.width * self.height:
            raise ValueError("cells must match grid dimensions")

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        dx, dy = x - self.origin_x, y - self.origin_y
        if not math.isfinite(dx) or not math.isfinite(dy):
            return None
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        gx = (c * dx + s * dy) / self.resolution
        gy = (-s * dx + c * dy) / self.resolution
        if not math.isfinite(gx) or not math.isfinite(gy):
            return None
        gx = _normalize_cell_coordinate(gx, self.width)
        gy = _normalize_cell_coordinate(gy, self.height)
        ix, iy = math.floor(gx), math.floor(gy)
        return (ix, iy) if 0 <= ix < self.width and 0 <= iy < self.height else None


@dataclass(frozen=True)
class SearchWindow:
    translation_radius: float = 3.0
    translation_step: float = 0.5
    yaw_radius: float = math.pi / 2
    yaw_step: float = math.radians(15.0)
    hit_distance: float = 0.25
    ambiguity_margin: float = 0.05
    max_scan_points: int = 180

    def __post_init__(self) -> None:
        for name in ("translation_radius", "translation_step", "yaw_radius", "yaw_step", "hit_distance"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self.ambiguity_margin) or self.ambiguity_margin < 0.0:
            raise ValueError("ambiguity_margin must be finite and non-negative")
        if not isinstance(self.max_scan_points, int) or isinstance(self.max_scan_points, bool) or self.max_scan_points < 1:
            raise ValueError("max_scan_points must be a positive integer")


@dataclass(frozen=True)
class PoseScore:
    pose: Pose2D
    overlap: float
    mean_distance: float
    score: float
    points_used: int


@dataclass(frozen=True)
class SearchResult:
    best: PoseScore
    runner_up: PoseScore | None
    ambiguous: bool


def build_distance_field(grid: GridMap) -> tuple[float, ...]:
    """Return obstacle distance in metres for every grid cell."""
    distances = [math.inf] * len(grid.cells)
    queue: list[tuple[float, int]] = []
    for index, value in enumerate(grid.cells):
        if value >= grid.occupied_threshold:
            distances[index] = 0.0
            heapq.heappush(queue, (0.0, index))

    neighbours = (
        (-1, -1, math.sqrt(2.0)), (0, -1, 1.0), (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0), (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)), (0, 1, 1.0), (1, 1, math.sqrt(2.0)),
    )
    while queue:
        distance, index = heapq.heappop(queue)
        if distance != distances[index]:
            continue
        x, y = index % grid.width, index // grid.width
        for dx, dy, factor in neighbours:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < grid.width and 0 <= ny < grid.height):
                continue
            neighbour = ny * grid.width + nx
            candidate = distance + factor * grid.resolution
            if candidate < distances[neighbour]:
                distances[neighbour] = candidate
                heapq.heappush(queue, (candidate, neighbour))
    return tuple(distances)


def score_pose(
    grid: GridMap,
    field: Sequence[float],
    points: Sequence[ScanPoint],
    pose: Pose2D,
    hit_distance: float,
) -> PoseScore:
    if len(field) != len(grid.cells):
        raise ValueError("distance field must match grid dimensions")
    if not math.isfinite(hit_distance) or hit_distance <= 0.0:
        raise ValueError("hit_distance must be finite and positive")
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    total_distance = 0.0
    hits = 0
    used = 0
    outside = False
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            continue
        used += 1
        cell = grid.world_to_cell(pose.x + c * point.x - s * point.y, pose.y + s * point.x + c * point.y)
        if cell is None:
            outside = True
            continue
        distance = field[cell[1] * grid.width + cell[0]]
        total_distance += distance
        hits += distance <= hit_distance
    if outside:
        overlap, mean_distance = 0.0, math.inf
    else:
        mean_distance = total_distance / used if used else math.inf
        overlap = hits / used if used else 0.0
    return PoseScore(pose, overlap, mean_distance, _score_value(overlap, mean_distance), used)


def coarse_search(grid: GridMap, points: Sequence[ScanPoint], initial: Pose2D, window: SearchWindow) -> SearchResult:
    field = build_distance_field(grid)
    sampled_points = _evenly_sample(points, window.max_scan_points)
    poses = _candidate_poses(initial, window)
    bounds_by_yaw = {
        yaw: _rotated_point_bounds(points, yaw - grid.origin_yaw)
        for yaw in dict.fromkeys(pose.yaw for pose in poses)
    }
    candidates = []
    for pose in poses:
        score = score_pose(grid, field, sampled_points, pose, window.hit_distance)
        if _candidate_has_outside_endpoint(grid, points, pose, bounds_by_yaw[pose.yaw]):
            score = PoseScore(score.pose, 0.0, math.inf, _score_value(0.0, math.inf), score.points_used)
        candidates.append(score)
    candidates.sort(key=lambda item: (-item.score, -item.overlap, item.mean_distance, item.pose.x, item.pose.y, item.pose.yaw))
    best = candidates[0]
    runner_up = next((candidate for candidate in candidates[1:] if _is_distinct_pose(best.pose, candidate.pose)), None)
    ambiguous = runner_up is not None and best.score - runner_up.score < window.ambiguity_margin
    return SearchResult(best, runner_up, ambiguous)


def angle_distance(first: float, second: float) -> float:
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


def _candidate_poses(initial: Pose2D, window: SearchWindow) -> tuple[Pose2D, ...]:
    translations = _offsets(window.translation_radius, window.translation_step)
    yaws = _offsets(window.yaw_radius, window.yaw_step)
    return tuple(Pose2D(initial.x + dx, initial.y + dy, initial.yaw + dyaw) for dyaw in yaws for dy in translations for dx in translations)


def _offsets(radius: float, step: float) -> tuple[float, ...]:
    count = int(math.floor(radius / step))
    offsets = [index * step for index in range(-count, count + 1)]
    if not math.isclose(abs(offsets[0]), radius):
        offsets.extend((-radius, radius))
    return tuple(sorted(set(offsets)))


def _evenly_sample(points: Sequence[ScanPoint], maximum: int) -> tuple[ScanPoint, ...]:
    if len(points) <= maximum:
        return tuple(points)
    if maximum == 1:
        return (points[0],)
    return tuple(points[round(index * (len(points) - 1) / (maximum - 1))] for index in range(maximum))


def _rotated_point_bounds(points: Sequence[ScanPoint], yaw: float) -> tuple[float, float, float, float] | None:
    c, s = math.cos(yaw), math.sin(yaw)
    min_x = min_y = math.inf
    max_x = max_y = -math.inf
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            continue
        x = c * point.x - s * point.y
        y = s * point.x + c * point.y
        min_x, max_x = min(min_x, x), max(max_x, x)
        min_y, max_y = min(min_y, y), max(max_y, y)
    return None if math.isinf(min_x) else (min_x, max_x, min_y, max_y)


def _candidate_has_outside_endpoint(
    grid: GridMap,
    points: Sequence[ScanPoint],
    pose: Pose2D,
    bounds: tuple[float, float, float, float] | None,
) -> bool:
    status = _aabb_boundary_status(grid, pose, bounds)
    return status is True or (status is None and _has_outside_endpoint_exact(grid, points, pose))


def _aabb_boundary_status(
    grid: GridMap, pose: Pose2D, bounds: tuple[float, float, float, float] | None
) -> bool | None:
    if bounds is None:
        return False
    dx, dy = pose.x - grid.origin_x, pose.y - grid.origin_y
    if not math.isfinite(dx) or not math.isfinite(dy):
        return None
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    translation_x = (c * dx + s * dy) / grid.resolution
    translation_y = (-s * dx + c * dy) / grid.resolution
    if not math.isfinite(translation_x) or not math.isfinite(translation_y):
        return None
    min_x, max_x, min_y, max_y = bounds
    values = (
        (translation_x + min_x / grid.resolution, grid.width),
        (translation_x + max_x / grid.resolution, grid.width),
        (translation_y + min_y / grid.resolution, grid.height),
        (translation_y + max_y / grid.resolution, grid.height),
    )
    uncertainty = _aabb_uncertainty_cells(grid, pose, bounds)
    states = tuple(_classify_cell_coordinate(value, extent, uncertainty) for value, extent in values)
    if any(state == "outside" for state in states):
        return True
    return False if all(state == "inside" for state in states) else None


def _has_outside_endpoint_exact(grid: GridMap, points: Sequence[ScanPoint], pose: Pose2D) -> bool:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            continue
        x = pose.x + c * point.x - s * point.y
        y = pose.y + s * point.x + c * point.y
        if not math.isfinite(x) or not math.isfinite(y) or grid.world_to_cell(x, y) is None:
            return True
    return False


def _aabb_uncertainty_cells(grid: GridMap, pose: Pose2D, bounds: tuple[float, float, float, float]) -> float:
    scale = max(
        1.0,
        abs(pose.x), abs(pose.y), abs(grid.origin_x), abs(grid.origin_y),
        *(abs(value) for value in bounds),
    ) / grid.resolution
    return 128.0 * math.ulp(scale)


def _classify_cell_coordinate(value: float, extent: int, uncertainty: float) -> str:
    if not math.isfinite(value):
        return "uncertain"
    value = _normalize_cell_coordinate(value, extent)
    if value < -uncertainty or value > extent + uncertainty:
        return "outside"
    if uncertainty < value < extent - uncertainty:
        return "inside"
    return "uncertain"


def _cell_coordinate_is_inside(value: float, extent: int) -> bool:
    value = _normalize_cell_coordinate(value, extent)
    return 0.0 <= value < extent


def _normalize_cell_coordinate(value: float, extent: int) -> float:
    """Snap only round-off-scale values to cell-grid boundaries."""
    tolerance = 32.0 * max(math.ulp(value), math.ulp(float(extent)), math.ulp(1.0))
    if abs(value) <= tolerance:
        return 0.0
    if abs(value - extent) <= tolerance:
        return float(extent)
    return value


def _score_value(overlap: float, mean_distance: float) -> float:
    return overlap - 0.20 * min(mean_distance, 1.0)


def _is_distinct_pose(first: Pose2D, second: Pose2D) -> bool:
    return math.hypot(first.x - second.x, first.y - second.y) >= 0.75 or angle_distance(first.yaw, second.yaw) >= math.radians(20.0)
