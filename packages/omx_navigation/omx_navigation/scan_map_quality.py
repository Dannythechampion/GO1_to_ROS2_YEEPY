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
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        gx = (c * dx + s * dy) / self.resolution
        gy = (-s * dx + c * dy) / self.resolution
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
    candidates = []
    for pose in _candidate_poses(initial, window):
        score = score_pose(grid, field, sampled_points, pose, window.hit_distance)
        if _has_outside_endpoint(grid, points, pose):
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


def _has_outside_endpoint(grid: GridMap, points: Sequence[ScanPoint], pose: Pose2D) -> bool:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            continue
        if grid.world_to_cell(pose.x + c * point.x - s * point.y, pose.y + s * point.x + c * point.y) is None:
            return True
    return False


def _score_value(overlap: float, mean_distance: float) -> float:
    return overlap - 0.20 * min(mean_distance, 1.0)


def _is_distinct_pose(first: Pose2D, second: Pose2D) -> bool:
    return math.hypot(first.x - second.x, first.y - second.y) >= 0.75 or angle_distance(first.yaw, second.yaw) >= math.radians(20.0)
