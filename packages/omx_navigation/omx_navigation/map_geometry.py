"""ROS-independent occupancy-map geometry for localization and goals."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from statistics import median
from typing import Iterable, Optional, Sequence, Tuple


@dataclass
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Tuple[int, ...]
    _clearance_distances: Tuple[float, ...] = field(init=False, repr=False)
    _occupied_distances: Tuple[float, ...] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError("map dimensions and resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy data size does not match map dimensions")
        self._clearance_distances = self._build_distance_field(include_unknown=True)
        self._occupied_distances = self._build_distance_field(include_unknown=False)

    def world_to_local(self, x: float, y: float) -> Tuple[float, float]:
        """Return map-local coordinates, accounting for a rotated map origin."""
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        local_x, local_y = self.world_to_local(x, y)
        column = math.floor(local_x / self.resolution)
        row = math.floor(local_y / self.resolution)
        if column < 0 or row < 0 or column >= self.width or row >= self.height:
            return None
        return int(column), int(row)

    def occupancy(self, column: int, row: int) -> int:
        return self.data[column + row * self.width]

    def obstacle_distance(self, column: int, row: int) -> float:
        return self._clearance_distances[column + row * self.width]

    def occupied_distance(self, column: int, row: int) -> float:
        return self._occupied_distances[column + row * self.width]

    def boundary_distance(self, x: float, y: float) -> float:
        local_x, local_y = self.world_to_local(x, y)
        return min(
            local_x,
            local_y,
            self.width * self.resolution - local_x,
            self.height * self.resolution - local_y,
        )

    def has_hazard_within(self, x: float, y: float, clearance: float) -> bool:
        """Check exact point-to-cell-area distance in the local clearance window."""
        local_x, local_y = self.world_to_local(x, y)
        radius = max(0.0, float(clearance))
        first_column = max(0, math.floor((local_x - radius) / self.resolution))
        last_column = min(
            self.width - 1, math.floor((local_x + radius) / self.resolution)
        )
        first_row = max(0, math.floor((local_y - radius) / self.resolution))
        last_row = min(
            self.height - 1, math.floor((local_y + radius) / self.resolution)
        )
        for row in range(first_row, last_row + 1):
            bottom = row * self.resolution
            top = bottom + self.resolution
            for column in range(first_column, last_column + 1):
                if self.occupancy(column, row) >= 0 and self.occupancy(column, row) < 50:
                    continue
                left = column * self.resolution
                right = left + self.resolution
                dx = max(left - local_x, 0.0, local_x - right)
                dy = max(bottom - local_y, 0.0, local_y - top)
                if math.hypot(dx, dy) < radius:
                    return True
        return False

    def _build_distance_field(self, *, include_unknown: bool) -> Tuple[float, ...]:
        count = self.width * self.height
        distances = [math.inf] * count
        queue = []
        for index, occupancy in enumerate(self.data):
            if occupancy >= 50 or (include_unknown and occupancy < 0):
                distances[index] = 0.0
                heapq.heappush(queue, (0.0, index))
        neighbours = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )
        while queue:
            distance, index = heapq.heappop(queue)
            if distance != distances[index]:
                continue
            column = index % self.width
            row = index // self.width
            for dc, dr, cost in neighbours:
                nc, nr = column + dc, row + dr
                if nc < 0 or nr < 0 or nc >= self.width or nr >= self.height:
                    continue
                neighbour = nc + nr * self.width
                candidate = distance + cost * self.resolution
                if candidate < distances[neighbour]:
                    distances[neighbour] = candidate
                    heapq.heappush(queue, (candidate, neighbour))
        return tuple(distances)


@dataclass(frozen=True)
class GoalValidation:
    accepted: bool
    reason: str
    yaw: Optional[float] = None


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    values = tuple(float(value) for value in (qx, qy, qz, qw))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion values must be finite")
    qx, qy, qz, qw = values
    if abs(qx) > 1e-4 or abs(qy) > 1e-4:
        raise ValueError("goal orientation must be planar")
    norm = math.sqrt(sum(value * value for value in values))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("goal quaternion must be normalized")
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def validate_goal_pose(
    grid: OccupancyMap,
    x: float,
    y: float,
    qz: float,
    qw: float,
    minimum_clearance: float,
    qx: float = 0.0,
    qy: float = 0.0,
) -> GoalValidation:
    if not all(math.isfinite(float(value)) for value in (x, y, minimum_clearance)):
        return GoalValidation(False, "goal values must be finite")
    try:
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
    except ValueError as exc:
        return GoalValidation(False, str(exc))
    cell = grid.world_to_cell(x, y)
    if cell is None:
        return GoalValidation(False, "goal is outside map bounds")
    occupancy = grid.occupancy(*cell)
    if occupancy < 0:
        return GoalValidation(False, "goal is in unknown space")
    if occupancy >= 50:
        return GoalValidation(False, "goal cell is occupied")
    if (
        grid.boundary_distance(x, y) < minimum_clearance
        or grid.has_hazard_within(x, y, minimum_clearance)
    ):
        return GoalValidation(False, "goal clearance is below minimum")
    return GoalValidation(True, "accepted", yaw)


@dataclass(frozen=True)
class ScanScore:
    valid_beams: int
    median_residual: float
    p80_residual: float


def score_scan_pose(
    grid: OccupancyMap,
    *,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    minimum_beams: int = 100,
) -> ScanScore:
    residuals = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = sensor_yaw + angle_min + index * angle_increment
        endpoint_x = sensor_x + distance * math.cos(angle)
        endpoint_y = sensor_y + distance * math.sin(angle)
        cell = grid.world_to_cell(endpoint_x, endpoint_y)
        if cell is None:
            continue
        residuals.append(grid.occupied_distance(*cell))
    if len(residuals) < minimum_beams:
        raise ValueError(
            f"valid beams {len(residuals)} below required {minimum_beams}"
        )
    ordered = sorted(residuals)
    p80_index = max(0, math.ceil(0.8 * len(ordered)) - 1)
    return ScanScore(len(ordered), median(ordered), ordered[p80_index])
