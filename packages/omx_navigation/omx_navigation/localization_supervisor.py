"""ROS orchestration for safe pose-graph localization initialization."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from threading import Lock

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from omx_navigation.localization_state import (
    ErrorCode,
    LocalizationPolicy,
    LocalizationState,
    LocalizationStateMachine,
    QualityObservation,
)
from omx_navigation.ros_conversions import (
    grid_map_from_values,
    laser_ranges_to_points,
    quaternion_to_yaw,
)
from omx_navigation.scan_map_quality import (
    Pose2D,
    SearchWindow,
    build_distance_field,
    coarse_search,
    score_pose,
)


_MESSAGES_KO = {
    ErrorCode.NONE: "정상",
    ErrorCode.INPUT_MISSING: "지도 또는 센서 입력이 오래되었거나 없습니다.",
    ErrorCode.POSE_OUTSIDE_MAP: "초기 자세가 map 좌표계 또는 지도 범위를 벗어났습니다.",
    ErrorCode.ALIGNMENT_TIMEOUT: "초기 자세 정렬 시간이 초과되었습니다.",
    ErrorCode.LOW_OVERLAP: "스캔과 지도의 겹침이 부족합니다.",
    ErrorCode.AMBIGUOUS: "정렬 결과가 모호합니다.",
    ErrorCode.ODOM_RESET: "오도메트리 재설정이 감지되었습니다.",
    ErrorCode.TF_CONFLICT: "AMCL TF 발행자 충돌이 감지되었습니다.",
    ErrorCode.EXTRINSIC_UNCALIBRATED: "외부 파라미터 보정이 필요합니다.",
}


def _coarse_search_with_field(grid, points, initial, window):
    """Build the immutable map field once and return it with the initial search."""
    field = build_distance_field(grid)
    return coarse_search(grid, points, initial, window, field), field


class LocalizationSupervisor(Node):
    """Connect pure scan-map quality checks to the ROS localization lifecycle."""

    def __init__(self) -> None:
        super().__init__("localization_supervisor")
        self._input_max_age = float(self.declare_parameter("input_max_age", 0.50).value)
        self._diagnostics_csv = str(self.declare_parameter("diagnostics_csv", "").value)
        self._window = SearchWindow(
            translation_radius=float(self.declare_parameter("coarse_search_translation_radius", 3.0).value),
            translation_step=float(self.declare_parameter("coarse_search_translation_step", 0.5).value),
            yaw_radius=float(self.declare_parameter("coarse_search_yaw_radius", math.pi / 2).value),
            yaw_step=float(self.declare_parameter("coarse_search_yaw_step", math.radians(15.0)).value),
        )
        self._machine = LocalizationStateMachine(LocalizationPolicy())
        self._camera_init_frame = str(self.declare_parameter("camera_init_frame", "camera_init").value)
        self._source_base_frame = str(self.declare_parameter("source_base_frame", "body").value)
        self._base_frame = str(self.declare_parameter("base_frame", "body_nav").value)
        self._grid = None
        self._points = None
        self._map_received_at = None
        self._scan_received_at = None
        self._initial_pose = None
        self._generation = 0
        self._scan_sequence = 0
        self._required_scan_sequence = None
        self._active_search_generation = None
        self._active_search_grid = None
        self._completed_search_generation = None
        self._search_future = None
        self._pending_search = None
        self._search_lock = Lock()
        self._search_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="localization_search")
        self._refined_pose = None
        self._search_pose_available = False
        self._overlap = 0.0
        self._ambiguity_margin = 0.0
        self._ambiguous = False
        self._quality_error_latched = False
        self._distance_field = None
        self._quality_received_at = None
        self._slam_pose = None
        self._slam_received_at = None
        self._slam_epoch = None
        self._last_slam_pose = None
        self._slam_position_jump = 0.0
        self._slam_yaw_jump = 0.0
        self._tf_received_at = None
        self._odom_position = None
        self._odom_source_stamp = None
        self._odom_received_at = None
        self._odom_reset = False
        self._last_transition = self._machine._transition()
        self._csv_file = None
        self._csv_writer = None
        self._open_diagnostics_csv()

        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/slam_localization/initialpose", 10)
        self._status_publisher = self.create_publisher(String, "~/status", 10)
        self._ready_publisher = self.create_publisher(Bool, "~/ready", 10)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_subscription = self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self._scan_subscription = self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self._odom_subscription = self.create_subscription(Odometry, "/Odometry", self._on_odom, qos_profile_sensor_data)
        self._initialpose_subscription = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, reliable_qos)
        self._slam_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, "/slam_localization/pose", self._on_slam_pose, reliable_qos)
        self._tf_subscription = self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)
        self._status_timer = self.create_timer(0.5, self._on_status_timer)
        self._ready_timer = self.create_timer(0.1, self._on_ready_heartbeat)

    def _on_map(self, message: OccupancyGrid) -> None:
        try:
            origin = message.info.origin
            grid = grid_map_from_values(
                message.info.width, message.info.height, message.info.resolution,
                origin.position.x, origin.position.y,
                quaternion_to_yaw(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w),
                message.data,
            )
            if grid != self._grid:
                self._distance_field = None
                self._quality_received_at = None
                self._completed_search_generation = None
            self._grid = grid
            self._map_received_at = self._now()
            self._try_start_search()
        except (TypeError, ValueError):
            self._grid = None
            self._map_received_at = None
            self._distance_field = None
            self._quality_received_at = None

    def _on_scan(self, message: LaserScan) -> None:
        try:
            self._points = laser_ranges_to_points(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max, self._window.max_scan_points,
            )
            self._scan_received_at = self._now()
            self._scan_sequence += 1
            self._refresh_continuous_quality(self._scan_received_at)
        except (TypeError, ValueError):
            self._points = None
            self._scan_received_at = None
            self._quality_received_at = None
        self._try_start_search()

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._begin_initial_pose_generation()
        if message.header.frame_id != "map":
            self._last_transition = self._machine.reject_initial_pose(self._now())
            return
        try:
            pose = message.pose.pose
            initial = Pose2D(pose.position.x, pose.position.y, quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ))
        except (TypeError, ValueError):
            return
        if self._grid is not None and self._grid.world_to_cell(initial.x, initial.y) is None:
            self._last_transition = self._machine.reject_initial_pose(self._now())
            return
        self._initial_pose = (initial, tuple(message.pose.covariance))
        self._slam_pose = None
        self._slam_received_at = None
        self._slam_epoch = None
        self._last_slam_pose = None
        self._slam_position_jump = 0.0
        self._slam_yaw_jump = 0.0
        self._search_pose_available = False
        self._refined_pose = None
        self._overlap = 0.0
        self._ambiguity_margin = 0.0
        self._ambiguous = False
        self._quality_error_latched = False
        self._quality_received_at = None
        self._odom_reset = False
        self._last_transition = self._machine.receive_initial_pose(self._now())

    def _begin_initial_pose_generation(self) -> None:
        """Invalidate queued work before accepting or rejecting a user click."""
        with self._search_lock:
            self._generation += 1
            self._required_scan_sequence = self._scan_sequence
            self._pending_search = None
            self._initial_pose = None
            if self._search_future is not None:
                self._search_future.cancel()

    def _on_slam_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != "map":
            return
        try:
            pose = message.pose.pose
            current = Pose2D(pose.position.x, pose.position.y, quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ))
            now = self._now()
            if self._slam_epoch is None or now < self._slam_epoch:
                return
            if self._grid is not None and self._grid.world_to_cell(current.x, current.y) is None:
                self._slam_pose = None
                self._slam_received_at = None
                self._last_slam_pose = None
                self._last_transition = self._machine.reject_initial_pose(now)
                return
            if self._last_slam_pose is not None:
                self._slam_position_jump = math.hypot(current.x - self._last_slam_pose.x, current.y - self._last_slam_pose.y)
                self._slam_yaw_jump = abs((current.yaw - self._last_slam_pose.yaw + math.pi) % (2.0 * math.pi) - math.pi)
            self._slam_pose = current
            self._last_slam_pose = current
            self._slam_received_at = now
            self._refresh_continuous_quality(now)
        except (TypeError, ValueError):
            self._slam_pose = None
            self._slam_received_at = None
            self._quality_received_at = None

    def _on_odom(self, message: Odometry) -> None:
        now = self._now()
        position = message.pose.pose.position
        current = (float(position.x), float(position.y))
        if not all(math.isfinite(value) for value in current):
            self._odom_received_at = None
            return
        source_stamp = self._source_stamp(message, now)
        if self._odom_position is not None and self._odom_source_stamp is not None:
            elapsed = source_stamp - self._odom_source_stamp
            speed = math.hypot(current[0] - self._odom_position[0], current[1] - self._odom_position[1]) / elapsed if elapsed > 0.0 else math.inf
            if elapsed <= 0.0 or speed > 3.0:
                self._odom_reset = True
        self._odom_position = current
        self._odom_source_stamp = source_stamp
        self._odom_received_at = now

    def _on_tf(self, message: TFMessage) -> None:
        for transform in getattr(message, "transforms", ()):
            parent = getattr(getattr(transform, "header", None), "frame_id", "")
            child = getattr(transform, "child_frame_id", "")
            if parent == self._camera_init_frame and child in (self._source_base_frame, self._base_frame):
                self._tf_received_at = self._now()
                return

    def _try_start_search(self) -> None:
        if self._initial_pose is None or self._grid is None or self._points is None:
            return
        initial, covariance = self._initial_pose
        if self._grid.world_to_cell(initial.x, initial.y) is None:
            self._last_transition = self._machine.observe(
                self._observation(self._now(), pose_available=False)
            )
            return
        with self._search_lock:
            if self._required_scan_sequence is None or self._scan_sequence <= self._required_scan_sequence:
                return
            if self._completed_search_generation == self._generation or self._active_search_generation == self._generation:
                return
            snapshot = (self._generation, self._grid, self._points, initial, covariance)
            if self._search_future is not None:
                if self._search_future.cancel():
                    self._search_future = None
                    self._active_search_generation = None
                    self._active_search_grid = None
                else:
                    self._pending_search = snapshot
                    return
            self._submit_search_locked(snapshot)

    def _submit_search_locked(self, snapshot) -> None:
        """Submit with `_search_lock` held; worker never mutates node state."""
        generation, grid, points, initial, _covariance = snapshot
        self._active_search_generation = generation
        self._active_search_grid = grid
        self._search_future = self._search_executor.submit(
            _coarse_search_with_field, grid, points, initial, self._window
        )

    def _apply_search_result(self) -> None:
        with self._search_lock:
            if self._search_future is None or not self._search_future.done():
                return
            future, self._search_future = self._search_future, None
            generation, self._active_search_generation = self._active_search_generation, None
            search_grid, self._active_search_grid = self._active_search_grid, None
        try:
            outcome = future.result()
            if isinstance(outcome, tuple):
                result, field = outcome
            else:  # Backward-compatible with test doubles and older queued work.
                result, field = outcome, None
        except Exception:
            result, field = None, None
        restart_on_latest_grid = False
        with self._search_lock:
            result_is_current = search_grid is not None and search_grid == self._grid
            if result is not None and result_is_current and generation == self._generation and self._initial_pose is not None and self._grid is not None and self._points is not None:
                self._completed_search_generation = generation
                self._distance_field = field
                _initial, covariance = self._initial_pose
                self._overlap = result.best.overlap
                self._search_pose_available = True
                self._ambiguous = result.ambiguous
                self._ambiguity_margin = (
                    result.best.score - result.runner_up.score if result.runner_up is not None else 1.0
                )
                if result.best.overlap < self._machine.policy.min_overlap or result.ambiguous:
                    self._quality_error_latched = True
                elif self._grid.world_to_cell(result.best.pose.x, result.best.pose.y) is None:
                    self._refined_pose = None
                    self._last_transition = self._machine.reject_initial_pose(self._now())
                else:
                    self._refined_pose = result.best.pose
                    self._publish_refined_pose(result.best.pose, covariance)
            elif generation == self._generation and not result_is_current:
                restart_on_latest_grid = True
            pending, self._pending_search = self._pending_search, None
            if pending is not None and pending[0] == self._generation and self._search_future is None:
                self._submit_search_locked(pending)
                restart_on_latest_grid = False
        if restart_on_latest_grid:
            self._try_start_search()

    def _on_status_timer(self) -> None:
        self._evaluate_state(self._now())
        self._publish_status()

    def _on_ready_heartbeat(self) -> None:
        self._evaluate_state(self._now())
        self._publish_ready()

    def _evaluate_state(self, now: float) -> None:
        self._apply_search_result()
        self._last_transition = self._machine.observe(
            self._observation(
                now,
                pose_available=True if self._quality_error_latched else None,
            )
        )
        if self._last_transition.republish_initial_pose and self._refined_pose is not None and self._initial_pose is not None:
            self._publish_refined_pose(self._refined_pose, self._initial_pose[1])

    def _observation(self, now: float, *, pose_available: bool | None = None) -> QualityObservation:
        if pose_available is None:
            pose_available = self._slam_pose is not None and self._slam_pose_fresh(now)
        inputs_fresh = (
            True
            if self._quality_error_latched
            else self._inputs_fresh(now) and self._tf_fresh(now)
        )
        return QualityObservation(
            now=now,
            inputs_fresh=inputs_fresh,
            pose_available=pose_available,
            overlap=self._finite_or_zero(self._overlap),
            ambiguity_margin=(0.0 if self._ambiguous else self._finite_or_zero(self._ambiguity_margin)),
            position_jump=self._slam_position_jump,
            yaw_jump=self._slam_yaw_jump,
            odom_reset=self._odom_reset,
            tf_conflict=self._has_amcl(),
        )

    def _publish_refined_pose(self, pose: Pose2D, covariance) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        message.pose.covariance = [self._finite_or_zero(value) for value in covariance[:36]]
        if len(message.pose.covariance) < 36:
            message.pose.covariance.extend([0.0] * (36 - len(message.pose.covariance)))
        self._initialpose_publisher.publish(message)
        self._slam_epoch = self._now()
        self._slam_pose = None
        self._slam_received_at = None
        self._last_slam_pose = None
        self._slam_position_jump = 0.0
        self._slam_yaw_jump = 0.0
        self._quality_received_at = None
        self._overlap = 0.0

    def _publish_status(self) -> None:
        transition = self._last_transition
        status = {
            "state": transition.state.value,
            "error": transition.error.value,
            "message_ko": _MESSAGES_KO[transition.error],
            "attempt": self._machine.attempts,
            "overlap": self._finite_or_zero(self._overlap),
            "ambiguity_margin": self._finite_or_zero(self._ambiguity_margin),
            "stamp": self._now(),
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        self._status_publisher.publish(message)
        self._publish_ready()
        if self._csv_writer is not None:
            self._csv_writer.writerow(status)
            self._csv_file.flush()

    def _publish_ready(self) -> None:
        ready = Bool()
        ready.data = self._machine.state is LocalizationState.READY
        self._ready_publisher.publish(ready)

    def _inputs_fresh(self, now: float | None = None) -> bool:
        now = self._now() if now is None else now
        # A map-server snapshot is durable state, unlike a scan.  It must be
        # structurally valid, but clicking initial pose must not require it to
        # be republished.  The scan is live data and therefore age-limited.
        return (
            self._grid is not None
            and self._scan_received_at is not None
            and self._odom_received_at is not None
            and self._quality_received_at is not None
            and 0.0 <= now - self._scan_received_at <= self._input_max_age
            and 0.0 <= now - self._odom_received_at <= self._input_max_age
            and 0.0 <= now - self._quality_received_at <= self._input_max_age
        )

    def _refresh_continuous_quality(self, now: float) -> None:
        """Score the latest scan at the current SLAM pose after this pose epoch."""
        if (
            self._grid is None
            or self._distance_field is None
            or self._points is None
            or self._slam_pose is None
            or self._slam_epoch is None
            or self._scan_received_at is None
            or self._slam_received_at is None
            or self._scan_received_at < self._slam_epoch
            or self._slam_received_at < self._slam_epoch
        ):
            self._quality_received_at = None
            return
        try:
            quality = score_pose(
                self._grid,
                self._distance_field,
                self._points,
                self._slam_pose,
                self._window.hit_distance,
            )
        except (TypeError, ValueError):
            self._quality_received_at = None
            return
        self._overlap = quality.overlap
        self._quality_received_at = now

    def _slam_pose_fresh(self, now: float) -> bool:
        return self._slam_epoch is not None and self._slam_received_at is not None and self._slam_received_at >= self._slam_epoch and 0.0 <= now - self._slam_received_at <= self._input_max_age

    def _tf_fresh(self, now: float) -> bool:
        return self._tf_received_at is not None and 0.0 <= now - self._tf_received_at <= self._input_max_age

    @staticmethod
    def _source_stamp(message, fallback: float) -> float:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        seconds = getattr(stamp, "sec", 0) if stamp is not None else 0
        nanoseconds = getattr(stamp, "nanosec", 0) if stamp is not None else 0
        try:
            seconds = float(seconds)
            nanoseconds = float(nanoseconds)
        except (TypeError, ValueError):
            return fallback
        if not all(math.isfinite(value) for value in (seconds, nanoseconds)) or seconds < 0.0 or not 0.0 <= nanoseconds < 1_000_000_000.0:
            return fallback
        value = seconds + nanoseconds / 1_000_000_000.0
        return value if math.isfinite(value) and value > 0.0 else fallback

    def _has_amcl(self) -> bool:
        return any(str(name).strip("/").split("/")[-1] == "amcl" for name in self.get_node_names())

    def _open_diagnostics_csv(self) -> None:
        if not self._diagnostics_csv:
            return
        path = Path(self._diagnostics_csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = path.open("a", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=("state", "error", "message_ko", "attempt", "overlap", "ambiguity_margin", "stamp"))
        if self._csv_file.tell() == 0:
            self._csv_writer.writeheader()
            self._csv_file.flush()

    def destroy_node(self):
        self._search_executor.shutdown(wait=False, cancel_futures=True)
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
        return super().destroy_node()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _finite_or_zero(value) -> float:
        return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LocalizationSupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
