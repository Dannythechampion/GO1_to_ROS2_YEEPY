"""ROS orchestration for safe pose-graph localization initialization."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
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
from omx_navigation.scan_map_quality import Pose2D, SearchWindow, coarse_search


_MESSAGES_KO = {
    ErrorCode.NONE: "정상",
    ErrorCode.INPUT_MISSING: "지도 또는 스�� 입력이 오래되었거나 없습니다.",
    ErrorCode.POSE_OUTSIDE_MAP: "초기 자세가 map 좌표계 또는 지도 범위를 ��어났습니다.",
    ErrorCode.ALIGNMENT_TIMEOUT: "초기 자세 정렬 시간이 초과되었습니다.",
    ErrorCode.LOW_OVERLAP: "스��과 지도의 겹침이 부족합니다.",
    ErrorCode.AMBIGUOUS: "정렬 결과가 모호합니다.",
    ErrorCode.ODOM_RESET: "오도메트리 재설정이 감지되었습니다.",
    ErrorCode.TF_CONFLICT: "AMCL TF 발행자 충돌이 감지되었습니다.",
    ErrorCode.EXTRINSIC_UNCALIBRATED: "외부 파라미터 보정이 필요합니다.",
}


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
        self._grid = None
        self._points = None
        self._map_received_at = None
        self._scan_received_at = None
        self._initial_pose = None
        self._refined_pose = None
        self._search_pose_available = False
        self._overlap = 0.0
        self._ambiguity_margin = 0.0
        self._ambiguous = False
        self._slam_pose = None
        self._slam_received_at = None
        self._odom_position = None
        self._odom_received_at = None
        self._odom_reset = False
        self._last_transition = self._machine._transition()
        self._csv_file = None
        self._csv_writer = None
        self._open_diagnostics_csv()

        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/slam_localization/initialpose", 10)
        self._status_publisher = self.create_publisher(String, "~/status", 10)
        self._ready_publisher = self.create_publisher(Bool, "~/ready", 10)
        self._map_subscription = self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self._scan_subscription = self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self._odom_subscription = self.create_subscription(Odometry, "/Odometry", self._on_odom, 10)
        self._initialpose_subscription = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10)
        self._slam_pose_subscription = self.create_subscription(PoseStamped, "/slam_toolbox/pose", self._on_slam_pose, 10)
        self._tf_subscription = self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self._status_timer = self.create_timer(0.5, self._on_status_timer)

    def _on_map(self, message: OccupancyGrid) -> None:
        try:
            origin = message.info.origin
            self._grid = grid_map_from_values(
                message.info.width, message.info.height, message.info.resolution,
                origin.position.x, origin.position.y,
                quaternion_to_yaw(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w),
                message.data,
            )
            self._map_received_at = self._now()
        except (TypeError, ValueError):
            self._grid = None
        self._try_align()

    def _on_scan(self, message: LaserScan) -> None:
        try:
            self._points = laser_ranges_to_points(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max, self._window.max_scan_points,
            )
            self._scan_received_at = self._now()
        except (TypeError, ValueError):
            self._points = None
        self._try_align()

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
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
        self._search_pose_available = False
        self._last_transition = self._machine.receive_initial_pose(self._now())
        self._try_align()

    def _on_slam_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id != "map":
            return
        try:
            pose = message.pose
            self._slam_pose = Pose2D(pose.position.x, pose.position.y, quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ))
            self._slam_received_at = self._now()
        except (TypeError, ValueError):
            self._slam_pose = None

    def _on_odom(self, message: Odometry) -> None:
        now = self._now()
        position = message.pose.pose.position
        current = (float(position.x), float(position.y))
        if not all(math.isfinite(value) for value in current):
            return
        if self._odom_position is not None and self._odom_received_at is not None:
            elapsed = now - self._odom_received_at
            speed = math.hypot(current[0] - self._odom_position[0], current[1] - self._odom_position[1]) / elapsed if elapsed > 0.0 else math.inf
            if elapsed <= 0.0 or speed > 3.0:
                self._odom_reset = True
        self._odom_position = current
        self._odom_received_at = now

    def _on_tf(self, _message: TFMessage) -> None:
        # The subscription makes TF freshness observable in rosbag diagnostics.
        return

    def _try_align(self) -> None:
        if self._initial_pose is None or self._grid is None or self._points is None or not self._inputs_fresh():
            return
        initial, covariance = self._initial_pose
        if self._grid.world_to_cell(initial.x, initial.y) is None:
            self._last_transition = self._machine.observe(self._observation(pose_available=False))
            return
        result = coarse_search(self._grid, self._points, initial, self._window)
        self._overlap = result.best.overlap
        self._search_pose_available = True
        self._ambiguous = result.ambiguous
        self._ambiguity_margin = (
            result.best.score - result.runner_up.score if result.runner_up is not None else 1.0
        )
        if result.best.overlap < self._machine.policy.min_overlap or result.ambiguous:
            # Search completed against a valid user pose, so preserve its
            # specific quality error instead of classifying it as missing SLAM.
            self._last_transition = self._machine.observe(self._observation(pose_available=True))
            return
        self._refined_pose = result.best.pose
        self._publish_refined_pose(result.best.pose, covariance)

    def _on_status_timer(self) -> None:
        self._last_transition = self._machine.observe(self._observation())
        if self._last_transition.republish_initial_pose and self._refined_pose is not None and self._initial_pose is not None:
            self._publish_refined_pose(self._refined_pose, self._initial_pose[1])
        self._publish_status()

    def _observation(self, pose_available: bool | None = None) -> QualityObservation:
        now = self._now()
        if pose_available is None:
            pose_available = self._search_pose_available or (
                self._slam_pose is not None and self._slam_pose_fresh(now)
            )
        return QualityObservation(
            now=now,
            inputs_fresh=self._inputs_fresh(now),
            pose_available=pose_available,
            overlap=self._finite_or_zero(self._overlap),
            ambiguity_margin=(0.0 if self._ambiguous else self._finite_or_zero(self._ambiguity_margin)),
            position_jump=0.0,
            yaw_jump=0.0,
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
        ready = Bool()
        ready.data = transition.state is LocalizationState.READY
        self._ready_publisher.publish(ready)
        if self._csv_writer is not None:
            self._csv_writer.writerow(status)
            self._csv_file.flush()

    def _inputs_fresh(self, now: float | None = None) -> bool:
        now = self._now() if now is None else now
        return all(value is not None and 0.0 <= now - value <= self._input_max_age for value in (self._map_received_at, self._scan_received_at))

    def _slam_pose_fresh(self, now: float) -> bool:
        return self._slam_received_at is not None and 0.0 <= now - self._slam_received_at <= self._input_max_age

    def _has_amcl(self) -> bool:
        return any(str(name).rstrip("/") == "/amcl" or str(name) == "amcl" for name in self.get_node_names())

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
