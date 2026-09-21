"""ROS orchestration for safe pose-graph localization initialization."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
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
from omx_navigation.runtime_shutdown import (
    NORMAL_SHUTDOWN_EXCEPTIONS,
    shutdown_context,
)
from omx_navigation.drift_monitor import (
    DriftAction,
    DriftCheck,
    DriftDecision,
    DriftMonitor,
    DriftPolicy,
)
from omx_navigation.pose_tracking import compose_pose, pose_difference
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
    refine_pose_locally,
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
    # Also raised with no AMCL running: any map->camera_init discontinuity that
    # no pose we pushed explains. The old text named AMCL and sent operators
    # looking for a second publisher that did not exist.
    ErrorCode.TF_CONFLICT: "map→camera_init 변환이 설명되지 않는 불연속을 보였습니다(다른 TF 발행자 또는 SLAM 위치 급변).",
    ErrorCode.EXTRINSIC_UNCALIBRATED: "외부 파라미터 보정이 필요합니다.",
    ErrorCode.POSE_DRIFT: "추적 자세가 지도와 어긋났고 자동 보정으로 해결되지 않았습니다. 초기 자세를 다시 지정하세요.",
}

# A map->camera_init jump that puts the base within this of a pose we just
# pushed is our own correction arriving, not a conflict. Measured 2026-09-18:
# slam_toolbox settled 0.34 m and 3.4 deg away from the pushed pose, because the
# coarse search that produces it works on a 0.5 m grid.
_EXPECTED_CORRECTION_TRANSLATION = 0.50
_EXPECTED_CORRECTION_YAW = math.radians(10.0)
# The corrected transform reproduces the pose slam_toolbox reports on
# /slam_localization/pose exactly (measured 0.000 m / 0.00 deg), so agreement
# between the two confirms the correction has landed.
_SLAM_AGREEMENT_TRANSLATION = 0.05
_SLAM_AGREEMENT_YAW = math.radians(1.0)
_STATUS_FIELDS = (
    "state", "error", "message_ko", "attempt", "overlap", "ambiguity_margin", "stamp",
    "consistency_gap", "drift_offset_x", "drift_offset_y", "drift_offset_yaw_deg",
    "drift_corrections", "tf_corrections_explained",
)


@dataclass(frozen=True)
class _CorrectionExpectation:
    """A pose we pushed to slam_toolbox whose arrival must not look like a conflict."""

    pose: Pose2D
    deadline: float
    # Initialization pushes are confirmed by slam_toolbox's pose handshake and
    # then end. A drift correction arrives while slam_toolbox is still
    # publishing poses from before it, so it simply expires.
    confirm_with_slam: bool


def _coarse_search_with_field(grid, points, initial, window):
    """Build the immutable map field once and return it with the initial search."""
    field = build_distance_field(grid)
    return coarse_search(grid, points, initial, window, field), field


class LocalizationSupervisor(Node):
    """Connect pure scan-map quality checks to the ROS localization lifecycle."""

    def __init__(self) -> None:
        super().__init__("localization_supervisor")
        self._input_max_age = float(self.declare_parameter("input_max_age", 0.50).value)
        self._source_clock_skew = float(self.declare_parameter("source_clock_skew", 0.05).value)
        self._slam_tf_future_offset = float(self.declare_parameter("slam_tf_future_offset", 0.50).value)
        self._diagnostics_csv = str(self.declare_parameter("diagnostics_csv", "").value)
        self._window = SearchWindow(
            translation_radius=float(self.declare_parameter("coarse_search_translation_radius", 3.0).value),
            translation_step=float(self.declare_parameter("coarse_search_translation_step", 0.5).value),
            yaw_radius=float(self.declare_parameter("coarse_search_yaw_radius", math.pi / 2).value),
            yaw_step=float(self.declare_parameter("coarse_search_yaw_step", math.radians(15.0)).value),
        )
        self._machine = LocalizationStateMachine(LocalizationPolicy())
        self._drift_check_period = float(self.declare_parameter("drift_check_period", 2.0).value)
        self._correction_expect_window = float(self.declare_parameter("correction_expect_window", 5.0).value)
        for name, value in (
            ("drift_check_period", self._drift_check_period),
            ("correction_expect_window", self._correction_expect_window),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self._drift_monitor = DriftMonitor(DriftPolicy(
            gap_threshold=float(self.declare_parameter("drift_gap_threshold", 0.08).value),
            auto_correct=bool(self.declare_parameter("drift_auto_correct", True).value),
        ))
        self._camera_init_frame = str(self.declare_parameter("camera_init_frame", "camera_init").value)
        self._source_base_frame = str(self.declare_parameter("source_base_frame", "body").value)
        self._base_frame = str(self.declare_parameter("base_frame", "body_nav").value)
        self._grid = None
        self._points = None
        self._map_received_at = None
        self._scan_received_at = None
        self._scan_source_stamp = None
        self._initial_pose = None
        self._initial_pose_epoch = None
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
        self._quality_source_stamp = None
        self._slam_pose = None
        self._slam_received_at = None
        self._slam_source_stamp = None
        self._slam_pose_handshake = False
        self._slam_epoch = None
        self._map_camera_pose = None
        self._camera_base_pose = None
        self._current_base_pose = None
        self._map_camera_received_at = None
        self._camera_base_received_at = None
        self._map_camera_source_stamp = None
        self._camera_base_source_stamp = None
        self._last_map_camera_pose = None
        self._tf_position_jump = 0.0
        self._tf_yaw_jump = 0.0
        self._tf_conflict = False
        self._odom_position = None
        self._odom_source_stamp = None
        self._odom_received_at = None
        self._odom_reset = False
        self._expectation = None
        self._tf_corrections_explained = 0
        self._drift_future = None
        self._drift_job = None
        self._last_drift_check_at = None
        self._last_drift_decision: DriftDecision | None = None
        self._last_transition = self._machine._transition()
        self._csv_file = None
        self._csv_writer = None
        self._open_diagnostics_csv()

        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/slam_localization/initialpose", 10)
        self._status_publisher = self.create_publisher(String, "~/status", 10)
        self._ready_publisher = self.create_publisher(Bool, "~/ready", 10)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest_sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._map_subscription = self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self._scan_subscription = self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self._odom_subscription = self.create_subscription(Odometry, "/Odometry", self._on_odom, latest_sensor_qos)
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
        now = self._now()
        source_stamp = self._source_stamp(message, None)
        try:
            if source_stamp is None or not self._source_event_is_current(source_stamp, now):
                raise ValueError("scan source stamp is invalid or stale")
            self._points = laser_ranges_to_points(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max, self._window.max_scan_points,
            )
            self._scan_received_at = now
            self._scan_source_stamp = source_stamp
            self._scan_sequence += 1
            self._refresh_continuous_quality(now)
        except (TypeError, ValueError):
            self._points = None
            self._scan_received_at = None
            self._scan_source_stamp = None
            self._quality_received_at = None
            self._quality_source_stamp = None
        self._try_start_search()

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        now = self._now()
        self._begin_initial_pose_generation()
        if getattr(getattr(message, "header", None), "frame_id", None) != "map":
            self._last_transition = self._machine.reject_initial_pose(now)
            return
        try:
            pose = message.pose.pose
            values = (
                float(pose.position.x), float(pose.position.y), float(pose.position.z),
                float(pose.orientation.x), float(pose.orientation.y),
                float(pose.orientation.z), float(pose.orientation.w),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("initial pose must be finite")
            quaternion = values[3:]
            quaternion_norm = math.hypot(*quaternion)
            if not math.isfinite(quaternion_norm) or quaternion_norm <= 1e-12:
                raise ValueError("initial pose quaternion must not be near zero")
            initial = Pose2D(values[0], values[1], quaternion_to_yaw(*quaternion))
            raw_covariance = message.pose.covariance
            if isinstance(raw_covariance, (str, bytes)):
                raise ValueError("initial pose covariance must be numeric")
            covariance = tuple(raw_covariance)
            if len(covariance) != 36:
                raise ValueError("initial pose covariance must contain 36 values")
            if any(isinstance(value, bool) for value in covariance):
                raise ValueError("initial pose covariance must be numeric")
            covariance = tuple(float(value) for value in covariance)
            if not all(math.isfinite(value) for value in covariance):
                raise ValueError("initial pose covariance must be finite")
        except (AttributeError, TypeError, ValueError):
            self._last_transition = self._machine.reject_initial_pose(now)
            return
        if self._grid is not None and self._grid.world_to_cell(initial.x, initial.y) is None:
            self._last_transition = self._machine.reject_initial_pose(now)
            return
        self._initial_pose = (initial, covariance)
        self._initial_pose_epoch = now
        self._last_transition = self._machine.receive_initial_pose(now)

    def _begin_initial_pose_generation(self) -> None:
        """Invalidate queued work before accepting or rejecting a user click."""
        with self._search_lock:
            self._generation += 1
            self._required_scan_sequence = self._scan_sequence
            self._pending_search = None
            self._initial_pose = None
            self._initial_pose_epoch = None
            if self._search_future is not None:
                self._search_future.cancel()
            if self._drift_future is not None and self._drift_future.cancel():
                self._drift_future = None
                self._drift_job = None
        self._slam_pose = None
        self._slam_received_at = None
        self._slam_source_stamp = None
        self._slam_pose_handshake = False
        self._slam_epoch = None
        self._reset_tf_tracking()
        self._search_pose_available = False
        self._refined_pose = None
        self._overlap = 0.0
        self._ambiguity_margin = 0.0
        self._ambiguous = False
        self._quality_error_latched = False
        self._quality_received_at = None
        self._quality_source_stamp = None
        self._odom_reset = False
        self._expectation = None
        self._reset_drift_tracking()

    def _on_slam_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != "map":
            return
        try:
            now = self._now()
            source_stamp = self._source_stamp(message, None)
            if (
                source_stamp is None
                or self._slam_epoch is None
                or not self._source_is_fresh(source_stamp, now)
            ):
                raise ValueError("SLAM pose source stamp is invalid or stale")
            pose = message.pose.pose
            current = Pose2D(pose.position.x, pose.position.y, quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ))
            if source_stamp < self._slam_epoch and not self._answers_pending_push(current):
                raise ValueError("SLAM pose predates the push and is not its answer")
            if self._grid is not None and self._grid.world_to_cell(current.x, current.y) is None:
                self._slam_pose = None
                self._slam_received_at = None
                self._slam_source_stamp = None
                self._slam_pose_handshake = False
                self._last_transition = self._machine.reject_initial_pose(now)
                return
            self._slam_pose = current
            self._slam_received_at = now
            self._slam_source_stamp = source_stamp
            self._slam_pose_handshake = True
            self._confirm_expected_correction()
            self._refresh_continuous_quality(now)
        except (TypeError, ValueError):
            self._slam_pose = None
            self._slam_received_at = None
            self._slam_source_stamp = None
            self._slam_pose_handshake = False
            self._quality_received_at = None

    def _on_odom(self, message: Odometry) -> None:
        now = self._now()
        source_stamp = self._source_stamp(message, None)
        try:
            position = message.pose.pose.position
            current = (float(position.x), float(position.y))
            if (
                not all(math.isfinite(value) for value in current)
                or source_stamp is None
                or not self._source_event_is_current(source_stamp, now, self._slam_epoch)
            ):
                raise ValueError("odometry source data is invalid or stale")
        except (AttributeError, TypeError, ValueError):
            self._odom_received_at = None
            self._odom_source_stamp = None
            return
        if self._odom_position is not None and self._odom_source_stamp is not None:
            elapsed = source_stamp - self._odom_source_stamp
            speed = math.hypot(current[0] - self._odom_position[0], current[1] - self._odom_position[1]) / elapsed if elapsed > 0.0 else math.inf
            if elapsed <= 0.0 or speed > 3.0:
                self._odom_reset = True
        self._odom_position = current
        self._odom_source_stamp = source_stamp
        self._odom_received_at = now

    def _on_tf(self, message: TFMessage) -> None:
        now = self._now()
        for transform in getattr(message, "transforms", ()):
            parent = getattr(getattr(transform, "header", None), "frame_id", "")
            child = getattr(transform, "child_frame_id", "")
            if parent == "map" and child == self._camera_init_frame:
                try:
                    raw_source_stamp = self._source_stamp(transform, None)
                    if raw_source_stamp is None:
                        raise ValueError("map transform source stamp is invalid")
                    source_stamp = raw_source_stamp - self._slam_tf_future_offset
                    if not self._source_event_is_current(source_stamp, now, self._slam_epoch):
                        raise ValueError("map transform source stamp is stale")
                    current = self._planar_tf(transform)
                except (AttributeError, TypeError, ValueError):
                    self._map_camera_pose = None
                    self._map_camera_received_at = None
                    self._map_camera_source_stamp = None
                    self._current_base_pose = None
                    continue
                self._track_map_camera_jump(current, now)
                self._map_camera_pose = current
                self._map_camera_received_at = now
                self._map_camera_source_stamp = source_stamp
            elif parent == self._camera_init_frame and child == self._base_frame:
                try:
                    source_stamp = self._source_stamp(transform, None)
                    if source_stamp is None or not self._source_event_is_current(source_stamp, now, self._slam_epoch):
                        raise ValueError("base transform source stamp is invalid or stale")
                    self._camera_base_pose = self._planar_tf(transform)
                except (AttributeError, TypeError, ValueError):
                    self._camera_base_pose = None
                    self._camera_base_received_at = None
                    self._camera_base_source_stamp = None
                    self._current_base_pose = None
                    continue
                self._camera_base_received_at = now
                self._camera_base_source_stamp = source_stamp
        self._compose_current_base_pose()
        self._confirm_expected_correction()
        self._refresh_continuous_quality(now)

    def _track_map_camera_jump(self, current: Pose2D, now: float) -> None:
        """Flag a map->camera_init discontinuity unless one of our pushes explains it."""
        # Before the first push slam_toolbox is still settling from
        # map_start_pose; its first scan match jumps 20 degrees or more. On
        # 2026-09-18 that latched TF_CONFLICT at startup in three sessions
        # before anyone had set a pose, and nothing consumes the transform yet.
        if self._slam_epoch is None:
            self._last_map_camera_pose = None
            return
        if self._last_map_camera_pose is not None:
            position_jump, yaw_jump = pose_difference(current, self._last_map_camera_pose)
            self._tf_position_jump = position_jump
            self._tf_yaw_jump = yaw_jump
            policy = self._machine.policy
            if position_jump > policy.max_position_jump or yaw_jump > policy.max_yaw_jump:
                if self._jump_lands_on_expected_pose(current, now):
                    # Our own correction arriving. Transforms from before it can
                    # still follow the reset, so a baseline taken from one of
                    # them made every successful lock look like a conflict.
                    self._tf_corrections_explained += 1
                    self._tf_position_jump = 0.0
                    self._tf_yaw_jump = 0.0
                else:
                    self._tf_conflict = True
        self._last_map_camera_pose = current

    def _jump_lands_on_expected_pose(self, map_camera: Pose2D, now: float) -> bool:
        expectation = self._expectation
        if expectation is None or now > expectation.deadline or self._camera_base_pose is None:
            return False
        try:
            landed = compose_pose(map_camera, self._camera_base_pose)
        except ValueError:
            return False
        translation, yaw = pose_difference(landed, expectation.pose)
        return translation <= _EXPECTED_CORRECTION_TRANSLATION and yaw <= _EXPECTED_CORRECTION_YAW

    def _answers_pending_push(self, slam_pose: Pose2D) -> bool:
        """Whether a SLAM pose stamped before our push is slam_toolbox's answer to it.

        slam_toolbox applies a pushed pose to the next scan it *receives*, and
        that scan was usually acquired before the push: LiDAR to /scan to
        slam_toolbox's TF filter takes 50-100 ms. At rest it is also the only
        answer, since minimum_travel stops further matching. Requiring the
        answer's stamp to follow the push turned every lock into a coin toss:
        on 2026-09-18 the second click only locked on its retry push, and
        replaying that session with the robot parked lost all five pushes.
        A pre-push pose is taken as the answer only while an initialization
        push is pending and only if it lands on the pose we pushed.
        """
        expectation = self._expectation
        if expectation is None or not expectation.confirm_with_slam:
            return False
        translation, yaw = pose_difference(slam_pose, expectation.pose)
        return translation <= _EXPECTED_CORRECTION_TRANSLATION and yaw <= _EXPECTED_CORRECTION_YAW

    def _confirm_expected_correction(self) -> None:
        """End an initialization expectation once slam_toolbox has visibly applied it.

        After this, a discontinuity can no longer be our correction arriving,
        which keeps a jump shortly after lock failing closed.
        """
        expectation = self._expectation
        if (
            expectation is None
            or not expectation.confirm_with_slam
            or not self._slam_pose_handshake
            or self._slam_pose is None
            or self._current_base_pose is None
            # A pose from a scan acquired before the push may still be one
            # matched before slam_toolbox saw it. It can complete the
            # handshake but not prove the correction was applied: confirming
            # on it would turn the real correction, arriving next, into a
            # TF_CONFLICT. The expectation then simply runs out.
            or self._slam_source_stamp is None
            or self._slam_epoch is None
            or self._slam_source_stamp < self._slam_epoch
        ):
            return
        base_translation, base_yaw = pose_difference(self._current_base_pose, self._slam_pose)
        push_translation, push_yaw = pose_difference(self._slam_pose, expectation.pose)
        if (
            base_translation <= _SLAM_AGREEMENT_TRANSLATION
            and base_yaw <= _SLAM_AGREEMENT_YAW
            and push_translation <= _EXPECTED_CORRECTION_TRANSLATION
            and push_yaw <= _EXPECTED_CORRECTION_YAW
        ):
            self._expectation = None

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
        if self._expectation is not None and now > self._expectation.deadline:
            self._expectation = None
        self._apply_drift_result()
        self._last_transition = self._machine.observe(
            self._observation(
                now,
                pose_available=True if self._quality_error_latched else None,
            )
        )
        if self._last_transition.republish_initial_pose and self._refined_pose is not None and self._initial_pose is not None:
            self._publish_refined_pose(self._refined_pose, self._initial_pose[1])
        elif self._last_transition.republish_initial_pose and self._initial_pose is not None:
            self._prepare_coarse_search_retry()
        self._maybe_start_drift_check(now)

    def _maybe_start_drift_check(self, now: float) -> None:
        """Refine around the tracked pose in the background while it is in use."""
        if (
            self._machine.state not in (LocalizationState.READY, LocalizationState.DEGRADED)
            or self._expectation is not None
            or self._quality_error_latched
            or self._grid is None
            or self._distance_field is None
            or self._points is None
            or self._current_base_pose is None
            or self._camera_base_pose is None
            or self._quality_received_at is None
            or (
                self._last_drift_check_at is not None
                and now - self._last_drift_check_at < self._drift_check_period
            )
            # A correction computed from a stale scan and pose would be pushed
            # onto a robot that has since moved.
            or not self._inputs_fresh(now)
            or not self._tf_fresh(now)
        ):
            return
        with self._search_lock:
            if self._drift_future is not None or self._search_future is not None:
                return
            self._drift_job = (self._generation, now, self._grid, self._current_base_pose, self._camera_base_pose)
            self._drift_future = self._search_executor.submit(
                refine_pose_locally, self._grid, self._distance_field, self._points, self._current_base_pose
            )
        self._last_drift_check_at = now

    def _apply_drift_result(self) -> None:
        with self._search_lock:
            if self._drift_future is None or not self._drift_future.done():
                return
            future, self._drift_future = self._drift_future, None
            job, self._drift_job = self._drift_job, None
        try:
            refinement = future.result()
        except Exception:
            return
        generation, stamp, grid, tracked, camera_base = job
        # A click, a new lock or a correction since the job started makes it
        # describe a pose that is no longer the one being tracked.
        if generation != self._generation or grid is not self._grid or self._expectation is not None:
            return
        try:
            decision = self._drift_monitor.observe(DriftCheck(
                stamp, refinement.gap, tracked, refinement.best.pose, refinement.best.overlap, camera_base,
            ))
        except ValueError:
            return
        self._last_drift_decision = decision
        if decision.action is DriftAction.CORRECT and decision.correction is not None:
            self._publish_drift_correction(decision.correction, decision)
        elif decision.action is DriftAction.ESCALATE:
            self.get_logger().error(
                f"Tracked pose has drifted from the map ({decision.reason}); "
                f"score gap {decision.gap:.3f}, offset ({decision.offset[0]:+.2f} m, "
                f"{decision.offset[1]:+.2f} m, {math.degrees(decision.offset[2]):+.1f} deg)"
            )

    def _publish_drift_correction(self, pose: Pose2D, decision: DriftDecision) -> None:
        covariance = self._initial_pose[1] if self._initial_pose is not None else (0.0,) * 36
        self._initialpose_publisher.publish(self._pose_message(pose, covariance))
        self._expectation = _CorrectionExpectation(
            pose, self._now() + self._correction_expect_window, confirm_with_slam=False
        )
        self.get_logger().warning(
            f"Tracked pose drifted (score gap {decision.gap:.3f}); pushed correction to "
            f"x={pose.x:.2f}, y={pose.y:.2f}, yaw={math.degrees(pose.yaw):.1f} deg "
            f"(correction {self._drift_monitor.corrections})"
        )

    def _reset_drift_tracking(self) -> None:
        """Forget drift evidence.

        Never takes `_search_lock`: `_publish_refined_pose` runs with it held.
        An in-flight job needs no cancelling here, because `_apply_drift_result`
        drops any result whose generation, grid or expectation no longer match.
        """
        self._drift_monitor.reset()
        self._last_drift_decision = None
        self._last_drift_check_at = None

    def _prepare_coarse_search_retry(self) -> None:
        """Require a newer scan before retrying a failed coarse alignment."""
        with self._search_lock:
            self._completed_search_generation = None
            self._required_scan_sequence = self._scan_sequence
            self._pending_search = None
        self._search_pose_available = False
        self._quality_error_latched = False
        self._overlap = 0.0
        self._ambiguity_margin = 0.0
        self._ambiguous = False
        self._quality_received_at = None
        self._quality_source_stamp = None

    def _observation(self, now: float, *, pose_available: bool | None = None) -> QualityObservation:
        if pose_available is None:
            pose_available = self._slam_pose_fresh(now)
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
            position_jump=self._tf_position_jump,
            yaw_jump=self._tf_yaw_jump,
            odom_reset=self._odom_reset,
            tf_conflict=self._has_amcl() or self._tf_conflict,
            pose_drift=self._drift_monitor.escalated,
        )

    def _pose_message(self, pose: Pose2D, covariance) -> PoseWithCovarianceStamped:
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
        return message

    def _publish_refined_pose(self, pose: Pose2D, covariance) -> None:
        self._initialpose_publisher.publish(self._pose_message(pose, covariance))
        self._slam_epoch = self._now()
        self._slam_pose = None
        self._slam_received_at = None
        self._slam_source_stamp = None
        self._slam_pose_handshake = False
        self._reset_tf_tracking()
        self._quality_received_at = None
        self._quality_source_stamp = None
        self._overlap = 0.0
        self._expectation = _CorrectionExpectation(
            pose, self._slam_epoch + self._correction_expect_window, confirm_with_slam=True
        )
        self._reset_drift_tracking()

    def _publish_status(self) -> None:
        transition = self._last_transition
        decision = self._last_drift_decision
        offset = decision.offset if decision is not None else (0.0, 0.0, 0.0)
        status = {
            "state": transition.state.value,
            "error": transition.error.value,
            "message_ko": _MESSAGES_KO[transition.error],
            "attempt": self._machine.attempts,
            "overlap": self._finite_or_zero(self._overlap),
            # Best-vs-runner-up margin of the search that produced the current
            # lock. It is not recomputed while tracking (see drift_monitor):
            # tracking quality is `consistency_gap`, which is.
            "ambiguity_margin": self._finite_or_zero(self._ambiguity_margin),
            "stamp": self._now(),
            "consistency_gap": self._finite_or_zero(decision.gap if decision is not None else 0.0),
            "drift_offset_x": self._finite_or_zero(offset[0]),
            "drift_offset_y": self._finite_or_zero(offset[1]),
            "drift_offset_yaw_deg": self._finite_or_zero(math.degrees(offset[2])),
            "drift_corrections": self._drift_monitor.corrections,
            "tf_corrections_explained": self._tf_corrections_explained,
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
            and self._scan_source_stamp is not None
            and self._odom_received_at is not None
            and self._odom_source_stamp is not None
            and self._quality_received_at is not None
            and self._quality_source_stamp is not None
            and 0.0 <= now - self._scan_received_at <= self._input_max_age
            and 0.0 <= now - self._odom_received_at <= self._input_max_age
            and 0.0 <= now - self._quality_received_at <= self._input_max_age
            and self._source_is_fresh(self._scan_source_stamp, now)
            and self._source_is_fresh(self._odom_source_stamp, now)
            and self._source_is_fresh(self._quality_source_stamp, now)
        )

    def _refresh_continuous_quality(self, now: float) -> None:
        """Score the latest scan at the current composed base pose."""
        if (
            self._grid is None
            or self._distance_field is None
            or self._points is None
            or self._current_base_pose is None
            or self._slam_epoch is None
            or self._scan_received_at is None
            or self._scan_source_stamp is None
            or self._scan_source_stamp < self._slam_epoch
            or not self._slam_pose_fresh(now)
            or not self._tf_fresh(now)
        ):
            self._quality_received_at = None
            self._quality_source_stamp = None
            return
        try:
            quality = score_pose(
                self._grid,
                self._distance_field,
                self._points,
                self._current_base_pose,
                self._window.hit_distance,
                # Watching a verified pose, not choosing one: a lone beam past
                # the map edge is one unmatched beam, not a lost fix.
                disqualify_outside=False,
            )
        except (TypeError, ValueError):
            self._quality_received_at = None
            self._quality_source_stamp = None
            return
        self._overlap = quality.overlap
        self._quality_received_at = now
        self._quality_source_stamp = min(
            self._scan_source_stamp,
            self._map_camera_source_stamp,
            self._camera_base_source_stamp,
        )

    def _slam_pose_fresh(self, now: float) -> bool:
        return (
            self._slam_epoch is not None
            and self._slam_pose_handshake
            and self._current_base_pose is not None
        )

    def _tf_fresh(self, now: float) -> bool:
        return (
            self._slam_epoch is not None
            and self._map_camera_received_at is not None
            and self._camera_base_received_at is not None
            and self._map_camera_source_stamp is not None
            and self._camera_base_source_stamp is not None
            and self._map_camera_received_at >= self._slam_epoch
            and self._camera_base_received_at >= self._slam_epoch
            and self._map_camera_source_stamp >= self._slam_epoch
            and self._camera_base_source_stamp >= self._slam_epoch
            and 0.0 <= now - self._map_camera_received_at <= self._input_max_age
            and 0.0 <= now - self._camera_base_received_at <= self._input_max_age
            and self._source_is_fresh(self._map_camera_source_stamp, now)
            and self._source_is_fresh(self._camera_base_source_stamp, now)
        )

    @staticmethod
    def _planar_tf(transform) -> Pose2D:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            float(translation.x), float(translation.y), float(translation.z),
            float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("transform must be finite")
        quaternion_norm = math.hypot(*values[3:])
        if not math.isfinite(quaternion_norm) or quaternion_norm <= 1e-12:
            raise ValueError("transform quaternion must not be near zero")
        return Pose2D(values[0], values[1], quaternion_to_yaw(*values[3:]))

    def _compose_current_base_pose(self) -> None:
        if self._map_camera_pose is None or self._camera_base_pose is None:
            self._current_base_pose = None
            return
        try:
            self._current_base_pose = compose_pose(self._map_camera_pose, self._camera_base_pose)
        except ValueError:
            self._current_base_pose = None

    def _reset_tf_tracking(self) -> None:
        self._map_camera_pose = None
        self._camera_base_pose = None
        self._current_base_pose = None
        self._map_camera_received_at = None
        self._camera_base_received_at = None
        self._map_camera_source_stamp = None
        self._camera_base_source_stamp = None
        self._last_map_camera_pose = None
        self._tf_position_jump = 0.0
        self._tf_yaw_jump = 0.0
        self._tf_conflict = False

    @staticmethod
    def _source_stamp(message, fallback: float | None) -> float | None:
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

    def _source_event_is_current(
        self,
        source_stamp: float,
        now: float,
        epoch: float | None = None,
    ) -> bool:
        required_epoch = epoch
        if required_epoch is None:
            required_epoch = self._slam_epoch if self._slam_epoch is not None else self._initial_pose_epoch
        return (
            self._source_is_fresh(source_stamp, now)
            and (required_epoch is None or source_stamp >= required_epoch)
        )

    def _source_is_fresh(self, source_stamp: float, now: float) -> bool:
        age = now - source_stamp
        return -self._source_clock_skew <= age <= self._input_max_age

    def _has_amcl(self) -> bool:
        return any(str(name).strip("/").split("/")[-1] == "amcl" for name in self.get_node_names())

    def _open_diagnostics_csv(self) -> None:
        if not self._diagnostics_csv:
            return
        path = Path(self._diagnostics_csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = path.open("a", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=_STATUS_FIELDS)
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
    except NORMAL_SHUTDOWN_EXCEPTIONS:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        shutdown_context(rclpy)
