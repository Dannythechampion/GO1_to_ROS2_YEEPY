"""AMCL initial-pose seeding and independent readiness supervision."""

from __future__ import annotations

import math
import time
from typing import Dict, Optional

from .localization_state import (
    LocalizationObservation,
    LocalizationState,
    LocalizationStatus,
    LocalizationSupervisorCore,
)
from .map_geometry import OccupancyMap, ScanScore, score_scan_pose
from .pose_config import PlanarPose, PoseConfigurationError, load_planar_pose


def initial_pose_values(pose: PlanarPose) -> Dict[str, float | str]:
    return {
        "frame_id": "map",
        "x": pose.x,
        "y": pose.y,
        "qz": math.sin(pose.yaw / 2.0),
        "qw": math.cos(pose.yaw / 2.0),
        "covariance_x": 0.04,
        "covariance_y": 0.04,
        "covariance_yaw": 0.0305,
    }


def status_line(status: LocalizationStatus) -> str:
    return (
        f"state={status.state.value} ready={'true' if status.ready else 'false'} "
        f"reason={status.reason}"
    )


def _yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def odometry_is_stationary(linear_x: float, linear_y: float, angular_z: float) -> bool:
    return math.hypot(float(linear_x), float(linear_y)) <= 0.01 and abs(
        float(angular_z)
    ) <= 0.01


try:
    import rclpy
    from builtin_interfaces.msg import Time as TimeMessage
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:
    rclpy = None
    Node = object


if rclpy is not None:

    class LocalizationSupervisor(Node):
        def __init__(self) -> None:
            super().__init__("localization_supervisor")
            self.declare_parameter("start_pose_file", "")
            self.declare_parameter("initial_pose_arm", False)
            self.declare_parameter("map_topic", "/map")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/Odometry")
            self.declare_parameter("amcl_pose_topic", "/amcl_pose")
            self.declare_parameter("odom_frame", "camera_init")
            self.declare_parameter("base_frame", "body")
            self.declare_parameter("heartbeat_rate", 10.0)

            pose_result = load_planar_pose(
                str(self.get_parameter("start_pose_file").value),
                expected_kind="start",
            )
            self._core = LocalizationSupervisorCore(
                pose_result.pose,
                initial_pose_arm=bool(self.get_parameter("initial_pose_arm").value),
            )
            self._map: Optional[OccupancyMap] = None
            self._scan: Optional[LaserScan] = None
            self._odom: Optional[Odometry] = None
            self._amcl: Optional[PoseWithCovarianceStamped] = None
            self._scan_score: Optional[ScanScore] = None
            self._consecutive_scans = 0
            self._last_tf_stamp = 0.0

            status_qos = QoSProfile(depth=1)
            status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            status_qos.reliability = ReliabilityPolicy.RELIABLE
            map_qos = QoSProfile(depth=1)
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            map_qos.reliability = ReliabilityPolicy.RELIABLE

            self._ready_pub = self.create_publisher(Bool, "/localization/ready", 10)
            self._status_pub = self.create_publisher(
                String, "/localization/status", status_qos
            )
            self._initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10
            )
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("map_topic").value),
                self._map_callback,
                map_qos,
            )
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._scan_callback,
                10,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odom_topic").value),
                self._odom_callback,
                10,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("amcl_pose_topic").value),
                self._amcl_callback,
                10,
            )
            self.create_service(Trigger, "/localization/reset", self._reset_callback)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            rate = float(self.get_parameter("heartbeat_rate").value)
            if rate <= 0.0:
                raise ValueError("heartbeat_rate must be positive")
            self._timer = self.create_timer(1.0 / rate, self._timer_callback)
            self._publish_status()

        @staticmethod
        def _seconds(stamp: TimeMessage) -> float:
            return stamp.sec + stamp.nanosec * 1e-9

        def _map_callback(self, message: OccupancyGrid) -> None:
            origin = message.info.origin
            self._map = OccupancyMap(
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=_yaw(
                    origin.orientation.x,
                    origin.orientation.y,
                    origin.orientation.z,
                    origin.orientation.w,
                ),
                data=tuple(message.data),
            )

        def _scan_callback(self, message: LaserScan) -> None:
            self._scan = message
            if self._core.observe_scan_stamp(self._seconds(message.header.stamp)):
                self._scan_score = None
                self._consecutive_scans = 0
                self._publish_status()
                return
            if self._map is None:
                return
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map",
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=0.05),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._scan_score = score_scan_pose(
                    self._map,
                    ranges=message.ranges,
                    angle_min=message.angle_min,
                    angle_increment=message.angle_increment,
                    range_min=message.range_min,
                    range_max=message.range_max,
                    sensor_x=translation.x,
                    sensor_y=translation.y,
                    sensor_yaw=_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
                    minimum_beams=100,
                )
                self._consecutive_scans += 1
            except (TransformException, ValueError):
                self._scan_score = None
                self._consecutive_scans = 0

        def _odom_callback(self, message: Odometry) -> None:
            self._odom = message
            pose = message.pose.pose
            twist = message.twist.twist
            commanded = (
                math.hypot(twist.linear.x, twist.linear.y) > 0.01
                or abs(twist.angular.z) > 0.01
            )
            restarted = self._core.observe_odometry(
                self._seconds(message.header.stamp),
                message.header.frame_id,
                pose.position.x,
                pose.position.y,
                _yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                commanded=commanded,
            )
            if restarted:
                self._publish_status()

        def _amcl_callback(self, message: PoseWithCovarianceStamped) -> None:
            self._amcl = message

        def _seed_tf(self, stamp: TimeMessage) -> bool:
            odom_frame = str(self.get_parameter("odom_frame").value)
            base_frame = str(self.get_parameter("base_frame").value)
            try:
                self._tf_buffer.lookup_transform(
                    odom_frame,
                    base_frame,
                    Time.from_msg(stamp),
                    timeout=Duration(seconds=0.02),
                )
                return True
            except TransformException:
                return False

        def _required_tf(self, stamp: TimeMessage) -> bool:
            odom_frame = str(self.get_parameter("odom_frame").value)
            base_frame = str(self.get_parameter("base_frame").value)
            observation_time = Time.from_msg(stamp)
            try:
                first = self._tf_buffer.lookup_transform(
                    "map", odom_frame, observation_time, timeout=Duration(seconds=0.02)
                )
                self._tf_buffer.lookup_transform(
                    odom_frame,
                    base_frame,
                    observation_time,
                    timeout=Duration(seconds=0.02),
                )
                del first
                self._last_tf_stamp = self._seconds(stamp)
                return True
            except TransformException:
                return False

        def _publish_seed(self, pose: PlanarPose) -> None:
            values = initial_pose_values(pose)
            message = PoseWithCovarianceStamped()
            message.header.frame_id = str(values["frame_id"])
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.pose.position.x = float(values["x"])
            message.pose.pose.position.y = float(values["y"])
            message.pose.pose.orientation.z = float(values["qz"])
            message.pose.pose.orientation.w = float(values["qw"])
            message.pose.covariance[0] = float(values["covariance_x"])
            message.pose.covariance[7] = float(values["covariance_y"])
            message.pose.covariance[35] = float(values["covariance_yaw"])
            self._initial_pose_pub.publish(message)

        def _timer_callback(self) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            scan_stamp = (
                self._seconds(self._scan.header.stamp) if self._scan is not None else 0.0
            )
            odom_stamp = (
                self._seconds(self._odom.header.stamp) if self._odom is not None else 0.0
            )
            stationary = False
            if self._odom is not None:
                twist = self._odom.twist.twist
                stationary = odometry_is_stationary(
                    twist.linear.x, twist.linear.y, twist.angular.z
                )
            amcl_topic = str(self.get_parameter("amcl_pose_topic").value)
            amcl_live = self.count_publishers(amcl_topic) > 0
            initial_pose_subscribed = self.count_subscribers("/initialpose") > 0
            seed_tf_ok = (
                self._scan is not None and self._seed_tf(self._scan.header.stamp)
            )
            inputs_available = (
                self._map is not None
                and self._scan is not None
                and self._odom is not None
                and amcl_live
                and initial_pose_subscribed
                and stationary
                and seed_tf_ok
                and 0.0 <= now - scan_stamp <= 0.30
                and 0.0 <= now - odom_stamp <= 0.30
            )
            self._core.set_inputs_available(inputs_available)
            seed = self._core.consume_seed_request(now)
            if seed is not None:
                self._publish_seed(seed)

            if (
                self._amcl is not None
                and self._scan is not None
                and self._odom is not None
                and self._scan_score is not None
            ):
                pose = self._amcl.pose.pose
                covariance = self._amcl.pose.covariance
                amcl_stamp = self._seconds(self._amcl.header.stamp)
                scan_stamp = self._seconds(self._scan.header.stamp)
                odom_stamp = self._seconds(self._odom.header.stamp)
                tf_ok = self._required_tf(self._scan.header.stamp)
                self._core.observe(
                    LocalizationObservation(
                        now=now,
                        amcl_stamp=amcl_stamp,
                        x=pose.position.x,
                        y=pose.position.y,
                        yaw=_yaw(
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ),
                        covariance_x=covariance[0],
                        covariance_y=covariance[7],
                        covariance_yaw=covariance[35],
                        pose_age=max(0.0, now - amcl_stamp),
                        scan_age=max(0.0, now - scan_stamp),
                        odom_age=max(0.0, now - odom_stamp),
                        tf_age=max(0.0, now - self._last_tf_stamp),
                        tf_ok=tf_ok,
                        valid_beams=self._scan_score.valid_beams,
                        consecutive_scans=self._consecutive_scans,
                        median_residual=self._scan_score.median_residual,
                        p80_residual=self._scan_score.p80_residual,
                    )
                )
            self._publish_status()

        def _reset_callback(self, _request, response):
            if not self._core.request_reset():
                response.success = False
                response.message = "localization reset is unavailable"
                return response
            response.success = True
            response.message = "localization reset accepted"
            return response

        def _publish_status(self) -> None:
            status = self._core.status()
            ready = Bool()
            ready.data = status.ready
            self._ready_pub.publish(ready)
            text = String()
            text.data = status_line(status)
            self._status_pub.publish(text)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are required")
    rclpy.init(args=args)
    node = None
    try:
        node = LocalizationSupervisor()
        rclpy.spin(node)
    except PoseConfigurationError as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
