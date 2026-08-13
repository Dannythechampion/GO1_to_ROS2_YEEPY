"""Collect stable AMCL samples and write a commissioned start pose."""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import List, Optional

from .pose_config import PlanarPose, write_planar_pose_atomic


class CommissioningError(ValueError):
    pass


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float
    covariance_x: float
    covariance_y: float
    covariance_yaw: float
    linear_speed: float
    angular_speed: float


class StartPoseEstimator:
    def __init__(
        self,
        sample_count: int = 10,
        max_linear_speed: float = 0.01,
        max_angular_speed: float = 0.01,
        max_position_spread: float = 0.10,
        max_yaw_spread: float = math.radians(5.0),
    ) -> None:
        if sample_count < 2:
            raise ValueError("sample_count must be at least 2")
        self.sample_count = sample_count
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_position_spread = max_position_spread
        self.max_yaw_spread = max_yaw_spread
        self.samples: List[PoseSample] = []

    def add(self, sample: PoseSample) -> Optional[PlanarPose]:
        values = tuple(sample.__dict__.values())
        if not all(math.isfinite(float(value)) for value in values):
            raise CommissioningError("sample values must be finite")
        if abs(sample.linear_speed) > self.max_linear_speed or abs(
            sample.angular_speed
        ) > self.max_angular_speed:
            raise CommissioningError("robot is moving")
        if (
            sample.covariance_x > 0.04
            or sample.covariance_y > 0.04
            or sample.covariance_yaw > 0.0305
        ):
            raise CommissioningError("sample covariance exceeds readiness limits")
        if self.samples and sample.stamp <= self.samples[-1].stamp:
            raise CommissioningError("sample timestamp must increase")
        self.samples.append(sample)
        if len(self.samples) < self.sample_count:
            return None
        if len(self.samples) > self.sample_count:
            raise CommissioningError("commissioning set is already complete")

        xs = [item.x for item in self.samples]
        ys = [item.y for item in self.samples]
        if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) > self.max_position_spread:
            raise CommissioningError("position spread exceeds limit")
        sin_mean = sum(math.sin(item.yaw) for item in self.samples) / len(self.samples)
        cos_mean = sum(math.cos(item.yaw) for item in self.samples) / len(self.samples)
        yaw = math.atan2(sin_mean, cos_mean)
        yaw_errors = [
            abs((item.yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi)
            for item in self.samples
        ]
        if max(yaw_errors) > self.max_yaw_spread:
            raise CommissioningError("yaw spread exceeds limit")
        return PlanarPose("map", median(xs), median(ys), yaw)


try:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ImportError:  # Pure estimator remains testable without ROS installed.
    rclpy = None
    Node = object


if rclpy is not None:

    class CommissionStartPoseNode(Node):
        def __init__(self) -> None:
            super().__init__("commission_start_pose")
            self.declare_parameter("output_path", "")
            self.declare_parameter("sample_count", 10)
            self.declare_parameter("overwrite", False)
            output_path = str(self.get_parameter("output_path").value)
            if not output_path:
                raise ValueError("output_path must be explicitly configured")
            self._output_path = output_path
            self._overwrite = bool(self.get_parameter("overwrite").value)
            self._estimator = StartPoseEstimator(
                sample_count=int(self.get_parameter("sample_count").value)
            )
            self._linear_speed = float("inf")
            self._angular_speed = float("inf")
            self._complete = False
            self.create_subscription(Odometry, "/Odometry", self._odom_callback, 10)
            self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", self._pose_callback, 10
            )

        def _odom_callback(self, message: Odometry) -> None:
            self._linear_speed = math.hypot(
                message.twist.twist.linear.x, message.twist.twist.linear.y
            )
            self._angular_speed = abs(message.twist.twist.angular.z)

        def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
            if self._complete:
                return
            q = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            covariance = message.pose.covariance
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            try:
                result = self._estimator.add(
                    PoseSample(
                        stamp=stamp,
                        x=message.pose.pose.position.x,
                        y=message.pose.pose.position.y,
                        yaw=yaw,
                        covariance_x=covariance[0],
                        covariance_y=covariance[7],
                        covariance_yaw=covariance[35],
                        linear_speed=self._linear_speed,
                        angular_speed=self._angular_speed,
                    )
                )
            except CommissioningError as exc:
                self.get_logger().warning(str(exc))
                return
            if result is not None:
                write_planar_pose_atomic(
                    self._output_path,
                    result,
                    kind="start",
                    overwrite=self._overwrite,
                )
                self._complete = True
                self.get_logger().info(f"saved start pose to {self._output_path}")


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are required")
    rclpy.init(args=args)
    node = CommissionStartPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
