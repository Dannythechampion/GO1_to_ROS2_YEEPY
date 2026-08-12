"""Publish a planar navigation frame derived from the FAST-LIO body transform."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from omx_navigation.planar_transform import planarize_transform


class PlanarBaseFrame(Node):
    """Broadcast ``odom_frame -> planar_base_frame`` without changing FAST-LIO TF."""

    _ERROR_LOG_INTERVAL_SECONDS = 5.0

    def __init__(self) -> None:
        super().__init__("planar_base_frame")
        self._odom_frame = self._frame_parameter("odom_frame", "camera_init")
        self._source_base_frame = self._frame_parameter("source_base_frame", "body")
        self._planar_base_frame = self._frame_parameter("planar_base_frame", "body_nav")
        self._validate_frame_topology()
        publish_rate = self._positive_parameter("publish_rate", 20.0)
        # Keep the 20 Hz publisher responsive even while upstream TF is absent.
        self._transform_timeout = self._nonnegative_parameter("transform_timeout", 0.04)
        self._last_error_log_at: float | None = None
        self._transform_error_active = False

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_planar_transform)

    def _publish_planar_transform(self) -> None:
        try:
            source = self._buffer.lookup_transform(
                self._odom_frame,
                self._source_base_frame,
                Time(),
                timeout=Duration(seconds=self._transform_timeout),
            )
            planar = planarize_transform(
                source.transform.translation.x,
                source.transform.translation.y,
                source.transform.translation.z,
                source.transform.rotation.x,
                source.transform.rotation.y,
                source.transform.rotation.z,
                source.transform.rotation.w,
            )
        except (TransformException, ValueError) as error:
            self._report_transform_error(error)
            return

        if self._transform_error_active:
            self.get_logger().info("Planar base transform lookup recovered")
            self._transform_error_active = False
            self._last_error_log_at = None

        output = TransformStamped()
        # Preserve the TF sample time: using the timer's current time would make
        # delayed data look newer than the FAST-LIO pose from which it was derived.
        output.header.stamp = source.header.stamp
        output.header.frame_id = self._odom_frame
        output.child_frame_id = self._planar_base_frame
        output.transform.translation.x, output.transform.translation.y, output.transform.translation.z = (
            planar.translation
        )
        output.transform.rotation.x, output.transform.rotation.y, output.transform.rotation.z, output.transform.rotation.w = (
            planar.quaternion
        )
        self._broadcaster.sendTransform(output)

    def _report_transform_error(self, error: Exception) -> None:
        now = time.monotonic()
        if (
            self._last_error_log_at is None
            or now - self._last_error_log_at >= self._ERROR_LOG_INTERVAL_SECONDS
        ):
            self.get_logger().warn(f"Cannot publish planar base transform: {error}")
            self._last_error_log_at = now
        self._transform_error_active = True

    def _frame_parameter(self, name: str, default: str) -> str:
        value = self.declare_parameter(name, default).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a non-empty frame name")
        return value.strip()

    def _positive_parameter(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default).value
        if not _is_finite_number(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return float(value)

    def _nonnegative_parameter(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default).value
        if not _is_finite_number(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return float(value)

    def _validate_frame_topology(self) -> None:
        if self._odom_frame == self._planar_base_frame:
            raise ValueError("planar_base_frame must differ from odom_frame")
        if self._odom_frame == self._source_base_frame:
            raise ValueError("source_base_frame must differ from odom_frame")
        if self._source_base_frame == self._planar_base_frame:
            raise ValueError("planar_base_frame must differ from source_base_frame to avoid TF feedback")


def _is_finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PlanarBaseFrame()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
