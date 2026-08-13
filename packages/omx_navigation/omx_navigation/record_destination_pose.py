"""Record the next validated RViz goal as the fixed destination."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Union

from .map_geometry import OccupancyMap, validate_goal_pose
from .pose_config import PlanarPose, write_planar_pose_atomic


def save_destination(
    path: Union[str, Path], pose: PlanarPose, *, overwrite: bool
) -> None:
    write_planar_pose_atomic(path, pose, kind="destination", overwrite=overwrite)


try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
except ImportError:
    rclpy = None
    Node = object


if rclpy is not None:

    class DestinationRecorder(Node):
        def __init__(self) -> None:
            super().__init__("record_destination_pose")
            self.declare_parameter("output_path", "")
            self.declare_parameter("map_topic", "/map")
            self.declare_parameter("goal_topic", "/goal_pose")
            self.declare_parameter("minimum_clearance", 0.35)
            self.declare_parameter("overwrite", False)
            self._output = str(self.get_parameter("output_path").value)
            if not self._output:
                raise ValueError("output_path must be explicitly configured")
            self._complete = False
            self._map = None
            map_qos = QoSProfile(depth=1)
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("map_topic").value),
                self._map_cb,
                map_qos,
            )
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("goal_topic").value),
                self._goal_cb,
                10,
            )

        @staticmethod
        def _yaw(q) -> float:
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )

        def _map_cb(self, message: OccupancyGrid) -> None:
            origin = message.info.origin
            self._map = OccupancyMap(
                message.info.width,
                message.info.height,
                message.info.resolution,
                origin.position.x,
                origin.position.y,
                self._yaw(origin.orientation),
                tuple(message.data),
            )

        def _goal_cb(self, message: PoseStamped) -> None:
            if self._complete or self._map is None:
                return
            if message.header.frame_id != "map":
                self.get_logger().error("destination frame must be map")
                return
            pose = PlanarPose(
                "map",
                message.pose.position.x,
                message.pose.position.y,
                self._yaw(message.pose.orientation),
            )
            validation = validate_goal_pose(
                self._map,
                pose.x,
                pose.y,
                math.sin(pose.yaw / 2.0),
                math.cos(pose.yaw / 2.0),
                float(self.get_parameter("minimum_clearance").value),
            )
            if not validation.accepted:
                self.get_logger().error(validation.reason)
                return
            save_destination(
                self._output,
                pose,
                overwrite=bool(self.get_parameter("overwrite").value),
            )
            self._complete = True
            self.get_logger().info(f"saved destination to {self._output}")


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are required")
    rclpy.init(args=args)
    node = DestinationRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
