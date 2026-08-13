"""Single owner for fixed-destination and RViz NavigateToPose goals."""

from __future__ import annotations

import math
from typing import Optional

from .map_geometry import OccupancyMap, quaternion_to_yaw, validate_goal_pose
from .mission_state import (
    GoalSource,
    MissionEffect,
    MissionStateMachine,
)
from .pose_config import PlanarPose, PoseConfigurationError, load_planar_pose


def mission_status_line(machine: MissionStateMachine) -> str:
    return (
        f"state={machine.state.value} active={'true' if machine.active else 'false'} "
        f"source={machine.source.value} reason={machine.reason}"
    )


def planar_pose_from_values(
    frame_id: str,
    x: float,
    y: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> PlanarPose:
    if frame_id != "map":
        raise ValueError("goal frame must be map")
    return PlanarPose("map", x, y, quaternion_to_yaw(qx, qy, qz, qw))


try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None
    Node = object


if rclpy is not None:

    class FixedMissionManager(Node):
        def __init__(self) -> None:
            super().__init__("fixed_mission_manager")
            self.declare_parameter("destination_pose_file", "")
            self.declare_parameter("minimum_clearance", 0.35)
            self.declare_parameter("heartbeat_rate", 10.0)
            destination_result = load_planar_pose(
                str(self.get_parameter("destination_pose_file").value),
                expected_kind="destination",
            )
            self._machine = MissionStateMachine(destination_result.pose)
            self._map: Optional[OccupancyMap] = None
            self._goal_handle = None
            self._mission_id = 0

            status_qos = QoSProfile(depth=1)
            status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            status_qos.reliability = ReliabilityPolicy.RELIABLE
            map_qos = QoSProfile(depth=1)
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            self._active_pub = self.create_publisher(Bool, "/mission/active", 10)
            self._status_pub = self.create_publisher(
                String, "/mission/status", status_qos
            )
            self.create_subscription(Bool, "/localization/ready", self._loc_cb, 10)
            self.create_subscription(Bool, "/motion_gate/enabled", self._gate_cb, 10)
            self.create_subscription(PoseStamped, "/goal_pose", self._rviz_cb, 10)
            self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
            self.create_service(Trigger, "/mission/start", self._start_cb)
            self.create_service(Trigger, "/mission/cancel", self._cancel_cb)
            self._action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            rate = float(self.get_parameter("heartbeat_rate").value)
            self._timer = self.create_timer(1.0 / rate, self._timer_cb)
            self._publish_status()

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        @staticmethod
        def _yaw(q) -> float:
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )

        def _loc_cb(self, message: Bool) -> None:
            decision = self._machine.update_localization(message.data, self._now())
            self._apply(decision)

        def _gate_cb(self, message: Bool) -> None:
            decision = self._machine.update_motion_gate(message.data, self._now())
            self._apply(decision)

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

        def _valid(self, pose: PlanarPose) -> bool:
            if self._map is None:
                self._machine.reason = "occupancy map is unavailable"
                return False
            validation = validate_goal_pose(
                self._map,
                pose.x,
                pose.y,
                math.sin(pose.yaw / 2.0),
                math.cos(pose.yaw / 2.0),
                float(self.get_parameter("minimum_clearance").value),
            )
            if not validation.accepted:
                self._machine.reason = validation.reason
            return validation.accepted

        def _start_cb(self, _request, response):
            goal = self._machine.destination
            decision = self._machine.request_fixed(
                now=self._now(), goal_valid=goal is not None and self._valid(goal)
            )
            response.success = decision.effect is MissionEffect.SEND_GOAL
            response.message = decision.reason
            self._apply(decision)
            return response

        def _rviz_cb(self, message: PoseStamped) -> None:
            q = message.pose.orientation
            try:
                pose = planar_pose_from_values(
                    message.header.frame_id,
                    message.pose.position.x,
                    message.pose.position.y,
                    q.x,
                    q.y,
                    q.z,
                    q.w,
                )
            except (PoseConfigurationError, ValueError) as exc:
                self._machine.reason = str(exc)
                self._publish_status()
                return
            decision = self._machine.request_rviz(
                pose, now=self._now(), goal_valid=self._valid(pose)
            )
            self._apply(decision)

        def _cancel_cb(self, _request, response):
            decision = self._machine.request_cancel()
            response.success = decision.effect is MissionEffect.CANCEL_GOAL
            response.message = decision.reason
            self._apply(decision)
            return response

        def _apply(self, decision) -> None:
            if decision.effect is MissionEffect.SEND_GOAL:
                self._send_goal(decision.goal)
            elif decision.effect is MissionEffect.CANCEL_GOAL:
                self._cancel_goal()
            self._publish_status()

        def _send_goal(self, pose: PlanarPose) -> None:
            if not self._action.server_is_ready():
                self._machine.update_action_server(False)
                self._machine.goal_response(False)
                self._publish_status()
                return
            self._mission_id += 1
            mission_id = self._mission_id
            message = NavigateToPose.Goal()
            message.pose.header.frame_id = "map"
            message.pose.header.stamp = self.get_clock().now().to_msg()
            message.pose.pose.position.x = pose.x
            message.pose.pose.position.y = pose.y
            message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
            message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
            future = self._action.send_goal_async(message)
            future.add_done_callback(
                lambda done: self._goal_response(done, mission_id)
            )

        def _goal_response(self, future, mission_id: int) -> None:
            if mission_id != self._mission_id:
                return
            try:
                self._goal_handle = future.result()
            except Exception as exc:
                self._machine.goal_response(False)
                self._machine.reason = f"goal send failed: {exc}"
                self._publish_status()
                return
            accepted = bool(self._goal_handle and self._goal_handle.accepted)
            self._machine.goal_response(accepted)
            if accepted:
                result = self._goal_handle.get_result_async()
                result.add_done_callback(
                    lambda done: self._action_result(done, mission_id)
                )
            self._publish_status()

        def _action_result(self, future, mission_id: int) -> None:
            if mission_id != self._mission_id:
                return
            status = future.result().status
            outcome = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(status, "FAILED")
            self._machine.action_result(outcome)
            self._goal_handle = None
            self._publish_status()

        def _cancel_goal(self) -> None:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

        def _timer_cb(self) -> None:
            self._machine.update_action_server(self._action.server_is_ready())
            now = self._now()
            if self._machine.active:
                if (
                    self._machine._localization_stamp is not None
                    and now - self._machine._localization_stamp > 0.30
                ):
                    self._apply(
                        self._machine.update_localization(False, now)
                    )
                if (
                    self._machine._gate_stamp is not None
                    and now - self._machine._gate_stamp > 0.30
                ):
                    self._apply(self._machine.update_motion_gate(False, now))
            self._publish_status()

        def _publish_status(self) -> None:
            active = Bool()
            active.data = self._machine.active
            self._active_pub.publish(active)
            status = String()
            status.data = mission_status_line(self._machine)
            self._status_pub.publish(status)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are required")
    rclpy.init(args=args)
    node = FixedMissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
