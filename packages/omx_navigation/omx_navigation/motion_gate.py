"""ROS adapter for the standalone fail-closed motion gate."""

from __future__ import annotations

import time
from typing import Optional

from .motion_gate_core import GateResult, MotionGateCore, VelocityCommand


def gate_status_line(result: GateResult) -> str:
    return f"enabled={'true' if result.enabled else 'false'} reason={result.reason}"


try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None
    Node = object


if rclpy is not None:

    class MotionGate(Node):
        def __init__(self) -> None:
            super().__init__("motion_gate")
            self.declare_parameter("cmd_vel_input", "/cmd_vel_nav")
            self.declare_parameter("cmd_vel_output", "/cmd_vel_safe")
            self.declare_parameter("localization_topic", "/localization/ready")
            self.declare_parameter("estop_topic", "/emergency_stop")
            self.declare_parameter("mission_stop_topic", "/mission/stop_required")
            self.declare_parameter("heartbeat_rate", 10.0)
            self.declare_parameter("localization_timeout", 0.30)
            self.declare_parameter("estop_timeout", 0.30)
            self.declare_parameter("command_timeout", 0.25)
            self.declare_parameter("mission_stop_timeout", 0.30)
            self.declare_parameter("max_linear_speed", 0.20)
            self.declare_parameter("max_angular_speed", 0.40)

            self._core = MotionGateCore(
                localization_timeout=float(
                    self.get_parameter("localization_timeout").value
                ),
                estop_timeout=float(self.get_parameter("estop_timeout").value),
                command_timeout=float(self.get_parameter("command_timeout").value),
                mission_stop_timeout=float(
                    self.get_parameter("mission_stop_timeout").value
                ),
                max_linear_speed=float(
                    self.get_parameter("max_linear_speed").value
                ),
                max_angular_speed=float(
                    self.get_parameter("max_angular_speed").value
                ),
            )
            status_qos = QoSProfile(depth=1)
            status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            status_qos.reliability = ReliabilityPolicy.RELIABLE
            self._safe_pub = self.create_publisher(
                Twist, str(self.get_parameter("cmd_vel_output").value), 10
            )
            self._enabled_pub = self.create_publisher(
                Bool, "/motion_gate/enabled", 10
            )
            self._status_pub = self.create_publisher(
                String, "/motion_gate/status", status_qos
            )
            self.create_subscription(
                Twist,
                str(self.get_parameter("cmd_vel_input").value),
                self._command_callback,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("localization_topic").value),
                self._localization_callback,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("estop_topic").value),
                self._estop_callback,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("mission_stop_topic").value),
                self._mission_stop_callback,
                10,
            )
            self.create_service(Trigger, "/motion_gate/arm", self._arm_callback)
            self.create_service(
                Trigger, "/motion_gate/disarm", self._disarm_callback
            )
            rate = float(self.get_parameter("heartbeat_rate").value)
            if rate <= 0.0:
                raise ValueError("heartbeat_rate must be positive")
            self._timer = self.create_timer(1.0 / rate, self._timer_callback)

        def _now(self) -> float:
            return time.monotonic()

        def _localization_callback(self, message: Bool) -> None:
            self._core.update_localization(message.data, self._now())

        def _estop_callback(self, message: Bool) -> None:
            self._core.update_estop(message.data, self._now())

        def _command_callback(self, message: Twist) -> None:
            self._core.update_command(
                VelocityCommand(
                    message.linear.x, message.linear.y, message.angular.z
                ),
                self._now(),
            )

        def _mission_stop_callback(self, message: Bool) -> None:
            self._core.update_mission_stop(message.data, self._now())

        def _arm_callback(self, _request, response):
            result = self._core.arm(self._now())
            response.success = result.accepted
            response.message = result.reason
            self._publish(self._core.evaluate(self._now()))
            return response

        def _disarm_callback(self, _request, response):
            self._core.disarm()
            response.success = True
            response.message = "motion gate disarmed"
            self._publish(self._core.evaluate(self._now()))
            return response

        def _timer_callback(self) -> None:
            self._publish(self._core.evaluate(self._now()))

        def _publish(self, result: GateResult) -> None:
            twist = Twist()
            twist.linear.x = result.command.vx
            twist.linear.y = result.command.vy
            twist.angular.z = result.command.yaw
            self._safe_pub.publish(twist)
            enabled = Bool()
            enabled.data = result.enabled
            self._enabled_pub.publish(enabled)
            status = String()
            status.data = gate_status_line(result)
            self._status_pub.publish(status)

        def publish_stop(self) -> None:
            self._core.disarm()
            self._publish(self._core.evaluate(self._now()))


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are required")
    rclpy.init(args=args)
    node: Optional[MotionGate] = None
    try:
        node = MotionGate()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
