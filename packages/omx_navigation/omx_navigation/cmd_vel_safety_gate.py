"""ROS2 wrapper for the localization-aware velocity safety gate."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from omx_navigation.cmd_vel_gate_core import VelocityCommand, VelocityGate


class CmdVelSafetyGate(Node):
    """Forward Nav2 velocity only while localization reports fresh readiness."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_safety_gate")
        ready_timeout = self.declare_parameter("ready_timeout", 0.30).value
        command_timeout = self.declare_parameter("command_timeout", 0.30).value
        self._gate = VelocityGate(ready_timeout, command_timeout)
        # `/cmd_vel` remains volatile: retaining a moving command could replay it
        # when the driver reconnects. Closed-state timer events publish fresh stops.
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._ready_subscription = self.create_subscription(
            Bool, "/localization_supervisor/ready", self._on_ready, 10
        )
        self._command_subscription = self.create_subscription(
            Twist, "/cmd_vel_nav", self._on_command, 10
        )
        self._watchdog = self.create_timer(0.05, self._on_watchdog)

    def _on_ready(self, message: Bool) -> None:
        # A false transition publishes one immediate stop; the 20 Hz timer below
        # repeats stops while the gate remains closed.
        self._publish_decision(self._gate.update_ready(message.data, self._now()))

    def _on_command(self, message: Twist) -> None:
        command = VelocityCommand(
            vx=message.linear.x,
            vy=message.linear.y,
            yaw=message.angular.z,
        )
        self._publish_decision(self._gate.filter(command, self._now()))

    def _on_watchdog(self) -> None:
        self._publish_decision(self._gate.watchdog(self._now()))

    def _publish_decision(self, decision) -> None:
        if not decision.publish:
            return
        message = Twist()
        message.linear.x = decision.command.vx
        message.linear.y = decision.command.vy
        message.angular.z = decision.command.yaw
        self._publisher.publish(message)

    def publish_stop(self) -> None:
        """Best-effort zero command for process shutdown."""
        try:
            self._publisher.publish(Twist())
        except Exception:  # Shutdown may already have torn down the publisher.
            pass

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None) -> None:
    node = None
    rclpy.init(args=args)
    try:
        node = CmdVelSafetyGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.publish_stop()
                except Exception:
                    pass
                try:
                    node.destroy_node()
                except Exception:
                    pass
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
