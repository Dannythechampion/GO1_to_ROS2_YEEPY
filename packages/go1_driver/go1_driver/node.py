"""ROS 2 cmd_vel to Unitree Go1 HighLevel UDP safety bridge."""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .command_filter import CommandFilter, MotionCommand, apply_watchdog
from .unitree_adapter import UnitreeHighLevel


class Go1Driver(Node):
    """Filter cmd_vel and optionally forward it to the physical Go1."""

    def __init__(self) -> None:
        super().__init__("go1_driver")

        self.declare_parameter("arm", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("applied_topic", "/go1/cmd_vel_applied")
        self.declare_parameter("state_topic", "/go1/control_state")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("cmd_timeout", 0.35)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_yaw_speed", 0.40)
        self.declare_parameter("linear_deadband", 0.025)
        self.declare_parameter("yaw_deadband", 0.04)
        self.declare_parameter("holonomic", True)
        self.declare_parameter("invert_lateral", False)
        self.declare_parameter(
            "sdk_path",
            "/mnt/t500/go1_sdk/unitree_legged_sdk/lib/python/arm64",
        )
        self.declare_parameter("robot_ip", "192.168.123.161")
        self.declare_parameter("robot_port", 8082)
        self.declare_parameter("local_port", 8080)
        self.declare_parameter("shutdown_stand_repeats", 30)

        self._arm = bool(self.get_parameter("arm").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._shutdown_stand_repeats = int(
            self.get_parameter("shutdown_stand_repeats").value
        )
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self._cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout must be positive")
        if self._shutdown_stand_repeats < 1:
            raise ValueError("shutdown_stand_repeats must be at least 1")

        self._filter = CommandFilter(
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_yaw_speed=float(self.get_parameter("max_yaw_speed").value),
            linear_deadband=float(self.get_parameter("linear_deadband").value),
            yaw_deadband=float(self.get_parameter("yaw_deadband").value),
            holonomic=bool(self.get_parameter("holonomic").value),
            invert_lateral=bool(self.get_parameter("invert_lateral").value),
        )

        self._lock = threading.Lock()
        self._last_command = MotionCommand.stand("waiting for cmd_vel")
        self._last_command_time: Optional[float] = None
        self._last_reported_reason: Optional[str] = None
        self._shutdown_sent = False

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        applied_topic = str(self.get_parameter("applied_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self._applied_pub = self.create_publisher(Twist, applied_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self._subscription = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 1
        )

        self._robot: Optional[UnitreeHighLevel] = None
        if self._arm:
            self._robot = UnitreeHighLevel(
                sdk_path=str(self.get_parameter("sdk_path").value),
                robot_ip=str(self.get_parameter("robot_ip").value),
                robot_port=int(self.get_parameter("robot_port").value),
                local_port=int(self.get_parameter("local_port").value),
            )

        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)
        mode = "ARMED" if self._arm else "DRY-RUN"
        self.get_logger().warning(
            f"Go1 driver started in {mode} mode; listening on {cmd_vel_topic}"
        )

    def _cmd_vel_callback(self, message: Twist) -> None:
        command = self._filter.filter(
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        with self._lock:
            self._last_command = command
            self._last_command_time = time.monotonic()

    def _current_command(self) -> MotionCommand:
        with self._lock:
            command = self._last_command
            last_time = self._last_command_time
        return apply_watchdog(
            command=command,
            last_command_time=last_time,
            now=time.monotonic(),
            timeout=self._cmd_timeout,
        )

    def _timer_callback(self) -> None:
        command = self._current_command()

        applied = Twist()
        applied.linear.x = command.vx
        applied.linear.y = command.vy
        applied.angular.z = command.yaw
        self._applied_pub.publish(applied)

        state = String()
        state.data = (
            f"{'ARMED' if self._arm else 'DRY-RUN'} mode={command.mode} "
            f"vx={command.vx:.3f} vy={command.vy:.3f} "
            f"yaw={command.yaw:.3f} reason={command.reason}"
        )
        self._state_pub.publish(state)

        if self._robot is not None:
            self._robot.send(command)

        if command.reason != self._last_reported_reason:
            self.get_logger().info(state.data)
            self._last_reported_reason = command.reason

    def shutdown_robot(self) -> None:
        if self._shutdown_sent:
            return
        self._shutdown_sent = True
        if self._robot is not None:
            self.get_logger().warning("Sending repeated stand commands before shutdown")
            self._robot.stand(repeats=self._shutdown_stand_repeats)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[Go1Driver] = None
    try:
        node = Go1Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
