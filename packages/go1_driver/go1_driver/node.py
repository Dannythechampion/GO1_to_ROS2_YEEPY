"""ROS 2 cmd_vel to Unitree Go1 HighLevel UDP safety bridge."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

from .arbitration import CommandArbiter
from .command_filter import STAND_MODE, CommandFilter, MotionCommand, apply_watchdog
from .execution_monitor import ExecutionGuard, ExecutionMonitor, ExecutionState, ExecutionVerdict
from .robot_state import NO_REMOTE, LinkMonitor, OverrideLatch, RobotState
from .unitree_adapter import UnitreeHighLevel

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # Supports the pure-Python safety tests without ROS installed.
    class ExternalShutdownException(Exception):
        """Fallback matching rclpy's normal external-shutdown signal."""


NORMAL_SHUTDOWN_EXCEPTIONS = (KeyboardInterrupt, ExternalShutdownException)
ARMED_CONFIRMATION_TOKEN = "GO1_ARMED_AND_ESTOP_READY"
STATUS_PERIOD = 0.1


def validate_arming(arm: bool, armed_confirmation: str) -> None:
    if arm and armed_confirmation != ARMED_CONFIRMATION_TOKEN:
        raise RuntimeError(
            "arm=true requires armed_confirmation=" + ARMED_CONFIRMATION_TOKEN
        )


def _yaw_from_quaternion(orientation) -> float:
    x, y, z, w = (float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Go1Driver(Node):
    """Filter cmd_vel and optionally forward it to the physical Go1."""

    def __init__(self) -> None:
        super().__init__("go1_driver")

        self.declare_parameter("arm", False)
        self.declare_parameter("armed_confirmation", "")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("applied_topic", "/go1/cmd_vel_applied")
        self.declare_parameter("state_topic", "/go1/control_state")
        self.declare_parameter("robot_state_topic", "/go1/robot_state")
        self.declare_parameter("manual_override_topic", "/go1/manual_override")
        self.declare_parameter("execution_fault_topic", "/go1/execution_fault")
        # Odometry for the commanded-vs-executed check; empty disables it.
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("cmd_timeout", 0.35)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_yaw_speed", 0.40)
        self.declare_parameter("linear_deadband", 0.025)
        self.declare_parameter("yaw_deadband", 0.04)
        self.declare_parameter("holonomic", True)
        self.declare_parameter("invert_lateral", False)
        self.declare_parameter("remote_stick_deadband", 0.10)
        self.declare_parameter("override_release_s", 1.0)
        self.declare_parameter("link_timeout_s", 0.5)
        self.declare_parameter("rearm_zero_s", 0.5)
        self.declare_parameter("refusal_hold_s", 5.0)
        self.declare_parameter("uncommanded_hold_s", 1.0)
        self.declare_parameter("log_period_s", 1.0)
        self.declare_parameter(
            "sdk_path",
            "/mnt/t500/go1_sdk/unitree_legged_sdk/lib/python/arm64",
        )
        self.declare_parameter("robot_ip", "192.168.123.161")
        self.declare_parameter("robot_port", 8082)
        self.declare_parameter("local_port", 8080)
        self.declare_parameter("shutdown_stand_repeats", 30)

        self._arm = bool(self.get_parameter("arm").value)
        validate_arming(
            self._arm, str(self.get_parameter("armed_confirmation").value)
        )
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._shutdown_stand_repeats = int(
            self.get_parameter("shutdown_stand_repeats").value
        )
        self._stick_deadband = float(self.get_parameter("remote_stick_deadband").value)
        self._log_period = float(self.get_parameter("log_period_s").value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self._cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout must be positive")
        if self._shutdown_stand_repeats < 1:
            raise ValueError("shutdown_stand_repeats must be at least 1")
        if not math.isfinite(self._stick_deadband) or self._stick_deadband < 0.0:
            raise ValueError("remote_stick_deadband must be finite and non-negative")
        if not math.isfinite(self._log_period) or self._log_period <= 0.0:
            raise ValueError("log_period_s must be finite and positive")

        self._filter = CommandFilter(
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_yaw_speed=float(self.get_parameter("max_yaw_speed").value),
            linear_deadband=float(self.get_parameter("linear_deadband").value),
            yaw_deadband=float(self.get_parameter("yaw_deadband").value),
            holonomic=bool(self.get_parameter("holonomic").value),
            invert_lateral=bool(self.get_parameter("invert_lateral").value),
        )
        self._override = OverrideLatch(float(self.get_parameter("override_release_s").value))
        self._link = LinkMonitor(float(self.get_parameter("link_timeout_s").value))
        self._arbiter = CommandArbiter(float(self.get_parameter("rearm_zero_s").value))
        self._execution = ExecutionMonitor()
        self._guard = ExecutionGuard(
            refusal_hold=float(self.get_parameter("refusal_hold_s").value),
            uncommanded_hold=float(self.get_parameter("uncommanded_hold_s").value),
        )

        self._lock = threading.Lock()
        self._last_command = MotionCommand.stand("waiting for cmd_vel")
        self._last_requested = (0.0, 0.0, 0.0)
        self._last_command_time: Optional[float] = None
        self._last_reported_reason: Optional[str] = None
        self._shutdown_sent = False
        self._robot_state: Optional[RobotState] = None
        self._execution_verdict = ExecutionVerdict(ExecutionState.UNKNOWN)
        self._execution_fault: Optional[ExecutionState] = None
        self._last_status_at: Optional[float] = None
        self._last_log_at: Optional[float] = None
        self._link_was_up = False

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        applied_topic = str(self.get_parameter("applied_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self._applied_pub = self.create_publisher(Twist, applied_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self._robot_state_pub = self.create_publisher(
            String, str(self.get_parameter("robot_state_topic").value), 10
        )
        self._override_pub = self.create_publisher(
            Bool, str(self.get_parameter("manual_override_topic").value), 10
        )
        self._fault_pub = self.create_publisher(
            Bool, str(self.get_parameter("execution_fault_topic").value), 10
        )
        self._subscription = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 1
        )
        odom_topic = str(self.get_parameter("odom_topic").value).strip()
        # A robot that is not armed does not move, so judging whether it
        # followed its commands would only ever report refusals.
        self._monitor_execution = self._arm and bool(odom_topic)
        if self._monitor_execution:
            self._odom_subscription = self.create_subscription(
                Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
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
            + ("" if self._monitor_execution else "; execution monitoring off")
        )

    def _cmd_vel_callback(self, message: Twist) -> None:
        command = self._filter.filter(
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        requested = tuple(
            value if math.isfinite(value) else 0.0
            for value in (float(message.linear.x), float(message.linear.y), float(message.angular.z))
        )
        with self._lock:
            self._last_command = command
            self._last_requested = requested
            self._last_command_time = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        try:
            pose = message.pose.pose
            self._execution.pose(
                time.monotonic(), float(pose.position.x), float(pose.position.y),
                _yaw_from_quaternion(pose.orientation),
            )
        except (AttributeError, TypeError, ValueError):
            pass

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
        now = time.monotonic()
        requested = self._current_command()
        command = self._arbiter.decide(now, requested, self._override.active)

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
            self._robot_state = self._robot.send(command)
            self._link.update(now, self._robot.last_reply_fresh)
            remote = self._robot_state.remote if self._link.up(now) else NO_REMOTE
            # Takes effect from the next cycle, 10 ms later.
            self._override.update(now, remote.active(self._stick_deadband))
        if self._monitor_execution:
            if self._override.active:
                # The operator is driving; that is known, not a fault to find.
                self._execution.reset()
                self._guard.clear()
            else:
                self._execution.command(now, command.vx, command.vy, command.yaw)

        if self._last_status_at is None or now - self._last_status_at >= STATUS_PERIOD:
            self._last_status_at = now
            self._update_execution(now)
            self._publish_status(now, command, requested)

        if command.reason != self._last_reported_reason:
            self.get_logger().info(self._log_line(state.data))
            self._last_reported_reason = command.reason
            self._last_log_at = now
        elif self._should_log_periodically(now, command, requested):
            # Transitions alone read like a command trace and are not one: on
            # 09-18 three transition lines were taken for 20 s of commands.
            self.get_logger().info(self._log_line(state.data))
            self._last_log_at = now

    def _update_execution(self, now: float) -> None:
        if not self._monitor_execution or self._override.active:
            return
        self._execution_verdict = self._execution.evaluate(now)
        fault = self._guard.update(now, self._execution_verdict)
        if fault is None:
            return
        verdict = self._execution_verdict
        self.get_logger().error(
            f"Robot is not following commands ({fault.value}): commanded "
            f"{verdict.commanded_distance:.2f} m / {math.degrees(verdict.commanded_turn):.0f} deg, "
            f"measured {verdict.achieved_distance:.2f} m / {math.degrees(verdict.achieved_turn):.0f} deg "
            f"over {self._execution.policy.window:.0f} s; holding stand"
        )
        self._execution_fault = fault
        self._arbiter.block(f"execution fault {fault.value}")
        self._guard.clear()
        self._execution.reset()

    def _publish_status(self, now: float, command: MotionCommand, requested: MotionCommand) -> None:
        override = self._override.active
        blocked = self._arbiter.blocked_reason
        if blocked is None:
            self._execution_fault = None
        fault_active = self._execution_fault is not None and blocked is not None

        override_message = Bool()
        override_message.data = override
        self._override_pub.publish(override_message)
        fault_message = Bool()
        fault_message.data = fault_active
        self._fault_pub.publish(fault_message)

        link_up = self._link.up(now) if self._robot is not None else None
        if self._robot is not None and link_up != self._link_was_up:
            if link_up:
                self.get_logger().info("Go1 link up: live HighState replies")
            else:
                self.get_logger().error("Go1 link down: no live HighState reply")
            self._link_was_up = bool(link_up)

        robot = self._robot_state
        verdict = self._execution_verdict
        with self._lock:
            raw = self._last_requested
        report = {
            "armed": self._arm,
            "link_up": link_up,
            "manual_override": override,
            "execution_fault": self._execution_fault.value if fault_active else None,
            "arbiter": blocked,
            "requested": {"vx": raw[0], "vy": raw[1], "yaw": raw[2], "reason": requested.reason},
            "applied": {"mode": command.mode, "vx": command.vx, "vy": command.vy, "yaw": command.yaw, "reason": command.reason},
            "execution": {
                "state": verdict.state.value,
                "commanded_distance": verdict.commanded_distance,
                "achieved_distance": verdict.achieved_distance,
                "commanded_turn_deg": math.degrees(verdict.commanded_turn),
                "achieved_turn_deg": math.degrees(verdict.achieved_turn),
            },
            "robot": None if robot is None else {
                "live": robot.live,
                "mode": robot.mode,
                "velocity": list(robot.velocity),
                "yaw_speed": robot.yaw_speed,
                "body_height": robot.body_height,
                "range_obstacle": list(robot.range_obstacle),
                "battery_soc": robot.battery_soc,
                "remote": {
                    "present": robot.remote.present,
                    "active": robot.remote.active(self._stick_deadband),
                    "buttons": robot.remote.buttons,
                    "lx": robot.remote.lx, "ly": robot.remote.ly,
                    "rx": robot.remote.rx, "ry": robot.remote.ry, "l2": robot.remote.l2,
                },
            },
        }
        message = String()
        message.data = json.dumps(report, separators=(",", ":"), allow_nan=False, default=str)
        self._robot_state_pub.publish(message)

    def _should_log_periodically(self, now: float, command: MotionCommand, requested: MotionCommand) -> bool:
        if self._last_log_at is not None and now - self._last_log_at < self._log_period:
            return False
        interesting = (
            command.mode != STAND_MODE
            or requested.mode != STAND_MODE
            or self._override.active
            or self._arbiter.blocked_reason is not None
        )
        return interesting

    def _log_line(self, applied: str) -> str:
        with self._lock:
            raw = self._last_requested
        line = f"{applied} | requested vx={raw[0]:.3f} vy={raw[1]:.3f} yaw={raw[2]:.3f}"
        robot = self._robot_state
        if robot is not None:
            line += (
                f" | robot mode={robot.mode} v=({robot.velocity[0]:.2f},{robot.velocity[1]:.2f})"
                f" yaw_rate={robot.yaw_speed:.2f} remote={'ACTIVE' if robot.remote.active(self._stick_deadband) else ('on' if robot.remote.present else 'off')}"
            )
        if self._monitor_execution and self._execution_verdict.state is not ExecutionState.UNKNOWN:
            line += f" | executed {self._execution_verdict.ratio:.0%}"
        return line

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
    except NORMAL_SHUTDOWN_EXCEPTIONS:
        pass
    finally:
        if node is not None:
            node.shutdown_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
