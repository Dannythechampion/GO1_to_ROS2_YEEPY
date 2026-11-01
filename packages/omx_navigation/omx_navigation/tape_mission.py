"""Run a taped mission: Nav2 to each recorded spot, then dock on its tape.

Nav2 covers the distance and stops within `xy_goal_tolerance` (0.35 m); the
dock step then drives the two wall strips' midpoint onto the LiDAR from the
live cloud, so where the robot ends up depends on the tape rather than on
along-corridor localization -- the direction that cannot be observed from wall
geometry and that made the 2026-10-25 drive hunt.

Wiring, all of it through what the stack already gates:

- goals are published on `/goal_pose` for `rviz_goal_bridge`, which remains
  the only way a goal reaches Nav2 and refuses it without READY; the outcome
  comes back on `/navigation/goal_status`.
- dock commands go to `/cmd_vel_nav`, so `cmd_vel_safety_gate` still gates
  them on `/localization_supervisor/ready`. Nothing is published there until
  Nav2's goal has finished and its velocity smoother has run down, so the two
  never command the robot at once.

Start with `ros2 service call /mission/start std_srvs/srv/Trigger`, stop with
`/mission/cancel`. Progress is published as JSON on `/mission/status`.
"""

import json
import math
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from omx_navigation.corridor_dock import corridor_fix, forward_clearance
from omx_navigation.mission_file import default_mission_path, load_mission
from omx_navigation.mission_runner import ActionKind, MissionRunner, MissionState
from omx_navigation.runtime_shutdown import NORMAL_SHUTDOWN_EXCEPTIONS, shutdown_context
from omx_navigation.tape_mission_core import FrameWindow, navigation_outcome, split_frame

STATUS_TOPIC = "/mission/status"
# The robot's own outline about the LiDAR, from the 2026-10-25 mount
# measurement: nose 0.32 m ahead, and the Go1's 0.645 m body puts the tail
# about 0.33 m behind.
NOSE = 0.32
TAIL = 0.33


class TapeMission(Node):
    def __init__(self) -> None:
        super().__init__("tape_mission")
        self._mission_path = self.declare_parameter(
            "mission_file", str(default_mission_path())
        ).value
        lidar_topic = self.declare_parameter("lidar_topic", "/livox/lidar").value
        goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        cancel_service = self.declare_parameter(
            "navigate_cancel_service", "/navigate_to_pose/_action/cancel_goal"
        ).value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        rate = float(self.declare_parameter("control_rate", 10.0).value)
        self._window = FrameWindow(float(self.declare_parameter("accumulate_seconds", 0.2).value))
        # Nav2's velocity smoother keeps publishing on /cmd_vel_nav after the
        # controller stops: its velocity_timeout (0.35 s) runs out, then it
        # decelerates to zero. Dock commands wait that out with room to spare.
        self._dock_start_delay = float(self.declare_parameter("dock_start_delay", 1.5).value)
        self._clearance_margin = float(self.declare_parameter("clearance_margin", 0.10).value)
        self._ready_timeout = float(self.declare_parameter("ready_timeout", 0.5).value)
        # How long the bridge has to acknowledge a goal before the mission
        # assumes nothing is listening.
        self._goal_ack_timeout = float(self.declare_parameter("goal_ack_timeout", 3.0).value)

        self._runner = None
        self._goal = None
        self._goal_sent_at = None
        self._goal_acknowledged = False
        self._dock_entered_at = None
        self._dock_step = None
        self._last_fix = None
        self._last_reason = ""
        self._ready_at = None
        self._manual_override = False
        self._execution_fault = False

        self._goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self._cmd_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self._cancel_client = self.create_client(CancelGoal, cancel_service)

        # The Livox message only exists where the driver is built, so it is
        # imported here rather than at module level, where tests import this.
        from livox_ros_driver2.msg import CustomMsg

        self.create_subscription(CustomMsg, lidar_topic, self._on_lidar, qos_profile_sensor_data)
        self.create_subscription(String, "/navigation/goal_status", self._on_goal_status, 10)
        self.create_subscription(Bool, "/localization_supervisor/ready", self._on_ready, 10)
        self.create_subscription(Bool, "/go1/manual_override", self._on_override, 10)
        self.create_subscription(Bool, "/go1/execution_fault", self._on_fault, 10)
        self.create_service(Trigger, "/mission/start", self._on_start)
        self.create_service(Trigger, "/mission/cancel", self._on_cancel)
        self.create_timer(1.0 / rate, self._on_control)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f"Tape mission ready; mission file {self._mission_path}")

    # -- inputs -----------------------------------------------------------

    def _on_lidar(self, message) -> None:
        if self._runner is None or self._runner.state is not MissionState.DOCKING:
            return
        markers = self._runner.active_dock_policy.markers
        bright, channel = split_frame(
            ((p.x, p.y, p.z, p.reflectivity) for p in message.points),
            markers.reflectivity_threshold, 0.20, -0.15, 0.25,
        )
        self._window.add(self._now(), bright, channel)

    def _on_goal_status(self, message: String) -> None:
        if self._runner is None or self._runner.state is not MissionState.NAVIGATING or self._goal is None:
            return
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(status, dict):
            return
        goal = status.get("goal") or {}
        if (
            isinstance(goal.get("x"), (int, float)) and isinstance(goal.get("y"), (int, float))
            and math.hypot(goal["x"] - self._goal.x, goal["y"] - self._goal.y) < 1e-3
        ):
            self._goal_acknowledged = True
        outcome = navigation_outcome(status, self._goal)
        if outcome is None:
            return
        if not outcome:
            self._last_reason = f"navigation {status.get('state')}: {status.get('reason') or 'no reason given'}"
        self._handle(self._runner.on_navigation_result(outcome, self._now()))

    def _on_ready(self, message: Bool) -> None:
        self._ready_at = self._now() if message.data else None

    def _on_override(self, message: Bool) -> None:
        self._manual_override = bool(message.data)

    def _on_fault(self, message: Bool) -> None:
        self._execution_fault = bool(message.data)

    # -- services ---------------------------------------------------------

    def _on_start(self, _request, response):
        if self._runner is not None and self._runner.state in (
            MissionState.NAVIGATING, MissionState.DOCKING, MissionState.GRASPING
        ):
            response.success = False
            response.message = f"a mission is already running ({self._runner.state.value})"
            return response
        try:
            mission = load_mission(self._mission_path)
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = f"cannot load {self._mission_path}: {exc}"
            return response
        untaped = mission.untaped_docks()
        if untaped:
            response.success = False
            response.message = (
                f"{', '.join(untaped)} never recorded at its tape; stand the robot there and run "
                "`ros2 run omx_navigation tape_waypoint record <name>`"
            )
            return response
        if any(step.kind == "grasp" for step in mission.sequence):
            response.success = False
            response.message = "this node has no arm; remove the grasp step"
            return response
        if not self._localization_ready():
            response.success = False
            response.message = "localization is not READY"
            return response
        self._runner = MissionRunner(mission)
        self._last_reason = ""
        self._dock_step = None
        self._handle(self._runner.start(self._now()))
        response.success = self._runner.state is not MissionState.ABORTED
        response.message = f"started: {self._describe_step()}"
        return response

    def _on_cancel(self, _request, response):
        if self._runner is None or self._runner.state not in (MissionState.NAVIGATING, MissionState.DOCKING):
            response.success = False
            response.message = "no mission is running"
            return response
        self._last_reason = "cancelled by the operator"
        self._abort_now()
        response.success = True
        response.message = "cancelled"
        return response

    # -- control ----------------------------------------------------------

    def _on_control(self) -> None:
        if self._runner is None:
            return
        now = self._now()
        state = self._runner.state
        if state is MissionState.NAVIGATING:
            if (not self._goal_acknowledged and self._goal_sent_at is not None
                    and now - self._goal_sent_at > self._goal_ack_timeout):
                self._last_reason = "rviz_goal_bridge never acknowledged the goal"
                self._abort_now()
            return
        if state is not MissionState.DOCKING:
            return
        if now - self._dock_entered_at < self._dock_start_delay:
            return
        self._window.prune(now)
        policy = self._runner.active_dock_policy
        fix = corridor_fix(self._window.bright(), policy)
        channel = self._window.channel()
        self._last_fix = fix
        action = self._runner.update(
            now,
            fix,
            localization_ready=self._localization_ready(),
            manual_override=self._manual_override,
            execution_fault=self._execution_fault,
            forward_clearance=forward_clearance(channel, ignore_within=NOSE),
            rear_clearance=forward_clearance(channel, ignore_within=TAIL, behind=True),
            minimum_clearance=self._clearance_margin,
        )
        self._handle(action)

    def _handle(self, action) -> None:
        now = self._now()
        if action.kind is ActionKind.NAVIGATE:
            self._send_goal(action.waypoint)
        elif action.kind is ActionKind.DRIVE:
            if self._dock_step != self._runner.step_index:
                # First DRIVE of a dock step carries no velocity: Nav2 has just
                # finished, and its smoother still owns /cmd_vel_nav.
                self._dock_step = self._runner.step_index
                self._goal = None
                self._dock_entered_at = now
                self._last_fix = None
                self._window.clear()
                self.get_logger().info(f"Docking: {self._describe_step()}")
                self._publish_status()
                return
            self._publish_twist(action.vx, action.vy)
        elif action.kind is ActionKind.DONE:
            self._stop_robot()
            self._last_reason = ""
            self.get_logger().info("Mission complete")
            self._publish_status()
        elif action.kind is ActionKind.ABORT:
            self._stop_robot()
            self._last_reason = self._last_reason or action.reason
            self.get_logger().error(f"Mission aborted: {self._last_reason}")
            self._publish_status()
        elif action.kind is ActionKind.GRASP:
            self._last_reason = "grasp step reached, but this node has no arm"
            self._abort_now()

    def _send_goal(self, waypoint) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint.x)
        pose.pose.position.y = float(waypoint.y)
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        self._goal = waypoint
        self._goal_sent_at = self._now()
        self._goal_acknowledged = False
        self._dock_entered_at = None
        self._dock_step = None
        self._goal_publisher.publish(pose)
        self.get_logger().info(
            f"Navigating to {waypoint.name}: x={waypoint.x:.2f} y={waypoint.y:.2f} yaw={waypoint.yaw:.2f}"
        )
        self._publish_status()

    def _abort_now(self) -> None:
        """Stop whatever is moving the robot, whichever controller that is."""
        if self._runner.state is MissionState.NAVIGATING:
            self._cancel_navigation()
        self._handle(self._runner.abort(self._last_reason))

    def _cancel_navigation(self) -> None:
        # The goal was sent by the bridge, so there is no handle here. An empty
        # CancelGoal request (zero id, zero stamp) cancels every goal on the
        # action server, which is the right scope: nothing else should be
        # driving while a mission is being cancelled.
        if not self._cancel_client.service_is_ready():
            self.get_logger().error("Nav2 cancel service unavailable; stop the robot with the remote")
            return
        self._cancel_client.call_async(CancelGoal.Request())

    def _stop_robot(self) -> None:
        if self._dock_entered_at is not None:
            self._publish_twist(0.0, 0.0)
        self._dock_entered_at = None
        self._goal = None

    def _publish_twist(self, vx: float, vy: float) -> None:
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        self._cmd_publisher.publish(twist)

    # -- reporting --------------------------------------------------------

    def _describe_step(self) -> str:
        if self._runner is None:
            return ""
        index = self._runner.step_index
        sequence = self._runner.mission.sequence
        if index >= len(sequence):
            return "done"
        step = sequence[index]
        if step.kind == "navigate":
            return f"navigate to {step.waypoint}"
        if step.kind == "dock":
            return f"dock at {self._runner.mission.dock_waypoint(index).name}"
        return step.kind

    def _publish_status(self) -> None:
        if self._runner is None:
            status = {"state": MissionState.IDLE.value, "step": None, "reason": "", "stamp": self._now()}
        else:
            fix = self._last_fix if self._runner.state is MissionState.DOCKING else None
            status = {
                "state": self._runner.state.value,
                "step": self._runner.step_index,
                "doing": self._describe_step(),
                "reason": self._last_reason,
                "fix": None if fix is None else {
                    "mid_x": round(fix.mid_x, 3), "mid_y": round(fix.mid_y, 3),
                    "yaw_error": round(fix.yaw_error, 3), "width": round(fix.width, 3),
                    "points": fix.points,
                },
                "stamp": self._now(),
            }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _localization_ready(self) -> bool:
        return self._ready_at is not None and self._now() - self._ready_at <= self._ready_timeout

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def stop(self) -> None:
        if self._runner is not None and self._runner.state is MissionState.DOCKING:
            self._publish_twist(0.0, 0.0)
            # Give the zero command a moment to leave before the node goes.
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TapeMission()
    try:
        rclpy.spin(node)
    except NORMAL_SHUTDOWN_EXCEPTIONS:
        pass
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            shutdown_context(rclpy)


if __name__ == "__main__":
    main()
