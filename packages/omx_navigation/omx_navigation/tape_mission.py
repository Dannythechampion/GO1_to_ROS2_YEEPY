"""Run a taped mission: Nav2 toward each spot, then stop on its tape.

Nav2 covers the distance; `TapeDock` stops on the two wall strips from the
live cloud, so where the robot ends up depends on the tape rather than on
along-corridor localization -- the direction that cannot be observed from wall
geometry and that made the 2026-10-25 drive hunt. Nav2 is not waited on to
the end: once the spot's tape is in view near the goal, the goal is cancelled
and the dock takes over (`HandoffWatch`), so the goal checker's in-and-out
near the goal can no longer keep the robot moving.

Services (std_srvs/Trigger):

- `/mission/record_start` -- the robot stands at the start tape: look for
  1.5 s standing and record the spot.
- `/mission/teach_goal` -- then click 2D Goal Pose near the goal tape in RViz.
  The robot navigates there, hands over to the goal tape, stops on it, and
  records the spot. No goal coordinates have to be known beforehand.
- `/mission/start` -- run the recorded mission (goal, dock, start, dock).
- `/mission/cancel` -- stop whatever is running.

Wiring, all through what the stack already gates: goals go to `/goal_pose`
for `rviz_goal_bridge` (the only way into Nav2, READY-gated) and outcomes come
back on `/navigation/goal_status`; dock commands go to `/cmd_vel_nav` behind
`cmd_vel_safety_gate`.

Every control cycle is written to `decisions.jsonl` in the session directory
(`log_dir`, default `$GO1_SESSION_DIR`) and published on `/mission/decision`:
what the robot saw, what each rule concluded, and what was commanded. The
file survives the process being killed; see `DecisionLog`.
"""

import json
import math
import os
import time
from datetime import datetime, timezone

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from omx_navigation.corridor_dock import (
    corridor_fix,
    forward_clearance,
    median_fix,
    tape_pose,
    unmeasured_corridor,
)
from omx_navigation.mission import Mission, Step, Waypoint, record_waypoint
from omx_navigation.mission_file import default_mission_path, load_mission, save_mission
from omx_navigation.mission_runner import ActionKind, MissionRunner, MissionState
from omx_navigation.runtime_shutdown import NORMAL_SHUTDOWN_EXCEPTIONS, shutdown_context
from omx_navigation.livox_cdr import points_view, split_points
from omx_navigation.tape_mission_core import (
    DecisionLog,
    FrameWindow,
    HandoffWatch,
    navigation_outcome,
    tape_near_goal,
)

STATUS_TOPIC = "/mission/status"
DECISION_TOPIC = "/mission/decision"
PACKAGE = "omx_navigation"
# The robot's own outline about the LiDAR, from the 2026-10-25 mount
# measurement: nose 0.32 m ahead, and the Go1's 0.645 m body puts the tail
# about 0.33 m behind.
NOSE = 0.32
TAIL = 0.33
ACTIVE = (MissionState.NAVIGATING, MissionState.DOCKING, MissionState.GRASPING)


def default_log_dir() -> str:
    explicit = os.environ.get("GO1_SESSION_DIR")
    if explicit:
        return explicit
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    root = "/mnt/t500/go1_field_logs" if os.path.isdir("/mnt/t500") else os.path.expanduser("~/.ros/go1_field_logs")
    return os.path.join(root, f"tape_{stamp}")


class TapeMission(Node):
    def __init__(self) -> None:
        super().__init__("tape_mission")
        self._mission_path = self.declare_parameter("mission_file", str(default_mission_path())).value
        lidar_topic = self.declare_parameter("lidar_topic", "/livox/lidar").value
        goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        cancel_service = self.declare_parameter(
            "navigate_cancel_service", "/navigate_to_pose/_action/cancel_goal"
        ).value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        self._base_frame = self.declare_parameter("base_frame", "body_nav").value
        self._teach_name = self.declare_parameter("teach_waypoint", "goal").value
        rate = float(self.declare_parameter("control_rate", 10.0).value)
        self._window = FrameWindow(float(self.declare_parameter("accumulate_seconds", 0.2).value))
        self._handoff = HandoffWatch(
            float(self.declare_parameter("handoff_radius", 1.0).value),
            int(self.declare_parameter("handoff_frames", 3).value),
        )
        self._record_seconds = float(self.declare_parameter("record_seconds", 1.5).value)
        self._clearance_margin = float(self.declare_parameter("clearance_margin", 0.10).value)
        self._ready_timeout = float(self.declare_parameter("ready_timeout", 0.5).value)
        # How long the bridge has to acknowledge a goal before the mission
        # assumes nothing is listening.
        self._goal_ack_timeout = float(self.declare_parameter("goal_ack_timeout", 3.0).value)
        log_dir = self.declare_parameter("log_dir", default_log_dir()).value
        self._log = DecisionLog(os.path.join(log_dir, "decisions.jsonl"))

        self._runner = None
        self._mode = None            # None, "mission", "teach_wait", "teach"
        self._recording = None       # {"name", "until", "fixes", "then"}
        self._goal = None
        self._goal_sent_at = None
        self._goal_acknowledged = False
        self._goal_published_by_us = False
        self._dock_step = None
        self._last_fix = None
        self._last_reason = ""
        self._last_nav_state = None
        self._ready_at = None
        self._manual_override = False
        self._execution_fault = False

        self._goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self._cmd_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self._decision_publisher = self.create_publisher(String, DECISION_TOPIC, 10)
        self._cancel_client = self.create_client(CancelGoal, cancel_service)

        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # The Livox message only exists where the driver is built, so it is
        # imported here rather than at module level, where tests import this.
        from livox_ros_driver2.msg import CustomMsg

        # Raw bytes: rclpy would build ~20,000 point objects per frame (130 ms
        # each on the Jetson); the numpy view takes ~1.5 ms. See livox_cdr.
        self.create_subscription(CustomMsg, lidar_topic, self._on_lidar, qos_profile_sensor_data, raw=True)
        self.create_subscription(String, "/navigation/goal_status", self._on_goal_status, 10)
        self.create_subscription(Bool, "/localization_supervisor/ready", self._on_ready, 10)
        self.create_subscription(Bool, "/go1/manual_override", self._on_override, 10)
        self.create_subscription(Bool, "/go1/execution_fault", self._on_fault, 10)
        self.create_service(Trigger, "/mission/start", self._on_start)
        self.create_service(Trigger, "/mission/record_start", self._on_record_start)
        self.create_service(Trigger, "/mission/teach_goal", self._on_teach_goal)
        self.create_service(Trigger, "/mission/cancel", self._on_cancel)
        self.create_timer(1.0 / rate, self._on_control)
        self.create_timer(1.0, self._publish_status)
        self._event("node_started", mission_file=self._mission_path, log=self._log.path)
        self.get_logger().info(f"Tape mission ready; mission file {self._mission_path}; log {self._log.path}")

    # -- inputs -----------------------------------------------------------

    def _watching_tape(self) -> bool:
        if self._recording is not None:
            return True
        return self._runner is not None and (
            self._runner.state is MissionState.DOCKING or self._runner.docks_next
        )

    def _detector_policy(self):
        if self._recording is not None or self._runner is None:
            return unmeasured_corridor()
        return self._runner.active_dock_policy

    def _on_lidar(self, message: bytes) -> None:
        if not self._watching_tape():
            return
        try:
            points = points_view(message)
        except ValueError as exc:
            self.get_logger().warning(f"unreadable Livox frame: {exc}")
            return
        bright, channel = split_points(
            points, self._detector_policy().markers.reflectivity_threshold, 0.20, -0.15, 0.25,
        )
        self._window.add(self._now(), bright, channel)

    def _on_goal_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(status, dict):
            return
        goal = status.get("goal") or {}
        state = status.get("state")
        if self._mode == "teach_wait":
            if state in ("SENT", "ACCEPTED") and isinstance(goal.get("x"), (int, float)):
                self._begin_teach(goal)
            return
        if self._runner is None or self._runner.state is not MissionState.NAVIGATING or self._goal is None:
            return
        if (
            isinstance(goal.get("x"), (int, float)) and isinstance(goal.get("y"), (int, float))
            and math.hypot(goal["x"] - self._goal.x, goal["y"] - self._goal.y) < 1e-3
        ):
            self._goal_acknowledged = True
            if state != self._last_nav_state:
                self._last_nav_state = state
                self._event("nav_status", state=state, reason=status.get("reason"),
                            distance_remaining=status.get("distance_remaining"))
        outcome = navigation_outcome(status, self._goal)
        if outcome is None:
            return
        policy = self._runner.active_dock_policy
        pose = self._robot_pose()
        target = self._runner.navigation_target
        distance = None if pose is None else math.hypot(pose[0] - target.x, pose[1] - target.y)
        in_view = tape_near_goal(distance, self._last_fix, policy, self._handoff.radius)
        if not outcome:
            self._event("nav_failed", state=state, reason=status.get("reason"), distance_to_goal=distance,
                        tape_near_goal=in_view)
            if not in_view:
                self._last_reason = f"navigation {state}: {status.get('reason') or 'no reason given'}"
        self._handle(self._runner.on_navigation_result(outcome, self._now(), tape_in_view=in_view))

    def _on_ready(self, message: Bool) -> None:
        self._ready_at = self._now() if message.data else None

    def _on_override(self, message: Bool) -> None:
        self._manual_override = bool(message.data)

    def _on_fault(self, message: Bool) -> None:
        self._execution_fault = bool(message.data)

    # -- services ---------------------------------------------------------

    def _busy(self) -> str:
        if self._recording is not None:
            return f"recording {self._recording['name']}"
        if self._mode == "teach_wait":
            return "waiting for a 2D Goal Pose to teach"
        if self._runner is not None and self._runner.state in ACTIVE:
            return f"a mission is running ({self._runner.state.value})"
        return ""

    def _on_record_start(self, _request, response):
        busy = self._busy()
        if busy or not self._localization_ready():
            response.success = False
            response.message = busy or "localization is not READY; push 2D Pose Estimate first"
            return response
        self._begin_recording("start", then=None)
        response.success = True
        response.message = f"looking at the start tape for {self._record_seconds:.1f} s; watch /mission/status"
        return response

    def _on_teach_goal(self, _request, response):
        busy = self._busy()
        if busy or not self._localization_ready():
            response.success = False
            response.message = busy or "localization is not READY"
            return response
        self._mode = "teach_wait"
        self._last_reason = ""
        self._event("teach_armed", waypoint=self._teach_name)
        response.success = True
        response.message = f"click 2D Goal Pose near the {self._teach_name} tape in RViz"
        self._publish_status()
        return response

    def _on_start(self, _request, response):
        busy = self._busy()
        if busy:
            response.success = False
            response.message = busy
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
                f"{', '.join(untaped)} never recorded at its tape; use /mission/record_start "
                "and /mission/teach_goal first"
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
        self._mode = "mission"
        self._runner = MissionRunner(mission)
        self._reset_run()
        self._event("mission_started", sequence=[s.kind + (":" + s.waypoint if s.waypoint else "")
                                                 for s in mission.sequence])
        self._handle(self._runner.start(self._now()))
        response.success = self._runner.state is not MissionState.ABORTED
        response.message = f"started: {self._describe_step()}"
        return response

    def _on_cancel(self, _request, response):
        if self._mode == "teach_wait" or self._recording is not None:
            self._mode = None
            self._recording = None
            self._last_reason = "cancelled by the operator"
            self._event("cancelled")
            response.success = True
            response.message = "cancelled"
            return response
        if self._runner is None or self._runner.state not in ACTIVE:
            response.success = False
            response.message = "nothing is running"
            return response
        self._last_reason = "cancelled by the operator"
        self._abort_now()
        response.success = True
        response.message = "cancelled"
        return response

    # -- teach and record -------------------------------------------------

    def _begin_teach(self, goal: dict) -> None:
        yaw = goal.get("yaw") if isinstance(goal.get("yaw"), (int, float)) else 0.0
        target = Waypoint(self._teach_name, float(goal["x"]), float(goal["y"]), float(yaw))
        mission = Mission({target.name: target}, (Step("navigate", target.name), Step("dock")))
        # The spot's wall gap is what this run measures, so the detector
        # cannot be held to the 2.00 m default.
        self._runner = MissionRunner(mission, dock_policy=unmeasured_corridor())
        self._mode = "teach"
        self._reset_run()
        self._event("teach_started", goal=target.as_dict())
        action = self._runner.start(self._now())
        # The operator's click already went to Nav2 through the bridge.
        self._track_goal(action.waypoint, published=False)

    def _begin_recording(self, name: str, then) -> None:
        self._window.clear()
        self._recording = {"name": name, "until": self._now() + self._record_seconds, "fixes": [], "then": then}
        self._event("recording_started", waypoint=name)
        self._publish_status()

    def _step_recording(self, now: float) -> None:
        recording = self._recording
        self._window.prune(now)
        fix = corridor_fix(self._window.bright(), unmeasured_corridor())
        self._last_fix = fix
        if fix is not None:
            recording["fixes"].append(fix)
        self._decision({"phase": "RECORD", "waypoint": recording["name"], "fixes": len(recording["fixes"])}, fix)
        if now < recording["until"]:
            return
        self._recording = None
        name = recording["name"]
        fix = median_fix(recording["fixes"])
        pose = self._robot_pose()
        if fix is None or len(recording["fixes"]) < 3:
            self._last_reason = f"{name}: tape pair not seen while recording; run tape_waypoint check"
            self._event("record_failed", waypoint=name, reason=self._last_reason)
            self.get_logger().error(self._last_reason)
        elif pose is None:
            self._last_reason = f"{name}: no {self._map_frame} -> {self._base_frame} transform"
            self._event("record_failed", waypoint=name, reason=self._last_reason)
            self.get_logger().error(self._last_reason)
        else:
            x, y, yaw = tape_pose(*pose, fix)
            mission = self._load_or_seed()
            mission = record_waypoint(mission, name, round(x, 4) + 0.0, round(y, 4) + 0.0,
                                      round(yaw, 4) + 0.0, round(fix.width, 4))
            save_mission(self._mission_path, mission)
            self._last_reason = (f"recorded {name}: x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f} deg, "
                                 f"walls {fix.width:.3f} m apart, robot {math.hypot(fix.mid_x, fix.mid_y):.3f} m "
                                 "off the tape")
            self._event("recorded", waypoint=name, x=x, y=y, yaw=yaw, wall_gap=fix.width,
                        robot_pose=pose, fix=_fix_dict(fix), samples=len(recording["fixes"]),
                        untaped=mission.untaped_docks())
            self.get_logger().info(self._last_reason)
        if self._mode == "teach":
            self._mode = None
        self._publish_status()

    def _load_or_seed(self):
        if os.path.exists(self._mission_path):
            return load_mission(self._mission_path)
        from ament_index_python.packages import get_package_share_directory

        return load_mission(os.path.join(get_package_share_directory(PACKAGE), "config", "tape_mission.yaml"))

    # -- control ----------------------------------------------------------

    def _on_control(self) -> None:
        now = self._now()
        if self._recording is not None:
            self._step_recording(now)
            return
        if self._runner is None:
            return
        state = self._runner.state
        if state is MissionState.NAVIGATING:
            self._step_navigation(now)
        elif state is MissionState.DOCKING:
            self._step_dock(now)

    def _step_navigation(self, now: float) -> None:
        if (not self._goal_acknowledged and self._goal_sent_at is not None
                and now - self._goal_sent_at > self._goal_ack_timeout):
            self._last_reason = "rviz_goal_bridge never acknowledged the goal"
            self._abort_now()
            return
        if not self._runner.docks_next:
            return
        self._window.prune(now)
        policy = self._runner.active_dock_policy
        fix = corridor_fix(self._window.bright(), policy)
        self._last_fix = fix
        pose = self._robot_pose()
        target = self._runner.navigation_target
        distance = None if pose is None else math.hypot(pose[0] - target.x, pose[1] - target.y)
        hand_over = self._handoff.update(distance, fix, policy)
        self._decision({"phase": "NAVIGATE", "distance_to_goal": distance, "handoff_count": self._handoff.count,
                        "handoff_radius": self._handoff.radius, "nav_state": self._last_nav_state}, fix, pose)
        if hand_over:
            self._event("handoff", distance_to_goal=distance, fix=_fix_dict(fix), robot_pose=pose)
            self._cancel_navigation()
            self._handle(self._runner.on_tape_handoff(now))

    def _step_dock(self, now: float) -> None:
        self._window.prune(now)
        policy = self._runner.active_dock_policy
        fix = corridor_fix(self._window.bright(), policy)
        channel = self._window.channel()
        self._last_fix = fix
        front = forward_clearance(channel, ignore_within=NOSE)
        rear = forward_clearance(channel, ignore_within=TAIL, behind=True)
        action = self._runner.update(
            now, fix,
            localization_ready=self._localization_ready(),
            manual_override=self._manual_override,
            execution_fault=self._execution_fault,
            forward_clearance=front,
            rear_clearance=rear,
            minimum_clearance=self._clearance_margin,
        )
        self._decision(dict(action.detail, front_clearance=_finite(front), rear_clearance=_finite(rear),
                            vx=action.vx, vy=action.vy, frames=self._window.frames,
                            bright=len(self._window.bright())), fix)
        if action.kind in (ActionKind.NAVIGATE, ActionKind.DONE, ActionKind.GRASP):
            self._event("dock_arrived", why=action.reason, rounds=self._runner.last_dock_rounds,
                        fix=_fix_dict(fix), robot_pose=self._robot_pose())
        self._handle(action)

    def _handle(self, action) -> None:
        now = self._now()
        if action.kind is ActionKind.NAVIGATE:
            self._track_goal(action.waypoint, published=True)
        elif action.kind is ActionKind.DRIVE:
            if self._dock_step != self._runner.step_index:
                self._dock_step = self._runner.step_index
                self._goal = None
                self._window.clear()
                self._event("dock_started", spot=self._describe_step(), why=action.reason,
                            wall_gap=self._runner.active_dock_policy.markers.strip_separation)
                self.get_logger().info(f"Docking: {self._describe_step()} ({action.reason})")
                self._publish_status()
            # Zero included: while settling and measuring the dock holds the
            # robot standing, and says so every cycle.
            self._publish_twist(action.vx, action.vy)
        elif action.kind is ActionKind.DONE:
            self._publish_twist(0.0, 0.0)
            self._dock_step = None
            self._event("mission_done", why=action.reason)
            if self._mode == "teach":
                self._begin_recording(self._teach_name, then=None)
            else:
                self._mode = None
                self._last_reason = action.reason
                self.get_logger().info(f"Mission complete: {action.reason}")
            self._publish_status()
        elif action.kind is ActionKind.ABORT:
            self._publish_twist(0.0, 0.0)
            self._dock_step = None
            self._mode = None
            self._last_reason = self._last_reason or action.reason
            self._event("aborted", reason=self._last_reason, detail=action.detail)
            self.get_logger().error(f"Mission aborted: {self._last_reason}")
            self._publish_status()
        elif action.kind is ActionKind.GRASP:
            self._last_reason = "grasp step reached, but this node has no arm"
            self._abort_now()

    def _track_goal(self, waypoint, published: bool) -> None:
        self._goal = waypoint
        self._goal_sent_at = self._now()
        self._goal_acknowledged = not published
        self._goal_published_by_us = published
        self._last_nav_state = None
        self._dock_step = None
        self._handoff.reset()
        self._window.clear()
        if published:
            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(waypoint.x)
            pose.pose.position.y = float(waypoint.y)
            pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
            pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
            self._goal_publisher.publish(pose)
        self._event("navigate", waypoint=waypoint.name, goal=waypoint.as_dict(), published=published,
                    robot_pose=self._robot_pose())
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
        # The goal went through the bridge, so there is no handle here. An
        # empty CancelGoal (zero id, zero stamp) cancels every goal on the
        # server, which is the right scope: nothing else should be driving.
        if not self._cancel_client.service_is_ready():
            self.get_logger().error("Nav2 cancel service unavailable; stop the robot with the remote")
            self._event("cancel_unavailable")
            return
        self._cancel_client.call_async(CancelGoal.Request())
        self._event("nav_cancel_sent")

    def _publish_twist(self, vx: float, vy: float) -> None:
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        self._cmd_publisher.publish(twist)

    def _reset_run(self) -> None:
        self._last_reason = ""
        self._dock_step = None
        self._last_fix = None
        self._handoff.reset()
        self._window.clear()

    # -- reporting --------------------------------------------------------

    def _decision(self, detail: dict, fix, pose="lookup") -> None:
        if pose == "lookup":
            pose = self._robot_pose()
        record = {
            "t": round(self._now(), 3),
            "kind": "cycle",
            "mode": self._mode,
            "state": None if self._runner is None else self._runner.state.value,
            "step": None if self._runner is None else self._runner.step_index,
            "doing": self._describe_step(),
            "robot": None if pose is None else [round(v, 4) for v in pose],
            "fix": _fix_dict(fix),
            "ready": self._localization_ready(),
            "override": self._manual_override,
            "fault": self._execution_fault,
            **{key: (round(value, 4) if isinstance(value, float) else value) for key, value in detail.items()},
        }
        self._log.write(record)
        self._decision_publisher.publish(String(data=json.dumps(record, separators=(",", ":"))))

    def _event(self, name: str, **fields) -> None:
        record = {"t": round(self._now(), 3), "kind": "event", "event": name,
                  "mode": self._mode,
                  "state": None if self._runner is None else self._runner.state.value, **fields}
        self._log.write(record, event=True)
        self._decision_publisher.publish(String(data=json.dumps(record, separators=(",", ":"), default=str)))

    def _describe_step(self) -> str:
        if self._recording is not None:
            return f"record {self._recording['name']}"
        if self._mode == "teach_wait":
            return "waiting for 2D Goal Pose"
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
        fix = self._last_fix
        status = {
            "state": "RECORDING" if self._recording is not None
            else "TEACH_WAIT" if self._mode == "teach_wait"
            else MissionState.IDLE.value if self._runner is None else self._runner.state.value,
            "mode": self._mode,
            "step": None if self._runner is None else self._runner.step_index,
            "doing": self._describe_step(),
            "dock_phase": None if self._runner is None or self._runner.state is not MissionState.DOCKING
            else self._runner.dock_phase.value,
            "reason": self._last_reason,
            "fix": _fix_dict(fix),
            "log": self._log.path,
            "stamp": self._now(),
        }
        self._status_publisher.publish(String(data=json.dumps(status, ensure_ascii=False, separators=(",", ":"))))

    def _robot_pose(self):
        try:
            if not self._tf_buffer.can_transform(self._map_frame, self._base_frame, Time()):
                return None
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time()).transform
        except Exception:  # tf2 raises several unrelated exception types
            return None
        q = transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.translation.x, transform.translation.y, yaw

    def _localization_ready(self) -> bool:
        return self._ready_at is not None and self._now() - self._ready_at <= self._ready_timeout

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def stop(self) -> None:
        try:
            if self._runner is not None and self._runner.state is MissionState.DOCKING:
                self._publish_twist(0.0, 0.0)
                # Give the zero command a moment to leave before the node goes.
                time.sleep(0.05)
            self._event("node_stopping")
        finally:
            self._log.close()


def _fix_dict(fix):
    if fix is None:
        return None
    return {"mid_x": round(fix.mid_x, 4), "mid_y": round(fix.mid_y, 4),
            "yaw_error": round(fix.yaw_error, 4), "width": round(fix.width, 4), "points": fix.points}


def _finite(value):
    return value if math.isfinite(value) else None


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
        except Exception:  # the context may already be gone after SIGINT
            pass
        node.destroy_node()
        shutdown_context(rclpy)


if __name__ == "__main__":
    main()
