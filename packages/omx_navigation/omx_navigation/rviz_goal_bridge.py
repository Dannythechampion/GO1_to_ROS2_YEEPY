"""Forward RViz's /goal_pose topic to Nav2's NavigateToPose action.

This node must be the only way a goal reaches Nav2. Humble's bt_navigator also
subscribes to `goal_pose` on its own; until 2026-09-18 the launch left that in
place, so every click reached Nav2 twice, the readiness gate here stopped
nothing, and the second click of a pair made this node lose the goal it was
tracking -- "Navigation goal reached" never appeared in an 18-minute run. The
launch files now remap bt_navigator's subscription away.

Everything the operator needs to see is published, not just logged:
`/navigation/goal_status` (JSON) on every transition and a marker at the goal
coloured by state, so a click that was ignored, a goal that was preempted and
a goal that arrived are all visible in RViz.

With `final_alignment` on, Nav2 only brings the robot near: once the tracked
pose is within `alignment.handoff_radius` of the goal (or Nav2 reports success
first) the Nav2 goal is cancelled and `final_alignment.FinalAligner` turns,
shifts and confirms the arrival with the localization supervisor's scan
matches. Its commands go to `alignment_command_topic`, which the velocity
safety gate prefers over Nav2's, so readiness and every inhibitor still apply.
A goal is SUCCEEDED only after that confirmation; it reports ALIGNING meanwhile.
"""

from dataclasses import fields
import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from omx_navigation.final_alignment import AlignmentPhase, AlignmentPolicy, FinalAligner, ScanCheck
from omx_navigation.goal_gate import GoalGate
from omx_navigation.ros_conversions import quaternion_to_yaw
from omx_navigation.runtime_shutdown import (
    NORMAL_SHUTDOWN_EXCEPTIONS,
    shutdown_context,
)
from omx_navigation.scan_map_quality import Pose2D

STATUS_TOPIC = "/navigation/goal_status"
MARKER_TOPIC = "/navigation/goal_marker"
LOCALIZATION_STATUS_TOPIC = "/localization_supervisor/status"
ALIGNMENT_PERIOD = 0.10
INHIBIT_TOPICS = {
    "manual override": "/go1/manual_override",
    "robot not following commands": "/go1/execution_fault",
}
_COLOURS = {
    "SENT": (1.0, 0.8, 0.0),
    "ACCEPTED": (1.0, 0.8, 0.0),
    "ALIGNING": (1.0, 0.8, 0.0),
    "SUCCEEDED": (0.1, 0.9, 0.2),
    "PREEMPTED": (0.6, 0.6, 0.6),
}
_FAILED = (0.95, 0.2, 0.2)
_RESULT_STATES = {
    getattr(GoalStatus, "STATUS_SUCCEEDED", 4): "SUCCEEDED",
    getattr(GoalStatus, "STATUS_CANCELED", 5): "CANCELED",
    getattr(GoalStatus, "STATUS_ABORTED", 6): "ABORTED",
}


def _goal_summary(pose) -> dict:
    header = getattr(pose, "header", None)
    position = getattr(getattr(pose, "pose", None), "position", None)
    orientation = getattr(getattr(pose, "pose", None), "orientation", None)
    summary = {"frame": getattr(header, "frame_id", "") or ""}
    for axis in ("x", "y"):
        value = getattr(position, axis, None)
        summary[axis] = float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None
    try:
        summary["yaw"] = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
    except (AttributeError, TypeError):
        summary["yaw"] = None
    if summary["yaw"] is not None and not math.isfinite(summary["yaw"]):
        summary["yaw"] = None
    return summary


def _alignment_summary(step):
    if step is None:
        return None

    def finite(value):
        return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None

    error_yaw = finite(step.error_yaw)
    return {
        "phase": step.phase.value,
        "error_xy": finite(step.error_xy),
        "error_yaw_deg": None if error_yaw is None else math.degrees(error_yaw),
    }


class RvizGoalBridge(Node):
    """Let the stock RViz '2D Goal Pose' tool control Nav2."""

    def __init__(self) -> None:
        super().__init__("rviz_goal_bridge")

        goal_topic = self.declare_parameter("goal_topic", "goal_pose").value
        action_name = self.declare_parameter(
            "navigate_to_pose_action", "navigate_to_pose"
        ).value
        ready_timeout_value = self.declare_parameter("ready_timeout", 0.30).value
        if isinstance(ready_timeout_value, bool):
            raise ValueError("ready_timeout must be a positive finite number")
        self._ready_timeout = float(ready_timeout_value)
        if not math.isfinite(self._ready_timeout) or self._ready_timeout <= 0.0:
            raise ValueError("ready_timeout must be a positive finite number")
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._goal_gate = GoalGate()
        self._active_goal_handle = None
        self._active_token = None
        self._pending_token = None
        self._pending_authority_generation = None
        self._goals = {}
        self._goal_token = 0
        # Bumped whenever the right to drive is withdrawn (localization lost,
        # remote taken, robot refusing); a goal requested before the bump is
        # cancelled the moment Nav2 accepts it.
        self._authority_generation = 0
        self._last_ready_at = None
        self._cancel_in_flight = False
        self._cancel_retry_needed = False
        self._cancel_reason = None
        self._last_feedback_log_ns = 0
        self._last_feedback_publish_ns = 0
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self._marker_publisher = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self._goal_subscription = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal_pose, 10
        )
        self._ready_subscription = self.create_subscription(
            Bool, "/localization_supervisor/ready", self._on_ready, 10
        )
        self._inhibit_subscriptions = [
            self.create_subscription(
                Bool, topic, lambda message, name=name: self._on_inhibit(name, message), 10
            )
            for name, topic in INHIBIT_TOPICS.items()
        ]
        self._watchdog_timer = self.create_timer(
            min(0.10, self._ready_timeout / 2.0), self._on_ready_watchdog
        )

        self._alignment_enabled = bool(self.declare_parameter("final_alignment", False).value)
        self._alignment_policy = AlignmentPolicy(**{
            item.name: self.declare_parameter(f"alignment.{item.name}", item.default).value
            for item in fields(AlignmentPolicy)
        })
        self._global_frame = self.declare_parameter("global_frame", "map").value
        self._base_frame = self.declare_parameter("base_frame", "body_nav").value
        alignment_topic = self.declare_parameter("alignment_command_topic", "/cmd_vel_align").value
        self._aligner = None
        self._aligning_token = None
        self._alignment_phase = None
        self._last_alignment_report_ns = 0
        # The goal whose Nav2 run was cancelled to hand over, and the authority
        # generation it was handed over under.
        self._handoff_token = None
        self._handoff_generation = None
        self._scan_check = None
        if self._alignment_enabled:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._alignment_publisher = self.create_publisher(Twist, alignment_topic, 10)
            self._localization_status_subscription = self.create_subscription(
                String, LOCALIZATION_STATUS_TOPIC, self._on_localization_status, 10
            )
            self._alignment_timer = self.create_timer(ALIGNMENT_PERIOD, self._on_alignment_timer)
        self.get_logger().info(
            f"Forwarding '{goal_topic}' goals to '{action_name}'"
            + (" with final alignment" if self._alignment_enabled else "")
        )

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        self._expire_ready_if_stale()
        if not self._goal_gate.accept_goal():
            reason = self._goal_gate.refusal_reason()
            self.get_logger().warning(f"Ignoring navigation goal: {reason}")
            self._report("IGNORED", pose, reason=reason)
            return
        if not pose.header.frame_id:
            self.get_logger().error("Ignoring goal without a frame_id")
            self._report("IGNORED", pose, reason="goal has no frame_id")
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error(
                "Nav2 NavigateToPose action is not available yet; try the goal again"
            )
            self._report("IGNORED", pose, reason="Nav2 is not ready")
            return

        self._stop_alignment("PREEMPTED", "a newer goal was sent")
        goal = NavigateToPose.Goal()
        goal.pose = pose
        replacing = self._pending_token is not None or self._active_goal_handle is not None
        self.get_logger().info(
            "%s goal in %s: x=%.2f, y=%.2f"
            % ("Replacing the current goal with a new" if replacing else "Sending", pose.header.frame_id,
               pose.pose.position.x, pose.pose.position.y)
        )
        self._goal_token += 1
        token = self._goal_token
        self._goals[token] = pose
        for stale in [old for old in self._goals if old < token - 16]:
            del self._goals[stale]
        superseded = self._pending_token
        self._pending_token = token
        if superseded is not None:
            self._report("PREEMPTED", self._goals.get(superseded), token=superseded, reason="a newer goal was sent")
        authority_generation = self._authority_generation
        self._pending_authority_generation = authority_generation
        self._report("SENT", pose, token=token)
        future = self._action_client.send_goal_async(
            goal, feedback_callback=lambda feedback: self._on_feedback(feedback, token)
        )
        future.add_done_callback(
            lambda response: self._on_goal_response(response, token, authority_generation)
        )

    def _on_ready(self, message: Bool) -> None:
        if message.data:
            self._last_ready_at = self._now()
            self._goal_gate.update_ready(True)
        else:
            self._revoke_readiness()

    def _on_inhibit(self, name: str, message: Bool) -> None:
        if self._goal_gate.update_inhibit(name, bool(message.data)):
            self.get_logger().warning(f"Cancelling navigation: {name}")
            self._withdraw_authority(name)

    def _on_goal_response(self, future, token, authority_generation=None) -> None:
        error = "no goal handle"
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy reports transport failures through the future
            goal_handle = None
            error = exc
        if token != self._pending_token:
            # Superseded while in flight. If Nav2 took it anyway, it is not the
            # goal anyone wants now; the newer request preempts it, and an
            # explicit cancel covers a newer request that Nav2 rejects.
            if goal_handle is not None and getattr(goal_handle, "accepted", False):
                self._cancel_obsolete(goal_handle)
            return
        self._pending_token = None
        if authority_generation is None:
            authority_generation = self._pending_authority_generation
        self._pending_authority_generation = None
        pose = self._goals.get(token)
        if goal_handle is None:
            self.get_logger().error(f"Failed to send navigation goal: {error}")
            self._report("REJECTED", pose, token=token, reason=f"send failed: {error}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected the navigation goal")
            self._report("REJECTED", pose, token=token, reason="Nav2 rejected the goal")
            return

        self.get_logger().info("Navigation goal accepted")
        previous_token = self._active_token
        if self._active_goal_handle is not None and previous_token is not None:
            # Nav2 preempts the old goal itself; its result arrives later and is
            # recognised as stale by token.
            self._report("PREEMPTED", self._goals.get(previous_token), token=previous_token,
                         reason="replaced by a newer goal")
        self._active_goal_handle = goal_handle
        self._active_token = token
        self._cancel_in_flight = False
        self._cancel_retry_needed = False
        self._goal_gate.set_goal_active(True)
        self._report("ACCEPTED", pose, token=token)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_result(result, goal_handle, token))
        # Any withdrawal after the request was created permanently invalidates
        # this goal, even if the right to drive recovers before Nav2 accepts.
        self._expire_ready_if_stale()
        if authority_generation != self._authority_generation or not self._goal_gate.accept_goal():
            self._cancel_active_goal(goal_handle, token)

    def _on_ready_watchdog(self) -> None:
        self._expire_ready_if_stale()
        if self._cancel_retry_needed and not self._cancel_in_flight:
            self._cancel_active_goal()

    def _on_feedback(self, feedback_msg, token=None) -> None:
        if token is not None and token != self._active_token:
            return
        distance = feedback_msg.feedback.distance_remaining
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_publish_ns >= 1_000_000_000:
            self._last_feedback_publish_ns = now_ns
            self._report("ACCEPTED", self._goals.get(self._active_token), token=self._active_token,
                         distance_remaining=distance)
        if now_ns - self._last_feedback_log_ns < 5_000_000_000:
            return
        self._last_feedback_log_ns = now_ns
        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _on_result(self, future, goal_handle=None, token=None) -> None:
        is_current = (goal_handle is None or self._active_goal_handle is goal_handle) and (
            token is None or self._active_token == token
        )
        self._clear_active_goal(goal_handle, token)
        try:
            wrapped_result = future.result()
        except Exception as exc:
            if is_current:
                self.get_logger().error(f"Navigation action failed: {exc}")
                self._report("ABORTED", self._goals.get(token), token=token, reason=str(exc))
            return
        if not is_current:
            return  # A goal we replaced; its PREEMPTED status is already out.

        state = _RESULT_STATES.get(wrapped_result.status, "ABORTED")
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            state = "SUCCEEDED"
        handed_over = self._handoff_token == token and self._handoff_generation == self._authority_generation
        if self._handoff_token == token:
            self._handoff_token = None
        if (
            self._alignment_enabled
            and (state == "SUCCEEDED" or (state == "CANCELED" and handed_over))
            and self._start_alignment(token)
        ):
            self._cancel_reason = None
            return
        if state == "SUCCEEDED":
            self.get_logger().info("Navigation goal reached")
        else:
            self.get_logger().warning(
                f"Navigation finished with action status {wrapped_result.status}"
            )
        reason = self._cancel_reason if state == "CANCELED" and self._cancel_reason else ""
        self._report(state, self._goals.get(token), token=token, reason=reason)
        self._cancel_reason = None

    def _cancel_obsolete(self, goal_handle) -> None:
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"Failed to cancel a superseded navigation goal: {exc}")

    def _cancel_active_goal(self, goal_handle=None, token=None) -> None:
        handle = goal_handle or self._active_goal_handle
        if handle is None or self._cancel_in_flight:
            return
        try:
            self._cancel_in_flight = True
            self._cancel_retry_needed = False
            future = handle.cancel_goal_async()
            active_token = self._active_token if token is None else token
            future.add_done_callback(lambda response: self._on_cancel_response(response, handle, active_token))
        except Exception as exc:
            self._cancel_in_flight = False
            self._cancel_retry_needed = True
            self.get_logger().error(f"Failed to cancel navigation goal: {exc}")

    def _on_cancel_response(self, future, goal_handle, token) -> None:
        if self._active_goal_handle is not goal_handle or self._active_token != token:
            return
        self._cancel_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._cancel_retry_needed = True
            self.get_logger().error(f"Navigation cancel request failed: {exc}")
            return
        if not getattr(response, "goals_canceling", None):
            self._cancel_retry_needed = True
            self.get_logger().warning("Nav2 did not accept the navigation cancel request; retrying")

    def _clear_active_goal(self, goal_handle=None, token=None) -> None:
        if (goal_handle is None or self._active_goal_handle is goal_handle) and (token is None or self._active_token == token):
            self._active_goal_handle = None
            self._active_token = None
            self._cancel_in_flight = False
            self._cancel_retry_needed = False
            self._goal_gate.set_goal_active(False)

    def _expire_ready_if_stale(self) -> None:
        if (
            self._goal_gate.ready
            and (
                self._last_ready_at is None
                or self._now() - self._last_ready_at > self._ready_timeout
            )
        ):
            self.get_logger().warning("Localization readiness heartbeat timed out")
            self._revoke_readiness()

    def _revoke_readiness(self) -> None:
        was_ready = self._goal_gate.ready
        self._last_ready_at = None
        self._goal_gate.update_ready(False)
        if was_ready or self._active_goal_handle is not None:
            self._withdraw_authority("localization not ready")

    def _withdraw_authority(self, reason: str) -> None:
        self._authority_generation += 1
        self._stop_alignment("CANCELED", reason)
        if self._active_goal_handle is not None:
            self._cancel_reason = reason
            self._cancel_active_goal()

    # -- final alignment ------------------------------------------------------

    def _on_localization_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            check = ScanCheck(
                float(status["drift_check_stamp"]),
                float(status["overlap"]),
                Pose2D(
                    float(status["drift_offset_x"]),
                    float(status["drift_offset_y"]),
                    math.radians(float(status["drift_offset_yaw_deg"])),
                ),
            )
        except (ValueError, KeyError, TypeError):
            return
        if check.stamp > 0.0:  # 0.0: no scan check since the last lock
            self._scan_check = check

    def _on_alignment_timer(self) -> None:
        if self._aligner is not None:
            self._apply_alignment_step(
                self._aligner.step(self._robot_pose(), self._now(), self._scan_check)
            )
            return
        token = self._active_token
        if token is None or self._handoff_token == token or self._cancel_in_flight:
            return
        goal = self._goal_pose2d(self._goals.get(token))
        robot = self._robot_pose() if goal is not None else None
        if robot is None:
            return
        distance = math.hypot(goal.x - robot.x, goal.y - robot.y)
        if distance > self._alignment_policy.handoff_radius:
            return
        # Taking over here, before Nav2's own xy tolerance, keeps DWB out of the
        # RotateToGoal / path-critic oscillation it never left on 09-18 and 10-25.
        self.get_logger().info(f"{distance:.2f} m from the goal; final alignment takes over from Nav2")
        self._handoff_token = token
        self._handoff_generation = self._authority_generation
        self._cancel_active_goal()

    def _start_alignment(self, token) -> bool:
        """Begin aligning to goal `token`; False when it cannot be aligned at all."""
        pose = self._goals.get(token)
        goal = self._goal_pose2d(pose)
        if goal is None:
            return False
        if not self._goal_gate.accept_goal():
            self._report("CANCELED", pose, token=token, reason=self._goal_gate.refusal_reason())
            return True
        robot = self._robot_pose()
        if robot is None:
            self.get_logger().error("Final alignment cannot start: the robot pose is unavailable")
            self._report("ABORTED", pose, token=token, reason="final alignment: robot pose is unavailable")
            return True
        self._aligner = FinalAligner(self._alignment_policy)
        self._aligning_token = token
        self._alignment_phase = None
        self._goal_gate.set_goal_active(True)
        self._apply_alignment_step(self._aligner.start(goal, robot, self._now()))
        return True

    def _apply_alignment_step(self, step) -> None:
        token = self._aligning_token
        pose = self._goals.get(token)
        self._publish_alignment_command(*step.command)
        if step.done:
            self._finish_alignment()
            if step.phase is AlignmentPhase.SUCCEEDED:
                self.get_logger().info(f"Navigation goal reached and confirmed ({step.reason})")
                self._report("SUCCEEDED", pose, token=token, reason=step.reason, alignment=step)
            else:
                self.get_logger().warning(f"Final alignment failed: {step.reason}")
                self._report("ABORTED", pose, token=token, reason=f"final alignment: {step.reason}", alignment=step)
            return
        now_ns = self.get_clock().now().nanoseconds
        if step.phase is not self._alignment_phase or now_ns - self._last_alignment_report_ns >= 1_000_000_000:
            self._alignment_phase = step.phase
            self._last_alignment_report_ns = now_ns
            self._report("ALIGNING", pose, token=token, reason=step.phase.value, alignment=step)

    def _stop_alignment(self, state: str, reason: str) -> None:
        if self._aligner is None:
            return
        token = self._aligning_token
        self._publish_alignment_command(0.0, 0.0, 0.0)
        self._finish_alignment()
        self.get_logger().warning(f"Final alignment stopped: {reason}")
        self._report(state, self._goals.get(token), token=token, reason=reason)

    def _finish_alignment(self) -> None:
        self._aligner = None
        self._aligning_token = None
        self._alignment_phase = None
        self._goal_gate.set_goal_active(False)

    def _publish_alignment_command(self, vx: float, vy: float, yaw: float) -> None:
        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(yaw)
        self._alignment_publisher.publish(message)

    def _goal_pose2d(self, pose):
        if pose is None:
            return None
        goal = _goal_summary(pose)
        if goal["frame"] != self._global_frame or None in (goal["x"], goal["y"], goal["yaw"]):
            return None
        return Pose2D(goal["x"], goal["y"], goal["yaw"])

    def _robot_pose(self):
        """The tracked base pose in the goal frame, or None when it is not current."""
        try:
            transform = self._tf_buffer.lookup_transform(self._global_frame, self._base_frame, Time())
            stamp = transform.header.stamp
            age = self._now() - (stamp.sec + stamp.nanosec / 1_000_000_000.0)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        except Exception:  # tf2 raises several lookup/extrapolation types
            return None
        if age > self._alignment_policy.pose_timeout or not all(
            math.isfinite(value) for value in (translation.x, translation.y)
        ):
            return None
        return Pose2D(float(translation.x), float(translation.y), yaw)

    def _report(self, state, pose, *, token=None, reason="", distance_remaining=None, alignment=None) -> None:
        goal = _goal_summary(pose) if pose is not None else {"frame": "", "x": None, "y": None, "yaw": None}
        status = {
            "state": state,
            "reason": reason,
            "token": token,
            "goal": goal,
            "distance_remaining": (
                float(distance_remaining)
                if isinstance(distance_remaining, (int, float)) and math.isfinite(distance_remaining)
                else None
            ),
            "alignment": _alignment_summary(alignment),
            "stamp": self._now(),
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        self._status_publisher.publish(message)
        if goal["frame"] and goal["x"] is not None and goal["y"] is not None:
            self._publish_marker(state, goal, reason, status["distance_remaining"])

    def _publish_marker(self, state, goal, reason, distance_remaining) -> None:
        colour = _COLOURS.get(state, _FAILED)
        yaw = goal["yaw"] or 0.0
        if state == "SUCCEEDED":
            text = "ARRIVED"
        elif state in ("SENT", "ACCEPTED"):
            text = "GOAL" if distance_remaining is None else f"GOAL {distance_remaining:.1f} m"
        else:
            text = state if not reason else f"{state}: {reason}"
        for marker_id, marker_type, z, scale in (
            (0, Marker.ARROW, 0.05, (0.6, 0.12, 0.12)),
            (1, Marker.TEXT_VIEW_FACING, 0.6, (0.0, 0.0, 0.25)),
        ):
            marker = Marker()
            marker.header.frame_id = goal["frame"]
            marker.ns = "navigation_goal"
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.position.x = goal["x"]
            marker.pose.position.y = goal["y"]
            marker.pose.position.z = z
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b = colour
            marker.color.a = 1.0
            marker.text = text if marker_type == Marker.TEXT_VIEW_FACING else ""
            self._marker_publisher.publish(marker)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizGoalBridge()
    try:
        rclpy.spin(node)
    except NORMAL_SHUTDOWN_EXCEPTIONS:
        pass
    finally:
        node.destroy_node()
        shutdown_context(rclpy)


if __name__ == "__main__":
    main()
