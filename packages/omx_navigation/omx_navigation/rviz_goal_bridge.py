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
"""

import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from omx_navigation.goal_gate import GoalGate
from omx_navigation.runtime_shutdown import (
    NORMAL_SHUTDOWN_EXCEPTIONS,
    shutdown_context,
)

STATUS_TOPIC = "/navigation/goal_status"
MARKER_TOPIC = "/navigation/goal_marker"
INHIBIT_TOPICS = {
    "manual override": "/go1/manual_override",
    "robot not following commands": "/go1/execution_fault",
}
_COLOURS = {
    "SENT": (1.0, 0.8, 0.0),
    "ACCEPTED": (1.0, 0.8, 0.0),
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
        self.get_logger().info(
            f"Forwarding '{goal_topic}' goals to '{action_name}'"
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
        if self._active_goal_handle is not None:
            self._cancel_reason = reason
            self._cancel_active_goal()

    def _report(self, state, pose, *, token=None, reason="", distance_remaining=None) -> None:
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
