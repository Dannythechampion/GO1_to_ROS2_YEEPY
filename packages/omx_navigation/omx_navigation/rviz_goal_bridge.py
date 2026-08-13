"""Forward RViz's /goal_pose topic to Nav2's NavigateToPose action."""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from omx_navigation.goal_gate import GoalGate


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
        self._pending_ready_generation = None
        self._goal_token = 0
        self._ready_generation = 0
        self._last_ready_at = None
        self._cancel_in_flight = False
        self._cancel_retry_needed = False
        self._goal_subscription = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal_pose, 10
        )
        self._ready_subscription = self.create_subscription(
            Bool, "/localization_supervisor/ready", self._on_ready, 10
        )
        self._watchdog_timer = self.create_timer(
            min(0.10, self._ready_timeout / 2.0), self._on_ready_watchdog
        )
        self._last_feedback_log_ns = 0
        self.get_logger().info(
            f"Forwarding '{goal_topic}' goals to '{action_name}'"
        )

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        self._expire_ready_if_stale()
        if not self._goal_gate.accept_goal():
            self.get_logger().warning("Ignoring navigation goal until localization is ready")
            return
        if self._pending_token is not None or self._active_goal_handle is not None:
            self.get_logger().warning("Ignoring navigation goal while another goal is pending or active")
            return
        if not pose.header.frame_id:
            self.get_logger().error("Ignoring goal without a frame_id")
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error(
                "Nav2 NavigateToPose action is not available yet; try the goal again"
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            "Sending goal in %s: x=%.2f, y=%.2f"
            % (pose.header.frame_id, pose.pose.position.x, pose.pose.position.y)
        )
        self._goal_token += 1
        token = self._goal_token
        self._pending_token = token
        ready_generation = self._ready_generation
        self._pending_ready_generation = ready_generation
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(
            lambda response: self._on_goal_response(response, token, ready_generation)
        )

    def _on_ready(self, message: Bool) -> None:
        if message.data:
            self._last_ready_at = self._now()
            self._goal_gate.update_ready(True)
        else:
            self._revoke_readiness()

    def _on_goal_response(self, future, token, ready_generation=None) -> None:
        if token != self._pending_token:
            return
        self._pending_token = None
        if ready_generation is None:
            ready_generation = self._pending_ready_generation
        self._pending_ready_generation = None
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy reports transport failures through the future
            self.get_logger().error(f"Failed to send navigation goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected the navigation goal")
            return

        self.get_logger().info("Navigation goal accepted")
        self._active_goal_handle = goal_handle
        self._active_token = token
        self._goal_gate.set_goal_active(True)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_result(result, goal_handle, token))
        # Any false/timeout epoch after request creation permanently invalidates
        # this goal, even if readiness recovers before Nav2 accepts it.
        self._expire_ready_if_stale()
        if ready_generation != self._ready_generation or not self._goal_gate.ready:
            self._cancel_active_goal(goal_handle, token)

    def _on_ready_watchdog(self) -> None:
        self._expire_ready_if_stale()
        if self._cancel_retry_needed and not self._cancel_in_flight:
            self._cancel_active_goal()

    def _on_feedback(self, feedback_msg) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 5_000_000_000:
            return
        self._last_feedback_log_ns = now_ns
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _on_result(self, future, goal_handle=None, token=None) -> None:
        self._clear_active_goal(goal_handle, token)
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Navigation action failed: {exc}")
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation goal reached")
        else:
            self.get_logger().warning(
                f"Navigation finished with action status {wrapped_result.status}"
            )

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
        if self._goal_gate.ready:
            self._ready_generation += 1
        self._last_ready_at = None
        should_cancel = self._goal_gate.update_ready(False)
        if should_cancel or self._active_goal_handle is not None:
            self._cancel_active_goal()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
