"""Forward RViz's /goal_pose topic to Nav2's NavigateToPose action."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class RvizGoalBridge(Node):
    """Let the stock RViz '2D Goal Pose' tool control Nav2."""

    def __init__(self) -> None:
        super().__init__("rviz_goal_bridge")

        goal_topic = self.declare_parameter("goal_topic", "goal_pose").value
        action_name = self.declare_parameter(
            "navigate_to_pose_action", "navigate_to_pose"
        ).value
        self._server_timeout = float(
            self.declare_parameter("server_timeout", 2.0).value
        )

        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._goal_subscription = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal_pose, 10
        )
        self._last_feedback_log_ns = 0
        self.get_logger().info(
            f"Forwarding '{goal_topic}' goals to '{action_name}'"
        )

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        if not pose.header.frame_id:
            self.get_logger().error("Ignoring goal without a frame_id")
            return

        if not self._action_client.wait_for_server(timeout_sec=self._server_timeout):
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
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy reports transport failures through the future
            self.get_logger().error(f"Failed to send navigation goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected the navigation goal")
            return

        self.get_logger().info("Navigation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 5_000_000_000:
            return
        self._last_feedback_log_ns = now_ns
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _on_result(self, future) -> None:
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
