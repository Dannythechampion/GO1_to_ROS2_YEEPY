#!/usr/bin/env python3
"""Runtime verification for the ROS2 Go1 driver in disarmed mode."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class DryRunVerifier(Node):
    def __init__(self) -> None:
        super().__init__("go1_driver_dry_run_verifier")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.applied_messages: list[tuple[float, float, float]] = []
        self.states: list[str] = []
        self.create_subscription(Twist, "/go1/cmd_vel_applied", self._on_applied, 10)
        self.create_subscription(String, "/go1/control_state", self._on_state, 10)

    def _on_applied(self, message: Twist) -> None:
        self.applied_messages.append(
            (message.linear.x, message.linear.y, message.angular.z)
        )

    def _on_state(self, message: String) -> None:
        self.states.append(message.data)


def spin_until(node: Node, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise RuntimeError(f"Timed out waiting for {description}")


def main() -> int:
    rclpy.init()
    node = DryRunVerifier()
    try:
        spin_until(
            node,
            lambda: node.publisher.get_subscription_count() > 0,
            8.0,
            "the go1_driver /cmd_vel subscription",
        )

        command = Twist()
        command.linear.x = 0.30
        command.linear.y = 0.30

        publish_deadline = time.monotonic() + 0.50
        while time.monotonic() < publish_deadline:
            node.publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

        def limited_diagonal_seen() -> bool:
            for vx, vy, yaw in node.applied_messages:
                if vx > 0.0 and vy > 0.0 and math.isclose(
                    math.hypot(vx, vy), 0.20, rel_tol=1e-3, abs_tol=1e-3
                ):
                    return math.isclose(vx, vy, rel_tol=1e-3, abs_tol=1e-3) and yaw == 0.0
            return False

        spin_until(node, limited_diagonal_seen, 3.0, "a direction-preserving 0.20 m/s limit")

        applied_index = len(node.applied_messages)
        state_index = len(node.states)

        def watchdog_stop_seen() -> bool:
            recent_applied = node.applied_messages[applied_index:]
            recent_states = node.states[state_index:]
            zero_seen = any(
                abs(vx) < 1e-9 and abs(vy) < 1e-9 and abs(yaw) < 1e-9
                for vx, vy, yaw in recent_applied
            )
            timeout_seen = any("watchdog timeout" in state for state in recent_states)
            return zero_seen and timeout_seen

        spin_until(node, watchdog_stop_seen, 3.0, "the stale-command watchdog stop")

        if not any("DRY-RUN" in state for state in node.states):
            raise RuntimeError("Driver did not report DRY-RUN state")

        print("PASS: diagonal direction preserved and limited to 0.20 m/s")
        print("PASS: watchdog changed stale command to exact zero")
        print("PASS: driver reported DRY-RUN mode")
        return 0
    except Exception as exc:  # noqa: BLE001 - command-line verifier reports all failures
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
