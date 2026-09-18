#!/usr/bin/env python3
"""Verify the whole /cmd_vel chain against a running stack while the Go1 is off.

`verify_go1_driver_dry_run.py` starts a driver of its own and checks the driver
in isolation. This verifier instead probes the stack that is already running --
the one `jetson_field_deploy.sh dry-run` launches -- so it also covers the piece
the isolated test cannot see: the localization gate between Nav2 and the driver.

Everything here holds with the robot switched off, because nothing past the
driver is exercised: in DRY-RUN the driver opens no UDP socket to the robot.

Run it after `jetson_field_deploy.sh dry-run` is up:

  source /opt/ros/humble/setup.bash
  source /mnt/t500/go1_ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=100
  python3 migration/verify_cmd_vel_chain_go1_off.py
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

# The driver clamps to max_linear_speed, and a float round-trip must not turn a
# correct clamp into a failure.
SPEED_LIMIT = 0.20
SPEED_LIMIT_TOLERANCE = 0.001
TEST_SPEED = 0.15
TEST_YAW = 0.10


class ChainVerifier:
    def __init__(self, node) -> None:
        self.node = node
        self.cmd_vel: list[Twist] = []
        self.applied: list[Twist] = []
        self.states: list[str] = []
        self.ready: list[bool] = []
        node.create_subscription(Twist, "/cmd_vel", self.cmd_vel.append, 10)
        node.create_subscription(Twist, "/go1/cmd_vel_applied", self.applied.append, 10)
        node.create_subscription(String, "/go1/control_state", self._on_state, 10)
        node.create_subscription(Bool, "/localization_supervisor/ready", self._on_ready, 10)
        self.nav_publisher = node.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.direct_publisher = node.create_publisher(Twist, "/cmd_vel", 10)

    def _on_state(self, message: String) -> None:
        self.states.append(message.data)

    def _on_ready(self, message: Bool) -> None:
        self.ready.append(message.data)

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def drive(self, publisher, vx: float, yaw: float, seconds: float) -> None:
        message = Twist()
        message.linear.x = vx
        message.angular.z = yaw
        deadline = time.time() + seconds
        while time.time() < deadline:
            publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.05)


def require_disarmed(verifier: ChainVerifier) -> None:
    """Refuse to publish velocity unless the driver reports DRY-RUN.

    This verifier publishes straight onto /cmd_vel, which is the driver's input.
    Against an ARMED driver that is a movement command, so an unseen or armed
    driver has to stop the run rather than start one.
    """
    verifier.spin(3.0)
    if not verifier.states:
        raise RuntimeError(
            "no /go1/control_state seen -- is the dry-run stack running in this ROS_DOMAIN_ID?"
        )
    if not all("DRY-RUN" in state for state in verifier.states):
        raise RuntimeError(
            "driver is not in DRY-RUN; refusing to publish /cmd_vel at an armed driver"
        )
    print("PASS: driver is present and reports DRY-RUN")


def check_gate_blocks_nav(verifier: ChainVerifier) -> None:
    if verifier.ready and verifier.ready[-1]:
        print("SKIP: localization is READY, so the gate is open by design")
        return
    verifier.cmd_vel.clear()
    verifier.drive(verifier.nav_publisher, TEST_SPEED, 0.0, 3.0)
    verifier.spin(0.5)
    peak = max((abs(message.linear.x) for message in verifier.cmd_vel), default=0.0)
    if peak != 0.0:
        raise RuntimeError(
            f"Nav2 velocity reached /cmd_vel at {peak:.3f} m/s while localization was not ready"
        )
    print(
        "PASS: gate held /cmd_vel at zero for %d messages while localization was not ready"
        % len(verifier.cmd_vel)
    )


def check_driver_applies_command(verifier: ChainVerifier) -> None:
    verifier.applied.clear()
    verifier.states.clear()
    verifier.drive(verifier.direct_publisher, TEST_SPEED, TEST_YAW, 3.0)
    verifier.spin(0.5)
    peak = max((message.linear.x for message in verifier.applied), default=0.0)
    if abs(peak - TEST_SPEED) > 0.001:
        raise RuntimeError(f"driver applied {peak:.3f} m/s for a {TEST_SPEED:.3f} m/s command")
    walking = [state for state in verifier.states if "mode=2" in state]
    if not walking:
        raise RuntimeError("driver never reported mode=2 for a non-zero command")
    print(f"PASS: driver applied {peak:.3f} m/s and entered mode=2")
    print(f"      {walking[len(walking) // 2]}")


def check_speed_limit(verifier: ChainVerifier) -> None:
    verifier.applied.clear()
    verifier.drive(verifier.direct_publisher, 1.5, 0.0, 2.0)
    verifier.spin(0.5)
    peak = max((message.linear.x for message in verifier.applied), default=0.0)
    if not 0.0 < peak <= SPEED_LIMIT + SPEED_LIMIT_TOLERANCE:
        raise RuntimeError(f"1.500 m/s command was applied as {peak:.3f} m/s")
    print(f"PASS: 1.500 m/s command was clamped to {peak:.3f} m/s")


def check_watchdog(verifier: ChainVerifier) -> None:
    verifier.applied.clear()
    verifier.spin(2.5)
    tail = [message.linear.x for message in verifier.applied[-10:]]
    if not tail:
        raise RuntimeError("driver stopped publishing /go1/cmd_vel_applied")
    if max(abs(value) for value in tail) != 0.0:
        raise RuntimeError(f"stale command was not zeroed: {tail}")
    print("PASS: watchdog returned the stale command to exact zero")


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("cmd_vel_chain_verifier")
    verifier = ChainVerifier(node)
    try:
        require_disarmed(verifier)
        check_gate_blocks_nav(verifier)
        check_driver_applies_command(verifier)
        check_speed_limit(verifier)
        check_watchdog(verifier)
        print("PASS: /cmd_vel chain verified with the Go1 powered off")
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
