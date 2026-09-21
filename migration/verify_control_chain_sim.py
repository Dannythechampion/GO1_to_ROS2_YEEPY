#!/usr/bin/env python3
"""Run the armed Go1 driver, the goal bridge and a stand-in Nav2 together, with no robot.

The armed driver path talks to the Unitree SDK, so until a session with the
robot it never runs anywhere. This verifier gives it a simulated SDK -- a
`robot_interface` module whose robot follows or ignores commands, carries a
remote, and can drop its link -- and checks the contract the 2026-09-18 field
session showed was missing, with real rclpy, real messages and real timers:

  * commands reach the robot while nobody else is in control;
  * a deflected remote stick holds stand, and the navigation goal is cancelled;
  * releasing the remote does not resume the old command until cmd_vel has
    been zero;
  * a robot that ignores commands raises an execution fault that clears only
    after a zero command;
  * an RViz click before localization is ready is visibly ignored, a goal that
    arrives is reported ARRIVED, and a new click replaces the active goal;
  * a lost link is reported.

Needs ROS 2 Humble with this repository's packages importable, e.g.

  source /opt/ros/humble/setup.bash
  source /mnt/t500/go1_ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=42    # any domain in 0-101 that nothing else uses
  python3 migration/verify_control_chain_sim.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

FAKE_SDK = r'''
"""Simulated unitree_legged_sdk HighLevel binding for verify_control_chain_sim.py."""
import struct


class _Sim:
    def __init__(self):
        self.live = True
        self.follow = True
        self.remote = bytes(40)
        self.last_cmd = (0, [0.0, 0.0], 0.0)


SIM = _Sim()


def remote_frame(ly=0.0, lx=0.0, rx=0.0, ry=0.0, l2=0.0, buttons=0):
    frame = bytes((0xFE, 0xEF)) + struct.pack("<H5f", buttons, lx, rx, ry, l2, ly)
    return frame + bytes(40 - len(frame))


class HighCmd:
    def __init__(self):
        self.mode = 0
        self.gaitType = 0
        self.velocity = [0.0, 0.0]
        self.yawSpeed = 0.0
        self.bodyHeight = 0.0
        self.euler = [0.0, 0.0, 0.0]
        self.footRaiseHeight = 0.0
        self.reserve = 0


class _Imu:
    def __init__(self):
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 9.8]
        self.gyroscope = [0.0, 0.0, 0.0]


class _Bms:
    def __init__(self):
        self.SOC = 81


class HighState:
    def __init__(self):
        self.head = [0, 0]
        self.imu = _Imu()
        self.footForce = [0, 0, 0, 0]
        self.bms = _Bms()
        self.mode = 0
        self.velocity = [0.0, 0.0, 0.0]
        self.yawSpeed = 0.0
        self.bodyHeight = 0.0
        self.rangeObstacle = [0.0, 0.6, 2.0, 2.0]
        self.wirelessRemote = [0] * 40


class UDP:
    def __init__(self, level, local_port, ip, port):
        pass

    def InitCmdData(self, cmd):
        pass

    def Recv(self):
        return 128 if SIM.live else 0

    def GetRecv(self, state):
        if not SIM.live:
            return
        mode, velocity, yaw = SIM.last_cmd
        state.head = [0xFE, 0xEF]
        state.footForce = [20, 21, 19, 22]
        state.mode = mode
        state.velocity = [velocity[0], velocity[1], 0.0] if SIM.follow else [0.0, 0.0, 0.0]
        state.yawSpeed = yaw if SIM.follow else 0.0
        state.bodyHeight = 0.29
        state.wirelessRemote = list(SIM.remote)

    def SetSend(self, cmd):
        SIM.last_cmd = (cmd.mode, list(cmd.velocity), cmd.yawSpeed)

    def Send(self):
        pass
'''


def main() -> int:
    sdk_dir = Path(tempfile.mkdtemp(prefix="fake_unitree_sdk_"))
    (sdk_dir / "robot_interface.py").write_text(FAKE_SDK, encoding="utf-8")
    sys.path.insert(0, str(sdk_dir))

    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker

    import robot_interface
    from go1_driver.node import ARMED_CONFIRMATION_TOKEN, Go1Driver
    from omx_navigation.rviz_goal_bridge import RvizGoalBridge

    sim = robot_interface.SIM
    rclpy.init(args=[
        "--ros-args",
        "-p", "arm:=true",
        "-p", f"armed_confirmation:={ARMED_CONFIRMATION_TOKEN}",
        "-p", f"sdk_path:={sdk_dir}",
        "-p", "odom_topic:=/Odometry",
    ])

    class Harness(Node):
        """Operator, localization and Nav2 stand-ins, plus a simulated odometry."""

        def __init__(self):
            super().__init__("control_chain_harness")
            group = ReentrantCallbackGroup()
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
            self.goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
            self.ready = self.create_publisher(Bool, "/localization_supervisor/ready", 10)
            self.odom = self.create_publisher(Odometry, "/Odometry", 10)
            self.statuses = []
            self.markers = []
            self.override = None
            self.fault = None
            self.robot_state = None
            self.create_subscription(String, "/navigation/goal_status", lambda m: self.statuses.append(json.loads(m.data)), 50)
            self.create_subscription(Marker, "/navigation/goal_marker", self.markers.append, 50)
            self.create_subscription(Bool, "/go1/manual_override", lambda m: setattr(self, "override", m.data), 10)
            self.create_subscription(Bool, "/go1/execution_fault", lambda m: setattr(self, "fault", m.data), 10)
            self.create_subscription(String, "/go1/robot_state", lambda m: setattr(self, "robot_state", json.loads(m.data)), 10)
            self.command = (0.0, 0.0, 0.0)
            self.localization_ready = False
            self.pose = [0.0, 0.0, 0.0]
            self.create_timer(0.05, self._tick, callback_group=group)
            # Stand-in for bt_navigator's NavigateToPose server.
            self.nav_hold = 1.5
            self.executing = []
            self.server = ActionServer(
                self, NavigateToPose, "navigate_to_pose", self._execute,
                goal_callback=lambda _goal: GoalResponse.ACCEPT,
                handle_accepted_callback=self._accepted,
                cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                callback_group=group,
            )

        def _tick(self):
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = self.command
            self.cmd.publish(twist)
            if self.localization_ready:
                self.ready.publish(Bool(data=True))
            mode, velocity, yaw = sim.last_cmd
            if sim.follow and mode == 2:
                c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
                self.pose[0] += 0.05 * (c * velocity[0] - s * velocity[1])
                self.pose[1] += 0.05 * (s * velocity[0] + c * velocity[1])
                self.pose[2] += 0.05 * yaw
            odom = Odometry()
            odom.header.frame_id = "camera_init"
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[0], self.pose[1]
            odom.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
            self.odom.publish(odom)

        def _accepted(self, handle):
            # bt_navigator aborts the running goal when a new one arrives.
            for running in list(self.executing):
                running.preempted = True
            handle.preempted = False
            self.executing.append(handle)
            handle.execute()

        def _execute(self, handle):
            feedback = NavigateToPose.Feedback()
            started = time.monotonic()
            try:
                while time.monotonic() - started < self.nav_hold:
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return NavigateToPose.Result()
                    if handle.preempted:
                        handle.abort()
                        return NavigateToPose.Result()
                    feedback.distance_remaining = float(self.nav_hold - (time.monotonic() - started))
                    handle.publish_feedback(feedback)
                    time.sleep(0.1)
                handle.succeed()
                return NavigateToPose.Result()
            finally:
                self.executing.remove(handle)

        def click(self, x, y):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            self.goal.publish(pose)

    harness = Harness()
    driver = Go1Driver()
    bridge = RvizGoalBridge()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (harness, driver, bridge):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    results = []

    def check(label, condition, detail=""):
        results.append(bool(condition))
        print(("PASS " if condition else "FAIL ") + label + (f"  [{detail}]" if detail and not condition else ""))

    def wait_for(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()

    def last_state(state=None):
        for item in reversed(harness.statuses):
            if state is None or item["state"] == state:
                return item
        return None

    try:
        wait_for(lambda: harness.robot_state is not None, 5.0)
        check("driver publishes /go1/robot_state while armed", harness.robot_state and harness.robot_state["armed"])
        check("link to the simulated Go1 is up", wait_for(lambda: harness.robot_state["link_up"] is True, 3.0))

        harness.click(1.0, 2.0)
        check("click before localization is ready is IGNORED with the reason",
              wait_for(lambda: last_state("IGNORED") is not None, 3.0)
              and last_state("IGNORED")["reason"] == "localization not ready")
        check("the ignored click is shown as a marker",
              wait_for(lambda: any(m.type == Marker.TEXT_VIEW_FACING and m.text.startswith("IGNORED") for m in harness.markers), 2.0))

        harness.localization_ready = True
        time.sleep(0.5)
        harness.nav_hold = 1.5
        harness.click(3.0, -1.0)
        check("goal is sent and accepted", wait_for(lambda: last_state("ACCEPTED") is not None, 5.0))
        check("arrival is published as SUCCEEDED", wait_for(lambda: last_state("SUCCEEDED") is not None, 6.0))
        check("arrival is shown as an ARRIVED marker",
              wait_for(lambda: any(m.type == Marker.TEXT_VIEW_FACING and m.text == "ARRIVED" for m in harness.markers), 2.0))

        harness.nav_hold = 30.0
        count = len(harness.statuses)
        harness.click(5.0, 0.0)
        wait_for(lambda: any(s["state"] == "ACCEPTED" for s in harness.statuses[count:]), 5.0)
        harness.click(6.0, 1.0)
        check("a new click replaces the active goal (old one PREEMPTED)",
              wait_for(lambda: any(s["state"] == "PREEMPTED" for s in harness.statuses[count:]), 5.0))
        time.sleep(1.0)
        newest = last_state("ACCEPTED")
        check("the replacing goal is the one being tracked",
              newest is not None and newest["goal"]["x"] == 6.0 and not any(
                  s["state"] == "ABORTED" for s in harness.statuses[count:]))

        harness.command = (0.15, 0.0, 0.2)
        time.sleep(1.0)
        check("commands reach the robot while nobody else is in control",
              wait_for(lambda: sim.last_cmd[0] == 2 and abs(sim.last_cmd[1][0] - 0.15) < 1e-6, 2.0), sim.last_cmd)

        sim.remote = robot_interface.remote_frame(ly=0.6)
        check("a deflected remote stick is manual override", wait_for(lambda: harness.override is True, 2.0))
        check("override holds stand", wait_for(lambda: sim.last_cmd[0] == 0, 1.0), sim.last_cmd)
        check("the navigation goal is cancelled on override",
              wait_for(lambda: (last_state("CANCELED") or {}).get("reason") == "manual override", 5.0))
        harness.click(7.0, 0.0)
        check("new goals are refused while the remote is in use",
              wait_for(lambda: (last_state("IGNORED") or {}).get("reason") == "manual override", 3.0))

        sim.remote = robot_interface.remote_frame()
        check("override ends after the remote has been idle", wait_for(lambda: harness.override is False, 3.0))
        time.sleep(1.0)
        check("the old command does not resume on release", sim.last_cmd[0] == 0, sim.last_cmd)
        harness.command = (0.0, 0.0, 0.0)
        time.sleep(1.0)
        harness.command = (0.10, 0.0, 0.0)
        check("a new command after a zero resumes motion",
              wait_for(lambda: sim.last_cmd[0] == 2 and abs(sim.last_cmd[1][0] - 0.10) < 1e-6, 2.0), sim.last_cmd)

        sim.follow = False
        harness.command = (0.0, 0.0, 0.4)
        check("a robot that ignores commands raises an execution fault",
              wait_for(lambda: harness.fault is True, 15.0), harness.robot_state and harness.robot_state["execution"])
        check("the fault holds stand", wait_for(lambda: sim.last_cmd[0] == 0, 1.0), sim.last_cmd)
        sim.follow = True
        harness.command = (0.0, 0.0, 0.0)
        check("the fault clears after a zero command", wait_for(lambda: harness.fault is False, 3.0))

        sim.live = False
        check("a lost link is reported", wait_for(lambda: harness.robot_state["link_up"] is False, 2.0))
        sim.live = True
        check("the link comes back", wait_for(lambda: harness.robot_state["link_up"] is True, 2.0))
    finally:
        executor.shutdown()
        for node in (bridge, driver, harness):
            node.destroy_node()
        rclpy.try_shutdown()

    passed = sum(results)
    print(f"\n{passed}/{len(results)} checks passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
