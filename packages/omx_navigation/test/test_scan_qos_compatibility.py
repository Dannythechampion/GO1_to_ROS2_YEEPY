import time

import pytest


rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from omx_navigation.localization_supervisor import LocalizationSupervisor


def test_localization_supervisor_receives_best_effort_laser_scan():
    rclpy.init()
    supervisor = LocalizationSupervisor()
    publisher_node = Node("best_effort_scan_test_publisher")
    executor = SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(publisher_node)

    scan_qos = QoSProfile(depth=10)
    scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    scan_qos.durability = DurabilityPolicy.VOLATILE
    publisher = publisher_node.create_publisher(LaserScan, "/scan", scan_qos)

    try:
        deadline = time.monotonic() + 2.0
        while supervisor._scan is None and time.monotonic() < deadline:
            message = LaserScan()
            message.header.frame_id = "body"
            message.header.stamp = publisher_node.get_clock().now().to_msg()
            message.range_min = 0.2
            message.range_max = 10.0
            message.ranges = [1.0]
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)

        assert supervisor._scan is not None
    finally:
        executor.remove_node(publisher_node)
        executor.remove_node(supervisor)
        publisher_node.destroy_node()
        supervisor.destroy_node()
        rclpy.shutdown()
