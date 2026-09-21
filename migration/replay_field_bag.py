#!/usr/bin/env python3
"""Feed a recorded field session's raw sensor inputs into a live navigation stack.

A session bag also records what the stack computed on the day (slam_toolbox's
map -> camera_init, planar_base_frame's body_nav). Playing it back whole would
fight the live nodes. This player republishes only what the robot itself
produced -- /scan, /Odometry, FAST-LIO's camera_init -> body transform -- plus
the operator's /initialpose clicks, re-stamped to the current time, so the
launched slam_toolbox, localization supervisor and Nav2 run closed loop on the
field data exactly as they would on the robot.

  ros2 launch omx_navigation go1_posegraph_navigation.launch.py rviz:=false ...
  python3 migration/replay_field_bag.py <session>/rosbag --start 0 --end 600

Needs ROS 2 Humble (rclpy, rosbag2_py). Wall-clock stamps, real-time pace.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

RAW_TRANSFORMS = {("camera_init", "body")}
TYPES = {
    "/scan": LaserScan,
    "/Odometry": Odometry,
    "/tf": TFMessage,
    "/initialpose": PoseWithCovarianceStamped,
    "/goal_pose": PoseStamped,
}


def shift(stamp, offset_ns: int):
    shifted = Time(seconds=stamp.sec, nanoseconds=stamp.nanosec).nanoseconds + offset_ns
    return Time(nanoseconds=shifted).to_msg()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="the session's rosbag directory")
    parser.add_argument("--start", type=float, default=0.0, help="seconds into the bag")
    parser.add_argument("--end", type=float, default=float("inf"), help="seconds into the bag")
    parser.add_argument("--no-initialpose", action="store_true", help="do not replay the operator's clicks")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    rclpy.init()
    node = rclpy.create_node("field_bag_replay")
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    publishers = {
        "/scan": node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data),
        "/Odometry": node.create_publisher(Odometry, "/Odometry", 10),
        "/tf": node.create_publisher(TFMessage, "/tf", 100),
        "/initialpose": node.create_publisher(PoseWithCovarianceStamped, "/initialpose", reliable),
    }
    first_ns = None
    offset_ns = None
    wall_start = None
    counts = {topic: 0 for topic in publishers}
    try:
        while reader.has_next():
            topic, data, recorded_ns = reader.read_next()
            if topic not in publishers or (topic == "/initialpose" and args.no_initialpose):
                continue
            first_ns = recorded_ns if first_ns is None else first_ns
            elapsed = (recorded_ns - first_ns) / 1e9
            if elapsed < args.start:
                continue
            if elapsed > args.end:
                break
            if wall_start is None:
                wall_start = time.monotonic() - (elapsed - args.start)
                offset_ns = node.get_clock().now().nanoseconds - recorded_ns
            delay = wall_start + (elapsed - args.start) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            message = deserialize_message(data, TYPES[topic])
            if topic == "/tf":
                message.transforms = [
                    transform for transform in message.transforms
                    if (transform.header.frame_id, transform.child_frame_id) in RAW_TRANSFORMS
                ]
                if not message.transforms:
                    continue
                for transform in message.transforms:
                    transform.header.stamp = shift(transform.header.stamp, offset_ns)
            else:
                message.header.stamp = shift(message.header.stamp, offset_ns)
            publishers[topic].publish(message)
            counts[topic] += 1
            if topic == "/initialpose":
                print(f"replayed operator /initialpose at bag t+{elapsed:.1f} s", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print("published:", counts, flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
