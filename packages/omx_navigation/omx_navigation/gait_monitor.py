"""Log how steadily the Go1 carries its sensors, window by window.

Subscribes to the MID-360's IMU (`/livox/imu`, in g) and the Go1 trunk IMU
(`/go1/imu`, m/s^2, published by go1_driver while armed), summarises each
0.5 s window with `gait_stability.window_stats`, tags it with what the robot
was doing -- mission state, dock phase, whether a walk command was applied --
and writes one CSV row per window per source.

The CSV is flushed every row and fsynced every second, so a killed process
or a power cut keeps everything up to the last second. Each window is also
published as JSON on `/gait/stability` so it lands in the bag beside the
scans it explains.
"""

import csv
import json
import os
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from omx_navigation.gait_stability import ImuWindow, WindowStats, mount_amplification
from omx_navigation.runtime_shutdown import NORMAL_SHUTDOWN_EXCEPTIONS, shutdown_context

EXTRA_FIELDS = ("mission_state", "dock_phase", "walking", "mount_amplification")


class GaitMonitor(Node):
    def __init__(self) -> None:
        super().__init__("gait_monitor")
        log_dir = self.declare_parameter("log_dir", os.environ.get("GO1_SESSION_DIR", ".")).value
        period = float(self.declare_parameter("window", 0.5).value)
        os.makedirs(log_dir, exist_ok=True)
        self._path = os.path.join(log_dir, "stability.csv")
        new = not os.path.exists(self._path) or os.path.getsize(self._path) == 0
        self._file = open(self._path, "a", encoding="utf-8", newline="", buffering=1)
        self._csv = csv.writer(self._file)
        if new:
            self._csv.writerow(list(WindowStats.CSV_FIELDS) + list(EXTRA_FIELDS))
        self._synced_at = time.monotonic()
        self._windows = {"lidar": ImuWindow("lidar", period), "trunk": ImuWindow("trunk", period)}
        self._latest = {"lidar": None, "trunk": None}
        self._mission_state = ""
        self._dock_phase = ""
        self._walking = False
        self._publisher = self.create_publisher(String, "/gait/stability", 10)
        self.create_subscription(Imu, self.declare_parameter("lidar_imu_topic", "/livox/imu").value,
                                 lambda m: self._on_imu("lidar", m), qos_profile_sensor_data)
        self.create_subscription(Imu, self.declare_parameter("trunk_imu_topic", "/go1/imu").value,
                                 lambda m: self._on_imu("trunk", m), qos_profile_sensor_data)
        self.create_subscription(Twist, "/go1/cmd_vel_applied", self._on_applied, 10)
        self.create_subscription(String, "/mission/status", self._on_mission, 10)
        self.get_logger().info(f"Writing gait stability to {self._path}")

    def _on_imu(self, source: str, message: Imu) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        sample = (stamp, message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                  message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z)
        stats = self._windows[source].add(sample)
        if stats is None:
            return
        self._latest[source] = stats
        amplification = mount_amplification(self._latest["lidar"], self._latest["trunk"]) if source == "lidar" else None
        extra = [self._mission_state, self._dock_phase, int(self._walking),
                 None if amplification is None else round(amplification, 3)]
        self._csv.writerow(stats.row() + extra)
        now = time.monotonic()
        if now - self._synced_at >= 1.0:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._synced_at = now
        report = dict(zip(WindowStats.CSV_FIELDS + EXTRA_FIELDS, stats.row() + extra))
        self._publisher.publish(String(data=json.dumps(report, separators=(",", ":"))))
        if stats.verdict == "UNSTABLE":
            self.get_logger().warning(f"{source} UNSTABLE: {stats.reasons}")

    def _on_applied(self, message: Twist) -> None:
        self._walking = bool(message.linear.x or message.linear.y or message.angular.z)

    def _on_mission(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        self._mission_state = status.get("state") or ""
        self._dock_phase = status.get("dock_phase") or ""

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()
            self._file = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitMonitor()
    try:
        rclpy.spin(node)
    except NORMAL_SHUTDOWN_EXCEPTIONS:
        pass
    finally:
        node.close()
        node.destroy_node()
        shutdown_context(rclpy)


if __name__ == "__main__":
    main()
