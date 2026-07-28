import math
import sqlite3
import time
from pathlib import Path
import uuid

import pytest
import yaml

from go1_mapping.session_guard import (
    HealthWindow,
    RosbagHealthWindow,
    evaluate_and_record,
    pose_return_error,
    quaternion_yaw,
    should_abort_disk,
)

REQUIRED_HZ = {"lidar": 8.0, "imu": 100.0, "odom": 8.0}


def test_health_window_suppresses_errors_during_initialization():
    window = HealthWindow(REQUIRED_HZ, 1.0, 15.0, 0.0)
    assert window.evaluate(10.0) == []


def test_health_window_reports_rate_gap_and_sensor_timestamp_regression():
    window = HealthWindow(REQUIRED_HZ, 1.0, 15.0, 0.0)
    for stamp in (10.0, 10.5, 11.0):
        window.observe("lidar", stamp, sensor_stamp_sec=stamp)
    for stamp in (14.00, 14.01, 14.02):
        window.observe("imu", stamp, sensor_stamp_sec=stamp)
    window.observe("imu", 14.03, sensor_stamp_sec=13.99)
    for stamp in (14.0, 14.125, 14.25):
        window.observe("odom", stamp, sensor_stamp_sec=stamp)
    errors = window.evaluate(16.1)
    assert any(error.startswith("lidar rate") for error in errors)
    assert any(error.startswith("lidar gap") for error in errors)
    assert any("imu sensor timestamp regression" in error for error in errors)


def test_receipt_regression_is_reported_without_negative_gap_or_rate():
    window = HealthWindow({"lidar": 1.0}, 1.0, 0.0, 0.0)
    window.observe("lidar", 5.0)
    window.observe("lidar", 4.0)
    measures = window.measurements(5.0)
    assert measures["max_gaps_sec"]["lidar"] >= 0.0
    assert measures["rates_hz"]["lidar"] == 0.0
    assert any("receipt timestamp regression" in error for error in window.evaluate(5.0))


def test_disk_threshold_is_strictly_below_abort_limit():
    assert should_abort_disk(49.99, 50.0) is True
    assert should_abort_disk(50.0, 50.0) is False


def test_quaternion_yaw_normalizes_scaled_quaternion_and_rejects_invalid_values():
    assert quaternion_yaw(0.0, 0.0, 10.0, 0.0) == pytest.approx(math.pi)
    with pytest.raises(ValueError, match="quaternion"):
        quaternion_yaw(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="quaternion"):
        quaternion_yaw(0.0, 0.0, math.inf, 1.0)


def test_health_record_is_atomic_and_pose_error_wraps_yaw():
    tmp_path = Path.cwd() / ".superpowers" / "sdd" / ("task5-health-" + uuid.uuid4().hex)
    tmp_path.mkdir(parents=True)
    reference = {"position": {"x": 1.0, "y": 2.0}, "orientation": {"z": 10.0, "w": 0.0}}
    latest = {"position": {"x": 4.0, "y": 6.0}, "orientation": {"z": -10.0, "w": 0.0}}
    assert pose_return_error(reference, latest) == {"xy_m": pytest.approx(5.0), "yaw_rad": pytest.approx(0.0)}
    exit_code, payload = evaluate_and_record(session_dir=tmp_path, window=HealthWindow({"lidar": 8.0}, 1.0, 0.0, 0.0), now_sec=2.0, free_gib_value=49.99, abort_free_gib=50.0, first_stable_pose=reference, latest_pose=latest)
    failure_path = tmp_path / "validation" / "guard_failure.yaml"
    assert exit_code == 2
    assert yaml.safe_load(failure_path.read_text())["reasons"] == payload["reasons"]
    assert not list(failure_path.parent.glob(".guard_failure.yaml.*.tmp"))


@pytest.mark.parametrize("failed_targets", [{"health.yaml"}, {"guard_failure.yaml"}, {"health.yaml", "guard_failure.yaml"}])
def test_write_failures_are_safety_violations(failed_targets):
    tmp_path = Path.cwd() / ".superpowers" / "sdd" / ("task5-write-" + uuid.uuid4().hex)
    tmp_path.mkdir(parents=True)
    calls = []
    def writer(path, payload):
        calls.append(path.name)
        if path.name in failed_targets:
            raise OSError(path.name + " unavailable")
    code, payload = evaluate_and_record(session_dir=tmp_path, window=HealthWindow({"lidar": 1.0}, 1.0, 0.0, 0.0), now_sec=0.0, free_gib_value=100.0, writer=writer)
    assert code == 2
    assert any("write failed" in reason for reason in payload["reasons"])
    assert payload["write_errors"]
    assert "health.yaml" in calls
    assert "guard_failure.yaml" in calls

def test_source_stamp_reset_after_valid_stamp_is_reported_and_recovers():
    window = HealthWindow({"lidar": 1.0}, 1.0, 0.0, 0.0)
    window.observe("lidar", 1.0, sensor_stamp_sec=100.0)
    window.observe("lidar", 2.0, sensor_stamp_sec=None)
    assert any("source timestamp reset/invalid" in error for error in window.evaluate(2.0))
    window.observe("lidar", 3.0, sensor_stamp_sec=101.0)
    assert not any("source timestamp reset/invalid" in error for error in window.evaluate(3.0))


def test_main_shuts_down_only_after_spin_once_returns(monkeypatch):
    import go1_mapping.session_guard as guard_module
    events = []
    class FakeRclpy:
        def __init__(self): self.running = True
        def init(self, args=None): events.append("init")
        def ok(self): return self.running
        def spin_once(self, node, timeout_sec): events.append("spin_once"); node.timer()
        def shutdown(self): events.append("shutdown"); self.running = False
    class FakeGuard:
        def __init__(self): self.node = self; self.exit_code = 0
        def timer(self): events.append("callback"); self.exit_code = 2
        def destroy_node(self): events.append("destroy")
    assert guard_module.main(rclpy_module=FakeRclpy(), guard_factory=FakeGuard) == 2
    assert events == ["init", "spin_once", "callback", "destroy", "shutdown"]

def test_main_treats_keyboard_interrupt_as_clean_shutdown():
    import go1_mapping.session_guard as guard_module
    events = []
    class FakeRclpy:
        def __init__(self): self.running = True
        def init(self, args=None): events.append("init")
        def ok(self): return self.running
        def spin_once(self, node, timeout_sec): raise KeyboardInterrupt
        def shutdown(self): events.append("shutdown"); self.running = False
    class FakeGuard:
        def __init__(self): self.node = self; self.exit_code = 0
        def destroy_node(self): events.append("destroy")
    assert guard_module.main(rclpy_module=FakeRclpy(), guard_factory=FakeGuard) == 0
    assert events == ["init", "destroy", "shutdown"]


def test_rosbag_health_window_uses_recorded_message_rates():
    tmp_path = Path.cwd() / ".superpowers" / "sdd" / ("task5-bag-" + uuid.uuid4().hex)
    bag_dir = tmp_path / "bag" / "raw"
    bag_dir.mkdir(parents=True)
    connection = sqlite3.connect(bag_dir / "raw_0.db3")
    connection.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER);"
    )
    names = ["/livox/lidar", "/livox/imu", "/Odometry"]
    now_ns = time.time_ns()
    for topic_id, name in enumerate(names, 1):
        connection.execute("INSERT INTO topics VALUES(?, ?)", (topic_id, name))
    rows = []
    for topic_id, hz in enumerate((10, 200, 10), 1):
        step_ns = 1_000_000_000 // hz
        rows.extend(
            (topic_id, now_ns - 10_000_000_000 + index * step_ns)
            for index in range(hz * 10)
        )
    connection.executemany(
        "INSERT INTO messages(topic_id, timestamp) VALUES(?, ?)", rows
    )
    connection.commit()
    connection.close()

    window = RosbagHealthWindow(
        tmp_path, REQUIRED_HZ, 1.0, 0.0, time.monotonic() - 20.0
    )
    measurements = window.measurements(time.monotonic())
    assert measurements["rates_hz"] == pytest.approx(
        {"lidar": 10.0, "imu": 200.0, "odom": 10.0}, rel=0.01
    )
    assert window.evaluate(time.monotonic()) == []