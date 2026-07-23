import math
from pathlib import Path
import uuid

import yaml

from go1_mapping.session_guard import (
    HealthWindow,
    evaluate_and_record,
    pose_return_error,
    quaternion_yaw,
    should_abort_disk,
)


REQUIRED_HZ = {"lidar": 8.0, "imu": 100.0, "odom": 8.0}


def test_health_window_suppresses_errors_during_initialization():
    window = HealthWindow(
        REQUIRED_HZ, max_gap_sec=1.0, initialization_sec=15.0, started_at=0.0
    )

    assert window.evaluate(10.0) == []


def test_health_window_reports_rate_gap_and_sensor_timestamp_regression():
    window = HealthWindow(
        REQUIRED_HZ, max_gap_sec=1.0, initialization_sec=15.0, started_at=0.0
    )
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


def test_disk_threshold_is_strictly_below_abort_limit():
    assert should_abort_disk(49.99, 50.0) is True
    assert should_abort_disk(50.0, 50.0) is False


def test_health_record_is_atomic_and_pose_error_wraps_yaw():
    tmp_path = Path.cwd() / ".superpowers" / "sdd" / ("task5-health-" + uuid.uuid4().hex)
    tmp_path.mkdir(parents=True)
    reference = {"position": {"x": 1.0, "y": 2.0}, "orientation": {"z": 1.0, "w": 0.0}}
    latest = {
        "position": {"x": 4.0, "y": 6.0},
        "orientation": {"z": math.sin(-math.pi / 2.0), "w": math.cos(-math.pi / 2.0)},
    }

    assert quaternion_yaw(0.0, 0.0, 1.0, 0.0) == pytest_approx(math.pi)
    assert pose_return_error(reference, latest) == {
        "xy_m": pytest_approx(5.0),
        "yaw_rad": pytest_approx(0.0),
    }

    exit_code, payload = evaluate_and_record(
        session_dir=tmp_path,
        window=HealthWindow({"lidar": 8.0}, 1.0, 0.0, 0.0),
        now_sec=2.0,
        free_gib_value=49.99,
        abort_free_gib=50.0,
        first_stable_pose=reference,
        latest_pose=latest,
    )

    failure_path = tmp_path / "validation" / "guard_failure.yaml"
    assert exit_code == 2
    assert payload["reasons"]
    assert yaml.safe_load(failure_path.read_text())["reasons"] == payload["reasons"]
    assert not list(failure_path.parent.glob(".guard_failure.yaml.*.tmp"))


def pytest_approx(value):
    import pytest

    return pytest.approx(value)
