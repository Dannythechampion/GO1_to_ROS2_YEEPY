import math
from pathlib import Path

import pytest

from measure_localization_latency import (
    AmclWindow,
    StationaryTracker,
    TopicLimits,
    TopicTracker,
    evaluate_fast_lio_diagnostics,
    evaluate_topic,
    parse_fast_lio_diagnostics,
)


ROOT = Path(__file__).parents[1]
WRAPPER = ROOT / "migration" / "verify_fast_lio_latency.sh"


def test_topic_tracker_reports_observed_rate_age_and_backward_stamp():
    tracker = TopicTracker()
    tracker.add(received_monotonic=10.1, header_sec=100.00, ros_now_sec=100.02)
    tracker.add(received_monotonic=10.2, header_sec=100.10, ros_now_sec=100.13)
    tracker.add(received_monotonic=10.3, header_sec=100.05, ros_now_sec=100.14)

    result = tracker.summary()

    assert result["count"] == 3
    assert result["rate_hz"] == pytest.approx(10.0)
    assert result["age_p95_sec"] == pytest.approx(0.09)
    assert result["age_max_sec"] == pytest.approx(0.09)
    assert result["reversals"] == 1
    assert result["future_stamps"] == 0


def test_topic_gate_rejects_slow_old_or_future_data():
    limits = TopicLimits(rate_min=9.0, rate_max=11.0, age_p95=0.10, age_max=0.30)

    assert evaluate_topic(
        {
            "count": 20,
            "rate_hz": 10.0,
            "age_p95_sec": 0.05,
            "age_max_sec": 0.20,
            "reversals": 0,
            "future_stamps": 0,
        },
        limits,
    )["passed"] is True
    rejected = evaluate_topic(
        {
            "count": 20,
            "rate_hz": 8.9,
            "age_p95_sec": 0.11,
            "age_max_sec": 0.31,
            "reversals": 1,
            "future_stamps": 1,
        },
        limits,
    )
    assert rejected == {
        "passed": False,
        "failures": [
            "rate_below_min",
            "age_p95_above_max",
            "age_max_above_max",
            "timestamp_reversal",
            "future_timestamp",
        ],
    }


def test_stationary_tracker_rejects_peak_motion_over_limit():
    tracker = StationaryTracker(max_linear=0.01, max_angular=0.01)
    tracker.add(linear_x=0.006, linear_y=0.008, angular_z=0.009)
    tracker.add(linear_x=0.011, linear_y=0.0, angular_z=0.0)

    result = tracker.summary()

    assert result["max_linear_speed"] == pytest.approx(0.011)
    assert result["max_angular_speed"] == pytest.approx(0.009)
    assert result["passed"] is False


def test_amcl_window_accepts_stable_wraparound_yaw_samples():
    window = AmclWindow(limit=10)
    for index in range(10):
        yaw = math.radians(179.0 if index % 2 == 0 else -179.0)
        window.add(
            x=1.0 + index * 0.005,
            y=2.0,
            yaw=yaw,
            cov_x=0.02,
            cov_y=0.02,
            cov_yaw=0.01,
        )

    result = window.summary()

    assert result["count"] == 10
    assert result["position_spread_m"] == pytest.approx(0.045)
    assert result["yaw_spread_rad"] == pytest.approx(math.radians(1.0))
    assert result["passed"] is True


def test_amcl_window_rejects_covariance_and_spread_limits_independently():
    window = AmclWindow(limit=10)
    for index in range(10):
        window.add(
            x=index * 0.02,
            y=0.0,
            yaw=index * 0.02,
            cov_x=0.05 if index == 0 else 0.01,
            cov_y=0.01,
            cov_yaw=0.01,
        )

    result = window.summary()

    assert result["passed"] is False
    assert result["failures"] == [
        "covariance_x_above_max",
        "position_spread_above_max",
        "yaw_spread_above_max",
    ]


def test_fast_lio_log_parser_summarizes_queue_age_and_processing():
    lines = [
        "[INFO] [fast_lio_realtime] queue_depth=1 front_age=0.020000 drops=0 "
        "process_ms=45.000 max_process_ms=45.000 imu_margin=0.005000",
        "[WARN] [fast_lio_realtime] queue_depth=3 front_age=0.350000 drops=2 "
        "process_ms=120.000 max_process_ms=120.000 imu_margin=-0.010000",
    ]

    summary = parse_fast_lio_diagnostics(lines)

    assert summary == {
        "count": 2,
        "queue_depth_first": 1,
        "queue_depth_last": 3,
        "queue_depth_max": 3,
        "front_age_max_sec": pytest.approx(0.35),
        "drops_last": 2,
        "process_p95_ms": pytest.approx(120.0),
        "process_max_ms": pytest.approx(120.0),
        "imu_margin_min_sec": pytest.approx(-0.01),
    }


def test_fast_lio_diagnostic_gate_reports_each_latency_boundary():
    result = evaluate_fast_lio_diagnostics(
        {
            "count": 2,
            "queue_depth_first": 1,
            "queue_depth_last": 3,
            "queue_depth_max": 3,
            "front_age_max_sec": 0.35,
            "drops_last": 2,
            "process_p95_ms": 120.0,
            "process_max_ms": 120.0,
            "imu_margin_min_sec": -0.01,
        }
    )

    assert result == {
        "passed": False,
        "failures": [
            "queue_depth_above_max",
            "front_age_above_max",
            "process_p95_above_max",
        ],
    }


def test_shell_wrapper_enforces_disarm_uniqueness_and_evidence_capture():
    text = WRAPPER.read_text(encoding="utf-8")

    assert text.index("source /opt/ros/humble/setup.bash") < text.index(
        "set -euo pipefail"
    )
    assert "ros2 param get /go1_driver arm" in text
    assert "Boolean value is: False" in text
    assert "ros2 topic echo /go1/control_state --once" in text
    assert "DRY-RUN" in text
    assert "ros2 topic info /livox/lidar" in text
    assert "Publisher count: 1" in text
    assert "pgrep -x fastlio_mapping" in text
    assert "--fast-lio-log" in text
    assert "--log-start-line" in text
    assert "measure_localization_latency.py" in text
    assert 'runtime_root="${GO1_RUNTIME_ROOT:-/mnt/t500/go1_runtime}"' in text
    assert 'evidence_root="$runtime_root/latency"' in text


def test_shell_wrapper_restricts_observe_only_and_checks_amcl_log_errors():
    text = WRAPPER.read_text(encoding="utf-8")

    assert "--observe-only is valid only with fast-lio" in text
    assert "Message Filter dropping message" in text
    assert "Timed out waiting for transform" in text
    assert "extrapolation" in text
    assert "fast_lio_pid_after" in text
    assert "FAST-LIO process restarted during measurement" in text
