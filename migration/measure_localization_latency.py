#!/usr/bin/env python3
"""Measure ROS header age without subscribing to large Livox point messages."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Deque, Dict, Optional


def percentile(values: list[float], quantile: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    rank = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[rank]


class TopicTracker:
    def __init__(self) -> None:
        self.ages: list[float] = []
        self.received: list[float] = []
        self.last_header_sec: Optional[float] = None
        self.reversals = 0
        self.future_stamps = 0

    def add(
        self, *, received_monotonic: float, header_sec: float, ros_now_sec: float
    ) -> None:
        age = ros_now_sec - header_sec
        if age < 0.0:
            self.future_stamps += 1
        if self.last_header_sec is not None and header_sec < self.last_header_sec:
            self.reversals += 1
        self.ages.append(age)
        self.received.append(received_monotonic)
        self.last_header_sec = header_sec

    def summary(self) -> dict:
        count = len(self.received)
        elapsed = self.received[-1] - self.received[0] if count >= 2 else 0.0
        rate = (count - 1) / elapsed if elapsed > 0.0 else 0.0
        return {
            "count": count,
            "rate_hz": rate,
            "age_p95_sec": percentile(self.ages, 0.95),
            "age_max_sec": max(self.ages) if self.ages else None,
            "reversals": self.reversals,
            "future_stamps": self.future_stamps,
        }


@dataclass(frozen=True)
class TopicLimits:
    rate_min: float = 9.0
    rate_max: float = 11.0
    age_p95: float = 0.10
    age_max: float = 0.30


def evaluate_topic(summary: dict, limits: TopicLimits) -> dict:
    failures: list[str] = []
    if summary["count"] < 2:
        failures.append("insufficient_samples")
    if summary["rate_hz"] < limits.rate_min:
        failures.append("rate_below_min")
    if summary["rate_hz"] > limits.rate_max:
        failures.append("rate_above_max")
    age_p95 = summary["age_p95_sec"]
    if age_p95 is None or age_p95 > limits.age_p95:
        failures.append("age_p95_above_max")
    age_max = summary["age_max_sec"]
    if age_max is None or age_max > limits.age_max:
        failures.append("age_max_above_max")
    if summary["reversals"]:
        failures.append("timestamp_reversal")
    if summary["future_stamps"]:
        failures.append("future_timestamp")
    return {"passed": not failures, "failures": failures}


FAST_LIO_DIAGNOSTIC_PATTERN = re.compile(
    r"\[fast_lio_realtime\] "
    r"queue_depth=(?P<queue_depth>\d+) "
    r"front_age=(?P<front_age>-?[0-9.]+) "
    r"drops=(?P<drops>\d+) "
    r"process_ms=(?P<process_ms>[0-9.]+) "
    r"max_process_ms=(?P<max_process_ms>[0-9.]+) "
    r"imu_margin=(?P<imu_margin>-?[0-9.]+)"
)


def parse_fast_lio_diagnostics(lines: list[str]) -> dict:
    samples = []
    for line in lines:
        match = FAST_LIO_DIAGNOSTIC_PATTERN.search(line)
        if match is None:
            continue
        samples.append(
            {
                "queue_depth": int(match.group("queue_depth")),
                "front_age": float(match.group("front_age")),
                "drops": int(match.group("drops")),
                "process_ms": float(match.group("process_ms")),
                "max_process_ms": float(match.group("max_process_ms")),
                "imu_margin": float(match.group("imu_margin")),
            }
        )
    if not samples:
        return {
            "count": 0,
            "queue_depth_first": None,
            "queue_depth_last": None,
            "queue_depth_max": None,
            "front_age_max_sec": None,
            "drops_last": None,
            "process_p95_ms": None,
            "process_max_ms": None,
            "imu_margin_min_sec": None,
        }
    return {
        "count": len(samples),
        "queue_depth_first": samples[0]["queue_depth"],
        "queue_depth_last": samples[-1]["queue_depth"],
        "queue_depth_max": max(sample["queue_depth"] for sample in samples),
        "front_age_max_sec": max(sample["front_age"] for sample in samples),
        "drops_last": samples[-1]["drops"],
        "process_p95_ms": percentile(
            [sample["process_ms"] for sample in samples], 0.95
        ),
        "process_max_ms": max(sample["process_ms"] for sample in samples),
        "imu_margin_min_sec": min(sample["imu_margin"] for sample in samples),
    }


def evaluate_fast_lio_diagnostics(summary: dict) -> dict:
    failures: list[str] = []
    if summary["count"] == 0:
        failures.append("diagnostics_missing")
    if summary["queue_depth_max"] is not None and summary["queue_depth_max"] > 2:
        failures.append("queue_depth_above_max")
    if (
        summary["front_age_max_sec"] is not None
        and summary["front_age_max_sec"] > 0.30
    ):
        failures.append("front_age_above_max")
    if (
        summary["process_p95_ms"] is not None
        and summary["process_p95_ms"] > 100.0
    ):
        failures.append("process_p95_above_max")
    return {"passed": not failures, "failures": failures}


class StationaryTracker:
    def __init__(self, *, max_linear: float = 0.01, max_angular: float = 0.01) -> None:
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.count = 0
        self.observed_linear = 0.0
        self.observed_angular = 0.0

    def add(self, *, linear_x: float, linear_y: float, angular_z: float) -> None:
        self.count += 1
        self.observed_linear = max(
            self.observed_linear, math.hypot(linear_x, linear_y)
        )
        self.observed_angular = max(self.observed_angular, abs(angular_z))

    def summary(self) -> dict:
        failures: list[str] = []
        if self.count == 0:
            failures.append("insufficient_samples")
        if self.observed_linear > self.max_linear:
            failures.append("linear_speed_above_max")
        if self.observed_angular > self.max_angular:
            failures.append("angular_speed_above_max")
        return {
            "count": self.count,
            "max_linear_speed": self.observed_linear,
            "max_angular_speed": self.observed_angular,
            "passed": not failures,
            "failures": failures,
        }


@dataclass(frozen=True)
class AmclSample:
    x: float
    y: float
    yaw: float
    cov_x: float
    cov_y: float
    cov_yaw: float


class AmclWindow:
    def __init__(
        self,
        *,
        limit: int = 10,
        max_position_spread: float = 0.10,
        max_yaw_spread: float = math.radians(5.0),
        max_cov_x: float = 0.04,
        max_cov_y: float = 0.04,
        max_cov_yaw: float = 0.0305,
    ) -> None:
        if limit < 2:
            raise ValueError("limit must be at least 2")
        self.limit = limit
        self.samples: Deque[AmclSample] = deque(maxlen=limit)
        self.max_position_spread = max_position_spread
        self.max_yaw_spread = max_yaw_spread
        self.max_cov_x = max_cov_x
        self.max_cov_y = max_cov_y
        self.max_cov_yaw = max_cov_yaw

    def add(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        cov_x: float,
        cov_y: float,
        cov_yaw: float,
    ) -> None:
        values = (x, y, yaw, cov_x, cov_y, cov_yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("AMCL sample values must be finite")
        self.samples.append(AmclSample(*values))

    def summary(self) -> dict:
        samples = list(self.samples)
        if samples:
            xs = [sample.x for sample in samples]
            ys = [sample.y for sample in samples]
            position_spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            sin_mean = sum(math.sin(sample.yaw) for sample in samples) / len(samples)
            cos_mean = sum(math.cos(sample.yaw) for sample in samples) / len(samples)
            mean_yaw = math.atan2(sin_mean, cos_mean)
            yaw_spread = max(
                abs((sample.yaw - mean_yaw + math.pi) % (2.0 * math.pi) - math.pi)
                for sample in samples
            )
            cov_x_max = max(sample.cov_x for sample in samples)
            cov_y_max = max(sample.cov_y for sample in samples)
            cov_yaw_max = max(sample.cov_yaw for sample in samples)
        else:
            position_spread = 0.0
            yaw_spread = 0.0
            cov_x_max = cov_y_max = cov_yaw_max = 0.0

        failures: list[str] = []
        if len(samples) < self.limit:
            failures.append("insufficient_samples")
        if cov_x_max > self.max_cov_x:
            failures.append("covariance_x_above_max")
        if cov_y_max > self.max_cov_y:
            failures.append("covariance_y_above_max")
        if cov_yaw_max > self.max_cov_yaw:
            failures.append("covariance_yaw_above_max")
        if position_spread > self.max_position_spread:
            failures.append("position_spread_above_max")
        if yaw_spread > self.max_yaw_spread:
            failures.append("yaw_spread_above_max")
        return {
            "count": len(samples),
            "covariance_x_max": cov_x_max,
            "covariance_y_max": cov_y_max,
            "covariance_yaw_max": cov_yaw_max,
            "position_spread_m": position_spread,
            "yaw_spread_rad": yaw_spread,
            "passed": not failures,
            "failures": failures,
        }


try:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except ImportError:  # Pure statistics remain testable without ROS installed.
    rclpy = None
    Node = object


def stamp_seconds(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


if rclpy is not None:

    class LatencyProbe(Node):
        def __init__(self, *, mode: str, record_start: float) -> None:
            super().__init__("localization_latency_probe")
            self.mode = mode
            self.record_start = record_start
            self.topics: Dict[str, TopicTracker] = {
                "/Odometry": TopicTracker(),
                "/scan": TopicTracker(),
            }
            if mode == "amcl":
                self.topics["/amcl_pose"] = TopicTracker()
            self.stationary = StationaryTracker()
            self.amcl = AmclWindow(limit=10)
            self.create_subscription(
                Odometry, "/Odometry", self._odom_callback, qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
            )
            if mode == "amcl":
                self.create_subscription(
                    PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, 10
                )

        def _record_topic(self, name: str, message) -> bool:
            received = time.monotonic()
            if received < self.record_start:
                return False
            now = self.get_clock().now().nanoseconds * 1e-9
            self.topics[name].add(
                received_monotonic=received,
                header_sec=stamp_seconds(message),
                ros_now_sec=now,
            )
            return True

        def _odom_callback(self, message: Odometry) -> None:
            if not self._record_topic("/Odometry", message):
                return
            twist = message.twist.twist
            self.stationary.add(
                linear_x=twist.linear.x,
                linear_y=twist.linear.y,
                angular_z=twist.angular.z,
            )

        def _scan_callback(self, message: LaserScan) -> None:
            self._record_topic("/scan", message)

        def _amcl_callback(self, message: PoseWithCovarianceStamped) -> None:
            if not self._record_topic("/amcl_pose", message):
                return
            pose = message.pose.pose
            q = pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            covariance = message.pose.covariance
            self.amcl.add(
                x=pose.position.x,
                y=pose.position.y,
                yaw=yaw,
                cov_x=covariance[0],
                cov_y=covariance[7],
                cov_yaw=covariance[35],
            )


def build_result(
    *,
    mode: str,
    duration: float,
    warmup: float,
    topic_trackers: Dict[str, TopicTracker],
    stationary: StationaryTracker,
    amcl: AmclWindow,
    limits: TopicLimits,
) -> dict:
    topics = {}
    all_passed = True
    for name, tracker in topic_trackers.items():
        summary = tracker.summary()
        gate = evaluate_topic(summary, limits)
        topics[name] = {**summary, **gate}
        all_passed = all_passed and gate["passed"]
    stationary_result = stationary.summary()
    all_passed = all_passed and stationary_result["passed"]
    result = {
        "mode": mode,
        "duration_sec": duration,
        "warmup_sec": warmup,
        "sample_duration_sec": duration - warmup,
        "limits": {
            "rate_min_hz": limits.rate_min,
            "rate_max_hz": limits.rate_max,
            "age_p95_sec": limits.age_p95,
            "age_max_sec": limits.age_max,
        },
        "topics": topics,
        "stationary": stationary_result,
    }
    if mode == "amcl":
        amcl_result = amcl.summary()
        result["amcl"] = amcl_result
        all_passed = all_passed and amcl_result["passed"]
    result["passed"] = all_passed
    return result


def write_json_atomic(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--warmup", type=float, default=0.0)
    parser.add_argument("--mode", choices=("fast-lio", "amcl"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fast-lio-log", type=Path, required=True)
    parser.add_argument("--log-start-line", type=int, default=0)
    parser.add_argument("--age-p95", type=float, default=0.10)
    parser.add_argument("--age-max", type=float, default=0.30)
    parser.add_argument("--rate-min", type=float, default=9.0)
    parser.add_argument("--rate-max", type=float, default=11.0)
    parser.add_argument("--observe-only", action="store_true")
    args = parser.parse_args(argv)
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.warmup < 0.0 or args.warmup >= args.duration:
        parser.error("--warmup must be non-negative and less than --duration")
    if args.log_start_line < 0:
        parser.error("--log-start-line must be non-negative")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are required")
    limits = TopicLimits(
        rate_min=args.rate_min,
        rate_max=args.rate_max,
        age_p95=args.age_p95,
        age_max=args.age_max,
    )
    started = time.monotonic()
    rclpy.init()
    node = LatencyProbe(mode=args.mode, record_start=started + args.warmup)
    try:
        deadline = started + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = build_result(
            mode=args.mode,
            duration=args.duration,
            warmup=args.warmup,
            topic_trackers=node.topics,
            stationary=node.stationary,
            amcl=node.amcl,
            limits=limits,
        )
        log_lines = args.fast_lio_log.read_text(
            encoding="utf-8", errors="replace"
        ).splitlines()[args.log_start_line :]
        diagnostic_summary = parse_fast_lio_diagnostics(log_lines)
        diagnostic_gate = evaluate_fast_lio_diagnostics(diagnostic_summary)
        result["fast_lio_diagnostics"] = {
            **diagnostic_summary,
            **diagnostic_gate,
        }
        result["passed"] = result["passed"] and diagnostic_gate["passed"]
        result["observe_only"] = args.observe_only
        write_json_atomic(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["passed"] or args.observe_only else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
