#!/usr/bin/env python3
"""Patch pinned FAST-LIO source with observable, bounded latency behavior."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


VALID_MODES = {"diagnostic", "bounded"}
DIAGNOSTIC_MARKER = "FAST_LIO_REALTIME_DIAGNOSTICS"
BOUNDED_MARKER = "FAST_LIO_BOUNDED_BUFFER"

OLD_QOS = "(lid_topic, 20, livox_pcl_cbk);"
INTERMEDIATE_QOS = "(lid_topic, rclcpp::QoS(rclcpp::KeepLast(1)), livox_pcl_cbk);"
SENSOR_QOS = "(lid_topic, rclcpp::SensorDataQoS().keep_last(1), livox_pcl_cbk);"

DEQUE_BLOCK = """deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
"""

DIAGNOSTIC_GLOBALS = """deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

// FAST_LIO_REALTIME_DIAGNOSTICS
size_t fast_lio_lidar_drop_count = 0;
double fast_lio_last_process_ms = 0.0;
double fast_lio_max_process_ms = 0.0;

// Caller must hold mtx_buffer.
void clear_lidar_buffers_locked()
{
    fast_lio_lidar_drop_count += lidar_buffer.size();
    lidar_buffer.clear();
    time_buffer.clear();
    lidar_pushed = false;
}
"""

SYNC_START = """bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
"""

LOCKED_SYNC_START = """bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
"""

TIMER_START = """    void timer_callback()
    {
        if(sync_packages(Measures))
"""

INSTRUMENTED_TIMER_START = """    void timer_callback()
    {
        const double fast_lio_process_start = omp_get_wtime();
        auto fast_lio_process_guard = std::shared_ptr<void>(
            nullptr, [fast_lio_process_start](void *) {
                fast_lio_last_process_ms =
                    (omp_get_wtime() - fast_lio_process_start) * 1000.0;
                if (fast_lio_last_process_ms > fast_lio_max_process_ms) {
                    fast_lio_max_process_ms = fast_lio_last_process_ms;
                }
            });
        if(sync_packages(Measures))
"""

MAP_TIMER_SETUP = """        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));
"""

DIAGNOSTIC_TIMER_SETUP = """        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        auto realtime_diagnostics_period_ms = std::chrono::milliseconds(1000);
        realtime_diagnostics_timer_ = rclcpp::create_timer(
            this, this->get_clock(), realtime_diagnostics_period_ms,
            std::bind(&LaserMappingNode::realtime_diagnostics_callback, this));
"""

MAP_CALLBACK_START = """    void map_publish_callback()
    {
"""

DIAGNOSTIC_CALLBACK = """    void realtime_diagnostics_callback()
    {
        size_t queue_depth = 0;
        size_t drop_count = 0;
        double front_age = 0.0;
        double imu_margin = 0.0;
        double process_ms = 0.0;
        double max_process_ms = 0.0;
        const double ros_now = this->get_clock()->now().seconds();
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            queue_depth = lidar_buffer.size();
            drop_count = fast_lio_lidar_drop_count;
            if (!time_buffer.empty()) {
                const double measured_age = ros_now - time_buffer.front();
                front_age = measured_age > 0.0 ? measured_age : 0.0;
            }
            imu_margin = last_timestamp_imu - lidar_end_time;
            process_ms = fast_lio_last_process_ms;
            max_process_ms = fast_lio_max_process_ms;
        }
        const char *format =
            "[fast_lio_realtime] queue_depth=%zu front_age=%.6f drops=%zu "
            "process_ms=%.3f max_process_ms=%.3f imu_margin=%.6f";
        if (queue_depth > 2 || front_age > 0.30) {
            RCLCPP_WARN(
                this->get_logger(), format, queue_depth, front_age, drop_count,
                process_ms, max_process_ms, imu_margin);
        } else {
            RCLCPP_INFO(
                this->get_logger(), format, queue_depth, front_age, drop_count,
                process_ms, max_process_ms, imu_margin);
        }
    }

    void map_publish_callback()
    {
"""

TIMER_MEMBERS = """    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
"""

DIAGNOSTIC_TIMER_MEMBERS = """    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::TimerBase::SharedPtr realtime_diagnostics_timer_;
"""


def replace_exactly_once(source: str, old: str, new: str, label: str) -> str:
    count = source.count(old)
    if count != 1:
        raise RuntimeError(f"expected exactly one {label}, found {count}")
    return source.replace(old, new, 1)


def ensure_sensor_qos(source: str) -> str:
    if SENSOR_QOS in source:
        return source
    candidates = [value for value in (OLD_QOS, INTERMEDIATE_QOS) if value in source]
    if len(candidates) != 1 or source.count(candidates[0]) != 1:
        raise RuntimeError("expected exactly one FAST-LIO Livox subscription")
    return source.replace(candidates[0], SENSOR_QOS, 1)


def add_diagnostics(source: str) -> str:
    source = ensure_sensor_qos(source)
    source = replace_exactly_once(
        source, DEQUE_BLOCK, DIAGNOSTIC_GLOBALS, "FAST-LIO deque declaration block"
    )

    clear_line = "        lidar_buffer.clear();"
    clear_count = source.count(clear_line)
    if clear_count != 2:
        raise RuntimeError(
            f"expected exactly two LiDAR callback reset sites, found {clear_count}"
        )
    source = source.replace(clear_line, "        clear_lidar_buffers_locked();")

    source = replace_exactly_once(
        source, SYNC_START, LOCKED_SYNC_START, "sync_packages entry"
    )
    source = replace_exactly_once(
        source, TIMER_START, INSTRUMENTED_TIMER_START, "timer_callback entry"
    )
    source = replace_exactly_once(
        source, MAP_TIMER_SETUP, DIAGNOSTIC_TIMER_SETUP, "map timer setup"
    )
    source = replace_exactly_once(
        source, MAP_CALLBACK_START, DIAGNOSTIC_CALLBACK, "map callback entry"
    )
    source = replace_exactly_once(
        source, TIMER_MEMBERS, DIAGNOSTIC_TIMER_MEMBERS, "timer member block"
    )
    return source


def patch(source_path: Path, mode: str = "diagnostic") -> None:
    if mode not in VALID_MODES:
        raise ValueError(f"unsupported mode: {mode}")
    source = source_path.read_text(encoding="utf-8")
    if DIAGNOSTIC_MARKER in source:
        if mode == "diagnostic" or BOUNDED_MARKER in source:
            return
    transformed = add_diagnostics(source)
    if mode == "bounded":
        raise RuntimeError("bounded FAST-LIO patch mode is not implemented yet")
    source_path.write_text(transformed, encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=sorted(VALID_MODES), default="diagnostic")
    parser.add_argument("source", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    patch(args.source, args.mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
