#!/usr/bin/env python3
"""Prevent FAST-LIO startup work from leaving Livox frames queued forever."""

import sys
from pathlib import Path


OLD = "(lid_topic, 20, livox_pcl_cbk);"
INTERMEDIATE = "(lid_topic, rclcpp::QoS(rclcpp::KeepLast(1)), livox_pcl_cbk);"
NEW = "(lid_topic, rclcpp::SensorDataQoS().keep_last(1), livox_pcl_cbk);"


def patch(source_path: Path) -> None:
    source = source_path.read_text(encoding="utf-8")
    if NEW in source:
        return
    candidates = [item for item in (OLD, INTERMEDIATE) if item in source]
    if len(candidates) != 1 or source.count(candidates[0]) != 1:
        raise RuntimeError(f"expected exactly one FAST-LIO Livox subscription in {source_path}")
    source_path.write_text(source.replace(candidates[0], NEW), encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} LASER_MAPPING_CPP")
    patch(Path(sys.argv[1]))
