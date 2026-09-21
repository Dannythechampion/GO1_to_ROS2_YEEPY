#!/usr/bin/env python3
"""Replay a recorded localization session against the map, without ROS.

The field Jetson is only reachable on site. Everything the localization
supervisor decides can nevertheless be re-derived afterwards from the session
rosbag (sqlite3 + CDR) and the map, using the same omx_navigation code that
runs on the robot. This is how the 2026-09-18 drift was measured and how the
drift policy was validated; rerun it on every new session bag.

  python3 tools/replay_localization.py overlap BAG_DB3
  python3 tools/replay_localization.py drift BAG_DB3
  python3 tools/replay_localization.py export-checks BAG_DB3 --out checks.csv

`overlap` reproduces the supervisor's continuous overlap per minute.
`drift` runs the drift monitor as the supervisor would (every 2 s while
tracking) and scores every correction it would issue on the scans that follow:
a correction is only good if later scans match the map better with it.
`export-checks` writes the refinement results the drift test fixture uses.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from bisect import bisect_right
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "omx_navigation"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from rosbag_lite import Cdr, Session as BagSession  # noqa: E402

from omx_navigation.drift_monitor import DriftAction, DriftCheck, DriftMonitor, DriftPolicy  # noqa: E402
from omx_navigation.pose_tracking import compose_pose, invert_pose  # noqa: E402
from omx_navigation.ros_conversions import laser_ranges_to_points  # noqa: E402
from omx_navigation.scan_map_quality import (  # noqa: E402
    GridMap,
    Pose2D,
    build_distance_field,
    refine_pose_locally,
    score_pose,
)

DEFAULT_MAP = ROOT / "maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml"
MAX_SCAN_POINTS = 180
HIT_DISTANCE = 0.25


class Session(BagSession):
    """The session bag plus the scan decoding localization replay needs."""

    def transforms(self):
        for received, stamp, parent, child, pose in super().transforms():
            yield received, stamp, parent, child, Pose2D(pose.x, pose.y, pose.yaw)

    def scans(self):
        for received, data in self.messages("/scan"):
            reader = Cdr(data)
            stamp, _frame = reader.header()
            angle_min, _angle_max, angle_increment, _time_increment, _scan_time, range_min, range_max = (
                reader.f32() for _ in range(7)
            )
            ranges = reader.f32_sequence()
            yield received, stamp, laser_ranges_to_points(
                ranges, angle_min, angle_increment, range_min, range_max, MAX_SCAN_POINTS
            )


class LatestTransform:
    """What the supervisor sees: the newest received value of one TF edge."""

    def __init__(self):
        self.received: list[float] = []
        self.poses: list[Pose2D] = []

    def add(self, received: float, pose: Pose2D) -> None:
        self.received.append(received)
        self.poses.append(pose)

    def at(self, received: float) -> Pose2D | None:
        index = bisect_right(self.received, received) - 1
        return None if index < 0 else self.poses[index]


def load_map(yaml_path: Path) -> GridMap:
    """Reproduce nav2_map_server's trinary PGM loading, row flip included."""
    meta = {}
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            meta[key.strip()] = value.strip()
    image = (yaml_path.parent / meta["image"]).read_bytes()
    magic, size, maxval, pixels = image.split(b"\n", 3)
    width, height = map(int, size.split())
    if magic != b"P5" or int(maxval) != 255 or len(pixels) != width * height:
        raise SystemExit(f"unsupported map image {meta['image']}")
    occupied, free = float(meta["occupied_thresh"]), float(meta["free_thresh"])
    negate = int(meta["negate"])
    origin = [float(value) for value in meta["origin"].strip("[]").split(",")]
    cells = [0] * (width * height)
    for row in range(height):
        base = (height - row - 1) * width
        for column in range(width):
            shade = pixels[row * width + column] / 255.0
            occupancy = shade if negate else 1.0 - shade
            cells[base + column] = 100 if occupancy > occupied else (0 if occupancy < free else -1)
    return GridMap(width, height, float(meta["resolution"]), origin[0], origin[1], origin[2], tuple(cells))


def tracked_checks(session: Session, grid: GridMap, field, period: float):
    """Yield (received, points, tracked map->base, camera_init->base) every `period` s."""
    map_camera, camera_base = LatestTransform(), LatestTransform()
    for received, _stamp, parent, child, pose in session.transforms():
        if parent == "map" and child == "camera_init":
            map_camera.add(received, pose)
        elif parent == "camera_init" and child == "body_nav":
            camera_base.add(received, pose)
    last = -math.inf
    for received, _stamp, points in session.scans():
        if received - last < period:
            continue
        first, second = map_camera.at(received), camera_base.at(received)
        if first is None or second is None:
            continue
        last = received
        yield received, points, compose_pose(first, second), second


def ready_windows(session: Session) -> list[tuple[float, float]]:
    """READY spans from the supervisor's own status topic."""
    import json

    spans, start = [], None
    for received, data in session.messages("/localization_supervisor/status"):
        state = json.loads(Cdr(data).string())["state"]
        stamp = received
        if state == "READY" and start is None:
            start = stamp
        elif state != "READY" and start is not None:
            spans.append((start, stamp))
            start = None
    if start is not None:
        spans.append((start, math.inf))
    return spans


def command_overlap(args) -> None:
    session, grid = Session(args.bag), load_map(args.map)
    field = build_distance_field(grid)
    per_minute: dict[int, list[float]] = {}
    first = None
    for received, points, tracked, _camera in tracked_checks(session, grid, field, 0.5):
        first = received if first is None else first
        quality = score_pose(grid, field, points, tracked, HIT_DISTANCE, disqualify_outside=False)
        per_minute.setdefault(int((received - first) // 60), []).append(quality.overlap)
    for minute, values in sorted(per_minute.items()):
        print(f"minute {minute:3d}: overlap mean {statistics.mean(values):.3f} (n={len(values)})")


def command_export_checks(args) -> None:
    session, grid = Session(args.bag), load_map(args.map)
    field = build_distance_field(grid)
    spans = ready_windows(session)
    with open(args.out, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "t", "gap", "tracked_x", "tracked_y", "tracked_yaw", "refined_x", "refined_y", "refined_yaw",
            "refined_overlap", "camera_x", "camera_y", "camera_yaw",
        ))
        first = None
        for received, points, tracked, camera in tracked_checks(session, grid, field, args.period):
            if not any(start <= received < end for start, end in spans):
                continue
            first = received if first is None else first
            result = refine_pose_locally(grid, field, points, tracked)
            best = result.best.pose
            writer.writerow([f"{received - first:.3f}"] + [f"{value:.6f}" for value in (
                result.gap, tracked.x, tracked.y, tracked.yaw, best.x, best.y, best.yaw,
                result.best.overlap, camera.x, camera.y, camera.yaw,
            )])
    print(f"wrote {args.out}")


def command_drift(args) -> None:
    session, grid = Session(args.bag), load_map(args.map)
    field = build_distance_field(grid)
    spans = ready_windows(session)
    scans = list(session.scans())
    map_camera, camera_base = LatestTransform(), LatestTransform()
    for received, _stamp, parent, child, pose in session.transforms():
        if parent == "map" and child == "camera_init":
            map_camera.add(received, pose)
        elif parent == "camera_init" and child == "body_nav":
            camera_base.add(received, pose)

    def future_overlap(start: float, corrected_map_camera: Pose2D) -> tuple[float, float]:
        without, with_correction = [], []
        for received, _stamp, points in scans:
            if not start < received <= start + args.horizon:
                continue
            first, second = map_camera.at(received), camera_base.at(received)
            without.append(score_pose(grid, field, points, compose_pose(first, second), HIT_DISTANCE, disqualify_outside=False).overlap)
            with_correction.append(score_pose(grid, field, points, compose_pose(corrected_map_camera, second), HIT_DISTANCE, disqualify_outside=False).overlap)
        return statistics.mean(without), statistics.mean(with_correction)

    monitor = DriftMonitor(DriftPolicy())
    first, improved, worse, corrections = None, 0, 0, 0
    for received, points, tracked, camera in tracked_checks(session, grid, field, 2.0):
        if not any(start <= received < end for start, end in spans):
            continue
        first = received if first is None else first
        result = refine_pose_locally(grid, field, points, tracked)
        decision = monitor.observe(DriftCheck(received, result.gap, tracked, result.best.pose, result.best.overlap, camera))
        if decision.action is DriftAction.CORRECT:
            corrections += 1
            before, after = future_overlap(received, compose_pose(decision.correction, invert_pose(camera)))
            improved += after > before + 0.01
            worse += after < before - 0.01
            print(f"t+{received - first:7.1f} CORRECT  next {args.horizon:.0f}s overlap {before:.3f} -> {after:.3f}")
        elif decision.action is DriftAction.ESCALATE:
            print(f"t+{received - first:7.1f} ESCALATE ({decision.reason})")
    print(f"corrections: {corrections}, improved later scans: {improved}, made them worse: {worse}")
    print("note: a replay cannot apply a correction, so drift that one correction")
    print("would have cleared keeps reappearing here and eventually escalates.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("overlap", "drift", "export-checks"):
        command = sub.add_parser(name)
        command.add_argument("bag", type=Path, help="rosbag_0.db3 of a posegraph session")
        command.add_argument("--map", type=Path, default=DEFAULT_MAP)
        if name == "export-checks":
            command.add_argument("--out", type=Path, required=True)
            command.add_argument("--period", type=float, default=2.0)
        if name == "drift":
            command.add_argument("--horizon", type=float, default=20.0)
    args = parser.parse_args()
    {"overlap": command_overlap, "drift": command_drift, "export-checks": command_export_checks}[args.command](args)


if __name__ == "__main__":
    main()
