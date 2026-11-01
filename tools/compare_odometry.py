#!/usr/bin/env python3
"""Did the map pose keep up with how far the robot actually walked?

    python3 tools/compare_odometry.py <rosbag dir> <decisions.jsonl> [--start T] [--end T]

Compares four measures of the same walk over a window (default: the first
`navigate` event in the decision log to the next hand-over, dock or abort):

- FAST-LIO odometry (`/Odometry`, camera_init frame),
- the Go1's own leg odometry (`/go1/robot_state` velocity, integrated),
- the commanded distance (`applied` in `/go1/robot_state`),
- the map-frame pose the mission logged every cycle (`decisions.jsonl`).

On 2026-11-01 the first three agreed within 2% over 8 m while the map pose
had moved 3.5 m: localization was pulling the robot back along the corridor,
and the supervisor stayed READY throughout because a corridor scan matches
equally well anywhere along it. This is the check that would have caught it.

Needs no ROS: reads the sqlite3 bag directly.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sqlite3
import struct
import sys


def _odometry_xy(blob: bytes):
    """Position of a CDR-serialized nav_msgs/Odometry, without rclpy."""
    position = 4 + 8                                     # encapsulation, stamp
    (length,) = struct.unpack_from("<I", blob, position)
    position += 4 + length                               # frame_id
    position = 4 + ((position - 4 + 3) // 4) * 4
    (length,) = struct.unpack_from("<I", blob, position)
    position += 4 + length                               # child_frame_id
    position = 4 + ((position - 4 + 7) // 8) * 8
    x, y, _z = struct.unpack_from("<3d", blob, position)
    return x, y


def _string_json(blob: bytes):
    """A std_msgs/String holding JSON."""
    return json.loads(blob[8:].split(b"\x00")[0])


def read_bag(bag: str, start: float, end: float):
    odometry, states = [], []
    for path in sorted(glob.glob(bag.rstrip("/") + "/*.db3")):
        connection = sqlite3.connect(path)
        topics = {name: ident for ident, name in connection.execute("select id, name from topics")}
        for name, sink in (("/Odometry", odometry), ("/go1/robot_state", states)):
            if name not in topics:
                continue
            for stamp, data in connection.execute(
                "select timestamp, data from messages where topic_id=? and timestamp between ? and ?",
                (topics[name], int((start - 1) * 1e9), int(end * 1e9)),
            ):
                try:
                    value = _odometry_xy(bytes(data)) if name == "/Odometry" else _string_json(bytes(data))
                except (struct.error, ValueError):
                    continue
                sink.append((stamp / 1e9, value))
    odometry.sort(key=lambda item: item[0])
    states.sort(key=lambda item: item[0])
    return odometry, states


def window_from_decisions(records):
    events = [r for r in records if r.get("kind") == "event"]
    start = next((e["t"] for e in events if e.get("event") == "navigate"), None)
    if start is None:
        return None, None
    end = next((e["t"] for e in events if e["t"] > start and e.get("event") in
                ("handoff", "dock_started", "aborted", "nav_paused", "mission_done")), None)
    return start, end


def net(points):
    return math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1]) if len(points) > 1 else 0.0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag")
    parser.add_argument("decisions")
    parser.add_argument("--start", type=float)
    parser.add_argument("--end", type=float)
    parser.add_argument("--step", type=float, default=6.0, help="seconds between rows of the lag table")
    args = parser.parse_args(argv)

    records = []
    with open(args.decisions, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            try:
                records.append(json.loads(line))
            except ValueError:
                continue
    start, end = window_from_decisions(records)
    start = args.start if args.start is not None else start
    end = args.end if args.end is not None else end
    if start is None or end is None:
        print("no window: pass --start and --end")
        return 2

    odometry, states = read_bag(args.bag, start, end)
    poses = [(r["t"], r["robot"]) for r in records
             if r.get("kind") == "cycle" and r.get("robot") and start <= r["t"] <= end]
    if not odometry or not poses:
        print(f"not enough data: {len(odometry)} odometry, {len(poses)} map poses")
        return 1

    leg = command = 0.0
    legs = [(start, 0.0)]
    for (t_a, a), (t_b, _b) in zip(states, states[1:]):
        dt = t_b - t_a
        if dt <= 0 or dt > 0.5:
            continue
        velocity = (a.get("robot") or {}).get("velocity") or [0.0, 0.0]
        leg += math.hypot(*velocity) * dt
        applied = a.get("applied") or {}
        command += math.hypot(applied.get("vx", 0.0), applied.get("vy", 0.0)) * dt
        legs.append((t_b, leg))

    print(f"window {end - start:.1f} s")
    print(f"  FAST-LIO odometry  {net([v for _t, v in odometry]):6.2f} m net")
    print(f"  Go1 leg odometry   {leg:6.2f} m walked" + ("" if states else "  (no /go1/robot_state)"))
    print(f"  commanded          {command:6.2f} m")
    print(f"  map pose           {net([(p[0], p[1]) for _t, p in poses]):6.2f} m net")
    print("\n  t (s)   FAST-LIO   legs    map    map lags FAST-LIO by")
    t = start
    while t <= end + 1e-6:
        o = [v for s, v in odometry if s <= t]
        m = [(p[0], p[1]) for s, p in poses if s <= t]
        g = [v for s, v in legs if s <= t]
        if len(o) > 1 and len(m) > 1:
            print(f"  {t - start:5.1f}  {net(o):7.2f}  {g[-1] if g else 0.0:6.2f}  {net(m):6.2f}   {net(o) - net(m):+6.2f} m")
        t += args.step
    lag = net([v for _t, v in odometry]) - net([(p[0], p[1]) for _t, p in poses])
    if lag > 0.5:
        print(f"\nThe map pose fell {lag:.2f} m behind odometry: localization was not following the robot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
