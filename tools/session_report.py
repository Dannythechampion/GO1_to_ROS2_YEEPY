#!/usr/bin/env python3
"""Summarise one field session from its rosbag, without ROS.

  python3 tools/session_report.py /mnt/t500/localization_logs/<session>/rosbag/rosbag_0.db3

The 2026-09-18 review started from three driver log lines that looked like a
command trace and were not one, and concluded the opposite of what happened.
This report reads what was recorded instead: what the operator asked for, what
the goal did, who was in control, whether the robot followed its commands, and
how localization held up. Sections whose topics a session did not record say so.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "go1_driver"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from go1_driver.command_filter import CommandFilter  # noqa: E402
from go1_driver.execution_monitor import ExecutionGuard, ExecutionMonitor  # noqa: E402
from rosbag_lite import (  # noqa: E402
    Session,
    decode_bool,
    decode_odometry,
    decode_pose_in_header,
    decode_string,
    decode_twist,
)

EXPECTED_TOPICS = (
    "/goal_pose", "/navigation/goal_status", "/localization_supervisor/status", "/cmd_vel",
    "/go1/cmd_vel_applied", "/Odometry", "/go1/robot_state", "/go1/manual_override", "/go1/execution_fault",
)


def _json_messages(session: Session, topic: str):
    for received, data in session.messages(topic):
        try:
            yield received, json.loads(decode_string(data))
        except (ValueError, UnicodeDecodeError):
            continue


def _true_intervals(session: Session, topic: str) -> list[tuple[float, float]]:
    intervals, start, last = [], None, None
    for received, data in session.messages(topic):
        value = decode_bool(data)
        if value and start is None:
            start = received
        elif not value and start is not None:
            intervals.append((start, received))
            start = None
        last = received
    if start is not None:
        intervals.append((start, last))
    return intervals


def localization_section(session: Session, origin: float) -> list[str]:
    statuses = list(_json_messages(session, "/localization_supervisor/status"))
    if not statuses:
        return ["localization: /localization_supervisor/status not recorded"]
    states = Counter(item["state"] for _stamp, item in statuses)
    errors = Counter(item["error"] for _stamp, item in statuses if item["error"] != "NONE")
    total = len(statuses)
    lines = ["localization: " + ", ".join(f"{state} {100 * count / total:.0f}%" for state, count in states.most_common())]
    if errors:
        lines.append("  errors: " + ", ".join(f"{error} x{count}" for error, count in errors.most_common()))
    last = statuses[-1][1]
    if "consistency_gap" in last:
        gaps = [item.get("consistency_gap", 0.0) for _stamp, item in statuses if item["state"] in ("READY", "DEGRADED")]
        lines.append(
            f"  tracking: max consistency gap {max(gaps, default=0.0):.3f}, "
            f"drift corrections {last.get('drift_corrections', 0)}, "
            f"own TF corrections recognised {last.get('tf_corrections_explained', 0)}"
        )
    per_minute: dict[int, list[float]] = {}
    for stamp, item in statuses:
        if item["state"] == "READY":
            per_minute.setdefault(int((stamp - origin) // 60), []).append(item["overlap"])
    if per_minute:
        lines.append("  READY overlap by minute: " + " ".join(
            f"{minute}:{statistics.mean(values):.2f}" for minute, values in sorted(per_minute.items())
        ))
    return lines


def goals_section(session: Session, origin: float) -> list[str]:
    clicks = [(stamp, decode_pose_in_header(data)) for stamp, data in session.messages("/goal_pose")]
    statuses = list(_json_messages(session, "/navigation/goal_status"))
    lines = []
    if session.has("/goal_pose"):
        lines.append(f"goals: {len(clicks)} RViz click(s)")
    if not statuses:
        lines.append("goals: /navigation/goal_status not recorded" if not lines else "  no goal status recorded")
        return lines
    if not lines:
        lines.append("goals:")
    shown = [item for item in statuses if item[1]["state"] != "ACCEPTED" or item[1].get("distance_remaining") is None]
    for stamp, item in shown:
        goal = item.get("goal") or {}
        where = f"({goal['x']:.2f}, {goal['y']:.2f})" if goal.get("x") is not None and goal.get("y") is not None else ""
        reason = f"  [{item['reason']}]" if item.get("reason") else ""
        lines.append(f"  t+{stamp - origin:7.1f}  {item['state']:<9} {where}{reason}")
    arrivals = sum(1 for _stamp, item in statuses if item["state"] == "SUCCEEDED")
    lines.append(f"  arrived: {arrivals}")
    return lines


def control_section(session: Session, origin: float) -> list[str]:
    lines = []
    for topic, label in (("/go1/manual_override", "remote in control"), ("/go1/execution_fault", "robot not following")):
        if not session.has(topic):
            lines.append(f"{label}: {topic} not recorded")
            continue
        intervals = _true_intervals(session, topic)
        total = sum(end - start for start, end in intervals)
        spans = ", ".join(f"t+{start - origin:.0f}..{end - origin:.0f}" for start, end in intervals[:8])
        lines.append(f"{label}: {len(intervals)} time(s), {total:.0f} s" + (f" ({spans})" if spans else ""))
    reports = [item for _stamp, item in _json_messages(session, "/go1/robot_state")]
    robots = [item["robot"] for item in reports if item.get("robot")]
    if robots:
        link_down = sum(1 for item in reports if item.get("link_up") is False)
        battery = [item["battery_soc"] for item in robots if item.get("battery_soc")]
        modes = Counter(item["mode"] for item in robots)
        lines.append(
            f"robot: link down in {link_down} of {len(reports)} reports, battery min "
            f"{min(battery) if battery else '?'}%, modes {dict(modes)}"
        )
    return lines


def execution_section(session: Session, origin: float) -> list[str]:
    """Replay the driver's commanded-vs-executed judgement on what was recorded."""
    if not session.has("/Odometry"):
        return ["execution: /Odometry not recorded"]
    if session.has("/go1/cmd_vel_applied"):
        source = "/go1/cmd_vel_applied"
        commands = [(stamp, *decode_twist(data)) for stamp, data in session.messages(source)]
    elif session.has("/cmd_vel"):
        source = "/cmd_vel through the driver's filter"
        command_filter = CommandFilter()
        commands = []
        for stamp, data in session.messages("/cmd_vel"):
            filtered = command_filter.filter(*decode_twist(data))
            commands.append((stamp, filtered.vx, filtered.vy, filtered.yaw))
    else:
        return ["execution: no command topic recorded"]
    poses = [(stamp, decode_odometry(data)) for stamp, data in session.messages("/Odometry")]
    events = sorted([(stamp, 0, values) for stamp, *values in commands] + [(stamp, 1, pose) for stamp, pose in poses],
                    key=lambda item: (item[0], item[1]))
    monitor, guard = ExecutionMonitor(), ExecutionGuard()
    faults, quiet_until = [], -math.inf
    next_check = events[0][0] if events else 0.0
    for stamp, kind, values in events:
        if kind == 0:
            monitor.command(stamp, *values)
        else:
            monitor.pose(stamp, values.x, values.y, values.yaw)
        while stamp >= next_check:
            if next_check >= quiet_until:
                verdict = monitor.evaluate(next_check)
                fault = guard.update(next_check, verdict)
                if fault is not None:
                    faults.append((next_check, fault.value, verdict))
                    guard.clear()
                    monitor.reset()
                    quiet_until = next_check + 10.0
            next_check += 0.1
    lines = [f"execution (from {source}): {len(faults)} episode(s) where the robot did not do what it was told"]
    for stamp, fault, verdict in faults:
        lines.append(
            f"  t+{stamp - origin:7.1f}  {fault:<18} commanded {verdict.commanded_distance:.2f} m/"
            f"{math.degrees(verdict.commanded_turn):.0f} deg, measured {verdict.achieved_distance:.2f} m/"
            f"{math.degrees(verdict.achieved_turn):.0f} deg"
        )
    return lines


def report(db3: Path) -> str:
    session = Session(db3)
    span = session.span()
    if span is None:
        return f"{db3}: empty bag"
    origin, end = span
    missing = [topic for topic in EXPECTED_TOPICS if not session.has(topic)]
    lines = [f"session {db3} ({end - origin:.0f} s)"]
    if missing:
        lines.append("not recorded: " + ", ".join(missing))
    for section in (localization_section, goals_section, control_section, execution_section):
        lines.append("")
        lines.extend(section(session, origin))
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path, help="rosbag_0.db3 of one session")
    print(report(parser.parse_args().bag))


if __name__ == "__main__":
    main()
