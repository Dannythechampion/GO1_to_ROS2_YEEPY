"""Explain a taped field session from its logs, without ROS.

    ros2 run omx_navigation tape_report /mnt/t500/go1_field_logs/<session>
    python3 -m omx_navigation.tape_report <session>          # on the laptop

Reads `decisions.jsonl` (what the mission saw and decided, every cycle) and
`stability.csv` (how steadily the gait carried the sensors) and writes
`report.md` beside them, plus `report.png` when matplotlib is available.

The report leads with **findings** -- each one a rule applied to the logs, so
it says what was measured and why it matters, not just that something went
wrong. Below them: the timeline, every navigation leg and how it ended, every
dock round with what the standing measurement said, and stability by phase.
Both inputs may end in a torn line from a killed process; it is skipped.
"""

from __future__ import annotations

import csv
import math
import os
import sys
from collections import defaultdict

from omx_navigation.tape_mission_core import read_decision_log

STABILITY_NUMBERS = ("roll_std", "pitch_std", "tilt_max", "gyro_rms_xy", "gyro_rms_z", "vert_std",
                     "mount_amplification")


def read_stability(path) -> list:
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8", errors="replace", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                for name in ("t_start", "t_end") + STABILITY_NUMBERS:
                    value = row.get(name)
                    row[name] = float(value) if value not in (None, "", "None") else None
            except ValueError:
                continue                       # a torn last row
            if row.get("t_start") is None or row.get("verdict") not in ("OK", "WOBBLY", "UNSTABLE"):
                continue
            rows.append(row)
    return rows


def analyse(records: list, stability: list) -> dict:
    events = [r for r in records if r.get("kind") == "event"]
    cycles = [r for r in records if r.get("kind") == "cycle"]
    t0 = records[0]["t"] if records else 0.0
    legs, docks, findings = [], [], []

    # Navigation legs: from each `navigate` event to what ended it.
    for index, event in enumerate(events):
        if event["event"] != "navigate":
            continue
        end = next((e for e in events[index + 1:] if e["event"] in
                    ("handoff", "dock_started", "aborted", "navigate", "mission_done")), None)
        statuses = [e.get("state") for e in events[index + 1:]
                    if e["event"] == "nav_status" and (end is None or e["t"] <= end["t"])]
        leg_cycles = [c for c in cycles if c.get("phase") == "NAVIGATE"
                      and c["t"] >= event["t"] and (end is None or c["t"] <= end["t"])]
        seen = sum(1 for c in leg_cycles if c.get("fix"))
        legs.append({
            "waypoint": event.get("waypoint"),
            "start": event["t"] - t0,
            "duration": None if end is None else end["t"] - event["t"],
            "ended_by": None if end is None else end["event"],
            "nav_states": statuses,
            "handoff_distance": end.get("distance_to_goal") if end and end["event"] == "handoff" else None,
            "tape_seen_cycles": seen,
            "cycles": len(leg_cycles),
        })

    # Docks: from `dock_started` to arrival or abort.
    for index, event in enumerate(events):
        if event["event"] != "dock_started":
            continue
        end = next((e for e in events[index + 1:] if e["event"] in ("dock_arrived", "aborted")), None)
        dock_cycles = [c for c in cycles if c.get("phase") in
                       ("SETTLE", "MEASURE", "DRIVE", "PAUSED", "ARRIVED", "ABORTED")
                       and c["t"] >= event["t"] and (end is None or c["t"] <= end["t"])]
        missing = sum(1 for c in dock_cycles if not c.get("fix"))
        points = [c["fix"]["points"] for c in dock_cycles if c.get("fix")]
        driving = [c for c in dock_cycles if c.get("phase") == "DRIVE"]
        docks.append({
            "spot": event.get("spot"),
            "why": event.get("why"),
            "start": event["t"] - t0,
            "duration": None if end is None else end["t"] - event["t"],
            "result": None if end is None else ("ARRIVED" if end["event"] == "dock_arrived" else "ABORTED"),
            "final": None if end is None else (end.get("why") or end.get("reason")),
            "rounds": (end or {}).get("rounds") or [],
            "cycles": len(dock_cycles),
            "missing_fix": missing,
            "points_median": _percentile(points, 50),
            "points_min": min(points) if points else None,
            "drive_cycles": len(driving),
            "paused_cycles": sum(1 for c in dock_cycles if c.get("phase") == "PAUSED"),
            "max_speed": max((math.hypot(c.get("vx") or 0.0, c.get("vy") or 0.0) for c in dock_cycles), default=0.0),
            "trace": [(c["t"] - t0, c.get("ex"), c.get("ey"), c.get("phase")) for c in dock_cycles],
        })

    recorded = [e for e in events if e["event"] == "recorded"]
    record_failures = [e for e in events if e["event"] == "record_failed"]
    aborts = [e for e in events if e["event"] == "aborted"]
    ready_drops = sum(1 for a, b in zip(cycles, cycles[1:]) if a.get("ready") and not b.get("ready"))
    override_cycles = sum(1 for c in cycles if c.get("override"))

    # -- findings: each a rule over what was logged --------------------------
    for event in aborts:
        findings.append(("ABORT", f"aborted at +{event['t'] - t0:.1f} s: {event.get('reason')}"))
    for event in record_failures:
        findings.append(("RECORD", f"{event.get('waypoint')} not recorded: {event.get('reason')}"))
    for leg in legs:
        if leg["ended_by"] == "handoff":
            findings.append(("INFO", f"leg to {leg['waypoint']}: handed to the tape {leg['handoff_distance']:.2f} m "
                                     f"from the goal after {leg['duration']:.1f} s; Nav2's final approach was not needed"
                             if leg["handoff_distance"] is not None else f"leg to {leg['waypoint']} handed over"))
        if leg["cycles"] and leg["ended_by"] not in ("handoff", "dock_started") and leg["tape_seen_cycles"] == 0:
            findings.append(("NAV", f"leg to {leg['waypoint']}: the tape was never in view near the goal "
                                    f"({leg['cycles']} cycles watched) -- check the recorded spot or the tape"))
        if leg["nav_states"].count("ACCEPTED") and "SUCCEEDED" not in leg["nav_states"] and leg["ended_by"] == "dock_started":
            findings.append(("NAV", f"leg to {leg['waypoint']}: Nav2 failed but the tape was in view, so it docked"))
    for dock in docks:
        rounds = dock["rounds"]
        if dock["result"] == "ARRIVED" and rounds:
            last = rounds[-1]
            findings.append(("INFO", f"{dock['spot']}: stopped {last['ex'] * 100:+.1f} cm along / "
                                     f"{last['ey'] * 100:+.1f} cm across in {len(rounds)} rounds, {dock['duration']:.1f} s"))
        if len(rounds) >= 4:
            findings.append(("DOCK", f"{dock['spot']}: {len(rounds)} rounds -- the robot does not land where "
                                     "commanded; see pulse_followed and v_min per round"))
        not_followed = [r for r in rounds if r.get("pulse_followed") is False]
        if not_followed:
            findings.append(("DOCK", f"{dock['spot']}: {len(not_followed)} pulse(s) not followed; v_min rose to "
                                     f"{rounds[-1]['v_min']:.3f} m/s -- the Go1 ignores slower commands"))
        if dock["cycles"] and dock["missing_fix"] / dock["cycles"] > 0.2:
            findings.append(("TAPE", f"{dock['spot']}: no fix in {100 * dock['missing_fix'] / dock['cycles']:.0f}% "
                                     f"of cycles (min {dock['points_min']} points) -- tape too short, too far, or "
                                     "another reflector in view"))
        if dock["paused_cycles"]:
            findings.append(("LOC", f"{dock['spot']}: paused {dock['paused_cycles'] / 10:.1f} s waiting for READY"))
    if ready_drops:
        findings.append(("LOC", f"READY dropped {ready_drops} time(s) during the session"))
    if override_cycles:
        findings.append(("SAFETY", f"the remote was in use for {override_cycles / 10:.1f} s of logged cycles"))

    by_phase = defaultdict(list)
    for row in stability:
        phase = (row.get("dock_phase") or row.get("mission_state") or "IDLE") + (" walking" if row.get("walking") == "1" else "")
        by_phase[(row["source"], phase)].append(row)
    stability_summary = {}
    for key, rows in sorted(by_phase.items()):
        summary = {"windows": len(rows),
                   "wobbly": sum(1 for r in rows if r["verdict"] == "WOBBLY"),
                   "unstable": sum(1 for r in rows if r["verdict"] == "UNSTABLE")}
        for name in STABILITY_NUMBERS:
            values = [r[name] for r in rows if r.get(name) is not None]
            summary[name] = (_percentile(values, 50), _percentile(values, 95))
        stability_summary[key] = summary
        if summary["unstable"]:
            findings.append(("GAIT", f"{key[0]} IMU, {key[1]}: {summary['unstable']}/{len(rows)} windows UNSTABLE"))
        amp = summary["mount_amplification"][0]
        if key[0] == "lidar" and amp is not None and amp > 1.5:
            findings.append(("GAIT", f"{key[1]}: the LiDAR rocks {amp:.1f}x the trunk -- the riser amplifies the gait"))
    if not stability:
        findings.append(("GAIT", "no stability windows logged (is gait_monitor running, /livox/imu present?)"))

    return {"t0": t0, "events": events, "legs": legs, "docks": docks, "recorded": recorded,
            "findings": findings, "stability": stability_summary, "cycles": len(cycles)}


def render(result: dict) -> str:
    t0 = result["t0"]
    out = ["# Tape session report", ""]
    out += ["## Findings", ""]
    order = {"ABORT": 0, "SAFETY": 1, "RECORD": 2, "NAV": 3, "DOCK": 4, "TAPE": 5, "LOC": 6, "GAIT": 7, "INFO": 8}
    for tag, text in sorted(result["findings"], key=lambda f: order.get(f[0], 9)):
        out.append(f"- **{tag}** {text}")
    if not result["findings"]:
        out.append("- nothing logged")
    out += ["", "## Timeline", "", "| t (s) | event | detail |", "|---|---|---|"]
    for event in result["events"]:
        detail = {k: v for k, v in event.items() if k not in ("t", "kind", "event", "mode", "state", "rounds")}
        out.append(f"| {event['t'] - t0:7.1f} | {event['event']} | {_short(detail)} |")
    out += ["", "## Navigation legs", "", "| to | start | duration | ended by | Nav2 states | handoff at | tape seen |",
            "|---|---|---|---|---|---|---|"]
    for leg in result["legs"]:
        out.append(f"| {leg['waypoint']} | {leg['start']:.1f} | {_num(leg['duration'])} | {leg['ended_by']} | "
                   f"{' > '.join(s for s in leg['nav_states'] if s)} | {_num(leg['handoff_distance'], 'm')} | "
                   f"{leg['tape_seen_cycles']}/{leg['cycles']} |")
    out += ["", "## Docks", ""]
    for dock in result["docks"]:
        out += [f"### {dock['spot']} -- {dock['result']}", "",
                f"- started +{dock['start']:.1f} s ({dock['why']}), took {_num(dock['duration'], 's')}",
                f"- final: {dock['final']}",
                f"- fix missing in {dock['missing_fix']}/{dock['cycles']} cycles; points median "
                f"{dock['points_median']}, min {dock['points_min']}",
                f"- driving cycles {dock['drive_cycles']}, peak speed {dock['max_speed']:.3f} m/s, "
                f"paused {dock['paused_cycles']} cycles", "",
                "| round | t (s) | along (cm) | across (cm) | v_min | pulse followed | inside |",
                "|---|---|---|---|---|---|---|"]
        for r in dock["rounds"]:
            out.append(f"| {r['round']} | {r['t']:.1f} | {r['ex'] * 100:+.1f} | {r['ey'] * 100:+.1f} | "
                       f"{r['v_min']:.3f} | {r.get('pulse_followed')} | {r['within']} |")
        out.append("")
    out += ["## Gait stability (median / p95 per window)", "",
            "| IMU | phase | windows | wobbly | unstable | roll std deg | pitch std deg | gyro xy rad/s | bounce m/s2 | riser x |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for (source, phase), s in result["stability"].items():
        out.append(f"| {source} | {phase} | {s['windows']} | {s['wobbly']} | {s['unstable']} | "
                   f"{_pair(s['roll_std'])} | {_pair(s['pitch_std'])} | {_pair(s['gyro_rms_xy'])} | "
                   f"{_pair(s['vert_std'])} | {_pair(s['mount_amplification'])} |")
    out += ["", "## How the decisions are made", "",
            "- **Hand-over**: while navigating to a taped spot, the dock takes over once the robot is within "
            "`handoff_radius` of the goal in the map *and* the spot's pair is seen within `max_correction` for "
            "`handoff_frames` cycles in a row. Nav2's goal is then cancelled, so its final approach can never hunt.",
            "- **Dock**: settle (zero, Go1 stands), measure the median standing fix, judge: inside "
            "±1.5 cm along / ±5 cm across -> ARRIVED. Otherwise drive (closed loop beyond 6 cm, a timed pulse "
            "within) and repeat, at most 8 rounds. A pulse the next standing measurement shows was not followed "
            "raises v_min. Arrival is never judged while walking.",
            "- **Aborts** always command zero: manual override, execution fault, READY gone for 5 s, tape lost "
            "for 3 s, a pair beyond max_correction, a clearance breached in the direction of motion, 90 s total.",
            ""]
    return "\n".join(out)


def plot(result: dict, stability: list, path: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    t0 = result["t0"]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for dock in result["docks"]:
        trace = [p for p in dock["trace"] if p[1] is not None]
        if trace:
            axes[0].plot([p[0] for p in trace], [p[1] * 100 for p in trace], label=f"{dock['spot']} along")
            axes[0].plot([p[0] for p in trace], [p[2] * 100 for p in trace], "--", label=f"{dock['spot']} across")
    axes[0].axhspan(-1.5, 1.5, color="green", alpha=0.1)
    axes[0].set_ylabel("dock error (cm)")
    axes[0].legend(loc="upper right", fontsize=8)
    for source in ("lidar", "trunk"):
        rows = [r for r in stability if r["source"] == source]
        if rows:
            axes[1].plot([r["t_end"] - t0 for r in rows], [r["gyro_rms_xy"] for r in rows], label=f"{source} gyro xy")
    axes[1].set_ylabel("rad/s RMS per window")
    axes[1].set_xlabel("session time (s)")
    axes[1].legend(loc="upper right", fontsize=8)
    for event in result["events"]:
        if event["event"] in ("navigate", "handoff", "dock_started", "dock_arrived", "aborted", "recorded"):
            for axis in axes:
                axis.axvline(event["t"] - t0, color="grey", lw=0.6)
            axes[0].text(event["t"] - t0, axes[0].get_ylim()[1], event["event"], rotation=90, fontsize=7, va="top")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: tape_report <session directory>")
        return 2
    session = argv[0]
    decisions = os.path.join(session, "decisions.jsonl")
    if not os.path.exists(decisions):
        print(f"no decisions.jsonl in {session}")
        return 1
    records = read_decision_log(decisions)
    stability = read_stability(os.path.join(session, "stability.csv"))
    result = analyse(records, stability)
    text = render(result)
    with open(os.path.join(session, "report.md"), "w", encoding="utf-8") as handle:
        handle.write(text)
    plotted = plot(result, stability, os.path.join(session, "report.png"))
    print(text)
    print(f"\nwrote {os.path.join(session, 'report.md')}" + (" and report.png" if plotted else ""))
    return 0


def _percentile(values, q):
    values = sorted(v for v in values if v is not None)
    if not values:
        return None
    index = min(len(values) - 1, max(0, round(q / 100 * (len(values) - 1))))
    return values[index]


def _num(value, unit=""):
    return "-" if value is None else f"{value:.2f}{(' ' + unit) if unit else ''}"


def _pair(pair):
    a, b = pair
    return "-" if a is None else f"{a:.2f} / {b:.2f}"


def _short(detail: dict) -> str:
    parts = []
    for key, value in detail.items():
        if isinstance(value, float):
            value = round(value, 3)
        elif isinstance(value, dict):
            value = {k: round(v, 3) if isinstance(v, float) else v for k, v in value.items()}
        elif isinstance(value, (list, tuple)):
            value = [round(v, 3) if isinstance(v, float) else v for v in value]
        parts.append(f"{key}={value}")
    return ", ".join(parts).replace("|", "/")[:300]


if __name__ == "__main__":
    sys.exit(main())
