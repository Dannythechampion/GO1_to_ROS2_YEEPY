"""Hand-over, crash-safe logs and gait stability for the taped field session."""

import math
import os
import subprocess
import sys
import textwrap

import pytest

from omx_navigation.corridor_dock import CorridorDockPolicy, CorridorFix
from omx_navigation.gait_stability import ImuWindow, mount_amplification, window_stats
from omx_navigation.tape_mission_core import DecisionLog, HandoffWatch, read_decision_log


def fix(x=0.1, y=0.0):
    return CorridorFix(x, y, 0.0, 2.0, 80)


# -- hand-over ----------------------------------------------------------------


def test_handoff_needs_the_goal_close_and_the_pair_steady():
    policy = CorridorDockPolicy()
    watch = HandoffWatch(radius=1.0, frames=3)
    assert not watch.update(0.8, fix(), policy)
    assert not watch.update(0.8, fix(), policy)
    assert watch.update(0.8, fix(), policy)


def test_the_start_pair_in_view_far_from_the_goal_never_hands_over():
    """Setting off, the start tape is in view: that must not count as arriving."""
    policy = CorridorDockPolicy()
    watch = HandoffWatch()
    assert not any(watch.update(6.0, fix(), policy) for _ in range(20))


def test_one_missed_frame_restarts_the_count():
    policy = CorridorDockPolicy()
    watch = HandoffWatch(frames=3)
    watch.update(0.5, fix(), policy)
    watch.update(0.5, fix(), policy)
    assert not watch.update(0.5, None, policy)
    assert not watch.update(0.5, fix(), policy)


def test_a_pair_the_dock_would_refuse_never_hands_over():
    policy = CorridorDockPolicy()
    watch = HandoffWatch(frames=1)
    assert not watch.update(0.5, fix(x=0.9), policy)      # beyond max_correction
    assert not watch.update(None, fix(), policy)          # no transform: wait for Nav2


# -- crash-safe decision log ----------------------------------------------------


def test_decision_log_round_trips_and_survives_a_torn_line(tmp_path):
    path = tmp_path / "session" / "decisions.jsonl"
    log = DecisionLog(path)
    log.write({"t": 1.0, "kind": "cycle", "x": float("inf")})
    log.write({"t": 2.0, "kind": "event", "event": "handoff"}, event=True)
    log.close()
    with open(path, "a", encoding="utf-8") as handle:
        handle.write('{"t": 3.0, "kind": "cy')        # the process died mid-line
    records = read_decision_log(path)
    assert [r["t"] for r in records] == [1.0, 2.0]
    assert records[1]["event"] == "handoff"


def test_decision_log_keeps_every_line_written_before_a_kill(tmp_path):
    """SIGKILL gives no chance to close; every flushed line must still be there."""
    path = tmp_path / "decisions.jsonl"
    script = textwrap.dedent(f"""
        import os, signal
        from omx_navigation.tape_mission_core import DecisionLog
        log = DecisionLog({str(path)!r})
        for i in range(50):
            log.write({{"t": i, "kind": "cycle"}}, event=(i % 10 == 0))
        os.kill(os.getpid(), signal.SIGTERM if os.name == "nt" else signal.SIGKILL)
    """)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    subprocess.run([sys.executable, "-c", script], env=env, check=False)
    assert [r["t"] for r in read_decision_log(path)] == list(range(50))


# -- gait stability -------------------------------------------------------------


def samples(n=100, rate=200.0, rock_deg=0.0, rock_hz=2.0, gyro=0.0, g_units=True, bounce=0.0):
    out = []
    for i in range(n):
        t = i / rate
        angle = math.radians(rock_deg) * math.sin(2 * math.pi * rock_hz * t)
        g = 1.0 if g_units else 9.80665
        az = g * math.cos(angle) * (1.0 + bounce * math.sin(2 * math.pi * 3 * t))
        ay = g * math.sin(angle)
        out.append((t, gyro * math.sin(2 * math.pi * rock_hz * t), 0.0, 0.0, 0.0, ay, az))
    return out


def test_a_standing_robot_is_ok_in_either_unit():
    for g_units in (True, False):
        stats = window_stats("lidar", samples(g_units=g_units))
        assert stats.verdict == "OK"
        assert stats.roll_std == pytest.approx(0.0, abs=0.01)
        assert stats.tilt_max == pytest.approx(0.0, abs=0.01)


def test_rocking_is_graded_by_how_far_it_rocks():
    assert window_stats("trunk", samples(rock_deg=1.0)).verdict == "OK"
    assert window_stats("trunk", samples(rock_deg=4.0)).verdict == "WOBBLY"
    unstable = window_stats("trunk", samples(rock_deg=10.0, gyro=1.5))
    assert unstable.verdict == "UNSTABLE"
    assert "rocking" in unstable.reasons and "rate" in unstable.reasons


def test_bounce_shows_as_vertical_spread():
    stats = window_stats("lidar", samples(bounce=0.3))
    assert stats.vert_std > 1.5
    assert stats.verdict in ("WOBBLY", "UNSTABLE")


def test_windows_close_on_time():
    window = ImuWindow("lidar", period=0.5)
    closed = [s for s in (window.add(sample) for sample in samples(n=250)) if s is not None]
    assert len(closed) == 2
    assert closed[0].samples == 100


def test_mount_amplification_compares_the_riser_with_the_trunk():
    lidar = window_stats("lidar", samples(gyro=0.6))
    trunk = window_stats("trunk", samples(gyro=0.3))
    assert mount_amplification(lidar, trunk) == pytest.approx(2.0, rel=0.05)
    assert mount_amplification(lidar, window_stats("trunk", samples())) is None
