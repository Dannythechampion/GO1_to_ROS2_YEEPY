"""The offline bag tools decode what the stack records and report it faithfully."""

from __future__ import annotations

import json
import math
import sqlite3
import struct
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

import rosbag_lite  # noqa: E402
import session_report  # noqa: E402


class Writer:
    """CDR encoder mirroring rosbag_lite.Cdr, for building test bags."""

    def __init__(self):
        self.buffer = bytearray(b"\x00\x01\x00\x00")

    def _align(self, size):
        self.buffer += b"\x00" * ((-(len(self.buffer) - 4)) % size)

    def _pack(self, fmt, size, value):
        self._align(size)
        self.buffer += struct.pack("<" + fmt, value)
        return self

    def u8(self, value):
        return self._pack("B", 1, value)

    def u32(self, value):
        return self._pack("I", 4, value)

    def i32(self, value):
        return self._pack("i", 4, value)

    def f64(self, value):
        return self._pack("d", 8, value)

    def string(self, text):
        data = text.encode("utf-8") + b"\x00"
        self.u32(len(data))
        self.buffer += data
        return self

    def header(self, stamp, frame):
        self.i32(int(stamp)).u32(int(round((stamp - int(stamp)) * 1e9)))
        return self.string(frame)

    def pose(self, x, y, yaw):
        for value in (x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)):
            self.f64(value)
        return self

    def bytes(self):
        return bytes(self.buffer)


def twist(vx, vy, wz):
    writer = Writer()
    for value in (vx, vy, 0.0, 0.0, 0.0, wz):
        writer.f64(value)
    return writer.bytes()


def odometry(stamp, x, y, yaw):
    return Writer().header(stamp, "camera_init").string("body").pose(x, y, yaw).bytes()


def pose_stamped(stamp, x, y, yaw):
    return Writer().header(stamp, "map").pose(x, y, yaw).bytes()


def string(text):
    return Writer().string(text).bytes()


def boolean(value):
    return Writer().u8(1 if value else 0).bytes()


def write_bag(path: Path, messages: dict[str, list[tuple[float, bytes]]]) -> Path:
    connection = sqlite3.connect(str(path))
    connection.execute("create table topics(id integer primary key, name text, type text, serialization_format text, offered_qos_profiles text)")
    connection.execute("create table messages(id integer primary key, topic_id integer, timestamp integer, data blob)")
    for topic_id, (topic, items) in enumerate(messages.items(), start=1):
        connection.execute("insert into topics values (?, ?, '', 'cdr', '')", (topic_id, topic))
        for stamp, data in items:
            connection.execute(
                "insert into messages(topic_id, timestamp, data) values (?, ?, ?)",
                (topic_id, int(round(stamp * 1e9)), data),
            )
    connection.commit()
    connection.close()
    return path


def test_decoders_round_trip_the_recorded_message_types():
    assert rosbag_lite.decode_twist(twist(0.15, -0.02, 0.4)) == pytest.approx((0.15, -0.02, 0.4))
    pose = rosbag_lite.decode_odometry(odometry(12.5, 1.0, -2.0, 0.7))
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, -2.0, 0.7))
    pose = rosbag_lite.decode_pose_in_header(pose_stamped(3.0, -4.0, 5.0, -2.9))
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((-4.0, 5.0, -2.9))
    assert rosbag_lite.decode_string(string('{"state":"정상"}')) == '{"state":"정상"}'
    assert rosbag_lite.decode_bool(boolean(True)) is True
    assert rosbag_lite.decode_bool(boolean(False)) is False


def test_report_tells_the_story_of_a_refused_goal(tmp_path):
    """A goal is sent, the robot turns nothing while told to, the driver flags
    it and navigation cancels -- all of it read back from the bag."""
    start = 1000.0
    status = lambda state, **fields: json.dumps({
        "state": state, "error": "NONE", "overlap": 0.8, "consistency_gap": 0.01,
        "drift_corrections": 0, "tf_corrections_explained": 1, **fields,
    })
    goal = {"frame": "map", "x": 3.0, "y": -1.0, "yaw": 0.0}
    messages = {
        "/localization_supervisor/status": [(start + 0.5 * i, string(status("READY"))) for i in range(40)],
        "/goal_pose": [(start + 0.2, pose_stamped(start + 0.2, 3.0, -1.0, 0.0))],
        "/navigation/goal_status": [
            (start + 0.3, string(json.dumps({"state": "SENT", "reason": "", "goal": goal}))),
            (start + 0.4, string(json.dumps({"state": "ACCEPTED", "reason": "", "goal": goal, "distance_remaining": None}))),
            (start + 8.6, string(json.dumps({"state": "CANCELED", "reason": "robot not following commands", "goal": goal}))),
        ],
        "/go1/cmd_vel_applied": [(start + 0.1 * i, twist(0.0, 0.0, 0.4 if i < 90 else 0.0)) for i in range(120)],
        "/Odometry": [(start + 0.1 * i, odometry(start + 0.1 * i, 1.0, 2.0, 0.3)) for i in range(120)],
        "/go1/execution_fault": [(start + 0.1 * i, boolean(8.2 <= 0.1 * i < 9.5)) for i in range(120)],
        "/go1/manual_override": [(start + 0.1 * i, boolean(False)) for i in range(120)],
    }
    text = session_report.report(write_bag(tmp_path / "rosbag_0.db3", messages))

    assert "localization: READY 100%" in text
    assert "own TF corrections recognised 1" in text
    assert "goals: 1 RViz click(s)" in text
    assert "CANCELED  (3.00, -1.00)  [robot not following commands]" in text
    assert "arrived: 0" in text
    assert "robot not following: 1 time(s)" in text
    assert "remote in control: 0 time(s)" in text
    assert "execution (from /go1/cmd_vel_applied): 1 episode(s)" in text
    assert "NOT_EXECUTING" in text


def test_report_names_what_an_old_session_did_not_record(tmp_path):
    bag = write_bag(tmp_path / "old.db3", {"/cmd_vel": [(1.0, twist(0.1, 0.0, 0.0))]})
    text = session_report.report(bag)
    assert "not recorded: /goal_pose" in text
    assert "localization: /localization_supervisor/status not recorded" in text
    assert "execution: /Odometry not recorded" in text
