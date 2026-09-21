"""Read a rosbag2 sqlite3 session without ROS installed.

Field bags have to be analysed where the Jetson is not: the laptop has no
ROS 2 message definitions. The handful of message types the Go1 stack records
are simple enough to decode from CDR directly.
"""

from __future__ import annotations

import math
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


class Cdr:
    """Minimal little-endian XCDR1 reader."""

    def __init__(self, data: bytes):
        self.data = bytes(data)
        self.offset = 4  # encapsulation header

    def _align(self, size: int) -> None:
        self.offset += (-(self.offset - 4)) % size

    def _unpack(self, fmt: str, size: int):
        self._align(size)
        value = struct.unpack_from("<" + fmt, self.data, self.offset)[0]
        self.offset += size
        return value

    def u8(self) -> int:
        return self._unpack("B", 1)

    def u32(self) -> int:
        return self._unpack("I", 4)

    def i32(self) -> int:
        return self._unpack("i", 4)

    def f32(self) -> float:
        return self._unpack("f", 4)

    def f64(self) -> float:
        return self._unpack("d", 8)

    def string(self) -> str:
        length = self.u32()
        value = self.data[self.offset:self.offset + length - 1].decode("utf-8", "replace")
        self.offset += length
        return value

    def f32_sequence(self) -> tuple[float, ...]:
        count = self.u32()
        self._align(4)
        values = struct.unpack_from("<%df" % count, self.data, self.offset)
        self.offset += 4 * count
        return values

    def header(self) -> tuple[float, str]:
        seconds = self.i32()
        nanoseconds = self.u32()
        return seconds + nanoseconds * 1e-9, self.string()

    def pose(self) -> Pose:
        x, y, _z = self.f64(), self.f64(), self.f64()
        qx, qy, qz, qw = self.f64(), self.f64(), self.f64(), self.f64()
        return Pose(x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


def decode_twist(data: bytes) -> tuple[float, float, float]:
    reader = Cdr(data)
    vx, vy, _vz, _wx, _wy, wz = (reader.f64() for _ in range(6))
    return vx, vy, wz


def decode_pose_in_header(data: bytes) -> Pose:
    """PoseStamped, PoseWithCovarianceStamped and Odometry all start this way."""
    reader = Cdr(data)
    reader.header()
    return reader.pose()


def decode_odometry(data: bytes) -> Pose:
    reader = Cdr(data)
    reader.header()
    reader.string()  # child_frame_id
    return reader.pose()


def decode_string(data: bytes) -> str:
    return Cdr(data).string()


def decode_bool(data: bytes) -> bool:
    return bool(Cdr(data).u8())


class Session:
    def __init__(self, db3: Path):
        self.path = Path(db3)
        self.connection = sqlite3.connect(str(self.path))
        self.topics = {name: topic_id for topic_id, name in self.connection.execute("select id,name from topics")}

    def has(self, topic: str) -> bool:
        return topic in self.topics

    def messages(self, topic: str) -> Iterator[tuple[float, bytes]]:
        """(receive time in seconds, CDR payload) in recording order."""
        if topic not in self.topics:
            return
        for stamp, data in self.connection.execute(
            "select timestamp,data from messages where topic_id=? order by timestamp", (self.topics[topic],)
        ):
            yield stamp / 1e9, data

    def span(self) -> tuple[float, float] | None:
        row = self.connection.execute("select min(timestamp), max(timestamp) from messages").fetchone()
        return None if row is None or row[0] is None else (row[0] / 1e9, row[1] / 1e9)

    def transforms(self):
        """(receive time, stamp, parent, child, pose) for every transform on /tf."""
        for received, data in self.messages("/tf"):
            reader = Cdr(data)
            for _ in range(reader.u32()):
                stamp, parent = reader.header()
                child = reader.string()
                yield received, stamp, parent, child, reader.pose()
