"""Read a serialized livox_ros_driver2/CustomMsg with numpy, without rclpy.

Deserializing a MID-360 frame in rclpy builds ~20,000 Python point objects,
ten times a second. On 2026-11-01 that alone held `tape_mission` at 112% CPU
on the Jetson; with the bag recorders and the gait monitor on top the load
average reached 12 on 12 cores, FAST-LIO stalled for up to 572 ms, scan,
odometry and TF went stale together, READY dropped and the bridge cancelled
the drive. Subscribing with `raw=True` and viewing the point array in place
costs a few milliseconds instead.

Layout (XCDR1, little endian, after the 4-byte encapsulation header; every
field aligned to its own size relative to the start of the payload):

    int32  header.stamp.sec      uint32 header.stamp.nanosec
    string header.frame_id       (uint32 length incl. NUL, then bytes)
    uint64 timebase              uint32 point_num
    uint8  lidar_id              uint8[3] rsvd
    uint32 points length, then each CustomPoint:
        uint32 offset_time  float32 x y z  uint8 reflectivity tag line
        -> 19 bytes, padded to 20 because the next point starts with a uint32
"""

from __future__ import annotations

import struct

import numpy as np

POINT_DTYPE = np.dtype({
    "names": ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
    "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": 20,
})


def _align(position: int, size: int) -> int:
    return (position + size - 1) // size * size


def points_view(buffer: bytes) -> np.ndarray:
    """The frame's points as a structured array (a view; no per-point work)."""
    if len(buffer) < 4 or buffer[1] != 0x01:
        raise ValueError("not a little-endian CDR payload")
    data = memoryview(buffer)[4:]
    position = 8                                    # stamp
    (length,) = struct.unpack_from("<I", data, position)
    position += 4 + length                          # frame_id
    position = _align(position, 8) + 8              # timebase
    position = _align(position, 4) + 4              # point_num
    position += 1 + 3                               # lidar_id, rsvd
    position = _align(position, 4)
    (count,) = struct.unpack_from("<I", data, position)
    position += 4
    need = count * POINT_DTYPE.itemsize
    available = len(data) - position
    if available < need - 1:
        raise ValueError(f"truncated frame: {count} points need {need - 1} bytes, have {available}")
    raw = data[position:position + need]
    if len(raw) < need:
        # The last point carries no trailing padding byte.
        raw = bytes(raw) + b"\x00" * (need - len(raw))
    return np.frombuffer(raw, dtype=POINT_DTYPE, count=count)


def split_points(points: np.ndarray, threshold: float, half_width: float, band_lo: float, band_hi: float):
    """Same contract as `tape_mission_core.split_frame`, vectorised.

    Returns (bright, channel) as lists of (x, y, z, reflectivity) tuples;
    both are small, so the Python that follows stays cheap.
    """
    x, y, z, r = points["x"], points["y"], points["z"], points["reflectivity"]
    bright_mask = r >= threshold
    channel_mask = (np.abs(y) <= half_width) & (z >= band_lo) & (z <= band_hi)
    bright = list(zip(x[bright_mask].tolist(), y[bright_mask].tolist(),
                      z[bright_mask].tolist(), r[bright_mask].tolist()))
    channel = list(zip(x[channel_mask].tolist(), y[channel_mask].tolist(),
                       z[channel_mask].tolist(), r[channel_mask].tolist()))
    return bright, channel
