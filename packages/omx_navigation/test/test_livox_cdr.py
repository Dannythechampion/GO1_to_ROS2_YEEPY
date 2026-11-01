"""The numpy view of a serialized CustomMsg matches what was serialized."""

import struct

import numpy as np
import pytest

from omx_navigation.corridor_dock import CorridorDockPolicy, CorridorFix
from omx_navigation.livox_cdr import points_view, split_points
from omx_navigation.tape_mission_core import split_frame, tape_near_goal


def cdr_frame(points, frame_id="livox_frame"):
    """Serialize like rmw_fastrtps does: XCDR1 LE, payload-relative alignment."""
    body = bytearray()

    def align(size):
        while len(body) % size:
            body.append(0)

    body += struct.pack("<iI", 1793536000, 5)
    name = frame_id.encode() + b"\x00"
    body += struct.pack("<I", len(name)) + name
    align(8)
    body += struct.pack("<Q", 123456789)
    align(4)
    body += struct.pack("<I", len(points))
    body += bytes([1, 0, 0, 0])
    align(4)
    body += struct.pack("<I", len(points))
    for index, (x, y, z, r) in enumerate(points):
        align(4)
        body += struct.pack("<Ifff", index, x, y, z) + bytes([r, 0, index % 4])
    return b"\x00\x01\x00\x00" + bytes(body)


@pytest.mark.parametrize("frame_id", ["livox_frame", "x", "abcdefg"])   # shifts every later alignment
def test_points_round_trip_through_cdr(frame_id):
    points = [(1.0, -0.5, 0.25, 200), (0.1, 0.2, -0.3, 20), (-2.0, 1.0, 0.0, 255)]
    view = points_view(cdr_frame(points, frame_id))
    assert len(view) == 3
    got = list(zip(view["x"].tolist(), view["y"].tolist(), view["z"].tolist(), view["reflectivity"].tolist()))
    assert np.allclose(np.array(got, dtype=float), np.array(points, dtype=float), atol=1e-6)


def test_an_empty_frame_and_a_truncated_one():
    assert len(points_view(cdr_frame([]))) == 0
    with pytest.raises(ValueError):
        points_view(cdr_frame([(1.0, 1.0, 0.0, 200)] * 5)[:-30])


def test_vectorised_split_agrees_with_the_python_one():
    rng = np.random.default_rng(3)
    points = [(float(x), float(y), float(z), int(r)) for x, y, z, r in zip(
        rng.uniform(-3, 3, 500), rng.uniform(-2, 2, 500), rng.uniform(-0.5, 0.5, 500), rng.integers(0, 256, 500))]
    view = points_view(cdr_frame(points))
    fast = split_points(view, 120.0, 0.20, -0.15, 0.25)
    slow = split_frame(points, 120.0, 0.20, -0.15, 0.25)
    for a, b in zip(fast, slow):
        assert len(a) == len(b)
        assert np.allclose(np.array(a, dtype=float), np.array(b, dtype=float), atol=1e-6)


def test_the_start_tape_far_from_the_goal_cannot_rescue_a_failed_navigation():
    """What happened on 2026-11-01: 8 m from the goal, start tape 0.19 m behind."""
    policy = CorridorDockPolicy()
    behind = CorridorFix(-0.19, -0.01, 0.0, 2.03, 200)
    assert not tape_near_goal(8.05, behind, policy, 1.0)
    assert tape_near_goal(0.6, behind, policy, 1.0)
    assert not tape_near_goal(None, behind, policy, 1.0)
