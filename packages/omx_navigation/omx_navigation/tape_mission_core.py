"""ROS-free pieces of the tape mission node, kept here so they can be tested.

Two things the node does every cycle have nothing to do with ROS:

- **Thinning the cloud.** A MID-360 frame is ~20,000 points and the dock
  needs only the tape-bright ones plus a robot-wide channel fore and aft for
  the clearance net. Splitting once per frame keeps the Python loop small
  enough for 10 Hz on the Jetson.
- **Reading navigation results.** Goals go through `rviz_goal_bridge`, which
  must stay the only way a goal reaches Nav2 (see its docstring), so the
  mission learns the outcome from `/navigation/goal_status` and has to match
  the status to the goal it published.
"""

from __future__ import annotations

import math
from collections import deque

# Every state rviz_goal_bridge reports that ends a goal. IGNORED and REJECTED
# end it before Nav2 ever moved, which matters as much: a mission that waits
# on a goal the bridge refused waits forever.
SUCCEEDED = "SUCCEEDED"
FAILED_STATES = frozenset({"ABORTED", "CANCELED", "REJECTED", "IGNORED", "PREEMPTED"})
GOAL_MATCH_TOLERANCE = 1e-3


def split_frame(points, threshold: float, half_width: float, band_lo: float, band_hi: float):
    """Split one frame into tape-bright returns and the clearance channel.

    `points` is any iterable of (x, y, z, reflectivity). Returns two lists of
    the same tuples: everything at or above `threshold`, and everything inside
    the robot-wide channel and height band, ahead or behind.
    """
    bright = []
    channel = []
    for point in points:
        x, y, z, reflectivity = point
        if reflectivity >= threshold:
            bright.append(point)
        if abs(y) <= half_width and band_lo <= z <= band_hi:
            channel.append(point)
    return bright, channel


class FrameWindow:
    """The last `seconds` of split frames, merged.

    One frame puts ~30 points on a 4.5 cm strip 1 m away, only just over the
    detector's 20. Two frames double that while the robot, at the dock's
    0.08 m/s, moves under 2 cm.
    """

    def __init__(self, seconds: float) -> None:
        if not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or seconds <= 0.0:
            raise ValueError("seconds must be finite and positive")
        self.seconds = float(seconds)
        self._frames = deque()

    def add(self, stamp: float, bright, channel) -> None:
        self._frames.append((stamp, bright, channel))
        self.prune(stamp)

    def prune(self, now: float) -> None:
        while self._frames and now - self._frames[0][0] > self.seconds:
            self._frames.popleft()

    def clear(self) -> None:
        self._frames.clear()

    @property
    def frames(self) -> int:
        return len(self._frames)

    def bright(self) -> list:
        return [point for _stamp, bright, _channel in self._frames for point in bright]

    def channel(self) -> list:
        return [point for _stamp, _bright, channel in self._frames for point in channel]


def navigation_outcome(status: dict, waypoint) -> bool | None:
    """True or False once the bridge has finished the goal sent to `waypoint`.

    None while it is still running, or when the status is about some other
    goal -- an operator click in RViz, say, which the bridge would report as
    preempting ours, and that arrives here as PREEMPTED for ours.
    """
    if not isinstance(status, dict):
        return None
    goal = status.get("goal") or {}
    x, y = goal.get("x"), goal.get("y")
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return None
    if math.hypot(x - waypoint.x, y - waypoint.y) > GOAL_MATCH_TOLERANCE:
        return None
    state = status.get("state")
    if state == SUCCEEDED:
        return True
    if state in FAILED_STATES:
        return False
    return None
