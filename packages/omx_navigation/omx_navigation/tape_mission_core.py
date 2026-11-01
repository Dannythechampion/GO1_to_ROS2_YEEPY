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


class HandoffWatch:
    """When to stop navigating and let the tape take over.

    Three conditions, each for a reason:

    - **Close to the goal in the map** (`radius`). The start pair is in view
      when the robot sets off; without this the mission would "arrive" before
      leaving.
    - **The spot's pair in view, within `max_correction`.** That is the dock's
      own test for the right pair, so the hand-over can never start a dock the
      dock would refuse.
    - **For `frames` consecutive cycles.** One frame can catch anything shiny.

    A missing distance (no map->body_nav transform) never hands over; Nav2
    keeps going and its own result decides.
    """

    def __init__(self, radius: float = 1.0, frames: int = 3) -> None:
        if not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("radius must be finite and positive")
        if not isinstance(frames, int) or frames < 1:
            raise ValueError("frames must be a positive integer")
        self.radius = float(radius)
        self.frames = frames
        self.count = 0

    def reset(self) -> None:
        self.count = 0

    def update(self, distance_to_goal, fix, policy) -> bool:
        self.count = self.count + 1 if tape_near_goal(distance_to_goal, fix, policy, self.radius) else 0
        return self.count >= self.frames


def tape_near_goal(distance_to_goal, fix, policy, radius: float) -> bool:
    """The goal's pair is plausibly what is in view: close in the map, and in range.

    Every rule that lets the tape take over from Nav2 must pass this. On
    2026-11-01 a navigation cancelled 0.4 m out of the start was "rescued"
    onto the start tape, because the rescue checked only that some pair was in
    view, and the dock then tried to back the robot 0.46 m onto it.
    """
    close = distance_to_goal is not None and distance_to_goal <= radius
    return close and fix is not None and fix.correction(policy) <= policy.max_correction


class DecisionLog:
    """Append-only JSON lines that survive the process being killed.

    Each record is one line written and flushed at once, so a SIGKILL or a
    power cut loses at most the line being written -- the 2026-10-25 bag lost
    everything because its index lived at the end of the file. The OS buffer
    is forced to disk at most every `sync_period` seconds, and immediately for
    events, which are the lines an investigation starts from.
    """

    def __init__(self, path, sync_period: float = 1.0, clock=None) -> None:
        import os
        import time

        self._os = os
        self._clock = clock or time.monotonic
        self.path = str(path)
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self._file = open(self.path, "a", encoding="utf-8", buffering=1)
        self._sync_period = sync_period
        self._synced_at = self._clock()

    def write(self, record: dict, event: bool = False) -> None:
        import json

        if self._file is None:
            return
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":"), default=_jsonable)
        self._file.write(line + "\n")
        self._file.flush()
        now = self._clock()
        if event or now - self._synced_at >= self._sync_period:
            self._os.fsync(self._file.fileno())
            self._synced_at = now

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._os.fsync(self._file.fileno())
            self._file.close()
            self._file = None


def read_decision_log(path) -> list:
    """Every complete record; a line cut off by a crash is skipped, not fatal."""
    import json

    records = []
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            try:
                records.append(json.loads(line))
            except ValueError:
                continue
    return records


def _jsonable(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, "value"):
        return value.value
    return str(value)
