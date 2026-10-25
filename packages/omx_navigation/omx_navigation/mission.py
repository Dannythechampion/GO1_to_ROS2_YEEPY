"""Named poses recorded ahead of time, and the sequence that visits them.

The scenario is fixed: stand at a recorded start, navigate to a recorded goal
beside the chair, dock on the corridor markers, let the arm work, come back.
Holding that as data rather than code means the poses can be re-recorded after
the furniture moves without touching the stack, and that the sequence is
reviewable in one file.

Poses are in the `map` frame, which is the frame slam_toolbox localises in, so
they survive restarts as long as the posegraph does. `yaw` is the heading the
base holds, in radians.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    """A recorded base pose in the `map` frame."""

    name: str
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("waypoint name must be a non-empty string")
        for field_name in ("x", "y", "yaw"):
            if not _is_finite_number(getattr(self, field_name)):
                raise ValueError(f"{self.name}.{field_name} must be a finite number")

    def as_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw}


@dataclass(frozen=True)
class Step:
    """One action in the scenario.

    `kind` is "navigate", "dock" or "grasp". Only "navigate" carries a
    waypoint; the other two act where the robot already stands.
    """

    kind: str
    waypoint: str | None = None

    KINDS = ("navigate", "dock", "grasp")

    def __post_init__(self) -> None:
        if self.kind not in Step.KINDS:
            raise ValueError(f"unknown step kind {self.kind!r}; expected one of {Step.KINDS}")
        if self.kind == "navigate":
            if not isinstance(self.waypoint, str) or not self.waypoint.strip():
                raise ValueError("a navigate step needs a waypoint name")
        elif self.waypoint is not None:
            raise ValueError(f"a {self.kind} step does not take a waypoint")


@dataclass(frozen=True)
class Mission:
    waypoints: dict
    sequence: tuple

    def __post_init__(self) -> None:
        if not self.sequence:
            raise ValueError("a mission needs at least one step")
        for step in self.sequence:
            if step.kind == "navigate" and step.waypoint not in self.waypoints:
                raise ValueError(
                    f"step navigates to {step.waypoint!r}, which is not a recorded waypoint"
                )
        # Docking drives the base from a detection rather than from the map, so
        # it only makes sense once navigation has put the robot in range of the
        # markers. Catching it here beats discovering it beside the chair.
        for previous, step in zip((None, *self.sequence), self.sequence):
            if step.kind == "dock" and (previous is None or previous.kind != "navigate"):
                raise ValueError("a dock step must follow a navigate step")

    def waypoint(self, name: str) -> Waypoint:
        try:
            return self.waypoints[name]
        except KeyError:
            raise ValueError(f"no recorded waypoint named {name!r}") from None


def mission_from_dict(data: dict) -> Mission:
    """Build a mission from parsed YAML.

    Kept separate from file reading so the stack never needs a filesystem to
    test a scenario, and so a mission can arrive over a topic later.
    """
    if not isinstance(data, dict):
        raise ValueError("mission must be a mapping")
    raw_waypoints = data.get("waypoints")
    if not isinstance(raw_waypoints, dict) or not raw_waypoints:
        raise ValueError("mission needs a non-empty 'waypoints' mapping")
    waypoints = {}
    for name, pose in raw_waypoints.items():
        if not isinstance(pose, dict):
            raise ValueError(f"waypoint {name!r} must be a mapping with x, y and yaw")
        missing = {"x", "y", "yaw"} - set(pose)
        if missing:
            raise ValueError(f"waypoint {name!r} is missing {sorted(missing)}")
        waypoints[name] = Waypoint(name, pose["x"], pose["y"], pose["yaw"])

    raw_sequence = data.get("sequence")
    if not isinstance(raw_sequence, list) or not raw_sequence:
        raise ValueError("mission needs a non-empty 'sequence' list")
    sequence = []
    for entry in raw_sequence:
        if isinstance(entry, str):
            sequence.append(Step(entry))
        elif isinstance(entry, dict) and len(entry) == 1:
            kind, waypoint = next(iter(entry.items()))
            sequence.append(Step(kind, waypoint))
        else:
            raise ValueError(f"cannot read sequence entry {entry!r}")
    return Mission(waypoints, tuple(sequence))


def mission_to_dict(mission: Mission) -> dict:
    """Round-trip of `mission_from_dict`, for re-recording waypoints."""
    sequence = []
    for step in mission.sequence:
        sequence.append({step.kind: step.waypoint} if step.waypoint else step.kind)
    return {
        "waypoints": {name: point.as_dict() for name, point in mission.waypoints.items()},
        "sequence": sequence,
    }


def record_waypoint(mission: Mission, name: str, x: float, y: float, yaw: float) -> Mission:
    """Return a mission with `name` replaced by the pose given.

    Re-recording is how the scenario adapts after the chair or the markers
    move: drive the robot to the spot, capture its pose, write the file back.
    """
    updated = dict(mission.waypoints)
    updated[name] = Waypoint(name, x, y, _wrap(yaw))
    return Mission(updated, mission.sequence)


def _wrap(angle: float) -> float:
    if not _is_finite_number(angle):
        raise ValueError("yaw must be a finite number")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
