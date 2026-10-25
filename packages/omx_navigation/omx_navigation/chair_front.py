"""Measure the chair's front edge once, from where it can be seen, and hold it.

Close in, the chair cannot be watched. The MID-360's field stops at -7 deg, so
at the 0.33 m the seat's front edge sits from the LiDAR when the robot is
parked, only the 0.04 m under the sensor is visible -- and the 2026-10-26
recording put the chair's top at about -0.04 m in this frame. The same
forward sector is where the arm and its basket return a few hundred points.
The moment precision matters most is the moment the edge drops out of view.

From about a metre the edge is in plain view, and the chair does not move. So
the edge is measured there, averaged over frames while the robot stands after
Nav2, and latched in FAST-LIO's odometry frame (`camera_init`). From then on
the gap is the latched edge minus the robot's own odometry, which over the
last metre drifts millimetres -- unlike map localization, which hunted by
decimetres on 2026-10-25 and cannot carry a 0.035 m band.

Knowing exactly where the edge is still does not stop the robot in front of
it. `StoppingModel` carries the other half: how far the Go1 keeps going after
it is told to stop. Its numbers have no defaults because nobody has measured
them yet, and a guessed stopping distance next to a chair is worse than none.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OdomPose:
    """`body_nav` in the odometry frame: the LiDAR origin, planarized."""

    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        for name in ("x", "y", "yaw"):
            if not _is_finite_number(getattr(self, name)):
                raise ValueError(f"pose {name} must be a finite number")

    def to_odom(self, bx: float, by: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * bx - s * by, self.y + s * bx + c * by

    def to_body(self, ox: float, oy: float) -> tuple[float, float]:
        dx, dy = ox - self.x, oy - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy


@dataclass(frozen=True)
class ChairFrontPolicy:
    # A channel a little wider than the 0.45 m chair, straight ahead.
    half_width: float = 0.30
    # Height band in the LiDAR frame that holds the seat's front edge. The
    # chair's top sat near -0.04 on 2026-10-26; the lower bound is a
    # placeholder until a recording shows where the seat edge and legs fall.
    band_lo: float = -0.30
    band_hi: float = 0.05
    # Past the robot's nose (0.32) and its arm, so the robot never measures
    # itself, and close enough that the edge is still densely sampled.
    range_min: float = 0.40
    range_max: float = 1.60
    min_points: int = 30
    # The front is a low quantile of depth rather than the minimum, so one
    # stray return does not move it, and a low one so the estimate errs
    # towards the robot.
    front_quantile: float = 0.05
    # Depth of the slab behind the front used for the face's width and tilt.
    face_depth: float = 0.06
    # Frames to average while standing, and how much the edge may wander
    # across them before the reading is distrusted.
    min_frames: int = 10
    max_spread: float = 0.01
    # Tape-measure calibration: true edge minus measured edge, in metres.
    # Livox range on a dark seat fabric can sit off by a centimetre or two.
    range_bias: float = 0.0

    def __post_init__(self) -> None:
        for name in ("half_width", "range_min", "range_max", "face_depth", "max_spread"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not _is_finite_number(self.range_bias):
            raise ValueError("range_bias must be a finite number")
        if self.band_lo >= self.band_hi:
            raise ValueError("band_lo must be below band_hi")
        if self.range_min >= self.range_max:
            raise ValueError("range_min must be below range_max")
        if not 0.0 <= self.front_quantile < 0.5:
            raise ValueError("front_quantile must be in [0, 0.5)")
        for name, floor in (("min_points", 3), ("min_frames", 1)):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < floor:
                raise ValueError(f"{name} must be an integer of at least {floor}")


@dataclass(frozen=True)
class FrontReading:
    """The face ahead in `body_nav`, from one scan.

    `front_x, centre_y` is a point on the face line; `normal_yaw` is the
    direction from the robot into the face, perpendicular to it.
    """

    front_x: float
    centre_y: float
    normal_yaw: float
    width: float
    points: int


def measure_front(points, policy: ChairFrontPolicy | None = None) -> FrontReading | None:
    """Find the face of whatever stands in the channel ahead, or return None.

    `points` is any iterable of (x, y, z, reflectivity) in the LiDAR frame.
    """
    policy = policy or ChairFrontPolicy()
    ahead = []
    for x, y, z, _reflectivity in points:
        if abs(y) > policy.half_width:
            continue
        if not (policy.band_lo <= z <= policy.band_hi):
            continue
        if not (policy.range_min <= x <= policy.range_max):
            continue
        ahead.append((x, y))
    if len(ahead) < policy.min_points:
        return None

    front = _quantile(sorted(x for x, _ in ahead), policy.front_quantile)
    face = [(x, y) for x, y in ahead if x <= front + policy.face_depth]
    if len(face) < 3:
        return None

    # Fit the face as x = a + b*y. A face square to the robot has b = 0.
    mean_x = sum(x for x, _ in face) / len(face)
    mean_y = sum(y for _, y in face) / len(face)
    syy = sum((y - mean_y) ** 2 for _, y in face)
    b = 0.0 if syy < 1e-9 else sum((x - mean_x) * (y - mean_y) for x, y in face) / syy
    # Put the line at the face's near side rather than through its middle, so
    # a tilted face is not reported further away than its nearest part.
    a = _quantile(sorted(x - b * y for x, y in face), policy.front_quantile)

    lo = min(y for _, y in face)
    hi = max(y for _, y in face)
    centre_y = (lo + hi) / 2.0
    front_x = a + b * centre_y
    # Tangent along the face is (b, 1); the normal into it is (1, -b).
    normal_yaw = math.atan2(-b, 1.0)
    # Calibration moves the edge along that normal.
    front_x += policy.range_bias * math.cos(normal_yaw)
    centre_y += policy.range_bias * math.sin(normal_yaw)
    return FrontReading(front_x, centre_y, normal_yaw, hi - lo, len(ahead))


@dataclass(frozen=True)
class LatchedFront:
    """The chair's face, fixed in the odometry frame."""

    x: float
    y: float
    normal_yaw: float
    width: float
    frames: int
    spread: float

    def _normal(self) -> tuple[float, float]:
        return math.cos(self.normal_yaw), math.sin(self.normal_yaw)

    def distance_ahead(self, ox: float, oy: float) -> float:
        """How far an odom point is in front of the face, along its normal."""
        nx, ny = self._normal()
        return (self.x - ox) * nx + (self.y - oy) * ny

    def nose_clearance(self, pose: OdomPose, nose_x: float, half_width: float) -> float:
        """Gap between the face and the nearest front corner of the robot.

        Corners rather than the centre, so a robot that is slightly turned is
        measured by the corner that will touch first.
        """
        return min(
            self.distance_ahead(*pose.to_odom(nose_x, side))
            for side in (-half_width, half_width)
        )

    def standoff(self, gap: float, nose_x: float, lateral: float = 0.0) -> tuple[float, float]:
        """Odom position for `body_nav` that leaves the nose `gap` from the face.

        `lateral` slides the stop along the face, for an object that is not on
        the chair's centreline.
        """
        nx, ny = self._normal()
        back = nose_x + gap
        return self.x - nx * back - ny * lateral, self.y - ny * back + nx * lateral


class FrontLatch:
    """Average readings taken while standing, then fix the face in odom.

    Each reading is moved into odom with the pose from its own scan before it
    is averaged, so the body swaying on its legs does not smear the estimate.
    """

    def __init__(self, policy: ChairFrontPolicy | None = None) -> None:
        self.policy = policy or ChairFrontPolicy()
        self.reset()

    def reset(self) -> None:
        self._samples: list[tuple[float, float, float, float]] = []
        self._latched: LatchedFront | None = None

    @property
    def latched(self) -> LatchedFront | None:
        return self._latched

    def add(self, reading: FrontReading | None, pose: OdomPose | None) -> LatchedFront | None:
        if self._latched is not None:
            return self._latched
        if reading is None or pose is None:
            return None
        ox, oy = pose.to_odom(reading.front_x, reading.centre_y)
        self._samples.append((ox, oy, _wrap(reading.normal_yaw + pose.yaw), reading.width))
        # Keep a sliding window, so a disturbed start ages out instead of
        # blocking the latch for good.
        self._samples = self._samples[-self.policy.min_frames:]
        if len(self._samples) < self.policy.min_frames:
            return None

        count = len(self._samples)
        normal_yaw = math.atan2(
            sum(math.sin(s[2]) for s in self._samples) / count,
            sum(math.cos(s[2]) for s in self._samples) / count,
        )
        mx = sum(s[0] for s in self._samples) / count
        my = sum(s[1] for s in self._samples) / count
        # Spread is measured along the normal, the direction that matters.
        nx, ny = math.cos(normal_yaw), math.sin(normal_yaw)
        offsets = [(s[0] - mx) * nx + (s[1] - my) * ny for s in self._samples]
        spread = math.sqrt(sum(d * d for d in offsets) / count)
        if spread > self.policy.max_spread:
            return None
        width = sum(s[3] for s in self._samples) / count
        self._latched = LatchedFront(mx, my, normal_yaw, width, count, spread)
        return self._latched


@dataclass(frozen=True)
class StoppingModel:
    """How the Go1 stops, measured rather than assumed.

    Stopping distance is modelled as `latency * v + overshoot`: the robot
    keeps its speed for `latency` seconds after a stop is commanded (scan,
    processing, the gait finishing its step and switching to STAND), then
    slides a further `overshoot`. Both come from a stop test in front of a
    wall; neither has a default.
    """

    latency: float
    overshoot: float
    # Nose-to-chair gap that must never be crossed.
    min_clearance: float

    def __post_init__(self) -> None:
        if not _is_finite_number(self.latency) or self.latency <= 0.0:
            raise ValueError("latency must be finite and positive")
        for name in ("overshoot", "min_clearance"):
            value = getattr(self, name)
            if not _is_finite_number(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

    def stopping_distance(self, speed: float) -> float:
        return self.latency * max(speed, 0.0) + self.overshoot

    def max_speed_toward(self, clearance: float) -> float:
        """Fastest approach that still stops before `min_clearance`."""
        room = clearance - self.min_clearance - self.overshoot
        return max(room, 0.0) / self.latency


def limit_toward(
    vx: float,
    vy: float,
    pose: OdomPose,
    front: LatchedFront,
    model: StoppingModel,
    nose_x: float,
    half_width: float,
) -> tuple[float, float, bool]:
    """Cut the part of a body-frame command that heads into the face.

    Only the component along the face normal is limited; sliding along the
    face and backing away pass through. Returns the command and whether it
    was cut. A cut can leave less than the driver's deadband, which the driver
    turns into a stop -- the robot then stands short, which is the side to err
    on beside a chair.
    """
    yaw = front.normal_yaw - pose.yaw
    nx, ny = math.cos(yaw), math.sin(yaw)
    toward = vx * nx + vy * ny
    if toward <= 0.0:
        return vx, vy, False
    allowed = model.max_speed_toward(front.nose_clearance(pose, nose_x, half_width))
    if toward <= allowed:
        return vx, vy, False
    cut = toward - allowed
    return vx - cut * nx, vy - cut * ny, True


def _quantile(ordered: list[float], q: float) -> float:
    index = min(int(q * len(ordered)), len(ordered) - 1)
    return ordered[index]


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )
