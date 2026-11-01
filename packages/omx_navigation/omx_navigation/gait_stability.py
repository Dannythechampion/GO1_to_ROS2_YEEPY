"""How steadily the gait carries the sensor, from IMU windows.

Two IMUs are worth comparing. The Go1's sits in the trunk and feels the gait
itself; the MID-360's sits 0.13 m up on the riser and feels the gait plus
whatever the riser adds. A riser that whips makes the LiDAR's IMU read
markedly more than the trunk's, and that sway lands directly in every scan
-- the tape fix, slam_toolbox's matching, FAST-LIO's deskew.

Per window, per source:

- `roll_std`, `pitch_std` (deg): how much the body rocks. Taken from the
  gravity direction in the accelerometer, which is what a tilting scan sees.
- `tilt_max` (deg): the worst lean in the window.
- `gyro_rms_xy` (rad/s): roll/pitch rate, the jolt that smears a scan.
- `gyro_rms_z` (rad/s): yaw rate; large while standing means it is turning.
- `vert_std` (m/s^2): vertical bounce, from the acceleration magnitude.

The verdict thresholds are starting values for a trotting quadruped, not
measured on this robot; the CSV keeps the raw numbers so they can be refitted
from the first field session.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

GRAVITY = 9.80665


@dataclass(frozen=True)
class StabilityThresholds:
    wobbly_angle_std: float = 2.0       # deg
    unstable_angle_std: float = 5.0
    wobbly_gyro_xy: float = 0.35        # rad/s
    unstable_gyro_xy: float = 0.8
    unstable_tilt: float = 15.0         # deg
    wobbly_vert_std: float = 1.5        # m/s^2
    unstable_vert_std: float = 3.5


@dataclass(frozen=True)
class WindowStats:
    source: str
    t_start: float
    t_end: float
    samples: int
    roll_mean: float
    pitch_mean: float
    roll_std: float
    pitch_std: float
    tilt_max: float
    gyro_rms_xy: float
    gyro_rms_z: float
    vert_std: float
    verdict: str
    reasons: str

    CSV_FIELDS = ("source", "t_start", "t_end", "samples", "roll_mean", "pitch_mean", "roll_std",
                  "pitch_std", "tilt_max", "gyro_rms_xy", "gyro_rms_z", "vert_std", "verdict", "reasons")

    def row(self) -> list:
        return [getattr(self, name) if not isinstance(getattr(self, name), float)
                else round(getattr(self, name), 5) for name in self.CSV_FIELDS]


def window_stats(source: str, samples, thresholds: StabilityThresholds | None = None) -> WindowStats | None:
    """Summarise `samples`: (t, gx, gy, gz, ax, ay, az) with gyro in rad/s.

    Acceleration may be in g (the Livox driver) or m/s^2 (the Go1); the unit
    is told apart by the magnitude, which is about 1 or about 9.8 at rest.
    """
    thresholds = thresholds or StabilityThresholds()
    samples = [s for s in samples if all(math.isfinite(v) for v in s)]
    if len(samples) < 3:
        return None
    norms = sorted(math.sqrt(s[4] ** 2 + s[5] ** 2 + s[6] ** 2) for s in samples)
    scale = GRAVITY if norms[len(norms) // 2] < 3.0 else 1.0
    rolls, pitches, tilts, verts = [], [], [], []
    gyro_xy = gyro_z = 0.0
    for _t, gx, gy, gz, ax, ay, az in samples:
        ax, ay, az = ax * scale, ay * scale, az * scale
        rolls.append(math.degrees(math.atan2(ay, az)))
        pitches.append(math.degrees(math.atan2(-ax, math.hypot(ay, az))))
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        tilts.append(math.degrees(math.acos(max(-1.0, min(1.0, az / norm)))) if norm > 0 else 0.0)
        verts.append(norm)
        gyro_xy += gx * gx + gy * gy
        gyro_z += gz * gz
    n = len(samples)
    roll_std, pitch_std = _std(rolls), _std(pitches)
    gyro_rms_xy, gyro_rms_z = math.sqrt(gyro_xy / n), math.sqrt(gyro_z / n)
    vert_std = _std(verts)
    tilt_max = max(tilts)

    unstable, wobbly = [], []
    angle_std = max(roll_std, pitch_std)
    if angle_std >= thresholds.unstable_angle_std:
        unstable.append(f"rocking {angle_std:.1f} deg")
    elif angle_std >= thresholds.wobbly_angle_std:
        wobbly.append(f"rocking {angle_std:.1f} deg")
    if gyro_rms_xy >= thresholds.unstable_gyro_xy:
        unstable.append(f"roll/pitch rate {gyro_rms_xy:.2f} rad/s")
    elif gyro_rms_xy >= thresholds.wobbly_gyro_xy:
        wobbly.append(f"roll/pitch rate {gyro_rms_xy:.2f} rad/s")
    if tilt_max >= thresholds.unstable_tilt:
        unstable.append(f"tilt {tilt_max:.1f} deg")
    if vert_std >= thresholds.unstable_vert_std:
        unstable.append(f"bounce {vert_std:.1f} m/s2")
    elif vert_std >= thresholds.wobbly_vert_std:
        wobbly.append(f"bounce {vert_std:.1f} m/s2")
    verdict = "UNSTABLE" if unstable else "WOBBLY" if wobbly else "OK"
    return WindowStats(source, samples[0][0], samples[-1][0], n, _mean(rolls), _mean(pitches),
                       roll_std, pitch_std, tilt_max, gyro_rms_xy, gyro_rms_z, vert_std, verdict,
                       "; ".join(unstable + wobbly))


class ImuWindow:
    """Collect IMU samples and emit `WindowStats` every `period` seconds."""

    def __init__(self, source: str, period: float = 0.5, thresholds: StabilityThresholds | None = None) -> None:
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("period must be finite and positive")
        self.source = source
        self.period = period
        self.thresholds = thresholds or StabilityThresholds()
        self._samples = []

    def add(self, sample):
        """Add one sample; return the closed window's stats when one closes."""
        if self._samples and sample[0] - self._samples[0][0] >= self.period:
            stats = window_stats(self.source, self._samples, self.thresholds)
            self._samples = [sample]
            return stats
        self._samples.append(sample)
        return None


def mount_amplification(lidar: WindowStats | None, trunk: WindowStats | None):
    """How much more the LiDAR rocks than the trunk under it, or None."""
    if lidar is None or trunk is None or trunk.gyro_rms_xy < 0.02:
        return None
    return lidar.gyro_rms_xy / trunk.gyro_rms_xy


def _mean(values):
    return sum(values) / len(values)


def _std(values):
    mean = _mean(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))
