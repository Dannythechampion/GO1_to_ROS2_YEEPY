"""ROS-independent decision logic for tracking drift after localization lock.

The supervisor locks with `coarse_search` and then watches only overlap. Replaying
the 2026-09-18 Hanyang 9F run showed what that misses: at the far end of the
corridor the tracked pose picked up a ~4 degree heading error that
slam_toolbox never corrected (it processes no scan while the robot stands
still), overlap sank from 0.86 to about 0.65 and stayed there, and the
supervisor kept reporting READY because 0.65 is above the 0.45 floor.

A bounded local refinement around the tracked pose measures the drift
directly. Over the same replay its score gap stayed at a median 0.015
(p90 0.032) while Nav2 was driving well, and rose to a median 0.18 once the
heading error was in place, so a threshold between them separates the two
cleanly. This module turns a stream of those checks into actions.

What a correction may trust is also measured, not assumed. In a corridor the
refined position along the corridor axis wanders from check to check (the map
cannot tell those poses apart) while heading and lateral position repeat to
within a degree and a few centimetres. Correcting heading alone is wrong: it
is coupled to the lateral offset, and on the replay it made later scans match
worse. Applying the per-component median of the implied map->odom corrections
as one transform is right: across all 109 corrections this policy would have
issued on that run, the overlap of the following 20 s of scans improved every
time (parked: 0.71 -> 0.85) and never got worse.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from enum import Enum

from omx_navigation.pose_tracking import compose_pose, invert_pose, pose_difference
from omx_navigation.scan_map_quality import Pose2D


class DriftAction(str, Enum):
    NONE = "NONE"
    CORRECT = "CORRECT"
    ESCALATE = "ESCALATE"


@dataclass(frozen=True)
class DriftPolicy:
    # A check counts as drifted when a nearby pose beats the tracked one by this
    # much score. Replay: tracking p90 0.032, drifted p10 0.12.
    gap_threshold: float = 0.08
    # Consecutive drifted checks before anything happens. With 0.5 Hz checks
    # this is 6 s of evidence; the replay had no three-in-a-row while tracking.
    required_checks: int = 3
    # The implied map->odom correction must agree across those checks. It is
    # compared in map->odom space because that transform is what drifted, and
    # it does not move with the robot the way map->base does. Heading is held
    # tight; translation only loosely, because along a corridor it is not
    # observable and its spread is expected (median 0.22 m while parked).
    agreement_translation: float = 0.45
    agreement_yaw: float = math.radians(2.0)
    # The corrected pose must itself be an acceptable match.
    min_corrected_overlap: float = 0.45
    auto_correct: bool = True
    correction_interval: float = 10.0
    max_corrections: int = 3
    correction_window: float = 60.0
    # Drift that no correction clears within this long becomes a quality error.
    escalate_after: float = 12.0

    def __post_init__(self) -> None:
        for name in (
            "gap_threshold", "agreement_translation", "agreement_yaw",
            "correction_interval", "correction_window", "escalate_after",
        ):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0.0 <= self.min_corrected_overlap <= 1.0:
            raise ValueError("min_corrected_overlap must be within [0.0, 1.0]")
        for name in ("required_checks", "max_corrections"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class DriftCheck:
    """One refinement result, taken at `stamp` seconds."""

    stamp: float
    gap: float
    tracked: Pose2D
    refined: Pose2D
    refined_overlap: float
    camera_base: Pose2D

    def implied_map_camera(self) -> Pose2D:
        """The map->odom transform that would put the base at `refined`."""
        return compose_pose(self.refined, invert_pose(self.camera_base))


@dataclass(frozen=True)
class DriftDecision:
    action: DriftAction
    drifting: bool
    consecutive: int
    gap: float
    offset: tuple[float, float, float]
    correction: Pose2D | None = None
    reason: str = ""


@dataclass
class DriftMonitor:
    policy: DriftPolicy = field(default_factory=DriftPolicy)

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Forget all evidence, e.g. after the operator re-initializes."""
        self._streak: list[DriftCheck] = []
        self._drift_since: float | None = None
        self._corrections: list[float] = []
        self._escalated = False

    @property
    def escalated(self) -> bool:
        return self._escalated

    @property
    def corrections(self) -> int:
        return len(self._corrections)

    def observe(self, check: DriftCheck) -> DriftDecision:
        values = (check.stamp, check.gap, check.refined_overlap)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("drift check values must be finite")
        offset = (
            check.refined.x - check.tracked.x,
            check.refined.y - check.tracked.y,
            (check.refined.yaw - check.tracked.yaw + math.pi) % (2.0 * math.pi) - math.pi,
        )
        policy = self.policy
        if check.gap < policy.gap_threshold:
            self._streak = []
            self._drift_since = None
            self._escalated = False
            return DriftDecision(DriftAction.NONE, False, 0, check.gap, offset)

        self._streak = (self._streak + [check])[-policy.required_checks:]
        if self._drift_since is None:
            self._drift_since = check.stamp
        consecutive = len(self._streak)
        if consecutive < policy.required_checks:
            return DriftDecision(DriftAction.NONE, True, consecutive, check.gap, offset, reason="accumulating evidence")

        self._corrections = [
            stamp for stamp in self._corrections if check.stamp - stamp < policy.correction_window
        ]
        consistent = self._streak_is_consistent()
        good_match = check.refined_overlap >= policy.min_corrected_overlap
        rate_ok = (
            not self._corrections
            or check.stamp - self._corrections[-1] >= policy.correction_interval
        ) and len(self._corrections) < policy.max_corrections
        if policy.auto_correct and consistent and good_match and rate_ok:
            correction = compose_pose(self._median_map_camera(), check.camera_base)
            self._corrections.append(check.stamp)
            # Fresh evidence is required after a correction: the next checks
            # must show whether it worked.
            self._streak = []
            return DriftDecision(
                DriftAction.CORRECT, True, consecutive, check.gap, offset,
                correction=correction, reason="consistent drift",
            )
        # Each correction restarts the clock: it is evidence being acted on, and
        # only drift that outlives the correction budget becomes an error.
        clock_start = max([self._drift_since, *self._corrections])
        if check.stamp - clock_start >= policy.escalate_after:
            self._escalated = True
            if not policy.auto_correct:
                reason = "automatic correction disabled"
            elif not consistent:
                reason = "drift direction is not consistent"
            elif not good_match:
                reason = "corrected pose is not a good match"
            else:
                reason = "corrections did not clear the drift"
            return DriftDecision(DriftAction.ESCALATE, True, consecutive, check.gap, offset, reason=reason)
        return DriftDecision(DriftAction.NONE, True, consecutive, check.gap, offset, reason="waiting for a usable correction")

    def _streak_is_consistent(self) -> bool:
        implied = [check.implied_map_camera() for check in self._streak]
        for index, first in enumerate(implied):
            for second in implied[index + 1:]:
                translation, yaw = pose_difference(first, second)
                if translation > self.policy.agreement_translation or yaw > self.policy.agreement_yaw:
                    return False
        return True

    def _median_map_camera(self) -> Pose2D:
        """Per-component median of the implied map->odom corrections.

        Yaw is taken as offsets from the newest correction so the median never
        straddles the +-pi seam.
        """
        implied = [check.implied_map_camera() for check in self._streak]
        reference = implied[-1].yaw
        yaw = reference + statistics.median(
            (pose.yaw - reference + math.pi) % (2.0 * math.pi) - math.pi for pose in implied
        )
        return Pose2D(
            statistics.median(pose.x for pose in implied),
            statistics.median(pose.y for pose in implied),
            (yaw + math.pi) % (2.0 * math.pi) - math.pi,
        )
