"""Pure AMCL readiness and localization-session state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple

from .pose_config import PlanarPose, PoseConfigurationError


class LocalizationState(Enum):
    BOOT = "BOOT"
    UNCOMMISSIONED = "UNCOMMISSIONED"
    WAITING_FOR_INPUTS = "WAITING_FOR_INPUTS"
    SEEDING = "SEEDING"
    VERIFYING = "VERIFYING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    RELOCALIZING = "RELOCALIZING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class LocalizationObservation:
    now: float
    amcl_stamp: float
    x: float
    y: float
    yaw: float
    covariance_x: float
    covariance_y: float
    covariance_yaw: float
    pose_age: float
    scan_age: float
    odom_age: float
    tf_age: float
    tf_ok: bool
    valid_beams: int
    consecutive_scans: int
    median_residual: float
    p80_residual: float


@dataclass(frozen=True)
class LocalizationStatus:
    state: LocalizationState
    ready: bool
    reason: str


def angle_distance(first: float, second: float) -> float:
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


class LocalizationSupervisorCore:
    def __init__(
        self, start_pose: Optional[PlanarPose], *, initial_pose_arm: bool
    ) -> None:
        if start_pose is None and initial_pose_arm:
            raise PoseConfigurationError(
                "initial_pose_arm requires a configured start pose"
            )
        self.start_pose = start_pose
        self.initial_pose_arm = bool(initial_pose_arm)
        self.state = (
            LocalizationState.UNCOMMISSIONED
            if start_pose is None
            else LocalizationState.WAITING_FOR_INPUTS
        )
        self.reason = (
            "start pose is not commissioned"
            if start_pose is None
            else "waiting for map, scan, odometry, and AMCL"
        )
        self._ready = False
        self._seed_time: Optional[float] = None
        self._seed_consumed = False
        self._good_since: Optional[float] = None
        self._initial_ready_completed = False
        self._last_odom: Optional[Tuple[float, str, float, float, float]] = None

    @property
    def ready(self) -> bool:
        return self._ready

    def status(self) -> LocalizationStatus:
        return LocalizationStatus(self.state, self.ready, self.reason)

    def set_inputs_available(self, available: bool) -> LocalizationStatus:
        if self.start_pose is None:
            return self.status()
        if not available:
            self._ready = False
            self.state = LocalizationState.WAITING_FOR_INPUTS
            self.reason = "waiting for required inputs"
            return self.status()
        if not self.initial_pose_arm:
            self.state = LocalizationState.WAITING_FOR_INPUTS
            self.reason = "automatic initial pose seeding is disarmed"
            return self.status()
        if self.state in {
            LocalizationState.WAITING_FOR_INPUTS,
            LocalizationState.RELOCALIZING,
        }:
            self.state = LocalizationState.SEEDING
            self.reason = "initial pose seed requested"
            self._seed_consumed = False
        return self.status()

    def consume_seed_request(self, now: float) -> Optional[PlanarPose]:
        if self.state not in {LocalizationState.SEEDING, LocalizationState.RELOCALIZING}:
            return None
        if self._seed_consumed or not self.initial_pose_arm or self.start_pose is None:
            return None
        self._seed_consumed = True
        self._seed_time = float(now)
        self._good_since = None
        self._ready = False
        self.state = LocalizationState.VERIFYING
        self.reason = "waiting for post-seed AMCL convergence"
        return self.start_pose

    def observe(self, observation: LocalizationObservation) -> LocalizationStatus:
        if self.state not in {
            LocalizationState.VERIFYING,
            LocalizationState.READY,
            LocalizationState.DEGRADED,
        }:
            return self.status()
        failure = self._failure_reason(observation)
        if failure is not None:
            self._good_since = None
            self._ready = False
            if self.state is LocalizationState.READY or self._initial_ready_completed:
                self.state = LocalizationState.DEGRADED
            else:
                self.state = LocalizationState.VERIFYING
            self.reason = failure
            return self.status()

        if self._good_since is None:
            self._good_since = observation.now
        if observation.now - self._good_since >= 2.0:
            self._ready = True
            self._initial_ready_completed = True
            self.state = LocalizationState.READY
            self.reason = "AMCL and scan-to-map checks are stable"
        else:
            self._ready = False
            self.state = LocalizationState.VERIFYING
            self.reason = "readiness hold time not yet satisfied"
        return self.status()

    def _failure_reason(self, observation: LocalizationObservation) -> Optional[str]:
        if self._seed_time is None or observation.amcl_stamp <= self._seed_time:
            return "amcl pose predates current seed"
        if not observation.tf_ok:
            return "required TF chain is unavailable"
        if max(
            observation.pose_age,
            observation.scan_age,
            observation.odom_age,
            observation.tf_age,
        ) > 0.30:
            return "required localization data is stale"
        if (
            observation.covariance_x > 0.04
            or observation.covariance_y > 0.04
            or observation.covariance_yaw > 0.0305
        ):
            return "AMCL covariance exceeds limit"
        if not self._initial_ready_completed and self.start_pose is not None:
            position_delta = math.hypot(
                observation.x - self.start_pose.x,
                observation.y - self.start_pose.y,
            )
            if position_delta > 0.25 or angle_distance(
                observation.yaw, self.start_pose.yaw
            ) > math.radians(15.0):
                return "AMCL start pose delta exceeds limit"
        if observation.valid_beams < 100:
            return "not enough valid scan beams"
        if observation.consecutive_scans < 10:
            return "not enough consecutive scans"
        if observation.median_residual > 0.15:
            return "scan median residual exceeds limit"
        if observation.p80_residual > 0.30:
            return "scan p80 residual exceeds limit"
        return None

    def observe_odometry(
        self,
        stamp: float,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
        *,
        commanded: bool,
    ) -> bool:
        current = (float(stamp), str(frame_id), float(x), float(y), float(yaw))
        restarted = False
        if self._last_odom is not None:
            previous_stamp, previous_frame, previous_x, previous_y, previous_yaw = (
                self._last_odom
            )
            elapsed = current[0] - previous_stamp
            restarted = current[0] < previous_stamp or current[1] != previous_frame
            if not restarted and 0.0 <= elapsed <= 0.50 and not commanded:
                restarted = (
                    math.hypot(current[2] - previous_x, current[3] - previous_y) > 0.50
                    or angle_distance(current[4], previous_yaw) > math.radians(20.0)
                )
        self._last_odom = current
        if restarted:
            self._ready = False
            self._good_since = None
            self._seed_consumed = False
            self.state = LocalizationState.RELOCALIZING
            self.reason = "localization session restart detected"
        return restarted

    def request_reset(self) -> bool:
        if self.start_pose is None or not self.initial_pose_arm:
            return False
        self._ready = False
        self._good_since = None
        self._seed_consumed = False
        self.state = LocalizationState.RELOCALIZING
        self.reason = "explicit reset requested"
        return True
