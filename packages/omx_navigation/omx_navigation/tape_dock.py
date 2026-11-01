"""Stop on a taped line by measuring standing still, never while walking.

The 2026-10-25 drive never stopped because arrival was judged continuously on
a quantity that moved with the robot: the pose wobbled across the goal
checker's tolerance and every crossing started another approach. A trotting
Go1 sways its sensor by a centimetre or two, so the same trap is waiting for
any dock that compares a live fix to a few-centimetre tolerance while walking.

This controller removes it by construction:

1. **Drive** toward the tape midpoint in translation only. Far out it is
   closed loop on the live fix, stopping early by what the command will still
   carry. Inside `nudge_radius` it is a timed **pulse** instead: a constant
   `v` for `distance / v` seconds. A delayed, lagging gait still covers
   `v * T` for a pulse, while a closed loop on a fix that sways a centimetre
   would only chase noise at that range.
2. **Settle**: command zero; the driver drops to STAND and the sway dies out.
3. **Measure**: take the median of the fixes while standing.
4. Inside tolerance -> **ARRIVED**. Otherwise drive again.

Arrival is decided only in step 4, from a standing median, so walking sway can
never flip it. Each round is bounded in time and the rounds are bounded in
number; when they run out the controller accepts a looser stop if it has one
and otherwise aborts -- either way it ends commanding zero. There is no path
that keeps the robot moving until someone kills it.

Two things about the Go1 are unknown until it walks, and both are handled
rather than assumed:

- **The smallest speed it actually follows.** The driver zeroes anything
  under 0.025 m/s (on the vector, not per axis). The command never goes below
  `v_min`, and a round that makes no progress -- a drive that stalls, or a
  pulse the standing measurement says did not move the robot -- raises
  `v_min` for the next one.
- **How far it coasts after a stop.** Stopping early by `v_cmd * lead` takes
  the expected coast off; whatever is left shows up in the standing
  measurement and the next round corrects it.

The axes are judged separately because they matter differently: along the
corridor (x) is the line the user asked to stop on; across it (y) is only
centring between the walls.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class DockPhase(str, Enum):
    IDLE = "IDLE"
    SETTLE = "SETTLE"
    MEASURE = "MEASURE"
    DRIVE = "DRIVE"
    PAUSED = "PAUSED"
    ARRIVED = "ARRIVED"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class TapeDockPolicy:
    # What counts as stopped on the line, judged standing.
    x_tolerance: float = 0.015
    y_tolerance: float = 0.05
    # What a drive aims for: tighter, so a little coast still lands inside.
    x_aim: float = 0.006
    y_aim: float = 0.025
    # Accepted when the rounds run out. Beyond it the dock aborts.
    x_accept: float = 0.04
    y_accept: float = 0.10
    kp: float = 1.0
    v_max: float = 0.08
    # Above the driver's 0.025 m/s deadband with room to spare.
    v_min: float = 0.04
    v_min_step: float = 0.015
    v_min_limit: float = 0.08
    # The command's expected coast: stop this many seconds of travel early.
    lead: float = 0.30
    # Inside this, corrections are timed pulses rather than closed loop.
    nudge_radius: float = 0.06
    # A pulse that moved the standing error by less than this share of what
    # it commanded was not followed.
    pulse_follow_ratio: float = 0.3
    settle_time: float = 1.0
    measure_time: float = 0.6
    min_measurements: int = 3
    # A drive with no progress for this long ends the round as stalled.
    stall_time: float = 2.0
    stall_progress: float = 0.005
    max_rounds: int = 8
    # How long a fix may be missing before a drive stops, and before the
    # whole dock gives up.
    detection_hold: float = 0.4
    detection_timeout: float = 3.0
    # READY may drop for a moment (the safety gate zeroes commands anyway);
    # the dock waits this long for it before aborting.
    ready_grace: float = 5.0
    total_timeout: float = 90.0

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not (self.x_aim < self.x_tolerance <= self.x_accept and self.y_aim < self.y_tolerance <= self.y_accept):
            raise ValueError("expected aim < tolerance <= accept on both axes")
        if not (self.v_min <= self.v_min_limit <= self.v_max):
            raise ValueError("expected v_min <= v_min_limit <= v_max")
        # The driver zeroes |v| < 0.025 m/s; a floor under that would stall
        # silently, which is exactly the dead zone that cost two sessions.
        if self.v_min <= 0.025:
            raise ValueError("v_min must exceed the driver's 0.025 m/s linear deadband")
        for name in ("min_measurements", "max_rounds"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be an integer")


@dataclass(frozen=True)
class DockCommand:
    vx: float
    vy: float
    phase: DockPhase
    reason: str = ""
    # Everything the decision was made from, for the decision log.
    detail: dict = field(default_factory=dict)


class TapeDock:
    """Drive the tape midpoint onto the LiDAR, judging arrival standing."""

    def __init__(self, policy: TapeDockPolicy | None = None) -> None:
        self.policy = policy or TapeDockPolicy()
        self.reset()

    def reset(self) -> None:
        self._phase = DockPhase.IDLE
        self._target = (0.0, 0.0)
        self._started = None
        self._phase_since = None
        self._round = 0
        self._v_min = self.policy.v_min
        self._samples = []
        self._last_fix_at = None
        self._last_error = None
        self._best_error = None
        self._progress_at = None
        self._paused_from = None
        self._history = []
        self._pulse = None
        self._pulse_from = None

    @property
    def phase(self) -> DockPhase:
        return self._phase

    @property
    def rounds(self) -> list:
        """One entry per standing measurement: what each round achieved."""
        return list(self._history)

    def start(self, now: float, target_x: float = 0.0, target_y: float = 0.0) -> None:
        """Begin with a settle, so whatever drove here has come to rest."""
        self.reset()
        self._target = (float(target_x), float(target_y))
        self._started = now
        self._enter(DockPhase.SETTLE, now)

    def update(self, now: float, fix, *, localization_ready: bool = True,
               manual_override: bool = False, execution_fault: bool = False) -> DockCommand:
        """One control cycle. `fix` has `mid_x`, `mid_y` in body_nav, or is None."""
        p = self.policy
        if self._phase in (DockPhase.IDLE, DockPhase.ARRIVED, DockPhase.ABORTED):
            return self._command(self._phase)
        if manual_override:
            return self._abort("manual override")
        if execution_fault:
            return self._abort("robot not following commands")
        if now - self._started > p.total_timeout:
            return self._abort(f"dock took longer than {p.total_timeout:.0f} s")

        if fix is not None:
            self._last_fix_at = now
            error = (fix.mid_x - self._target[0], fix.mid_y - self._target[1])
        else:
            error = None
        if self._last_fix_at is None:
            if now - self._started > p.detection_timeout:
                return self._abort("tape pair never seen")
        elif now - self._last_fix_at > p.detection_timeout:
            return self._abort(f"tape pair lost for {now - self._last_fix_at:.1f} s")

        if not localization_ready:
            if self._phase is not DockPhase.PAUSED:
                self._paused_from = (self._phase, now)
                self._enter(DockPhase.PAUSED, now)
            if now - self._phase_since > p.ready_grace:
                return self._abort(f"localization not READY for {p.ready_grace:.0f} s")
            return self._command(DockPhase.PAUSED, "waiting for READY", error)
        if self._phase is DockPhase.PAUSED:
            # Whatever moved during the pause, a fresh standing look decides.
            self._enter(DockPhase.SETTLE, now)

        if self._phase is DockPhase.SETTLE:
            if now - self._phase_since >= p.settle_time:
                self._samples = []
                self._enter(DockPhase.MEASURE, now)
            else:
                return self._command(DockPhase.SETTLE, "settling", error)

        if self._phase is DockPhase.MEASURE:
            if error is not None:
                self._samples.append(error)
            enough = len(self._samples) >= p.min_measurements and now - self._phase_since >= p.measure_time
            if not enough:
                return self._command(DockPhase.MEASURE, "measuring", error)
            return self._judge(now)

        # DRIVE
        if self._pulse is not None:
            vx, vy, until = self._pulse
            if now >= until:
                self._pulse = None
                return self._end_drive(now, "pulse done")
            return self._command(DockPhase.DRIVE, "pulse", error, vx, vy)
        if error is None:
            if now - self._last_fix_at > p.detection_hold:
                return self._end_drive(now, "tape pair dropped out while driving")
            return self._command(DockPhase.DRIVE, "holding for a fix", None)
        ex, ey = error
        self._last_error = error
        magnitude = math.hypot(ex, ey if abs(ey) > p.y_aim else 0.0)
        if self._best_error is None or magnitude < self._best_error - p.stall_progress:
            self._best_error = magnitude
            self._progress_at = now
        if now - self._progress_at > p.stall_time:
            self._v_min = min(self._v_min + p.v_min_step, p.v_min_limit)
            return self._end_drive(now, f"no progress for {p.stall_time:.0f} s; v_min now {self._v_min:.3f}")
        vx, vy = self._velocity(ex, ey)
        speed = math.hypot(vx, vy)
        # Stop early by what the command will still carry.
        reach_x = abs(ex) <= p.x_aim + (abs(vx) * p.lead if speed else 0.0)
        reach_y = abs(ey) <= p.y_aim + (abs(vy) * p.lead if speed else 0.0)
        if (reach_x and reach_y) or speed == 0.0:
            return self._end_drive(now, "reached the aim")
        return self._command(DockPhase.DRIVE, "driving", error, vx, vy)

    # -- internals --------------------------------------------------------

    def _velocity(self, ex: float, ey: float):
        p = self.policy
        # Inside its aim an axis stops contributing, so the residual on the
        # other one is not dragged out by a centring the user does not need.
        gx = ex if abs(ex) > p.x_aim else 0.0
        gy = ey if abs(ey) > p.y_aim else 0.0
        norm = math.hypot(gx, gy)
        if norm == 0.0:
            return 0.0, 0.0
        speed = min(max(p.kp * norm, self._v_min), p.v_max)
        return speed * gx / norm, speed * gy / norm

    def _judge(self, now: float) -> DockCommand:
        p = self.policy
        ex = _median([s[0] for s in self._samples])
        ey = _median([s[1] for s in self._samples])
        self._round += 1
        within = abs(ex) <= p.x_tolerance and abs(ey) <= p.y_tolerance
        followed = None
        if self._pulse_from is not None:
            (bx, by), commanded = self._pulse_from
            moved = math.hypot(ex - bx, ey - by)
            followed = moved >= p.pulse_follow_ratio * commanded
            if not followed:
                self._v_min = min(self._v_min + p.v_min_step, p.v_min_limit)
            self._pulse_from = None
        self._history.append({"round": self._round, "ex": ex, "ey": ey, "samples": len(self._samples),
                              "v_min": self._v_min, "within": within, "pulse_followed": followed,
                              "t": now - self._started})
        if within:
            self._enter(DockPhase.ARRIVED, now)
            return self._command(DockPhase.ARRIVED, f"stopped {ex * 100:+.1f} cm along, {ey * 100:+.1f} cm across",
                                 (ex, ey))
        if self._round >= p.max_rounds:
            if abs(ex) <= p.x_accept and abs(ey) <= p.y_accept:
                self._enter(DockPhase.ARRIVED, now)
                return self._command(DockPhase.ARRIVED,
                                     f"accepted after {self._round} rounds at {ex * 100:+.1f} / {ey * 100:+.1f} cm",
                                     (ex, ey))
            return self._abort(f"{self._round} rounds left it {ex * 100:+.1f} / {ey * 100:+.1f} cm off")
        self._best_error = None
        self._progress_at = now
        self._enter(DockPhase.DRIVE, now)
        gx = ex if abs(ex) > p.x_aim else 0.0
        gy = ey if abs(ey) > p.y_aim else 0.0
        distance = math.hypot(gx, gy)
        if 0.0 < distance <= p.nudge_radius:
            speed = self._v_min
            vx, vy = speed * gx / distance, speed * gy / distance
            self._pulse = (vx, vy, now + distance / speed)
            self._pulse_from = ((ex, ey), distance)
            return self._command(DockPhase.DRIVE, f"round {self._round + 1}: pulse {distance * 100:.1f} cm",
                                 (ex, ey), vx, vy)
        vx, vy = self._velocity(ex, ey)
        return self._command(DockPhase.DRIVE, f"round {self._round + 1}", (ex, ey), vx, vy)

    def _end_drive(self, now: float, reason: str) -> DockCommand:
        self._enter(DockPhase.SETTLE, now)
        return self._command(DockPhase.SETTLE, reason, self._last_error)

    def _abort(self, reason: str) -> DockCommand:
        self._phase = DockPhase.ABORTED
        return self._command(DockPhase.ABORTED, reason)

    def _enter(self, phase: DockPhase, now: float) -> None:
        self._phase = phase
        self._phase_since = now

    def _command(self, phase, reason="", error=None, vx=0.0, vy=0.0) -> DockCommand:
        detail = {"round": self._round, "v_min": self._v_min}
        if error is not None:
            detail["ex"], detail["ey"] = error
        return DockCommand(vx, vy, phase, reason, detail)


def _median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0
