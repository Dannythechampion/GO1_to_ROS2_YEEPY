#!/usr/bin/env python3
"""Decide why the Go1 answers HighLevel UDP but does not walk.

The 2026-08-18 field session reached READY localization and confirmed that
commands arrive at the SDK with correct values, yet the robot never moved and
reported `mode=1` continuously while being commanded `mode=0`. That points at
something other than this stack owning the sport controller. This tool answers
that question from data instead of inference.

Two facts settle it, and both are readable from `HighState`:

* `wirelessRemote` carries the remote controller's raw frame. A powered-on RC
  puts a `0xFE 0xEF` header and changing stick bytes there, and the sport
  controller follows the RC over any external HighCmd. Silent bytes rule the
  RC out.
* `mode` reports the mode the sport controller actually adopted. If it never
  follows what we command, an onboard program owns the controller.

Observation mode never commands motion: it holds `mode=0` (idle stand, the
neutral this driver already uses) and only listens. The walk probe moves the
robot and therefore demands the same explicit token the driver does.

Usage:
  python3 diagnose_go1_walk.py [--sdk-path DIR] [--seconds N]
  python3 diagnose_go1_walk.py --walk GO1_ARMED_AND_ESTOP_READY [--vx 0.1]
"""

from __future__ import annotations

import argparse
import importlib
import os
import statistics
import sys
import time

ARMED_TOKEN = "GO1_ARMED_AND_ESTOP_READY"
DEFAULT_SDK_PATH = "/mnt/t500/go1_sdk/unitree_legged_sdk/lib/python/arm64"
HIGHLEVEL = 0xEE
IDLE_MODE = 0
WALK_MODE = 2
TROT_GAIT = 1
RC_HEADER = (0xFE, 0xEF)
# rangeObstacle reports the Go1's own proximity sensors. A reading pinned well
# inside this bound while the robot is standing still is not a passer-by; it is
# something bolted in front of the sensor, and the sport controller refuses
# forward walking toward it while still allowing rotation in place.
OBSTACLE_BLOCK_M = 0.50
OBSTACLE_CLEAR_M = 2.0
# The controller needs a moment to switch modes, so a phase legitimately starts
# in the previous mode. Requiring a quarter of the window tolerates that while
# still rejecting the handful of stray samples that a refusal produces.
ADOPTION_FRACTION = 0.25

MODE_NAMES = {
    0: "idle stand",
    1: "force stand",
    2: "velocity walk",
    3: "position walk",
    4: "path",
    5: "stand down",
    6: "stand up",
    7: "damping",
    8: "recovery stand",
}


def load_sdk(sdk_path: str):
    expanded = os.path.abspath(os.path.expanduser(sdk_path))
    if not os.path.isdir(expanded):
        raise SystemExit(f"ERROR: Unitree SDK python directory not found: {expanded}")
    if expanded not in sys.path:
        sys.path.insert(0, expanded)
    try:
        return importlib.import_module("robot_interface")
    except ImportError as exc:
        raise SystemExit(f"ERROR: cannot import robot_interface from {expanded}: {exc}")


def remote_bytes(state) -> tuple[int, ...]:
    """Return wirelessRemote as plain ints, whatever the binding hands back."""
    raw = getattr(state, "wirelessRemote", None)
    if raw is None:
        return ()
    try:
        return tuple(int(value) & 0xFF for value in raw)
    except TypeError:
        return ()


class Phase:
    """Accumulate what the robot reported while one command was held."""

    def __init__(self, label: str, commanded_mode: int) -> None:
        self.label = label
        self.commanded_mode = commanded_mode
        self.modes: list[int] = []
        self.body_heights: list[float] = []
        self.forward_speeds: list[float] = []
        self.remote_frames: set[tuple[int, ...]] = set()
        self.remote_header_seen = False
        self.remote_nonzero_seen = False
        self.obstacle: list[list[float]] = [[], [], [], []]
        self.samples = 0

    def observe(self, state) -> None:
        self.samples += 1
        self.modes.append(int(state.mode))
        self.body_heights.append(float(state.bodyHeight))
        try:
            self.forward_speeds.append(float(state.velocity[0]))
        except (IndexError, TypeError):
            pass
        ranges = getattr(state, "rangeObstacle", None)
        if ranges is not None:
            for index in range(min(4, len(ranges))):
                try:
                    self.obstacle[index].append(float(ranges[index]))
                except (TypeError, ValueError):
                    pass
        frame = remote_bytes(state)
        if frame:
            self.remote_frames.add(frame)
            if tuple(frame[:2]) == RC_HEADER:
                self.remote_header_seen = True
            if any(frame):
                self.remote_nonzero_seen = True

    @property
    def mode_histogram(self) -> dict[int, int]:
        histogram: dict[int, int] = {}
        for mode in self.modes:
            histogram[mode] = histogram.get(mode, 0) + 1
        return histogram

    @property
    def adoption_fraction(self) -> float:
        if not self.modes:
            return 0.0
        return self.mode_histogram.get(self.commanded_mode, 0) / len(self.modes)

    @property
    def followed_command(self) -> bool:
        return self.adoption_fraction >= ADOPTION_FRACTION

    @property
    def blocking_sensors(self) -> list[tuple[int, float, float]]:
        """Sensors that stayed close for the whole phase, worst first."""
        blocking = []
        for index, samples in enumerate(self.obstacle):
            if not samples:
                continue
            low, high = min(samples), max(samples)
            # An all-zero channel is an unused sensor, not a contact reading.
            if high <= 0.0 or high >= OBSTACLE_BLOCK_M:
                continue
            blocking.append((index, low, high))
        blocking.sort(key=lambda item: item[2])
        return blocking

    def report(self) -> None:
        print(f"\n--- {self.label} ---")
        if not self.samples:
            print("  no HighState replies at all -- the robot is not answering")
            return
        commanded = MODE_NAMES.get(self.commanded_mode, "?")
        print(f"  commanded mode : {self.commanded_mode} ({commanded}), {self.samples} replies")
        parts = [
            f"{mode} ({MODE_NAMES.get(mode, '?')}) x{count}"
            for mode, count in sorted(self.mode_histogram.items(), key=lambda kv: -kv[1])
        ]
        print(f"  reported mode  : {', '.join(parts)}")
        print(
            "  followed our command: %s (%.1f%% of replies)"
            % ("YES" if self.followed_command else "NO", 100.0 * self.adoption_fraction)
        )
        if self.body_heights:
            print(
                "  bodyHeight     : min %.3f  max %.3f"
                % (min(self.body_heights), max(self.body_heights))
            )
        if self.forward_speeds:
            print(
                "  velocity[0]    : min %.3f  max %.3f  mean %.3f"
                % (
                    min(self.forward_speeds),
                    max(self.forward_speeds),
                    statistics.fmean(self.forward_speeds),
                )
            )
        for index, samples in enumerate(self.obstacle):
            if samples and max(samples) > 0.0:
                print(
                    "  rangeObstacle[%d]: min %.3f  max %.3f%s"
                    % (
                        index,
                        min(samples),
                        max(samples),
                        "  <-- pinned close" if max(samples) < OBSTACLE_BLOCK_M else "",
                    )
                )
        print(
            "  wirelessRemote : header=%s  nonzero=%s  distinct frames=%d"
            % (
                "PRESENT" if self.remote_header_seen else "absent",
                "yes" if self.remote_nonzero_seen else "no",
                len(self.remote_frames),
            )
        )


def run_phase(udp, cmd, state, label, mode, seconds, rate, vx=0.0, yaw=0.0) -> Phase:
    phase = Phase(label, mode)
    period = 1.0 / rate
    deadline = time.monotonic() + seconds
    print(f"\n[{label}] holding mode={mode} for {seconds:.0f}s at {rate:.0f} Hz ...")
    while time.monotonic() < deadline:
        udp.Recv()
        udp.GetRecv(state)
        phase.observe(state)

        cmd.mode = mode
        cmd.gaitType = TROT_GAIT if mode == WALK_MODE else 0
        cmd.speedLevel = 0
        cmd.footRaiseHeight = 0.0
        cmd.bodyHeight = 0.0
        cmd.euler = [0.0, 0.0, 0.0]
        cmd.velocity = [float(vx), 0.0]
        cmd.yawSpeed = float(yaw)
        cmd.reserve = 0
        udp.SetSend(cmd)
        udp.Send()
        time.sleep(period)
    return phase


def verdict(idle: Phase, walk: Phase | None) -> int:
    print("\n================ VERDICT ================")
    phases = [phase for phase in (idle, walk) if phase is not None]

    if not any(phase.samples for phase in phases):
        print("NO REPLY: the robot never answered on the HighLevel port.")
        print("  Check the 192.168.123.0/24 link and that sport mode is up.")
        return 2

    rc_active = any(phase.remote_header_seen for phase in phases)
    rc_changing = any(len(phase.remote_frames) > 1 for phase in phases)
    if rc_active:
        print("CAUSE FOUND: the remote controller is transmitting.")
        print(
            "  wirelessRemote carries the RC's 0xFE 0xEF frame%s."
            % (" and the bytes keep changing" if rc_changing else "")
        )
        print("  The sport controller follows the RC over external HighCmd, so")
        print("  this stack's commands are accepted but never acted on.")
        print("  FIX: power the remote controller OFF, then re-run this tool.")
        return 1

    if not idle.followed_command:
        stuck = sorted(idle.mode_histogram.items(), key=lambda kv: -kv[1])
        stuck_mode = stuck[0][0] if stuck else -1
        print("CAUSE FOUND: another program owns the sport controller.")
        print(
            "  We commanded mode=0 (idle stand) but the robot stayed at mode=%d (%s),"
            % (stuck_mode, MODE_NAMES.get(stuck_mode, "?"))
        )
        print("  and the RC is silent, so the RC is not the one holding it.")
        print("  FIX: log into the Go1 boards and stop the autostart programs that")
        print("       command sport mode, then re-run:")
        print("         ssh pi@192.168.123.161      # and the Nano boards .13 / .14 / .15")
        print("         ps aux | grep -Ei 'ai|vision|autostart|sport|mqtt'")
        print("  Also confirm the robot is in normal sport mode, not AI mode.")
        return 1

    if walk is None:
        print("IDLE OK: the robot followed mode=0 and the RC is silent.")
        print("  Nothing is holding the controller while idle.")
        print("  Re-run with --walk %s to test whether mode=2 is honoured." % ARMED_TOKEN)
        return 0

    if not walk.followed_command:
        print("CAUSE FOUND: idle is accepted but forward walking is refused.")
        print("  mode=0 was adopted, mode=2 was not. The controller is up and")
        print("  listening, so this is a refusal of the velocity mode itself.")
        blocking = walk.blocking_sensors or idle.blocking_sensors
        if blocking:
            print("")
            print("  The Go1's own proximity sensors were pinned close the whole time:")
            for index, low, high in blocking:
                print(
                    "    rangeObstacle[%d] held %.3f-%.3f m (clear reads %.1f m)"
                    % (index, low, high, OBSTACLE_CLEAR_M)
                )
            print("  A reading that never moves while the robot stands still is not a")
            print("  passer-by. The sport controller refuses to walk into it, which is")
            print("  why rotation in place is accepted and forward motion is not.")
            print("  FIX: look at what sits within ~%.0f cm of the front sensors --" % (100 * OBSTACLE_BLOCK_M))
            print("       the LiDAR mount, its bracket or cabling are the usual culprits.")
            print("       Clear it, then re-run this probe.")
            print("  To tell a fixed obstruction from the room: carry the robot to open")
            print("  space and re-run in observation mode. Readings that stay pinned are")
            print("  mounted on the robot; readings that open up were the room.")
        else:
            print("  No proximity sensor was pinned, so obstacle avoidance is not it.")
            print("  FIX: confirm the robot is in normal sport mode rather than AI mode,")
            print("       stand it fully with the RC, then re-run this probe.")
        return 1

    moved = bool(walk.forward_speeds) and max(walk.forward_speeds) > 0.02
    if moved:
        print("PASS: the robot adopted mode=2 and reported forward velocity.")
        print("  The HighLevel command path is healthy end to end.")
        return 0

    print("INCONCLUSIVE: mode=2 was adopted but no forward velocity was reported.")
    print("  Watch the robot during the walk phase and note whether the legs move.")
    return 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-path", default=DEFAULT_SDK_PATH)
    parser.add_argument("--robot-ip", default="192.168.123.161")
    parser.add_argument("--robot-port", type=int, default=8082)
    parser.add_argument("--local-port", type=int, default=8080)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--walk",
        default="",
        help=f"move the robot; requires the exact token {ARMED_TOKEN}",
    )
    parser.add_argument("--vx", type=float, default=0.10)
    arguments = parser.parse_args(argv)

    if arguments.seconds <= 0.0:
        raise SystemExit("ERROR: --seconds must be positive")
    if arguments.rate <= 0.0:
        raise SystemExit("ERROR: --rate must be positive")
    walk_requested = bool(arguments.walk)
    if walk_requested and arguments.walk != ARMED_TOKEN:
        raise SystemExit(f"ERROR: --walk requires the exact token {ARMED_TOKEN}")
    if walk_requested and not 0.0 < arguments.vx <= 0.20:
        raise SystemExit("ERROR: --vx must be within (0, 0.20] m/s")

    sdk = load_sdk(arguments.sdk_path)
    udp = sdk.UDP(HIGHLEVEL, arguments.local_port, arguments.robot_ip, arguments.robot_port)
    cmd = sdk.HighCmd()
    state = sdk.HighState()
    udp.InitCmdData(cmd)

    print(f"Probing {arguments.robot_ip}:{arguments.robot_port} from local port {arguments.local_port}")
    if walk_requested:
        print("!! WALK PROBE ARMED -- clear the area and keep the e-stop within reach !!")
    else:
        print("Observation only: mode=0 is held throughout, no motion is commanded.")

    idle = run_phase(
        udp, cmd, state, "phase 1: idle stand", IDLE_MODE, arguments.seconds, arguments.rate
    )
    walk = None
    if walk_requested:
        walk = run_phase(
            udp,
            cmd,
            state,
            "phase 2: velocity walk",
            WALK_MODE,
            arguments.seconds,
            arguments.rate,
            vx=arguments.vx,
        )
        run_phase(
            udp, cmd, state, "phase 3: return to idle", IDLE_MODE, 2.0, arguments.rate
        )

    idle.report()
    if walk is not None:
        walk.report()
    return verdict(idle, walk)


if __name__ == "__main__":
    raise SystemExit(main())
