"""Check the tape, record a taped spot, or localize from the start tape.

    ros2 run omx_navigation tape_waypoint check
    ros2 run omx_navigation tape_waypoint record start
    ros2 run omx_navigation tape_waypoint init

`check` is for hanging the tape: it reports every bright strip the LiDAR sees,
how much of it falls in the detector's height band, and whether the pair
would dock. It needs only the Livox driver.

`record NAME` stores the spot the robot is standing at. It records the tape,
not the robot: the strips' midpoint in `map`, the heading squared to the
corridor, and the measured wall-to-wall gap. That ties the waypoint to where
localization believes the tape is, which is exactly the frame Nav2 will be
sent in, so a localization bias along the corridor is the same at recording
and at navigation and cancels -- a click on the map would carry it instead.
It needs the full stack, READY.

`init` replaces the 2D Pose Estimate click when the robot stands at the
recorded start tape, facing the way it faced when the spot was recorded. It
publishes `/initialpose` once, exactly as RViz would.
"""

import argparse
import math
import os
import sys
import time

from omx_navigation.corridor_dock import (
    CorridorDockPolicy,
    corridor_fix,
    median_fix,
    robot_pose_at,
    survey_strips,
    tape_pose,
    unmeasured_corridor,
)
from omx_navigation.mission import record_waypoint
from omx_navigation.mission_file import default_mission_path, load_mission, save_mission

PACKAGE = "omx_navigation"
# 20 points per strip per detection; check warns below twice that, because
# the edges of a strip return dimmer and walking pitches the band.
COMFORTABLE_POINTS = 40
# RViz's own 2D Pose Estimate covariance, so the supervisor sees the same
# uncertainty it is tuned for.
INITIAL_COVARIANCE_XY = 0.25
INITIAL_COVARIANCE_YAW = 0.06853891945200942


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="tape_waypoint", description=__doc__.split("\n\n")[0])
    parser.add_argument("--mission", default=None, help="mission file (default: $GO1_MISSION_FILE or ~/.ros/go1_tape_mission.yaml)")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--seconds", type=float, default=2.0, help="how long to look")
    parser.add_argument("--window", type=float, default=0.2, help="seconds of cloud per detection")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check", help="report the strips the LiDAR sees")
    record = commands.add_parser("record", help="record the taped spot the robot stands at")
    record.add_argument("name")
    record.add_argument("--map-frame", default="map")
    record.add_argument("--base-frame", default="body_nav")
    init = commands.add_parser("init", help="publish /initialpose from a recorded tape")
    init.add_argument("name", nargs="?", default="start")
    init.add_argument("--map-frame", default="map")
    args = parser.parse_args(argv)
    mission_path = args.mission or str(default_mission_path())

    import rclpy

    rclpy.init()
    listener = _CloudListener(args.lidar_topic, args.window)
    try:
        if args.command == "check":
            return _check(listener, args)
        if args.command == "record":
            return _record(listener, args, mission_path)
        return _init(listener, args, mission_path)
    finally:
        listener.destroy_node()
        rclpy.try_shutdown()


class _CloudListener:
    """Collects Livox frames into windows of `window` seconds."""

    def __init__(self, topic: str, window: float) -> None:
        from livox_ros_driver2.msg import CustomMsg
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        self.node = Node("tape_waypoint")
        self.window = window
        self._frames = []
        self.node.create_subscription(CustomMsg, topic, self._on_frame, qos_profile_sensor_data)

    def _on_frame(self, message) -> None:
        self._frames.append((time.monotonic(),
                             [(p.x, p.y, p.z, p.reflectivity) for p in message.points]))

    def windows(self, seconds: float):
        """Spin for `seconds` and return the frames grouped into windows."""
        import rclpy

        self._frames = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        groups = []
        current, started = [], None
        for stamp, points in self._frames:
            if started is None or stamp - started > self.window:
                if current:
                    groups.append(current)
                current, started = [], stamp
            current.extend(points)
        if current:
            groups.append(current)
        return groups

    def destroy_node(self) -> None:
        self.node.destroy_node()


def _look(listener, seconds, policy):
    windows = listener.windows(seconds)
    if not windows:
        print("No Livox frames arrived. Is livox_ros_driver2 running on this ROS_DOMAIN_ID?")
        return None, []
    fix = median_fix(corridor_fix(points, policy) for points in windows)
    return fix, windows


def _check(listener, args) -> int:
    policy = unmeasured_corridor()
    fix, windows = _look(listener, args.seconds, policy)
    if not windows:
        return 2
    band = (policy.markers.band_lo, policy.markers.band_hi)
    surveys = [survey_strips(points, policy) for points in windows]
    # Report the window with the most strips, then the per-window averages.
    widest = max(surveys, key=len)
    print(f"{len(windows)} windows of {args.window:.1f} s. Detector band z {band[0]:+.2f}..{band[1]:+.2f} m "
          f"(LiDAR frame), reflectivity >= {policy.markers.reflectivity_threshold:.0f}.")
    if not widest:
        print("No tape-bright returns within 2 m. Is the tape retroreflective (prismatic)?")
        return 1
    problems = 0
    candidates = 0
    for strip in widest:
        same = [s for survey in surveys for s in survey
                if math.hypot(s.x - strip.x, s.y - strip.y) < 0.15]
        in_band = sum(s.in_band for s in same) / len(windows)
        z_lo = min(s.z_lo for s in same)
        z_hi = max(s.z_hi for s in same)
        head = (f"  strip at bearing {math.degrees(strip.bearing):+6.1f} deg, {strip.range:.2f} m: "
                f"{in_band:5.1f} pts/window in band, seen z {z_lo:+.2f}..{z_hi:+.2f}, "
                f"peak {max(s.peak for s in same):.0f}")
        if in_band < policy.markers.min_points_per_strip:
            # Lights, signs, anything shiny above or below the band: the
            # detector never counts it, so it is no reason to worry.
            print(head + "  (outside the band, ignored by the detector)")
            continue
        candidates += 1
        # Coverage is advice; only a thin strip is a problem. The bottom of
        # what can be seen is the -7 deg FOV edge, not the tape's end.
        notes = []
        if in_band < COMFORTABLE_POINTS:
            notes.append(f"only {in_band:.0f} points in band (want {COMFORTABLE_POINTS})")
            if z_hi < band[1]:
                notes.append(f"top {100 * (band[1] - z_hi):.0f} cm short of the band")
        problems += bool(notes)
        print(head + ("  <- " + "; ".join(notes) if notes else "  ok"))
    if candidates != 2:
        print(f"{candidates} strips in the band; the dock needs exactly two within 2 m.")
        problems += 1
    if fix is None:
        print("Pair NOT detected: the dock would refuse here.")
        return 1
    print(f"Pair detected: midpoint x {fix.mid_x:+.3f} y {fix.mid_y:+.3f} m, wall-to-wall {fix.width:.3f} m, "
          f"yaw {math.degrees(fix.yaw_error):+.1f} deg.")
    return 1 if problems else 0


def _record(listener, args, mission_path) -> int:
    if not os.path.exists(mission_path):
        from ament_index_python.packages import get_package_share_directory

        template = os.path.join(get_package_share_directory(PACKAGE), "config", "tape_mission.yaml")
        mission = load_mission(template)
        print(f"Seeding {mission_path} from {template}")
    else:
        mission = load_mission(mission_path)
    if args.name not in mission.waypoints:
        print(f"{args.name!r} is not a waypoint in {mission_path}; known: {', '.join(mission.waypoints)}")
        return 2
    if not _wait_ready(listener.node, 3.0):
        print("Localization is not READY; the recorded pose would mean nothing. Localize first.")
        return 1
    pose = _lookup_pose(listener.node, args.map_frame, args.base_frame, 3.0)
    if pose is None:
        print(f"No transform {args.map_frame} -> {args.base_frame}.")
        return 1
    policy = unmeasured_corridor()
    fix, windows = _look(listener, args.seconds, policy)
    if fix is None:
        print("Tape pair not detected here. Run `tape_waypoint check` and fix the tape first.")
        return 1
    x, y, yaw = tape_pose(*pose, fix)
    # `+ 0.0` keeps a rounded -0.0 out of the file.
    mission = record_waypoint(mission, args.name, round(x, 4) + 0.0, round(y, 4) + 0.0,
                              round(yaw, 4) + 0.0, round(fix.width, 4))
    save_mission(mission_path, mission)
    print(f"Recorded {args.name}: x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f} deg, "
          f"wall-to-wall {fix.width:.3f} m (robot was {fix.correction(policy):.3f} m off the tape) "
          f"-> {mission_path}")
    return 0


def _init(listener, args, mission_path) -> int:
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    mission = load_mission(mission_path)
    spot = mission.waypoint(args.name)
    if not spot.recorded_at_tape:
        print(f"{args.name!r} was never recorded at its tape.")
        return 1
    policy = CorridorDockPolicy().with_separation(spot.marker_separation)
    fix, _windows = _look(listener, args.seconds, policy)
    if fix is None:
        print(f"The {args.name} tape pair is not in view; is the robot standing at it?")
        return 1
    if fix.correction(policy) > policy.max_correction:
        print(f"The pair is {fix.correction(policy):.2f} m away; stand the robot at the tape.")
        return 1
    x, y, yaw = robot_pose_at(spot.x, spot.y, spot.yaw, fix)
    publisher = listener.node.create_publisher(
        PoseWithCovarianceStamped, "/initialpose",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    if not _wait_for_subscriber(listener.node, publisher, 3.0):
        print("Nothing subscribes to /initialpose; is the localization supervisor running?")
        return 1
    # The writer sees the reader before the reader has matched the writer;
    # a message sent in that gap is dropped even on a reliable topic, which is
    # what happened on the robot the first time. Give discovery a second.
    _spin_for(listener.node, 1.0)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = args.map_frame
    message.header.stamp = listener.node.get_clock().now().to_msg()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.pose.covariance[0] = INITIAL_COVARIANCE_XY
    message.pose.covariance[7] = INITIAL_COVARIANCE_XY
    message.pose.covariance[35] = INITIAL_COVARIANCE_YAW
    publisher.publish(message)
    # Stay up long enough for the reliable writer to deliver it.
    _spin_for(listener.node, 1.0)
    print(f"Published /initialpose x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f} deg from the {args.name} tape. "
          "Watch /localization_supervisor/status for READY.")
    return 0


def _wait_ready(node, seconds) -> bool:
    from std_msgs.msg import Bool

    seen = []
    subscription = node.create_subscription(Bool, "/localization_supervisor/ready",
                                            lambda m: seen.append(bool(m.data)), 10)
    _spin_for(node, seconds, until=lambda: bool(seen))
    node.destroy_subscription(subscription)
    return bool(seen) and seen[-1]


def _lookup_pose(node, map_frame, base_frame, seconds):
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    _listener = TransformListener(buffer, node)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        _spin_for(node, 0.1)
        if buffer.can_transform(map_frame, base_frame, Time()):
            transform = buffer.lookup_transform(map_frame, base_frame, Time()).transform
            q = transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return transform.translation.x, transform.translation.y, yaw
    return None


def _wait_for_subscriber(node, publisher, seconds) -> bool:
    _spin_for(node, seconds, until=lambda: publisher.get_subscription_count() > 0)
    return publisher.get_subscription_count() > 0


def _spin_for(node, seconds, until=None) -> None:
    import rclpy

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if until is not None and until():
            return


if __name__ == "__main__":
    sys.exit(main())
