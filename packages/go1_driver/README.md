# Go1 ROS2 Driver

ROS2 Humble port of the existing Go1 `cmd_vel` safety bridge. The node is
disarmed by default, so it can be built and tested without the Unitree SDK or a
physical robot.

## Build

Copy this package into the ROS2 workspace and build it.

```bash
cp -a /mnt/t500/go1_ros2_project/packages/go1_driver /mnt/t500/go1_ros2_ws/src/go1_driver
cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select go1_driver
source install/setup.bash
```

## Dry-run

```bash
ros2 launch go1_driver go1_driver.launch.py arm:=false
```

In another terminal:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08, y: 0.08, z: 0.0}, angular: {z: 0.0}}"
ros2 topic echo /go1/cmd_vel_applied
ros2 topic echo /go1/control_state
```

Stop the publisher and confirm the watchdog changes the applied command to
zero after `cmd_timeout`.

## Armed mode

Direct `go1_driver arm:=true` is intentionally rejected because it bypasses
the localization gate, recording, ARM64 wrapper checks, and live sensor checks.
After the full dry-run and emergency-stop checklist, use only the canonical
field runner from the repository root:

```bash
./migration/jetson_field_deploy.sh armed GO1_ARMED_AND_ESTOP_READY /mnt/t500/go1_ros2_ws
```

The configured `sdk_path` is under `/mnt/t500/go1_sdk`. It must contain the
archived Unitree SDK v3.8.6 rebuilt for Python 3.10; do not mix v3.5.1 files.
It must contain a matching
`robot_interface.cpython-<ABI>-aarch64-linux-gnu.so`.

## Who is in control

The Go1's sport controller follows the handheld remote over any HighCmd. On
2026-09-18 the stack kept commanding a Nav2 goal for minutes while the
operator drove the robot by remote, and could not tell (see
`migration/FIELD_SESSION_2026-09-18.md`). While armed the driver now reads
every HighState reply and publishes:

| topic | type | meaning |
|---|---|---|
| `/go1/robot_state` | `std_msgs/String` (JSON, 10 Hz) | link, mode, velocity, `rangeObstacle`, battery, remote frame, requested vs applied command, execution verdict |
| `/go1/manual_override` | `std_msgs/Bool` (10 Hz) | a key is pressed or a stick is past `remote_stick_deadband`; holds until the remote has been idle for `override_release_s` |
| `/go1/execution_fault` | `std_msgs/Bool` (10 Hz) | the robot refused commanded motion for `refusal_hold_s`, or moved without a command for `uncommanded_hold_s` (judged by net FAST-LIO odometry on `odom_topic`) |

Either one holds stand, and `rviz_goal_bridge` cancels the Nav2 goal on it.
Afterwards nothing moves until `/cmd_vel` has been zero or silent for
`rearm_zero_s`, so a goal set before a takeover cannot resume on its own. A
powered remote with centred sticks is not an override.

Besides every change of command reason, the log records requested vs applied
command, the robot's own mode and velocity, the remote state and the executed
share once a second while anything is moving or held.
