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

Do not arm until the wrapper has been rebuilt for the new Jetson Python ABI,
the dry-run checks pass, and the robot is supported with an emergency stop
ready.

```bash
ros2 launch go1_driver go1_driver.launch.py arm:=true
```

The configured `sdk_path` is under `/mnt/t500/go1_sdk`. It must contain the
archived Unitree SDK v3.8.6 rebuilt for Python 3.10; do not mix v3.5.1 files.
It must contain a matching
`robot_interface.cpython-<ABI>-aarch64-linux-gnu.so`.
