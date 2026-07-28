# Go1 ROS2 Project

Ubuntu 22.04 / ROS2 Humble deployment project for Unitree Go1 and Livox
MID-360. Start with `migration/README.md` and execute each Gate in order.

For the complete new-map to autonomous-driving workflow, including the exact
Jetson terminal commands, see [`docs/GO1_NAV2_END_TO_END.md`](docs/GO1_NAV2_END_TO_END.md).

All writable project data lives under `/mnt/t500`: the repository,
`go1_ros2_ws`, Unitree SDK build, third-party sources, maps, and audit output.
The target Jetson already has ROS2 Humble Desktop; bootstrap preserves that
installation and adds only missing dependencies.

The original Ubuntu 20.04 / ROS1 snapshot is intentionally kept in a separate
private repository. It must not be copied wholesale into this ROS2 workspace.
See `ROS1_ARCHIVE.md` for the immutable archive commit and SDK v3.8.6 baseline.
