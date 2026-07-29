# Go1 ROS2 Project

Ubuntu 22.04 / ROS2 Humble deployment project for Unitree Go1 and Livox
MID-360. Start with `migration/README.md` and execute each Gate in order.

All writable project data lives under `/mnt/t500`: the repository,
`go1_ros2_ws`, Unitree SDK build, third-party sources, maps, and audit output.
The target Jetson already has ROS2 Humble Desktop; bootstrap preserves that
installation and adds only missing dependencies.

The original Ubuntu 20.04 / ROS1 snapshot is intentionally kept in a separate
private repository. It must not be copied wholesale into this ROS2 workspace.
See `ROS1_ARCHIVE.md` for the immutable archive commit and SDK v3.8.6 baseline.

## 3D PCD localization

The `nav2-workflow_3D` branch adds `omx_pcd_localization`, which keeps
FAST-LIO's `camera_init -> body` odometry and aligns the live body-frame cloud
against a saved PCD with NDT followed by GICP. The accepted 6DoF result is
published as `map -> camera_init`; Nav2 continues to use a 2D occupancy map for
planning, without starting AMCL.

The sensor-generated PCD is a runtime input and is not embedded in this code
branch. Follow the Korean
[`GO1_3D_LOCALIZATION_RUNBOOK.md`](docs/GO1_3D_LOCALIZATION_RUNBOOK.md) for the
terminal-by-terminal execution sequence. Package parameters and safety behavior
are documented in
[`packages/omx_pcd_localization/README.md`](packages/omx_pcd_localization/README.md).
