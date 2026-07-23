# ROS2 Humble Existing-Map Go1 Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 기존 `scans_new` 지도, FAST-LIO odometry, MID-360 LaserScan 및 ROS2 Nav2를 연결하고 Go1을 움직이지 않은 상태에서 RViz 경로계획까지 검증한다.

**Architecture:** 기존 범용 SLAM 구성은 유지하고 `omx_navigation`에 기존 지도 전용 파라미터와 통합 launch를 추가한다. FAST-LIO의 `camera_init -> body`를 임시 odom/base 프레임으로 사용하며, Nav2 출력은 Go1 드라이버의 `arm=false`와 `0.20 m/s`, `0.40 rad/s` 상한으로 이중 차단한다.

**Tech Stack:** ROS2 Humble, Nav2, AMCL, DWB, `pointcloud_to_laserscan`, FAST-LIO2, Livox ROS Driver 2, RViz2, Python launch, pytest, Bash

## Global Constraints

- 기본 `ROS_DOMAIN_ID`는 `100`이다.
- 지도는 저장소에 넣지 않고 Jetson의 `/mnt/t500/maps/scans_new.yaml`을 사용한다.
- 영구적인 `base_link -> lidar` TF와 최종 FAST-LIO extrinsic은 추가하지 않는다.
- 기본 프레임은 `map -> camera_init -> body`이다.
- 모든 자동 검증에서 Go1 드라이버는 `arm=false`이다.
- Nav2 속도 상한은 전진 `0.20 m/s`, 회전 `0.40 rad/s`를 넘지 않는다.
- controller와 LaserScan 기준 주기는 `10 Hz`이다.
- 실제 Go1 이동과 새 SLAM 지도 작성은 이 계획에 포함하지 않는다.

---

### Task 1: 기존 지도 전용 Nav2/DWB 파라미터

**Files:**
- Create: `packages/omx_navigation/config/nav2_existing_map_params.yaml`
- Create: `packages/omx_navigation/test/test_existing_map_params.py`
- Modify: `packages/omx_navigation/package.xml`

**Interfaces:**
- Consumes: `/scan`, `/Odometry`, `map -> camera_init -> body`
- Produces: AMCL, NavFn, DWB, costmap, velocity smoother가 공유하는 ROS2 파라미터 파일

- [ ] **Step 1: Write the failing parameter contract test**

```python
from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).parents[1] / "config" / "nav2_existing_map_params.yaml"
)


def params(node):
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[node]["ros__parameters"]


def test_fast_lio_frames_and_topics():
    amcl = params("amcl")
    assert amcl["global_frame_id"] == "map"
    assert amcl["odom_frame_id"] == "camera_init"
    assert amcl["base_frame_id"] == "body"
    assert amcl["scan_topic"] == "scan"


def test_ros1_dwa_tuning_intent_is_preserved():
    controller = params("controller_server")
    follow = controller["FollowPath"]
    goal = controller["general_goal_checker"]
    assert controller["controller_frequency"] == 10.0
    assert goal["xy_goal_tolerance"] == 0.20
    assert goal["yaw_goal_tolerance"] == 0.15
    assert follow["sim_time"] == 2.0
    assert follow["vx_samples"] == 10
    assert follow["vy_samples"] == 1
    assert follow["vtheta_samples"] == 20
    assert follow["PathAlign.scale"] == 40.0
    assert follow["PathDist.scale"] == 40.0
    assert follow["GoalAlign.scale"] == 20.0
    assert follow["GoalDist.scale"] == 20.0
    assert follow["BaseObstacle.scale"] == 0.01
    assert follow["Oscillation.oscillation_reset_dist"] == 0.20


def test_nav2_never_exceeds_go1_driver_limits():
    follow = params("controller_server")["FollowPath"]
    smoother = params("velocity_smoother")
    assert follow["max_vel_x"] <= 0.20
    assert follow["max_vel_theta"] <= 0.40
    assert smoother["max_velocity"] == [0.20, 0.0, 0.40]
    assert smoother["min_velocity"] == [0.0, 0.0, -0.40]
```

- [ ] **Step 2: Run the contract test and verify it fails**

Run:

```bash
python3 -m pytest packages/omx_navigation/test/test_existing_map_params.py -q
```

Expected: FAIL because `nav2_existing_map_params.yaml` does not exist.

- [ ] **Step 3: Add the dedicated Nav2 parameter file**

Copy the node structure from `config/nav2_params.yaml`, then apply these exact
existing-map values:

```yaml
amcl:
  ros__parameters:
    global_frame_id: map
    odom_frame_id: camera_init
    base_frame_id: body
    scan_topic: scan
    max_beams: 60
    min_particles: 300
    max_particles: 1200
    robot_model_type: nav2_amcl::DifferentialMotionModel

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.15
      movement_time_allowance: 20.0
    general_goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      stateful: true
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.15
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.20
      max_vel_y: 0.0
      max_vel_theta: 0.40
      min_speed_xy: 0.0
      max_speed_xy: 0.20
      min_speed_theta: 0.0
      acc_lim_x: 0.50
      acc_lim_y: 0.0
      acc_lim_theta: 1.00
      decel_lim_x: -0.50
      decel_lim_y: 0.0
      decel_lim_theta: -1.00
      vx_samples: 10
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 2.0
      linear_granularity: 0.05
      angular_granularity: 0.05
      trans_stopped_velocity: 0.03
      critics:
        [RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist]
      BaseObstacle.scale: 0.01
      PathAlign.scale: 40.0
      PathAlign.forward_point_distance: 0.10
      PathDist.scale: 40.0
      GoalAlign.scale: 20.0
      GoalAlign.forward_point_distance: 0.10
      GoalDist.scale: 20.0
      Oscillation.oscillation_reset_dist: 0.20
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 8.0
      RotateToGoal.lookahead_time: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: camera_init
      robot_base_frame: body
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 4
      height: 4
      robot_radius: 0.25
      inflation_layer:
        inflation_radius: 0.20
        cost_scaling_factor: 2.0

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: body
      update_frequency: 1.0
      publish_frequency: 0.5
      robot_radius: 0.25
      track_unknown_space: true
      inflation_layer:
        inflation_radius: 0.20
        cost_scaling_factor: 2.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.20
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    global_frame: camera_init
    robot_base_frame: body
    cycle_frequency: 5.0
    max_rotational_vel: 0.40
    min_rotational_vel: 0.10
    rotational_acc_lim: 1.00

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 10.0
    feedback: OPEN_LOOP
    max_velocity: [0.20, 0.0, 0.40]
    min_velocity: [0.0, 0.0, -0.40]
    max_accel: [0.50, 0.0, 1.00]
    max_decel: [-0.50, 0.0, -1.00]
    odom_topic: /Odometry
    velocity_timeout: 0.35
```

Keep the remaining Nav2 plugin declarations from the working Humble
`nav2_params.yaml`, changing every `odom`/`base_link` reference to
`camera_init`/`body`.

Add test dependencies:

```xml
<test_depend>python3-pytest</test_depend>
<test_depend>python3-yaml</test_depend>
```

- [ ] **Step 4: Run the parameter tests**

Run:

```bash
python3 -m pytest packages/omx_navigation/test/test_existing_map_params.py -q
```

Expected: `3 passed`.

- [ ] **Step 5: Commit the tested parameter profile**

```bash
git add packages/omx_navigation/config/nav2_existing_map_params.yaml \
  packages/omx_navigation/test/test_existing_map_params.py \
  packages/omx_navigation/package.xml
git commit -m "Add conservative existing-map Nav2 profile"
```

### Task 2: Scan projection and safe integrated launch

**Files:**
- Create: `packages/omx_navigation/config/mid360_scan.yaml`
- Create: `packages/omx_navigation/launch/go1_existing_map.launch.py`
- Create: `packages/omx_navigation/test/test_existing_map_launch.py`
- Modify: `packages/omx_navigation/package.xml`

**Interfaces:**
- Consumes: `/cloud_registered_body`, `/Odometry`, map YAML launch argument
- Produces: `/scan`, Nav2 stack, optional RViz, optional `go1_driver arm=false`

- [ ] **Step 1: Write the failing launch contract test**

```python
from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch" / "go1_existing_map.launch.py"
SCAN = ROOT / "config" / "mid360_scan.yaml"


def test_launch_defaults_are_safe():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("map"' in text
    assert 'default_value="/mnt/t500/maps/scans_new.yaml"' in text
    assert 'DeclareLaunchArgument("arm", default_value="false")' in text
    assert 'DeclareLaunchArgument("ros_domain_id", default_value="100")' in text
    assert 'default_value="/cloud_registered_body"' in text
    assert 'default_value="/Odometry"' in text
    assert 'default_value="camera_init"' in text
    assert 'default_value="body"' in text


def test_scan_projection_is_low_load_and_body_framed():
    text = SCAN.read_text(encoding="utf-8")
    for expected in (
        "target_frame: body",
        "min_height: -0.20",
        "max_height: 0.60",
        "angle_increment: 0.0174533",
        "scan_time: 0.10",
        "range_max: 10.0",
    ):
        assert expected in text
```

- [ ] **Step 2: Run it and verify the missing files fail**

Run:

```bash
python3 -m pytest packages/omx_navigation/test/test_existing_map_launch.py -q
```

Expected: FAIL because the launch and scan config do not exist.

- [ ] **Step 3: Add the low-load point cloud projection**

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: body
    transform_tolerance: 0.05
    min_height: -0.20
    max_height: 0.60
    angle_min: -3.1415927
    angle_max: 3.1415927
    angle_increment: 0.0174533
    scan_time: 0.10
    range_min: 0.20
    range_max: 10.0
    use_inf: true
    inf_epsilon: 1.0
```

- [ ] **Step 4: Add the integrated launch**

`go1_existing_map.launch.py` must:

```python
SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id"))
```

and launch:

```python
Node(
    package="pointcloud_to_laserscan",
    executable="pointcloud_to_laserscan_node",
    name="pointcloud_to_laserscan",
    remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
    parameters=[scan_params_file],
)

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(package_share, "launch", "rviz_navigation.launch.py")
    ),
    launch_arguments={
        "slam": "false",
        "map": map_yaml,
        "params_file": nav2_params_file,
        "scan_topic": scan_topic,
        "odom_topic": odom_topic,
        "odom_frame": odom_frame,
        "base_frame": base_frame,
        "rviz": rviz,
        "rviz_config": rviz_config,
    }.items(),
)

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(go1_share, "launch", "go1_driver.launch.py")
    ),
    condition=IfCondition(start_go1_driver),
    launch_arguments={"arm": arm, "cmd_vel_topic": "/cmd_vel_smoothed"}.items(),
)
```

Declare exact safe defaults:

```python
DeclareLaunchArgument("map", default_value="/mnt/t500/maps/scans_new.yaml")
DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body")
DeclareLaunchArgument("scan_topic", default_value="/scan")
DeclareLaunchArgument("odom_topic", default_value="/Odometry")
DeclareLaunchArgument("odom_frame", default_value="camera_init")
DeclareLaunchArgument("base_frame", default_value="body")
DeclareLaunchArgument("ros_domain_id", default_value="100")
DeclareLaunchArgument("rviz", default_value="true")
DeclareLaunchArgument("start_go1_driver", default_value="true")
DeclareLaunchArgument("arm", default_value="false")
```

Do not launch Livox or FAST-LIO inside this file; they remain separate sensor
processes so that sensor restart does not tear down localization and Nav2.

Add:

```xml
<exec_depend>go1_driver</exec_depend>
<exec_depend>pointcloud_to_laserscan</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
```

- [ ] **Step 5: Run tests and Python syntax checks**

Run:

```bash
python3 -m pytest packages/omx_navigation/test/test_existing_map_launch.py -q
python3 -m py_compile \
  packages/omx_navigation/launch/go1_existing_map.launch.py \
  packages/omx_navigation/launch/rviz_navigation.launch.py
```

Expected: `2 passed`; `py_compile` exits 0.

- [ ] **Step 6: Commit the integrated launch**

```bash
git add packages/omx_navigation/config/mid360_scan.yaml \
  packages/omx_navigation/launch/go1_existing_map.launch.py \
  packages/omx_navigation/test/test_existing_map_launch.py \
  packages/omx_navigation/package.xml
git commit -m "Add safe existing-map navigation launch"
```

### Task 3: Low-load RViz and Korean operator documentation

**Files:**
- Create: `packages/omx_navigation/rviz/go1_existing_map_low_load.rviz`
- Modify: `packages/omx_navigation/README.md`
- Create: `packages/omx_navigation/test/test_low_load_rviz.py`

**Interfaces:**
- Consumes: `/map`, `/scan`, `/plan`, `/local_plan`, costmaps, TF
- Produces: top-down RViz operation and exact dry-run commands

- [ ] **Step 1: Write the failing RViz contract test**

```python
from pathlib import Path


RVIZ = (
    Path(__file__).parents[1]
    / "rviz"
    / "go1_existing_map_low_load.rviz"
)


def test_rviz_is_top_down_and_does_not_render_raw_pointcloud():
    text = RVIZ.read_text(encoding="utf-8")
    assert "Class: rviz_default_plugins/TopDownOrtho" in text
    assert "Fixed Frame: map" in text
    assert "Frame Rate: 15" in text
    assert "Value: /scan" in text
    assert "Value: /goal_pose" in text
    assert "rviz_default_plugins/PointCloud2" not in text
```

- [ ] **Step 2: Verify the test fails**

Run:

```bash
python3 -m pytest packages/omx_navigation/test/test_low_load_rviz.py -q
```

Expected: FAIL because the RViz profile does not exist.

- [ ] **Step 3: Create the low-load RViz profile**

Start from `rviz/nav2.rviz`, then:

```yaml
Global Options:
  Fixed Frame: map
  Frame Rate: 15
```

Keep only Grid, Map, TF, LaserScan, Global Plan, Local Plan, Global Costmap and
Local Costmap. Disable RobotModel when `/robot_description` is absent. Use
LaserScan depth `1`, path buffer length `1`, costmap update depth `1`, and
TopDownOrtho. Do not add PointCloud2.

- [ ] **Step 4: Replace the corrupted README with Korean instructions**

Document these exact commands:

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
ros2 launch omx_navigation go1_existing_map.launch.py arm:=false
```

Also document that `2D Pose Estimate` is allowed, `2D Goal Pose` only computes
paths in this stage, and `arm:=true` is prohibited until a later hardware gate.

- [ ] **Step 5: Verify the RViz and README artifacts**

Run:

```bash
python3 -m pytest packages/omx_navigation/test/test_low_load_rviz.py -q
grep -F "arm:=false" packages/omx_navigation/README.md
grep -F "ROS_DOMAIN_ID=100" packages/omx_navigation/README.md
```

Expected: `1 passed`; both grep commands print matching lines.

- [ ] **Step 6: Commit RViz and documentation**

```bash
git add packages/omx_navigation/rviz/go1_existing_map_low_load.rviz \
  packages/omx_navigation/README.md \
  packages/omx_navigation/test/test_low_load_rviz.py
git commit -m "Add low-load RViz dry-run workflow"
```

### Task 4: Map preparation and arm-false runtime verifier

**Files:**
- Create: `migration/prepare_existing_map.sh`
- Create: `migration/verify_existing_map_navigation.sh`
- Create: `migration/test_existing_map_scripts.py`

**Interfaces:**
- Consumes: ROS2 CLI, source map path, running domain 100 stack
- Produces: `/mnt/t500/maps/scans_new.{yaml,pgm}` and a non-motion gate report

- [ ] **Step 1: Write the failing shell contract test**

```python
from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_map_script_requires_both_map_files():
    text = (ROOT / "migration" / "prepare_existing_map.sh").read_text()
    assert 'test -f "$source_yaml"' in text
    assert 'test -f "$source_pgm"' in text
    assert 'install -m 0644 "$source_yaml" "$target_dir/scans_new.yaml"' in text
    assert 'install -m 0644 "$source_pgm" "$target_dir/scans_new.pgm"' in text


def test_runtime_verifier_enforces_arm_false_and_core_topics():
    text = (ROOT / "migration" / "verify_existing_map_navigation.sh").read_text()
    assert "export ROS_DOMAIN_ID=100" in text
    assert "ros2 param get /go1_driver arm" in text
    for topic in ("/scan", "/Odometry", "/map", "/amcl_pose", "/cmd_vel"):
        assert topic in text
    assert "tf2_echo map camera_init" in text
    assert "tf2_echo camera_init body" in text
```

- [ ] **Step 2: Verify the scripts are missing**

Run:

```bash
python3 -m pytest migration/test_existing_map_scripts.py -q
```

Expected: FAIL because both scripts do not exist.

- [ ] **Step 3: Implement map preparation**

`prepare_existing_map.sh` must use:

```bash
#!/usr/bin/env bash
set -euo pipefail

source_yaml="${1:-/home/unicon/ros2_ws/src/navigation2/nav2_map_server/map/scans_new.yaml}"
source_pgm="${2:-${source_yaml%.yaml}.pgm}"
target_dir="${3:-/mnt/t500/maps}"

test -f "$source_yaml"
test -f "$source_pgm"
mkdir -p "$target_dir"
install -m 0644 "$source_yaml" "$target_dir/scans_new.yaml"
install -m 0644 "$source_pgm" "$target_dir/scans_new.pgm"
sed -i 's#^[[:space:]]*image:.*#image: scans_new.pgm#' "$target_dir/scans_new.yaml"
```

After writing, mark it executable.

- [ ] **Step 4: Implement the non-motion verifier**

`verify_existing_map_navigation.sh` must:

```bash
#!/usr/bin/env bash
set -euo pipefail
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

timeout 10 ros2 topic hz /scan
timeout 10 ros2 topic hz /Odometry
timeout 10 ros2 topic echo /map --once
timeout 10 ros2 topic echo /amcl_pose --once
timeout 10 ros2 run tf2_ros tf2_echo map camera_init
timeout 10 ros2 run tf2_ros tf2_echo camera_init body

arm_value="$(ros2 param get /go1_driver arm)"
test "$arm_value" = "Boolean value is: False"

timeout 10 ros2 topic echo /go1/control_state --once
timeout 10 ros2 topic echo /cmd_vel --once
```

Print a final `PASS: existing-map navigation is active with arm=false` only
after every gate succeeds.

- [ ] **Step 5: Run static script tests**

Run:

```bash
python3 -m pytest migration/test_existing_map_scripts.py -q
bash -n migration/prepare_existing_map.sh
bash -n migration/verify_existing_map_navigation.sh
```

Expected: `2 passed`; both `bash -n` commands exit 0.

- [ ] **Step 6: Commit deployment helpers**

```bash
git add migration/prepare_existing_map.sh \
  migration/verify_existing_map_navigation.sh \
  migration/test_existing_map_scripts.py
git commit -m "Add existing-map dry-run verification gates"
```

### Task 5: Jetson deployment and no-motion integration verification

**Files:**
- Modify: `/mnt/t500/go1_ros2_project` by fast-forwarding the implementation commits
- Modify: `/mnt/t500/go1_ros2_ws/src/omx_navigation` by staging the committed package
- Create locally at runtime: `/mnt/t500/maps/scans_new.yaml`
- Create locally at runtime: `/mnt/t500/maps/scans_new.pgm`

**Interfaces:**
- Consumes: all committed package and migration artifacts
- Produces: a running ROS2 domain 100 stack and recorded gate results

- [ ] **Step 1: Run the repository test suite before deployment**

Run:

```bash
python3 -m pytest packages/go1_driver/test packages/omx_navigation/test \
  migration/test_existing_map_scripts.py -q
git diff --check
```

Expected: all tests pass and `git diff --check` exits 0.

- [ ] **Step 2: Synchronize the committed files to the Jetson canonical checkout**

Use the Git branch if GitHub push access is available. If the remote still
rejects the branch, transfer only committed files and verify the Jetson tree
matches `git archive HEAD` before building. Do not copy `.git`, build, install,
log or map files.

- [ ] **Step 3: Prepare the existing map and stage the package**

Run on Jetson:

```bash
cd /mnt/t500/go1_ros2_project
./migration/prepare_existing_map.sh
./migration/stage_local_ros2_packages.sh /mnt/t500/go1_ros2_ws
```

Expected: `/mnt/t500/maps/scans_new.yaml` references `scans_new.pgm`, and the
workspace package matches the committed package.

- [ ] **Step 4: Build and validate launch descriptions**

Run on Jetson:

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
cd /mnt/t500/go1_ros2_ws
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
ros2 launch omx_navigation go1_existing_map.launch.py --show-args
```

Expected: both packages finish successfully and launch defaults show
`arm=false`, `camera_init`, `body`, and the existing map path.

- [ ] **Step 5: Verify the repaired Nav2 dependency**

Run:

```bash
test -f /opt/ros/humble/lib/libdiagnostic_updater.so
ldd /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager |
  grep -F "libdiagnostic_updater.so"
```

Expected: the library exists and `ldd` resolves it without `not found`.

- [ ] **Step 6: Start sensor, odometry, and navigation in domain 100**

Run in separate Jetson terminals:

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch omx_navigation go1_existing_map.launch.py arm:=false
```

- [ ] **Step 7: Run the arm-false gate verifier**

Run:

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_existing_map_navigation.sh
```

Expected: topic and TF gates pass; `/go1_driver arm` is false; final PASS line
is printed.

- [ ] **Step 8: Verify RViz localization and path planning without motion**

In RViz:

1. Confirm the existing map and `/scan` are visible in top-down view.
2. Use `2D Pose Estimate` once at the robot's approximate existing-map pose.
3. Confirm `/amcl_pose` changes and scan overlays the map.
4. Use `2D Goal Pose` for a nearby map goal.
5. Confirm global and local paths appear.
6. Confirm `/cmd_vel` stays within `0.20 m/s` and `0.40 rad/s`.
7. Cancel the goal and confirm the command returns to zero.
8. Confirm the physical Go1 did not move.

- [ ] **Step 9: Record results and commit final runbook updates**

Add the observed topic rates, lifecycle states, TF results, map suitability and
any remaining LiDAR mounting limitation to `packages/omx_navigation/README.md`.

```bash
git add packages/omx_navigation/README.md
git commit -m "Document verified ROS2 existing-map dry run"
```

- [ ] **Step 10: Push the branch**

```bash
git push -u origin codex/ros2-existing-map-navigation
```

Expected: the branch is available on
`Dannythechampion/GO1_to_ROS2_YEEPY`. PR creation is optional and requires
GitHub CLI or the GitHub connector after the push succeeds.
