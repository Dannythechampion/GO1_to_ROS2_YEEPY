# 융합교육관 9층 하이브리드 매핑 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 노트북에서 개발·검증한 ROS2 Humble 매핑 패키지를 Jetson에 배포해 Livox MID-360 원본 rosbag, bounded-memory 분할 PCD, 폐루프 2D 지도 및 검증 보고서를 안전하게 생성한다.

**Architecture:** 새 `go1_mapping` ament_cmake 패키지가 FAST-LIO의 `/cloud_registered`를 외부 C++ writer로 분할 저장하고, `/cloud_registered_body`를 LaserScan으로 변환해 slam_toolbox에 제공한다. Python guard와 finalizer가 세션 경로·토픽 상태·디스크·지도 저장·산출물 검증을 담당하며, Jetson은 처리와 저장을 하고 노트북 WSL은 RViz만 실행한다.

**Tech Stack:** ROS2 Humble, C++17, rclcpp, PCL, Python 3.10, rclpy, pytest, GoogleTest, Livox MID-360, FAST-LIO2, pointcloud_to_laserscan, slam_toolbox, rosbag2, CycloneDDS

## Global Constraints

- 모든 개발 커밋은 `codex/hanyang-9f-mapping` 브랜치에만 만든다.
- Go1 드라이버, AMCL 및 Nav2를 매핑 launch에서 실행하지 않는다.
- FAST-LIO 내부 PCD 저장은 항상 `pcd_save_en: false`다.
- 실기 결과는 `/mnt/t500/maps/hanyang_9f/<session_id>` 아래에만 저장한다.
- PCD chunk는 300 frame 또는 256MiB 중 먼저 도달하는 조건으로 닫는다.
- `/mnt/t500` 여유 공간은 시작 전 100GiB 이상이어야 하고 수집 중 50GiB 미만이면 세션을 중단한다.
- rosbag2는 `/livox/lidar`, `/livox/imu`, `/Odometry`, `/tf`, `/tf_static`만 기록한다.
- rosbag2는 파일 단위 zstd 압축과 4GiB 분할을 사용한다.
- slam_toolbox 지도 해상도는 0.05m이고 `do_loop_closing: true`다.
- PCD 투영 지도는 기하 검증용이며 Nav2 최종 지도로 자동 승격하지 않는다.
- 현장 수집 전 라이다 높이를 측정하며 PCD 투영 단계에 `--sensor-height-m`로 명시한다.
- 로컬 x86_64 검증을 통과하기 전 Jetson 파일이나 workspace를 변경하지 않는다.

---

## 파일 구조

```text
packages/go1_mapping/
├─ CMakeLists.txt
├─ package.xml
├─ README.md
├─ config/
│  ├─ fast_lio_mapping_safe.yaml
│  ├─ mapping_session.yaml
│  ├─ pointcloud_to_scan_mapping.yaml
│  └─ slam_toolbox_hanyang_9f.yaml
├─ include/go1_mapping/
│  ├─ pcd_chunk_buffer.hpp
│  └─ pcd_projection.hpp
├─ launch/
│  ├─ laptop_rviz.launch.py
│  └─ mapping_session.launch.py
├─ go1_mapping/
│  ├─ __init__.py
│  ├─ manifest.py
│  ├─ map_finalizer.py
│  ├─ session_guard.py
│  └─ validation_report.py
├─ resource/go1_mapping
├─ rviz/hanyang_9f_mapping.rviz
├─ src/
│  ├─ pcd_chunk_buffer.cpp
│  ├─ pcd_chunk_writer.cpp
│  ├─ pcd_projection.cpp
│  └─ pcd_to_grid.cpp
└─ test/
   ├─ test_configs.py
   ├─ test_launch_contract.py
   ├─ test_manifest.py
   ├─ test_pcd_chunk_buffer.cpp
   ├─ test_pcd_projection.cpp
   ├─ test_session_guard.py
   └─ test_validation_report.py

migration/
├─ deploy_go1_mapping.sh
├─ stage_local_ros2_packages.sh
├─ test_go1_mapping_scripts.py
└─ verify_go1_mapping_preflight.sh
```

`pcd_chunk_buffer`는 ROS와 파일 형식을 분리한 bounded buffer다.
`pcd_chunk_writer`는 ROS 구독·parameter·flush service만 담당한다.
`pcd_projection`은 순수 PCL/그리드 로직이고 `pcd_to_grid`는 CLI다.
Python 모듈은 세션 제어와 보고서 생성만 담당한다.

---

### Task 1: `go1_mapping` 패키지 골격과 설치 계약

**Files:**
- Create: `packages/go1_mapping/CMakeLists.txt`
- Create: `packages/go1_mapping/package.xml`
- Create: `packages/go1_mapping/resource/go1_mapping`
- Create: `packages/go1_mapping/go1_mapping/__init__.py`
- Create: `packages/go1_mapping/test/test_configs.py`
- Modify: `migration/stage_local_ros2_packages.sh`
- Modify: `migration/test_existing_map_scripts.py`

**Interfaces:**
- Consumes: 기존 저장소의 `packages/` 및 `/mnt/t500/go1_ros2_ws/src` 배치 규칙
- Produces: `go1_mapping` ament_cmake 패키지와 Jetson staging 대상

- [ ] **Step 1: 패키지 및 staging 실패 테스트 작성**

`packages/go1_mapping/test/test_configs.py`:

```python
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]


def test_package_declares_mapping_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    names = {element.text for element in package}
    for dependency in {
        "ament_cmake",
        "ament_cmake_python",
        "rclcpp",
        "rclpy",
        "sensor_msgs",
        "std_srvs",
        "pcl_conversions",
        "pointcloud_to_laserscan",
        "slam_toolbox",
        "rosbag2",
    }:
        assert dependency in names


def test_cmake_installs_launch_config_rviz_and_python_package():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "ament_python_install_package(${PROJECT_NAME})" in cmake
    assert "install(DIRECTORY launch config rviz" in cmake
    assert "add_executable(pcd_chunk_writer" in cmake
    assert "add_executable(pcd_to_grid" in cmake
```

`migration/test_existing_map_scripts.py`에 추가:

```python
def test_stage_script_includes_go1_mapping():
    text = (ROOT / "migration" / "stage_local_ros2_packages.sh").read_text()
    assert 'mapping_source="$repo_root/packages/go1_mapping"' in text
    assert 'mapping_target="$src_dir/go1_mapping"' in text
    assert '"$mapping_source/package.xml"' in text
    assert 'cp -a "$mapping_source" "$mapping_target"' in text
```

- [ ] **Step 2: 테스트가 파일 부재로 실패하는지 확인**

Run:

```powershell
python -m pytest -q -p no:cacheprovider `
  packages/go1_mapping/test/test_configs.py `
  migration/test_existing_map_scripts.py
```

Expected: `package.xml` 또는 `go1_mapping` staging 문자열 부재로 FAIL.

- [ ] **Step 3: 패키지 manifest와 CMake 골격 구현**

`packages/go1_mapping/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>go1_mapping</name>
  <version>0.1.0</version>
  <description>Safe hybrid 3D PCD and 2D corridor mapping for Livox MID-360.</description>
  <maintainer email="maintainer@example.com">OMX-AI</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>diagnostic_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>pcl_ros</depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>livox_ros_driver2</exec_depend>
  <exec_depend>fast_lio</exec_depend>
  <exec_depend>pointcloud_to_laserscan</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>rosbag2</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`packages/go1_mapping/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(go1_mapping)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

ament_python_install_package(${PROJECT_NAME})

add_library(pcd_chunk_buffer src/pcd_chunk_buffer.cpp)
target_include_directories(pcd_chunk_buffer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_chunk_buffer ${PCL_LIBRARIES})

add_executable(pcd_chunk_writer src/pcd_chunk_writer.cpp)
target_link_libraries(pcd_chunk_writer pcd_chunk_buffer ${PCL_LIBRARIES})
ament_target_dependencies(
  pcd_chunk_writer rclcpp sensor_msgs std_srvs pcl_conversions)

add_library(pcd_projection src/pcd_projection.cpp)
target_include_directories(pcd_projection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_projection ${PCL_LIBRARIES})

add_executable(pcd_to_grid src/pcd_to_grid.cpp)
target_link_libraries(pcd_to_grid pcd_projection ${PCL_LIBRARIES})

install(TARGETS
  pcd_chunk_buffer pcd_chunk_writer pcd_projection pcd_to_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  go1_mapping/session_guard.py
  go1_mapping/map_finalizer.py
  go1_mapping/validation_report.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_configs test/test_configs.py)
endif()

ament_package()
```

빈 파일을 생성한다.

```text
packages/go1_mapping/resource/go1_mapping
packages/go1_mapping/go1_mapping/__init__.py
```

- [ ] **Step 4: staging 스크립트에 패키지 추가**

`migration/stage_local_ros2_packages.sh`의 source/target 선언에 추가:

```bash
mapping_source="$repo_root/packages/go1_mapping"
mapping_target="$src_dir/go1_mapping"
```

필수 파일 검사에 추가:

```bash
"$mapping_source/package.xml"
"$mapping_source/CMakeLists.txt"
```

target overwrite 검사 배열을 다음으로 변경:

```bash
for target in "$go1_target" "$omx_target" "$mapping_target"; do
```

기존 패키지 복사 뒤에 추가:

```bash
cp -a "$mapping_source" "$mapping_target"
```

출력과 build 명령의 package 목록에 `go1_mapping`을 추가한다.

- [ ] **Step 5: 정적 테스트 통과 확인**

Run:

```powershell
python -m pytest -q -p no:cacheprovider `
  packages/go1_mapping/test/test_configs.py `
  migration/test_existing_map_scripts.py
```

Expected: PASS.

- [ ] **Step 6: 커밋**

```bash
git add packages/go1_mapping migration/stage_local_ros2_packages.sh \
  migration/test_existing_map_scripts.py
git commit -m "Add hybrid mapping package skeleton"
```

---

### Task 2: 안전 매핑 설정 파일

**Files:**
- Create: `packages/go1_mapping/config/fast_lio_mapping_safe.yaml`
- Create: `packages/go1_mapping/config/pointcloud_to_scan_mapping.yaml`
- Create: `packages/go1_mapping/config/slam_toolbox_hanyang_9f.yaml`
- Create: `packages/go1_mapping/config/mapping_session.yaml`
- Extend: `packages/go1_mapping/test/test_configs.py`

**Interfaces:**
- Consumes: `/livox/lidar`, `/livox/imu`, FAST-LIO `camera_init -> body`
- Produces: `/cloud_registered`, `/cloud_registered_body`, `/scan`, `/map_slam`

- [ ] **Step 1: 설정 계약 실패 테스트 작성**

`test_configs.py`에 추가:

```python
import yaml


def load(name):
    return yaml.safe_load((ROOT / "config" / name).read_text())


def test_fast_lio_mapping_disables_internal_pcd_and_heavy_map():
    params = load("fast_lio_mapping_safe.yaml")["/**"]["ros__parameters"]
    assert params["pcd_save"]["pcd_save_en"] is False
    assert params["pcd_save"]["interval"] == 300
    assert params["publish"]["map_en"] is False
    assert params["publish"]["effect_map_en"] is False
    assert params["publish"]["dense_publish_en"] is False
    assert params["publish"]["scan_publish_en"] is True
    assert params["publish"]["scan_bodyframe_pub_en"] is True


def test_scan_projection_is_low_latency_body_frame():
    params = load("pointcloud_to_scan_mapping.yaml")[
        "pointcloud_to_laserscan"
    ]["ros__parameters"]
    assert params["target_frame"] == "body"
    assert params["queue_size"] == 1
    assert params["scan_time"] == 0.1
    assert params["range_min"] == 0.5
    assert params["range_max"] == 20.0


def test_slam_owns_only_map_slam_to_camera_init():
    params = load("slam_toolbox_hanyang_9f.yaml")[
        "slam_toolbox"
    ]["ros__parameters"]
    assert params["map_frame"] == "map_slam"
    assert params["odom_frame"] == "camera_init"
    assert params["base_frame"] == "body"
    assert params["scan_topic"] == "/scan"
    assert params["mode"] == "mapping"
    assert params["do_loop_closing"] is True
    assert params["resolution"] == 0.05
    assert params["scan_queue_size"] == 1


def test_session_limits_match_design():
    params = load("mapping_session.yaml")["mapping_session"]
    assert params["frames_per_chunk"] == 300
    assert params["max_buffer_bytes"] == 268435456
    assert params["preflight_free_gib"] == 100
    assert params["abort_free_gib"] == 50
    assert params["bag_split_bytes"] == 4294967296
```

- [ ] **Step 2: 설정 파일 부재로 실패 확인**

Run:

```powershell
python -m pytest -q -p no:cacheprovider `
  packages/go1_mapping/test/test_configs.py
```

Expected: 첫 번째 config 파일에서 `FileNotFoundError`.

- [ ] **Step 3: FAST-LIO 안전 설정 작성**

`fast_lio_mapping_safe.yaml`은 기존
`packages/omx_navigation/config/fast_lio_mid360_navigation.yaml`을
기준으로 작성하되 다음 값을 정확히 사용한다.

```yaml
/**:
  ros__parameters:
    feature_extract_enable: false
    point_filter_num: 3
    max_iteration: 3
    filter_size_surf: 0.5
    filter_size_map: 0.5
    cube_side_length: 1000.0
    runtime_pos_log_enable: false
    map_file_path: "./unused.pcd"
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      time_sync_en: false
      time_offset_lidar_to_imu: 0.0
    preprocess:
      lidar_type: 1
      scan_line: 4
      blind: 0.5
      timestamp_unit: 3
      scan_rate: 10
    mapping:
      acc_cov: 0.1
      gyr_cov: 0.1
      b_acc_cov: 0.0001
      b_gyr_cov: 0.0001
      fov_degree: 360.0
      det_range: 100.0
      extrinsic_est_en: true
      extrinsic_T: [-0.011, -0.02329, 0.04412]
      extrinsic_R: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    publish:
      path_en: false
      effect_map_en: false
      map_en: false
      scan_publish_en: true
      dense_publish_en: false
      scan_bodyframe_pub_en: true
    pcd_save:
      pcd_save_en: false
      interval: 300
```

- [ ] **Step 4: LaserScan과 slam_toolbox 설정 작성**

`pointcloud_to_scan_mapping.yaml`:

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: body
    transform_tolerance: 0.10
    min_height: -0.30
    max_height: 1.30
    angle_min: -3.141592653589793
    angle_max: 3.141592653589793
    angle_increment: 0.008726646259972
    scan_time: 0.1
    range_min: 0.5
    range_max: 20.0
    use_inf: true
    queue_size: 1
```

`slam_toolbox_hanyang_9f.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: HuberLoss
    odom_frame: camera_init
    map_frame: map_slam
    base_frame: body
    scan_topic: /scan
    mode: mapping
    transform_publish_period: 0.05
    map_update_interval: 2.0
    resolution: 0.05
    max_laser_range: 20.0
    min_laser_range: 0.5
    minimum_time_interval: 0.1
    transform_timeout: 0.5
    tf_buffer_duration: 30.0
    scan_queue_size: 1
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.10
    minimum_travel_heading: 0.08726646259971647
    do_loop_closing: true
    loop_search_maximum_distance: 5.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    enable_interactive_mode: false
    use_map_saver: true
    throttle_scans: 1
```

- [ ] **Step 5: 세션 한계 설정 작성**

`mapping_session.yaml`:

```yaml
mapping_session:
  session_root: /mnt/t500/maps/hanyang_9f
  frames_per_chunk: 300
  max_buffer_bytes: 268435456
  preflight_free_gib: 100
  abort_free_gib: 50
  bag_split_bytes: 4294967296
  lidar_min_hz: 8.0
  imu_min_hz: 100.0
  odom_min_hz: 8.0
  max_input_gap_sec: 1.0
  initialization_sec: 15.0
  return_position_tolerance_m: 0.5
  return_yaw_tolerance_deg: 10.0
```

- [ ] **Step 6: 테스트와 YAML parse 확인**

Run:

```powershell
python -m pytest -q -p no:cacheprovider `
  packages/go1_mapping/test/test_configs.py
```

Expected: PASS.

- [ ] **Step 7: 커밋**

```bash
git add packages/go1_mapping/config packages/go1_mapping/test/test_configs.py
git commit -m "Add safe corridor mapping profiles"
```

---

### Task 3: 세션 경로와 manifest 순수 Python 모듈

**Files:**
- Create: `packages/go1_mapping/go1_mapping/manifest.py`
- Create: `packages/go1_mapping/test/test_manifest.py`

**Interfaces:**
- Consumes: `session_root: Path`, 선택적인 `session_id: str`
- Produces: `SessionPaths`, `create_session()`, `write_manifest_atomic()`

- [ ] **Step 1: 실패 테스트 작성**

`test_manifest.py`:

```python
from pathlib import Path
import re
import yaml

from go1_mapping.manifest import (
    create_session,
    write_manifest_atomic,
)


def test_create_session_builds_complete_tree(tmp_path):
    paths = create_session(tmp_path, "20260724_090000")
    assert paths.root == tmp_path / "20260724_090000"
    assert paths.bag.is_dir()
    assert paths.pcd.is_dir()
    assert paths.slam.is_dir()
    assert paths.pcd2d.is_dir()
    assert paths.validation.is_dir()


def test_create_session_generates_timestamp_id(tmp_path):
    paths = create_session(tmp_path, "")
    assert re.fullmatch(r"\d{8}_\d{6}", paths.root.name)


def test_create_session_refuses_existing_or_unsafe_id(tmp_path):
    create_session(tmp_path, "20260724_090000")
    for value in ("20260724_090000", "../escape", "contains space"):
        try:
            create_session(tmp_path, value)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted unsafe session id: {value}")


def test_manifest_write_is_parseable_and_has_no_partial(tmp_path):
    target = tmp_path / "manifest.yaml"
    write_manifest_atomic(target, {"session_id": "20260724_090000"})
    assert yaml.safe_load(target.read_text())["session_id"] == "20260724_090000"
    assert not (tmp_path / "manifest.yaml.partial").exists()
```

- [ ] **Step 2: 모듈 부재 실패 확인**

Run:

```powershell
$env:PYTHONPATH = "$PWD\packages\go1_mapping"
python -m pytest -q -p no:cacheprovider `
  packages/go1_mapping/test/test_manifest.py
```

Expected: `ModuleNotFoundError: go1_mapping.manifest`.

- [ ] **Step 3: 최소 구현**

`manifest.py`:

```python
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import os
import re

import yaml


SESSION_ID = re.compile(r"^\d{8}_\d{6}$")


@dataclass(frozen=True)
class SessionPaths:
    root: Path
    bag: Path
    pcd: Path
    slam: Path
    pcd2d: Path
    validation: Path


def create_session(session_root: Path, session_id: str) -> SessionPaths:
    resolved_root = Path(session_root).resolve()
    value = session_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    if not SESSION_ID.fullmatch(value):
        raise ValueError(f"invalid session id: {value}")
    root = resolved_root / value
    try:
        root.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        raise ValueError(f"session already exists: {root}") from exc
    paths = SessionPaths(
        root=root,
        bag=root / "bag",
        pcd=root / "pcd",
        slam=root / "slam_toolbox",
        pcd2d=root / "pcd2d",
        validation=root / "validation",
    )
    for directory in paths.__dict__.values():
        if directory != root:
            directory.mkdir()
    return paths


def write_manifest_atomic(target: Path, data: dict) -> None:
    target = Path(target)
    partial = target.with_name(target.name + ".partial")
    partial.write_text(
        yaml.safe_dump(data, sort_keys=True, allow_unicode=True),
        encoding="utf-8",
    )
    os.replace(partial, target)
```

- [ ] **Step 4: 테스트 통과**

Run:

```powershell
$env:PYTHONPATH = "$PWD\packages\go1_mapping"
python -m pytest -q -p no:cacheprovider `
  packages/go1_mapping/test/test_manifest.py
```

Expected: `4 passed`.

- [ ] **Step 5: 커밋**

```bash
git add packages/go1_mapping/go1_mapping/manifest.py \
  packages/go1_mapping/test/test_manifest.py
git commit -m "Add mapping session manifest contract"
```

---

### Task 4: bounded-memory PCD chunk writer

**Files:**
- Create: `packages/go1_mapping/include/go1_mapping/pcd_chunk_buffer.hpp`
- Create: `packages/go1_mapping/src/pcd_chunk_buffer.cpp`
- Create: `packages/go1_mapping/src/pcd_chunk_writer.cpp`
- Create: `packages/go1_mapping/test/test_pcd_chunk_buffer.cpp`
- Modify: `packages/go1_mapping/CMakeLists.txt`

**Interfaces:**
- `ChunkBuffer(std::size_t max_frames, std::size_t max_bytes)`
- `bool should_flush_before(std::size_t incoming_points) const`
- `void append(const pcl::PointCloud<pcl::PointXYZI>& cloud)`
- `pcl::PointCloud<pcl::PointXYZI>::Ptr take()`
- ROS input `/cloud_registered`, service `/pcd_chunk_writer/flush`
- Parameters: `output_dir`, `allowed_root`, `frames_per_chunk=300`, `max_buffer_bytes=268435456`

- [ ] **Step 1: 경계 조건 실패 테스트 작성**

`test_pcd_chunk_buffer.cpp`에서 다음 세 경우를 고정한다.

```cpp
TEST(ChunkBuffer, FlushesBeforeFrameLimitIsExceeded) {
  ChunkBuffer buffer(2, 1024 * 1024);
  pcl::PointCloud<pcl::PointXYZI> cloud; cloud.resize(1);
  buffer.append(cloud); buffer.append(cloud);
  EXPECT_TRUE(buffer.should_flush_before(cloud.size()));
}
TEST(ChunkBuffer, FlushesBeforeByteLimitIsExceeded) {
  ChunkBuffer buffer(100, sizeof(pcl::PointXYZI) * 2);
  pcl::PointCloud<pcl::PointXYZI> cloud; cloud.resize(2);
  buffer.append(cloud);
  EXPECT_TRUE(buffer.should_flush_before(1));
}
TEST(ChunkBuffer, TakeClearsAccounting) {
  ChunkBuffer buffer(300, 268435456);
  pcl::PointCloud<pcl::PointXYZI> cloud; cloud.resize(5);
  buffer.append(cloud); auto taken = buffer.take();
  EXPECT_EQ(taken->size(), 5U);
  EXPECT_EQ(buffer.frame_count(), 0U);
  EXPECT_EQ(buffer.byte_count(), 0U);
}
```

- [ ] **Step 2: GTest target 등록 후 compile failure 확인**

```cmake
ament_add_gtest(test_pcd_chunk_buffer test/test_pcd_chunk_buffer.cpp)
target_link_libraries(test_pcd_chunk_buffer pcd_chunk_buffer ${PCL_LIBRARIES})
```

```bash
colcon test --packages-select go1_mapping --ctest-args -R test_pcd_chunk_buffer --output-on-failure
```

Expected: header 부재로 FAIL.

- [ ] **Step 3: 순수 buffer 구현**

buffer가 비어 있으면 `should_flush_before()`는 false다. 그렇지 않으면
`frame_count + 1 > max_frames` 또는
`byte_count + incoming_points * sizeof(PointXYZI) > max_bytes`일 때 true다.
단일 incoming frame 자체가 256MiB를 넘으면 append하지 않고 명시적 error를 반환해
hard memory bound를 지킨다. `append()`는 point/frame/byte를 누적하고 `take()`는
현재 cloud를 반환한 뒤 모든 accounting을 0으로 초기화한다.

- [ ] **Step 4: ROS writer와 원자적 저장 구현**

`output_dir`가 canonical `allowed_root` 아래인지 path component 단위로 확인한다.
SensorDataQoS depth 1로 cloud를 구독하고 flush 조건이면 기존 buffer부터 저장한다.
파일은 `chunk_000000.pcd.partial`에 binary compressed PCD로 쓴 뒤 성공할 때만
`chunk_000000.pcd`로 rename한다. 빈 flush는 파일 없이 성공한다. 저장 실패는
fatal/non-zero 종료로 전파한다. signal handler에서는 PCL I/O를 하지 않고 정상
종료 전에 finalizer가 flush service를 호출한다.

- [ ] **Step 5: build/test 통과**

```bash
colcon build --symlink-install --packages-select go1_mapping
colcon test --packages-select go1_mapping --ctest-args -R test_pcd_chunk_buffer --output-on-failure
colcon test-result --verbose
```

Expected: build 성공, 3 tests PASS.

- [ ] **Step 6: 커밋**

```bash
git add packages/go1_mapping/include packages/go1_mapping/src packages/go1_mapping/test/test_pcd_chunk_buffer.cpp packages/go1_mapping/CMakeLists.txt
git commit -m "Add bounded PCD chunk writer"
```

---

### Task 5: 입력 rate, gap, disk session guard

**Files:**
- Create: `packages/go1_mapping/go1_mapping/session_guard.py`
- Create: `packages/go1_mapping/test/test_session_guard.py`
- Modify: `packages/go1_mapping/package.xml`
- Modify: `packages/go1_mapping/CMakeLists.txt`

**Interfaces:**
- `HealthWindow(required_hz, max_gap_sec, initialization_sec, started_at)`
- `observe(topic: str, stamp_sec: float) -> None`
- `evaluate(now_sec: float) -> list[str]`
- Parameters: `session_dir`, `abort_free_gib=50`, topic별 최소 Hz, gap 1초, 초기화 15초
- 정상 exit 0, 안전 조건 위반 exit 2

- [ ] **Step 1: 실패 테스트 작성**

초기화 10초에는 error가 없어야 하고, 15초 이후 2Hz lidar는 rate error, 마지막
수신 1초 초과는 gap error, 역행 timestamp는 regression error가 나야 한다.
`should_abort_disk(49.99, 50)`는 true, `(50, 50)`은 false로 고정한다.

- [ ] **Step 2: module 부재 실패 확인**

```powershell
$env:PYTHONPATH = "$PWD\packages\go1_mapping"
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_session_guard.py
```

- [ ] **Step 3: pure monitoring core 구현**

각 topic 최근 10초 stamp를 deque에 보관하고 `(count-1)/(last-first)`로 rate를
계산한다. 초기화 이후 미수신, 최소 rate 미달, 마지막 수신 1초 초과, timestamp
역행을 각각 error로 반환한다. `free_gib()`는 `shutil.disk_usage().free / 2**30`,
abort 비교는 strict `<`다.

- [ ] **Step 4: rclpy wrapper 구현**

`/livox/lidar` CustomMsg, `/livox/imu` Imu, `/Odometry` Odometry를 SensorDataQoS
으로 구독한다. 1Hz timer마다 rate, 최대 gap, 최초 안정 odometry pose, 최신 pose,
두 pose의 XY/yaw 차이를 `validation/health.yaml`에 atomic write한다. 위반 시 같은
측정값과 원인을 `validation/guard_failure.yaml`에 atomic write하고 exit 2로 끝낸다.
`nav_msgs`, `livox_ros_driver2`, `python3-yaml` dependency를 추가한다.

- [ ] **Step 5: 테스트와 커밋**

```powershell
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_session_guard.py
```

Expected: 4 passed.

```bash
git add packages/go1_mapping/go1_mapping/session_guard.py packages/go1_mapping/test/test_session_guard.py packages/go1_mapping/package.xml packages/go1_mapping/CMakeLists.txt
git commit -m "Add mapping session health guard"
```

---

### Task 6: mapping session 통합 launch와 rosbag 계약

**Files:**
- Create: `packages/go1_mapping/launch/mapping_session.launch.py`
- Create: `packages/go1_mapping/test/test_launch_contract.py`
- Modify: `packages/go1_mapping/go1_mapping/manifest.py`
- Modify: `packages/go1_mapping/CMakeLists.txt`

**Interfaces:**
- Arguments: `session_root`, `session_id`, `ros_domain_id`, `start_livox`, `start_fast_lio`
- Livox `msg_MID360_launch.py`, FAST-LIO `mapping.launch.py`, `rviz:=false`
- writer/guard/rosbag non-zero exit이면 전체 launch shutdown

- [ ] **Step 1: 정적 계약 실패 테스트 작성**

테스트는 Go1/Nav2 문자열 부재, Livox/FAST-LIO include, `TimerAction(period=3.0)`과
`TimerAction(period=8.0)`, cloud/scan/map remap, rosbag topic allowlist, 4GiB/zstd,
`OnProcessExit`/`Shutdown` 존재를 검사한다.

- [ ] **Step 2: launch 부재 실패 확인**

```powershell
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_launch_contract.py
```

- [ ] **Step 3: OpaqueFunction session 생성**

`create_session()` 후 manifest에 session id/status running/domain 100, frames
`camera_init/body/map_slam`, 다섯 input/output topic, UTC 시작 시각을 atomic write한다.
기존 session id는 덮어쓰지 않는다.

- [ ] **Step 4: 단계적 bring-up 구현**

즉시 Livox와 rosbag을 시작해 센서 초기화 구간도 원본 bag에 남긴다. 3초 후
FAST-LIO, 8초 후 writer, pointcloud_to_laserscan, slam_toolbox, guard를 시작한다.
remap은 `cloud_in -> /cloud_registered_body`, `scan -> /scan`, slam map
`/map_slam`이다.

```python
bag_command = [
    "ros2", "bag", "record", "--output", str(paths.bag / "raw"),
    "--max-bag-size", "4294967296", "--compression-mode", "file",
    "--compression-format", "zstd", "/livox/lidar", "/livox/imu",
    "/Odometry", "/tf", "/tf_static",
]
```

writer, guard, rosbag 각각 non-zero exit에 `Shutdown(reason=...)`를 emit한다.

- [ ] **Step 5: 정적/syntax/build 검증**

```powershell
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_launch_contract.py
python -m py_compile packages/go1_mapping/launch/mapping_session.launch.py
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go1_mapping mapping_session.launch.py --show-args
```

Expected: 4 tests PASS, 다섯 argument 표시.

- [ ] **Step 6: 커밋**

```bash
git add packages/go1_mapping/launch/mapping_session.launch.py packages/go1_mapping/test/test_launch_contract.py packages/go1_mapping/go1_mapping/manifest.py packages/go1_mapping/CMakeLists.txt
git commit -m "Add guarded mapping session launch"
```

---

### Task 7: PCD merge와 2D geometry reference

**Files:**
- Create: `packages/go1_mapping/include/go1_mapping/pcd_projection.hpp`
- Create: `packages/go1_mapping/src/pcd_projection.cpp`
- Create: `packages/go1_mapping/src/pcd_to_grid.cpp`
- Create: `packages/go1_mapping/test/test_pcd_projection.cpp`
- Modify: `packages/go1_mapping/CMakeLists.txt`

**Interfaces:**
- `ProjectionBounds z_bounds(double sensor_height_m)`
- `Grid project_occupied(cloud, double resolution, ProjectionBounds)`
- CLI `pcd_to_grid --input-dir DIR --output-pcd FILE --output-map PREFIX --sensor-height-m METERS --resolution 0.05 --voxel-size 0.10`

- [ ] **Step 1: projection 실패 테스트 작성**

sensor height 1.05m일 때 z bounds `[-0.90, 0.75]`, obstacle cell 0, unknown 205,
free 254는 0개임을 GTest로 고정한다.

- [ ] **Step 2: compile failure 확인**

```bash
colcon test --packages-select go1_mapping --ctest-args -R test_pcd_projection --output-on-failure
```

- [ ] **Step 3: projection 구현**

`z_bounds(h)={-h+0.15,-h+1.80}`. 유효 XY bounds에 사방 1m padding, 전체 cell
205 초기화, z 범위 장애물 cell만 0으로 만든다. ray tracing/free inference는 하지
않는다. YAML에는 resolution 0.05와
`go1_mapping_role: geometry_reference_only`를 반드시 기록한다.

- [ ] **Step 4: bounded offline merge CLI 구현**

`chunk_*.pcd`를 순서대로 하나씩 읽고 chunk별 0.10m voxel filter 후 append하며,
최종 cloud에도 0.10m filter를 적용한다. merged PCD는 binary compressed다. chunk 0개,
sensor height `<=0`, output이 입력 `chunk_*.pcd` 중 하나와 같은 경로이면 non-zero로
거부한다. `merged.pcd.partial`에 쓴 뒤 원자적으로 rename해 중단 시 원본 chunk를 보존한다.

- [ ] **Step 5: build/test와 커밋**

```bash
colcon build --symlink-install --packages-select go1_mapping
colcon test --packages-select go1_mapping --ctest-args -R test_pcd_projection --output-on-failure
colcon test-result --verbose
git add packages/go1_mapping/include/go1_mapping/pcd_projection.hpp packages/go1_mapping/src/pcd_projection.cpp packages/go1_mapping/src/pcd_to_grid.cpp packages/go1_mapping/test/test_pcd_projection.cpp packages/go1_mapping/CMakeLists.txt
git commit -m "Add PCD geometry projection pipeline"
```

---

### Task 8: 안전한 종료, map 저장, validation report

**Files:**
- Create: `packages/go1_mapping/go1_mapping/map_finalizer.py`
- Create: `packages/go1_mapping/go1_mapping/validation_report.py`
- Create: `packages/go1_mapping/test/test_validation_report.py`
- Modify: `packages/go1_mapping/go1_mapping/manifest.py`
- Modify: `packages/go1_mapping/CMakeLists.txt`

**Interfaces:**
- `ros2 run go1_mapping map_finalizer.py --session-dir DIR --sensor-height-m METERS`
- `build_report(session_dir: Path) -> dict`
- Services: `/pcd_chunk_writer/flush`, `/slam_toolbox/save_map`,
  `/slam_toolbox/serialize_map`

- [ ] **Step 1: 결과 계약 실패 테스트 작성**

minimal session fixture에 non-empty chunk, 1x1 PGM, relative image와 resolution 0.05
YAML을 만든다. `build_report()`가 slam map OK, chunk count 1, artifact SHA256 64자를
반환하는지 검사한다. resolution 0.10은 실패해야 하고 `sha256_file(b"abc")`는
`ba7816bf...15ad`로 고정한다.

- [ ] **Step 2: module 부재 실패 확인**

```powershell
$env:PYTHONPATH = "$PWD\packages\go1_mapping"
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_validation_report.py
```

- [ ] **Step 3: finalizer 구현**

launch가 살아 있는 동안 다음 순서로만 실행한다.

1. session이 allowed root 아래이고 manifest `status=running`인지 확인
2. PCD writer flush, timeout 10초
3. slam_toolbox save_map으로 `slam_toolbox/hanyang_9f.{pgm,yaml}` 저장
4. serialize_map으로 `slam_toolbox/hanyang_9f.posegraph` 저장
5. `pcd_to_grid`로 `pcd/merged.pcd`와 `pcd2d/geometry_reference.{pgm,yaml}` 생성
6. `validation/report.yaml` atomic write
7. 필수 check 통과 시 manifest status complete

실패 시 session을 삭제하지 않고 manifest에 status failed, failed_step, error를 atomic
write한 뒤 non-zero 종료한다.

- [ ] **Step 4: report 구현**

상대 image 경로가 YAML directory 밖으로 탈출하지 않는지, resolution 0.05, PGM
P2/P5와 양수 크기, non-empty chunks/merged/reference files, geometry-only role, guard
failure 부재를 검사한다. `validation/health.yaml`에서 rates 8/100/8Hz, gap 1초 이하,
최초 안정 pose 대비 최신 pose의 귀환 위치 0.5m/yaw 10도 이하를 검증한다. health나
귀환 값이 없으면 `needs_return_check`이며 complete가 아니다. 모든 artifact SHA256을
기록한다.

- [ ] **Step 5: 테스트와 커밋**

```powershell
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_validation_report.py
```

Expected: 3 passed.

```bash
git add packages/go1_mapping/go1_mapping/map_finalizer.py packages/go1_mapping/go1_mapping/validation_report.py packages/go1_mapping/go1_mapping/manifest.py packages/go1_mapping/test/test_validation_report.py packages/go1_mapping/CMakeLists.txt
git commit -m "Add mapping finalization and validation"
```

---

### Task 9: 저부하 노트북 RViz

**Files:**
- Create: `packages/go1_mapping/rviz/hanyang_9f_mapping.rviz`
- Create: `packages/go1_mapping/launch/laptop_rviz.launch.py`
- Extend: `packages/go1_mapping/test/test_launch_contract.py`

**Interfaces:**
- `ros2 launch go1_mapping laptop_rviz.launch.py`
- Fixed frame `map_slam`; displays `/map_slam`, `/scan`, `/Odometry`, TF
- PointCloud2 display는 기본 생성하지 않음

- [ ] **Step 1: 저부하 실패 테스트 작성**

RViz text에 fixed frame과 세 topic이 있고 PointCloud2가 없는지 검사한다. laptop
launch에는 rviz2만 있고 Go1/slam_toolbox가 없는지 검사한다.

- [ ] **Step 2: 파일 부재 실패 확인**

```powershell
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_launch_contract.py
```

- [ ] **Step 3: profile과 launch 구현**

Map alpha 0.7, LaserScan Points/size 0.03/decay 0.2, TF names false, update 10Hz로
제한한다. launch는 domain 100과 `rmw_cyclonedds_cpp`를 기본 설정하되 사용자의
`CYCLONEDDS_URI`는 덮어쓰지 않는다.

- [ ] **Step 4: 정적 검증과 WSL smoke test**

```powershell
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test/test_launch_contract.py
python -m py_compile packages/go1_mapping/launch/laptop_rviz.launch.py
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 15s ros2 launch go1_mapping laptop_rviz.launch.py
```

Expected: plugin load/crash 없음. 센서가 없을 때 fixed-frame warning만 허용.

- [ ] **Step 5: 커밋**

```bash
git add packages/go1_mapping/rviz/hanyang_9f_mapping.rviz packages/go1_mapping/launch/laptop_rviz.launch.py packages/go1_mapping/test/test_launch_contract.py
git commit -m "Add low-load laptop mapping RViz"
```

---

### Task 10: Jetson staging, deploy, preflight

**Files:**
- Create: `migration/deploy_go1_mapping.sh`
- Create: `migration/verify_go1_mapping_preflight.sh`
- Create: `migration/test_go1_mapping_scripts.py`
- Modify: `migration/stage_local_ros2_packages.sh`
- Create: `packages/go1_mapping/README.md`

**Interfaces:**
- Target `unicon@192.168.0.138`
- Branch `codex/hanyang-9f-mapping`
- Workspace `/mnt/t500/go1_ros2_ws`
- Session root `/mnt/t500/maps/hanyang_9f`
- Sensor `192.168.1.138`

- [ ] **Step 1: shell 안전 계약 실패 테스트 작성**

테스트는 preflight가 ROS를 source한 뒤 `set -u`를 적용하는지, `/mnt/t500`, 100/50
GiB, sensor IP, lidar/imu/odom, camera_init/body, Go1/Nav2 금지를 포함하는지 검사한다.
deploy는 dirty remote와 wrong branch를 거부하고 `--packages-up-to go1_mapping`을
build하며 `rm -rf`가 없어야 한다.

- [ ] **Step 2: shell 파일 부재 실패 확인**

```powershell
python -m pytest -q -p no:cacheprovider migration/test_go1_mapping_scripts.py
```

- [ ] **Step 3: preflight 구현**

순서는 ROS Humble source, workspace source, 그 뒤 `set -euo pipefail`이다. 기본
`--static` 모드는 user/host, domain 100, `/mnt/t500` free 100GiB, sensor ping,
Go1/Nav2 process/node 부재, sensor height 0.20..2.50m를 확인한다. optional `--live`
모드는 이미 실행 중인 Livox/FAST-LIO를 대상으로 rates 8/100/8Hz, gap 1초,
`tf2_echo camera_init body` 5초까지 확인한다. 한 launch가 드라이버를 소유하는 권장
운영에서는 시작 전 `--static`을 쓰고 실행 중 조건은 session guard가 강제한다.
50GiB 미만은 즉시 error, 50~100GiB는 새 session 시작을 막는다.

- [ ] **Step 4: deploy 구현**

```bash
target="${1:-unicon@192.168.0.138}"
remote_repo="${REMOTE_REPO:-/mnt/t500/GO1_to_ROS2_YEEPY}"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
required_branch="codex/hanyang-9f-mapping"
```

원격 repo/branch/clean worktree/local commit fetch 가능 여부를 read-only로 확인한 후
`git pull --ff-only`, staging, rosdep, `colcon build --symlink-install
--packages-up-to go1_mapping`만 실행한다. workspace/build 삭제와 강제 checkout은 금지한다.

- [ ] **Step 5: 운영 README 작성**

Jetson 명령:

```bash
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=100
./migration/verify_go1_mapping_preflight.sh 1.05 --static
ros2 launch go1_mapping mapping_session.launch.py
```

Laptop WSL 명령:

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch go1_mapping laptop_rviz.launch.py
```

종료 전 Jetson에서 finalizer를 실행한 뒤 mapping launch에 Ctrl+C를 한 번만 보낸다.
시작 위치는 고정하지 않고 특징이 충분한 곳에서 사진/표시/방향을 기록하고 같은 위치와
방향으로 복귀하도록 설명한다.

- [ ] **Step 6: script test/syntax와 커밋**

```powershell
python -m pytest -q -p no:cacheprovider migration/test_go1_mapping_scripts.py
wsl bash -n migration/deploy_go1_mapping.sh
wsl bash -n migration/verify_go1_mapping_preflight.sh
wsl bash -n migration/stage_local_ros2_packages.sh
```

```bash
git add migration/deploy_go1_mapping.sh migration/verify_go1_mapping_preflight.sh migration/test_go1_mapping_scripts.py migration/stage_local_ros2_packages.sh packages/go1_mapping/README.md
git commit -m "Add safe Jetson mapping deployment workflow"
```

---

### Task 11: 로컬 통합 검증과 현장 게이트

**Files:**
- Modify: `packages/go1_mapping/README.md`
- Create: `packages/go1_mapping/test/data/synthetic_corridor.pcd`
- Extend: `packages/go1_mapping/test/test_validation_report.py`
- Modify: `docs/superpowers/specs/2026-07-24-hanyang-9f-mapping-design.md`

**Interfaces:**
- 로컬: synthetic PCD의 merged PCD와 geometry reference
- 현장: 짧은 복도 session과 validation report
- Jetson 단계는 사용자가 실행을 별도로 승인한 뒤에만 수행

- [ ] **Step 1: 전체 로컬 테스트**

```powershell
$env:PYTHONPATH = "$PWD\packages\go1_mapping"
python -m pytest -q -p no:cacheprovider packages/go1_mapping/test migration/test_existing_map_scripts.py migration/test_go1_mapping_scripts.py
```

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go1_mapping
colcon test --packages-select go1_mapping --event-handlers console_direct+
colcon test-result --verbose
```

Expected: failed test 0.

- [ ] **Step 2: synthetic corridor offline pipeline**

fixture는 x 0..10m, y ±1.5m 벽, z -0.9..0.75m의 작은 ASCII PCD다.

```bash
mkdir -p /tmp/go1_mapping_synthetic/pcd
cp packages/go1_mapping/test/data/synthetic_corridor.pcd /tmp/go1_mapping_synthetic/pcd/chunk_000000.pcd
ros2 run go1_mapping pcd_to_grid --input-dir /tmp/go1_mapping_synthetic/pcd --output-pcd /tmp/go1_mapping_synthetic/merged.pcd --output-map /tmp/go1_mapping_synthetic/geometry_reference --sensor-height-m 1.05 --resolution 0.05 --voxel-size 0.10
```

Expected: PCD/PGM/YAML 생성, geometry-only role, PGM에 205와 0이 모두 존재.

- [ ] **Step 3: launch dry run**

```bash
ros2 launch go1_mapping mapping_session.launch.py --show-args
ros2 launch go1_mapping laptop_rviz.launch.py --show-args
```

Expected: package/file lookup error 없음. 센서 없이 실제 mapping session은 시작하지 않는다.

- [ ] **Step 4: 로컬 완료 커밋**

```bash
git add packages/go1_mapping/README.md packages/go1_mapping/test/data/synthetic_corridor.pcd packages/go1_mapping/test/test_validation_report.py docs/superpowers/specs/2026-07-24-hanyang-9f-mapping-design.md
git commit -m "Verify hybrid mapping workflow locally"
```

- [ ] **Step 5: 별도 승인 후 Jetson 배포/preflight**

```bash
./migration/deploy_go1_mapping.sh unicon@192.168.0.138
ssh unicon@192.168.0.138 'cd /mnt/t500/GO1_to_ROS2_YEEPY && ./migration/verify_go1_mapping_preflight.sh <measured_sensor_height_m> --static'
```

Expected: branch/worktree/disk/network/topic/TF 모두 PASS.

- [ ] **Step 6: 10~20m 짧은 복도 왕복 시험**

cart `<=0.5m/s`, 회전 `<=20deg/s`로 짧은 폐루프를 돈다. RViz에서 map/scan 정합,
`map_slam -> camera_init -> body`, loop closing 후 이중 벽 부재를 확인한다. guard
failure가 없고 복귀 위치 `<=0.5m`, yaw `<=10deg`여야 한다. finalizer report가
complete가 아니면 9층 전체로 확대하지 않고 session을 보존해 수정한다.

- [ ] **Step 7: 9층 전체 수집 게이트**

짧은 시험 통과 후에만 전체 복도/엘리베이터 홀/교차/계단 입구를 폐루프로 수집하고,
가능하면 반대 방향 두 번째 loop를 돈다. 최종 지도는 slam_toolbox map이며 PCD
projection은 geometry 비교용이다.

- [ ] **Step 8: 현장 상태 기록 커밋**

README에 실제 Jetson commit, sensor height, session id, report 경로를 기록한다.

```bash
git add packages/go1_mapping/README.md
git commit -m "Record Hanyang 9F mapping field validation"
```
