# Go1 3D PCD localization 실행 Runbook — RustDesk/Jetson RViz

이 문서는 `nav2-workflow_3D` 브랜치에서 MID-360, FAST-LIO, 저장 PCD 기반
3D localization, Nav2, Go1 driver를 실제 터미널 순서대로 실행하는 절차다.
외부 PC는 RustDesk로 Jetson 화면과 입력만 원격 제어하고, RViz를 포함한 모든
ROS2 프로세스는 Jetson에서 직접 실행한다고 가정한다.

사용하는 TF 구조는 다음과 같다.

```text
map --PCD localization--> camera_init --FAST-LIO--> body
```

- FAST-LIO: `/livox/lidar`와 `/livox/imu`로 `camera_init -> body` 계산
- PCD localizer: `/cloud_registered_body`를 저장 PCD에 NDT/GICP로 정합
- Nav2: `map`의 2D occupancy map으로 경로를 계획
- Go1 driver: 기본 `arm:=false`; 검증 중에는 실제 모터 명령을 전송하지 않음

`go1_existing_map.launch.py`는 AMCL fallback 경로다. 이 Runbook에서는 실행하지
않고 `go1_pcd_navigation.launch.py`만 사용한다.

## 실행 전제 — RustDesk로 Jetson 데스크톱 사용

1. 외부 PC에서 RustDesk를 열고 Jetson에 접속한다.
2. RustDesk에 표시된 **Jetson 데스크톱 안에서** Terminal을 연다.
3. Jetson Terminal 탭을 최소 4개 준비한다.
4. 외부 PC에서는 RViz나 ROS2 노드를 별도로 실행하지 않는다.

첫 Jetson Terminal에서 그래픽 세션을 확인한다.

```bash
hostname
echo "DISPLAY=${DISPLAY:-<empty>}"
test -n "${DISPLAY:-}"
command -v rviz2
```

`DISPLAY`가 비어 있으면 일반 SSH Terminal일 가능성이 있다. 이 상태에서
`rviz:=true`를 실행하지 말고, RustDesk로 보이는 Jetson 데스크톱에서 Terminal을
다시 연다. 이후 이 문서의 모든 “터미널”은 RustDesk 안의 Jetson Terminal을
뜻한다.

## 0. 공통 경로와 지도 확인

센서로 생성한 지도는 코드 브랜치와 분리해 보관한다. 아래 두 지도는 같은 mapping
세션에서 만들어졌고 서로 정렬되어 있어야 한다.

```bash
export GO1_PROJECT_ROOT=/mnt/t500/go1_ros2_project
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
export GO1_MAP_SESSION=/mnt/t500/maps/hanyang_9f/20260728_204825
export MAP_YAML="$GO1_MAP_SESSION/slam_toolbox/hanyang_9f.yaml"
export MAP_PGM="$GO1_MAP_SESSION/slam_toolbox/hanyang_9f.pgm"
export PCD_MAP="$GO1_MAP_SESSION/pcd/merged.pcd"

test -r "$MAP_YAML"
test -r "$MAP_PGM"
test -r "$PCD_MAP"
grep -E '^(image|resolution|origin):' "$MAP_YAML"
```

검증에 사용한 `merged.pcd`와 동일한 파일인지 확인하려면 다음을 실행한다.

```bash
echo '9af57fbb96dd9364e466913dd453ef26ad9573564134d649230e1d1a3685201d  '"$PCD_MAP" \
  | sha256sum --check
```

해시가 다르면 실행을 막는 조건은 아니지만, 현재
`config/hanyang_9f.yaml`의 `pcd_to_map_xyz_rpy` 보정값을 그대로 신뢰하면 안 된다.

## 1. 최초 1회 빌드

외부 의존성과 자체 패키지가 `/mnt/t500/go1_ros2_ws/src`에 준비되어 있다는
전제다. 새 workspace라면 먼저 프로젝트 루트의 migration Gate 1~7과
`stage_local_ros2_packages.sh`를 실행한다. 이 스크립트는 기존 패키지를 안전상
덮어쓰지 않는다.

```bash
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
cd "$GO1_ROS2_WS"

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select go1_driver omx_navigation omx_pcd_localization
source install/setup.bash

ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 pkg prefix go1_driver
ros2 pkg prefix omx_navigation
ros2 pkg prefix omx_pcd_localization
```

## 2. 터미널 1 — MID-360

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

다른 터미널에서 다음 토픽이 연속적으로 나오는지 확인한다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

timeout 10 ros2 topic hz /livox/lidar
timeout 10 ros2 topic hz /livox/imu
```

## 3. 터미널 2 — FAST-LIO odometry

로봇을 움직이지 않은 상태에서 실행해 IMU 초기화를 기다린다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_path:=/mnt/t500/go1_ros2_ws/install/omx_navigation/share/omx_navigation/config \
  config_file:=fast_lio_mid360_navigation.yaml \
  rviz:=false
```

이 navigation 프로필은 FAST-LIO의 PCD 저장과 자체 RViz를 끄고,
`/Odometry`와 `/cloud_registered_body`를 localization 입력으로 유지한다.

## 4. 터미널 3 — FAST-LIO 입력 게이트

3D localization을 시작하기 전에 다음 검사를 통과해야 한다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

timeout 10 ros2 topic hz /Odometry
timeout 10 ros2 topic hz /cloud_registered_body
ros2 topic echo /Odometry --once
ros2 run tf2_ros tf2_echo camera_init body
```

정지 상태에서 pose가 급격하게 움직이거나 point cloud가 흔들리고 갈라지면 여기서
중단한다. PCD localization이 FAST-LIO 오류를 대신 해결해 주지는 않는다.

## 5. 터미널 4 — Jetson RViz, 3D localization, Nav2, disarmed Go1

이 터미널에서도 지도 변수를 다시 선언해야 한다.

```bash
export ROS_DOMAIN_ID=100
export GO1_MAP_SESSION=/mnt/t500/maps/hanyang_9f/20260728_204825
export MAP_YAML="$GO1_MAP_SESSION/slam_toolbox/hanyang_9f.yaml"
export PCD_MAP="$GO1_MAP_SESSION/pcd/merged.pcd"
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

test -r "$MAP_YAML"
test -r "$PCD_MAP"

ros2 launch omx_pcd_localization go1_pcd_navigation.launch.py \
  map:="$MAP_YAML" \
  pcd_map:="$PCD_MAP" \
  rviz:=true \
  start_go1_driver:=true \
  arm:=false
```

이 launch는 AMCL을 시작하지 않는다. 다음 구성만 실행한다.

- PCD localizer
- 2D map server
- point cloud to LaserScan
- Nav2 planner/controller/costmap
- RViz
- `arm=false` Go1 driver

`rviz:=true`이므로 RViz는 Jetson에서 실행되고 그 창이 RustDesk 화면에 나타난다.
외부 PC는 RViz 화면 픽셀과 마우스·키보드 입력만 주고받으므로 외부 PC에 ROS2를
설치하거나 `ROS_DOMAIN_ID`를 설정할 필요가 없다.

RViz 창이 보이지 않으면 launch를 반복 실행하지 말고 RustDesk 안의 새 Jetson
Terminal에서 다음을 확인한다.

```bash
echo "DISPLAY=${DISPLAY:-<empty>}"
test -n "${DISPLAY:-}"
pgrep -af rviz2
ros2 node list | grep -Fx /rviz2
```

RustDesk 화면이 끊기거나 느린 현상은 원격 화면 전송 문제일 수 있다. 센서와
localization의 실제 상태는 화면 프레임률 대신 `/Odometry`, point cloud topic
rate와 `/pcd_localizer/status`로 판단한다.

## 6. RustDesk에서 Jetson RViz 초기 위치와 정합 확인

1. RustDesk 화면에 열린 Jetson RViz의 Fixed Frame이 `map`인지 확인한다.
2. `2D Pose Estimate`로 실제 위치의 `x`, `y`, `yaw`를 대략 지정한다.
3. `/pcd_localizer/aligned_cloud`가 저장 PCD와 겹치는지 확인한다.
4. `/scan`과 2D occupancy map의 벽이 겹치는지 확인한다.
5. 아래 상태가 `LOCALIZED fitness=...`가 될 때까지 목표를 보내지 않는다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 topic echo /pcd_localizer/status --once
ros2 topic echo /pcd_localizer/pose --once
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo map body
ros2 param get /go1_driver arm
```

마지막 명령은 반드시 다음을 출력해야 한다.

```text
Boolean value is: False
```

## 7. 자동 dry-run 검증

localizer가 `LOCALIZED` 상태인 동안 새 터미널에서 실행한다.

```bash
export ROS_DOMAIN_ID=100
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
cd /mnt/t500/go1_ros2_project

./migration/verify_pcd_localization.sh
```

이 스크립트는 핵심 topic, `camera_init -> body`, `map -> camera_init`, Nav2
lifecycle, AMCL 미실행, `arm=false`를 검사한다. 모든 항목이 `PASS`가 아니면
RViz goal을 보내지 않는다.

## 8. Nav2 계획 dry-run

1. `arm=false`를 다시 확인한다.
2. RViz의 `2D Goal Pose`로 가까운 목표를 지정한다.
3. Global Plan과 Local Plan이 생성되는지 확인한다.
4. `/cmd_vel`이 생성되더라도 Go1이 움직이지 않는지 확인한다.
5. 목표를 취소하고 `/cmd_vel`이 0으로 돌아오는지 확인한다.

```bash
ros2 param get /go1_driver arm
ros2 topic echo /cmd_vel --once
ros2 topic echo /go1/control_state --once
```

이 Runbook은 실기 구동을 승인하지 않는다. PCD–2D map 보정, LiDAR 고정
extrinsic, 정지 drift, 좁은 복도 재현 시험이 완료되기 전에는 `arm:=true`로
재실행하지 않는다.

## 9. 상태별 조치

| 상태 | 의미 | 우선 확인 |
| --- | --- | --- |
| `WAITING_FOR_INITIAL_POSE` | 초기 추정값 없음 | RViz `2D Pose Estimate` |
| `REJECTED ... tf_error` | 해당 시각의 odometry TF 없음 | FAST-LIO와 timestamp |
| `REJECTED ... too_few_scan_points` | 현재 scan 부족 | LiDAR 가림, 토픽, 필터 |
| `REJECTED ... too_few_target_points` | 초기 위치 주변 map 부족 | 초기 pose와 PCD 경로 |
| `REJECTED ... fitness` | 현재 scan과 PCD 불일치 | 지도, 환경 변화, 초기 pose |
| `REJECTED ... pose_jump` | 허용 범위보다 큰 보정 | 초기 pose 또는 odometry 점프 |
| `STALE` | 최근 5초간 승인된 정합 없음 | 센서, odometry, map 정합 |

`STALE` 상태에서는 안전을 위해 `map -> camera_init` TF 발행이 중단된다.

## 10. 종료 순서

RustDesk 연결을 먼저 끊으면 Jetson의 ROS2 프로세스가 계속 실행될 수 있다.
RustDesk 화면을 닫기 전에 각 Jetson Terminal에서 `Ctrl+C`로 다음 순서대로
종료한다.

1. 3D localization/Nav2/Go1 driver
2. FAST-LIO
3. Livox driver

다음 명령으로 주요 노드가 종료됐는지 확인한 뒤 RustDesk 연결을 끊는다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 node list | grep -E 'livox|fastlio|pcd_localizer|controller_server|go1_driver' || true
```

종료 후 Go1이 계속 stepping하면 리모컨 또는 비상 정지 절차로 즉시 중단한다.
