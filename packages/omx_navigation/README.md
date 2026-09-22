# Go1 ROS2 Humble 기존 지도 내비게이션

이 패키지는 Livox MID-360과 FAST-LIO2가 이미 실행 중인 Jetson에서 기존
2D posegraph로 SLAM Toolbox localization과 Nav2 경로계획을 수행합니다. AMCL은
레거시 fallback일 뿐 권장 운용 경로가 아닙니다. 기본 실행은
`arm:=false`이므로 `/cmd_vel`이 생성되어도 실제 Go1에는 동작 명령을
전송하지 않습니다.

## 현재 시험 프레임

LiDAR가 아직 영구 고정되지 않았기 때문에 최종 `base_link -> lidar` TF를
만들지 않습니다. 현재 FAST-LIO가 제공하는 프레임을 임시로 사용합니다.

```text
map -> camera_init -> body        FAST-LIO2의 6-DoF 자세
                   -> body_nav    planar_base_frame: x, y, yaw만
```

- `map -> camera_init`: slam_toolbox localization (유일한 발행자)
- `camera_init`: Nav2 odom 프레임
- `body_nav`: Nav2 base 프레임과 LaserScan 프레임 (롤·피치·높이 제거)
- `/Odometry`: FAST-LIO odometry
- `/cloud_registered_body`: LaserScan 변환 입력
- `/scan`: slam_toolbox, `localization_supervisor`, costmap 입력

전체 구조와 각 알고리즘 설명은 [루트 README](../../README.md)에 있습니다.

LiDAR를 고정한 뒤에는 실제 장착 위치를 측정해 고정 TF와 FAST-LIO
extrinsic을 다시 설정해야 합니다.

## 기존 지도 준비

수동 경계까지 반영한 검증 세션의 기본 주행 지도는 저장소에 포함되어 있습니다.

```text
/mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml
/mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.pgm
```

`go1_existing_map.launch.py`의 기본 `map` 인자는 위 YAML을 사용합니다.
별도의 운영 지도 디렉터리로 복사하려면 프로젝트 루트에서 실행합니다.

```bash
cd /mnt/t500/go1_ros2_project
./migration/prepare_existing_map.sh
```

복사된 지도는 다음 위치에 생성됩니다.

```text
/mnt/t500/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml
/mnt/t500/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.pgm
```

이 지도는 반사 의심 자유공간을 미확인 영역으로 되돌렸습니다. Global
costmap은 미확인 셀을 보존하고 NavFn은 `allow_unknown: false`로 해당 영역을
통과하는 경로를 만들지 않습니다.

## 빌드

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
cd /mnt/t500/go1_ros2_ws
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
```

ROS1 Noetic 작업공간과 토픽이 섞이지 않도록 모든 ROS2 시험에서
`ROS_DOMAIN_ID=100`을 사용합니다.

## 실행 순서

각 명령은 서로 다른 Jetson 터미널에서 실행합니다.

### 1. MID-360

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 2. FAST-LIO2

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/mnt/t500/go1_ros2_ws/install/omx_navigation/share/omx_navigation/config \
  config_file:=fast_lio_mid360_navigation.yaml \
  rviz:=false
```

FAST-LIO 자체 RViz와 PCD 저장은 이 시험에서 사용하지 않습니다. 저부하 프로필은
`map_en: false`, `dense_publish_en: false`, `pcd_save_en: false`로 설정되어 있습니다.

### 3. pose-graph localization, Nav2, 저부하 RViz

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch omx_navigation go1_posegraph_navigation.launch.py arm:=false
```

통합 launch는 다음을 시작합니다.

- `camera_init -> body_nav` 평면 프레임 (`planar_base_frame`)
- `/cloud_registered_body`를 `/scan`으로 변환
- 기존 지도 map server와 저장 posegraph를 불러온 SLAM Toolbox localization
- `localization_supervisor` (초기 정합, READY 판정, 드리프트 감시)
- Nav2 planner, controller, smoother, behavior, bt_navigator, velocity smoother
- `rviz_goal_bridge`와 `cmd_vel_safety_gate`
- 15 FPS 저부하 RViz, 그리고 `start_go1_driver:=true`일 때 Go1 드라이버

현장에서는 이 launch를 직접 쓰지 말고 `jetson_field_deploy.sh dry-run`을 씁니다
(아래 "Jetson 현장 실행"). 레거시 AMCL fallback은
`ros2 launch omx_navigation go1_existing_map.launch.py arm:=false`로 진단용으로만 남아
있습니다.

## RViz 시험

1. 기존 지도와 붉은색 `/scan`이 보이는지 확인합니다.
2. `2D Pose Estimate`로 지도상의 실제 위치와 방향을 한 번 대략 지정합니다(1 m, 90° 안).
3. supervisor가 `READY`가 되고(약 6초) `/scan`이 기존 지도 벽과 겹치는지 확인합니다.
4. `2D Goal Pose`로 가까운 목표를 지정합니다.
5. Global Plan과 Local Plan이 생성되는지 확인합니다.
6. 목표를 취소하고 속도 명령이 0으로 돌아오는지 확인합니다.

이 단계의 `2D Goal Pose`는 경로와 속도 계산만 확인하기 위한 것입니다.
`arm:=false`이므로 실제 Go1은 움직이지 않아야 합니다.

## ROS1 DWA 튜닝의 ROS2 적용값

ROS1에서 분석한 YAML 5개 중 실제 수정했던 파일은
`base_local_planner_params.yaml` 하나입니다. 그 설정 의도를 ROS2 DWB에
다음처럼 옮겼습니다.

| 목적 | ROS2 값 |
| --- | --- |
| 위치 도착 오차 | `0.20 m` |
| 방향 도착 오차 | `0.15 rad` |
| 경로 추종 가중치 | `PathAlign/PathDist: 40.0` |
| 목표 추종 가중치 | `GoalAlign/GoalDist: 20.0` |
| 장애물 critic | `BaseObstacle: 0.01` |
| 궤적 예측 시간 | `2.0 s` |
| 속도 샘플 | `vx=10`, `vy=1`, `vtheta=20` |
| oscillation reset 거리 | `0.20 m` |

예전 ROS1 최고속도 `0.30 m/s`, `0.60 rad/s`는 현재 Go1 드라이버 상한보다
큽니다. Nav2와 실제 드라이버의 속도 차이로 인한 경로 이탈과 목표 부근
흔들림을 줄이기 위해 ROS2에서는 각각 `0.20 m/s`, `0.40 rad/s`로
통일했습니다.

## 확인 명령

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered_body
ros2 topic hz /scan
ros2 topic hz /Odometry
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init body
ros2 param get /go1_driver arm
ros2 topic echo /cmd_vel
```

자동 무구동 검증은 다음 단계에서 실행합니다.

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_existing_map_navigation.sh
```

## Jetson 현장 실행

이 패키지는 fail-closed 현장 runner를 제공합니다. 실제 주행 전에는 LiDAR 고정,
FAST-LIO extrinsic, 물리 e-stop, 안전 스탠드 또는 넓은 시험 공간을 운영자가 직접
확인해야 합니다.

```bash
cd /mnt/t500/GO1_to_ROS2_YEEPY
./migration/jetson_field_deploy.sh stage "$PWD" /mnt/t500/go1_ros2_ws
./migration/build_unitree_go1_wrapper.sh /path/to/unitree_legged_sdk
./migration/jetson_field_deploy.sh build /mnt/t500/go1_ros2_ws
./migration/jetson_field_deploy.sh preflight /mnt/t500/go1_ros2_ws
```

MID-360과 FAST-LIO를 별도 터미널에서 시작한 다음 무구동 검증을 실행합니다.

```bash
./migration/jetson_field_deploy.sh dry-run /mnt/t500/go1_ros2_ws
```

dry-run은 항상 `arm:=false`이며 진단 기록을 켭니다. RViz에서 대략적인
`2D Pose Estimate`를 지정한 뒤 `READY/NONE`, `ready=true`, scan-map 정합과
`map -> camera_init -> body_nav`를 확인합니다. `/slam_localization/pose`는 한 번 오는
handshake이고, 이후 최신 scan과 두 TF edge가 연속 품질을 감시합니다.

dry-run을 `Ctrl-C`로 종료해 모든 노드가 exit 0인지 확인한 뒤, Go1을 안전하게
지지하고 운영자가 e-stop을 누를 준비가 된 경우에만 다음 명령을 사용합니다.

```bash
./migration/jetson_field_deploy.sh armed GO1_ARMED_AND_ESTOP_READY /mnt/t500/go1_ros2_ws
```

armed launch는 `arm:=true`, `start_go1_driver:=true`, `record_localization:=true`와
정확한 토큰을 모두 요구합니다. 새 launch에서 초기 자세와 READY를 다시 확인하고 첫
goal은 **0.3 m 이내**로 제한합니다. goal cancel, ready loss, watchdog 정지,
`Ctrl-C` 반복 stand, 물리 e-stop을 각각 확인하기 전에는 시험 반경을 늘리지 마십시오.

`INPUT_MISSING`, `LOW_OVERLAP`, `AMBIGUOUS`, `ODOM_RESET`, `TF_CONFLICT`,
`POSE_OUTSIDE_MAP`, `POSE_DRIFT`, `EXTRINSIC_UNCALIBRATED` 중 하나라도 표시되면
속도 gate가 닫힌 상태에서 원인을 먼저 복구하십시오. 기존 지도와 실시간 scan이 맞지
않으면 armed 실행 대신 지도를 다시 작성해야 합니다.

## 목표 경로와 localization 감시 (2026-09-18 현장 분석 반영)

근거와 측정값은 `migration/FIELD_SESSION_2026-09-18.md`에 있습니다.

- **목표 경로는 `rviz_goal_bridge` 하나뿐입니다.** Humble `bt_navigator`도 `goal_pose`를
  직접 구독해서 클릭이 Nav2에 두 번 들어갔고, 브리지의 READY 검사가 무력했습니다. 두
  navigation launch 모두 `bt_navigator`의 `goal_pose`를 막았습니다.
- **새 클릭은 진행 중 목표를 대체합니다**(무시하지 않음). localization 손실,
  `/go1/manual_override`, `/go1/execution_fault`가 들어오면 목표를 취소하고 새 목표를 거부합니다.
- **모든 목표 상태가 보입니다.** `/navigation/goal_status`(JSON)와 RViz `Navigation Goal`
  마커: 노랑 진행, 초록 `ARRIVED`, 빨강 `IGNORED:`/`CANCELED:` + 이유, 회색 `PREEMPTED`.
- **자체 보정은 TF 충돌이 아닙니다.** 첫 초기 자세 전의 점프는 추적하지 않고, 푸시 직후
  푸시한 자세 근처로 떨어지는 점프는 보정으로 인정합니다. slam_toolbox 자세로 보정이
  확인된 뒤의 불연속은 그대로 `TF_CONFLICT`입니다. 초기 자세는 **한 번**만 지정하면 됩니다.
- **slam_toolbox의 응답을 놓치지 않습니다.** slam_toolbox는 푸시 후 처음 *받은* scan으로
  응답하는데, 그 scan은 보통 푸시 전에 찍힌 것입니다. 푸시한 자세에 떨어지는 응답이면
  stamp가 앞서도 받아들입니다(정지 상태에서는 이 응답이 유일합니다). 이런 응답은 보정 확인
  창을 일찍 닫지 않습니다.
- status/CSV의 `missing_inputs`가 `INPUT_MISSING`의 원인을 적습니다: `map`, `scan`,
  `odometry`, `initial_pose`, `alignment`(검색 전), `slam_answer`, `tf`, `scan_match`.
  클릭 직후 `slam_answer,tf`는 정상이고, `map,alignment`가 계속되면 map_server가 active가
  아닌 것입니다(`verify_posegraph_navigation.sh preflight`가 확인).
- **추적 드리프트를 감시하고 고칩니다.** READY 동안 2초마다 추적 자세 주변을 국소 정제해
  `consistency_gap`을 계산합니다. 세 번 연속 0.08 이상이고 방향이 일치하면 보정 자세를
  slam_toolbox에 다시 넣고, 해결되지 않으면 `POSE_DRIFT`로 정지합니다.
  `drift_auto_correct:=false`로 자동 보정을 끌 수 있습니다.
- status/CSV의 `ambiguity_margin`은 **lock 시점 값**입니다. 주행 중 품질은
  `consistency_gap`, `drift_offset_*`, `drift_corrections`, `tf_corrections_explained`로 봅니다.
- 세션 bag에 `/goal_pose`, `/navigation/goal_status`, `/slam_localization/initialpose`,
  `/go1/*` 토픽이 추가로 기록됩니다. 분석은 `tools/session_report.py`와
  `tools/replay_localization.py`(ROS 없이 실행)로 합니다.
