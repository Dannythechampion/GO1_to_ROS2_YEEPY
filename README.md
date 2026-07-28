# Go1 ROS2 Project

Ubuntu 22.04 / ROS2 Humble 기반 Unitree Go1 및 Livox MID-360 자율주행
프로젝트입니다.

```text
매핑: Livox -> FAST-LIO -> /cloud_registered_body -> /scan
      -> SLAM Toolbox -> map.yaml + map.pgm

주행: Livox -> FAST-LIO -> /cloud_registered_body -> /scan
      -> Map Server + AMCL -> Nav2 -> /cmd_vel -> Go1 driver
```

전체 설치 Gate는 [`migration/README.md`](migration/README.md), 상세한 매핑 및
자율주행 절차는
[`docs/GO1_NAV2_END_TO_END.md`](docs/GO1_NAV2_END_TO_END.md)를 참고하십시오.

저장소 표준 배치에서는 repository, `go1_ros2_ws`, Unitree SDK, 외부 소스, 지도 및
점검 결과를 `/mnt/t500` 아래에 보관합니다. 기존 Jetson의 `~/ros2_ws`와
`~/ws_livox`를 유지하는 실행 방법도 아래에 함께 제공합니다.

원본 Ubuntu 20.04 / ROS1 구성은 별도의 비공개 저장소에 보관합니다. ROS1
workspace 전체를 이 ROS2 workspace로 복사하지 마십시오. 고정된 archive commit과
Unitree SDK v3.8.6 기준은 [`ROS1_ARCHIVE.md`](ROS1_ARCHIVE.md)를 참고하십시오.

> [!WARNING]
> `arm:=true`는 LiDAR 고정, TF/extrinsic, AMCL, costmap, goal 취소 및 watchdog
> 검증을 모두 마친 뒤 제한된 시험 구역에서만 사용하십시오.

## 1. Jetson에서 브랜치 받기

### 기존 저장소가 있는 경우

```bash
cd /mnt/t500/go1_ros2_project

git fetch origin
git switch agent/nav2-end-to-end-workflow
git pull --ff-only
```

### 새로 clone하는 경우

```bash
mkdir -p /mnt/t500
cd /mnt/t500

git clone \
  --branch agent/nav2-end-to-end-workflow \
  --single-branch \
  https://github.com/Dannythechampion/GO1_to_ROS2_YEEPY.git \
  go1_ros2_project
```

### 현재 사용 중인 `~/ros2_ws`에 반영하는 경우

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export GO1_MAP_ROOT="$HOME/maps"
export ROS_DOMAIN_ID=100

cd /mnt/t500/go1_ros2_project

mkdir -p "$GO1_ROS2_WS/src"
cp -a packages/go1_driver "$GO1_ROS2_WS/src/"
cp -a packages/omx_navigation "$GO1_ROS2_WS/src/"

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

cd "$GO1_ROS2_WS"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select go1_driver omx_navigation

source install/setup.bash
```

## 2. 새 장소 매핑

Livox, FAST-LIO 및 매핑 launch는 서로 다른 터미널에서 실행합니다. 모든
터미널에서 동일한 `ROS_DOMAIN_ID`를 사용해야 합니다.

### 터미널 A — Livox MID-360

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 터미널 B — FAST-LIO

FAST-LIO가 초기화되는 동안 Go1을 정지 상태로 유지합니다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch fast_lio mapping.launch.py \
  config_path:="$GO1_ROS2_WS/install/omx_navigation/share/omx_navigation/config" \
  config_file:=fast_lio_mid360_navigation.yaml \
  rviz:=false
```

### 터미널 C — `/scan`, SLAM Toolbox 및 RViz

Unitree 전용 컨트롤러로 수동 매핑하므로 Go1 ROS driver는 시작하지 않습니다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch omx_navigation go1_mapping.launch.py \
  start_go1_driver:=false \
  rviz:=true
```

X11 forwarding 또는 화면 연결이 없다면 RViz 없이 실행합니다.

```bash
ros2 launch omx_navigation go1_mapping.launch.py \
  start_go1_driver:=false \
  rviz:=false
```

### 매핑 파이프라인 검사

```bash
cd /mnt/t500/go1_ros2_project

GO1_ROS2_WS="$HOME/ros2_ws" \
  ./migration/verify_mapping_pipeline.sh
```

정상적인 주요 출력 주파수는 다음과 같습니다.

| 토픽 | 예상 주파수 |
| --- | ---: |
| `/livox/lidar` | 약 10 Hz |
| `/livox/imu` | 약 200 Hz |
| `/cloud_registered_body` | 약 10 Hz |
| `/scan` | 약 10 Hz |
| `/Odometry` | 약 10 Hz |

다음 TF도 존재해야 합니다.

```text
map -> camera_init -> body
```

### 지도 저장

기존 지도에 덮어쓰지 말고 장소와 버전을 구분한 디렉터리에 저장합니다.

```bash
mkdir -p "$HOME/maps/floor9_v001"

cd /mnt/t500/go1_ros2_project

GO1_ROS2_WS="$HOME/ros2_ws" \
GO1_MAP_ROOT="$HOME/maps" \
  ./migration/save_nav2_map.sh \
  "$HOME/maps/floor9_v001/go1_map"
```

생성되는 파일:

```text
~/maps/floor9_v001/go1_map.yaml
~/maps/floor9_v001/go1_map.pgm
```

## 3. 저장 지도에서 Nav2 dry-run

매핑 launch를 종료한 뒤 Livox와 FAST-LIO는 각각 한 인스턴스만 유지합니다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export GO1_MAP_ROOT="$HOME/maps"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$GO1_MAP_ROOT/floor9_v001/go1_map.yaml" \
  arm:=false
```

RViz에서 다음 순서로 확인합니다.

1. Fixed Frame을 `map`으로 설정합니다.
2. `2D Pose Estimate`로 실제 위치와 방향을 지정합니다.
3. `/scan`이 지도 벽과 겹치는지 확인합니다.
4. 가까운 곳에 `2D Goal Pose`를 지정합니다.
5. Global/Local plan 및 `/cmd_vel`이 생성되는지 확인합니다.
6. `arm:=false`이므로 실제 Go1이 움직이지 않는지 확인합니다.

### preflight 검증

```bash
cd /mnt/t500/go1_ros2_project

GO1_ROS2_WS="$HOME/ros2_ws" \
  ./migration/verify_existing_map_navigation.sh preflight
```

### 초기 위치 지정 후 localization 검증

```bash
cd /mnt/t500/go1_ros2_project

GO1_ROS2_WS="$HOME/ros2_ws" \
  ./migration/verify_existing_map_navigation.sh localized
```

필요하면 다음 토픽과 TF를 직접 확인합니다.

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /go1/cmd_vel_applied
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init body
ros2 param get /go1_driver arm
```

## 4. 실제 Go1 주행

다음 조건을 모두 통과하기 전에는 `arm:=true`를 사용하지 마십시오.

- LiDAR가 영구 고정되어 있고 TF/extrinsic이 실제 장착값과 일치합니다.
- `/scan`과 지도 벽이 정지 및 이동 중 모두 일치합니다.
- AMCL pose jump 또는 `map -> camera_init` TF 단절이 없습니다.
- footprint와 inflation radius가 실제 Go1 외곽보다 작지 않습니다.
- dry-run goal, goal cancel 및 watchdog zero를 확인했습니다.
- e-stop 담당자와 비상 정지 공간을 확보했습니다.

`arm:=false` launch를 `Ctrl-C`로 완전히 종료한 뒤 실제 주행 launch를 시작합니다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export GO1_MAP_ROOT="$HOME/maps"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source /mnt/t500/go1_sdk/setup_unitree_sdk.bash
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$GO1_MAP_ROOT/floor9_v001/go1_map.yaml" \
  arm:=true
```

권장 시험 순서:

1. 전방 `0.3 m`
2. 작은 제자리 회전
3. 전방 `0.5-1.0 m`
4. 정적 장애물 접근과 정지
5. 2-3 waypoint 구간
6. 복도 한 구역 반복
7. 승인된 9층 경로

현재 기본 제한은 다음과 같습니다.

| 항목 | 제한값 |
| --- | ---: |
| 전진 속도 | `0.20 m/s` |
| 회전 속도 | `0.40 rad/s` |
| Go1 command watchdog | `0.35 s` |

목표 취소나 timeout 후에도 stepping이 계속되거나 예상과 다른 방향으로 움직이면
즉시 시험을 중단하십시오.
