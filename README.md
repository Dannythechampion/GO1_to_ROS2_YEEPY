# Go1 ROS2 — Hanyang 9F Mapping & Nav2 Navigation

Ubuntu 22.04 / ROS2 Humble 기반 Unitree Go1 및 Livox MID-360 자율주행 프로젝트입니다.

이 프로젝트의 권장 운영 방식은 두 브랜치의 역할을 분리하는 것입니다.

| 단계 | 사용할 브랜치 | 역할 |
|---|---|---|
| 지도 생성 | `codex/hanyang-9f-mapping` | 3D PCD, rosbag, SLAM Toolbox 2D 지도 및 검증 결과 생성 |
| 자율주행 | `agent/nav2-end-to-end-workflow` | 저장 지도 AMCL localization, Nav2 경로계획 및 Go1 주행 |

전체 흐름:

```text
1. codex/hanyang-9f-mapping
   MID-360 + FAST-LIO
       ├─ 3D PCD
       ├─ rosbag
       ├─ SLAM Toolbox 2D 지도
       ├─ pose graph
       └─ validation report
                ↓
   slam_toolbox/hanyang_9f.yaml
                ↓
2. agent/nav2-end-to-end-workflow
   Map Server + AMCL + Nav2
                ↓
             /cmd_vel
                ↓
             Go1 driver
```

> [!WARNING]
> `arm:=true`는 LiDAR 고정, TF/extrinsic, 지도 품질, AMCL, costmap, goal 취소 및
> watchdog 검증을 모두 통과한 뒤 제한된 시험 구역에서만 사용하십시오.

## 1. 기본 경로

이 문서는 다음 경로를 기준으로 설명합니다.

```text
Git repository:  /mnt/t500/go1_ros2_project
ROS2 workspace:  /home/unicon/ros2_ws
Livox workspace: /home/unicon/ws_livox
Map storage:     /mnt/t500/maps/hanyang_9f
Unitree SDK:     /mnt/t500/go1_sdk
```

공통 환경 변수:

```bash
export GO1_PROJECT_ROOT="/mnt/t500/go1_ros2_project"
export GO1_ROS2_WS="$HOME/ros2_ws"
export GO1_MAP_ROOT="/mnt/t500/maps/hanyang_9f"
export ROS_DOMAIN_ID=100
```

---

# Part A — 한양대 9층 지도 생성

## 2. 매핑 브랜치 받기

작업 중인 파일이 없는지 먼저 확인합니다.

```bash
cd /mnt/t500/go1_ros2_project
git status --short
```

출력이 없을 때 브랜치를 전환합니다.

로컬 브랜치가 처음이라면:

```bash
git fetch origin

git switch --track \
  origin/codex/hanyang-9f-mapping
```

이미 로컬 브랜치가 있다면:

```bash
git switch codex/hanyang-9f-mapping
git pull --ff-only
```

확인:

```bash
git branch --show-current
git log -1 --oneline
```

예상 브랜치:

```text
codex/hanyang-9f-mapping
```

## 3. `go1_mapping` 패키지 배치

현재 사용 중인 `~/ros2_ws`에 매핑 패키지를 반영합니다.

```bash
export GO1_PROJECT_ROOT="/mnt/t500/go1_ros2_project"
export GO1_ROS2_WS="$HOME/ros2_ws"

mkdir -p "$GO1_ROS2_WS/src/go1_mapping"

cp -a \
  "$GO1_PROJECT_ROOT/packages/go1_mapping/." \
  "$GO1_ROS2_WS/src/go1_mapping/"
```

## 4. 매핑 패키지 빌드

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

if [ -r "$GO1_ROS2_WS/install/setup.bash" ]; then
  source "$GO1_ROS2_WS/install/setup.bash"
fi

cd "$GO1_ROS2_WS"

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y

colcon build \
  --symlink-install \
  --packages-up-to go1_mapping

source install/setup.bash
```

설치 확인:

```bash
ros2 pkg prefix go1_mapping
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio

ros2 launch go1_mapping \
  mapping_session.launch.py \
  --show-args
```

rosbag zstd compression plugin 오류가 발생하면:

```bash
sudo apt-get update

sudo apt-get install \
  ros-humble-rosbag2-compression-zstd
```

## 5. 매핑 전 하드웨어 점검

기존에 실행 중인 Livox, FAST-LIO, SLAM Toolbox가 있다면 각각 실행한 터미널에서 `Ctrl-C`로 종료합니다.

중복 노드 확인:

```bash
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"

ros2 node list |
  grep -E \
  'livox|laser_mapping|slam_toolbox|pcd_chunk_writer|mapping_session_guard'
```

매핑 시작 전에는 위 노드들이 남아 있지 않아야 합니다.

네트워크 확인:

```bash
ip -br address
ip route

ping -c 3 192.168.1.138
```

현재 확인된 장비 구성:

```text
Jetson LiDAR NIC: 192.168.1.5/24
MID-360:          192.168.1.138
```

Livox 설정 확인:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

LIVOX_CONFIG="$(
  ros2 pkg prefix livox_ros_driver2
)/share/livox_ros_driver2/config/MID360_config.json"

grep -nE \
  '"(cmd_data_ip|push_msg_ip|point_data_ip|imu_data_ip|log_data_ip|ip)"' \
  "$LIVOX_CONFIG"
```

저장공간과 filesystem 확인:

```bash
df -h /mnt/t500

findmnt \
  -no SOURCE,FSTYPE,OPTIONS \
  /mnt/t500
```

권장 조건:

```text
매핑 시작 전 여유 공간: 100 GiB 이상
강제 중단 기준:         50 GiB 미만
권장 filesystem:        ext4
```

`pcd_chunk_writer`는 `O_TMPFILE`과 `linkat(..., AT_EMPTY_PATH)`를 지원하는 Linux filesystem이 필요합니다.

## 6. 매핑 세션 시작

매핑 launch가 다음 항목을 한 번에 실행합니다.

```text
Livox MID-360
FAST-LIO
rosbag2
PCD chunk writer
pointcloud_to_laserscan
SLAM Toolbox
mapping session guard
```

Go1 driver와 Nav2는 실행하지 않습니다. 매핑 중 Go1은 Unitree 컨트롤러로 수동 조종합니다.

### 터미널 A — 매핑 launch

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export GO1_MAP_ROOT="/mnt/t500/maps/hanyang_9f"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

mkdir -p "$GO1_MAP_ROOT"

export SESSION_ID="$(date +%Y%m%d_%H%M%S)"
export SESSION_DIR="$GO1_MAP_ROOT/$SESSION_ID"

echo "SESSION_ID=$SESSION_ID"
echo "SESSION_DIR=$SESSION_DIR"
```

출력된 `SESSION_ID`를 기록한 뒤 실행합니다.

```bash
ros2 launch go1_mapping \
  mapping_session.launch.py \
  session_root:="$GO1_MAP_ROOT" \
  session_id:="$SESSION_ID" \
  ros_domain_id:=100 \
  start_livox:=true \
  start_fast_lio:=true
```

시작 순서:

```text
즉시: rosbag 및 Livox 시작
3초 후: FAST-LIO 시작
8초 후: PCD writer, /scan, SLAM Toolbox, guard 시작
```

이 터미널은 finalizer가 완료될 때까지 종료하지 마십시오.

## 7. 매핑 파이프라인 확인

### 터미널 B — 토픽 확인

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"
```

토픽 목록:

```bash
ros2 topic list |
  grep -E \
  'livox|Odometry|cloud_registered|scan|map_slam'
```

주파수 확인:

```bash
timeout 10 ros2 topic hz /livox/lidar
timeout 10 ros2 topic hz /livox/imu
timeout 10 ros2 topic hz /Odometry
timeout 10 ros2 topic hz /cloud_registered
timeout 10 ros2 topic hz /cloud_registered_body
timeout 10 ros2 topic hz /scan
```

필수 최소 주파수:

| 토픽 | 최소 주파수 |
|---|---:|
| `/livox/lidar` | 8 Hz |
| `/livox/imu` | 100 Hz |
| `/Odometry` | 8 Hz |

SLAM 지도 확인:

```bash
ros2 topic echo /map_slam --once
```

TF 확인:

```bash
timeout 10 ros2 run tf2_ros \
  tf2_echo map_slam camera_init

timeout 10 ros2 run tf2_ros \
  tf2_echo camera_init body
```

매핑 중 TF 구조:

```text
map_slam -> camera_init -> body
```

## 8. RViz 실행

이 브랜치에는 전용 `laptop_rviz.launch.py`가 실제로 포함되어 있지 않으므로 RViz를 직접 실행합니다.

X11 forwarding 또는 GUI 환경이 있는 터미널:

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

rviz2
```

RViz 설정:

| 항목 | 값 |
|---|---|
| Fixed Frame | `map_slam` |
| PointCloud2 | `/cloud_registered` |
| PointCloud2 | `/cloud_registered_body` |
| LaserScan | `/scan` |
| Map | `/map_slam` |
| Odometry | `/Odometry` |
| TF | 활성화 |

X11 오류가 발생하면 Jetson에서는 RViz 없이 매핑을 유지하고 ROS2 Humble이 설치된 GUI PC에서 동일한 `ROS_DOMAIN_ID=100`으로 RViz를 실행합니다.

## 9. 9층 매핑 경로

1. 출발 위치와 방향을 사진 또는 바닥 표시로 기록합니다.
2. 9층 외곽의 큰 폐루프를 먼저 주행합니다.
3. 내부 복도와 홀을 작은 loop로 연결합니다.
4. 같은 복도를 반대 방향으로 다시 통과합니다.
5. 엘리베이터 홀, 교차로 및 계단 입구를 포함합니다.
6. 빠른 회전과 사람 밀집 구간을 피합니다.
7. 마지막에는 출발 위치와 같은 방향으로 복귀합니다.
8. RViz에서 loop closure 후 벽이 이중으로 겹치지 않는지 확인합니다.

권장 초기 속도:

```text
전진: 약 0.10 m/s
회전: 약 0.20 rad/s 이하
```

최종 복귀 기준:

```text
출발점 위치 오차: 0.5 m 이하
출발 방향 오차:   10 deg 이하
```

## 10. 지도 finalization

> [!IMPORTANT]
> Mapping launch를 종료하기 전에 finalizer를 실행해야 합니다.
> PCD writer와 SLAM Toolbox service가 살아 있어야 합니다.

### 터미널 C — finalizer

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"
```

터미널 A에서 출력된 실제 `SESSION_ID`를 입력합니다.

```bash
export SESSION_ID="실제_SESSION_ID"
export SESSION_DIR="/mnt/t500/maps/hanyang_9f/$SESSION_ID"

read -rp \
  "바닥에서 LiDAR 중심까지 실제 높이(m): " \
  SENSOR_HEIGHT_M
```

Finalizer 실행:

```bash
ros2 run go1_mapping \
  map_finalizer.py \
  --session-dir "$SESSION_DIR" \
  --sensor-height-m "$SENSOR_HEIGHT_M"

echo "finalizer exit=$?"
```

Finalizer 실행 순서:

```text
1. PCD buffer flush
2. PCD chunk 검증
3. SLAM Toolbox PGM/YAML 저장
4. posegraph/data 저장
5. PCD 병합
6. PCD 기반 geometry reference 생성
7. validation report 생성
8. session manifest complete 처리
```

결과 확인:

```bash
grep -E \
  'status:|failed_step:|error:' \
  "$SESSION_DIR/validation/session_manifest.yaml"

grep -E \
  '^complete:|^needs_return_check:' \
  "$SESSION_DIR/validation/report.yaml"
```

정상 결과:

```text
status: complete
complete: true
needs_return_check: false
```

정상 완료 후 터미널 A의 mapping launch에 `Ctrl-C`를 한 번 보냅니다.

Finalizer가 실패하면 동일 세션에 결과를 덮어쓰지 않습니다. 기존 세션을 보존하고 원인을 해결한 뒤 새로운 `SESSION_ID`로 다시 매핑합니다.

## 11. 생성 파일 확인

```bash
find "$SESSION_DIR" \
  -maxdepth 3 \
  -type f \
  -printf '%P  %s bytes\n' |
  sort

ros2 bag info "$SESSION_DIR/bag/raw"
```

정상적인 결과 구조:

```text
<SESSION_DIR>/
├─ bag/
│  └─ raw/
├─ pcd/
│  ├─ chunk_*.pcd
│  └─ merged.pcd
├─ slam_toolbox/
│  ├─ hanyang_9f.pgm
│  ├─ hanyang_9f.yaml
│  ├─ hanyang_9f.posegraph
│  └─ hanyang_9f.data
├─ pcd2d/
│  ├─ geometry_reference.pgm
│  └─ geometry_reference.yaml
└─ validation/
   ├─ session_manifest.yaml
   ├─ health.yaml
   └─ report.yaml
```

## 12. Nav2에 사용할 지도

Nav2에서 사용할 주행용 지도:

```text
$SESSION_DIR/slam_toolbox/hanyang_9f.yaml
```

다음 파일은 Nav2 주행용 지도로 사용하지 않습니다.

| 파일 | 용도 |
|---|---|
| `pcd/merged.pcd` | 3D 지도 확인 및 보관 |
| `pcd2d/geometry_reference.yaml` | PCD 형상 비교용 |
| `slam_toolbox/hanyang_9f.posegraph` | SLAM 재개 및 수정 |
| `slam_toolbox/hanyang_9f.data` | pose graph 데이터 |

---

# Part B — 저장 지도에서 Nav2 자율주행

## 13. Nav2 브랜치로 전환

매핑 launch와 관련 노드가 모두 종료된 상태에서 실행합니다.

```bash
cd /mnt/t500/go1_ros2_project

git status --short
```

출력이 없다면:

```bash
git fetch origin
git switch agent/nav2-end-to-end-workflow
git pull --ff-only
```

브랜치 확인:

```bash
git branch --show-current
git log -1 --oneline
```

예상 브랜치:

```text
agent/nav2-end-to-end-workflow
```

지도는 `/mnt/t500/maps`에 저장되어 있으므로 Git 브랜치를 전환해도 삭제되지 않습니다.

## 14. Nav2 패키지 반영 및 빌드

```bash
export GO1_PROJECT_ROOT="/mnt/t500/go1_ros2_project"
export GO1_ROS2_WS="$HOME/ros2_ws"

mkdir -p "$GO1_ROS2_WS/src/go1_driver"
mkdir -p "$GO1_ROS2_WS/src/omx_navigation"

cp -a \
  "$GO1_PROJECT_ROOT/packages/go1_driver/." \
  "$GO1_ROS2_WS/src/go1_driver/"

cp -a \
  "$GO1_PROJECT_ROOT/packages/omx_navigation/." \
  "$GO1_ROS2_WS/src/omx_navigation/"
```

빌드:

```bash
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

if [ -r "$GO1_ROS2_WS/install/setup.bash" ]; then
  source "$GO1_ROS2_WS/install/setup.bash"
fi

cd "$GO1_ROS2_WS"

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y

colcon build \
  --symlink-install \
  --packages-select go1_driver omx_navigation

source install/setup.bash
```

확인:

```bash
ros2 pkg prefix go1_driver
ros2 pkg prefix omx_navigation
```

## 15. Livox 실행

### 터미널 A

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch livox_ros_driver2 \
  msg_MID360_launch.py
```

## 16. FAST-LIO 실행

### 터미널 B

FAST-LIO 초기화 중에는 Go1을 정지 상태로 유지합니다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch fast_lio \
  mapping.launch.py \
  config_path:="$GO1_ROS2_WS/install/omx_navigation/share/omx_navigation/config" \
  config_file:=fast_lio_mid360_navigation.yaml \
  rviz:=false
```

FAST-LIO 확인:

```bash
timeout 10 ros2 topic hz /livox/lidar
timeout 10 ros2 topic hz /livox/imu
timeout 10 ros2 topic hz /Odometry
timeout 10 ros2 topic hz /cloud_registered_body
```

## 17. 검증된 한양대 9층 지도 로드

### 터미널 C

실제 매핑 세션 ID를 입력합니다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

export SESSION_ID="실제_SESSION_ID"

export MAP_FILE="/mnt/t500/maps/hanyang_9f/$SESSION_ID/slam_toolbox/hanyang_9f.yaml"

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

test -s "$MAP_FILE"

echo "Using Nav2 map:"
echo "$MAP_FILE"
```

먼저 무구동 모드로 실행합니다.

```bash
ros2 launch omx_navigation \
  go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  arm:=false
```

X11 또는 GUI 연결이 없다면:

```bash
ros2 launch omx_navigation \
  go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  arm:=false \
  rviz:=false
```

## 18. RViz localization 절차

RViz에서 다음 순서로 진행합니다.

1. Fixed Frame을 `map`으로 설정합니다.
2. 지도 `hanyang_9f.yaml`이 표시되는지 확인합니다.
3. `2D Pose Estimate`를 선택합니다.
4. 실제 Go1의 위치와 방향을 지도에 지정합니다.
5. `/scan`이 지도 벽과 겹치는지 확인합니다.
6. Go1을 컨트롤러로 조금 움직입니다.
7. `/amcl_pose`가 갑자기 이동하지 않는지 확인합니다.
8. `map -> camera_init -> body` TF가 유지되는지 확인합니다.

## 19. Nav2 dry-run 확인

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"
```

토픽 확인:

```bash
ros2 topic list |
  grep -E \
  'scan|Odometry|map|amcl_pose|cmd_vel|go1'
```

주파수 확인:

```bash
timeout 10 ros2 topic hz /scan
timeout 10 ros2 topic hz /Odometry
```

지도 확인:

```bash
timeout 10 ros2 topic echo /map --once
```

초기 위치 설정 후 AMCL 확인:

```bash
timeout 10 ros2 topic echo /amcl_pose --once
```

TF 확인:

```bash
timeout 10 ros2 run tf2_ros \
  tf2_echo map camera_init

timeout 10 ros2 run tf2_ros \
  tf2_echo camera_init body
```

Go1 driver가 무구동 상태인지 확인:

```bash
ros2 param get /go1_driver arm
```

예상:

```text
Boolean value is: False
```

Nav2 lifecycle 확인:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

RViz에서 가까운 위치에 `2D Goal Pose`를 지정합니다.

다른 터미널에서 속도 명령을 확인합니다.

```bash
ros2 topic echo /cmd_vel
```

확인할 내용:

```text
Global plan 생성
Local plan 생성
/cmd_vel 생성
Go1은 실제로 움직이지 않음
Goal cancel 후 /cmd_vel이 0으로 복귀
```

## 20. 실제 Go1 저속 주행

다음 조건을 모두 통과하기 전에는 `arm:=true`를 사용하지 않습니다.

- LiDAR가 영구 고정되어 있습니다.
- FAST-LIO extrinsic이 실제 장착값과 일치합니다.
- `/scan`과 지도 벽이 정지 및 이동 중 모두 일치합니다.
- AMCL pose jump가 없습니다.
- `map -> camera_init -> body` TF가 안정적입니다.
- Nav2 global/local costmap이 정상입니다.
- dry-run goal 및 goal cancel을 확인했습니다.
- watchdog이 stale command를 정확한 zero로 변경합니다.
- e-stop 담당자와 시험 통제 구역이 준비됐습니다.

먼저 `arm:=false` launch를 `Ctrl-C`로 완전히 종료합니다.

Go1 driver 중복 확인:

```bash
ros2 node list |
  grep go1_driver || true
```

실제 주행:

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export ROS_DOMAIN_ID=100

export SESSION_ID="실제_SESSION_ID"

export MAP_FILE="/mnt/t500/maps/hanyang_9f/$SESSION_ID/slam_toolbox/hanyang_9f.yaml"

source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source /mnt/t500/go1_sdk/setup_unitree_sdk.bash
source "$GO1_ROS2_WS/install/setup.bash"

test -s "$MAP_FILE"

ros2 launch omx_navigation \
  go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  arm:=true
```

권장 시험 순서:

1. 전방 `0.3 m`
2. 정지 및 goal cancel
3. 작은 제자리 회전
4. 전방 `0.5-1.0 m`
5. 정적 장애물 접근 및 정지
6. 2-3 waypoint 구간
7. 복도 한 구역 반복
8. 승인된 9층 경로

현재 기본 제한:

| 항목 | 제한값 |
|---|---:|
| 최대 전진 속도 | `0.20 m/s` |
| 최대 회전 속도 | `0.40 rad/s` |
| Go1 command watchdog | `0.35 s` |

다음 상황에서는 즉시 시험을 중단합니다.

- AMCL pose jump
- `/scan`과 지도 벽 불일치
- `map -> camera_init` TF 단절
- goal cancel 후 Go1 stepping 지속
- 예상과 다른 전진 또는 회전 방향
- 과도한 흔들림
- costmap에 장애물이 반영되지 않음
- LiDAR, IMU 또는 Odometry timestamp 역행

## 21. 일반 재시작 순서

지도 생성이 완료된 이후의 일반 운영 순서:

```text
1. Livox MID-360 실행
2. FAST-LIO 실행
3. go1_existing_map.launch.py arm:=false
4. RViz 2D Pose Estimate
5. /scan과 지도 정합 확인
6. AMCL 및 TF 확인
7. 가까운 goal dry-run
8. goal cancel 및 watchdog 확인
9. arm:=false launch 완전 종료
10. 안전 승인 후 arm:=true 재시작
```

Livox, FAST-LIO 및 Go1 driver를 중복 실행하지 마십시오.

## 22. 최종 요약

```text
지도 제작:
codex/hanyang-9f-mapping

Nav2 주행용 지도:
<SESSION_DIR>/slam_toolbox/hanyang_9f.yaml

자율주행:
agent/nav2-end-to-end-workflow

사용하지 않을 지도:
<SESSION_DIR>/pcd2d/geometry_reference.yaml

3D 확인용:
<SESSION_DIR>/pcd/merged.pcd
```

현재 시스템은 FAST-LIO의 3D LiDAR·IMU odometry를 사용하고, SLAM Toolbox가 만든 2D 지도를 AMCL과 Nav2에서 사용하는 구조입니다.

객체 인식, 객체 분류 및 객체 추적 기능은 현재 두 브랜치에 포함되어 있지 않습니다.
