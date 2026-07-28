# Go1 + MID-360 + FAST-LIO + Nav2 전체 실행 가이드

이 문서는 새 장소의 2D 지도를 만든 뒤 저장 지도에서 AMCL로 위치를 추정하고,
Nav2가 생성한 `/cmd_vel`로 Go1을 주행시키는 전체 순서를 한곳에 정리한다.

```text
매핑: Livox -> FAST-LIO -> /cloud_registered_body -> /scan
      -> SLAM Toolbox -> map.yaml + map.pgm

주행: Livox -> FAST-LIO -> /cloud_registered_body -> /scan
      -> Map Server + AMCL -> Nav2 -> /cmd_vel -> Go1 driver
```

현재 프레임 계약은 LiDAR 장착이 확정되기 전의 시험 구성이다.

```text
map -> camera_init -> body
```

- `camera_init`: FAST-LIO odometry/Nav2 odom frame
- `body`: Nav2 base frame과 임시 LaserScan frame
- LiDAR 고정 후에는 실제 `base_link -> lidar` TF와 FAST-LIO extrinsic을 측정값으로
  교체해야 한다.

## 1. 브랜치 받기

새로 clone할 때:

```bash
mkdir -p /mnt/t500
cd /mnt/t500
git clone --branch agent/nav2-end-to-end-workflow --single-branch \
  https://github.com/Dannythechampion/GO1_to_ROS2_YEEPY.git \
  go1_ros2_project
cd /mnt/t500/go1_ros2_project
```

저장소가 이미 있을 때:

```bash
cd /mnt/t500/go1_ros2_project
git fetch origin
git switch agent/nav2-end-to-end-workflow
git pull --ff-only
```

현재 Jetson의 기존 `~/ros2_ws`를 유지한다면 아래 환경값을 모든 터미널에서
사용한다.

```bash
export GO1_ROS2_WS="$HOME/ros2_ws"
export GO1_MAP_ROOT="$HOME/maps"
export ROS_DOMAIN_ID=100
```

저장소 표준 배치 `/mnt/t500`을 사용하면 `GO1_ROS2_WS`와 `GO1_MAP_ROOT`는
생략할 수 있다.

## 2. 패키지 배치와 빌드

새 `/mnt/t500/go1_ros2_ws`라면:

```bash
cd /mnt/t500/go1_ros2_project
chmod +x migration/*.sh
./migration/bootstrap_ros2_humble.sh
./migration/import_ros2_dependencies.sh
./migration/stage_local_ros2_packages.sh
./migration/install_livox_sdk2.sh
./migration/build_livox_fastlio.sh
./migration/build_unitree_go1_wrapper.sh \
  /mnt/t500/go1_project_data/catkin_ws/src/unitree_legged_sdk

cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
```

이미 사용 중인 `~/ros2_ws`에 이 브랜치의 로컬 패키지만 반영할 때:

```bash
cd /path/to/GO1_to_ROS2_YEEPY
mkdir -p "$HOME/ros2_ws/src"
cp -a packages/go1_driver "$HOME/ros2_ws/src/"
cp -a packages/omx_navigation "$HOME/ros2_ws/src/"

cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
```

## 3. 하드웨어 및 토픽 Gate

사용자 확인값 기준 예시는 Jetson 유선 주소 `192.168.1.5/24`, MID-360 주소
`192.168.1.138`이다. 실제 장비 주소가 다르면 Livox JSON을 먼저 수정한다.

```bash
ip -br address
ip route
ping -c 3 192.168.1.138

LIVOX_CONFIG="$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/MID360_config.json"
grep -nE '"(cmd_data_ip|push_msg_ip|point_data_ip|imu_data_ip|ip)"' \
  "$LIVOX_CONFIG"
```

JSON의 host data IP는 MID-360에 연결된 Jetson NIC 주소와 같아야 하고, LiDAR
IP는 실제 MID-360 주소와 같아야 한다. Livox 노드가 여러 번 실행되어 UDP port가
중복 bind되지 않도록 한다.

## 4. 공통 터미널 환경

아래 블록을 새 터미널마다 먼저 실행한다. 표준 배치에서는 첫 두 `export`가 필요
없다.

```bash
export GO1_ROS2_WS="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
export GO1_MAP_ROOT="${GO1_MAP_ROOT:-/mnt/t500/maps}"
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"
```

현재처럼 Livox가 별도 `~/ws_livox`에 설치되어 있다면 마지막 source 전에 다음을
추가한다.

```bash
source "$HOME/ws_livox/install/setup.bash"
```

## 5. 새 장소 매핑

### 터미널 A: Livox MID-360

```bash
export GO1_ROS2_WS="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
test -r "$HOME/ws_livox/install/setup.bash" && \
  source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 터미널 B: FAST-LIO

로봇을 움직이지 않은 상태로 IMU 초기화를 기다린다.

```bash
export GO1_ROS2_WS="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
test -r "$HOME/ws_livox/install/setup.bash" && \
  source "$HOME/ws_livox/install/setup.bash"
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch fast_lio mapping.launch.py \
  config_path:="$GO1_ROS2_WS/install/omx_navigation/share/omx_navigation/config" \
  config_file:=fast_lio_mid360_navigation.yaml \
  rviz:=false
```

### 터미널 C: `/scan`, SLAM Toolbox, RViz

Unitree 전용 컨트롤러로 수동 매핑하므로 Go1 ROS driver는 기본적으로 시작하지
않는다.

```bash
export GO1_ROS2_WS="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch omx_navigation go1_mapping.launch.py \
  start_go1_driver:=false \
  rviz:=true
```

Jetson이 headless이거나 X11 연결이 없다면 `rviz:=false`로 실행한다. RViz가 필요한
경우 `ssh -Y`로 다시 접속하거나 ROS_DOMAIN_ID가 같은 GUI PC에서 RViz를 실행한다.

### 터미널 D: 매핑 파이프라인 확인

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_mapping_pipeline.sh
```

기존 `~/ros2_ws`를 사용하면:

```bash
cd /path/to/GO1_to_ROS2_YEEPY
GO1_ROS2_WS="$HOME/ros2_ws" ./migration/verify_mapping_pipeline.sh
```

필수 관찰값:

```text
/livox/lidar                 약 10 Hz
/livox/imu                   약 200 Hz
/cloud_registered_body       약 10 Hz
/scan                        약 10 Hz
/Odometry                    약 10 Hz
TF camera_init -> body       존재
TF map -> camera_init        존재
```

### 매핑 주행 방법

1. 9층 외곽의 큰 loop를 먼저 한 바퀴 돈다.
2. 복도와 내부 구역을 작은 loop로 연결한다.
3. 같은 복도를 양방향으로 통과하고 출발점으로 돌아와 loop closure를 확인한다.
4. 초기 속도는 약 `0.10 m/s`, yaw는 `0.20 rad/s` 이하로 유지한다.
5. 사람 밀집 시간, 유리벽만 보이는 구간, 센서가 가려지는 빠른 회전을 피한다.

### 지도 저장

기존 지도에 덮어쓰지 말고 매번 새 버전 폴더를 사용한다.

```bash
cd /mnt/t500/go1_ros2_project
./migration/save_nav2_map.sh \
  /mnt/t500/maps/floor9_v001/go1_map
```

기존 `~/ros2_ws` 및 `~/maps`를 사용하면:

```bash
cd /path/to/GO1_to_ROS2_YEEPY
GO1_ROS2_WS="$HOME/ros2_ws" GO1_MAP_ROOT="$HOME/maps" \
  ./migration/save_nav2_map.sh "$HOME/maps/floor9_v001/go1_map"
```

생성 결과:

```text
/mnt/t500/maps/floor9_v001/go1_map.yaml
/mnt/t500/maps/floor9_v001/go1_map.pgm
```

지도에 끊긴 벽, 겹친 복도, 큰 이중상, 실제 비율 오류가 있으면 저장본을 승인하지
말고 새 버전으로 다시 매핑한다.

## 6. 저장 지도 AMCL + Nav2 dry-run

매핑 launch를 종료한다. Livox와 FAST-LIO는 각각 한 인스턴스만 유지한다. 새 터미널
C에서 다음을 실행한다.

```bash
export GO1_ROS2_WS="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
export GO1_MAP_ROOT="${GO1_MAP_ROOT:-/mnt/t500/maps}"
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$GO1_MAP_ROOT/floor9_v001/go1_map.yaml" \
  arm:=false
```

RViz에서 다음 순서로 확인한다.

1. Fixed Frame을 `map`으로 둔다.
2. `2D Pose Estimate`로 실제 위치와 방향을 지정한다.
3. `/scan`이 지도 벽과 겹치는지 확인한다.
4. Go1을 컨트롤러로 조금 움직여 `/amcl_pose`가 연속적인지 확인한다.
5. 가까운 곳에 `2D Goal Pose`를 지정한다.
6. Global/Local plan 및 `/cmd_vel`이 생성되는지 확인한다.
7. `arm:=false`이므로 실제 Go1이 움직이지 않는지 확인한다.

검증:

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_existing_map_navigation.sh preflight

# 2D Pose Estimate 후
./migration/verify_existing_map_navigation.sh localized
```

추가 관찰:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /go1/cmd_vel_applied
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init body
ros2 param get /go1_driver arm
```

## 7. 실제 Go1 저속 Nav2 주행

다음을 모두 통과하기 전에는 `arm:=true`를 사용하지 않는다.

- LiDAR가 영구 고정되어 있고 TF/extrinsic이 실제 장착값과 일치한다.
- `/scan`과 지도 벽이 정지 및 이동 중 모두 일치한다.
- AMCL pose jump 및 `map -> camera_init` TF 단절이 없다.
- footprint와 inflation radius가 실제 Go1 외곽보다 작지 않다.
- dry-run goal, cancel, watchdog zero를 확인했다.
- e-stop 담당자와 비상 정지 공간을 확보했다.

먼저 `arm:=false` launch를 `Ctrl-C`로 완전히 종료하고 driver가 하나뿐인지 확인한다.

```bash
ros2 node list | grep go1_driver || true
```

그 다음 제한된 시험 구역에서 실행한다.

```bash
export GO1_ROS2_WS="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
export GO1_MAP_ROOT="${GO1_MAP_ROOT:-/mnt/t500/maps}"
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_sdk/setup_unitree_sdk.bash
source "$GO1_ROS2_WS/install/setup.bash"

ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$GO1_MAP_ROOT/floor9_v001/go1_map.yaml" \
  arm:=true
```

시험 순서:

1. 전방 `0.3 m`
2. 작은 제자리 회전
3. 전방 `0.5-1.0 m`
4. 정적 장애물 접근과 정지
5. 2-3 waypoint
6. 복도 한 구역 반복
7. 승인된 9층 경로

현재 제한은 `max_vel_x=0.20 m/s`, `max_vel_theta=0.40 rad/s`, Go1 driver
watchdog `0.35 s`이다. 목표 취소나 timeout 후에도 stepping이 계속되거나 예상과 다른
방향으로 움직이면 즉시 시험을 중단한다.

## 8. 재시작 순서

지도 생성이 끝난 뒤의 일반 운영 재시작 순서는 다음과 같다.

1. MID-360
2. FAST-LIO (`fast_lio_mid360_navigation.yaml`, `rviz:=false`)
3. `go1_existing_map.launch.py arm:=false`
4. RViz 2D Pose Estimate
5. `verify_existing_map_navigation.sh localized`
6. 가까운 goal dry-run
7. 안전 승인 후에만 `arm:=true`로 재시작

동일 노드를 두 번 실행하지 않는다. 특히 Livox UDP bind 실패와 Go1 driver 중복은
기존 프로세스를 먼저 종료한 뒤 해결한다.

## 9. 지도 및 로그 관리

지도는 용량과 현장 데이터 특성 때문에 Git에 직접 넣지 않고 Jetson의
`/mnt/t500/maps/<site>_vNNN/`에 보관한다. Git에는 launch, parameter, 검증 스크립트,
지도 버전 이름과 시험 결과만 기록한다.

권장 운영 로그:

```text
/livox/lidar, /livox/imu, /Odometry, /cloud_registered_body, /scan
/map, /amcl_pose, /path, /local_plan, /cmd_vel
/go1/cmd_vel_applied, /go1/control_state, /tf, /tf_static
```

각 지도 버전에 실제 장소, 생성일, resolution, origin, 지도 QA 결과, localization
반복 결과, 승인 경로를 함께 기록한다.
