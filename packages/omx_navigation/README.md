# Go1 ROS2 Humble 기존 지도 내비게이션

이 패키지는 Livox MID-360과 FAST-LIO2가 이미 실행 중인 Jetson에서 기존
2D 지도로 AMCL 로컬라이제이션과 Nav2 경로계획을 시험합니다. 기본 실행은
`arm:=false`이므로 `/cmd_vel_nav`가 생성되어도 실제 Go1에는 동작 명령을
전송하지 않습니다. Existing-map launch에서는 AMCL supervisor, motion gate와
fixed/RViz mission manager가 함께 실행됩니다. 실제 start/destination 좌표가
없으면 관련 기능은 fail-closed 상태로 유지됩니다.

## 현재 시험 프레임

LiDAR가 아직 영구 고정되지 않았기 때문에 최종 `base_link -> lidar` TF를
만들지 않습니다. 현재 FAST-LIO가 제공하는 프레임을 임시로 사용합니다.

```text
map -> camera_init -> body
```

- `camera_init`: Nav2 odom 프레임
- `body`: Nav2 base 프레임과 임시 LaserScan 프레임
- `/Odometry`: FAST-LIO odometry
- `/cloud_registered_body`: LaserScan 변환 입력
- `/scan`: AMCL과 costmap 입력

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

### 3. 기존 지도 AMCL, Nav2, 저부하 RViz

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch omx_navigation go1_existing_map.launch.py arm:=false
```

통합 launch는 다음을 시작합니다.

- `/cloud_registered_body`를 `/scan`으로 변환
- 기존 지도 map server
- AMCL
- Nav2 planner, controller, behavior 및 velocity smoother
- Go1 드라이버의 무구동 모드
- 15 FPS 저부하 RViz

## RViz 시험

1. 기존 지도와 붉은색 `/scan`이 보이는지 확인합니다.
2. `2D Pose Estimate`로 지도상의 실제 위치와 방향을 지정합니다.
3. `/scan`이 기존 지도 벽과 겹치는지 확인합니다.
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
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_safe
```

자동 무구동 검증은 다음 단계에서 실행합니다.

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_existing_map_navigation.sh
```

## 안전 제한

- 이번 단계에서는 `arm:=true`를 사용하지 않습니다.
- LiDAR가 고정되지 않았으므로 지도와 scan 정합은 임시 시험 결과입니다.
- 기존 지도와 실시간 scan이 맞지 않으면 새 지도를 작성해야 합니다.
- 실제 주행은 고정 TF·extrinsic, 정지 watchdog, 초기 자세와 좁은 복도
  검증을 모두 마친 뒤 별도 단계에서 진행합니다.
