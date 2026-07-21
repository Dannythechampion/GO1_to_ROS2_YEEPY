# OMX RViz Navigation

ROS 2 Humble 기준으로, 2D LiDAR 차동구동 로봇을 RViz의 **2D Goal Pose** 도구로
주행시키는 Nav2 패키지입니다. 기본 실행은 SLAM을 켜므로 지도 파일 없이 바로
시작할 수 있습니다.

## 로봇 쪽 인터페이스

이 패키지를 실행하기 전에 로봇 드라이버가 다음 인터페이스를 제공해야 합니다.

| 구분 | 기본값 | 역할 |
|---|---|---|
| Topic | `/scan` (`sensor_msgs/LaserScan`) | 2D LiDAR |
| Topic | `/odom` (`nav_msgs/Odometry`) | 휠 오도메트리 |
| Topic | `/cmd_vel` (`geometry_msgs/Twist`) | 로봇 속도 명령 |
| TF | `odom -> base_link` | 로봇 드라이버/오도메트리 노드가 발행 |
| TF | `base_link -> <laser_frame>` | URDF 또는 static TF가 발행 |

`map -> odom` TF는 mapping 모드에서는 SLAM Toolbox가, 저장 지도 모드에서는
AMCL이 발행합니다.

## 설치와 빌드

Ubuntu 22.04 / ROS 2 Humble에서:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rviz2

mkdir -p ~/omx_ws/src
cp -r /path/to/OMX-AI ~/omx_ws/src/omx_navigation
cd ~/omx_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

이미 이 저장소가 ROS 2 workspace의 `src/` 아래에 있다면 복사하지 않고 해당
workspace 루트에서 `rosdep`과 `colcon build`만 실행하면 됩니다.

## 1. SLAM과 동시에 내비게이션

로봇 드라이버와 `robot_state_publisher`를 먼저 실행한 뒤:

```bash
ros2 launch omx_navigation rviz_navigation.launch.py slam:=true
```

RViz에서 LiDAR, TF, Map이 정상 표시되는지 확인하고 상단의 **2D Goal Pose**를
눌러 지도 위에서 드래그하면 Nav2 목표가 전송됩니다. SLAM 초기 위치는 현재
오도메트리 위치로 잡히므로 `2D Pose Estimate`는 누르지 않습니다.

지도 저장:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/omx_map
```

## 2. 저장한 지도에서 내비게이션

```bash
ros2 launch omx_navigation rviz_navigation.launch.py \
  slam:=false map:=$HOME/maps/omx_map.yaml
```

이 모드에서는 먼저 RViz의 **2D Pose Estimate**로 로봇의 실제 초기 위치와 방향을
지정한 뒤 **2D Goal Pose**를 사용합니다.

## 토픽 또는 프레임 이름 변경

launch 인자로 바꿀 수 있습니다.

```bash
ros2 launch omx_navigation rviz_navigation.launch.py \
  scan_topic:=/lidar/scan \
  odom_topic:=/wheel/odom \
  base_frame:=base_footprint
```

Nav2의 최종 속도 출력은 `/cmd_vel`입니다. 로봇 드라이버가 다른 이름을 사용하면
드라이버 쪽 입력을 `/cmd_vel`로 remap합니다. `map`, `odom` 프레임명도 각각
`map_frame`, `odom_frame` launch 인자로 바꿀 수 있습니다.

추가 인자 확인:

```bash
ros2 launch omx_navigation rviz_navigation.launch.py --show-args
```

## 실제 로봇에 맞춰 먼저 조정할 값

- `config/nav2_params.yaml`의 `robot_radius`를 실제 외접 반지름으로 변경
- `max_vel_x`, `max_vel_theta`, 가감속 제한을 섀시 사양에 맞게 변경
- LiDAR 유효 거리에 맞춰 obstacle/raytrace range 변경
- 시뮬레이터를 사용할 때는 `use_sim_time:=true` 추가

빠른 연결 점검:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link <laser_frame>
```
