# Livox MID-360, FAST-LIO, RViz 실행 절차

이 문서는 Jetson에서 Livox MID-360 데이터를 받아 FAST-LIO ROS2로 처리하고,
RViz에서 포인트클라우드와 오도메트리를 확인하는 전체 순서를 정리한다.

검증 환경:

- Ubuntu 22.04 / ROS2 Humble
- Livox 드라이버 워크스페이스: `~/ws_livox`
- FAST-LIO 워크스페이스: `~/ros2_ws`
- Jetson MID-360 NIC 주소: `192.168.1.5/24`
- MID-360 주소: `192.168.1.138`
- Livox 출력: `/livox/lidar`, `/livox/imu`
- FAST-LIO 출력: `/Odometry`, `/cloud_registered`, `/path`, TF

## 1. 사전 설정 확인

### 1.1 네트워크

```bash
ip -br address
ip route
ping -c 3 192.168.1.138
```

MID-360이 연결된 인터페이스에는 `192.168.1.5/24`가 있어야 한다. 검증한
Jetson에서는 `eno1`에 Go1용 `192.168.123.126/24`와 MID-360용
`192.168.1.5/24`가 함께 설정되어 있다.

Livox 설정의 host IP와 LiDAR IP도 확인한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

LIVOX_CONFIG="$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/MID360_config.json"

grep -nE '"(cmd_data_ip|push_msg_ip|point_data_ip|imu_data_ip|log_data_ip|ip)"' \
  "$LIVOX_CONFIG"
```

기대 값:

```text
cmd_data_ip, push_msg_ip, point_data_ip, imu_data_ip: 192.168.1.5
LiDAR ip:                                             192.168.1.138
```

`bind failed`가 발생하면 JSON의 host IP가 Jetson NIC 주소와 일치하는지 먼저
확인한다. `found lidar not defined in the user-defined config`가 나오면 로그에
표시된 실제 LiDAR IP와 JSON의 LiDAR IP를 일치시킨다.

### 1.2 FAST-LIO 입력 토픽

원본 설정을 확인한다.

```bash
grep -nE 'lid_topic|imu_topic' \
  ~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

다음 값이어야 한다.

```yaml
lid_topic: "/livox/lidar"
imu_topic: "/livox/imu"
```

아직 기본 토픽을 사용한다면 백업 후 수정한다.

```bash
FASTLIO_CONFIG=~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
cp "$FASTLIO_CONFIG" "${FASTLIO_CONFIG}.bak"

sed -i \
  -e 's#/livox/custom_points#/livox/lidar#g' \
  -e 's#/imu/data#/livox/imu#g' \
  "$FASTLIO_CONFIG"
```

수정 후 FAST-LIO를 다시 빌드한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

colcon build --symlink-install --packages-select fast_lio
source ~/ros2_ws/install/setup.bash
```

## 2. 실행 순서

각 노드는 별도 터미널에서 실행한다. 새 터미널마다 해당 환경을 다시
`source`해야 한다.

### 터미널 1: Livox MID-360

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 터미널 2: Livox 입력 검증

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

timeout 10 ros2 topic hz /livox/lidar
timeout 10 ros2 topic hz /livox/imu
```

정상 기준:

```text
/livox/lidar: 약 10 Hz
/livox/imu:   약 200 Hz
```

### 터미널 3: FAST-LIO

IMU 초기화 동안 Go1을 평평한 곳에 정지시킨다. SSH 세션에서는 RViz를 끄고
FAST-LIO만 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:=false
```

`rviz` 인자 지원 여부는 다음 명령으로 확인할 수 있다.

```bash
ros2 launch fast_lio mapping.launch.py --show-args
```

### 터미널 4: FAST-LIO 출력 검증

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node info /laser_mapping
timeout 10 ros2 topic hz /Odometry
timeout 10 ros2 topic hz /cloud_registered
ros2 topic echo /Odometry --once
ros2 run tf2_ros tf2_echo camera_init body
```

정상 기준:

- `/laser_mapping`이 `/livox/lidar`와 `/livox/imu`를 구독한다.
- `/Odometry`와 `/cloud_registered`가 약 10 Hz로 발행된다.
- `/Odometry`의 `frame_id`는 `camera_init`, `child_frame_id`는 `body`이다.
- `camera_init -> body` TF가 계속 갱신된다.
- 정지 상태에서 위치와 자세가 급격히 튀지 않는다.

TF 실행 직후 한 번 나타나는 `Invalid frame ID`는 첫 TF가 도착하기 전의
일시적인 메시지일 수 있다. 이후 변환 값이 연속 출력되면 정상이다.

## 3. RViz 실행

### 3.1 Jetson 데스크톱에서 실행

Jetson에 연결된 화면의 데스크톱 터미널에서 실행하는 방법이 가장 간단하다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

rviz2 -d \
  "$(ros2 pkg prefix fast_lio)/share/fast_lio/rviz/fastlio.rviz"
```

### 3.2 X11 forwarding으로 실행

X11을 지원하는 클라이언트에서 Jetson에 접속한다.

```bash
ssh -Y unicon@192.168.0.138
echo "$DISPLAY"
```

`localhost:10.0`과 같은 값이 나온다면 위의 `rviz2` 명령을 실행한다.
`DISPLAY`가 비어 있으면 그래픽 전달이 설정되지 않은 것이므로 Jetson
데스크톱, 원격 데스크톱 또는 X 서버를 사용한다.

SSH 세션에서 `qt.qpa.xcb: could not connect to display`가 발생해도 FAST-LIO
노드 자체의 장애는 아니다. 화면 환경이 없는 세션에서 RViz만 실패한 것이다.

### 3.3 RViz 표시 설정

제공된 RViz 설정이 적용되지 않으면 다음 항목을 직접 설정한다.

| 항목 | 값 |
|---|---|
| Global Options / Fixed Frame | `camera_init` |
| PointCloud2 / Topic | `/cloud_registered` |
| Odometry / Topic | `/Odometry` |
| Path / Topic | `/path` |
| TF | 활성화 |

포인트클라우드가 화면 중앙에 매우 작게 보이면 데이터 오류가 아니라 카메라가
멀리 줌아웃된 상태일 수 있다. `Focus Camera`로 점군을 클릭한 뒤 마우스 휠로
확대한다. 포인트가 작으면 `CloudRegistered`의 `Size (Pixels)`를 3~4로
설정한다.

정상 화면에서는 `Global Status: Ok`가 표시되고, 주변 벽과 바닥의 컬러
포인트클라우드 및 이동 궤적이 보인다. 원격 렌더링의 낮은 FPS는 FAST-LIO의
센서 처리 주파수와 별개다.

## 4. 종료 순서

각 실행 터미널에서 `Ctrl+C`를 사용한다.

1. RViz
2. FAST-LIO
3. Livox 드라이버

강제 종료 후 다시 시작할 때는 중복 Livox 프로세스가 UDP 포트를 점유하지
않는지 확인한다.

```bash
pgrep -af livox
```

## 5. 현재 단계의 범위

이 절차는 FAST-LIO의 신규 mapping과 odometry 생성을 검증한다. 기존
`scans.pcd`에서 재위치 추정하는 localization과 Nav2의 `map -> odom` 연결은
별도 단계이며, 이 문서의 실행만으로 기존 지도 기반 자동주행이 활성화되지는
않는다.
