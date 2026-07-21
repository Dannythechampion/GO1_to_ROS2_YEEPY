# Ubuntu 22.04 Jetson 실행 Runbook

이 문서는 Ubuntu 20.04/ROS1 Go1 구성을 Ubuntu 22.04/ROS2 Humble Jetson으로
옮길 때 실제로 실행할 순서다. 각 Gate가 통과하기 전에는 다음 단계로 넘어가지
않는다.

기본 경로:

```text
프로젝트:          ~/projects/go1_ros2_project
ROS2 workspace:    ~/go1_ros2_ws
Unitree SDK:       ~/go1_sdk/unitree_legged_sdk
Livox SDK2:        ~/go1_third_party/Livox-SDK2
```

## 0. 프로젝트 가져오기

권장 방법은 비공개 Git 저장소를 clone하는 것이다.

```bash
mkdir -p ~/projects
cd ~/projects
git clone <PRIVATE_GIT_URL> go1_ros2_project
cd ~/projects/go1_ros2_project
```

Git 원격 저장소가 아직 없다면 PC에서 임시로 프로젝트를 전송할 수 있지만,
최종 구성은 Git clone으로 재현되어야 한다.

## 1. 새 Jetson 기준선 확인

```bash
cd ~/projects/go1_ros2_project
chmod +x migration/*.sh
./migration/audit_new_jetson.sh
```

Gate 1:

```bash
grep -E 'VERSION_ID|VERSION_CODENAME|aarch64|Python 3' \
  ~/migration_audit/new_jetson_system.txt
```

필수 조건:

```text
Ubuntu 22.04 / jammy
aarch64
충분한 디스크 여유 공간
```

## 2. ROS2 Humble 설치

```bash
cd ~/projects/go1_ros2_project
./migration/bootstrap_ros2_humble.sh
source ~/go1_ros2_env.bash
```

Gate 2:

```bash
test "$ROS_DISTRO" = humble
ros2 --help >/dev/null
colcon --help >/dev/null
vcs --help >/dev/null
```

## 3. 고정된 외부 ROS2 소스 가져오기

`~/go1_ros2_ws/src`가 비어 있을 때 먼저 실행한다.

```bash
cd ~/projects/go1_ros2_project
./migration/import_ros2_dependencies.sh
```

가져오는 빌드 대상:

```text
FAST_LIO_ROS2  2fffc570a25d0df172720bac034fbdb6a13d2162
livox_ros_driver2 13eb05e4e6dd7a765b934d0c5fd6236676a57b49
```

Gate 3:

```bash
vcs status ~/go1_ros2_ws/src
git -C ~/go1_ros2_ws/src/FAST_LIO_ROS2 rev-parse HEAD
git -C ~/go1_ros2_ws/src/livox_ros_driver2 rev-parse HEAD
```

## 4. Livox-SDK2 설치

```bash
cd ~/projects/go1_ros2_project
./migration/install_livox_sdk2.sh
```

Gate 4:

```bash
test -f /usr/local/lib/liblivox_lidar_sdk_static.a
git -C ~/go1_third_party/Livox-SDK2 rev-parse HEAD
```

예상 SDK2 커밋:

```text
f5d9375f84efe2b15bc0a052d3e18482ed13adf4
```

## 5. 자체 ROS2 패키지 배치

외부 의존성을 import한 다음 실행한다.

```bash
cd ~/projects/go1_ros2_project
./migration/stage_local_ros2_packages.sh
```

배치되는 패키지:

```text
~/go1_ros2_ws/src/go1_driver
~/go1_ros2_ws/src/omx_navigation
```

기존 `GO-_project_data/catkin_ws/src`는 ROS2 workspace에 복사하지 않는다.

## 6. Livox ROS2 및 FAST-LIO 빌드

```bash
cd ~/projects/go1_ros2_project
./migration/build_livox_fastlio.sh
```

Gate 6:

```bash
source ~/go1_ros2_ws/install/setup.bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
```

아직 LiDAR를 실행하지 않는다. 먼저 MID-360 NIC와 JSON/YAML IP를 실제 장비
값으로 수정한다.

## 7. Unitree Go1 Python wrapper 재빌드

현재 프로젝트에 보존된 실제 ROS1 SDK 스냅샷을 사용한다.

```bash
cd ~/projects/go1_ros2_project
./migration/build_unitree_go1_wrapper.sh \
  "$HOME/projects/go1_project_data/catkin_ws/src/unitree_legged_sdk"
source ~/go1_sdk/setup_unitree_sdk.bash
```

Gate 7:

```bash
python3 - <<'PY'
import robot_interface as sdk
print(sdk.__file__)
print(type(sdk.HighCmd()))
print(type(sdk.HighState()))
PY
```

이 단계에서는 `sdk.UDP(...)`를 생성하지 않고 import만 검사한다.

## 8. 자체 ROS2 패키지 빌드

```bash
cd ~/go1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
```

Gate 8:

```bash
ros2 pkg prefix go1_driver
ros2 pkg prefix omx_navigation
colcon test --packages-select go1_driver
colcon test-result --verbose
```

## 9. Go1 driver 자동 dry-run

Go1 Ethernet을 연결하지 않은 상태에서도 실행할 수 있다.

```bash
cd ~/projects/go1_ros2_project
./migration/verify_go1_driver_dry_run.sh
```

필수 PASS:

```text
diagonal direction preserved and limited to 0.20 m/s
watchdog changed stale command to exact zero
driver reported DRY-RUN mode
```

이 Gate 전에는 절대 `arm:=true`를 사용하지 않는다.

## 10. 물리 네트워크 연결

권장 분리:

```text
Wi-Fi:             SSH, 192.168.0.138
내장 Ethernet:     Go1, 192.168.123.126/24
USB Ethernet:      MID-360, 192.168.1.5/24
```

확인:

```bash
ip -br address
ip route
ping -c 3 192.168.123.161
ping -c 3 192.168.1.148
```

실제 Go1/MID-360 주소가 다르면 기존 설정과 장비 화면에서 확인한 값으로
바꾼다.

## 11. MID-360 단독 검증

```bash
source ~/go1_ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

다른 터미널:

```bash
source ~/go1_ros2_ws/install/setup.bash
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

Gate 11: 두 토픽이 연속 발행되고 CustomMsg/timestamp가 FAST-LIO 입력과
일치해야 한다.

## 12. FAST-LIO 단독 검증

로봇을 정지한 채 IMU 초기화를 수행한다.

```bash
source ~/go1_ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic echo /Odometry --once
```

Gate 12: 정지 drift가 급격하지 않고, 천천히 움직일 때 point cloud 정합과
odometry가 연속적이어야 한다.

## 13. TF 및 Nav2 dry-run

다음 chain을 확정한다.

```text
map -> odom -> base_link -> lidar
```

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar
```

Go1 driver는 계속 disarmed로 실행한다.

```bash
ros2 launch go1_driver go1_driver.launch.py arm:=false
```

Nav2 goal을 보낸 후 다음을 확인한다.

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /go1/cmd_vel_applied
```

## 14. 실제 Go1 저속 시험

다음 조건을 모두 만족한 후에만 진행한다.

- dry-run 자동 검증 통과
- Go1 ping 정상
- 리모컨과 비상 정지 준비
- 로봇 지지 또는 충분한 안전 공간 확보
- 속도 제한 `0.20 m/s`, yaw `0.40 rad/s` 이하

```bash
source ~/go1_sdk/setup_unitree_sdk.bash
source ~/go1_ros2_ws/install/setup.bash
ros2 launch go1_driver go1_driver.launch.py arm:=true
```

시험 순서:

```text
zero -> forward -> lateral -> diagonal -> yaw -> curved motion
```

zero 명령 또는 command timeout 뒤에도 stepping이 계속되면 즉시 중단한다.

## 15. 완료 증거 저장

각 Gate 결과를 저장한다.

```text
SYSTEM_INFO.md
NETWORK.md
TF_CONTRACT.md
CALIBRATION.md
TEST_RESULTS.md
ros2.repos
```

최종 완료는 다른 Ubuntu 22.04 Jetson에서 같은 Git 커밋을 clone하고 위 순서로
다시 빌드할 수 있을 때 인정한다.
