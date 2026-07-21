# Go1 Jetson ROS2 전환 계획

## 1. 목적

기존 ROS1 Noetic 기반 Jetson에서 사용하던 Unitree Go1, Livox MID-360,
FAST-LIO, 위치 추정, 내비게이션 및 Go1 속도 제어 구성을 새 ROS2 기반
Jetson으로 이전한다.

이 문서의 기본 목표 파이프라인은 다음과 같다.

```text
Livox MID-360
  -> livox_ros_driver2 (ROS2)
  -> FAST_LIO_ROS2
  -> Odometry + PointCloud + TF
  -> Nav2
  -> /cmd_vel
  -> Go1 ROS2 safety bridge
  -> unitree_legged_sdk
  -> Go1
```

> 중요: 기존 `GO-_project_data/catkin_ws`는 삭제하거나 ROS2 소스로 덮어쓰지
> 않는다. 기존 로봇 설정과 수정 사항을 확인하기 위한 읽기 전용 기준 자료로
> 보존한다.

---

## 2. 전환 원칙

1. ROS1 `catkin_ws`와 ROS2 `colcon` 워크스페이스를 분리한다.
2. 기존 ROS1 패키지를 한꺼번에 ROS2 워크스페이스로 복사하지 않는다.
3. 각 기능을 ROS2 대응 패키지로 교체하거나 필요한 로봇 전용 코드만 포팅한다.
4. IP, 토픽, frame ID, extrinsic, 지도와 튜닝 값은 기존 자료에서 선별하여 옮긴다.
5. 센서, LIO, TF, 제어, Nav2 순서로 단계별 검증한다.
6. 실제 Go1 출력은 모든 dry-run 검증이 끝날 때까지 비활성화한다.
7. 외부 저장소는 `main` 브랜치만 따라가지 말고 검증한 커밋을 기록하여 고정한다.

권장 디렉터리 구조:

```text
/mnt/t500/go1_project_data/     # 기존 ROS1 자료, 읽기 전용
└── catkin_ws/src/

/mnt/t500/go1_ros2_ws/          # 새 ROS2 워크스페이스
├── src/
│   ├── livox_ros_driver2/
│   ├── FAST_LIO_ROS2/
│   ├── go1_control/
│   ├── go1_bringup/
│   ├── go1_description/
│   └── omx_navigation/
├── build/                      # 생성물, 필요하면 삭제 후 재빌드 가능
├── install/                    # 생성물, 필요하면 삭제 후 재빌드 가능
└── log/                        # 생성물
```

현재 로컬 상위 폴더는 ROS2 코드와 `GO-_project_data`, `GO1-project`의 nested
Git 저장소가 섞여 있으므로 그 상위 폴더에서 바로 `git init`하지 않는다.
ROS2 배포용 파일만 새 디렉터리로 내보낸다.

Windows PowerShell에서 WSL을 이용한 예:

```powershell
wsl bash "/mnt/c/Users/kimgk/OneDrive/문서/OMX-AI/migration/create_ros2_git_export.sh" `
  "/mnt/c/Users/kimgk/OneDrive/문서/go1_ros2_project"
```

내보내기 결과에는 ROS2 패키지, migration 스크립트, 문서와 고정 의존성만
포함되며 ROS1 아카이브 및 nested `.git`은 포함되지 않는다. 결과 디렉터리는
독립 Git working tree로 초기화되고 파일이 staging된 상태이므로 검토 후 별도
비공개 원격 저장소에 commit/push한다.

---

## 3. 먼저 확정할 시스템 조합

새 Jetson에는 ROS2 Humble Desktop이 이미 설치되어 있다. 재설치하지 말고 먼저
다음 결과와 기존 설치 상태를 기록한 뒤, 누락된 개발 도구와 런타임 의존성만
보완한다.

```bash
uname -m
cat /etc/os-release
python3 --version
python3-config --extension-suffix
nvcc --version || true
dpkg -l | grep -E 'nvidia-jetpack|ros-'
```

확인 항목:

- Jetson 모델
- JetPack/L4T 버전
- Ubuntu 버전
- ROS2 배포판
- Python 버전과 ABI
- ARM64(`aarch64`) 여부
- CUDA 버전

이 문서의 기본 예시는 Ubuntu 22.04, ROS2 Humble, Python 3.10,
ARM64를 가정한다. 실제 Jetson이 Ubuntu 20.04 또는 다른 ROS2 배포판이라면
패키지 이름과 Python ABI를 해당 환경에 맞춘다.

저장소의 진단 스크립트를 새 Jetson으로 복사하거나 clone한 뒤 실행하면 위
정보가 한 파일에 저장된다.

```bash
chmod +x migration/audit_new_jetson.sh
./migration/audit_new_jetson.sh
```

기본 결과 경로:

```text
/mnt/t500/migration_audit/new_jetson_system.txt
```

기존 ROS1 Jetson에도 접근할 수 있다면 다음 스크립트로 패키지와 중요 데이터
목록을 수집한다.

```bash
chmod +x migration/audit_old_ros1_jetson.sh
./migration/audit_old_ros1_jetson.sh
```

기존 catkin workspace가 `/mnt/t500/go1_project_data/catkin_ws`가 아니라면:

```bash
CATKIN_WS=/mnt/t500/go1_project_data/catkin_ws \
  ./migration/audit_old_ros1_jetson.sh
```

ROS2 환경을 자동으로 `.bashrc`에서 불러오기 전에는 ROS1과 ROS2 setup 파일을
동시에 source하지 않는다.

---

## 4. 기존 ROS1 자료에서 보존할 항목

다음 항목은 ROS2에서 형식을 확인한 뒤 재사용한다.

- MID-360 LiDAR IP와 Jetson NIC IP
- `/livox/lidar`, `/livox/imu`의 실제 토픽 설정
- LiDAR-IMU extrinsic translation/rotation
- LiDAR-Go1 base 사이의 측정된 static transform
- FAST-LIO voxel, blind distance, timestamp, filter 및 PCD 저장 파라미터
- PCD, PGM, YAML 지도
- RViz 시각화 설정
- Go1 UDP 주소와 포트
- `cmd_vel` 최대 속도, deadband, watchdog timeout
- lateral 축 부호 반전 여부
- 기존 Jetson 전용 패키지 수정 내용

기존 저장소의 소스 기준점과 수정 사항은 다음 파일을 기준으로 확인한다.

```text
GO-_project_data/catkin_ws/UPSTREAMS.md
```

기존 `build`, `devel`, ROS1 바이너리와 ROS1용 `.launch` 파일은 ROS2에서 그대로
사용하지 않는다.

---

## 5. ROS1 패키지의 ROS2 처리 방침

| 기존 구성 | 상태 | ROS2 처리 방침 |
|---|---|---|
| `FAST_LIO` | 교체 | 고정 커밋의 `Ericsii/FAST_LIO_ROS2` 사용 |
| `livox_ros_driver2` | 재빌드 | ROS2/Humble 모드로 새로 빌드 |
| `FAST_LIO_LOCALIZATION` | 보류 | ROS2 저장소와 커밋 고정 및 실기 검증 전까지 미포함 |
| `sentry_nav`, `move_base` | 일부 대체 | `omx_navigation`과 Nav2로 필요한 기능만 구성 |
| `velocity_smoother_ema` | 교체 예정 | `nav2_velocity_smoother` 적용 후 동작 검증 |
| `unitree_ros_to_real` | 교체 | 안전 필터를 포함한 새 `go1_driver` 사용 |
| `go1_imu_pub` | 확인 필요 | `go1_driver` 상태/IMU 통합 여부 확인 후 포팅 결정 |
| `stereo_split` | 조건부 보류 | 카메라 사용이 확정될 때만 ROS2 `image_transport` 기반 포팅 |
| `vision_opencv` | 불필요 | 소스 복사 없이 Humble 배포판의 `cv_bridge` 사용 |
| `orb_slam3_ros` | 보류 | 현재 마이그레이션 범위에서 제외 |
| `pcd2pgm_package` | 보류 | ROS2 적용 방법 검증 전까지 미포함 |
| `unitree_legged_sdk` | 재빌드 | 아카이브 v3.8.6 전체를 ARM64/Python 3.10용으로 재빌드 |
| PCD/PGM/YAML/RViz | 데이터 이전 | 데이터와 튜닝 값은 보존하고 경로와 형식만 조정 |

`보류`와 `확인 필요` 항목은 현재 colcon 빌드 및 Jetson 배포 범위에 포함하지
않는다.

---

## 6. ROS2 워크스페이스 생성

```bash
mkdir -p /mnt/t500/go1_ros2_ws/src
cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
```

기본 개발 도구:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

`rosdep`이 아직 초기화되지 않은 시스템에서만 다음을 실행한다.

```bash
sudo rosdep init
rosdep update
```

이 저장소에는 Ubuntu 22.04 ARM64인지 먼저 확인한 뒤 ROS2 Humble과 공통
의존성을 설치하는 스크립트가 포함되어 있다. JetPack 핵심 패키지를 임의로
전체 업그레이드하지 않도록 `apt upgrade`는 자동 실행하지 않는다.

```bash
chmod +x migration/bootstrap_ros2_humble.sh
./migration/bootstrap_ros2_humble.sh
```

외부 ROS2 소스는 검증 기준 커밋을 기록한 `migration/ros2.repos`로 가져온다.

```bash
chmod +x migration/import_ros2_dependencies.sh
./migration/import_ros2_dependencies.sh
```

외부 의존성을 먼저 가져온 다음 이 저장소의 ROS2 패키지만 선별 배치한다.

```bash
chmod +x migration/stage_local_ros2_packages.sh
./migration/stage_local_ros2_packages.sh
```

현재 고정 기준:

| 저장소 | 브랜치 기준 | 고정 커밋 |
|---|---|---|
| `Ericsii/FAST_LIO_ROS2` | `ros2` | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| `Livox-SDK/livox_ros_driver2` | `master` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |

`unitree_ros2_to_real`은 Eloquent/v3.5.1 기준이라 이 자동 빌드 목록에 넣지
않는다. 포팅 비교용 커밋은 `migration/legacy_reference.repos`에 별도로 기록한다.

---

## 7. Livox MID-360 ROS2 드라이버

저장소:

- <https://github.com/Livox-SDK/livox_ros_driver2.git>

설치 예시:

```bash
cd /mnt/t500/go1_ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

cd /mnt/t500/go1_ros2_ws/src/livox_ros_driver2
git rev-parse HEAD
```

Livox ROS 드라이버보다 먼저 Livox-SDK2를 설치한다. 저장소의 스크립트는
MID-360 지원 SDK를 고정 커밋으로 빌드하고 `/usr/local`에 설치한 뒤 정적
라이브러리 존재 여부를 확인한다.

```bash
chmod +x migration/install_livox_sdk2.sh
./migration/install_livox_sdk2.sh
```

고정한 Livox-SDK2 커밋:

```text
f5d9375f84efe2b15bc0a052d3e18482ed13adf4
```

검증한 커밋 해시는 별도 설치 기록에 남긴다.

Livox-SDK2와 드라이버의 공식 의존성을 설치한 후 Humble 모드로 빌드한다.

```bash
source /opt/ros/humble/setup.bash
cd /mnt/t500/go1_ros2_ws/src/livox_ros_driver2
./build.sh humble
```

위 명령은 공식 단독 워크스페이스 절차다. 공식 `build.sh`는 상대 경로의
`build`, `devel`, `install` 전체를 삭제하므로 통합 `go1_ros2_ws`에서는 직접
실행하지 않는다. 이 프로젝트에서는 다음 스크립트가 ROS2 manifest와 launch를
준비하고 Livox 및 FAST-LIO 범위만 빌드한다.

```bash
chmod +x migration/build_livox_fastlio.sh
./migration/build_livox_fastlio.sh
```

MID-360 설정에서 다음을 수정한다.

- LiDAR IP
- Jetson host IP
- point/IMU data port
- 출력 형식
- publish frequency

FAST-LIO에서는 point별 timestamp가 필요하므로 MID-360을 CustomMsg 방식으로
실행하는 구성을 우선 사용한다.

```bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

검증:

```bash
ros2 topic list | grep livox
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/imu --once
```

통과 조건:

- LiDAR와 IMU가 끊기지 않고 발행된다.
- LiDAR 메시지 형식이 FAST_LIO_ROS2 설정과 일치한다.
- timestamp가 정상적으로 증가한다.
- 네트워크 packet loss가 지속적으로 발생하지 않는다.

---

## 8. FAST-LIO ROS2 설치

저장소:

- <https://github.com/Ericsii/FAST_LIO_ROS2.git>

```bash
cd /mnt/t500/go1_ros2_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git

cd /mnt/t500/go1_ros2_ws/src/FAST_LIO_ROS2
git submodule update --init --recursive
git rev-parse HEAD
```

기존 ROS1 `FAST_LIO/config/mid360.yaml`을 통째로 덮어쓰지 않는다. ROS2 기본
설정에 다음 값만 비교하여 이식한다.

- `lid_topic`
- `imu_topic`
- `extrinsic_T`
- `extrinsic_R`
- `blind`
- voxel/filter 설정
- `time_sync_en`
- map/PCD 저장 설정
- 기존 현장 튜닝 값

빌드:

```bash
cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash 2>/dev/null || true

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fast_lio
source install/setup.bash
```

실행 예시:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

실제 저장소의 launch argument는 다음 명령으로 먼저 확인한다.

```bash
ros2 launch fast_lio mapping.launch.py --show-args
```

검증:

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic echo /Odometry --once
ros2 run tf2_ros tf2_echo camera_init body
```

통과 조건:

- 정지 상태에서 급격한 pose drift가 없다.
- 수동으로 천천히 이동할 때 pose가 연속적으로 변한다.
- 벽과 바닥 point cloud가 이동 후에도 안정적으로 정합된다.
- `Failed to find match for field 'time'`와 같은 timestamp 경고가 없다.

---

## 9. TF와 좌표계 계약

Nav2 적용 전에 다음 TF 구조를 하나로 확정한다.

```text
map -> odom -> base_link -> lidar
```

FAST-LIO가 `camera_init`, `body`와 같은 frame을 사용할 수 있으므로 다음을
실측 결과에 맞춰 명시적으로 대응한다.

```text
camera_init 또는 FAST-LIO world frame -> odom/map 역할
body                                 -> IMU/LiDAR body 역할
base_link                            -> Go1 기준 중심
lidar                                -> MID-360 frame
```

확인 명령:

```bash
ros2 topic echo /Odometry --once
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar
```

주의사항:

- 같은 parent-child TF를 두 노드가 동시에 발행하지 않는다.
- LiDAR-base extrinsic은 추정값이 아니라 측정된 값을 사용한다.
- Nav2를 실행하기 전에 TF jump와 timestamp 지연을 제거한다.
- 3D odometry를 2D Nav2에 전달할 때 roll, pitch, z 처리 정책을 결정한다.

---

## 10. Unitree SDK 버전 방침

### 10.1 반드시 구분할 내용

이 마이그레이션의 유일한 SDK 기준은 ROS1 아카이브 커밋
`f18fa0fe1f9e6cdcdabb83e89b628b9bb7ad7b40`에 보관된
`unitree_legged_sdk v3.8.6` 전체 스냅샷이다. 다음 항목을 함께 사용한다.

```text
python_wrapper/python_interface.cpp
lib/cpp/arm64/libunitree_legged_sdk.a
lib/python/arm64/robot_interface.cpython-38-aarch64-linux-gnu.so
```

따라서 다음 두 경로 중 하나를 선택해야 한다.

#### 경로 A: 기존 Python bridge를 유지

- 기존에 보관된 동일 SDK 소스와 ARM64 정적 라이브러리를 사용한다.
- 새 Jetson Python ABI로 `robot_interface`를 다시 빌드한다.
- 현재 `GO1-project`의 필터와 watchdog을 ROS2 `rclpy`로 포팅한다.

#### 경로 B: C++ ROS2 bridge로 변경

- `rclcpp` 노드에서 Unitree SDK에 직접 링크한다.
- Python ABI 문제를 제거할 수 있다.
- 장기 운용과 배포에는 이 경로를 우선 고려한다.

v3.5.1의 라이브러리나 소스와 v3.8.6 wrapper를 임의로 섞지 않는다. SDK
버전 사이에서 UDP 생성자와 command/state 구조가 다를 수 있다.

---

## 11. 기존 SDK Python wrapper 재빌드

이 절차는 경로 A를 선택했을 때만 수행한다.

### 11.1 의존성

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-dev \
  python3-pip \
  pybind11-dev \
  libmsgpack-dev \
  libboost-all-dev
```

### 11.2 기존에 사용한 SDK 소스 준비

`GO-_project_data`의 고정 커밋에 보관된 SDK만 새 Jetson으로 복사한다.

```bash
mkdir -p /mnt/t500/go1_sdk
cp -a /mnt/t500/go1_project_data/catkin_ws/src/unitree_legged_sdk \
  /mnt/t500/go1_sdk/
cd /mnt/t500/go1_sdk/unitree_legged_sdk
```

임의의 최신 SDK나 v3.5.1 checkout으로 대체하지 않는다. 자동 빌드 스크립트는
README 버전 표기와 보관된 ARM64 라이브러리/wrapper 해시를 모두 확인한다.

### 11.3 빌드

```bash
cd /mnt/t500/go1_sdk/unitree_legged_sdk
rm -rf build

cmake -S . -B build \
  -DPYTHON_BUILD=ON \
  -DPYTHON_EXECUTABLE="$(command -v python3)" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build -j"$(nproc)"
```

저장소의 자동 빌드 스크립트를 사용할 수도 있다. 인자로는 기존 ROS1
스냅샷 안의 SDK 절대 경로를 전달한다.

```bash
chmod +x migration/build_unitree_go1_wrapper.sh
./migration/build_unitree_go1_wrapper.sh \
  "/mnt/t500/go1_project_data/catkin_ws/src/unitree_legged_sdk"
```

스크립트는 원본을 변경하지 않고 `/mnt/t500/go1_sdk/unitree_legged_sdk`에 복사한 후
다음을 검사한다.

- 실행 아키텍처가 `aarch64`인지
- ARM64 Unitree 정적 라이브러리와 wrapper 소스가 같은 스냅샷에 있는지
- 현재 Python의 extension suffix와 결과 `.so`가 일치하는지
- 결과가 ARM64 ELF인지
- `ldd`에 `not found`가 없는지
- `robot_interface`, `HighCmd`, `HighState` import가 성공하는지

결과 확인:

```bash
ls -lh /mnt/t500/go1_sdk/unitree_legged_sdk/lib/python/arm64/
python3-config --extension-suffix
```

Python 3.10 환경이라면 결과 파일은 다음과 유사해야 한다.

```text
robot_interface.cpython-310-aarch64-linux-gnu.so
```

### 11.4 오래된 pybind11 오류 처리

기존 bundled pybind11이 새 Python에서 컴파일되지 않으면 다음을 설치한다.

```bash
python3 -m pip install --user --upgrade pybind11
python3 -m pybind11 --cmakedir
```

그 후 `python_wrapper/CMakeLists.txt`의 다음 줄을:

```cmake
add_subdirectory(third-party/pybind11)
```

다음으로 변경한다.

```cmake
find_package(pybind11 CONFIG REQUIRED)
```

다시 빌드한다.

```bash
cd /mnt/t500/go1_sdk/unitree_legged_sdk
rm -rf build

cmake -S . -B build \
  -DPYTHON_BUILD=ON \
  -DPYTHON_EXECUTABLE="$(command -v python3)" \
  -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build -j"$(nproc)"
```

### 11.5 로봇에 연결하지 않는 import 검사

```bash
export UNITREE_SDK_ROOT="/mnt/t500/go1_sdk/unitree_legged_sdk"
export PYTHONPATH="$UNITREE_SDK_ROOT/lib/python/arm64:${PYTHONPATH:-}"

python3 - <<'PY'
import sys
import robot_interface as sdk

print("Python:", sys.version)
print("Module:", sdk.__file__)
print("HighCmd:", type(sdk.HighCmd()))
print("HighState:", type(sdk.HighState()))
print("Unitree wrapper import OK")
PY
```

바이너리 검사:

```bash
file "$UNITREE_SDK_ROOT"/lib/python/arm64/robot_interface*.so
ldd "$UNITREE_SDK_ROOT"/lib/python/arm64/robot_interface*.so
```

통과 조건:

- `ARM aarch64` 바이너리다.
- 파일의 CPython ABI가 현재 `python3`와 일치한다.
- `ldd` 결과에 `not found`가 없다.
- UDP 객체를 만들지 않은 import 검사에 성공한다.

기존 `cpython-38` 파일의 이름만 `cpython-310`으로 변경해서는 안 된다.

---

## 12. Go1 ROS2 safety bridge 포팅

기존 `GO1-project`의 다음 안전 동작을 유지한다.

- 기본값은 disarmed/dry-run
- linear/angular deadband
- translation vector 크기 제한
- yaw 속도 제한
- stale command watchdog
- 정지 명령은 정확한 zero와 stand mode로 변환
- lateral 축 반전 파라미터
- 적용된 명령을 별도 ROS2 토픽으로 발행

이 저장소의 `go1_ros2_driver`는 위 동작을 `rclpy`로 포팅한 ROS2 Humble
패키지다. 새 워크스페이스에 다음과 같이 넣는다.

```bash
cp -a /mnt/t500/go1_ros2_project/packages/go1_driver /mnt/t500/go1_ros2_ws/src/go1_driver
cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select go1_driver
source install/setup.bash
```

SDK와 로봇 없이 dry-run:

```bash
ros2 launch go1_driver go1_driver.launch.py arm:=false
```

빌드 후 자동 dry-run 검증:

```bash
chmod +x migration/verify_go1_driver_dry_run.sh
./migration/verify_go1_driver_dry_run.sh
```

자동 검증은 다음 조건을 확인한다.

- 드라이버가 `DRY-RUN` 상태로 실행됨
- `(0.30, 0.30)` 대각선 입력의 방향을 유지하면서 norm이 `0.20 m/s`로 제한됨
- 입력 중단 후 watchdog이 정확한 zero command를 발행함

권장 ROS2 인터페이스:

```text
Subscribe: /cmd_vel                  geometry_msgs/msg/Twist
Publish:   /go1/cmd_vel_applied      geometry_msgs/msg/Twist
Publish:   /go1/control_state        진단 상태 또는 std_msgs/msg/String
Params:    arm, max_linear, max_yaw, deadband_linear,
           deadband_yaw, cmd_timeout, invert_lateral,
           robot_ip, local_port, robot_port
```

검증 순서:

1. Unitree SDK를 import/link하지 않는 필터 단위 테스트
2. SDK import 또는 링크 검사
3. `arm=false` 상태에서 `/cmd_vel` 입력과 applied 출력 비교
4. watchdog timeout 후 applied command가 zero인지 확인
5. 대각선 명령에서 x/y 비율이 유지되는지 확인
6. 실제 UDP 송신 코드를 연결하되 계속 `arm=false` 유지
7. 지지대 위에서 제한된 저속 명령만 시험

dry-run 예시:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08, y: 0.08, z: 0.0}, angular: {z: 0.0}}"

ros2 topic echo /go1/cmd_vel_applied
```

최초 실제 시험 권장 제한:

```text
max translation: 0.20 m/s 이하
max yaw:         0.40 rad/s 이하
```

---

## 13. Nav2 적용

기존 `sentry_nav`와 ROS1 `move_base` 설정을 그대로 복사하지 않고 현재 ROS2
`omx_navigation` 패키지를 기준으로 Nav2 설정을 구성한다.

기본 설치 예시:

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2
```

Nav2 입력 계약:

```text
Odometry: /odom 또는 remap된 FAST-LIO odometry
TF:       map -> odom -> base_link
Sensor:   /scan 또는 Nav2 obstacle layer가 받을 PointCloud2
Output:   /cmd_vel
```

MID-360의 3D point cloud를 사용하는 방법은 다음 중 하나로 확정한다.

1. `pointcloud_to_laserscan`으로 2D `/scan` 생성
2. Nav2 voxel/spatio-temporal obstacle layer에서 PointCloud2 직접 사용

초기 통합은 디버깅이 쉬운 2D `/scan` 변환 방식을 권장한다. Go1이 전방과
측면으로 이동할 수 있으므로 Nav2 robot model과 controller가 holonomic 동작을
지원하도록 설정해야 한다.

확인 항목:

- footprint 또는 robot radius
- holonomic velocity (`linear.y`) 허용 여부
- 최대 속도와 가감속
- controller frequency
- obstacle/raytrace range
- costmap frame과 sensor frame
- progress/goal checker
- velocity smoother

Go1 실제 출력 없이 먼저 검증한다.

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /go1/cmd_vel_applied
```

---

## 14. Localization 및 지도 전략

FAST-LIO odometry만으로는 재부팅 후 기존 지도에서의 전역 위치가 자동으로
복구되지 않는다. 다음 중 한 가지를 별도 결정해야 한다.

### 선택 A: 2D Nav2 중심

- FAST-LIO로 odometry 생성
- 2D map을 생성/보존
- AMCL 또는 적합한 2D localization 사용
- `map -> odom`은 localization이 발행

### 선택 B: 3D PCD localization 중심

- 기존 `FAST_LIO_LOCALIZATION` 기능을 ROS2로 포팅하거나 검증된 ROS2 대체재 선정
- 기존 PCD map에 대한 초기 정합과 relocalization 구현
- localization 결과로 `map -> odom` 구성

첫 ROS2 통합에서는 선택 A로 전체 파이프라인을 먼저 완성하고, 3D 전역 위치
추정이 반드시 필요할 때 선택 B를 추가하는 방식을 권장한다.

---

## 15. 전체 통합 실행 순서

각 단계를 통과하기 전 다음 단계로 넘어가지 않는다.

### Gate 1: 시스템과 네트워크

- Jetson/ROS2/Python 버전 기록
- Go1 NIC와 MID-360 NIC 주소 확인
- LiDAR와 Go1에 각각 ping 또는 통신 확인

### Gate 2: MID-360

- LiDAR/IMU ROS2 토픽 정상
- timestamp와 주기 정상

### Gate 3: FAST-LIO

- `/Odometry`와 `/cloud_registered` 정상
- 정지 drift와 이동 정합 확인

### Gate 4: TF

- `map -> odom -> base_link -> lidar` 연결
- 중복 TF 없음
- TF timestamp 오류 없음

### Gate 5: Go1 bridge dry-run

- `arm=false`
- deadband, limit, diagonal, watchdog 통과
- SDK import/link 정상

### Gate 6: Nav2 dry-run

- RViz goal에 대해 `/cmd_vel` 출력
- Go1 bridge applied 출력 확인
- 실제 UDP 출력은 비활성화

### Gate 7: 저속 실제 시험

- 로봇을 지지하거나 충분한 안전 공간 확보
- 리모컨과 비상 정지 준비
- zero, 전진, 측면, 대각선, 제자리 회전 순으로 시험

### Gate 8: 짧은 자율주행

- 가까운 목표만 사용
- localization과 TF를 계속 모니터링
- 명령 timeout과 정지 동작 확인

---

## 16. 실제 Go1 시험 안전 체크리스트

- [ ] 리모컨과 비상 정지가 즉시 사용 가능하다.
- [ ] 주변에 사람과 장애물이 없다.
- [ ] 최초 시험은 로봇을 지지한 상태에서 수행한다.
- [ ] `arm=false` dry-run 테스트가 모두 통과했다.
- [ ] `/cmd_vel`이 사라지면 watchdog이 zero를 출력한다.
- [ ] zero 명령에서 Go1이 stepping 없이 정지한다.
- [ ] positive `linear.y` 방향을 실제로 확인했다.
- [ ] TF jump와 FAST-LIO localization loss가 없다.
- [ ] 속도 제한이 보수적으로 설정되어 있다.
- [ ] ROS2 노드 종료 시 마지막 zero/stand 명령을 보낸다.

다음 상황에서는 즉시 중단한다.

- TF jump
- FAST-LIO odometry 급변 또는 소실
- LiDAR/IMU 토픽 중단
- 예상과 반대인 측면 이동
- zero 명령 후 계속 stepping
- command watchdog 미작동
- 반복되는 빈 point cloud

---

## 17. 빌드 및 재현성 관리

워크스페이스 전체 빌드:

```bash
cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

소스 수정 후 문제가 발생했을 때만 생성물을 정리한다.

```bash
cd /mnt/t500/go1_ros2_ws
rm -rf build install log
colcon build --symlink-install
```

위 삭제 대상은 새 ROS2 워크스페이스의 생성물로만 제한한다. 기존
`go1_project_data`와 `src` 소스는 삭제하지 않는다.

재현성을 위해 다음을 저장한다.

- `ros2.repos`: 외부 저장소 URL과 고정 커밋
- `SYSTEM_INFO.md`: JetPack, Ubuntu, ROS2, Python, CUDA 버전
- `NETWORK.md`: NIC 이름, IP, device IP, UDP port
- `TF_CONTRACT.md`: frame 이름과 발행 노드
- `CALIBRATION.md`: LiDAR-IMU 및 LiDAR-base extrinsic
- `TEST_RESULTS.md`: 각 Gate의 통과 결과와 날짜

---

## 18. 최종 완료 조건

- [ ] 기존 ROS1 자료가 변경 없이 보존되어 있다.
- [ ] 새 ROS2 워크스페이스를 빈 환경에서 다시 빌드할 수 있다.
- [ ] MID-360 ROS2 토픽과 point timestamp가 정상이다.
- [ ] FAST_LIO_ROS2 odometry와 point cloud가 안정적이다.
- [ ] TF 트리가 단일하고 일관적이다.
- [ ] Go1 bridge가 기본 disarmed 상태다.
- [ ] deadband, 속도 제한, diagonal, watchdog 테스트가 통과한다.
- [ ] Nav2가 holonomic `/cmd_vel`을 생성한다.
- [ ] zero 명령과 노드 종료 시 Go1이 안전하게 정지한다.
- [ ] 지도 저장과 재시작 후 localization 절차가 문서화되어 있다.
- [ ] 모든 외부 저장소 커밋과 시스템 버전이 기록되어 있다.

---

## 19. 권장 구현 우선순위

```text
1. 새 Jetson 환경/네트워크 고정
2. livox_ros_driver2
3. FAST_LIO_ROS2
4. TF 계약
5. Go1 SDK import 또는 C++ 링크
6. Go1 ROS2 safety bridge dry-run
7. Nav2와 /cmd_vel 통합
8. 저속 실제 제어
9. 지도 저장과 localization
10. 전체 bringup launch 및 자동화
```

초기 목표는 한 번에 완전 자율주행을 구현하는 것이 아니라, 각 Gate에서 문제를
분리하여 마지막에 전체 파이프라인을 연결하는 것이다.
