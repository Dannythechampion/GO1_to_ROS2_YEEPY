# Go1 3D PCD localization

이 패키지는 FAST-LIO의 `camera_init -> body` 오도메트리를 유지하면서 저장된
`merged.pcd`에 실시간 `/cloud_registered_body`를 정합해
`map -> camera_init`을 발행한다. AMCL은 실행하지 않는다.

```text
MID-360 + IMU
  -> FAST-LIO 3D odometry (camera_init -> body)
  -> NDT coarse alignment
  -> GICP 6DoF refinement
  -> guarded map -> camera_init TF
  -> Nav2 2D planning
```

PCL의 NDT와 GICP는 근사 초기 자세가 필요한 로컬 정합기다. 따라서 시작할 때
RViz의 `2D Pose Estimate`로 실제 위치와 방향을 대략 지정해야 한다. 이 입력은
AMCL용이 아니라 첫 3D 정합의 초기값이다.

RustDesk로 Jetson 데스크톱을 제어하고 Jetson에서 RViz를 실행하는 전제의
MID-360, FAST-LIO, 3D localization, 검증 전체 명령은
[`docs/GO1_3D_LOCALIZATION_RUNBOOK.md`](../../docs/GO1_3D_LOCALIZATION_RUNBOOK.md)를
따른다.

## 지도

기본 3D 지도 경로는 다음과 같다.

```text
/mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/pcd/merged.pcd
```

센서로 생성한 지도 파일은 코드 브랜치에 포함하지 않는다. 실행 전에 위 경로에
`merged.pcd`를 배치하거나 launch의 `pcd_map:=` 인자로 실제 절대 경로를 전달한다.
개발 시 검증한 파일은 114,519개 포인트이며 SHA-256은 다음과 같다.

```text
9af57fbb96dd9364e466913dd453ef26ad9573564134d649230e1d1a3685201d
```

`config/hanyang_9f.yaml`의 `pcd_to_map_xyz_rpy`는 PCD 투영 지도와 SLAM
Toolbox 지도를 0.05 m/px에서 비교해 얻은 임시 강체 보정이다. 실제 주행 전에
`arm:=false`로 `/pcd_localizer/aligned_cloud`와 2D 지도의 벽이 겹치는지 확인하고
필요하면 이 값을 보정해야 한다.

## 빌드

```bash
cd /mnt/t500/go1_ros2_project
./migration/stage_local_ros2_packages.sh

cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select go1_driver omx_navigation omx_pcd_localization
source install/setup.bash
```

이미 `src/go1_driver` 또는 `src/omx_navigation`이 있으면 stage 스크립트가 안전을
위해 덮어쓰기를 거부한다. 기존 workspace를 직접 갱신하거나 새 workspace에서
시험한다.

## 실행

먼저 Livox와 FAST-LIO를 기존 절차대로 실행한 뒤 새 터미널에서 실행한다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch omx_pcd_localization go1_pcd_navigation.launch.py \
  rviz:=true \
  start_go1_driver:=true \
  arm:=false
```

RViz에서 `2D Pose Estimate`를 실제 위치와 방향에 가깝게 지정한다. 다음 상태가
나오기 전에는 목표를 보내거나 `arm:=true`로 전환하지 않는다.

```bash
ros2 topic echo /pcd_localizer/status
ros2 topic echo /pcd_localizer/pose
ros2 run tf2_ros tf2_echo map camera_init
```

성공 상태는 `LOCALIZED fitness=...`이다. `REJECTED`, `STALE`, TF 단절, 점군과
지도의 불일치가 하나라도 있으면 주행하지 않는다.

## 안전 게이트

- 초기 자세가 없으면 정합과 TF 발행을 시작하지 않는다.
- NDT와 GICP가 모두 수렴해야 한다.
- fitness, 위치/회전 점프, z, roll/pitch 한계를 모두 통과해야 한다.
- 최근 5초 이내 승인된 정합이 없으면 `map -> camera_init` TF 발행을 중단한다.
- 기본 launch는 `arm:=false`다.

최초 실기 시험은 정지 상태에서 30초 이상 정합 안정성을 확인한 뒤, 손으로 아주
천천히 이동하며 수행한다. 파라미터 튜닝 전에는 `arm:=true`를 사용하지 않는다.
