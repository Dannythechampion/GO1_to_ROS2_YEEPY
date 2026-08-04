# GO1 3D PCD Localization — 오늘 밤 실행 Runbook

**전제:** RustDesk로 Jetson 데스크톱에 접속, 모든 ROS2 프로세스는 Jetson에서 직접 실행.
**목표:** `arm:=false` 상태에서 3D localization → Nav2 경로 생성까지 확인.
**금지:** 이 문서의 게이트를 전부 통과하기 전에는 `arm:=true` 금지.

```
map --[pcd_localizer / NDT+GICP]--> camera_init --[FAST-LIO]--> body
```

## 오늘 반영되는 수정 요약

| 수정 | 효과 |
|---|---|
| MultiThreadedExecutor + callback group 분리 | NDT/GICP가 20 Hz `map -> camera_init` 발행을 막지 않음 |
| TF 스탬프에 0.2 s tolerance | Nav2가 TF 버퍼 끝을 넘겨 외삽하지 않음 |
| target / KD-tree / GICP 공분산 캐싱 | 사이클당 CPU 대폭 감소 |
| scan 15 m crop | 대응점 없는 점이 fitness를 오염시키지 않음 |
| `map_leaf 0.25`, `ndt_res 1.5` | NDT voxel 사용률 48.7 % → 82 % |
| `nav2_pcd_localization_params.yaml` (amcl 제거, `allow_unknown: false`, inflation 0.45) | TF 중복 불가 + 유리/반사 영역 미주행 |
| `hanyang_9f_annotated.yaml` 기본값 | 정리된 지도 사용 |
| `go1_pcd_localization.rviz` | `map_cloud` / `aligned_cloud`가 바로 보임 |
| `map_bounds_xy` 가드 | 2D 지도 밖 pose 거부 |
| odom reset 감지 | FAST-LIO 재시작 시 즉시 TF 중단 |

> **오늘의 운용 제약:** `merged.pcd`와 2D 지도의 정렬 오차는 원점에서 멀어질수록 커진다
> (0–5 m 0.10 m, 15–20 m 0.41 m). **첫 시험은 매핑 시작 지점(map 원점 `0, 0`) 기준
> 반경 10 m 안에서만** 하고, 그 밖으로 나갈 계획이면 먼저 PCD 재투영 지도를 만들 것.

---

## 0. 준비 — Jetson 데스크톱 확인

RustDesk로 보이는 Jetson 데스크톱에서 터미널을 **4개** 연다. 첫 터미널에서:

```bash
hostname
echo "DISPLAY=${DISPLAY:-<empty>}"
test -n "${DISPLAY:-}"
command -v rviz2
nproc && free -m | head -2
```

`DISPLAY`가 비어 있으면 일반 SSH다. RustDesk 안의 데스크톱 터미널을 다시 열 것.

모든 터미널에서 공통으로 실행할 것:

```bash
export ROS_DOMAIN_ID=100
export GO1_PROJECT_ROOT=/mnt/t500/go1_ros2_project
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"
```

---

## 1. 코드 반영 (터미널 1)

### 1-1. 작업 브랜치 생성

```bash
cd "$GO1_PROJECT_ROOT"
git fetch origin
git status --short          # 로컬 변경이 있으면 먼저 정리/커밋
git checkout -b nav2-workflow_3D-fix origin/nav2-workflow_3D
```

### 1-2. 지도 세션 복원 (3D 브랜치에는 지도가 없다)

`nav2-workflow_3D`는 지도 커밋 이전에 분기했다. `merged.pcd`와 정리된 2D 지도를 e2e 브랜치에서 가져온다.

```bash
cd "$GO1_PROJECT_ROOT"
git checkout origin/agent/nav2-end-to-end-workflow -- maps/

# 무결성 확인 — PDF 및 config에 기록된 값과 일치해야 한다
sha256sum maps/hanyang_9f/20260728_204825/pcd/merged.pcd
# 기대값: 9af57fbb96dd9364e466913dd453ef26ad9573564134d649230e1d1a3685201d

ls -l maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.{yaml,pgm}
head -3 maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml
```

`git checkout <ref> -- maps/` 는 파일을 스테이지에 올린다. 커밋하지 않고 두거나
`git commit -m "data: restore validated Hanyang 9F map session"` 로 남긴다.

### 1-3. 리뷰 수정본 적용

전달받은 `go1_3d_fixes/` 를 Jetson으로 복사한 뒤:

```bash
cd "$GO1_PROJECT_ROOT"
rsync -a --info=NAME ~/go1_3d_fixes/packages/   packages/
rsync -a --info=NAME ~/go1_3d_fixes/migration/  migration/
rsync -a --info=NAME ~/go1_3d_fixes/docs/       docs/
chmod +x migration/verify_pcd_localization.sh

git status --short
```

변경/추가되는 파일:

```
M packages/omx_pcd_localization/src/pcd_localizer.cpp
M packages/omx_pcd_localization/config/hanyang_9f.yaml
M packages/omx_pcd_localization/launch/go1_pcd_navigation.launch.py
M packages/omx_pcd_localization/CMakeLists.txt
M packages/omx_pcd_localization/package.xml
M packages/omx_pcd_localization/test/test_pcd_localization_contract.py
A packages/omx_pcd_localization/rviz/go1_pcd_localization.rviz
A packages/omx_navigation/config/nav2_pcd_localization_params.yaml
M migration/verify_pcd_localization.sh
A docs/GO1_TONIGHT_RUNBOOK.md
```

---

## 2. 빌드 (터미널 1)

### 2-1. workspace에 소스 반영

처음이면:

```bash
cd "$GO1_PROJECT_ROOT"
./migration/stage_local_ros2_packages.sh
```

이미 `$GO1_ROS2_WS/src`에 패키지가 있으면 stage 스크립트가 거부하므로 rsync로 갱신한다:

```bash
rsync -a --delete "$GO1_PROJECT_ROOT/packages/omx_pcd_localization/" \
                  "$GO1_ROS2_WS/src/omx_pcd_localization/"
rsync -a --delete "$GO1_PROJECT_ROOT/packages/omx_navigation/" \
                  "$GO1_ROS2_WS/src/omx_navigation/"
```

### 2-2. 빌드

```bash
cd "$GO1_ROS2_WS"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Jetson에서 4잡 이상 병렬 빌드는 메모리 부족을 유발할 수 있다
MAKEFLAGS="-j2" colcon build --symlink-install \
  --packages-select go1_driver omx_navigation omx_pcd_localization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

빌드 확인:

```bash
ros2 pkg prefix omx_pcd_localization
ls "$(ros2 pkg prefix omx_pcd_localization)/share/omx_pcd_localization/rviz/"
ls "$(ros2 pkg prefix omx_navigation)/share/omx_navigation/config/" | grep pcd
which pcd_localizer || ls "$(ros2 pkg prefix omx_pcd_localization)/lib/omx_pcd_localization/"
```

`nav2_pcd_localization_params.yaml`과 `go1_pcd_localization.rviz`가 install 트리에 보여야 한다.

### 2-3. (선택) 계약 테스트

```bash
cd "$GO1_ROS2_WS"
colcon test --packages-select omx_pcd_localization omx_navigation
colcon test-result --verbose
```

---

## 3. 터미널 2 — MID-360 드라이버

로봇을 **정지 상태**로 두고 시작한다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

---

## 4. 터미널 3 — FAST-LIO

IMU 초기화 동안(약 15초) 로봇을 움직이지 않는다.

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_path:=/mnt/t500/go1_ros2_ws/install/omx_navigation/share/omx_navigation/config \
  config_file:=fast_lio_mid360_navigation.yaml \
  rviz:=false
```

---

## 5. 게이트 A — 센서 / 오도메트리 (터미널 4)

```bash
export ROS_DOMAIN_ID=100
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"

cd /mnt/t500/go1_ros2_project
./migration/verify_pcd_localization.sh sensors
```

통과 조건: `/livox/lidar` ≈ 10 Hz, `/livox/imu` ≈ 200 Hz, `/Odometry` ≈ 10 Hz,
`/cloud_registered_body` ≈ 10 Hz, `camera_init -> body` TF 존재.

정지 상태에서 drift를 눈으로 확인:

```bash
timeout 30 ros2 topic echo /Odometry --field pose.pose.position
```

30초 동안 x, y가 수 cm 이상 움직이면 FAST-LIO가 불안정한 것이다.
**여기서 멈추고 원인을 먼저 해결한다.** 이 단계가 나쁘면 3D 정합도 반드시 나쁘다.

---

## 6. 터미널 4 — 3D localization + Nav2 실행

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash

ros2 launch omx_pcd_localization go1_pcd_navigation.launch.py \
  rviz:=true \
  start_go1_driver:=true \
  arm:=false
```

기본값으로 다음이 사용된다 (필요하면 인자로 덮어쓴다):

```
map      = /mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_annotated.yaml
pcd_map  = /mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/pcd/merged.pcd
```

기동 로그에서 다음 두 줄을 확인:

```
Loaded NNNNN map points from .../merged.pcd; waiting for /initialpose
```

`map_leaf_size: 0.25`에서 약 **21,500점**이면 정상이다.

경로가 다르면 실행 전에 `RuntimeError: Required localization inputs are missing:` 로
바로 멈춘다. 그때는 인자로 지정한다:

```bash
ros2 launch omx_pcd_localization go1_pcd_navigation.launch.py \
  map:=/절대/경로/hanyang_9f_annotated.yaml \
  pcd_map:=/절대/경로/merged.pcd \
  rviz:=true start_go1_driver:=true arm:=false
```

---

## 7. 초기 자세 입력 (RViz)

RViz가 뜨면:

1. `Existing Map`(2D 점유 지도)과 `PCD Map`(회색 점군)이 함께 보이는지 확인.
   두 개의 벽이 대략 겹쳐야 한다. **여기서부터 크게 어긋나 있으면 진행 금지.**
2. `Aligned Scan`(주황) display는 아직 비어 있다.
3. 툴바 **2D Pose Estimate** 로 로봇의 **실제 위치와 방향**을 지정한다.
   FAST-LIO를 시작한 지점이면 지도 원점 `(0, 0)` 근처다.
   화살표 방향(로봇 전방)까지 맞추는 것이 중요하다.

터미널 4의 로그:

```
Initial pose received at [x, y, 0.00]; starting 3D alignment
Accepted 3D alignment: fitness=... inlier=... margin=... correction=...m/...deg source=... target=...
```

`Aligned Scan`(주황)이 `PCD Map`(회색) 위에 겹쳐 나타나면 성공이다.

---

## 8. 게이트 B — 전체 검증 (새 터미널 5)

```bash
export ROS_DOMAIN_ID=100
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source "$GO1_ROS2_WS/install/setup.bash"

cd /mnt/t500/go1_ros2_project
./migration/verify_pcd_localization.sh
```

이 스크립트가 확인하는 것:

| 게이트 | 내용 |
|---|---|
| 1–2 | 센서 / FAST-LIO 토픽 rate, `camera_init -> body` |
| 3–4 | `/map`, `/pcd_localizer/{status,pose,map_cloud,aligned_cloud}`, `/scan` |
| 4 | 12초 동안 `LOCALIZED`가 나오고 `REJECTED`/`STALE`이 **한 번도** 없을 것 |
| 6 | `map -> camera_init`, `map -> body`, **publisher 1개**, **최대 gap ≤ 0.5 s** |
| 6 | `/amcl` 미실행 |
| 7 | Nav2 8개 lifecycle 노드 active (`velocity_smoother` 포함) |
| 8 | `/go1_driver arm=false`, CPU/메모리 스냅샷 |

`PASS: guarded 3D PCD localization and Nav2 are active` 가 나와야 다음 단계로 간다.

### 게이트 5 — 좌표 일치 (사람이 눈으로)

RViz에서 다음을 확인한다. 스크린샷을 남길 것.

- `Aligned Scan`의 벽이 `PCD Map`의 벽 위에 겹치는가
- 그 벽들이 `Existing Map`(2D)의 검은 셀과도 겹치는가
- 로봇 위치(TF `body` 축)가 흰색 자유 셀 안에 있는가 — 벽 안이면 진행 금지

---

## 9. 기준값 기록 (오늘의 핵심 산출물)

가드를 나중에 조일 수 있도록, **정지 상태로 2분** 두고 값을 수집한다.

```bash
timeout 120 ros2 topic echo /pcd_localizer/status --field data | tee /tmp/pcd_status.log
grep -c LOCALIZED /tmp/pcd_status.log
grep -c REJECTED  /tmp/pcd_status.log

# fitness / inlier / margin 분포
grep -o 'fitness=[0-9.]*' /tmp/pcd_status.log | cut -d= -f2 | sort -n | \
  awk '{a[NR]=$1} END{print "fitness  min",a[1],"med",a[int(NR/2)],"max",a[NR]}'
grep -o 'inlier=[0-9.]*'  /tmp/pcd_status.log | cut -d= -f2 | sort -n | \
  awk '{a[NR]=$1} END{print "inlier   min",a[1],"med",a[int(NR/2)],"max",a[NR]}'
grep -o 'margin=[-0-9.]*' /tmp/pcd_status.log | cut -d= -f2 | sort -n | \
  awk '{a[NR]=$1} END{print "margin   min",a[1],"med",a[int(NR/2)],"max",a[NR]}'
```

CPU 부하:

```bash
top -b -n 10 -d 1 | grep -E "pcd_localizer|fastlio|fast_lio" | \
  awk '{print $9}' | sort -n | tail -3
```

TF 연속성 (Nav2가 실제로 신뢰하는 값):

```bash
timeout 20 ros2 run tf2_ros tf2_monitor map camera_init
```

`Max Delta`가 0.5 s를 넘으면 **정합이 여전히 너무 느린 것**이다.
`config/hanyang_9f.yaml`에서 `registration_rate_hz: 0.5 -> 0.25`,
또는 `map_leaf_size: 0.25 -> 0.35`로 낮춘다.

**측정 후 다음 회차부터 적용할 것** (`config/hanyang_9f.yaml`):

```yaml
max_fitness_score: <관측 중앙값 x 3~5>
min_inlier_ratio: <관측 중앙값 x 0.6>
degeneracy_guard_enabled: true
min_degeneracy_margin: <관측 margin 최솟값 x 0.5>
```

---

## 10. Nav2 dry-run (여전히 arm=false)

로봇은 **움직이지 않는다.** `/cmd_vel`이 나와도 `arm:=false`라 드라이버가 무시한다.

1. RViz 툴바 **2D Goal Pose** 로 현재 위치에서 **2–3 m 이내, 흰색 자유 공간**에 목표를 찍는다.
2. `Global Plan`(경로)이 생성되는지 본다.
3. 새 터미널에서:

```bash
timeout 15 ros2 topic echo /cmd_vel
ros2 topic echo /plan --once | head -20
```

`/cmd_vel`에 0이 아닌 `linear.x` / `angular.z`가 나오고, 값이
`max_vel_x: 0.20`, `max_vel_theta: 0.40`을 넘지 않아야 한다.

목표를 취소한다:

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped "{}" 2>/dev/null || true
# 또는 RViz에서 Nav2 패널의 Cancel
```

> `allow_unknown: false` + 정리된 지도라서 자유 셀이 27,592개뿐이다.
> "경로 없음"이 나오면 대개 목표가 회색(미확인) 셀 위에 있는 것이다.
> **이건 버그가 아니라 의도된 안전 동작이다.** 흰색 셀 위로 목표를 옮긴다.

---

## 11. 종료 순서

역순으로 끈다. RustDesk를 먼저 끊으면 RViz가 DISPLAY를 잃고 프로세스가 남는다.

```
터미널 4 (Nav2 + localizer + RViz)  Ctrl-C
터미널 3 (FAST-LIO)                 Ctrl-C
터미널 2 (Livox)                    Ctrl-C
그 다음 RustDesk 연결 해제
```

잔여 프로세스 확인:

```bash
pgrep -a -f "pcd_localizer|rviz2|fastlio|fast_lio|livox" || echo "clean"
```

---

## 12. 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `RuntimeError: Required localization inputs are missing` | 지도 경로 | 1-2 단계 재확인, `map:=` / `pcd_map:=` 로 절대경로 지정 |
| `Loaded 8960 map points` | 옛 `map_leaf_size: 0.40` config가 install됨 | 2-1 rsync 후 재빌드 |
| `REJECTED tf_error=...` | `camera_init -> body` 없음 / 시간 불일치 | FAST-LIO 재확인, 게이트 A로 복귀 |
| `REJECTED too_few_target_points` | 초기 자세가 지도 밖 | 2D Pose Estimate를 지도 안쪽에 다시 |
| `REJECTED_INITIAL_POSE outside_map_bounds` | 지도 범위 밖 클릭 | x∈[-8.57, 11.88], y∈[-19.20, 10.10] 안으로 |
| `REJECTED fitness=0.9xxx` | 초기 자세 방향이 틀림 | 화살표 방향을 로봇 전방에 맞춰 다시 |
| `REJECTED pose_jump ...` | 초기 자세가 3 m 이상 어긋남 | 더 정확히 다시 지정 |
| `REJECTED odom_reset jump=...` | FAST-LIO 재시작 | 2D Pose Estimate 재입력 |
| `STALE no_accepted_alignment` | 5초간 정합 실패 | 로그의 직전 `REJECTED` 사유를 볼 것 |
| `tf2_monitor Max Delta > 0.5` | CPU 부족 | `registration_rate_hz` ↓ 또는 `map_leaf_size` ↑ |
| Nav2 `Timed out waiting for transform` | 위와 동일 | 위와 동일 |
| `Aligned Scan`은 맞는데 2D 지도 벽과 어긋남 | PCD↔2D drift | 원점 10 m 이내에서만 시험, 이후 PCD 재투영 지도 검토 |
| RViz가 아무것도 못 그림 | 아직 LOCALIZED 아님 | 초기 자세 입력 후 다시 확인 |

## 13. 되돌리기

```bash
cd /mnt/t500/go1_ros2_project
git checkout nav2-workflow_3D          # 또는 git stash
rsync -a --delete packages/omx_pcd_localization/ /mnt/t500/go1_ros2_ws/src/omx_pcd_localization/
rsync -a --delete packages/omx_navigation/       /mnt/t500/go1_ros2_ws/src/omx_navigation/
cd /mnt/t500/go1_ros2_ws && colcon build --symlink-install \
  --packages-select omx_navigation omx_pcd_localization
```

---

## 오늘 밤의 통과 기준

- [ ] 게이트 A 통과 (센서 + FAST-LIO 안정)
- [ ] `Loaded ~21500 map points` 로그 확인
- [ ] 2D Pose Estimate 후 `LOCALIZED` 도달
- [ ] `verify_pcd_localization.sh` 전체 통과 (특히 TF max delta ≤ 0.5 s)
- [ ] RViz에서 Aligned Scan ↔ PCD Map ↔ 2D 지도 벽 일치 (스크린샷)
- [ ] fitness / inlier / margin / CPU 기준값 2분 수집
- [ ] arm=false 상태로 Nav2 global plan + `/cmd_vel` 생성 확인
- [ ] 한 번도 `arm:=true` 로 전환하지 않음
