# Go1 저장 지도 기반 강건 초기 로컬라이제이션 설계

## 1. 목적

한양대 9층 저장 지도에서 사용자가 RViz `2D Pose Estimate`를 한 번만 대략
지정해도 Go1이 초기 위치를 보정하고, 로컬라이제이션이 확인된 뒤 Nav2 목적지
이동을 시작할 수 있게 한다.

목표 초기 오차 범위는 위치 `±3 m`, 방향 `±90 deg`다. 정합이 불확실할 때는
잘못된 위치를 채택하지 않고 명확한 실패 상태를 낸다. 향후 지도 전체 자동
초기화로 확장할 수 있도록 초기 후보 생성과 지속 추적의 책임을 분리한다.

## 2. 이번 작업 범위

2026-08-12 현재 Jetson 실기는 없지만 Ubuntu 22.04 WSL2에 ROS2 Humble과
`colcon`이 설치되어 있다. 다음 범위를 완료 대상으로 삼는다.

- GitHub `main` 최신 코드와 저장된 지도, pose graph, 검증 기록을 분석한다.
- Windows에서 실행 가능한 순수 Python 로직과 테스트를 작성한다.
- WSL Ubuntu 22.04의 ROS2 Humble에서 패키지를 실제로 `colcon build/test`한다.
- ROS2 launch의 인자 해석과 패키지 설치 결과를 WSL에서 검증한다.
- 현장에서 실패 원인을 수집할 상태 토픽, 한국어 진단 메시지, rosbag 실행 옵션을
  제공한다.
- 기존 내비게이션 코드의 안전·구성 문제를 수정하거나, 실측이 필요한 항목은
  코드가 보수적으로 거부하도록 만든다.

이번 작업에서 Jetson ARM64 빌드 성공, 센서 실측 성능, 실제 로봇 구동 성공을
주장하지 않는다. WSL x86_64 빌드·테스트 결과만 근거로 보고한다. `arm:=true`
운용 절차와 완전 자동 전역 로컬라이제이션도 범위 밖이다.

## 3. 근거와 문제 정의

현재 `main`은 AMCL을 사용하며 초기 입자 수가 `300..1200`, 복구 계수
`recovery_alpha_fast/slow`가 `0.0`이다. RViz의 초기 자세 주변에서 충분한 후보를
찾지 못하면 사용자가 `/initialpose`를 반복 발행해야 한다. 한양대 지도처럼 긴
복도와 반복 구조가 있는 환경에서는 입자 수만 늘리면 잘못된 대칭 위치로 수렴할
위험도 커진다.

별도 `nav2-workflow_3D` 브랜치의 PCD localizer는 NDT/GICP 단일 초기값을 사용하고
초기 회전 보정을 약 `60 deg`로 제한한다. 또한 설정 파일 자체가 PCD와 loop-closed
2D 지도 사이의 위치별 잔차가 최대 약 `0.41 m`임을 기록한다. LiDAR 장착 위치도
아직 변할 수 있으므로 3D PCD 정합을 운영 TF의 주 소유자로 채택하지 않는다.

저장 세션에는 `hanyang_9f.posegraph`와 `hanyang_9f.data`가 있고 검증 보고서는
완료 상태이며 출발점 복귀 오차를 약 `0.07 m`, `4.96 deg`로 기록한다. 따라서
pose graph와 같은 좌표계의 2D 스캔 정합을 초기·지속 로컬라이제이션에 사용하는
것이 현재 자료에 가장 일관적이다.

## 4. 아키텍처

운영 모드에서는 SLAM Toolbox localization만 `map -> camera_init` TF를 발행한다.
AMCL은 별도의 fallback launch에만 남기며 동시에 실행하지 않는다.

```text
MID-360
  -> FAST-LIO
       -> /Odometry, camera_init -> body
       -> /cloud_registered_body
  -> pointcloud_to_laserscan
       -> /scan
  -> SLAM Toolbox localization
       + hanyang_9f.posegraph/.data
       + one /initialpose
       -> /slam_toolbox/pose
       -> map -> camera_init
  -> localization_supervisor
       + scan_map_quality
       -> WAITING_INPUT / ALIGNING / VERIFYING / READY / DEGRADED / LOST
  -> Nav2 /cmd_vel_nav
  -> cmd_vel_safety_gate
  -> /cmd_vel
  -> go1_driver
```

정리된 `hanyang_9f_annotated.yaml`은 계속 Nav2 map server가 `/map`으로 발행한다.
SLAM Toolbox의 내부 occupancy map은 `/slam_localization/map`으로 remap해 두 지도
발행자가 충돌하지 않게 한다. 두 지도는 동일한 pose graph 좌표계를 사용한다.

## 5. 컴포넌트 책임

### 5.1 `localization_state.py`

ROS2에 의존하지 않는 상태 머신이다. 입력 freshness, 정합 품질, pose jump,
재시도 횟수와 timeout을 받아 다음 상태와 오류 코드를 반환한다. Windows 단위
테스트는 이 모듈을 직접 검증한다.

기본 상태 전이는 다음과 같다.

```text
WAITING_INPUT -> ALIGNING -> VERIFYING -> READY
                                  |          |
                                  v          v
                                LOST <- DEGRADED
```

초기 자세 한 번을 보관하고 최대 3회, 전체 20초 동안 같은 입력으로 자동 재시도한다.
새 클릭은 자동 복구가 모두 실패한 뒤에만 필요하다.

### 5.2 `scan_map_quality.py`

`nav_msgs/OccupancyGrid`, `sensor_msgs/LaserScan`, 현재 map pose를 사용해 다음 값을
계산한다.

- 유효 endpoint 수
- occupied cell까지 `0.25 m` 이내인 endpoint 비율
- endpoint 평균 지도 거리
- 현재 자세 주변의 대체 후보 점수와 최상·차상 후보 차이

지도 distance field와 점수 함수는 ROS2 비의존 코어로 분리한다. 지도 밖 접근,
빈 scan, `NaN/Inf`, 대칭 복도 후보를 결정적으로 테스트한다.

### 5.3 `localization_supervisor.py`

상태 머신과 품질 계산 결과를 ROS 토픽에 연결한다. 다음을 감시한다.

- `/scan`, `/Odometry`, `/slam_toolbox/pose`
- `camera_init -> body`, `map -> camera_init` TF
- 중복 TF 소유자와 FAST-LIO odom reset 징후
- 초기 자세가 지도 안에 있는지 여부

`~/status`에는 기계 판독 가능한 상태·오류 코드와 한국어 설명을 발행하고,
`~/ready`에는 safety gate가 사용할 boolean을 발행한다.

### 5.4 `cmd_vel_safety_gate.py`

Nav2 controller의 출력을 `/cmd_vel_nav`로 받고 다음 조건을 모두 만족할 때만
`/cmd_vel`로 전달한다.

- supervisor 상태가 `READY`
- ready heartbeat가 `0.30 s`보다 오래되지 않음
- 입력 명령이 `0.30 s`보다 오래되지 않음

조건이 깨지면 즉시 0 속도를 한 번 이상 발행하고 이후 명령을 차단한다. gate가
종료되어 출력이 끊겨도 go1_driver의 기존 `0.35 s` watchdog이 정지를 담당한다.

### 5.5 `rviz_goal_bridge.py`

`READY` 전에는 RViz goal을 거부하고 이유를 로그로 남긴다. `DEGRADED` 또는
`LOST`가 되면 진행 중 `NavigateToPose` goal을 취소한다.

### 5.6 통합 launch와 설정

`go1_posegraph_navigation.launch.py`는 다음을 하나의 dry-run 진입점으로 제공한다.

- 입력 파일과 지도 이미지 사전 검증
- pointcloud-to-laserscan
- SLAM Toolbox localization
- 정리된 2D map server
- Nav2
- supervisor, quality evaluator, cmd gate, RViz goal bridge
- 선택적 go1_driver, 기본 `arm:=false`
- 선택적 localization 진단 rosbag 기록

`slam_toolbox_localization_hanyang_9f.yaml`은 저장 pose graph를 사용하며 map/odom/base
frame을 각각 `map`, `camera_init`, `body`로 고정한다. 초기 탐색은 위치 `±3 m`,
방향 `±90 deg`를 포괄하도록 coarse search 설정을 확장하되 scan 처리율을 제한해
초기 계산 부하를 통제한다.

## 6. 상태와 오류 계약

기본 판정값은 Windows 코드와 ROS 설정에서 한 곳의 상수로 공유한다.

- 입력 토픽 최소 처리율: `5 Hz`
- 입력 및 TF 최대 age: `0.50 s`
- 초기 정합 전체 timeout: `20 s`
- 자동 초기 재시도: `3회`
- VERIFYING 최소 지속시간: `3 s`
- 최소 scan-map overlap: `0.45`
- 최대 연속 위치 jump: `0.30 m`
- 최대 연속 방향 jump: `10 deg`
- ready heartbeat timeout: `0.30 s`

오류 코드는 다음으로 고정한다.

- `INPUT_MISSING`
- `POSE_OUTSIDE_MAP`
- `ALIGNMENT_TIMEOUT`
- `LOW_OVERLAP`
- `AMBIGUOUS`
- `ODOM_RESET`
- `TF_CONFLICT`
- `EXTRINSIC_UNCALIBRATED`

`READY`가 아닌 모든 상태에서 실제 속도 전달은 금지한다. 모호한 후보는 실패로
간주하며 잘못된 위치를 선택하지 않는다.

## 7. 기존 코드 리뷰 반영

### 즉시 수정

- controller 출력과 실제 driver 입력 사이에 localization safety gate를 둔다.
- FAST-LIO topic/TF의 숨은 시작 순서를 명시적 preflight 상태로 바꾼다.
- `BaseObstacle.scale`을 `0.01`에서 Nav2 기본 수준인 `0.02`로 복구한다.
- localization, controller, costmap의 TF tolerance 기본값을 `0.50 s`로 맞춘다.
- 현재 `robot_radius: 0.25` 대신 보수적인 footprint 설정을 사용한다.
- source text 존재 여부만 검사하는 테스트를 실제 설정 파싱과 동작 테스트로
  보강한다.
- primary pose-graph 모드와 AMCL fallback이 동시에 TF를 발행하지 못하게 한다.

### 실측 전 보수적으로 유지

- LiDAR 장착이 고정되지 않았으므로 `extrinsic_est_en: true` 프로필은 dry-run에서만
  허용한다. 실제 구동 요청은 `EXTRINSIC_UNCALIBRATED`로 거부한다.
- `velocity_smoother`의 `OPEN_LOOP`는 FAST-LIO twist 지연 자료가 없으므로 유지한다.
- `use_composition: false`는 Windows 정적 검증 범위에서 유지하고 launch 인자로
  노출한다.
- 보행 pitch에 따른 바닥 유입은 중력 정렬 scan frame 인터페이스와 검증 코드를
  추가하되, 높이 임계값의 최종값은 현장 기록으로 판단한다.

## 8. 진단 기록

`record_localization:=true`일 때 timestamp 디렉터리에 다음 토픽을 기록한다.

- `/scan`, `/Odometry`, `/tf`, `/tf_static`
- `/initialpose`, `/slam_toolbox/pose`
- `/localization_supervisor/status`, `/localization_supervisor/ready`
- `/cmd_vel_nav`, `/cmd_vel`

대용량 `/cloud_registered_body`는 기본 기록에서 제외하고 launch 인자로 선택할 수
있게 한다. 상태 전이와 품질 수치는 동일 세션의 CSV 요약으로도 남긴다.

## 9. Windows 및 WSL 검증 전략

이번 작업의 완료 기준은 다음과 같다.

- 기존 Windows 기준 테스트 `36 passed, 1 skipped`가 회귀하지 않는다.
- 상태 머신의 정상, timeout, retry, degraded, lost, recovery 분기가 모두 테스트된다.
- 합성 occupancy map과 scan으로 정상 정합, 지도 밖, 낮은 overlap, 대칭 모호성이
  테스트된다.
- safety gate 코어가 READY 이외 상태와 stale heartbeat에서 항상 0 속도를 반환한다.
- launch/YAML/package manifest가 Python에서 파싱되고 frame, topic, 안전 기본값 계약을
  만족한다.
- ROS2가 없는 Windows에서도 import 가능한 코어 모듈과 ROS2 진입점을 분리한다.
- README와 실행 문서는 `arm:=false` dry-run까지만 안내하고 Jetson 성공을 주장하지
  않는다.

WSL 검증은 저장소 안에 `build/`, `install/`, `log/`를 만들지 않도록 WSL의 임시
작업공간에 필요한 패키지만 복사해 실행한다.

- `/opt/ros/humble/setup.bash`를 source한 clean shell에서 의존 패키지를 확인한다.
- `go1_driver`, `omx_navigation`을 `colcon build --symlink-install`로 빌드한다.
- `colcon test`와 `colcon test-result --verbose`에서 실패 0건을 확인한다.
- 설치된 launch에 대해 `ros2 launch ... --show-args`를 실행해 Python launch import,
  package share 조회, 기본 인자 구성을 확인한다.
- 센서 토픽과 Jetson 전용 Unitree SDK가 필요한 실제 node 실행은 하지 않는다.

## 10. 위험과 제한

- 저장된 실패 rosbag이 없어 `±3 m`, `±90 deg` 실기 성능은 이번 세션에서 증명할
  수 없다.
- SLAM Toolbox matcher 내부 응답을 직접 노출하지 않으므로 독립적인 scan-map
  overlap과 후보 모호성 검사를 병행한다.
- LiDAR 장착과 extrinsic이 바뀌면 올바른 소프트웨어도 안정적인 정합을 보장할 수
  없다. 따라서 현재 상태에서는 실제 구동을 허용하지 않는다.
- 긴 대칭 복도에서는 정답 자세와 잘못된 자세의 scan 점수가 비슷할 수 있다.
  이 경우 시스템은 READY 대신 `AMBIGUOUS`를 내도록 설계한다.
