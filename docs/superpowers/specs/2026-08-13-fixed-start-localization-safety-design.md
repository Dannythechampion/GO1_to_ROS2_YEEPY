# Go1 고정 출발점 자동 Localization 및 주행 안전 게이트 설계

작성일: 2026-08-13
대상 환경: Ubuntu 22.04, ROS 2 Humble, Unitree Go1, Livox MID-360, FAST-LIO, Nav2

## 1. 목적

Go1을 매번 동일한 바닥 타일에 앞발과 진행 방향을 맞춰 배치하는 운용을 전제로 한다. RViz의 `2D Pose Estimate`를 필수 절차에서 제거하고, 로봇이 완전히 정지한 상태에서 초기 위치를 자동 설정하고 검증한다.

초기 위치가 설정됐다는 사실만으로 주행을 허용하지 않는다. 센서, TF, 지도-스캔 정합 및 localization 신뢰도가 모두 검증된 동안에만 실제 Go1으로 속도 명령을 전달한다. 검증 실패 또는 운용 중 localization 상실 시에는 fail-closed 방식으로 `STAND` 명령을 유지한다.

## 2. 범위

### 포함

- 지도 좌표계의 고정 출발 자세를 파일로 관리
- RViz 없이 `/initialpose` 자동 발행
- Go1을 움직이지 않는 정지 상태 검증
- 센서/TF/localization 상태를 종합하는 supervisor
- localization과 E-stop 상태에 따른 속도 명령 차단
- localization 완료 전 RViz goal 거부
- AMCL 재시작 또는 FAST-LIO 재시작 감지와 자동 차단
- 초기 좌표를 나중에 측정할 수 있는 commissioning 절차와 안전한 미설정 상태
- dry-run, rosbag 재생 및 실제 로봇 단계별 검증

### 제외

- 지도 내 임의 위치에서의 global localization
- AprilTag, UWB, GNSS 등 외부 절대 위치 센서
- 로봇의 자동 초기 회전 또는 직선 이동
- 기존 지도 재작성
- DWB 성능 튜닝, footprint 교정 및 FAST-LIO extrinsic 교정 자체

마지막 세 항목은 별도의 안전 개선 작업으로 진행하되, 본 설계의 motion gate를 선행 조건으로 한다.

## 3. 운용 가정

- 앞발 두 개를 지정 타일 표시에 맞추고 몸체 방향선을 따라 배치한다.
- 배치 오차의 초기 설계 범위는 평면 위치 ±0.15 m, yaw ±10 deg이다.
- localization 중 Go1은 `STAND` 상태를 유지하며 자발적으로 움직이지 않는다.
- Livox와 FAST-LIO가 정상 초기화된 후 localization을 시작한다.
- 운용 localization은 AMCL만 사용한다.
- 물리 리모컨 또는 독립 E-stop 담당자는 기존 현장 안전 절차대로 유지한다. 소프트웨어 게이트는 물리 E-stop을 대체하지 않는다.

## 4. 대안과 결정

### 대안 A: AMCL 파라미터에 초기 좌표만 설정

구현은 가장 작지만, 잘못 배치되거나 잘못된 프리셋을 선택해도 주행이 허용될 수 있다. localization 성공 여부와 실제 구동 허가가 분리되므로 채택하지 않는다.

### 대안 B: 고정 자세 seed + 정지 검증 + motion gate

출발 자세를 자동 발행한 뒤 지도-스캔 정합, AMCL covariance, TF와 토픽 신선도를 검증한다. 성공 전에는 실제 속도 명령을 차단한다. 추가 센서 없이 현재 하드웨어로 구현 가능하고 실패 시 안전하게 정지할 수 있으므로 채택한다.

### 대안 C: 카메라와 AprilTag 기반 절대 자세

절대 기준이 명확하지만 카메라, 마커, 조명과 시야 관리가 추가된다. 타일 기반 배치와 대안 B의 현장 시험이 충분하지 않을 때 후속 단계로 검토한다.

## 5. 전체 구조

```text
start_pose.yaml
       |
       v
localization_supervisor ---- /initialpose ----> AMCL
       |                         |
       |                         +---- /amcl_pose
       |
       +---- /scan, /Odometry, TF, map
       |
       +---- /localization/ready       (10 Hz heartbeat)
       +---- /localization/state       (상태와 실패 이유)
                         |
/cmd_vel_nav ------------v
                    motion_gate <----- /safety/estop
                         |
                         +---- /cmd_vel_safe ----> go1_driver ----> Go1 UDP
```

Nav2 controller 출력은 `/cmd_vel_nav`로 remap한다. `go1_driver`는 `/cmd_vel_safe`만 구독한다. 따라서 Nav2 또는 다른 노드가 `/cmd_vel`을 발행해도 실제 로봇으로 직접 전달되지 않는다.

방어를 두 겹으로 구성한다.

1. `motion_gate`가 준비되지 않은 상태에서 0 속도만 발행한다.
2. `go1_driver`는 gate의 내부 상태를 해석하지 않고 `/cmd_vel_safe` timeout만 독립적으로 검사한다. gate가 종료되거나 출력이 끊기면 0.35 s 이내 `STAND`를 전송한다.

## 6. 구성 요소

### 6.1 `start_pose.yaml`

프리셋 하나는 다음 필드를 가진다.

| 필드 | 형식 | 값의 출처 |
| --- | --- | --- |
| `frame_id` | string | 항상 `map` |
| `pose.x` | double, m | 최초 commissioning에서 측정한 10회 자세의 평균 |
| `pose.y` | double, m | 최초 commissioning에서 측정한 10회 자세의 평균 |
| `pose.yaw` | double, rad | 최초 commissioning에서 측정한 10회 자세의 circular mean |
| `covariance.x` | double, m² | 최초값 `0.0225` |
| `covariance.y` | double, m² | 최초값 `0.0225` |
| `covariance.yaw` | double, rad² | 최초값 `0.0305` |

초기 variance는 위치 표준편차 0.15 m, yaw 표준편차 10 deg에 해당한다. 실제 배치 반복 시험 결과가 더 작더라도 최초 현장 검증 전에는 이보다 좁게 설정하지 않는다.

좌표는 최초 설치 시 기존 RViz 절차로 측정한다. 동일 타일에 10회 재배치해 얻은 자세의 평균을 프리셋 값으로 저장하고, 최대 오차가 설계 범위를 넘으면 바닥 표시를 개선한 뒤 다시 측정한다.

초기 좌표가 아직 없는 개발 단계에서는 launch 인자 `start_pose_file`을 빈 값으로 둔다. 이 경우 supervisor는 `UNCOMMISSIONED` 상태와 `ready=false`만 발행하며 `/initialpose`를 자동 발행하지 않는다. `arm:=false` dry-run은 허용하지만 `arm:=true`는 launch 단계에서 거부한다. 프리셋 파일이 지정됐지만 파싱 또는 스키마 검증에 실패한 경우에는 arm 값과 관계없이 launch를 실패시킨다.

### 6.2 초기 자세 commissioning

초기 위치 측정은 구현 완료를 막지 않으며, 실제 현장 적용 전에 다음 절차로 수행한다.

1. `arm:=false`, `start_pose_file:=""`로 bringup하고 기존 RViz `2D Pose Estimate`로 수동 localization한다.
2. Go1을 지정 타일에 맞춘 각 배치에서 AMCL covariance와 지도-스캔 정합이 안정된 뒤 commissioning 도구로 자세 한 개를 기록한다.
3. 로봇을 타일에서 완전히 치웠다가 다시 배치하는 절차를 총 10회 반복한다.
4. commissioning 도구는 x/y 산술 평균, yaw circular mean, 최대 위치 편차 및 최대 yaw 편차를 계산한다.
5. 최대 위치 편차가 0.15 m 또는 최대 yaw 편차가 10 deg를 넘으면 프리셋 생성을 거부한다.
6. 통과하면 명시적으로 지정한 출력 경로에 `start_pose.yaml`을 생성한다.
7. 생성된 파일을 `start_pose_file:=`로 전달하고 `arm:=false` 자동 localization 시험을 10회 통과한 뒤에만 armed 시험으로 진행한다.

commissioning 표본과 생성된 운영 좌표는 현장별 데이터이므로 소스 저장소에 기본값으로 커밋하지 않는다. 테스트에서는 별도의 fixture 프리셋을 사용한다.

### 6.3 `localization_supervisor`

단일 책임은 초기 자세 공급과 localization 상태 판정이다. 속도 명령은 처리하지 않는다.

입력:

- `/map`
- `/scan`
- `/Odometry`
- `/amcl_pose`
- TF `camera_init -> body`
- TF `map -> camera_init`

출력:

- `/initialpose`: transient-local `PoseWithCovarianceStamped`
- `/localization/ready`: 10 Hz `std_msgs/Bool` heartbeat
- `/localization/state`: transient-local `std_msgs/String`
- `/diagnostics`: 표준 diagnostic 상태

서비스:

- `/localization/reseed`: 현재 프리셋을 다시 발행하고 검증을 재시작
- `/localization/reset_fault`: 원인이 제거된 뒤 `FAULT`에서 `WAIT_INPUTS`로 복귀

`ready=true`는 latched 신호로 사용하지 않는다. 10 Hz heartbeat로 발행하고 소비자는 마지막 수신 후 0.30 s가 지나면 준비되지 않은 것으로 처리한다.

### 6.4 정지 상태 지도-스캔 검증기

AMCL covariance만으로 잘못된 지도 위치 수렴을 판정할 수 없으므로 독립적인 2D 정합 점수를 계산한다.

1. occupancy map의 occupied cell로 2D distance field를 만든다.
2. 최신 `map -> body` 후보 자세로 유효한 scan endpoint를 map 좌표로 변환한다.
3. 각 endpoint와 가장 가까운 occupied cell 사이의 거리를 계산한다.
4. 2초 동안 최소 10개 scan의 통계를 누적한다.

최초 기본 통과 기준:

- scan당 유효 beam 100개 이상
- 10개 이상 연속 scan 수집
- endpoint 거리 중앙값 0.15 m 이하
- endpoint 거리 80 percentile 0.30 m 이하
- AMCL 추정 자세와 프리셋의 평면 거리 0.25 m 이하
- AMCL 추정 yaw와 프리셋 yaw의 최단 각도 차이 15 deg 이하
- 위 조건을 2초 동안 연속 만족

문, 유리, 사람과 가구 변화의 영향을 고려해 한 프레임의 결과만으로 통과시키지 않는다. 이 값들은 rosbag과 현장 dry-run에서 false accept가 0회가 되도록 더 엄격한 방향으로만 조정한 후 운용한다.

### 6.5 `motion_gate`

`motion_gate`는 `go1_driver` 내부 기능이 아니라 `omx_navigation` 패키지의 독립 ROS 2 노드로 구현한다. Nav2와 하드웨어 driver 사이의 토픽 경계를 강제하며, driver에는 localization 판정이나 Nav2 의존성을 추가하지 않는다. 노드가 종료되면 `/cmd_vel_safe` 발행이 끊기고 기존 driver watchdog이 `STAND`로 전환한다.

입력:

- `/cmd_vel_nav`
- `/localization/ready`
- `/safety/estop`

출력:

- `/cmd_vel_safe`
- `/motion_gate/state`

전달 조건은 다음을 모두 만족해야 한다.

- localization heartbeat가 `true`이고 0.30 s 이내에 수신됨
- E-stop이 해제됨
- 입력 명령이 0.25 s 이내에 수신됨
- 값이 finite이고 설정 속도 제한 이내임

조건 하나라도 실패하면 즉시 0 속도를 발행한다. E-stop은 latch 방식이며 프로세스 시작 시 기본값은 정지 상태다. 해제는 명시적 service 호출로만 가능하다. localization이 한번 상실된 뒤 회복되더라도 자동으로 E-stop을 해제하거나 이전 goal을 재개하지 않는다.

### 6.6 `go1_driver` 보강

- 기본 입력 토픽을 `/cmd_vel_safe`로 변경한다.
- `arm`은 UDP 사용 여부만 결정하고 안전 준비 상태를 의미하지 않도록 문서화한다.
- localization과 E-stop 판정은 별도 `motion_gate`에만 둔다.
- `/cmd_vel_safe` timeout과 비정상 수치 발생 시 `MotionCommand.stand()`를 생성한다.
- 정지 원인을 `/go1/control_state`에 구조적으로 기록한다.
- shutdown 시 기존 repeated stand 동작을 유지한다.

### 6.7 RViz goal bridge

`/localization/ready`가 유효하지 않으면 goal을 action server로 전달하지 않고 거부 이유를 로그로 남긴다. 이는 사용자 경험을 위한 추가 방어이며, 실제 안전 보장은 motion gate와 driver가 담당한다.

## 7. 상태 머신

```text
BOOT
  -> UNCOMMISSIONED
  -> WAIT_INPUTS
  -> SEEDING
  -> VALIDATING
  -> READY
  -> FAULT
```

- `BOOT`: 설정 파일과 파라미터 검증
- `UNCOMMISSIONED`: start pose가 없으며 heartbeat false를 유지하는 dry-run 전용 상태
- `WAIT_INPUTS`: `/map`, `/scan`, `/Odometry`, `camera_init -> body`를 기다림
- `SEEDING`: `/initialpose`를 발행하고 AMCL 응답을 기다림
- `VALIDATING`: TF, covariance 및 지도-스캔 정합을 연속 검증
- `READY`: heartbeat true를 발행하며 조건을 계속 감시
- `FAULT`: heartbeat false를 발행하고 원인 제거 및 명시적 reset을 기다림

`READY` 중 다음 사건은 즉시 `FAULT`로 전환한다.

- `/scan` 또는 `/Odometry`가 0.30 s 이상 stale
- `camera_init -> body` 또는 `map -> camera_init` TF lookup 실패
- FAST-LIO 재시작으로 `camera_init` 원점이 불연속적으로 변경됨
- AMCL covariance가 1초 이상 허용치를 초과
- 지도-스캔 정합이 1초 이상 실패
- AMCL 노드 종료

FAULT 전환 시 진행 중 goal을 취소하고 motion gate와 driver는 `STAND`를 유지한다. 자동 reseed 및 자동 goal 재개는 하지 않는다.

## 8. AMCL 판정 기준

1차 구현의 AMCL 통과 기준은 다음과 같다.

- `/amcl_pose`가 seed 발행 뒤 생성됨
- `map -> camera_init -> body` TF chain이 존재함
- pose timestamp와 TF가 현재 시각 기준 0.30 s 이내임
- AMCL covariance `x <= 0.04 m^2`, `y <= 0.04 m^2`, `yaw <= 0.0305 rad^2`
- AMCL 추정 자세와 프리셋의 평면 거리 차이가 0.25 m 이하임
- AMCL 추정 yaw와 프리셋 yaw의 최단 각도 차이가 15 deg 이하임
- 지도-스캔 정합 기준을 2초 동안 연속 만족

정지 검증을 위해 AMCL 업데이트가 움직임 threshold에 막히지 않도록 startup 운용 설정에서는 `update_min_d`와 `update_min_a`를 0으로 설정한다. Jetson 부하가 증가하면 localization supervisor가 READY가 된 뒤 원래 운용값으로 되돌리는 동적 변경 대신, 매 scan 업데이트를 유지한 상태의 CPU 사용률을 먼저 측정한다. 동적 파라미터 전환은 1차 범위에 포함하지 않는다.

## 9. Launch 순서

단일 bringup이 다음 순서를 보장한다.

1. Livox 및 FAST-LIO 입력 존재 확인
2. map 파일과 start pose 설정을 검증하고, 미설정 dry-run이면 `UNCOMMISSIONED`로 제한
3. AMCL, supervisor 및 motion gate 시작
4. Nav2를 시작하되 controller 출력을 `/cmd_vel_nav`로 remap
5. Go1 driver를 `/cmd_vel_safe` 입력으로 시작
6. supervisor가 READY를 선언할 때까지 driver는 STAND 유지
7. 운영자가 localization 상태를 확인하고 E-stop을 명시적으로 해제
8. 이후에만 goal 입력 허용

`arm:=true`여도 6~7단계를 통과하지 않으면 걷기 명령은 전달되지 않는다. `start_pose_file`이 비어 있으면 `arm:=true` launch 자체를 거부한다.

## 10. 오류 처리와 복구

- 프리셋 미설정 + `arm:=false`: `UNCOMMISSIONED`, ready false, 자동 `/initialpose` 없음
- 프리셋 미설정 + `arm:=true`: launch 실패, UDP driver 시작 안 함
- 프리셋 파싱/스키마 오류: launch 실패
- 지도 없음: launch 실패
- 센서 또는 TF 미준비: `WAIT_INPUTS`, heartbeat false
- 정합 실패: `FAULT`, 프리셋 및 배치 확인 요구
- FAST-LIO 재시작: `FAULT`, goal 취소, 재배치 확인 후 수동 reseed
- AMCL 재시작: `FAULT`, 자동 goal 재개 금지
- supervisor 종료: motion gate가 heartbeat timeout을 감지해 0.30 s 이내 STAND
- motion gate 종료: driver의 `/cmd_vel_safe` watchdog으로 0.35 s 이내 STAND
- driver 입력 timeout: 기존 0.35 s보다 짧거나 같은 값으로 STAND
- E-stop 요청: 다른 상태와 무관하게 즉시 STAND, 명시적 reset 전까지 latch

## 11. 시험 전략

### 11.1 단위 시험

- start pose 스키마와 covariance 검증
- 10회 commissioning 표본의 평균, circular mean 및 허용 편차 판정
- 미설정 프리셋의 `UNCOMMISSIONED` 전이와 armed launch 거부
- 상태 전이와 fault latch
- heartbeat timeout
- NaN/Inf 및 속도 제한
- distance field 기반 scan score 계산

### 11.2 Launch/contract 시험

- Nav2 출력이 `/cmd_vel_nav`로만 연결됨
- driver 입력이 `/cmd_vel_safe`임
- `arm` 기본값 false 유지
- preset 미설정 dry-run 허용 및 armed launch 거부
- 지정된 preset 또는 map이 유효하지 않으면 launch 실패
- localization 미완료 상태에서 goal 거부

### 11.3 rosbag 재생 시험

같은 시작 타일에서 확보한 정지 rosbag과 잘못 배치한 rosbag을 사용한다.

- 정상 배치 10회: 10회 모두 5초 안에 READY
- 위치를 0.30 m 이동한 오배치 5회: READY 0회
- yaw를 20 deg 틀린 오배치 5회: READY 0회
- `/scan`, `/Odometry`, TF 중 하나를 중단: 0.30 s 이내 ready false
- localization이 상실돼도 `/cmd_vel_safe`는 0만 발행

### 11.4 실제 로봇 시험

1. `arm:=false`로 10회 재배치 및 READY 성공률 확인
2. 오배치 시험에서 false READY가 없는지 확인
3. localization 전 Nav2 goal을 보내 `/cmd_vel_safe`가 0인지 확인
4. READY 후 E-stop이 latch된 동안 `/cmd_vel_safe`가 0인지 확인
5. `arm:=true`, 최대 0.05 m/s로 1 m 직선 주행
6. `/scan` 차단과 supervisor 종료 시험에서 0.30 s 이내 STAND 확인
7. 통과 후 현재 제한인 0.20 m/s까지 단계적으로 상승

실제 로봇 시험에는 물리 리모컨/E-stop 담당자와 충분한 안전 공간이 필수다.

## 12. 완료 기준

- 정상 타일 배치 시 RViz 조작 없이 5초 이내 READY
- 초기 좌표가 없어도 `arm:=false`로 전체 노드와 gate를 안전하게 시험할 수 있음
- localization 과정에서 Go1이 움직이지 않음
- 정상 배치 10회 모두 READY, 지정 오배치 10회 모두 READY 거부
- 프리셋과 AMCL 결과가 0.25 m 또는 15 deg를 초과하면 READY 거부
- localization, 센서 또는 TF 상실 후 0.30 s 이내 안전 정지
- localization 전과 E-stop latch 중 실제 UDP walk 명령이 생성되지 않음
- 기존 command filter, watchdog 및 shutdown stand 시험이 유지됨

## 13. 후속 작업

본 설계 구현과 현장 검증 후 별도 설계로 다음을 진행한다.

1. `base_footprint` 기반 평면 TF 분리
2. Go1 실측 polygon footprint 및 Collision Monitor
3. FAST-LIO extrinsic 고정
4. 사족보행 pitch에 맞춘 point-cloud 높이 필터 조정
5. DWB obstacle critic과 velocity smoother 튜닝
