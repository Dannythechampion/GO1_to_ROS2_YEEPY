# FAST-LIO 실시간 버퍼 및 AMCL 시간 정합 설계

작성일: 2026-08-13
대상: Jetson ROS 2 Humble, Livox MID-360, FAST_LIO_ROS2, Nav2 AMCL
안전 조건: 전체 작업에서 Go1 제어는 `arm=false`를 유지하며 이동 명령을 활성화하지 않는다.

## 1. 목표

LiDAR 입력이 정상적인 10 Hz로 들어오는데도 FAST-LIO `/Odometry`, 파생 `/scan`,
AMCL `/amcl_pose`가 현재 시각보다 지속적으로 늦어지는 문제를 제거한다. 최종적으로
정지한 Go1에서 AMCL 위치 추정을 수행할 때 센서·odometry·TF의 시간 정합이 0.30초
안에 유지되어야 한다.

작업 순서는 다음과 같다.

1. FAST-LIO 내부 경계에서 큐 깊이와 프레임 나이를 계측해 지연 위치를 확인한다.
2. 확인된 내부 backlog만 제거하는 최소 변경을 적용한다.
3. 코드 빌드와 단위 테스트를 통과시킨다.
4. LiDAR/FAST-LIO를 10분간 연속 측정한다.
5. 기존 지도와 겹치는 장소에서 AMCL 위치 추정 및 시간 정합을 검증한다.

초기 위치 파일 저장, Nav2 목표 전송, 실제 주행은 이번 범위에 포함하지 않는다.

## 2. 현재 증거와 원인 가설

2026-08-13 Jetson 실측에서 짧은 구간에는 `/Odometry`, `/scan`, `/amcl_pose` 지연이
약 0.03초였지만, 장시간 관찰 중 `/Odometry`가 약 1.35초, `/scan`이 약 1.37초까지
늦어진 뒤 과거 시각을 일정하게 따라가는 현상이 재현되었다. `/Odometry`의 header
stamp 간격은 약 10 Hz로 고르게 유지되어, 출력 중단이 아니라 고정 길이 backlog를
처리하는 패턴이었다.

같은 세션에서 Livox packet은 호스트 수신 시각과 사실상 일치했고 MID-360은
`time_type=0`(NoSync) 호스트 timestamp 경로를 사용했다. 따라서 PTP나 센서 시계
offset은 주원인에서 제외한다. Livox driver 발행 큐와 raw packet 큐, DDS 구독 큐를
줄인 뒤에도 장시간 지연이 재발했으므로 남은 경계는 FAST-LIO 내부 큐다.

Jetson의 실제 `laserMapping.cpp`에는 다음 구조가 있다.

- `lidar_buffer`와 `time_buffer`는 제한 없는 `std::deque`다.
- LiDAR callback은 매 프레임을 `push_back()`한다.
- `sync_packages()`는 항상 `front()`를 선택한 뒤 IMU가 준비되면 하나만 제거한다.
- DDS `SensorDataQoS().keep_last(1)`은 아직 callback이 읽지 않은 DDS 표본만 제한하고,
  callback이 이미 내부 deque로 옮긴 표본은 제거하지 않는다.
- LiDAR timestamp 역행 시 현재 코드는 `lidar_buffer`만 비우고 `time_buffer`는 비우지
  않아 두 큐의 짝 불변식이 깨질 수 있다.

검증할 단일 가설은 다음과 같다.

> LiDAR callback 처리량이 scan 동기화/추정 처리량보다 순간적으로 앞서면서
> FAST-LIO 내부 deque에 쌓인 과거 프레임이 제거되지 않고, 모든 후속 odometry와
> scan 및 AMCL을 같은 양만큼 늦춘다.

## 3. 선택한 접근과 대안

### 선택: in-flight 1개 + 최신 대기 1개

IMU 동기화를 이미 기다리는 front 프레임은 보존하고, 그 뒤의 대기 프레임은 가장
최신 한 개만 유지한다. 총 LiDAR 내부 큐 깊이는 정상 상태에서 최대 2다. 계측을 먼저
넣어 가설을 확인한 후 같은 경계에 정책을 적용하므로 원인과 수정 효과를 분리해
판단할 수 있다.

이 방식은 scan 일부를 의도적으로 건너뛸 수 있지만, 로봇 위치 추정에서는 모든 과거
프레임 처리보다 현재 시각에 가까운 상태가 더 중요하다. MID-360 10 Hz 입력에서
일시적인 처리 초과가 무한 지연으로 바뀌지 않는다.

### 대안 1: MID-360 발행률 또는 point 수 감소

연산 부하는 줄지만 지도 정합에 사용할 기하 정보도 줄고, 내부 큐가 무제한이라는
원인을 그대로 남긴다. 선택한 정책 적용 후 실제 처리 시간이 100 ms를 지속적으로
초과할 때만 별도 튜닝 후보로 남긴다.

### 대안 2: FAST-LIO executor와 callback group 재설계

LiDAR preprocessing, IMU, mapping을 멀티스레드로 분리할 수 있지만 공유 전역 상태가
많아 변경 범위와 회귀 위험이 크다. 현재 목표에는 필요하지 않다.

## 4. 버퍼 정책과 동시성

`lidar_buffer[i]`와 `time_buffer[i]`는 항상 같은 scan을 나타내야 한다. 모든 삽입,
제거, 초기화는 하나의 mutex 아래에서 두 deque에 함께 수행한다. 현재 실행기는 단일
스레드지만 이 규칙으로 향후 executor 변경에도 데이터 경합과 짝 불일치를 막는다.

정책은 다음과 같다.

1. `lidar_pushed == false`이면 아직 IMU 동기화 중인 scan이 없다. 새 scan을 넣기 전에
   기존 대기 scan을 모두 제거하고 최신 scan 한 개만 둔다.
2. `lidar_pushed == true`이면 index 0은 IMU를 기다리는 in-flight scan이다. 이를
   제거하지 않고 index 1 이후는 모두 버린 다음 최신 scan을 index 1에 둔다.
3. in-flight scan이 현재 시각보다 0.20초 이상 오래되고 더 최신 대기 scan이 있으면,
   아직 IMU를 소비하지 않은 in-flight scan을 포기하고 최신 scan으로 교체한다.
4. timestamp 역행 시 LiDAR deque 두 개를 함께 비우고 `lidar_pushed`를 reset한다.
5. `sync_packages()`는 필요한 LiDAR와 IMU 표본을 mutex 아래에서 하나의
   `MeasureGroup`으로 분리하고, EKF·KD-tree·point cloud 처리는 lock 밖에서 수행한다.
6. IMU deque는 선택된 최신 scan의 종료 시각까지 기존 방식대로 소비한다. 건너뛴
   LiDAR보다 오래된 IMU는 다음 선택 scan을 동기화할 때 자연스럽게 제거한다.

0.20초는 10 Hz LiDAR 두 주기에 해당하고, 전체 localization safety gate의 최대 허용
나이 0.30초보다 작다. 이 값은 launch parameter로 노출하지 않고 이번 고정 센서
구성의 compile-time 정책으로 둔다. 실측이 처리량 부족을 보이면 버퍼 한도를 늘리는
대신 처리 부하를 별도 원인으로 분석한다.

## 5. 계측

FAST-LIO가 1 Hz 이하의 throttled 상태 로그를 남긴다. 대용량 CustomMsg를 외부 Python
subscriber로 관찰하면 관찰자 자체가 지연될 수 있으므로, 계측은 내부의 작은 숫자만
사용한다.

각 로그 표본에는 다음 필드가 포함된다.

- 현재 `lidar_buffer` 깊이
- 현재 front header age
- 누적 dropped/replaced LiDAR frame 수
- 직전 scan 처리 시간과 관찰 구간 최대 처리 시간
- `last_timestamp_imu - lidar_end_time` 동기화 여유

계측 로그는 경고 남발을 피하고, 큐 깊이가 2를 초과하거나 front age가 0.30초를
초과할 때만 warning으로 올린다. 정상 표본은 info 수준이다. 이 계측은 수정 전 가설
확인과 수정 후 soak 결과를 같은 정의로 비교하는 데 사용한다.

## 6. 배포 방식

이 저장소는 FAST_LIO_ROS2 원본을 vendoring하지 않고 고정 upstream commit을 Jetson
workspace에 checkout한다. 따라서 `migration/patch_fast_lio_low_latency.py`를 확장해
고정 commit의 알려진 source block을 검증하고 변환한다.

패치 스크립트는 다음 성질을 가져야 한다.

- 원본, 기존 DDS-only 패치 상태, 최종 상태를 구분한다.
- 최종 상태에 재실행하면 변경 없이 성공한다.
- 예상 source block의 개수나 내용이 다르면 부분 수정 없이 실패한다.
- source backup은 Jetson 배포 절차에서 계속 보존한다.
- Livox driver queue patch와 독립적으로 테스트하고 빌드한다.

## 7. 테스트

### 자동 단위 테스트

패치 fixture는 실제 pinned `laserMapping.cpp`의 관련 callback, deque, sync block을
포함한다. 다음을 먼저 실패하는 테스트로 추가한 뒤 구현한다.

- 내부 queue가 in-flight가 없을 때 최신 1개만 유지한다.
- in-flight가 있을 때 front와 최신 대기 1개만 유지한다.
- stale in-flight는 최신 대기가 있을 때만 교체한다.
- scan drop 시 `lidar_buffer`와 `time_buffer`가 항상 같은 크기다.
- timestamp 역행 시 두 deque와 in-flight 상태가 함께 reset된다.
- patch는 idempotent이며 예상하지 않은 upstream source를 거부한다.
- 생성 코드에 DDS sensor QoS depth 1과 1 Hz 계측이 모두 남는다.

기존 migration 및 `omx_navigation` 전체 pytest도 실행해 QoS, AMCL tolerance,
localization supervisor 회귀가 없는지 확인한다.

### Jetson 빌드 검증

`colcon build --packages-up-to fast_lio`를 깨끗하게 통과하고, build 후 실제 설치된
binary가 새 source로 생성됐는지 build timestamp와 process 재시작 시각을 기록한다.
Livox와 FAST-LIO는 각각 한 인스턴스만 실행해야 한다.

## 8. 실기 검증과 합격 기준

Go1은 평평한 곳에서 움직이지 않게 두고 `arm=false`를 유지한다. LiDAR와 Jetson만으로
FAST-LIO 측정은 가능하다. AMCL 정합은 불러온 기존 지도에 포함된 장소에서 수행한다.

### 단계 A: LiDAR/FAST-LIO 10분 soak

처음 60초 초기화 구간과 이후 9분을 함께 기록하되, 합격 통계는 초기화 완료 후
연속 9분을 사용한다.

- `/Odometry`와 `/scan` 발행률: 각 9~11 Hz
- 두 토픽 header age: p95 0.10초 이하, 최대 0.30초 이하
- FAST-LIO 내부 큐 깊이: 2 이하이며 시간에 따라 증가하지 않음
- 처리 시간: p95 100 ms 이하; 일시 초과가 있어도 backlog로 누적되지 않음
- timestamp 역행: 0회
- FAST-LIO/Livox process restart 또는 중복 publisher: 0회

### 단계 B: AMCL 시간 정합

기존 지도와 겹치는 장소에서 수동 `2D Pose Estimate`를 한 번 지정하고 3분 이상
정지 관찰한다.

- `/amcl_pose` header age: p95 0.10초 이하, 최대 0.30초 이하
- `map -> camera_init -> body` TF lookup failure/extrapolation: 0회
- costmap의 sensor origin 또는 stale scan drop: 0회
- 정지 속도: 선속도와 각속도 각각 0.01 이하
- AMCL covariance: x/y 각각 0.04 이하, yaw 0.0305 이하
- 10개 연속 표본의 위치 spread 0.10 m 이하, yaw spread 5도 이하
- scan-map residual: 기존 localization supervisor 기준을 통과

한 항목이라도 실패하면 초기 위치 저장이나 주행 단계로 넘어가지 않는다. 내부 큐가
안정적인데 처리 시간이 지속적으로 100 ms를 넘으면 point 수/voxel/CPU 부하를 다음
단일 원인으로 분리 측정한다. 내부 큐는 안정적이지만 AMCL만 늦으면 scan projection,
TF, AMCL callback 경계를 차례로 계측한다.

## 9. 완료 조건과 산출물

완료에는 다음 증거가 모두 필요하다.

- 커밋된 migration patch 및 단위 테스트
- 로컬 전체 테스트 성공 기록
- Jetson FAST-LIO build 성공 기록
- 수정 전후 내부 계측 비교
- 10분 LiDAR/FAST-LIO soak 원시 로그와 요약
- 3분 AMCL 시간 정합 로그와 요약
- 모든 안전 검증 동안 `arm=false`, 속도 0 확인

이 증거가 없으면 짧은 순간의 낮은 지연만으로 해결됐다고 판단하지 않는다.
