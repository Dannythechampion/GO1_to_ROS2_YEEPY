# Go1 이중 Goal 입력 Mission Manager 설계

작성일: 2026-08-13

## 1. 목적

Go1의 목적지 입력을 RViz에만 의존하지 않도록 한다. 운영자는 터미널에서 명시적으로 `/mission/start`를 호출해 등록된 엘리베이터 앞 목적지로 주행하거나, RViz `2D Goal Pose`로 지도상의 임의 목적지를 지정할 수 있다.

두 입력은 모두 하나의 `fixed_mission_manager` ROS 2 노드를 통과한다. RViz가 Nav2 `NavigateToPose` action을 직접 호출하는 우회 경로는 허용하지 않는다.

## 2. 범위

### 포함

- 등록 목적지 한 개를 위한 `destination_pose.yaml`
- `ros2 service call /mission/start std_srvs/srv/Trigger "{}"`
- RViz `/goal_pose`의 임의 목적지
- 진행 중 임무 취소 서비스
- localization 및 motion gate 상태 검사
- 지도상 goal 유효성 검사
- 임무 상태와 결과 발행
- 주행 중 새 goal 거부
- 초기 목적지 좌표가 없는 안전한 미설정 상태

### 제외

- 복수 named goal
- 여러 waypoint 순차 주행
- 자동 순찰 또는 반복 임무
- 임무 예약과 큐
- localization READY 직후 자동 출발
- 새 RViz goal로 진행 중 goal 자동 교체
- 웹/모바일 UI

## 3. 용어와 Nav2 역할

- `fixed_mission_manager`: goal 입력, 검증, 임무 상태 및 취소를 관리한다.
- NavFn: 현재 AMCL 위치에서 목적지까지 global path를 만든다.
- DWB: global path를 따라가며 local obstacle avoidance와 속도 명령을 생성한다.
- `motion_gate`: Nav2가 생성한 속도 명령을 실제 Go1에 전달할지 독립적으로 판단한다.

출발점은 고정된 경로의 첫 좌표가 아니다. Go1을 지정 타일에 배치하고 AMCL이 확인한 현재 위치가 실제 경로의 시작이 된다.

## 4. 대안과 결정

### 대안 A: RViz bridge 유지 + 별도 고정 목적지 스크립트

두 입력이 서로 다른 action client를 가지므로 동시에 goal을 보내거나 서로의 goal을 취소할 수 있다. 상태와 안전 정책이 분산되므로 채택하지 않는다.

### 대안 B: 단일 `fixed_mission_manager`

등록 목적지 서비스와 RViz 임의 goal을 한 노드가 받고 하나의 `NavigateToPose` action client만 소유한다. 현재 필요한 단일 목적지와 수동 RViz 운용을 모두 충족하며 충돌 정책을 한곳에서 강제할 수 있어 채택한다.

### 대안 C: 범용 named-goal 및 waypoint manager

확장성은 높지만 현재 요구하지 않는 장소 목록, 큐, waypoint 실행 정책을 추가한다. 1차 범위에서는 제외한다.

## 5. 아키텍처

```text
destination_pose.yaml
        |
        v
/mission/start -----------+
                          |
RViz /goal_pose ----------+--> fixed_mission_manager
                                   |  - localization READY
/localization/ready -------------->|  - motion gate released
/motion_gate/enabled -------------->|  - goal validity
/map ------------------------------>|  - single-active-goal policy
                                   |
/mission/cancel ------------------->|
                                   v
                             NavigateToPose
                                   |
                                   v
                         NavFn -> DWB -> /cmd_vel_nav
                                   |
                                   v
                              motion_gate
```

기존 `rviz_goal_bridge`의 역할은 `fixed_mission_manager`로 흡수한다. `/goal_pose`에서 Nav2로 직접 이어지는 다른 subscriber/action bridge는 launch contract 시험에서 금지한다.

## 6. 목적지 설정

`destination_pose.yaml`은 다음 스키마를 사용한다.

```yaml
frame_id: map
name: elevator_front
pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
```

위 좌표 예시는 테스트 fixture에만 사용한다. 실제 운영 파일은 나중에 현장에서 측정해 `/mnt/t500/go1_runtime/destination_pose.yaml`에 저장하며 저장소에는 커밋하지 않는다.

launch 인자 `destination_pose_file`의 기본값은 빈 문자열이다.

- 빈 값: manager 상태는 `UNCONFIGURED_DESTINATION`; `/mission/start`는 실패한다.
- 빈 값이어도 RViz 임의 goal은 사용할 수 있다.
- 경로가 지정됐으나 파일이 없거나 스키마가 잘못됨: launch 실패.
- `frame_id`는 `map`만 허용하고 모든 pose 값은 finite여야 한다.

## 7. 입력 인터페이스

### 7.1 등록 목적지 시작

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

성공 응답은 goal이 Nav2에 accepted됐다는 뜻이며 도착 완료를 뜻하지 않는다. 최종 결과는 `/mission/status`에서 확인한다.

거부 조건:

- destination이 설정되지 않음
- localization heartbeat가 false 또는 0.30 s 이상 stale
- motion gate가 disabled 또는 0.30 s 이상 stale
- 이미 임무가 진행 중이거나 cancel 중임
- goal이 지도상 유효하지 않음
- Nav2 action server가 준비되지 않음

### 7.2 RViz 임의 목적지

RViz `2D Goal Pose`가 발행하는 `/goal_pose`를 manager가 구독한다. 등록 목적지와 같은 검사를 통과한 경우에만 Nav2에 전달한다. RViz goal은 `destination_pose_file` 유무와 관계없이 사용할 수 있다.

RViz 입력은 다음 경우 명시적 로그와 status reason으로 거부한다.

- `frame_id`가 없거나 `map`이 아님
- 위치 또는 quaternion이 finite가 아님
- quaternion norm이 유효 범위 밖임
- READY/gate/지도/action server 조건 미충족
- 다른 임무 진행 중

거부된 RViz goal은 큐에 저장하거나 나중에 자동 실행하지 않는다.

### 7.3 임무 취소

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
```

ACTIVE 상태에서만 action goal의 `cancel_goal_async()`를 호출한다. 취소 요청 즉시 상태를 `CANCELING`으로 바꾸고, Nav2 취소 응답 후 `CANCELED`로 전환한다. 취소 완료 전에 새 goal은 받지 않는다.

## 8. 출력 인터페이스

- `/mission/status`: transient-local `std_msgs/String`
- `/mission/active`: 10 Hz `std_msgs/Bool` heartbeat

status 문자열은 최소 다음 필드를 포함한다.

```text
state=ACTIVE source=FIXED destination=elevator_front reason=goal_accepted
state=REJECTED source=RVIZ destination=adhoc reason=mission_already_active
state=SUCCEEDED source=FIXED destination=elevator_front reason=nav2_succeeded
```

사람이 읽을 수 있으면서 test script에서 안정적으로 파싱할 수 있도록 key=value 형식을 고정한다.

## 9. Goal 유효성 검사

모든 goal은 Nav2 action 전송 전에 다음을 만족해야 한다.

- frame은 `map`
- x, y, quaternion은 finite
- 정규화된 quaternion이며 yaw를 계산할 수 있음
- OccupancyGrid bounds 내부
- goal cell이 free (`0 <= occupancy < 50`)
- unknown cell (`-1`) 또는 occupied cell (`>= 50`)이 아님
- goal 중심에서 occupied/unknown cell까지 최소 0.35 m 이상

마지막 clearance는 Go1의 현재 임시 외접 반경을 보수적으로 반영한다. 실제 polygon footprint가 확정되면 별도 안전 설계 결과로 갱신한다.

## 10. 상태 머신

```text
BOOT
  -> UNCONFIGURED_DESTINATION
  -> IDLE
  -> SENDING
  -> ACTIVE
  -> CANCELING
  -> SUCCEEDED | FAILED | CANCELED
```

- destination 미설정 상태에서도 RViz goal을 받을 준비가 되면 `UNCONFIGURED_DESTINATION`을 유지한다.
- `/mission/start`는 `IDLE` 또는 `UNCONFIGURED_DESTINATION`에서 호출할 수 있지만 후자에서는 설정 오류로 거부한다.
- RViz goal은 `IDLE`과 `UNCONFIGURED_DESTINATION`에서 받을 수 있다.
- `SENDING`, `ACTIVE`, `CANCELING`에서는 모든 새 goal을 거부한다.
- `SUCCEEDED`, `FAILED`, `CANCELED`는 마지막 결과를 보존하는 비활성 상태다. 다음 유효 goal을 수락할 때 `SENDING`으로 전환한다.
- `/mission/active`는 `SENDING`, `ACTIVE`, `CANCELING`에서만 true다.

## 11. 주행 중 안전 사건

ACTIVE 중 다음 사건이 발생하면 진행 goal을 취소한다.

- `/localization/ready` false 또는 heartbeat timeout
- `/motion_gate/enabled` false 또는 heartbeat timeout
- AMCL supervisor가 `FAULT`
- Nav2 action failure 또는 rejection

안전 사건으로 취소된 임무는 자동 재개하지 않는다. localization과 gate가 회복되더라도 운영자가 `/mission/start`를 다시 호출하거나 RViz goal을 다시 지정해야 한다.

`motion_gate`는 별도 ROS 2 노드로 유지되며 `/motion_gate/enabled` 10 Hz Bool heartbeat를 추가한다. 이는 mission manager가 goal 수락 여부를 판단하기 위한 상태이고 실제 속도 차단은 기존 `/cmd_vel_safe` 경로에서 독립적으로 수행한다.

## 12. 명시적 출발 절차

등록 목적지 운용 절차:

1. Go1을 지정 타일에 배치한다.
2. AMCL supervisor가 READY인지 확인한다.
3. motion gate E-stop을 명시적으로 해제한다.
4. `/mission/start` 서비스를 호출한다.
5. manager가 저장된 엘리베이터 앞 pose를 Nav2에 전달한다.
6. 도착 성공 또는 실패 상태를 `/mission/status`에서 확인한다.

RViz 임의 목적지 운용 절차:

1. 같은 localization 및 gate 준비 절차를 수행한다.
2. RViz `2D Goal Pose`로 임의의 free 위치와 방향을 지정한다.
3. manager가 goal을 검사하고 허용 시 Nav2에 전달한다.

Localization READY만으로 자동 출발하지 않는다.

## 13. Destination commissioning

도착점 좌표는 나중에 현장에서 다음 절차로 정한다.

1. `arm:=false`이고 motion gate E-stop이 latch된 상태로 지도와 RViz를 실행한다.
2. 엘리베이터 앞에서 Go1이 정지할 위치와 방향을 RViz `2D Goal Pose`로 선택한다.
3. destination recorder를 명시적으로 실행해 다음 `/goal_pose` 한 건을 지정 출력 파일에 저장한다. Gate가 disabled이므로 manager는 주행 goal을 거부하지만 recorder는 후보 pose를 저장한다.
4. recorder는 기존 파일을 덮어쓰지 않고 frame, finite 값, map cell 및 0.35 m clearance를 검증한다.
5. 생성된 파일로 `/mission/start` dry-run을 반복 검증한다.

등록 지점은 엘리베이터 문 바로 앞이 아니라 문 개폐 영역과 사람 대기 공간을 침범하지 않는 안전 위치로 잡는다.

## 14. 시험 전략

### 단위 시험

- destination YAML strict parsing
- quaternion 및 map bounds 검사
- occupied/unknown/clearance 거부
- fixed와 RViz source 상태 전이
- ACTIVE/CANCELING 중 새 goal 거부
- READY/gate heartbeat timeout
- 안전 사건 발생 시 cancel 결정

### Action client 시험

- `/mission/start`가 저장 pose를 정확히 한 번 전송
- RViz goal이 같은 action client를 통해 정확히 한 번 전송
- action rejection, abort, success, cancel result 처리
- 주행 중 새 RViz goal과 `/mission/start`가 기존 goal을 교체하지 않음
- 취소 후에도 거부된 goal이 자동 실행되지 않음

### Launch/contract 시험

- 기존 `rviz_goal_bridge`가 실행되지 않음
- `fixed_mission_manager`만 `/goal_pose`를 구독하고 NavigateToPose client를 소유함
- destination 미설정 상태에서 RViz 경로가 유지됨
- destination 미설정 `/mission/start`가 실패함

### 실제 운영 전 시험

- `arm:=false`에서 등록 목적지 dry-run 10회
- `arm:=false`에서 서로 다른 RViz 임의 goal 10회
- 주행 중 새 goal 두 입력 모두 거부 확인
- localization/gate heartbeat 차단 시 action cancel 확인
- 실제 로봇은 최초 0.05 m/s 제한으로 수행

## 15. 완료 기준

- `/mission/start` 없이는 등록 목적지로 자동 출발하지 않는다.
- `/mission/start`가 등록된 엘리베이터 앞 goal을 한 번만 전송한다.
- RViz에서 유효한 임의 goal을 지정할 수 있다.
- 두 입력 모두 같은 validation과 action client를 사용한다.
- 진행 중에는 모든 새 goal을 거부하고 기존 goal을 유지한다.
- `/mission/cancel` 이후에만 새 goal을 받을 수 있다.
- localization 또는 gate 상실 시 active goal을 취소한다.
- destination 미설정 상태에서도 RViz 경로와 `arm:=false` 검증이 가능하다.
