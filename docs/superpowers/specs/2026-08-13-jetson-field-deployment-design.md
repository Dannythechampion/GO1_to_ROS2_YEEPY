# Jetson 현장 배포 및 안전 주행 설계

## 1. 목표

`codex/verified-posegraph-navigation` 브랜치를 NVIDIA Jetson AGX Orin 64GB,
Ubuntu 22.04, ROS 2 Humble 환경에 복사하여 다음 순서로 사용할 수 있게 한다.

1. 소스와 Unitree SDK wrapper를 ARM64에서 검증한다.
2. ROS 의존성을 설치하고 `go1_driver`, `omx_navigation`을 빌드한다.
3. 테스트가 실제로 수집되는지 확인한다.
4. Livox MID-360과 FAST-LIO 입력을 확인한다.
5. `arm=false`로 localization과 Nav2를 검증한다.
6. 운영자가 e-stop을 준비하고 명시적으로 확인한 경우에만 `arm=true` 주행을 허용한다.

기본 동작은 계속 비무장 상태다. 설치나 dry-run 명령만으로 Unitree SDK에 이동
명령을 보내서는 안 된다.

## 2. 채택한 접근법

기존 posegraph launch와 safety gate를 유지하면서 다음 세 계층을 추가·보강한다.

- **배포 계층:** Jetson staging, ARM64 wrapper, rosdep, build, test, artifact 검증
- **운영 계층:** `preflight`, `dry-run`, `armed`의 명시적인 현장 실행 단계
- **런타임 안전 계층:** localization readiness gate, command watchdog, 직접 SDK 정지,
  정상적인 ROS 종료

systemd 서비스 자동화는 이번 범위에 포함하지 않는다. 센서와 네트워크 상태를 사람이
확인해야 하는 최초 현장 시험에서 자동 재시작은 오히려 결함 원인을 숨길 수 있기 때문이다.

## 3. 배포 계약

### 3.1 staging

`migration/stage_local_ros2_packages.sh`는 다음 항목을 Jetson workspace에 복사한다.

- `go1_driver` 전체 패키지
- `omx_navigation`의 소스, 설정, launch, RViz, 지도, `test`
- 현장 preflight 및 verifier 스크립트

대상 디렉터리가 이미 존재하면 덮어쓰지 않고 실패한다. `.posegraph`와 `.data`는 설치 전과
설치 후 모두 비어 있지 않아야 한다.

### 3.2 ARM64와 Unitree wrapper

armed 전환은 다음 조건을 모두 만족해야 한다.

- `uname -m`이 `aarch64`
- Ubuntu 22.04와 ROS 2 Humble
- Unitree SDK v3.8.6 archive 및 고정 SHA-256 일치
- 현재 Python ABI에 맞는 `robot_interface` ARM64 확장 모듈
- `file`, `ldd`, Python import 검사 통과

x86_64 WSL은 빌드·launch·테스트 검증에만 사용하며 armed 검증을 통과할 수 없다.

### 3.3 테스트 수집

Jetson staged workspace에서 `colcon test`는 다음 조건을 만족해야 한다.

- `go1_driver` 테스트 수가 0보다 큼
- `omx_navigation` 테스트 수가 0보다 큼
- errors, failures, skipped가 모두 0

`0 tests` 성공 종료는 실패로 취급한다.

## 4. 현장 실행 인터페이스

현장 스크립트는 세 모드를 제공한다.

### 4.1 `preflight`

프로세스를 시작하지 않고 다음을 fail-closed로 검사한다.

- 아키텍처, OS, ROS 배포판
- `livox_ros_driver2`, `fast_lio`, Nav2, SLAM Toolbox, scan projection 패키지
- `go1_driver`, `omx_navigation` 설치 경로
- Unitree wrapper import와 ARM64 ABI
- posegraph, data, map YAML, PGM
- diagnostics 경로 생성·쓰기
- `ROS_DOMAIN_ID=100`

### 4.2 `dry-run`

Livox와 FAST-LIO는 기존 문서의 별도 터미널에서 먼저 실행한다. 현장 스크립트는 실제
토픽과 TF를 검사한 뒤 posegraph navigation을 다음 값으로 시작한다.

- `start_go1_driver:=true`
- `arm:=false`
- `record_localization:=true`
- Jetson에서는 `rviz:=false`

확인 대상은 `/livox/lidar`, `/livox/imu`, `/cloud_registered_body`, `/Odometry`, `/scan`,
`camera_init -> body`, localization status/ready, Nav2 lifecycle이다.

### 4.3 `armed`

armed 모드는 dry-run 검증을 대체하지 않는다. 다음 조건을 모두 요구한다.

- launch의 `arm:=true`
- `start_go1_driver:=true`
- `record_localization:=true`
- 확인 문자열 `GO1_ARMED_AND_ESTOP_READY`
- ARM64 Unitree wrapper preflight 성공
- 운영자가 e-stop과 시험 공간을 준비

확인 문자열은 shell history에 남을 수 있으므로 비밀값이 아니라 오조작 방지 토큰이다.
launch 자체에서도 동일 조건을 검사하여 현장 스크립트를 우회한 직접 실행을 차단한다.

## 5. 속도 및 종료 안전성

속도 경로는 다음 하나로 유지한다.

```text
Nav2 controller/behavior
  -> /nav2_controller_cmd_vel
  -> velocity_smoother
  -> /cmd_vel_nav
  -> cmd_vel_safety_gate
  -> /cmd_vel
  -> go1_driver
  -> Unitree SDK
```

localization `ready=false`, heartbeat timeout, command timeout에서는 gate가 0속도를 반복
발행한다. Go1 driver는 ROS publisher 상태와 무관하게 종료 시 Unitree SDK로 반복 stand를
직접 보낸다.

`KeyboardInterrupt`와 `rclpy.executors.ExternalShutdownException`은 정상 종료로 처리한다.
이미 종료된 context에 `rclpy.shutdown()`을 다시 호출하지 않는다. 적용 대상은 다음과 같다.

- `localization_supervisor`
- `planar_base_frame`
- `rviz_goal_bridge`
- `cmd_vel_safety_gate`
- `go1_driver`

## 6. 오류 처리

- 필수 파일·패키지·토픽·TF가 없으면 다음 단계로 진행하지 않는다.
- AMCL이 발견되면 posegraph localization과 함께 실행하지 않는다.
- supervisor가 `READY/NONE`이 아니면 goal을 전달하지 않는다.
- wrapper ABI·아키텍처·SDK hash가 다르면 armed 모드를 거부한다.
- diagnostics 경로에 쓸 수 없으면 armed 모드를 거부한다.
- launch 종료 중 일부 노드가 exit 1이면 검증 실패로 처리한다.
- 현장 검증 실패 시 `arm=false` 상태를 유지하고 원인 토픽·TF·상태를 출력한다.

## 7. 테스트 전략

모든 동작 변경은 테스트를 먼저 실패시키고 최소 구현으로 통과시킨다.

### 7.1 Windows/순수 테스트

- staging 목록에 `test`와 현장 스크립트가 포함되는지 검사
- armed 확인 토큰, `start_go1_driver`, recording 조합의 양·음성 계약
- x86_64에서 armed preflight가 거부되는지 검사
- 각 Python main이 `ExternalShutdownException`에서 exit 0인지 검사
- context가 이미 종료됐을 때 shutdown을 재호출하지 않는지 검사
- Go1 driver가 종료 시 stand를 정확히 한 번 시작하는지 검사

### 7.2 WSL Ubuntu 22.04/Humble

- fresh GitHub clone
- `bash -n`
- staging과 artifact 검사
- `rosdep install`
- `colcon build`
- non-zero `colcon test` 및 `colcon test-result`
- 소스 전체 pytest
- 설치 모듈 import와 `py_compile`
- `arm:=true` 잘못된 조합 거부
- `arm=false` 통합 launch와 정상 종료 exit 0

### 7.3 Jetson 현장

- ARM64 wrapper native build
- Livox/FAST-LIO 토픽 주기와 source timestamp
- 실제 두 TF edge와 extrinsic
- 대략적인 초기 pose 한 번으로 READY 진입
- 로봇을 들어 올리거나 안전 스탠드에 둔 상태의 첫 armed 시험
- 0.3m 이내 goal, 취소, localization 상실, Ctrl-C에서 정지

## 8. 완료 기준

코드 기준 완료는 Windows와 fresh WSL 검증이 모두 통과하고 GitHub 원격 브랜치 SHA가
일치할 때다. 실제 자율주행 완료는 Jetson 현장 체크리스트까지 통과한 뒤에만 선언한다.
Jetson 하드웨어가 없는 현재 환경에서는 “현장에서 실행 가능한 배포 후보”까지만 증명하며,
실센서·실모터 성공을 사전에 주장하지 않는다.
