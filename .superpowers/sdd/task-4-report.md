# Task 4 bounded-memory PCD chunk writer 구현 보고서

## 기준 및 커밋

- 기준 커밋: `6e39b111f7970ff0d02ee859c9755ad37c575e3e`
- 구현 커밋: `e2394b6e1ef1b1fb36ebcddabaf9c01e91f36f76` (`Add bounded PCD chunk writer`)

## 변경 파일

- `packages/go1_mapping/include/go1_mapping/pcd_chunk_buffer.hpp`
- `packages/go1_mapping/src/pcd_chunk_buffer.cpp`
- `packages/go1_mapping/src/pcd_chunk_writer.cpp`
- `packages/go1_mapping/test/test_pcd_chunk_buffer.cpp`
- `packages/go1_mapping/CMakeLists.txt`

## 구현 내용

- `ChunkBuffer`가 frame 수와 `sizeof(pcl::PointXYZI)` 기준 point payload byte 수를 추적한다.
- 기존 buffer가 비어 있지 않을 때 다음 frame이 300 frame 또는 268435456 byte 상한을 넘기기 전에 flush 신호를 낸다.
- `append()` 자체도 frame/byte 상한을 재검사한다. 단일 frame이 byte 상한을 넘거나 산술 overflow가 발생하면 `std::length_error`로 거부하므로 제한을 넘는 accounting 상태가 생기지 않는다.
- `take()`는 현재 cloud를 넘기고 모든 accounting을 0으로 초기화한다. 교체 cloud 할당을 상태 변경 전에 수행해 strong exception safety를 유지한다.
- writer는 `/cloud_registered`를 `SensorDataQoS`, depth 1로 구독하고 `/pcd_chunk_writer/flush` Trigger 서비스를 제공한다.
- writer 파라미터는 `output_dir`, `allowed_root`, `frames_per_chunk=300`, `max_buffer_bytes=268435456`이다.
- output 경로는 `canonical`/`weakly_canonical` 결과의 path component를 비교한다. 문자열 prefix와 allowed root 내부 symlink를 통한 이탈을 거부한다.
- 비어 있는 flush는 파일을 만들지 않는다. 비어 있지 않은 cloud는 `chunk_NNNNNN.pcd.partial`에 binary-compressed PCD로 저장하고 성공한 경우에만 같은 디렉터리의 `.pcd`로 rename한다.
- 저장 또는 rename 실패는 callback 예외로 전파되고 `main()`이 exit failure를 반환한다.
- 별도 signal handler 및 signal handler 내부 PCL I/O를 추가하지 않았다.
- CMake artifact guard를 Task 4와 Task 7 subsystem별로 분리해 projection 파일이 없어도 `pcd_chunk_buffer`, `pcd_chunk_writer`, GTest가 독립적으로 build/install된다.

## TDD 증거

### RED 1: buffer 계약

제품 header/source를 만들기 전에 GTest를 등록하고 다음 명령을 실행했다.

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/codex-task4-red/log build \
  --base-paths packages/go1_mapping --packages-select go1_mapping \
  --build-base /tmp/codex-task4-red/build \
  --install-base /tmp/codex-task4-red/install \
  --cmake-target test_pcd_chunk_buffer --event-handlers console_direct+
```

실제 결과: exit 1. `fatal error: go1_mapping/pcd_chunk_buffer.hpp: No such file or directory`로 의도한 기능 부재 실패를 확인했다.

첫 GREEN 시도에서는 test 파일 생성 patch의 마지막 namespace 닫힘 한 줄이 hunk 길이에 잘려 test compile error가 발생했다. test 오타를 바로잡은 뒤 같은 target을 재실행했다.

실제 GREEN 결과: 10개 `ChunkBuffer` GTest 전부 PASS.

### RED 2: writer target

writer 제품 코드를 추가하기 전에 다음 명령을 실행했다.

```bash
cmake --build /tmp/codex-task4-green-buffer/build/go1_mapping \
  --target pcd_chunk_writer
```

실제 결과: exit 1, `No rule to make target 'pcd_chunk_writer'`.

writer 구현 후 Humble/PCL 1.12 환경에서 독립 target build가 성공했다.

## 최종 검증

### 전체 build

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/codex-task4-final/log-rebuild build \
  --base-paths packages/go1_mapping --packages-select go1_mapping \
  --build-base /tmp/codex-task4-final/build \
  --install-base /tmp/codex-task4-final/install \
  --symlink-install --event-handlers console_direct+
```

실제 결과: exit 0, `Summary: 1 package finished`. Task 7 source가 없는 상태에서 buffer library, writer executable, GTest가 모두 build/install되었다. compile warning은 없었다.

### 요구된 buffer ctest

```bash
colcon --log-base /tmp/codex-task4-final/log-test-buffer test \
  --packages-select go1_mapping \
  --build-base /tmp/codex-task4-final/build \
  --install-base /tmp/codex-task4-final/install \
  --ctest-args -R test_pcd_chunk_buffer --output-on-failure
```

실제 결과: 10/10 GTest PASS, ctest 1/1 PASS.

### self-review 후 전체 package test

```bash
colcon --log-base /tmp/codex-task4-final/log-retest test \
  --packages-select go1_mapping \
  --build-base /tmp/codex-task4-final/build \
  --install-base /tmp/codex-task4-final/install
colcon test-result --test-result-base /tmp/codex-task4-final/build --verbose
```

실제 결과: `Summary: 20 tests, 0 errors, 0 failures, 0 skipped`.

### 경로 containment 단일 프로세스 확인

임시 allowed root를 사용해 writer 시작 검증을 수행했다.

- `/tmp/.../root`에 대해 `/tmp/.../root_evil/pcd`: exit 1
- `/tmp/.../root/link -> /tmp/.../outside`를 통한 symlink 이탈: exit 1
- 두 경우 모두 `output_dir must be below allowed_root by canonical path components` 확인

`git diff --check`와 staged diff check도 exit 0이었다.

## Self-review

- exact-limit append는 허용하고 frame/byte 상한을 한 단위라도 넘는 append는 상태 변경 전에 거부한다.
- `size_t` 곱셈 및 누적 비교는 overflow가 발생하지 않는 순서로 수행한다.
- 빈 buffer의 `should_flush_before()`는 false이며, oversized 단일 frame은 뒤이은 `append()`가 명시적으로 거부한다.
- output containment는 component 비교이며 prefix 및 symlink 우회 시작 검증이 실패하는 것을 확인했다.
- `.partial`과 최종 파일은 같은 디렉터리이므로 rename은 동일 filesystem 내 atomic rename 조건을 만족한다.
- 기존 `.pcd` 또는 `.partial` index를 건너뛰어 재시작 시 덮어쓰지 않는다.
- callback 저장 실패는 fatal log 후 예외가 `main()`으로 전파되어 non-zero 종료된다.
- Task 4 밖의 Go1, Nav2, Jetson 관련 파일은 변경하지 않았다.

## Concerns

- WSL의 multi-process ROS CLI 통합검증에서 writer는 정상 기동했지만, 별도 `ros2 service call` 프로세스가 Fast DDS service discovery를 완료하지 못하고 bounded timeout 후 `rcl node's context is invalid`를 출력했다. 반복 시도는 중단했다. 따라서 empty flush와 실제 1-point publish 후 `.partial` rename은 이 환경에서 end-to-end로 확인하지 못했으며, 구현은 Humble/PCL compile, buffer GTest, 경로 시작 검증 및 정적 API 검토로 확인했다.
- 메모리 상한 accounting은 요구 인터페이스대로 `point_count * sizeof(pcl::PointXYZI)` payload를 기준으로 한다. allocator 내부 capacity/재할당 순간의 구현 세부 overhead까지 byte 상한으로 계측하지 않는다.