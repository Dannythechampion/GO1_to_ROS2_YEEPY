# 융합교육관 9층 하이브리드 매핑 설계

## 1. 목적

Go1을 사용하지 않고 카트에 NVIDIA Jetson AGX Orin과 Livox MID-360을
단단히 고정해 융합교육관 9층의 복도, 엘리베이터 홀, 교차 구간 및 계단
입구를 수집한다. Jetson에서 ROS2 Humble 기반 매핑과 원본 데이터 저장을
수행하고 노트북에서는 RViz만 실행한다.

최종 산출물은 다음 두 계열을 모두 포함한다.

- FAST-LIO 기반 3D PCD 원본 및 병합·필터링 PCD
- Nav2에서 사용할 수 있는 0.05m 해상도의 2D PGM/YAML 지도

## 2. 범위와 제외 사항

### 포함

- MID-360 LiDAR 및 IMU 수집
- FAST-LIO odometry와 세계 좌표계 포인트클라우드 생성
- rosbag2 원본 기록
- 제한된 메모리를 사용하는 분할 PCD 저장
- PointCloud2에서 LaserScan 변환
- slam_toolbox의 2D 매핑과 폐루프 보정
- PCD 기반 2D 투영 지도 생성
- 두 2D 지도의 교차검증 및 결과 보고서
- 노트북 WSL Ubuntu 22.04/ROS2 Humble에서 개발과 시험
- Jetson ARM64 빌드 및 현장 배포 절차

### 제외

- Go1 드라이버와 실제 로봇 제어
- Nav2 자율주행
- AMCL localization
- 고정되지 않은 라이다 상태의 현장 수집
- 기존 `scans.yaml` 또는 `scans_new.yaml` 재사용
- 검증되지 않은 지도의 자동 최종 승격

## 3. 운용 장비와 네트워크

- Jetson SSH: `unicon@192.168.0.138`
- MID-360 센서망: `192.168.1.138`
- Jetson 저장소: `/mnt/t500/go1_ros2_project`
- Jetson ROS2 workspace: `/mnt/t500/go1_ros2_ws`
- 대용량 결과 저장 루트: `/mnt/t500/maps/hanyang_9f`
- ROS 설정: `ROS_DOMAIN_ID=100`
- RMW: `rmw_cyclonedds_cpp`

Jetson은 센서 처리, 매핑 및 저장을 담당한다. 노트북 WSL은 Jetson을
CycloneDDS peer로 사용하고 RViz 렌더링만 담당한다.

## 4. 권장 아키텍처

```text
MID-360 (/livox/lidar, /livox/imu)
        |
        +-- rosbag2
        |     `-- LiDAR, IMU, Odometry, TF 원본 기록
        |
        `-- FAST-LIO
              +-- camera_init -> body
              +-- /Odometry
              +-- /cloud_registered
              +-- /cloud_registered_body
              |
              +-- pcd_chunk_writer
              |     `-- 제한된 크기의 binary PCD 조각
              |
              `-- pointcloud_to_laserscan
                    `-- /scan
                          `-- slam_toolbox
                                +-- map_slam -> camera_init
                                +-- 폐루프 pose graph
                                `-- 2D OccupancyGrid
```

FAST-LIO 내부 PCD 저장은 `pcd_save_en: false`로 유지한다. 원본 구현은
지정 프레임 수를 메모리에 누적하고 `interval: -1`일 때 전체 세션을 한
파일에 보관하므로 장시간 수집에서 메모리 고갈 위험이 있다. 별도
`pcd_chunk_writer`가 `/cloud_registered`를 구독해 세션 디렉터리에
bounded-memory 방식으로 저장한다.

slam_toolbox 지도는 자유공간과 폐루프가 포함된 주 2D 지도다. PCD 투영
지도는 벽, 기둥 및 장애물 형상을 검증하는 보조 지도다. 두 결과는 자동
병합하지 않으며 검증을 통과한 결과만 최종 `map` 이름으로 승격한다.

관련 원본 문서:

- FAST-LIO PCD 저장 구현:
  https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp
- slam_toolbox Humble:
  https://docs.ros.org/en/humble/p/slam_toolbox/
- pointcloud_to_laserscan:
  https://github.com/ros-perception/pointcloud_to_laserscan
- rosbag2:
  https://github.com/ros2/rosbag2

## 5. 좌표계와 토픽 책임

### TF

```text
map_slam -> camera_init -> body
```

- FAST-LIO만 `camera_init -> body`를 발행한다.
- slam_toolbox만 `map_slam -> camera_init`을 발행한다.
- 추가 static TF로 FAST-LIO extrinsic을 중복 적용하지 않는다.
- 오프라인 PCD 투영 결과는 `map_pcd` 프레임을 사용한다.
- 최종 지도 검증 후에만 소비 측에서 전역 프레임을 `map`으로 사용한다.

### 주요 토픽

- 입력: `/livox/lidar`, `/livox/imu`
- FAST-LIO: `/Odometry`, `/cloud_registered`,
  `/cloud_registered_body`
- 2D 변환: `/scan`
- slam_toolbox 출력: `/map_slam`

`pointcloud_to_laserscan`은 `target_frame: body`, 입력 queue 1을 사용한다.
높이 범위는 라이다 장착 후 실제 높이를 측정하고 RViz 미리보기로 결정한다.

## 6. 소프트웨어 구성요소

새 ROS2 패키지 `packages/go1_mapping`을 추가한다.

```text
packages/go1_mapping/
├─ config/
│  ├─ fast_lio_mapping_safe.yaml
│  ├─ pointcloud_to_scan_mapping.yaml
│  ├─ slam_toolbox_hanyang_9f.yaml
│  └─ mapping_session.yaml
├─ launch/
│  ├─ mapping_session.launch.py
│  └─ laptop_rviz.launch.py
├─ go1_mapping/
│  ├─ session_guard.py
│  ├─ map_finalizer.py
│  └─ validation_report.py
├─ src/
│  └─ pcd_chunk_writer.cpp
├─ rviz/
│  └─ hanyang_9f_mapping.rviz
└─ test/
```

### `mapping_session.launch.py`

- 한 세션에서 Livox, FAST-LIO, PCD writer, LaserScan 변환,
  slam_toolbox 및 rosbag2를 실행한다.
- 세션 ID와 저장 루트를 launch argument로 받는다.
- Go1 드라이버, AMCL 및 Nav2를 실행하지 않는다.
- 구성요소 종료 순서를 관리해 rosbag과 PCD 잔여 데이터를 flush한다.

### `pcd_chunk_writer`

- `/cloud_registered`를 구독한다.
- 300 frame 또는 256MiB 중 먼저 도달하는 조건으로 조각을 닫는다.
- binary PCD 조각을 임시 이름으로 쓴 뒤 성공 시 원자적으로 이름을 바꾼다.
- 종료 시 남은 비어 있지 않은 버퍼를 마지막 PCD로 저장한다.
- 저장 실패를 진단 토픽과 프로세스 종료 코드로 알린다.

### `session_guard`

- LiDAR, IMU, Odometry 및 TF의 존재와 최신성을 확인한다.
- timestamp 역행과 입력 중단을 감지한다.
- 저장 경로가 `/mnt/t500` 아래인지 검증한다.
- 시작 전 `/mnt/t500` 여유 공간이 100GiB 이상인지 확인한다.
- 수집 중 여유 공간이 50GiB 미만이면 새 기록 중단을 요청한다.
- 현장 출발 전에 통과해야 하는 preflight 결과를 출력한다.

### `map_finalizer`

- PCD 조각을 병합하고 voxel filter를 적용해
  `merged_filtered.pcd`를 만든다.
- slam_toolbox pose graph와 PGM/YAML을 저장한다.
- PCD 높이 슬라이스를 `map_pcd` OccupancyGrid로 변환한다.
- 최종 파일의 해상도, image 상대 경로 및 YAML 형식을 정규화한다.
- PCD와 slam 지도를 자동 합성하지 않는다.

### `validation_report`

- 수집 시간, 토픽 주기, 입력 중단, timestamp 상태를 기록한다.
- PCD 조각 수, 포인트 수, 지도 크기와 점유 비율을 기록한다.
- 출발점 복귀 오차와 폐루프 결과를 기록한다.
- 산출물 SHA-256과 사용한 Git commit 및 파라미터를 기록한다.

## 7. 세션 저장 구조

```text
/mnt/t500/maps/hanyang_9f/
└─ <YYYYMMDD_HHMMSS>/
   ├─ bag/
   ├─ pcd/
   │  ├─ scans_0000.pcd
   │  ├─ scans_0001.pcd
   │  └─ merged_filtered.pcd
   ├─ slam_toolbox/
   │  ├─ hanyang_9f_slam.pgm
   │  ├─ hanyang_9f_slam.yaml
   │  └─ hanyang_9f.posegraph
   ├─ pcd2d/
   │  ├─ hanyang_9f_pcd.pgm
   │  └─ hanyang_9f_pcd.yaml
   └─ validation/
      ├─ session_manifest.yaml
      └─ report.md
```

루트 파티션, FAST-LIO 소스 디렉터리 및 저장소 내부에는 PCD나 bag을
저장하지 않는다.

## 8. 현장 수집 절차

1. Jetson과 MID-360을 같은 카트 프레임에 단단히 고정한다.
2. 라이다 높이와 대략적인 roll/pitch를 측정해 세션 메타데이터에 기록한다.
3. 현장에서 벽, 모서리 또는 홀 구조처럼 다시 식별할 수 있는 출발점을
   선택한다. 출발 위치는 세션마다 달라도 된다.
4. 출발점 사진, 바닥 표식 및 카트 방향을 기록한다.
5. `/mnt/t500` 여유 공간과 세션 출력 경로를 확인한다.
6. 카트를 움직이지 않고 최소 15초 동안 IMU를 초기화한다.
7. LiDAR, IMU, Odometry, cloud, TF, rosbag 및 PCD writer gate를 통과한다.
8. 선택한 출발점에서 직선 속도 0.5m/s 이하로 출발한다.
9. 복도, 엘리베이터 홀, 교차 구간 및 계단 입구를 포함해 순회한다.
10. 회전 속도 20deg/s 이하를 유지하고 주요 모서리에서 잠시 정지한다.
11. 처음 선택한 위치와 같은 방향으로 돌아와 최소 10초 정지한다.
12. 가능하면 반대 방향으로 한 번 더 순회한다.
13. 단일 종료 절차로 입력을 멈추고 rosbag과 PCD를 flush한다.
14. 오프라인 최종화와 검증을 수행한다.

## 9. 실패 처리

- 센서 입력 중단: 카트를 정지하고 중단 시각을 manifest에 기록한다.
- timestamp 역행: 세션을 최종 지도 후보에서 제외하고 bag 재처리를
  우선한다.
- 디스크 부족: 새 파일 생성을 중단하되 이미 기록한 파일은 보존한다.
- PCD 쓰기 실패: 해당 조각을 최종 병합에서 제외하고 오류를 보고한다.
- 폐루프 실패: 지도를 승격하지 않고 rosbag으로 파라미터를 조정해
  재처리한다.
- 동적 사람·문·유리 반사: slam 지도와 PCD 지도를 비교하고 자동 삭제하지
  않는다.
- 두 지도의 구조가 크게 다름: 자동 병합하지 않고 TF, 높이 필터,
  timestamp 및 폐루프를 순서대로 조사한다.

## 10. 검증 기준

세션은 다음 기준을 모두 만족해야 최종 지도 후보가 된다.

- `/livox/lidar`와 `/Odometry` 평균 주기가 각각 8Hz 이상이다.
- `/livox/imu` 평균 주기가 100Hz 이상이다.
- 필수 입력 토픽에 1초를 넘는 연속 공백이 없다.
- timestamp 역행이 없다.
- 모든 완료 표시된 PCD 조각이 비어 있지 않고 다시 열 수 있다.
- 동일 층 이동 중 Z 추정이 비정상적으로 발산하지 않는다.
- 선택한 출발점 복귀 위치 오차가 0.5m 이하이다.
- 복귀 방향 오차가 10도 이하이다.
- 복귀 오차는 slam_toolbox의 최적화된 첫 pose와 마지막 pose를 비교해 계산한다.
- 2D 지도 해상도는 0.05m이다.
- 복도 벽이 심하게 이중으로 생기거나 휘지 않는다.
- 엘리베이터 홀, 교차점 및 계단 입구를 구별할 수 있다.
- PGM/YAML의 image 경로, origin 및 threshold가 유효하다.
- 산출물 해시, Git commit 및 파라미터가 보고서에 기록된다.

## 11. 로컬 개발과 배포

개발은 노트북 WSL Ubuntu 22.04/ROS2 Humble에서 수행한다.

1. 합성 PointCloud2와 작은 고정 rosbag fixture로 단위·통합 테스트한다.
2. 모든 출력은 테스트 임시 디렉터리에 저장한다.
3. rosbag2는 `/livox/lidar`, `/livox/imu`, `/Odometry`, `/tf`,
   `/tf_static`만 기록한다.
4. rosbag2는 파일 단위 zstd 압축과 4GiB 크기 분할을 사용한다.
5. x86_64에서 설정, launch, chunk flush 및 최종화 테스트를 통과한다.
6. 작업은 `codex/hanyang-9f-mapping` 브랜치에만 커밋한다.
7. GitHub에 브랜치를 푸시한 뒤 Jetson에서 checkout/pull한다.
8. Jetson ARM64에서 `go1_mapping`만 선택 빌드한다.
9. 정지 센서 시험과 20~30m 짧은 복도 시험을 통과한다.
10. 이후 융합교육관 9층 전체 세션을 수집한다.

## 12. 완료 조건

- Go1과 Nav2 없이 단일 명령으로 안전한 매핑 세션을 시작·종료할 수 있다.
- 원본 rosbag, 분할 PCD, 병합 PCD, slam 2D 지도 및 PCD 2D 지도가 남는다.
- 수집 실패가 최종 지도로 조용히 승격되지 않는다.
- 노트북 RViz에서 지도, LaserScan, TF 및 trajectory를 확인할 수 있다.
- 현장 재수집 전에 rosbag으로 오프라인 재처리할 수 있다.
