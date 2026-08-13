# Fixed Start Commissioning

1. 양쪽 앞발을 지정한 타일 기준에 맞추고 GO1을 완전히 정지시킨다.
2. 물리 E-stop을 누르고 motion gate가 disarm인지 확인한다.
3. 기존 지도 launch를 `initial_pose_arm:=false arm:=false`로 실행한다.
4. `commission_start_pose`를 명시적 output path와 sample count 10으로 실행한다.
5. 이번 한 번만 RViz `2D Pose Estimate`를 사용해 AMCL을 수렴시킨다.
6. `/mnt/t500/go1_runtime/start_pose.yaml`을 확인하고 백업한다.
7. 세 번의 cold start에서 자동 seed와 readiness를 검증한다.

명령 전문은 [FIXED_START_AMCL_MISSION_COMMANDS.md](../FIXED_START_AMCL_MISSION_COMMANDS.md)를 따른다. 지도, 타일 기준 또는 센서 설치가 변하면 다시 commissioning한다.
