# Destination Commissioning

1. 물리 E-stop을 누르고 motion gate를 disarm한다.
2. static map과 `record_destination_pose`를 실행한다.
3. RViz `2D Goal Pose`로 엘리베이터 앞 대기 위치와 방향을 지정한다.
4. Recorder가 map bounds, occupancy와 0.35 m clearance를 통과한 pose만 저장하는지 확인한다.
5. `/mnt/t500/go1_runtime/destination_pose.yaml`을 확인하고 백업한다.
6. Mission manager를 재시작하고 `/mission/start`가 configured 상태인지 확인한다.

Recorder는 기본적으로 기존 파일을 덮어쓰지 않는다.
