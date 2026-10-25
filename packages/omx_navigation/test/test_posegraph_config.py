from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / "config"


def load_slam():
    return yaml.safe_load(
        (CONFIG / "slam_toolbox_localization_hanyang_9f.yaml").read_text(
            encoding="utf-8"
        )
    )


def load_nav2():
    return yaml.safe_load(
        (CONFIG / "nav2_posegraph_params.yaml").read_text(encoding="utf-8")
    )


def test_posegraph_localization_owns_map_to_odom_contract():
    params = load_slam()["slam_toolbox"]["ros__parameters"]
    assert params["mode"] == "localization"
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "camera_init"
    assert params["base_frame"] == "body_nav"
    assert params["map_file_name"].endswith("/hanyang_9f")
    assert params["scan_queue_size"] == 1
    assert params["correlation_search_space_dimension"] <= 1.0


def test_posegraph_localization_requests_startup_deserialization():
    params = load_slam()["slam_toolbox"]["ros__parameters"]
    assert params["map_start_pose"] == [0.0, 0.0, 0.0]


def test_posegraph_localization_enlarges_the_matching_stack():
    """slam_toolbox segfaults on a deserialized posegraph with the default stack.

    The localization node died with exit code -11 twice on 2026-10-25 -- once
    shortly after deserializing the map and once 410 s into the first armed
    drive, both while /scan was healthy. The mapping config already carried
    stack_size_to_use; this one did not, and the launch's RewrittenYaml only
    rewrites map_file_name, odom_frame, base_frame and scan_topic, so the node
    ran on the default 8 MB stack. Upstream ships the same value in
    mapper_params_localization.yaml for exactly this reason.
    """
    params = load_slam()["slam_toolbox"]["ros__parameters"]
    assert params["stack_size_to_use"] >= 40000000


def test_posegraph_progress_checker_counts_rotation_as_progress():
    """An in-place turn at the goal must not read as a stalled robot.

    SimpleGoalChecker runs stateful, so once the position tolerance is met it
    latches and only yaw is checked -- the robot then turns on the spot and
    translates nothing. SimpleProgressChecker measures translation alone, so
    that turn looks like no progress and the goal aborts once
    movement_time_allowance runs out; three goals ended that way with
    "Failed to make progress" on 2026-09-18. PoseProgressChecker accepts
    rotation as progress.
    """
    controller = load_nav2()["controller_server"]["ros__parameters"]
    progress = controller["progress_checker"]
    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_angle"] > 0.0
    goal = controller["general_goal_checker"]
    assert goal["stateful"] is True


def test_posegraph_arrival_tolerance_clears_the_observable_floor():
    """The arrival window must be wider than the pose offset the scan allows.

    Along a corridor the along-axis translation is not observable, so the
    refined-vs-tracked offset spreads: p90 0.309 m over the 2026-09-18 window
    the drift fixture treats as good tracking, and p90 0.200 m with a 0.340 m
    maximum over the 2026-10-25 armed drive. At 0.20 m the goal checker flipped
    in and out of tolerance and the robot hunted around both goals without
    arriving. DWB's own copy steers when RotateToGoal starts the final turn and
    has to agree with the checker.
    """
    controller = load_nav2()["controller_server"]["ros__parameters"]
    goal = controller["general_goal_checker"]
    assert goal["xy_goal_tolerance"] >= 0.30
    assert controller["FollowPath"]["xy_goal_tolerance"] == goal["xy_goal_tolerance"]


def test_posegraph_nav2_has_no_amcl_and_uses_safe_footprint():
    config = load_nav2()
    assert "amcl" not in config
    for name in ("local_costmap", "global_costmap"):
        params = config[name][name]["ros__parameters"]
        assert params["robot_base_frame"] == "body_nav"
        assert params["footprint"] == "[[0.37, 0.19], [0.37, -0.19], [-0.37, -0.19], [-0.37, 0.19]]"
        assert params["transform_tolerance"] == 0.5
    assert config["controller_server"]["ros__parameters"]["FollowPath"]["BaseObstacle.scale"] == 0.02


def test_velocity_smoother_preserves_default_backup_recovery():
    smoother = load_nav2()["velocity_smoother"]["ros__parameters"]
    assert smoother["min_velocity"][0] <= -0.05
    assert smoother["max_velocity"][0] >= 0.05
