from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).parents[1] / "config" / "nav2_existing_map_params.yaml"
)


def params(node):
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[node][
        "ros__parameters"
    ]


def test_fast_lio_frames_and_topics():
    amcl = params("amcl")
    assert amcl["global_frame_id"] == "map"
    assert amcl["odom_frame_id"] == "camera_init"
    assert amcl["base_frame_id"] == "body"
    assert amcl["scan_topic"] == "scan"


def test_ros1_dwa_tuning_intent_is_preserved():
    controller = params("controller_server")
    follow = controller["FollowPath"]
    goal = controller["general_goal_checker"]
    assert controller["controller_frequency"] == 10.0
    assert goal["xy_goal_tolerance"] == 0.20
    assert goal["yaw_goal_tolerance"] == 0.15
    assert follow["sim_time"] == 2.0
    assert follow["vx_samples"] == 10
    assert follow["vy_samples"] == 1
    assert follow["vtheta_samples"] == 20
    assert follow["PathAlign.scale"] == 40.0
    assert follow["PathDist.scale"] == 40.0
    assert follow["GoalAlign.scale"] == 20.0
    assert follow["GoalDist.scale"] == 20.0
    assert follow["BaseObstacle.scale"] == 0.01
    assert follow["Oscillation.oscillation_reset_dist"] == 0.20


def test_nav2_never_exceeds_go1_driver_limits():
    follow = params("controller_server")["FollowPath"]
    smoother = params("velocity_smoother")
    assert follow["max_vel_x"] <= 0.20
    assert follow["max_vel_theta"] <= 0.40
    assert smoother["max_velocity"] == [0.20, 0.0, 0.40]
    assert smoother["min_velocity"] == [0.0, 0.0, -0.40]


def test_global_planner_rejects_unknown_space():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    planner = params("planner_server")["GridBased"]
    assert global_costmap["track_unknown_space"] is True
    assert planner["allow_unknown"] is False
