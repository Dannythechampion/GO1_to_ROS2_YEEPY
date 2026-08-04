from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
SOURCE = PACKAGE / "src" / "pcd_localizer.cpp"
LAUNCH = PACKAGE / "launch" / "go1_pcd_navigation.launch.py"
CONFIG = PACKAGE / "config" / "hanyang_9f.yaml"
RVIZ = PACKAGE / "rviz" / "go1_pcd_localization.rviz"
NAV2_PARAMS = (
    REPO / "packages" / "omx_navigation" / "config" / "nav2_pcd_localization_params.yaml"
)
STAGE = REPO / "migration" / "stage_local_ros2_packages.sh"
VERIFY = REPO / "migration" / "verify_pcd_localization.sh"


def localizer_params():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["pcd_localizer"][
        "ros__parameters"
    ]


def test_config_points_to_the_validated_mapping_session():
    assert localizer_params()["map_path"].endswith(
        "/maps/hanyang_9f/20260728_204825/pcd/merged.pcd"
    )


def test_localizer_uses_coarse_and_fine_3d_registration():
    text = SOURCE.read_text(encoding="utf-8")
    assert "pcl::NormalDistributionsTransform" in text
    assert "pcl::GeneralizedIterativeClosestPoint" in text
    assert "ndt_.align(ndt_output, map_to_base_guess)" in text
    assert "gicp_.align(aligned, ndt_.getFinalTransformation())" in text


def test_localizer_fails_closed():
    text = SOURCE.read_text(encoding="utf-8")
    for guard in (
        "WAITING_FOR_INITIAL_POSE",
        "ndt_not_converged",
        "gicp_not_converged",
        "max_fitness_score_",
        "pose_jump",
        "non_ground_pose",
        "outside_map_bounds",
        "odom_reset",
        "3D localization is stale",
    ):
        assert guard in text


def test_tf_broadcast_is_not_blocked_by_registration():
    """The 20 Hz map -> odom timer must not share a callback group with NDT/GICP."""
    text = SOURCE.read_text(encoding="utf-8")
    assert "MultiThreadedExecutor" in text
    assert "registration_group_ = create_callback_group" in text
    assert "output_group_ = create_callback_group" in text
    assert "cloud_options.callback_group = registration_group_" in text
    assert "std::bind(&PcdLocalizer::publish_tf, this),\n      output_group_" in text
    assert "rclcpp::Duration::from_seconds(tf_transform_tolerance_)" in text
    assert "rclcpp::spin(" not in text


def test_registration_inputs_are_bounded():
    params = localizer_params()
    assert params["scan_max_range"] < params["local_map_radius"]
    assert params["target_refresh_distance"] > 0.0
    # PCL needs >= 6 points per NDT voxel; 0.40 m leaf at 1.0 m resolution
    # discarded half of them on this map.
    assert params["map_leaf_size"] <= 0.30
    assert params["ndt_resolution"] >= 1.5


def test_pose_is_constrained_to_the_2d_map():
    bounds = localizer_params()["map_bounds_xy"]
    assert len(bounds) == 4
    assert bounds[0] < bounds[2] and bounds[1] < bounds[3]


def test_launch_does_not_start_amcl_and_defaults_disarmed():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'package="omx_pcd_localization"' in text
    assert 'executable="pcd_localizer"' in text
    assert '"navigation_launch.py"' in text
    assert 'DeclareLaunchArgument("arm", default_value="false")' in text
    assert "localization_launch.py" not in text
    assert 'package="nav2_amcl"' not in text


def test_launch_uses_curated_map_and_3d_rviz_profile():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "hanyang_9f_annotated.yaml" in text
    assert "nav2_pcd_localization_params.yaml" in text
    assert "go1_pcd_localization.rviz" in text


def test_nav2_profile_has_no_amcl_and_rejects_unknown_space():
    config = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
    assert "amcl" not in config
    assert config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False
    for name in ("local_costmap", "global_costmap"):
        costmap = config[name][name]["ros__parameters"]
        assert costmap["inflation_layer"]["inflation_radius"] >= costmap["robot_radius"]
    assert config["global_costmap"]["global_costmap"]["ros__parameters"][
        "track_unknown_space"
    ] is True


def test_rviz_profile_shows_the_localizer_clouds():
    text = RVIZ.read_text(encoding="utf-8")
    assert "/pcd_localizer/map_cloud" in text
    assert "/pcd_localizer/aligned_cloud" in text
    assert "/initialpose" in text


def test_deployment_and_runtime_verification_include_3d_localizer():
    stage = STAGE.read_text(encoding="utf-8")
    verify = VERIFY.read_text(encoding="utf-8")
    assert 'pcd_target="$src_dir/omx_pcd_localization"' in stage
    assert "omx_pcd_localization" in stage
    assert "/pcd_localizer/status" in verify
    assert "/pcd_localizer/pose" in verify
    assert "LOCALIZED fitness=" in verify
    assert "AMCL is not running" in verify
    assert "go1_driver must remain arm=false" in verify
    assert "tf2_monitor" in verify
