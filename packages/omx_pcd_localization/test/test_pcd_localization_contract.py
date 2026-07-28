from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
SOURCE = PACKAGE / "src" / "pcd_localizer.cpp"
LAUNCH = PACKAGE / "launch" / "go1_pcd_navigation.launch.py"
CONFIG = PACKAGE / "config" / "hanyang_9f.yaml"
STAGE = REPO / "migration" / "stage_local_ros2_packages.sh"
VERIFY = REPO / "migration" / "verify_pcd_localization.sh"


def test_config_points_to_the_validated_mapping_session():
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    map_path = data["pcd_localizer"]["ros__parameters"]["map_path"]
    assert map_path.endswith(
        "/maps/hanyang_9f/20260728_204825/pcd/merged.pcd"
    )


def test_localizer_uses_coarse_and_fine_3d_registration():
    text = SOURCE.read_text(encoding="utf-8")
    assert "pcl::NormalDistributionsTransform" in text
    assert "pcl::GeneralizedIterativeClosestPoint" in text
    assert "ndt.align(ndt_output, map_to_base_guess)" in text
    assert "gicp.align(aligned, ndt.getFinalTransformation())" in text


def test_localizer_fails_closed():
    text = SOURCE.read_text(encoding="utf-8")
    for guard in (
        "WAITING_FOR_INITIAL_POSE",
        "ndt_not_converged",
        "gicp_not_converged",
        "max_fitness_score_",
        "pose_jump",
        "non_ground_pose",
        "3D localization is stale",
    ):
        assert guard in text


def test_launch_does_not_start_amcl_and_defaults_disarmed():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'package="omx_pcd_localization"' in text
    assert 'executable="pcd_localizer"' in text
    assert '"navigation_launch.py"' in text
    assert 'DeclareLaunchArgument("arm", default_value="false")' in text
    assert "localization_launch.py" not in text
    assert 'package="nav2_amcl"' not in text


def test_hanyang_config_has_full_6dof_calibration_and_safety_limits():
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    params = data["pcd_localizer"]["ros__parameters"]
    assert len(params["pcd_to_map_xyz_rpy"]) == 6
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "camera_init"
    assert params["base_frame"] == "body"
    assert params["max_fitness_score"] > 0.0
    assert params["localization_timeout_sec"] <= 5.0


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
