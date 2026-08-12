from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).parents[1]
    / "config"
    / "fast_lio_mid360_navigation.yaml"
)
README = Path(__file__).parents[1] / "README.md"


def test_navigation_profile_disables_heavy_outputs():
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    assert params["publish"]["path_en"] is False
    assert params["publish"]["effect_map_en"] is False
    assert params["publish"]["map_en"] is False
    assert params["publish"]["dense_publish_en"] is False
    assert params["publish"]["scan_publish_en"] is True
    assert params["publish"]["scan_bodyframe_pub_en"] is True
    assert params["pcd_save"]["pcd_save_en"] is False


def test_navigation_profile_keeps_live_mid360_inputs_and_online_extrinsic():
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    assert params["common"]["lid_topic"] == "/livox/lidar"
    assert params["common"]["imu_topic"] == "/livox/imu"
    assert params["preprocess"]["scan_rate"] == 10
    assert params["mapping"]["extrinsic_est_en"] is True


def test_scan_projection_uses_the_planar_navigation_frame():
    params = yaml.safe_load(
        (Path(__file__).parents[1] / "config" / "mid360_scan.yaml").read_text(
            encoding="utf-8"
        )
    )["pointcloud_to_laserscan"]["ros__parameters"]

    assert params["target_frame"] == "body_nav"


def test_readme_launches_fast_lio_with_low_load_navigation_profile():
    readme = README.read_text(encoding="utf-8")
    assert "config_file:=fast_lio_mid360_navigation.yaml" in readme
    assert "config_path:=" in readme
    assert "pcd_save_en: false" in readme
