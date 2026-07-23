from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).parents[1]
    / "config"
    / "fast_lio_mid360_navigation.yaml"
)


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
