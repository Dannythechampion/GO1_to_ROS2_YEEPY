from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).parents[1] / "config" / "nav2_existing_map_params.yaml"
)


def load_config():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_bt_navigator_uses_humble_default_plugin_libraries():
    bt_params = load_config()["bt_navigator"]["ros__parameters"]
    assert "plugin_lib_names" not in bt_params


def test_inflation_radius_is_not_smaller_than_robot_radius():
    config = load_config()
    for costmap_name in ("local_costmap", "global_costmap"):
        params = config[costmap_name][costmap_name]["ros__parameters"]
        assert (
            params["inflation_layer"]["inflation_radius"]
            >= params["robot_radius"]
        )


def test_humble_fallback_tolerates_fast_lio_tf_latency():
    config = load_config()
    assert config["amcl"]["ros__parameters"]["transform_tolerance"] == 0.5
    assert config["controller_server"]["ros__parameters"]["FollowPath"]["transform_tolerance"] == 0.5
    assert config["behavior_server"]["ros__parameters"]["transform_tolerance"] == 0.5
