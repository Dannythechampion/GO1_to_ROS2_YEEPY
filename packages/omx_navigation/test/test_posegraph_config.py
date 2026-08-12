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


def test_posegraph_nav2_has_no_amcl_and_uses_safe_footprint():
    config = load_nav2()
    assert "amcl" not in config
    for name in ("local_costmap", "global_costmap"):
        params = config[name][name]["ros__parameters"]
        assert params["robot_base_frame"] == "body_nav"
        assert params["footprint"] == "[[0.37, 0.19], [0.37, -0.19], [-0.37, -0.19], [-0.37, 0.19]]"
        assert params["transform_tolerance"] == 0.5
    assert config["controller_server"]["ros__parameters"]["FollowPath"]["BaseObstacle.scale"] == 0.02
