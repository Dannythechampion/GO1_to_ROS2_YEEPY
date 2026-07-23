from pathlib import Path


RVIZ = (
    Path(__file__).parents[1]
    / "rviz"
    / "go1_existing_map_low_load.rviz"
)


def test_rviz_is_top_down_and_does_not_render_raw_pointcloud():
    text = RVIZ.read_text(encoding="utf-8")
    assert "Class: rviz_default_plugins/TopDownOrtho" in text
    assert "Fixed Frame: map" in text
    assert "Frame Rate: 15" in text
    assert "Value: /scan" in text
    assert "Value: /goal_pose" in text
    assert "rviz_default_plugins/PointCloud2" not in text
