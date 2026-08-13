import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name("patch_fast_lio_low_latency.py")


def test_patch_keeps_only_the_latest_livox_frame(tmp_path):
    source = tmp_path / "laserMapping.cpp"
    source.write_text(
        "sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>"
        "(lid_topic, 20, livox_pcl_cbk);\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(source)],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "(lid_topic, rclcpp::SensorDataQoS().keep_last(1), livox_pcl_cbk);" in source.read_text(
        encoding="utf-8"
    )
