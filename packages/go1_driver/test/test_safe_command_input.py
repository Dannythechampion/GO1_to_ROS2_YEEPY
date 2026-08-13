from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_driver_defaults_to_the_motion_gate_output():
    node = (ROOT / "go1_driver" / "node.py").read_text(encoding="utf-8")
    launch = (ROOT / "launch" / "go1_driver.launch.py").read_text(encoding="utf-8")
    config = (ROOT / "config" / "go1_driver.yaml").read_text(encoding="utf-8")
    assert 'declare_parameter("cmd_vel_topic", "/cmd_vel_safe")' in node
    assert 'default_value="/cmd_vel_safe"' in launch
    assert "cmd_vel_topic: /cmd_vel_safe" in config


def test_driver_retains_independent_watchdog():
    config = (ROOT / "config" / "go1_driver.yaml").read_text(encoding="utf-8")
    assert "cmd_timeout: 0.35" in config
