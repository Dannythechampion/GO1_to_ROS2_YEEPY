import importlib
import sys
from types import ModuleType

import pytest


@pytest.fixture
def driver_module(monkeypatch):
    geometry = ModuleType("geometry_msgs.msg")
    geometry.Twist = object
    std = ModuleType("std_msgs.msg")
    std.String = object
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    rclpy.spin = lambda _node: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})

    for name, module in {
        "geometry_msgs.msg": geometry,
        "std_msgs.msg": std,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    sys.modules.pop("go1_driver.node", None)
    module = importlib.import_module("go1_driver.node")
    yield module
    sys.modules.pop("go1_driver.node", None)


def test_main_sends_stand_and_exits_cleanly_on_external_shutdown(
    driver_module, monkeypatch
):
    events = []

    class RecordingDriver:
        def shutdown_robot(self):
            events.append("stand")

        def destroy_node(self):
            events.append("destroy")

    monkeypatch.setattr(driver_module, "Go1Driver", RecordingDriver)
    driver_module.rclpy.init = lambda **_kwargs: events.append("init")
    driver_module.rclpy.spin = lambda _node: (_ for _ in ()).throw(
        driver_module.NORMAL_SHUTDOWN_EXCEPTIONS[1]()
    )
    driver_module.rclpy.ok = lambda: False
    driver_module.rclpy.shutdown = lambda: events.append("shutdown")

    driver_module.main()

    assert events == ["init", "stand", "destroy"]
