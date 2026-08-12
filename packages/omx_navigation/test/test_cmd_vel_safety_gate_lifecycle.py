import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


WRAPPER_PATH = (
    Path(__file__).parents[1] / "omx_navigation" / "cmd_vel_safety_gate.py"
)


def load_wrapper(monkeypatch, rclpy_module):
    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = type("Twist", (), {})
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy_module.node = rclpy_node
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)

    spec = importlib.util.spec_from_file_location("test_cmd_vel_safety_gate", WRAPPER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_main_shuts_down_when_node_creation_fails(monkeypatch):
    events = []
    rclpy = SimpleNamespace(
        init=lambda args: events.append("init"),
        shutdown=lambda: events.append("shutdown"),
        spin=lambda node: events.append("spin"),
    )
    wrapper = load_wrapper(monkeypatch, rclpy)

    class FailingNode:
        def __init__(self):
            raise ValueError("invalid timeout")

    monkeypatch.setattr(wrapper, "CmdVelSafetyGate", FailingNode)

    with pytest.raises(ValueError, match="invalid timeout"):
        wrapper.main()

    assert events == ["init", "shutdown"]


def test_main_shuts_down_when_stop_and_destroy_fail(monkeypatch):
    events = []
    rclpy = SimpleNamespace(
        init=lambda args: events.append("init"),
        shutdown=lambda: events.append("shutdown"),
        spin=lambda node: events.append("spin"),
    )
    wrapper = load_wrapper(monkeypatch, rclpy)

    class FailingCleanupNode:
        def publish_stop(self):
            events.append("stop")
            raise RuntimeError("publisher unavailable")

        def destroy_node(self):
            events.append("destroy")
            raise RuntimeError("destroy failed")

    monkeypatch.setattr(wrapper, "CmdVelSafetyGate", FailingCleanupNode)

    wrapper.main()

    assert events == ["init", "spin", "stop", "destroy", "shutdown"]
