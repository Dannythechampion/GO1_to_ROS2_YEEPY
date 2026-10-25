import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


WRAPPER_PATH = (
    Path(__file__).parents[1] / "omx_navigation" / "cmd_vel_safety_gate.py"
)


def load_wrapper(monkeypatch, rclpy_module, node_class=None):
    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = type(
        "Twist", (),
        {"__init__": lambda self: (
            setattr(self, "linear", SimpleNamespace(x=0.0, y=0.0))
            or setattr(self, "angular", SimpleNamespace(z=0.0))
        )},
    )
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = node_class or type("Node", (), {})
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


class GateNodeStub:
    overrides = {}

    def __init__(self, name):
        self.name = name
        self.publishers = []
        self.subscriptions = []

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.overrides.get(name, default))

    def create_publisher(self, _type, topic, _depth):
        publisher = SimpleNamespace(topic=topic, messages=[], publish=lambda message: publisher.messages.append(message))
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _type, topic, callback, _depth):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)


def test_gate_topic_parameters_have_safe_defaults_and_allow_overrides(monkeypatch):
    rclpy = SimpleNamespace(init=lambda **_: None, shutdown=lambda: None, spin=lambda _node: None)
    GateNodeStub.overrides = {}
    wrapper = load_wrapper(monkeypatch, rclpy, GateNodeStub)
    node = wrapper.CmdVelSafetyGate()
    assert {item.topic for item in node.publishers} == {"/cmd_vel"}
    assert {item.topic for item in node.subscriptions} == {
        "/cmd_vel_nav", "/cmd_vel_align", "/localization_supervisor/ready"
    }

    GateNodeStub.overrides = {"input_topic": "/custom_in", "output_topic": "/custom_out"}
    node = wrapper.CmdVelSafetyGate()
    assert {item.topic for item in node.publishers} == {"/custom_out"}
    assert "/custom_in" in {item.topic for item in node.subscriptions}


def test_alignment_commands_outrank_nav2_while_fresh(monkeypatch):
    """After the handoff the velocity smoother still ramps down on /cmd_vel_nav;
    those commands must not interleave with final alignment's."""
    rclpy = SimpleNamespace(init=lambda **_: None, shutdown=lambda: None, spin=lambda _node: None)
    GateNodeStub.overrides = {}
    wrapper = load_wrapper(monkeypatch, rclpy, GateNodeStub)
    node = wrapper.CmdVelSafetyGate()
    clock = SimpleNamespace(now=0.0)
    node._now = lambda: clock.now
    callbacks = {item.topic: item.callback for item in node.subscriptions}
    output = next(item for item in node.publishers if item.topic == "/cmd_vel").messages

    def twist(vx=0.0, yaw=0.0):
        message = sys.modules["geometry_msgs.msg"].Twist()
        message.linear.x = vx
        message.angular.z = yaw
        return message

    callbacks["/localization_supervisor/ready"](SimpleNamespace(data=True))
    callbacks["/cmd_vel_nav"](twist(vx=0.2))
    assert output[-1].linear.x == 0.2

    clock.now = 0.1
    callbacks["/localization_supervisor/ready"](SimpleNamespace(data=True))
    callbacks["/cmd_vel_align"](twist(yaw=0.25))
    callbacks["/cmd_vel_nav"](twist(vx=0.1))
    assert (output[-1].linear.x, output[-1].angular.z) == (0.0, 0.25)

    clock.now = 0.45  # alignment silent for longer than command_timeout
    callbacks["/localization_supervisor/ready"](SimpleNamespace(data=True))
    callbacks["/cmd_vel_nav"](twist(vx=0.1))
    assert output[-1].linear.x == 0.1


def test_alignment_commands_are_still_gated_on_localization(monkeypatch):
    rclpy = SimpleNamespace(init=lambda **_: None, shutdown=lambda: None, spin=lambda _node: None)
    GateNodeStub.overrides = {}
    wrapper = load_wrapper(monkeypatch, rclpy, GateNodeStub)
    node = wrapper.CmdVelSafetyGate()
    node._now = lambda: 0.0
    callbacks = {item.topic: item.callback for item in node.subscriptions}
    output = next(item for item in node.publishers if item.topic == "/cmd_vel").messages
    message = sys.modules["geometry_msgs.msg"].Twist()
    message.angular.z = 0.3

    callbacks["/cmd_vel_align"](message)

    assert output[-1].angular.z == 0.0


@pytest.mark.parametrize("value", ("", "   ", True, 7, None))
def test_gate_rejects_invalid_topic_parameters(monkeypatch, value):
    rclpy = SimpleNamespace(init=lambda **_: None, shutdown=lambda: None, spin=lambda _node: None)
    GateNodeStub.overrides = {"input_topic": value}
    wrapper = load_wrapper(monkeypatch, rclpy, GateNodeStub)
    with pytest.raises(ValueError, match="input_topic"):
        wrapper.CmdVelSafetyGate()


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


def test_main_treats_external_shutdown_as_clean_exit(monkeypatch):
    events = []
    rclpy = SimpleNamespace(
        init=lambda args: events.append("init"),
        shutdown=lambda: events.append("shutdown"),
    )
    wrapper = load_wrapper(monkeypatch, rclpy)
    rclpy.spin = lambda _node: (_ for _ in ()).throw(
        wrapper.NORMAL_SHUTDOWN_EXCEPTIONS[1]()
    )

    class RecordingNode:
        def publish_stop(self):
            events.append("stop")

        def destroy_node(self):
            events.append("destroy")

    monkeypatch.setattr(wrapper, "CmdVelSafetyGate", RecordingNode)

    wrapper.main()

    assert events == ["init", "stop", "destroy", "shutdown"]
