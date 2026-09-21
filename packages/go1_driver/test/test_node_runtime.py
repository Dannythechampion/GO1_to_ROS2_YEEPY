import importlib
import json
import math
import struct
import sys
from types import ModuleType, SimpleNamespace

import pytest

from go1_driver.robot_state import RemoteState, RobotState


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warning(self, text):
        self.lines.append(("warning", text))

    def error(self, text):
        self.lines.append(("error", text))


class Node:
    parameters = {}

    def __init__(self, *_args):
        self.publishers = {}
        self.subscriptions = {}
        self.timers = []
        self.logger = Logger()
        self._values = {}

    def declare_parameter(self, name, default):
        self._values[name] = type(self).parameters.get(name, default)
        return SimpleNamespace(value=self._values[name])

    def get_parameter(self, name):
        return SimpleNamespace(value=self._values[name])

    def create_publisher(self, _type, topic, _depth):
        self.publishers[topic] = Publisher(topic)
        return self.publishers[topic]

    def create_subscription(self, _type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic)

    def create_timer(self, period, callback):
        self.timers.append(SimpleNamespace(period=period, callback=callback))

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass


def twist(vx=0.0, vy=0.0, yaw=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=vx, y=vy, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=yaw),
    )


def odometry(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
    )))


class FakeRobot:
    def __init__(self):
        self.sent = []
        self.live = True
        self.remote = RemoteState(True)
        self.last_reply_fresh = False

    def send(self, command):
        self.sent.append(command)
        self.last_reply_fresh = self.live
        return RobotState(self.live, 2, (0.0, 0.0), 0.0, 0.29, (0.0, 0.6, 0.5, 2.0), 80, self.remote)

    def stand(self, repeats=30, period=0.01):
        self.sent.append("shutdown")


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now


@pytest.fixture
def driver(monkeypatch):
    modules = {}
    geometry = ModuleType("geometry_msgs.msg")
    geometry.Twist = lambda: twist()
    nav = ModuleType("nav_msgs.msg")
    nav.Odometry = object
    std = ModuleType("std_msgs.msg")
    std.String = type("String", (), {"__init__": lambda self: setattr(self, "data", "")})
    std.Bool = type("Bool", (), {"__init__": lambda self: setattr(self, "data", False)})
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    rclpy.spin = lambda _node: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    qos = ModuleType("rclpy.qos")
    qos.qos_profile_sensor_data = "sensor_data"
    modules.update({
        "geometry_msgs.msg": geometry, "nav_msgs.msg": nav, "std_msgs.msg": std,
        "rclpy": rclpy, "rclpy.node": node_module, "rclpy.qos": qos,
    })
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("go1_driver.node", None)
    module = importlib.import_module("go1_driver.node")
    clock = Clock()
    robot = FakeRobot()
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock.monotonic))
    monkeypatch.setattr(module, "UnitreeHighLevel", lambda **_kwargs: robot)

    def make(**parameters):
        Node.parameters = parameters
        return module.Go1Driver()

    yield SimpleNamespace(module=module, clock=clock, robot=robot, make=make)
    Node.parameters = {}
    sys.modules.pop("go1_driver.node", None)


def armed(driver, **parameters):
    return driver.make(arm=True, armed_confirmation=driver.module.ARMED_CONFIRMATION_TOKEN, **parameters)


def run(driver, node, seconds, command=None, step=0.01, odom=None):
    """Advance time, feeding cmd_vel (and optional odometry) and ticking the driver."""
    end = driver.clock.now + seconds
    while driver.clock.now < end - 1e-9:
        driver.clock.now = round(driver.clock.now + step, 6)
        if command is not None:
            node._cmd_vel_callback(command)
        if odom is not None:
            node._odom_callback(odom())
        node._timer_callback()


def last(node, topic):
    return node.publishers[topic].messages[-1]


def robot_report(node):
    return json.loads(last(node, "/go1/robot_state").data)


def test_dry_run_passes_commands_and_does_not_judge_execution(driver):
    node = driver.make()
    run(driver, node, 0.2, command=twist(0.15, 0.0, 0.2))
    applied = last(node, "/go1/cmd_vel_applied")
    assert applied.linear.x == pytest.approx(0.15)
    assert "/Odometry" not in node.subscriptions
    report = robot_report(node)
    assert report["armed"] is False and report["robot"] is None
    assert last(node, "/go1/manual_override").data is False


def test_remote_takeover_holds_stand_and_a_stale_goal_never_resumes(driver):
    node = armed(driver)
    run(driver, node, 0.3, command=twist(0.15, 0.0, 0.2))
    assert driver.robot.sent[-1].mode == 2

    driver.robot.remote = RemoteState(True, lx=0.0, ly=0.6)
    run(driver, node, 0.3, command=twist(0.15, 0.0, 0.2))
    assert driver.robot.sent[-1].mode == 0
    assert driver.robot.sent[-1].reason == "manual override: remote in use"
    assert last(node, "/go1/manual_override").data is True
    assert robot_report(node)["robot"]["remote"]["active"] is True

    # The operator lets go; Nav2 still has the old goal and keeps commanding it.
    driver.robot.remote = RemoteState(True)
    run(driver, node, 3.0, command=twist(0.15, 0.0, 0.2))
    assert last(node, "/go1/manual_override").data is False
    assert driver.robot.sent[-1].mode == 0
    assert driver.robot.sent[-1].reason.startswith("holding after manual override")

    # Navigation cancels its goal: zero for the re-arm period, then new commands pass.
    run(driver, node, 0.6, command=twist())
    run(driver, node, 0.1, command=twist(0.10, 0.0, 0.0))
    assert driver.robot.sent[-1].mode == 2
    assert driver.robot.sent[-1].vx == pytest.approx(0.10)


def test_commands_the_robot_ignores_become_an_execution_fault(driver):
    node = armed(driver)
    still = lambda: odometry(1.0, 2.0, 0.3)
    run(driver, node, 3.5, command=twist(0.0, 0.0, 0.4), odom=still)
    assert last(node, "/go1/execution_fault").data is False
    run(driver, node, 5.5, command=twist(0.0, 0.0, 0.4), odom=still)

    assert last(node, "/go1/execution_fault").data is True
    assert driver.robot.sent[-1].mode == 0
    assert robot_report(node)["execution_fault"] == "NOT_EXECUTING"
    assert any(level == "error" and "not following commands" in text for level, text in node.logger.lines)

    run(driver, node, 0.7, command=twist(), odom=still)
    assert last(node, "/go1/execution_fault").data is False


def test_link_loss_is_reported(driver):
    node = armed(driver)
    run(driver, node, 0.2, command=twist())
    assert robot_report(node)["link_up"] is True
    driver.robot.live = False
    run(driver, node, 0.7, command=twist())
    assert robot_report(node)["link_up"] is False
    assert any(level == "error" and "link down" in text for level, text in node.logger.lines)


def test_periodic_log_records_requested_and_applied_while_moving(driver):
    node = armed(driver)
    run(driver, node, 3.2, command=twist(0.0, 0.0, 0.3))
    lines = [text for level, text in node.logger.lines if level == "info" and "requested" in text]
    # One on the reason change plus one per second after it.
    assert len(lines) >= 4
    assert "requested vx=0.000 vy=0.000 yaw=0.300" in lines[-1]
    assert "robot mode=2" in lines[-1]


def test_non_finite_cmd_vel_cannot_break_the_state_report(driver):
    node = armed(driver)
    run(driver, node, 0.2, command=twist(float("nan"), 0.0, 0.0))
    report = robot_report(node)
    assert report["requested"]["vx"] == 0.0
    assert report["applied"]["mode"] == 0
