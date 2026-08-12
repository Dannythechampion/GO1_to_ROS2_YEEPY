import importlib
import sys
from types import ModuleType, SimpleNamespace

import pytest


class Future:
    def __init__(self, value=None):
        self.value = value
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        return self.value

    def complete(self):
        for callback in list(self.callbacks):
            callback(self)


class GoalHandle:
    accepted = True

    def __init__(self):
        self.cancelled = False
        self.cancel_future = Future()
        self.result_future = Future(SimpleNamespace(status=0))

    def cancel_goal_async(self):
        self.cancelled = True
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


class ActionClient:
    def __init__(self, *_args):
        self.sent = []
        self.responses = []
        self.wait_timeouts = []

    def wait_for_server(self, **_kwargs):
        self.wait_timeouts.append(_kwargs["timeout_sec"])
        return True

    def send_goal_async(self, goal, **_kwargs):
        self.sent.append(goal)
        future = Future()
        self.responses.append(future)
        return future


class Node:
    def __init__(self, *_args):
        self.subscriptions = []
        self.logger = SimpleNamespace(info=lambda *_: None, error=lambda *_: None, warning=lambda *_: None)

    def declare_parameter(self, _name, default):
        return SimpleNamespace(value=default)

    def create_subscription(self, _type, topic, callback, _depth):
        result = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(result)
        return result

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))


@pytest.fixture
def bridge_module(monkeypatch):
    geometry = ModuleType("geometry_msgs.msg")
    geometry.PoseStamped = object
    action_msgs = ModuleType("action_msgs.msg")
    action_msgs.GoalStatus = SimpleNamespace(STATUS_SUCCEEDED=0)
    nav2 = ModuleType("nav2_msgs.action")
    nav2.NavigateToPose = type("NavigateToPose", (), {"Goal": type("Goal", (), {})})
    std = ModuleType("std_msgs.msg")
    std.Bool = type("Bool", (), {"__init__": lambda self, data=False: setattr(self, "data", data)})
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda _node: None
    action = ModuleType("rclpy.action")
    action.ActionClient = ActionClient
    node = ModuleType("rclpy.node")
    node.Node = Node
    for name, module in {
        "geometry_msgs.msg": geometry, "action_msgs.msg": action_msgs,
        "nav2_msgs.action": nav2, "std_msgs.msg": std, "rclpy": rclpy,
        "rclpy.action": action, "rclpy.node": node,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("omx_navigation.rviz_goal_bridge", None)
    module = importlib.import_module("omx_navigation.rviz_goal_bridge")
    yield module
    sys.modules.pop("omx_navigation.rviz_goal_bridge", None)


def pose():
    return SimpleNamespace(header=SimpleNamespace(frame_id="map"), pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)))


def test_goal_bridge_rejects_until_ready_and_cancels_response_race(bridge_module):
    node = bridge_module.RvizGoalBridge()
    assert {item.topic for item in node.subscriptions} == {"goal_pose", "/localization_supervisor/ready"}
    node._on_goal_pose(pose())
    assert node._action_client.sent == []

    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    assert len(node._action_client.sent) == 1
    node._on_ready(SimpleNamespace(data=False))
    response = node._action_client.responses[0]
    response.value = GoalHandle()
    response.complete()
    handle = response.value
    assert handle.cancelled is True
    handle.cancel_future.complete()
    assert node._active_goal_handle is handle
    handle.result_future.complete()
    assert node._active_goal_handle is None


def test_goal_bridge_allows_one_inflight_or_active_goal_and_ignores_stale_callbacks(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    node._on_goal_pose(pose())
    assert len(node._action_client.sent) == 1
    pending = node._action_client.responses[0]
    pending.value = GoalHandle()
    pending.complete()
    node._on_goal_pose(pose())
    assert len(node._action_client.sent) == 1
    pending.value.result_future.complete()
    node._on_goal_pose(pose())
    assert len(node._action_client.sent) == 2
    assert node._action_client.wait_timeouts == [0.0, 0.0]


def test_goal_bridge_rejected_or_stale_response_never_claims_active_goal(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    response = node._action_client.responses[0]
    response.value = SimpleNamespace(accepted=False)
    response.complete()
    assert node._active_goal_handle is None
    node._on_goal_pose(pose())
    next_response = node._action_client.responses[1]
    node._on_goal_response(response, 1)  # stale callback after a new request
    assert node._active_goal_handle is None
    next_response.value = GoalHandle()
    next_response.complete()
    assert node._active_goal_handle is next_response.value


def test_goal_cancel_acknowledgement_keeps_handle_active_until_result(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    response = node._action_client.responses[0]
    response.value = GoalHandle()
    response.complete()
    handle = response.value
    node._on_ready(SimpleNamespace(data=False))
    handle.cancel_future.complete()
    assert node._active_goal_handle is handle
    node._on_goal_pose(pose())
    assert len(node._action_client.sent) == 1
    handle.result_future.complete()
    assert node._active_goal_handle is None
