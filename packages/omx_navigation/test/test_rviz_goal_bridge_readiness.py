import importlib
import json
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
        self.cancel_futures = []
        self.result_future = Future(SimpleNamespace(status=0))

    def cancel_goal_async(self):
        self.cancelled = True
        future = Future(SimpleNamespace(goals_canceling=[1]))
        self.cancel_futures.append(future)
        return future

    @property
    def cancel_future(self):
        return self.cancel_futures[-1]

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


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _namespace(**fields):
    return SimpleNamespace(**fields)


class Marker:
    ARROW = 0
    TEXT_VIEW_FACING = 9
    ADD = 0

    def __init__(self):
        self.header = _namespace(frame_id="")
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = _namespace(position=_namespace(x=0.0, y=0.0, z=0.0), orientation=_namespace(x=0.0, y=0.0, z=0.0, w=1.0))
        self.scale = _namespace(x=0.0, y=0.0, z=0.0)
        self.color = _namespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.text = ""


class Node:
    def __init__(self, *_args):
        self.subscriptions = []
        self.publishers = []
        self.timers = []
        self.clock = SimpleNamespace(seconds=0.0)
        self.logger = SimpleNamespace(info=lambda *_: None, error=lambda *_: None, warning=lambda *_: None)

    def declare_parameter(self, _name, default):
        return SimpleNamespace(value=default)

    def create_subscription(self, _type, topic, callback, _depth):
        result = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(result)
        return result

    def create_publisher(self, _type, topic, _depth):
        publisher = Publisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, period, callback):
        result = SimpleNamespace(period=period, callback=callback)
        self.timers.append(result)
        return result

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=round(self.clock.seconds * 1_000_000_000))
        )

    def destroy_node(self):
        self.destroyed = True


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
    std.String = type("String", (), {"__init__": lambda self: setattr(self, "data", "")})
    visualization = ModuleType("visualization_msgs.msg")
    visualization.Marker = Marker
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
        "rclpy.action": action, "rclpy.node": node, "visualization_msgs.msg": visualization,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("omx_navigation.rviz_goal_bridge", None)
    module = importlib.import_module("omx_navigation.rviz_goal_bridge")
    yield module
    sys.modules.pop("omx_navigation.rviz_goal_bridge", None)


def pose(x=1.0, y=2.0, frame_id="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def statuses(node):
    publisher = next(item for item in node.publishers if item.topic == "/navigation/goal_status")
    return [json.loads(message.data) for message in publisher.messages]


def markers(node):
    return next(item for item in node.publishers if item.topic == "/navigation/goal_marker").messages


def accept(node, index=-1):
    response = node._action_client.responses[index]
    response.value = GoalHandle()
    response.complete()
    return response.value


def test_main_treats_external_shutdown_as_clean_exit(bridge_module, monkeypatch):
    events = []

    class RecordingNode:
        def destroy_node(self):
            events.append("destroy")

    monkeypatch.setattr(bridge_module, "RvizGoalBridge", RecordingNode)
    bridge_module.rclpy.init = lambda **_kwargs: events.append("init")
    bridge_module.rclpy.spin = lambda _node: (_ for _ in ()).throw(
        bridge_module.NORMAL_SHUTDOWN_EXCEPTIONS[1]()
    )
    bridge_module.rclpy.shutdown = lambda: events.append("shutdown")

    bridge_module.main()

    assert events == ["init", "destroy", "shutdown"]


def test_goal_bridge_rejects_until_ready_and_cancels_response_race(bridge_module):
    node = bridge_module.RvizGoalBridge()
    assert {"goal_pose", "/localization_supervisor/ready"} <= {item.topic for item in node.subscriptions}
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


def test_new_goal_replaces_the_active_one_and_its_late_result_is_ignored(bridge_module):
    """Clicking again used to be ignored here while bt_navigator took the click
    on its own; with this node as the only path a new click must win."""
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    first = node._action_client.responses[0]
    first.value = GoalHandle()
    first.complete()
    assert node._active_goal_handle is first.value

    node._on_goal_pose(pose(3.0, 4.0))
    assert len(node._action_client.sent) == 2
    second = node._action_client.responses[1]
    second.value = GoalHandle()
    second.complete()
    assert node._active_goal_handle is second.value
    assert statuses(node)[-2]["state"] == "PREEMPTED"

    first.value.result_future.value = SimpleNamespace(status=6)
    first.value.result_future.complete()
    assert node._active_goal_handle is second.value
    assert statuses(node)[-1]["state"] == "ACCEPTED"
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


def test_pending_goal_is_cancelled_after_readiness_epoch_changes(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())

    node._on_ready(SimpleNamespace(data=False))
    node._on_ready(SimpleNamespace(data=True))
    response = node._action_client.responses[0]
    response.value = GoalHandle()
    response.complete()

    assert response.value.cancelled is True
    assert node._active_goal_handle is response.value


def test_cancel_rejection_is_retried_until_nav2_finishes_goal(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    response = node._action_client.responses[0]
    response.value = GoalHandle()
    response.complete()
    handle = response.value

    node._on_ready(SimpleNamespace(data=False))
    assert len(handle.cancel_futures) == 1
    handle.cancel_future.value = SimpleNamespace(goals_canceling=[])
    handle.cancel_future.complete()
    node.timers[0].callback()

    assert len(handle.cancel_futures) == 2
    assert node._active_goal_handle is handle


def test_ready_heartbeat_timeout_revokes_and_cancels_active_goal(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    response = node._action_client.responses[0]
    response.value = GoalHandle()
    response.complete()

    node.clock.seconds = 0.31
    node.timers[0].callback()

    assert node._goal_gate.ready is False
    assert response.value.cancelled is True


def test_click_before_localization_is_visibly_ignored(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_goal_pose(pose())

    status = statuses(node)[-1]
    assert status["state"] == "IGNORED"
    assert status["reason"] == "localization not ready"
    text = [marker for marker in markers(node) if marker.type == 9][-1]
    assert text.text == "IGNORED: localization not ready"
    assert text.color.r > 0.9 and text.color.g < 0.3


def test_arrival_is_published_as_a_status_and_a_green_marker(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose(5.0, -3.0))
    handle = accept(node)
    handle.result_future.value = SimpleNamespace(status=0)  # STATUS_SUCCEEDED in the stub
    handle.result_future.complete()

    assert [item["state"] for item in statuses(node)] == ["SENT", "ACCEPTED", "SUCCEEDED"]
    assert statuses(node)[-1]["goal"]["x"] == 5.0
    text = [marker for marker in markers(node) if marker.type == 9][-1]
    assert text.text == "ARRIVED"
    assert text.color.g > 0.8


def test_feedback_updates_distance_at_most_once_a_second(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    accept(node)
    token = node._active_token
    for seconds, distance in ((1.0, 3.0), (1.5, 2.8), (2.1, 2.5)):
        node.clock.seconds = seconds
        node._on_ready(SimpleNamespace(data=True))
        node._on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=distance)), token)
    distances = [item["distance_remaining"] for item in statuses(node) if item["distance_remaining"] is not None]
    assert distances == [3.0, 2.5]


def test_superseded_request_that_nav2_accepts_anyway_is_cancelled(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    node._on_goal_pose(pose(2.0, 2.0))
    late = accept(node, 0)
    assert late.cancelled is True
    assert node._active_goal_handle is None
    current = accept(node, 1)
    assert node._active_goal_handle is current


@pytest.mark.parametrize(("topic", "reason"), (
    ("/go1/manual_override", "manual override"),
    ("/go1/execution_fault", "robot not following commands"),
))
def test_taking_the_robot_away_cancels_the_goal_and_refuses_new_ones(bridge_module, topic, reason):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    handle = accept(node)
    inhibit = next(item for item in node.subscriptions if item.topic == topic).callback

    inhibit(SimpleNamespace(data=True))
    assert handle.cancelled is True
    node._on_goal_pose(pose(4.0, 4.0))
    assert len(node._action_client.sent) == 1
    assert statuses(node)[-1] == {**statuses(node)[-1], "state": "IGNORED", "reason": reason}

    handle.cancel_future.complete()
    handle.result_future.value = SimpleNamespace(status=5)
    handle.result_future.complete()
    canceled = [item for item in statuses(node) if item["state"] == "CANCELED"][-1]
    assert canceled["reason"] == reason

    inhibit(SimpleNamespace(data=False))
    node._on_goal_pose(pose(4.0, 4.0))
    assert len(node._action_client.sent) == 2


def test_pending_goal_is_cancelled_when_the_remote_is_taken_before_acceptance(bridge_module):
    node = bridge_module.RvizGoalBridge()
    node._on_ready(SimpleNamespace(data=True))
    node._on_goal_pose(pose())
    override = next(item for item in node.subscriptions if item.topic == "/go1/manual_override").callback
    override(SimpleNamespace(data=True))
    override(SimpleNamespace(data=False))
    handle = accept(node)
    assert handle.cancelled is True


def test_subscribes_to_every_input_that_can_withdraw_the_robot(bridge_module):
    node = bridge_module.RvizGoalBridge()
    assert {item.topic for item in node.subscriptions} == {
        "goal_pose", "/localization_supervisor/ready", "/go1/manual_override", "/go1/execution_fault",
    }
    assert {item.topic for item in node.publishers} == {"/navigation/goal_status", "/navigation/goal_marker"}

