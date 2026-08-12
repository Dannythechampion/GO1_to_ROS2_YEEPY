import importlib
import sys
from types import ModuleType, SimpleNamespace

import pytest


class FakeTransformException(Exception):
    pass


class FakeDuration:
    def __init__(self, *, seconds):
        self.seconds = seconds


class FakeTime:
    pass


class FakeLogger:
    def __init__(self):
        self.warnings = []
        self.infos = []

    def warn(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


class FakeNode:
    def __init__(self, *_args):
        self._parameters = {}
        self._logger = FakeLogger()
        self.destroyed = False

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self._parameters.get(name, default))

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        self.destroyed = True


class FakeBuffer:
    def __init__(self):
        self.calls = []
        self.responses = []

    def lookup_transform(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response


class FakeBroadcaster:
    def __init__(self, *_args):
        self.messages = []

    def sendTransform(self, message):
        self.messages.append(message)


class FakeTransformStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id=None)
        self.child_frame_id = None
        self.transform = SimpleNamespace(
            translation=SimpleNamespace(x=None, y=None, z=None),
            rotation=SimpleNamespace(x=None, y=None, z=None, w=None),
        )


def source_transform(stamp="source-stamp"):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.4),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.8660254037844386),
        ),
    )


@pytest.fixture
def planar_module(monkeypatch):
    original_module = sys.modules.get("omx_navigation.planar_base_frame")
    geometry_msgs = ModuleType("geometry_msgs.msg")
    geometry_msgs.TransformStamped = FakeTransformStamped
    rclpy = ModuleType("rclpy")
    rclpy.events = []
    rclpy.init = lambda **_kwargs: rclpy.events.append("init")
    rclpy.shutdown = lambda: rclpy.events.append("shutdown")
    rclpy.spin = lambda _node: rclpy.events.append("spin")
    rclpy_duration = ModuleType("rclpy.duration")
    rclpy_duration.Duration = FakeDuration
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_time = ModuleType("rclpy.time")
    rclpy_time.Time = FakeTime
    tf2_ros = ModuleType("tf2_ros")
    tf2_ros.Buffer = FakeBuffer
    tf2_ros.TransformBroadcaster = FakeBroadcaster
    tf2_ros.TransformException = FakeTransformException
    tf2_ros.TransformListener = lambda *_args: object()
    for name, module in {
        "geometry_msgs.msg": geometry_msgs,
        "rclpy": rclpy,
        "rclpy.duration": rclpy_duration,
        "rclpy.node": rclpy_node,
        "rclpy.time": rclpy_time,
        "tf2_ros": tf2_ros,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("omx_navigation.planar_base_frame", None)
    module = importlib.import_module("omx_navigation.planar_base_frame")
    yield module
    sys.modules.pop("omx_navigation.planar_base_frame", None)
    if original_module is not None:
        sys.modules["omx_navigation.planar_base_frame"] = original_module


def test_node_derives_body_nav_from_body_without_rebroadcasting_body(planar_module):
    node = planar_module.PlanarBaseFrame()
    node._buffer.responses = [source_transform()]

    node._publish_planar_transform()

    args, kwargs = node._buffer.calls[0]
    assert args[:2] == ("camera_init", "body")
    assert isinstance(args[2], FakeTime)
    assert kwargs["timeout"].seconds <= 0.04
    output = node._broadcaster.messages[0]
    assert output.header.frame_id == "camera_init"
    assert output.child_frame_id == "body_nav"
    assert output.header.stamp == "source-stamp"
    assert output.child_frame_id != "body"


@pytest.mark.parametrize(
    ("odom", "source", "planar"),
    (("camera_init", "body", "camera_init"), ("camera_init", "body", "body")),
)
def test_invalid_frame_topology_is_rejected(planar_module, odom, source, planar):
    node = planar_module.PlanarBaseFrame()
    node._odom_frame = odom
    node._source_base_frame = source
    node._planar_base_frame = planar

    with pytest.raises(ValueError):
        node._validate_frame_topology()


def test_lookup_failures_are_throttled_and_recovery_is_reported_once(planar_module, monkeypatch):
    node = planar_module.PlanarBaseFrame()
    node._buffer.responses = [
        FakeTransformException("missing tf"),
        FakeTransformException("missing tf"),
        FakeTransformException("missing tf"),
        source_transform(),
    ]
    moments = iter((0.0, 1.0, 5.0, 6.0))
    monkeypatch.setattr(planar_module.time, "monotonic", lambda: next(moments))

    node._publish_planar_transform()
    node._publish_planar_transform()
    node._publish_planar_transform()
    node._publish_planar_transform()

    assert len(node._logger.warnings) == 2
    assert len(node._logger.infos) == 1
    assert len(node._broadcaster.messages) == 1


def test_main_destroys_the_node_and_shuts_down_rclpy(planar_module):
    created = []
    original = planar_module.PlanarBaseFrame

    class RecordingNode(original):
        def __init__(self):
            super().__init__()
            created.append(self)

    planar_module.PlanarBaseFrame = RecordingNode
    planar_module.main()

    assert created[0].destroyed is True
    assert sys.modules["rclpy"].events == ["init", "spin", "shutdown"]
