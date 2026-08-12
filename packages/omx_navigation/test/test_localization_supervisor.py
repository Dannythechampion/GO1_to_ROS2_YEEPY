import importlib
import io
import json
import math
import sys
from types import ModuleType, SimpleNamespace

import pytest


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Clock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return SimpleNamespace(
            nanoseconds=int(self.seconds * 1_000_000_000),
            to_msg=lambda: SimpleNamespace(sec=int(self.seconds), nanosec=0),
        )


class Node:
    def __init__(self, *_args):
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.clock = Clock()
        self.node_names = []
        self.destroyed = False
        self.logger = SimpleNamespace(info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None)

    def declare_parameter(self, _name, default):
        return SimpleNamespace(value=default)

    def create_publisher(self, _type, topic, _depth):
        publisher = Publisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _type, topic, callback, qos):
        subscription = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return self.clock

    def get_node_names(self):
        return self.node_names

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        self.destroyed = True


class PoseWithCovarianceStamped:
    def __init__(self):
        self.header = SimpleNamespace(frame_id="", stamp=None)
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[],
        )


@pytest.fixture
def supervisor_module(monkeypatch):
    modules = {}
    for package, classes in {
        "geometry_msgs.msg": {"PoseStamped": object, "PoseWithCovarianceStamped": PoseWithCovarianceStamped},
        "nav_msgs.msg": {"OccupancyGrid": object, "Odometry": object},
        "sensor_msgs.msg": {"LaserScan": object},
        "std_msgs.msg": {"Bool": type("Bool", (), {"__init__": lambda self: setattr(self, "data", False)}), "String": type("String", (), {"__init__": lambda self: setattr(self, "data", "")})},
        "tf2_msgs.msg": {"TFMessage": object},
    }.items():
        module = ModuleType(package)
        for name, value in classes.items():
            setattr(module, name, value)
        modules[package] = module
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda _node: None
    modules["rclpy"] = rclpy
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    modules["rclpy.node"] = node_module
    qos_module = ModuleType("rclpy.qos")
    qos_module.QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
    qos_module.ReliabilityPolicy = SimpleNamespace(RELIABLE="reliable")
    qos_module.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile")
    qos_module.HistoryPolicy = SimpleNamespace(KEEP_LAST="keep_last")
    qos_module.qos_profile_sensor_data = "sensor_data"
    modules["rclpy.qos"] = qos_module
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("omx_navigation.localization_supervisor", None)
    module = importlib.import_module("omx_navigation.localization_supervisor")
    yield module
    sys.modules.pop("omx_navigation.localization_supervisor", None)


def header(frame_id="map"):
    return SimpleNamespace(frame_id=frame_id, stamp=SimpleNamespace(sec=0, nanosec=0))


def pose_message(x=0.0, y=0.0, frame_id="map", covariance=None):
    return SimpleNamespace(
        header=header(frame_id),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=list(covariance if covariance is not None else range(36)),
        ),
    )


def map_message():
    return SimpleNamespace(
        header=header(),
        info=SimpleNamespace(
            width=11, height=11, resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-5.0, y=-5.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * 121,
    )


def scan_message():
    return SimpleNamespace(
        header=header("laser"), ranges=[1.0], angle_min=0.0, angle_increment=1.0,
        range_min=0.1, range_max=10.0,
    )


def publisher(node, topic):
    return next(item for item in node.publishers if item.topic == topic)


def test_supervisor_wires_topics_refines_map_pose_and_emits_finite_json(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    module = supervisor_module
    result = SearchResult(PoseScore(Pose2D(1.0, 2.0, 0.5), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(module, "coarse_search", lambda *_args: result)
    node = module.LocalizationSupervisor()
    assert {sub.topic for sub in node.subscriptions} == {"/map", "/scan", "/Odometry", "/initialpose", "/slam_toolbox/pose", "/tf"}
    assert {pub.topic for pub in node.publishers} == {"/slam_localization/initialpose", "~/status", "~/ready"}

    node._on_map(map_message())
    node._on_scan(scan_message())
    covariance = [float(index) for index in range(36)]
    node._on_initial_pose(pose_message(covariance=covariance))
    node._on_scan(scan_message())
    node._on_status_timer()

    refined = publisher(node, "/slam_localization/initialpose").messages[-1]
    assert refined.header.frame_id == "map"
    assert refined.header.stamp.sec == 0
    assert refined.pose.covariance == covariance
    assert refined.pose.pose.position.x == pytest.approx(1.0)
    assert refined.pose.pose.orientation.z == pytest.approx(math.sin(0.25))

    node.clock.seconds = 0.1
    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    node._on_slam_pose(SimpleNamespace(header=header(), pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_status_timer()
    node.clock.seconds = 3.1
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    node._on_slam_pose(SimpleNamespace(header=header(), pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_status_timer()
    status = json.loads(publisher(node, "~/status").messages[-1].data)
    assert status["state"] == "READY"
    assert status["error"] == "NONE"
    assert math.isfinite(status["stamp"])
    assert publisher(node, "~/ready").messages[-1].data is True


def test_supervisor_rejects_stale_inputs_odom_time_rollback_and_amcl(supervisor_module):
    module = supervisor_module
    node = module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node.clock.seconds = 1.0
    node._on_initial_pose(pose_message())
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "INPUT_MISSING"

    node._on_odom(SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))) )
    node.clock.seconds = 0.5
    node._on_odom(SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0)))) )
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "ODOM_RESET"

    node = module.LocalizationSupervisor()
    node.node_names = ["/amcl"]
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "TF_CONFLICT"


@pytest.mark.parametrize(
    ("result", "error"),
    (
        ((0.1, False), "LOW_OVERLAP"),
        ((0.8, True), "AMBIGUOUS"),
    ),
)
def test_supervisor_reports_search_quality_failures(supervisor_module, monkeypatch, result, error):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    overlap, ambiguous = result
    search = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), overlap, 0.0, overlap, 1), None, ambiguous)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: search)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    node._on_scan(scan_message())
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == error


def test_supervisor_rejects_non_map_initialpose_and_flushes_csv(supervisor_module, monkeypatch):
    module = supervisor_module
    node = module.LocalizationSupervisor()
    node._on_initial_pose(pose_message(frame_id="odom"))
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "POSE_OUTSIDE_MAP"

    class FlushingCsv(io.StringIO):
        def __init__(self):
            super().__init__()
            self.flushed = False

        def flush(self):
            self.flushed = True
            super().flush()

    node._csv_file = FlushingCsv()
    node._csv_writer = module.csv.DictWriter(node._csv_file, fieldnames=("state", "error", "message_ko", "attempt", "overlap", "ambiguity_margin", "stamp"))
    monkeypatch.setitem(module._MESSAGES_KO, node._last_transition.error, '��표,따��표"')
    node._publish_status()
    assert node._csv_file.flushed is True
    assert '"��표,따��표"""' in node._csv_file.getvalue()
    node.destroy_node()
    assert node.destroyed is True


def test_supervisor_waits_for_post_click_scan_and_fresh_slam_tf_before_ready(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    search_calls = []
    result = SearchResult(PoseScore(Pose2D(1.0, 2.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: search_calls.append(1) or result)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_status_timer()
    assert search_calls == []

    node._on_scan(scan_message())
    node._on_status_timer()
    assert search_calls == [1]
    node.clock.seconds = 3.1
    node._on_scan(scan_message())
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["state"] != "READY"

    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    node._on_slam_pose(SimpleNamespace(header=header(), pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_status_timer()
    node.clock.seconds = 6.2
    node._on_scan(scan_message())
    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    node._on_slam_pose(SimpleNamespace(header=header(), pose=SimpleNamespace(position=SimpleNamespace(x=1.1, y=2.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["state"] == "READY"


def test_supervisor_clears_invalid_data_and_uses_required_qos(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    subscriptions = {subscription.topic: subscription for subscription in node.subscriptions}
    assert subscriptions["/map"].qos.durability == "transient_local"
    assert subscriptions["/scan"].qos == "sensor_data"
    assert subscriptions["/Odometry"].qos == "sensor_data"
    assert subscriptions["/tf"].qos == "sensor_data"

    node._on_map(map_message())
    invalid = map_message()
    invalid.data = []
    node._on_map(invalid)
    assert node._grid is None and node._map_received_at is None
    node._on_scan(scan_message())
    invalid_scan = scan_message()
    invalid_scan.range_min = 10.0
    invalid_scan.range_max = 1.0
    node._on_scan(invalid_scan)
    assert node._points is None and node._scan_received_at is None


def test_supervisor_uses_odom_source_time_and_resets_fault_on_new_initial_pose(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    odom = lambda x, sec: SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)),
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0))),
    )
    node._on_odom(odom(0.0, 10))
    node._on_odom(odom(0.1, 9))
    assert node._odom_reset is True
    node._on_initial_pose(pose_message())
    assert node._odom_reset is False


def test_supervisor_rejects_large_consecutive_slam_pose_jump(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    result = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: result)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_status_timer()
    node.clock.seconds = 0.1
    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    slam = lambda x: SimpleNamespace(header=header(), pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    node._on_slam_pose(slam(0.0))
    node._on_scan(scan_message())
    node._on_status_timer()
    node.clock.seconds = 0.2
    node._on_tf(SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"), child_frame_id="body_nav")]))
    node._on_slam_pose(slam(1.0))
    node._on_scan(scan_message())
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "POSE_OUTSIDE_MAP"


def test_initial_pose_captures_scan_sequence_so_later_map_does_not_reuse_pre_click_scan(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    calls = []
    result = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: calls.append(1) or result)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_map(map_message())
    node._on_status_timer()
    assert calls == []
    node._on_scan(scan_message())
    node._on_status_timer()
    assert calls == [1]


def test_republish_resets_slam_epoch_baseline_and_jump_metrics(supervisor_module):
    from omx_navigation.scan_map_quality import Pose2D

    node = supervisor_module.LocalizationSupervisor()
    node._last_slam_pose = Pose2D(1.0, 1.0, 1.0)
    node._slam_received_at = 2.0
    node._slam_position_jump = 3.0
    node._slam_yaw_jump = 2.0
    node._publish_refined_pose(Pose2D(0.0, 0.0, 0.0), [0.0] * 36)
    assert node._last_slam_pose is None
    assert node._slam_received_at is None
    assert node._slam_position_jump == 0.0
    assert node._slam_yaw_jump == 0.0


def test_source_stamp_rejects_out_of_range_nanoseconds(supervisor_module):
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=1_000_000_000)))
    assert supervisor_module.LocalizationSupervisor._source_stamp(message, 7.0) == 7.0


def test_rapid_clicks_keep_only_latest_search_queued(supervisor_module):
    class BlockingFuture:
        def __init__(self):
            self.cancel_called = False

        def done(self):
            return False

        def cancel(self):
            self.cancel_called = True
            return False

    class BlockingExecutor:
        def __init__(self):
            self.futures = []

        def submit(self, *_args):
            future = BlockingFuture()
            self.futures.append(future)
            return future

    node = supervisor_module.LocalizationSupervisor()
    node._search_executor = BlockingExecutor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    assert len(node._search_executor.futures) == 1
    assert node._search_executor.futures[0].cancel_called is True
