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
        subscription = SimpleNamespace(
            message_type=_type, topic=topic, callback=callback, qos=qos
        )
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


def ros_stamp(seconds):
    whole = math.floor(seconds)
    return SimpleNamespace(sec=whole, nanosec=round((seconds - whole) * 1_000_000_000))


def header(frame_id="map", stamp=0.000001):
    return SimpleNamespace(frame_id=frame_id, stamp=ros_stamp(stamp))


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


def map_message(*, occupied_world=()):
    data = [0] * 121
    for x, y in occupied_world:
        column = math.floor(x + 5.0)
        row = math.floor(y + 5.0)
        data[row * 11 + column] = 100
    return SimpleNamespace(
        header=header(),
        info=SimpleNamespace(
            width=11, height=11, resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-5.0, y=-5.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=data,
    )


def scan_message(*, ranges=(1.0,), stamp=0.000001):
    return SimpleNamespace(
        header=header("laser", stamp), ranges=list(ranges), angle_min=0.0, angle_increment=0.0,
        range_min=0.1, range_max=10.0,
    )


def odom_message(x=0.0, y=0.0, *, stamp=0.000001):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=ros_stamp(stamp)),
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))),
    )


def slam_message(x=0.0, y=0.0, yaw=0.0, *, stamp=0.000001):
    return SimpleNamespace(
        header=header(stamp=stamp),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                ),
            ),
        ),
    )


def tf_edge(parent, child, x=0.0, y=0.0, yaw=0.0, *, stamp=0.000001):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent, stamp=ros_stamp(stamp)),
        child_frame_id=child,
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=0.0),
            rotation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
            ),
        ),
    )


def relevant_tf(map_x=0.0, map_y=0.0, map_yaw=0.0, *, source_stamp=0.000001):
    return SimpleNamespace(transforms=[
        tf_edge("map", "camera_init", map_x, map_y, map_yaw, stamp=source_stamp + 0.5),
        tf_edge("camera_init", "body_nav", stamp=source_stamp),
    ])


class ImmediateFuture:
    def __init__(self, function, *args):
        self._result = function(*args)

    def done(self):
        return True

    def cancel(self):
        return False

    def result(self):
        return self._result


class ImmediateExecutor:
    def submit(self, function, *args):
        return ImmediateFuture(function, *args)

    def shutdown(self, **_kwargs):
        pass


def use_immediate_search(node):
    node._search_executor.shutdown(wait=False, cancel_futures=True)
    node._search_executor = ImmediateExecutor()


def initialize_high_quality_search(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    result = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.9, 0.0, 0.9, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args, **_kwargs: result)
    node = supervisor_module.LocalizationSupervisor()
    use_immediate_search(node)
    node._on_map(map_message(occupied_world=((1.0, 0.0),)))
    node._on_scan(scan_message())
    node.clock.seconds = 0.05
    node._on_initial_pose(pose_message())
    node.clock.seconds = 0.10
    node._on_scan(scan_message(stamp=0.10))
    node.clock.seconds = 0.20
    node._on_status_timer()
    return node


def feed_matching_live_inputs(node, now):
    node.clock.seconds = now
    node._on_odom(odom_message(stamp=now))
    node._on_tf(relevant_tf(source_stamp=now))
    node._on_scan(scan_message(stamp=now))
    node._on_slam_pose(slam_message(stamp=now))


def advance_to_ready(node):
    feed_matching_live_inputs(node, 0.30)
    node._on_ready_heartbeat()
    feed_matching_live_inputs(node, 3.40)
    node._on_ready_heartbeat()


def publisher(node, topic):
    return next(item for item in node.publishers if item.topic == topic)


@pytest.mark.parametrize("invalid_kind", ("missing_header", "malformed_covariance", "malformed", "nonfinite", "near_zero_quaternion", "overflow_quaternion"))
def test_invalid_new_initialpose_revokes_existing_ready_and_clears_prior_localization(invalid_kind, supervisor_module, monkeypatch):
    from omx_navigation.localization_state import ErrorCode, LocalizationState

    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    advance_to_ready(node)
    assert node._machine.state is LocalizationState.READY

    invalid = pose_message()
    if invalid_kind == "missing_header":
        del invalid.header
    elif invalid_kind == "malformed_covariance":
        invalid.pose.covariance = None
    elif invalid_kind == "malformed":
        invalid.pose.pose.orientation.w = "not-a-number"
    elif invalid_kind == "nonfinite":
        invalid.pose.pose.orientation.w = float("nan")
    elif invalid_kind == "near_zero_quaternion":
        invalid.pose.pose.orientation.w = 1e-13
    else:
        invalid.pose.pose.orientation.x = 1e308
        invalid.pose.pose.orientation.y = 1e308
        invalid.pose.pose.orientation.z = 1e308
        invalid.pose.pose.orientation.w = 1e308
    node._on_initial_pose(invalid)
    node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.LOST
    assert node._last_transition.error is ErrorCode.POSE_OUTSIDE_MAP
    assert node._slam_pose is None
    assert node._slam_pose_handshake is False
    assert node._current_base_pose is None
    assert node._quality_received_at is None
    assert publisher(node, "~/ready").messages[-1].data is False


@pytest.mark.parametrize(
    "invalid_covariance",
    (
        [],
        [0.0] * 35,
        [0.0] * 37,
        "0" * 36,
        [False] + [0.0] * 35,
        [0.0] * 35 + [float("nan")],
        [0.0] * 35 + [float("inf")],
    ),
    ids=("empty", "short", "long", "string", "bool", "nan", "infinity"),
)
def test_invalid_initialpose_covariance_revokes_existing_ready(invalid_covariance, supervisor_module, monkeypatch):
    from omx_navigation.localization_state import ErrorCode, LocalizationState

    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    advance_to_ready(node)
    assert node._machine.state is LocalizationState.READY

    invalid = pose_message()
    invalid.pose.covariance = invalid_covariance
    node._on_initial_pose(invalid)
    node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.LOST
    assert node._last_transition.error is ErrorCode.POSE_OUTSIDE_MAP
    assert publisher(node, "~/ready").messages[-1].data is False


def test_stationary_tf_updates_keep_verification_alive_after_one_slam_handshake(supervisor_module, monkeypatch):
    from omx_navigation.localization_state import LocalizationState

    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    node.clock.seconds = 0.30
    node._on_odom(odom_message(stamp=0.30))
    node._on_tf(relevant_tf(source_stamp=0.30))
    node._on_scan(scan_message(stamp=0.30))
    node._on_slam_pose(slam_message(stamp=0.30))
    node._on_ready_heartbeat()

    for now in (0.70, 1.10, 1.50, 1.90, 2.30, 2.70, 3.10, 3.50):
        node.clock.seconds = now
        node._on_odom(odom_message(stamp=now))
        node._on_tf(relevant_tf(source_stamp=now))
        node._on_scan(scan_message(stamp=now))
        node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.READY
    assert publisher(node, "~/ready").messages[-1].data is True


def test_map_tf_discontinuity_fails_closed_as_tf_conflict(supervisor_module, monkeypatch):
    from omx_navigation.localization_state import ErrorCode, LocalizationState

    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    node.clock.seconds = 0.30
    node._on_odom(odom_message(stamp=0.30))
    node._on_tf(relevant_tf(source_stamp=0.30))
    node._on_scan(scan_message(stamp=0.30))
    node._on_slam_pose(slam_message(stamp=0.30))
    node._on_ready_heartbeat()

    node.clock.seconds = 0.40
    node._on_odom(odom_message(stamp=0.40))
    node._on_tf(SimpleNamespace(transforms=[
        tf_edge("map", "camera_init", x=0.31, stamp=0.90),
        tf_edge("camera_init", "body_nav", stamp=0.40),
    ]))
    node._on_scan(scan_message(stamp=0.40))
    node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.LOST
    assert node._last_transition.error is ErrorCode.TF_CONFLICT
    assert publisher(node, "~/ready").messages[-1].data is False


def test_korean_diagnostic_messages_do_not_contain_replacement_characters(supervisor_module):
    assert all("\ufffd" not in message for message in supervisor_module._MESSAGES_KO.values())


def test_supervisor_wires_topics_refines_map_pose_and_emits_finite_json(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    module = supervisor_module
    result = SearchResult(PoseScore(Pose2D(1.0, 2.0, 0.5), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(module, "coarse_search", lambda *_args: result)
    node = module.LocalizationSupervisor()
    assert {sub.topic for sub in node.subscriptions} == {"/map", "/scan", "/Odometry", "/initialpose", "/slam_localization/pose", "/tf"}
    assert {pub.topic for pub in node.publishers} == {"/slam_localization/initialpose", "~/status", "~/ready"}

    node._on_map(map_message(occupied_world=((2.0, 2.0),)))
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
    node._on_odom(odom_message(stamp=0.1))
    node._on_tf(relevant_tf(1.0, 2.0, source_stamp=0.1))
    node._on_slam_pose(slam_message(1.0, 2.0, stamp=0.1))
    node._on_status_timer()
    node.clock.seconds = 3.1
    node._on_map(map_message(occupied_world=((2.0, 2.0),)))
    node._on_odom(odom_message(stamp=3.1))
    node._on_scan(scan_message(stamp=3.1))
    node._on_tf(relevant_tf(1.0, 2.0, source_stamp=3.1))
    node._on_slam_pose(slam_message(1.0, 2.0, stamp=3.1))
    node._on_status_timer()
    status = json.loads(publisher(node, "~/status").messages[-1].data)
    assert status["state"] == "READY"
    assert status["error"] == "NONE"
    assert math.isfinite(status["stamp"])
    assert publisher(node, "~/ready").messages[-1].data is True


def test_supervisor_uses_humble_pose_topic_and_type(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    subscription = next(
        item for item in node.subscriptions if item.topic == "/slam_localization/pose"
    )
    assert subscription.message_type is PoseWithCovarianceStamped


def test_supervisor_rejects_stale_inputs_odom_time_rollback_and_amcl(supervisor_module):
    module = supervisor_module
    node = module.LocalizationSupervisor()
    node._on_map(map_message(occupied_world=((2.0, 2.0),)))
    node._on_scan(scan_message())
    node.clock.seconds = 1.0
    node._on_initial_pose(pose_message())
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "INPUT_MISSING"

    node.clock.seconds = 1.1
    node._on_odom(odom_message(0.0, stamp=1.1))
    node.clock.seconds = 1.2
    node._on_odom(odom_message(1.0, stamp=1.05))
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
    node._on_map(map_message(occupied_world=((2.0, 2.0),)))
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_tf(relevant_tf(1.0, 2.0))
    node._on_odom(odom_message())
    node._on_scan(scan_message())
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == error


def test_low_overlap_retry_runs_coarse_search_again_on_a_new_scan(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    outcomes = iter((
        SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.1, 0.0, 0.1, 1), None, False),
        SearchResult(PoseScore(Pose2D(0.5, 0.0, 0.0), 0.9, 0.0, 0.9, 1), None, False),
    ))
    calls = []
    monkeypatch.setattr(
        supervisor_module,
        "coarse_search",
        lambda *_args, **_kwargs: calls.append(1) or next(outcomes),
    )
    node = supervisor_module.LocalizationSupervisor()
    use_immediate_search(node)
    node._on_map(map_message(occupied_world=((1.0, 0.0),)))
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_status_timer()
    assert calls == [1]
    assert publisher(node, "/slam_localization/initialpose").messages == []

    node.clock.seconds = 6.7
    node._on_ready_heartbeat()
    assert node._machine.attempts == 2
    node._on_status_timer()
    assert calls == [1]

    node.clock.seconds = 6.8
    node._on_scan(scan_message(stamp=6.8))
    node._on_status_timer()

    assert calls == [1, 1]
    assert publisher(node, "/slam_localization/initialpose").messages[-1].pose.pose.position.x == pytest.approx(0.5)


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
    monkeypatch.setitem(module._MESSAGES_KO, node._last_transition.error, '쉼표,따옴표"')
    node._publish_status()
    assert node._csv_file.flushed is True
    assert '"쉼표,따옴표"""' in node._csv_file.getvalue()
    node.destroy_node()
    assert node.destroyed is True


def test_supervisor_waits_for_post_click_scan_and_fresh_slam_tf_before_ready(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    search_calls = []
    result = SearchResult(PoseScore(Pose2D(1.0, 2.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: search_calls.append(1) or result)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message(occupied_world=((2.0, 2.0),)))
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_status_timer()
    assert search_calls == []

    node._on_scan(scan_message())
    node._on_status_timer()
    assert search_calls == [1]
    node.clock.seconds = 3.1
    node._on_scan(scan_message(stamp=3.1))
    node._on_odom(odom_message(stamp=3.1))
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["state"] != "READY"

    node._on_tf(relevant_tf(1.0, 2.0, source_stamp=3.1))
    node._on_slam_pose(slam_message(1.0, 2.0, stamp=3.1))
    node._on_status_timer()
    node.clock.seconds = 6.2
    node._on_odom(odom_message(stamp=6.2))
    node._on_scan(scan_message(stamp=6.2))
    node._on_tf(relevant_tf(1.0, 2.0, source_stamp=6.2))
    node._on_slam_pose(slam_message(1.1, 2.0, stamp=6.2))
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["state"] == "READY"


def test_supervisor_clears_invalid_data_and_uses_required_qos(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    subscriptions = {subscription.topic: subscription for subscription in node.subscriptions}
    assert subscriptions["/map"].qos.durability == "transient_local"
    assert subscriptions["/scan"].qos == "sensor_data"
    assert subscriptions["/Odometry"].qos == "sensor_data"
    assert subscriptions["/tf"].qos == "sensor_data"

    node._on_map(map_message(occupied_world=((1.0, 0.0), (2.0, 0.0))))
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
    node.clock.seconds = 10.0
    node._on_odom(odom_message(0.0, stamp=10.0))
    node.clock.seconds = 10.1
    node._on_odom(odom_message(0.1, stamp=9.9))
    assert node._odom_reset is True
    node._on_initial_pose(pose_message())
    assert node._odom_reset is False


def test_supervisor_keeps_slam_pose_as_one_shot_handshake(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    result = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: result)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message(occupied_world=((1.0, 0.0), (2.0, 0.0))))
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_status_timer()
    node.clock.seconds = 0.1
    node._on_odom(odom_message(stamp=0.1))
    node._on_tf(relevant_tf(source_stamp=0.1))
    slam = lambda x, stamp: slam_message(x=x, stamp=stamp)
    node._on_slam_pose(slam(0.0, 0.1))
    node._on_scan(scan_message(stamp=0.1))
    node._on_status_timer()
    node.clock.seconds = 0.2
    node._on_odom(odom_message(stamp=0.2))
    node._on_tf(relevant_tf(source_stamp=0.2))
    node._on_slam_pose(slam(1.0, 0.2))
    node._on_scan(scan_message(stamp=0.2))
    node._on_status_timer()
    assert json.loads(publisher(node, "~/status").messages[-1].data)["error"] == "NONE"


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


def test_republish_resets_tf_baselines_handshake_and_quality(supervisor_module):
    from omx_navigation.scan_map_quality import Pose2D

    node = supervisor_module.LocalizationSupervisor()
    node._last_map_camera_pose = Pose2D(1.0, 1.0, 1.0)
    node._map_camera_pose = Pose2D(1.0, 1.0, 1.0)
    node._camera_base_pose = Pose2D(0.1, 0.0, 0.0)
    node._current_base_pose = Pose2D(1.1, 1.0, 1.0)
    node._map_camera_received_at = 2.0
    node._camera_base_received_at = 2.0
    node._slam_received_at = 2.0
    node._slam_pose_handshake = True
    node._tf_position_jump = 3.0
    node._tf_yaw_jump = 2.0
    node._quality_received_at = 2.0
    node._publish_refined_pose(Pose2D(0.0, 0.0, 0.0), [0.0] * 36)
    assert node._last_map_camera_pose is None
    assert node._map_camera_pose is None
    assert node._camera_base_pose is None
    assert node._current_base_pose is None
    assert node._map_camera_received_at is None
    assert node._camera_base_received_at is None
    assert node._slam_received_at is None
    assert node._slam_pose_handshake is False
    assert node._tf_position_jump == 0.0
    assert node._tf_yaw_jump == 0.0
    assert node._quality_received_at is None


def test_tf_freshness_requires_each_edge_after_epoch_and_within_age(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 1.0
    node.clock.seconds = 1.0
    node._on_tf(relevant_tf(source_stamp=1.0))
    assert node._tf_fresh(1.0) is True

    node.clock.seconds = 1.1
    node._on_tf(relevant_tf(source_stamp=1.1))
    assert node._tf_fresh(1.1) is True

    node.clock.seconds = 1.7
    node._on_tf(SimpleNamespace(transforms=[tf_edge("camera_init", "body_nav", stamp=1.7)]))
    assert node._tf_fresh(1.7) is False


def test_same_clock_post_publish_tf_and_slam_callbacks_are_accepted(supervisor_module):
    from omx_navigation.scan_map_quality import Pose2D

    node = supervisor_module.LocalizationSupervisor()
    node.clock.seconds = 1.0
    node._publish_refined_pose(Pose2D(0.0, 0.0, 0.0), [0.0] * 36)
    node._on_tf(relevant_tf(source_stamp=1.0))
    node._on_slam_pose(slam_message(stamp=1.0))

    assert node._tf_fresh(1.0) is True
    assert node._slam_pose_handshake is True


def test_near_zero_tf_quaternion_is_rejected_fail_closed(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node.clock.seconds = 1.0
    invalid = tf_edge("map", "camera_init")
    invalid.transform.rotation.w = 1e-13
    node._on_tf(SimpleNamespace(transforms=[invalid, tf_edge("camera_init", "body_nav")]))

    assert node._map_camera_pose is None
    assert node._map_camera_received_at is None
    assert node._current_base_pose is None


def test_invalid_tf_clears_prior_edge_freshness_fail_closed(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 0.0
    node.clock.seconds = 0.1
    node._on_tf(relevant_tf())
    assert node._tf_fresh(0.1) is True

    invalid = tf_edge("map", "camera_init")
    invalid.transform.rotation.w = 0.0
    node.clock.seconds = 0.2
    node._on_tf(SimpleNamespace(transforms=[invalid]))

    assert node._map_camera_pose is None
    assert node._map_camera_received_at is None
    assert node._current_base_pose is None
    assert node._tf_fresh(0.2) is False


@pytest.mark.parametrize("value", (float("nan"), float("inf"), -float("inf")))
def test_nonfinite_tf_translation_z_is_rejected_fail_closed(supervisor_module, value):
    node = supervisor_module.LocalizationSupervisor()
    node.clock.seconds = 1.0
    invalid = tf_edge("map", "camera_init")
    invalid.transform.translation.z = value
    node._on_tf(SimpleNamespace(transforms=[invalid, tf_edge("camera_init", "body_nav")]))

    assert node._map_camera_pose is None
    assert node._map_camera_received_at is None
    assert node._current_base_pose is None


def test_nonfinite_map_tf_translation_z_closes_existing_ready_gate(supervisor_module, monkeypatch):
    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    advance_to_ready(node)
    assert publisher(node, "~/ready").messages[-1].data is True

    invalid = tf_edge("map", "camera_init")
    invalid.transform.translation.z = float("nan")
    node.clock.seconds = 3.5
    node._on_odom(odom_message())
    node._on_tf(SimpleNamespace(transforms=[invalid, tf_edge("camera_init", "body_nav")]))
    node._on_scan(scan_message())
    node._on_ready_heartbeat()

    assert publisher(node, "~/ready").messages[-1].data is False


def test_invalid_camera_base_tf_clears_edge_freshness(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 0.0
    node.clock.seconds = 0.1
    node._on_tf(relevant_tf())
    assert node._tf_fresh(0.1) is True

    invalid = tf_edge("camera_init", "body_nav")
    invalid.transform.rotation.w = 0.0
    node.clock.seconds = 0.2
    node._on_tf(SimpleNamespace(transforms=[invalid]))

    assert node._camera_base_pose is None
    assert node._camera_base_received_at is None
    assert node._current_base_pose is None
    assert node._tf_fresh(0.2) is False


def test_planar_tf_rejects_quaternion_with_nonfinite_norm(supervisor_module):
    oversized = tf_edge("map", "camera_init")
    oversized.transform.rotation.x = 1e308
    oversized.transform.rotation.y = 1e308
    oversized.transform.rotation.z = 1e308
    oversized.transform.rotation.w = 1e308

    with pytest.raises(ValueError, match="quaternion"):
        supervisor_module.LocalizationSupervisor._planar_tf(oversized)


def test_source_stamp_rejects_out_of_range_nanoseconds(supervisor_module):
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=1_000_000_000)))
    assert supervisor_module.LocalizationSupervisor._source_stamp(message, 7.0) == 7.0


@pytest.mark.parametrize("source_stamp", (9.0, 9.99))
def test_pre_epoch_slam_pose_delivered_late_does_not_complete_handshake(supervisor_module, source_stamp):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 10.0
    node.clock.seconds = 10.1

    node._on_slam_pose(slam_message(stamp=source_stamp))

    assert node._slam_pose_handshake is False
    assert node._slam_received_at is None


def test_pre_epoch_scan_delivered_late_cannot_drive_current_quality(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 10.0
    node.clock.seconds = 10.1

    node._on_scan(scan_message(stamp=9.0))

    assert node._scan_received_at is None
    assert node._points is None


def test_pre_epoch_tf_delivered_late_is_not_fresh(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 10.0
    node.clock.seconds = 10.1

    node._on_tf(relevant_tf(source_stamp=9.0))

    assert node._tf_fresh(10.1) is False
    assert node._current_base_pose is None


@pytest.mark.parametrize("message_factory", (
    lambda: scan_message(stamp=0.0),
    lambda: slam_message(stamp=0.0),
))
def test_zero_source_stamp_fails_closed(supervisor_module, message_factory):
    node = supervisor_module.LocalizationSupervisor()
    node._slam_epoch = 1.0
    node.clock.seconds = 1.1
    message = message_factory()

    if hasattr(message, "ranges"):
        node._on_scan(message)
        assert node._scan_received_at is None
    else:
        node._on_slam_pose(message)
        assert node._slam_pose_handshake is False


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


def test_new_click_without_scan_discards_older_pending_search(supervisor_module):
    class Future:
        def __init__(self):
            self._done = False
            self.cancel_called = False

        def done(self):
            return self._done

        def cancel(self):
            self.cancel_called = True
            return False

        def result(self):
            from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult
            return SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)

    class Executor:
        def __init__(self):
            self.futures = []

        def submit(self, *_args):
            future = Future()
            self.futures.append(future)
            return future

    node = supervisor_module.LocalizationSupervisor()
    node._search_executor = Executor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())  # gen1
    node._on_scan(scan_message())
    assert len(node._search_executor.futures) == 1
    node._on_initial_pose(pose_message())  # gen2
    node._on_scan(scan_message())          # gen2 becomes pending behind gen1
    node._on_initial_pose(pose_message())  # gen3, no post-click scan
    node._search_executor.futures[0]._done = True
    node._on_status_timer()
    assert len(node._search_executor.futures) == 1


def test_map_change_discards_running_search_and_restarts_on_latest_grid(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    result = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.8, 0.0, 0.8, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: result)

    class DeferredFuture:
        def __init__(self, function, *args):
            self._function = function
            self._args = args
            self._done = False
            self._result = None

        def finish(self):
            self._result = self._function(*self._args)
            self._done = True

        def done(self):
            return self._done

        def cancel(self):
            return False

        def result(self):
            return self._result

    class DeferredExecutor:
        def __init__(self):
            self.futures = []

        def submit(self, function, *args):
            future = DeferredFuture(function, *args)
            self.futures.append(future)
            return future

        def shutdown(self, **_kwargs):
            pass

    node = supervisor_module.LocalizationSupervisor()
    node._search_executor.shutdown(wait=False, cancel_futures=True)
    node._search_executor = DeferredExecutor()
    node._on_map(map_message(occupied_world=((1.0, 0.0),)))
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    assert len(node._search_executor.futures) == 1

    node._on_map(map_message(occupied_world=((2.0, 0.0),)))
    node._search_executor.futures[0].finish()
    node._on_status_timer()

    assert len(node._search_executor.futures) == 2
    assert node._distance_field is None


def test_ready_heartbeat_keeps_velocity_gate_fresh_for_more_than_one_second(supervisor_module):
    from omx_navigation.cmd_vel_gate_core import VelocityCommand, VelocityGate
    from omx_navigation.localization_state import LocalizationState
    from omx_navigation.scan_map_quality import build_distance_field

    node = supervisor_module.LocalizationSupervisor()
    node._machine.state = LocalizationState.READY
    node._last_transition = node._machine._transition()
    node._on_map(map_message(occupied_world=((1.0, 0.0),)))
    node._distance_field = build_distance_field(node._grid)
    node._slam_epoch = -0.001
    node._overlap = 0.8
    node._ambiguity_margin = 0.2
    assert sorted(timer.period for timer in node.timers) == [0.1, 0.5]
    gate = VelocityGate(ready_timeout=0.30, command_timeout=0.30)

    for step in range(12):
        now = step * 0.1
        node.clock.seconds = now
        event_stamp = max(now, 0.000001)
        node._on_odom(odom_message(stamp=event_stamp))
        node._on_scan(scan_message(stamp=event_stamp))
        node._on_tf(relevant_tf(source_stamp=event_stamp))
        node._on_slam_pose(slam_message(stamp=event_stamp))
        node._on_ready_heartbeat()
        heartbeat = publisher(node, "~/ready").messages[-1]
        gate.update_ready(heartbeat.data, now)
        assert gate.filter(VelocityCommand(0.1, 0.0, 0.0), now).reason == "command_passed"
        assert gate.watchdog(now + 0.09).reason == "command_fresh"

    node._machine.state = LocalizationState.LOST
    node._last_transition = node._machine._transition()
    node.clock.seconds = 1.2
    node._on_ready_heartbeat()
    decision = gate.update_ready(publisher(node, "~/ready").messages[-1].data, 1.2)
    assert decision.reason == "localization_not_ready"


def test_outside_coarse_candidate_is_rejected_before_refined_publish(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    outside = SearchResult(PoseScore(Pose2D(100.0, 100.0, 0.0), 0.9, 0.0, 0.9, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: outside)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_status_timer()

    assert publisher(node, "/slam_localization/initialpose").messages == []
    status = json.loads(publisher(node, "~/status").messages[-1].data)
    assert status["state"] == "LOST"
    assert status["error"] == "POSE_OUTSIDE_MAP"
    assert publisher(node, "~/ready").messages[-1].data is False


def test_outside_slam_pose_fails_closed(supervisor_module, monkeypatch):
    from omx_navigation.scan_map_quality import Pose2D, PoseScore, SearchResult

    inside = SearchResult(PoseScore(Pose2D(0.0, 0.0, 0.0), 0.9, 0.0, 0.9, 1), None, False)
    monkeypatch.setattr(supervisor_module, "coarse_search", lambda *_args: inside)
    node = supervisor_module.LocalizationSupervisor()
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._on_initial_pose(pose_message())
    node._on_scan(scan_message())
    node._on_status_timer()
    node.clock.seconds = 0.1
    node._on_slam_pose(slam_message(100.0, 100.0))
    node._on_status_timer()

    status = json.loads(publisher(node, "~/status").messages[-1].data)
    assert status["state"] == "LOST"
    assert status["error"] == "POSE_OUTSIDE_MAP"
    assert publisher(node, "~/ready").messages[-1].data is False


def test_namespaced_amcl_is_a_conflict_but_similar_name_is_not(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node.node_names = ["/fallback/amcl"]
    assert node._has_amcl() is True
    node.node_names = ["/fallback/amcl_helper"]
    assert node._has_amcl() is False


def test_ready_heartbeat_evaluates_namespaced_amcl_before_publishing(supervisor_module):
    from omx_navigation.localization_state import LocalizationState

    node = supervisor_module.LocalizationSupervisor()
    node._machine.state = LocalizationState.READY
    node._last_transition = node._machine._transition()
    node.node_names = ["/fallback/amcl"]
    node.clock.seconds = 0.1

    node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.LOST
    assert publisher(node, "~/ready").messages[-1].data is False


def test_ready_heartbeat_evaluates_stale_inputs_before_publishing(supervisor_module):
    from omx_navigation.localization_state import LocalizationState

    node = supervisor_module.LocalizationSupervisor()
    node.clock.seconds = 1.0
    node._on_map(map_message())
    node._on_scan(scan_message())
    node._slam_epoch = 0.9
    node._on_tf(relevant_tf())
    node._on_slam_pose(slam_message())
    node._overlap = 0.8
    node._ambiguity_margin = 0.2
    node._machine.state = LocalizationState.READY
    node._last_transition = node._machine._transition()
    node.clock.seconds = 1.51

    node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.DEGRADED
    assert publisher(node, "~/ready").messages[-1].data is False


def test_retry_republishes_refined_pose_once_across_both_timers(supervisor_module):
    from omx_navigation.scan_map_quality import Pose2D

    node = supervisor_module.LocalizationSupervisor()
    refined = Pose2D(0.0, 0.0, 0.0)
    node._initial_pose = (refined, tuple(0.0 for _ in range(36)))
    node._refined_pose = refined
    node._machine.receive_initial_pose(0.0)
    node.clock.seconds = 6.7

    node._on_ready_heartbeat()
    assert node._machine.attempts == 2
    assert len(publisher(node, "/slam_localization/initialpose").messages) == 1

    node._on_status_timer()

    assert len(publisher(node, "/slam_localization/initialpose").messages) == 1


def test_ready_degrades_when_current_scan_no_longer_matches_slam_pose(supervisor_module, monkeypatch):
    from omx_navigation.localization_state import ErrorCode, LocalizationState

    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    advance_to_ready(node)
    assert node._machine.state is LocalizationState.READY

    node.clock.seconds = 3.50
    node._on_odom(odom_message(stamp=3.50))
    node._on_tf(relevant_tf(source_stamp=3.50))
    node._on_scan(scan_message(ranges=(4.0,), stamp=3.50))
    node._on_slam_pose(slam_message(stamp=3.50))
    node._on_ready_heartbeat()

    assert node._overlap == 0.0
    assert node._machine.state is LocalizationState.DEGRADED
    assert node._last_transition.error is ErrorCode.LOW_OVERLAP
    assert publisher(node, "~/ready").messages[-1].data is False


def test_ready_degrades_when_odometry_stops_while_other_inputs_remain_fresh(supervisor_module, monkeypatch):
    from omx_navigation.localization_state import ErrorCode, LocalizationState

    node = initialize_high_quality_search(supervisor_module, monkeypatch)
    advance_to_ready(node)
    assert node._machine.state is LocalizationState.READY

    node.clock.seconds = 3.91
    node._on_tf(relevant_tf(source_stamp=3.91))
    node._on_scan(scan_message(stamp=3.91))
    node._on_slam_pose(slam_message(stamp=3.91))
    node._on_ready_heartbeat()

    assert node._machine.state is LocalizationState.DEGRADED
    assert node._last_transition.error is ErrorCode.INPUT_MISSING
    assert publisher(node, "~/ready").messages[-1].data is False
    node._publish_status()
    status = json.loads(publisher(node, "~/status").messages[-1].data)
    assert status["message_ko"] == "지도 또는 센서 입력이 오래되었거나 없습니다."


def test_nonfinite_odometry_clears_freshness(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    node.clock.seconds = 1.0
    node._on_odom(odom_message(stamp=1.0))
    assert node._odom_received_at == 1.0

    invalid = odom_message(x=float("nan"), stamp=1.0)
    node._on_odom(invalid)
    assert node._odom_received_at is None
