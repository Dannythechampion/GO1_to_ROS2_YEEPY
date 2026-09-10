import json
import importlib
import sys
from pathlib import Path
from types import ModuleType

import pytest

try:
    from omx_navigation.field_session import (
        SystemLogRecorder,
        build_session_metadata,
        resolve_system_capture_specs,
        write_session_metadata,
    )
except (ImportError, ModuleNotFoundError):
    SystemLogRecorder = None
    build_session_metadata = None
    resolve_system_capture_specs = None
    write_session_metadata = None


def test_session_metadata_captures_reproducibility_and_operating_mode(monkeypatch):
    assert build_session_metadata is not None, "field-session metadata support is missing"
    monkeypatch.setenv("ROS_DISTRO", "humble")
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")

    metadata = build_session_metadata(
        operating_mode="ARMED",
        session_id="posegraph_20260910T010203Z_42",
        utc_now="2026-09-10T01:02:03Z",
        hostname="go1-jetson",
        kernel_release="5.15-test",
        machine="aarch64",
    )

    assert metadata == {
        "schema_version": 1,
        "session_id": "posegraph_20260910T010203Z_42",
        "started_at_utc": "2026-09-10T01:02:03Z",
        "operating_mode": "ARMED",
        "host": {
            "hostname": "go1-jetson",
            "kernel_release": "5.15-test",
            "machine": "aarch64",
        },
        "ros": {"distro": "humble", "domain_id": "100"},
    }


def test_session_metadata_is_written_atomically_as_json(tmp_path: Path):
    assert write_session_metadata is not None, "field-session metadata support is missing"
    target = write_session_metadata(tmp_path, {"session_id": "trial-7"})

    assert target == tmp_path / "session_metadata.json"
    assert json.loads(target.read_text(encoding="utf-8")) == {"session_id": "trial-7"}
    assert not (tmp_path / "session_metadata.json.tmp").exists()


def test_system_capture_specs_record_tegrastats_and_kernel_without_a_shell(tmp_path: Path):
    assert resolve_system_capture_specs is not None, "system log capture support is missing"
    available = {
        "tegrastats": "/usr/bin/tegrastats",
        "journalctl": "/usr/bin/journalctl",
    }

    specs = resolve_system_capture_specs(tmp_path, available.get)

    assert specs == (
        (
            "tegrastats",
            ["/usr/bin/tegrastats", "--interval", "1000"],
            tmp_path / "tegrastats.log",
        ),
        (
            "kernel",
            ["/usr/bin/journalctl", "--kernel", "--follow", "--output=short-precise"],
            tmp_path / "kernel.log",
        ),
    )


def test_system_capture_specs_skip_tools_that_are_not_installed(tmp_path: Path):
    assert resolve_system_capture_specs is not None, "system log capture support is missing"

    assert resolve_system_capture_specs(tmp_path, lambda _name: None) == ()


def test_session_metadata_includes_launch_inputs_topics_and_source_revision(monkeypatch):
    assert build_session_metadata is not None, "field-session metadata support is missing"
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)

    metadata = build_session_metadata(
        operating_mode="DRY-RUN",
        session_id="trial",
        utc_now="2026-09-10T00:00:00Z",
        hostname="jetson",
        kernel_release="kernel",
        machine="aarch64",
        launch_inputs={"map": "/maps/hanyang.yaml", "search_radius": "1.0"},
        recorded_topics=["/scan", "/go1/control_state"],
        git_commit="abc123",
    )

    assert metadata["launch_inputs"] == {
        "map": "/maps/hanyang.yaml",
        "search_radius": "1.0",
    }
    assert metadata["recorded_topics"] == ["/scan", "/go1/control_state"]
    assert metadata["software"] == {"git_commit": "abc123"}


def test_field_session_recorder_stops_loggers_on_keyboard_shutdown(monkeypatch):
    events = []
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: events.append("init")
    rclpy.spin = lambda _node: (_ for _ in ()).throw(KeyboardInterrupt())
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: events.append("shutdown")
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    sys.modules.pop("omx_navigation.field_session_recorder", None)
    recorder_module = importlib.import_module("omx_navigation.field_session_recorder")

    class RecordingNode:
        def destroy_node(self):
            events.append("stop")

    monkeypatch.setattr(recorder_module, "FieldSessionRecorder", RecordingNode)

    recorder_module.main()

    assert events == ["init", "stop"]


def test_system_log_recorder_rejects_a_logger_that_exits_immediately(tmp_path: Path):
    assert SystemLogRecorder is not None, "system log recorder support is missing"
    specs = (
        (
            "tegrastats",
            [sys.executable, "-c", "raise SystemExit(3)"],
            tmp_path / "tegrastats.log",
        ),
    )
    recorder = SystemLogRecorder(
        tmp_path, specs=specs, required_labels=("tegrastats",)
    )

    with pytest.raises(RuntimeError, match="tegrastats.*exited"):
        recorder.start()

    recorder.stop()
    recorder.stop()
