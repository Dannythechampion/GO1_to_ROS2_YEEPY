import hashlib
import math
import subprocess
from pathlib import Path

import yaml
import pytest

from go1_mapping.manifest import write_manifest_atomic
from go1_mapping.map_finalizer import finalize_session
from go1_mapping.validation_report import build_report, sha256_file


def _pose(x=0.0, y=0.0, yaw=0.0):
    return {
        "position": {"x": x, "y": y},
        "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(yaw / 2.0),
            "w": math.cos(yaw / 2.0),
        },
    }


def _write_yaml(path: Path, payload: dict):
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")


def _valid_session(tmp_path: Path) -> Path:
    session = tmp_path / "20260724_090000"
    for relative in ("pcd", "slam_toolbox", "pcd2d", "validation"):
        (session / relative).mkdir(parents=True)

    (session / "pcd" / "chunk_000001.pcd").write_bytes(b"chunk-one")
    (session / "pcd" / "merged.pcd").write_bytes(b"merged")
    (session / "slam_toolbox" / "hanyang_9f.pgm").write_bytes(
        b"P5\n1 1\n255\n\x00"
    )
    _write_yaml(
        session / "slam_toolbox" / "hanyang_9f.yaml",
        {
            "image": "hanyang_9f.pgm",
            "resolution": 0.05,
            "origin": [0.0, 0.0, 0.0],
        },
    )
    (session / "slam_toolbox" / "hanyang_9f.posegraph").write_bytes(
        b"posegraph"
    )
    (session / "pcd2d" / "geometry_reference.pgm").write_text(
        "P2\n1 1\n255\n0\n", encoding="ascii"
    )
    _write_yaml(
        session / "pcd2d" / "geometry_reference.yaml",
        {
            "image": "geometry_reference.pgm",
            "resolution": 0.05,
            "go1_mapping_role": "geometry_reference_only",
        },
    )
    stable = _pose()
    latest = _pose(x=0.25, yaw=math.radians(5.0))
    _write_yaml(
        session / "validation" / "health.yaml",
        {
            "rates_hz": {"lidar": 8.0, "imu": 100.0, "odom": 8.0},
            "max_gaps_sec": {"lidar": 1.0, "imu": 1.0, "odom": 1.0},
            "first_stable_odometry_pose": stable,
            "latest_odometry_pose": latest,
            "return_error": {
                "xy_m": 0.25,
                "yaw_rad": math.radians(5.0),
            },
            "reasons": [],
            "write_errors": [],
        },
    )
    return session


def test_build_report_accepts_complete_session_and_hashes_all_artifacts(tmp_path):
    session = _valid_session(tmp_path)

    report = build_report(session)

    assert report["complete"] is True
    assert report["needs_return_check"] is False
    assert report["slam_map"]["ok"] is True
    assert report["pcd_chunks"]["count"] == 1
    assert report["pcd_chunks"]["duplicate_sha256"] == []
    required = {
        "pcd/chunk_000001.pcd",
        "pcd/merged.pcd",
        "slam_toolbox/hanyang_9f.pgm",
        "slam_toolbox/hanyang_9f.yaml",
        "slam_toolbox/hanyang_9f.posegraph",
        "pcd2d/geometry_reference.pgm",
        "pcd2d/geometry_reference.yaml",
        "validation/health.yaml",
    }
    assert set(report["artifacts"]) == required
    for artifact in report["artifacts"].values():
        assert artifact["size_bytes"] > 0
        assert len(artifact["sha256"]) == 64
        assert artifact["sha256"] == artifact["sha256"].lower()


def test_build_report_rejects_wrong_slam_resolution(tmp_path):
    session = _valid_session(tmp_path)
    slam_yaml = session / "slam_toolbox" / "hanyang_9f.yaml"
    payload = yaml.safe_load(slam_yaml.read_text(encoding="utf-8"))
    payload["resolution"] = 0.10
    _write_yaml(slam_yaml, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["slam_resolution"] is False
    assert any("resolution" in error for error in report["errors"])


def test_sha256_file_matches_known_abc_digest(tmp_path):
    target = tmp_path / "abc.bin"
    target.write_bytes(b"abc")

    assert sha256_file(target) == (
        "ba7816bf8f01cfea414140de5dae2223"
        "b00361a396177a9cb410ff61f20015ad"
    )
    assert sha256_file(target) == hashlib.sha256(b"abc").hexdigest()


def test_build_report_rejects_yaml_image_path_traversal(tmp_path):
    session = _valid_session(tmp_path)
    outside = tmp_path / "outside.pgm"
    outside.write_bytes(b"P5\n1 1\n255\n\x00")
    slam_yaml = session / "slam_toolbox" / "hanyang_9f.yaml"
    payload = yaml.safe_load(slam_yaml.read_text(encoding="utf-8"))
    payload["image"] = "../../outside.pgm"
    _write_yaml(slam_yaml, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["slam_map"] is False
    assert any("escapes" in error for error in report["errors"])


def test_build_report_rejects_slam_image_symlink_escape(tmp_path):
    session = _valid_session(tmp_path)
    outside = tmp_path / "outside.pgm"
    outside.write_bytes(b"P5\n1 1\n255\n\x00")
    image = session / "slam_toolbox" / "hanyang_9f.pgm"
    image.unlink()
    try:
        image.symlink_to(outside)
    except OSError as error:
        import pytest

        pytest.skip(f"symlink creation unavailable: {error}")

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["required_artifacts"] is False
    assert any("escapes" in error for error in report["errors"])


@pytest.mark.parametrize(
    "payload",
    [
        b"P5\n1 1\n255\n",
        b"P5\n2 1\n255\n\x00",
        b"P2\n0 1\n255\n0\n",
        b"P2\n1 1\n255\n",
        b"P6\n1 1\n255\n\x00",
        b"P5\n1 1\n10\n\x0b",
        b"P5\n1 1\n256\n\x01\x01",
    ],
)
def test_build_report_rejects_malformed_or_truncated_slam_pgm(
    tmp_path, payload
):
    session = _valid_session(tmp_path)
    (session / "slam_toolbox" / "hanyang_9f.pgm").write_bytes(payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["slam_map"] is False
    assert any("PGM" in error for error in report["errors"])


def test_build_report_rejects_duplicate_chunk_hashes_without_deleting_them(tmp_path):
    session = _valid_session(tmp_path)
    first = session / "pcd" / "chunk_000001.pcd"
    duplicate = session / "pcd" / "chunk_000002.pcd"
    duplicate.write_bytes(first.read_bytes())

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["unique_chunks"] is False
    assert report["pcd_chunks"]["duplicate_sha256"] == [sha256_file(first)]
    assert first.exists()
    assert duplicate.exists()


@pytest.mark.parametrize(
    "missing",
    ["health", "malformed_health", "first_stable_odometry_pose", "latest_odometry_pose"],
)
def test_build_report_requires_health_and_both_return_poses(tmp_path, missing):
    session = _valid_session(tmp_path)
    health_path = session / "validation" / "health.yaml"
    if missing == "health":
        health_path.unlink()
    elif missing == "malformed_health":
        _write_yaml(health_path, {"rates_hz": {}})
    else:
        payload = yaml.safe_load(health_path.read_text(encoding="utf-8"))
        payload[missing] = None
        _write_yaml(health_path, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["needs_return_check"] is True
    assert report["checks"]["return"] is False


@pytest.mark.parametrize(
    ("section", "topic", "value"),
    [
        ("rates_hz", "lidar", 7.999),
        ("rates_hz", "imu", 99.999),
        ("rates_hz", "odom", 7.999),
        ("max_gaps_sec", "lidar", 1.001),
        ("max_gaps_sec", "imu", 1.001),
        ("max_gaps_sec", "odom", 1.001),
        ("max_gaps_sec", "lidar", -0.001),
    ],
)
def test_build_report_enforces_health_rate_and_gap_thresholds(
    tmp_path, section, topic, value
):
    session = _valid_session(tmp_path)
    health_path = session / "validation" / "health.yaml"
    payload = yaml.safe_load(health_path.read_text(encoding="utf-8"))
    payload[section][topic] = value
    _write_yaml(health_path, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["health"] is False


@pytest.mark.parametrize(
    "latest",
    [_pose(x=0.5001), _pose(yaw=math.radians(10.01))],
)
def test_build_report_enforces_return_distance_and_yaw_thresholds(
    tmp_path, latest
):
    session = _valid_session(tmp_path)
    health_path = session / "validation" / "health.yaml"
    payload = yaml.safe_load(health_path.read_text(encoding="utf-8"))
    payload["latest_odometry_pose"] = latest
    _write_yaml(health_path, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["return"] is False


def test_build_report_wraps_return_yaw_across_pi_boundary(tmp_path):
    session = _valid_session(tmp_path)
    health_path = session / "validation" / "health.yaml"
    payload = yaml.safe_load(health_path.read_text(encoding="utf-8"))
    payload["first_stable_odometry_pose"] = _pose(yaw=math.radians(179.0))
    payload["latest_odometry_pose"] = _pose(yaw=math.radians(-179.0))
    _write_yaml(health_path, payload)

    report = build_report(session)

    assert report["complete"] is True
    assert report["return_check"]["yaw_deg"] == pytest.approx(2.0)


def test_build_report_rejects_guard_failure_and_preserves_file(tmp_path):
    session = _valid_session(tmp_path)
    guard_failure = session / "validation" / "guard_failure.yaml"
    _write_yaml(guard_failure, {"reasons": ["lidar gap"]})

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["guard"] is False
    assert guard_failure.exists()


def test_build_report_requires_exact_geometry_reference_role(tmp_path):
    session = _valid_session(tmp_path)
    reference_yaml = session / "pcd2d" / "geometry_reference.yaml"
    payload = yaml.safe_load(reference_yaml.read_text(encoding="utf-8"))
    payload["go1_mapping_role"] = "navigation_map"
    _write_yaml(reference_yaml, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["geometry_reference"] is False

def _mapping_session(tmp_path: Path) -> Path:
    session = tmp_path / "allowed" / "20260724_090000"
    for relative in ("pcd", "slam_toolbox", "pcd2d", "validation"):
        (session / relative).mkdir(parents=True)
    (session / "pcd" / "chunk_000001.pcd").write_bytes(b"chunk-one")
    stable = _pose()
    latest = _pose(x=0.25, yaw=math.radians(5.0))
    _write_yaml(
        session / "validation" / "health.yaml",
        {
            "rates_hz": {"lidar": 8.0, "imu": 100.0, "odom": 8.0},
            "max_gaps_sec": {"lidar": 1.0, "imu": 1.0, "odom": 1.0},
            "first_stable_odometry_pose": stable,
            "latest_odometry_pose": latest,
            "return_error": {"xy_m": 0.25, "yaw_rad": math.radians(5.0)},
            "reasons": [],
            "write_errors": [],
        },
    )
    write_manifest_atomic(
        session / "validation" / "session_manifest.yaml",
        {
            "session_id": session.name,
            "status": "running",
            "ros_domain_id": 100,
            "custom_field": {"must": "survive"},
        },
    )
    return session


def _successful_finalizer_dependencies(session: Path, events: list[str]):
    def service_caller(service_name, target, timeout_sec):
        assert timeout_sec == 10.0
        if service_name == "/pcd_chunk_writer/flush":
            assert target is None
            events.append("flush")
        elif service_name == "/slam_toolbox/save_map":
            events.append("save_map")
            Path(str(target) + ".pgm").write_bytes(b"P5\n1 1\n255\n\x00")
            _write_yaml(
                Path(str(target) + ".yaml"),
                {"image": "hanyang_9f.pgm", "resolution": 0.05},
            )
        elif service_name == "/slam_toolbox/serialize_map":
            events.append("serialize_map")
            Path(str(target) + ".posegraph").write_bytes(b"posegraph")
        else:
            raise AssertionError(f"unexpected service: {service_name}")
        return True

    def command_runner(argv, **kwargs):
        events.append("pcd_to_grid")
        assert kwargs == {
            "shell": False,
            "capture_output": True,
            "text": True,
            "timeout": 600.0,
            "check": False,
        }
        assert argv == [
            "installed-pcd-to-grid",
            "--input-dir",
            str(session / "pcd"),
            "--output-pcd",
            str(session / "pcd" / "merged.pcd"),
            "--output-map",
            str(session / "pcd2d" / "geometry_reference"),
            "--sensor-height-m",
            "0.75",
        ]
        (session / "pcd" / "merged.pcd").write_bytes(b"merged")
        (session / "pcd2d" / "geometry_reference.pgm").write_text(
            "P2\n1 1\n255\n0\n", encoding="ascii"
        )
        _write_yaml(
            session / "pcd2d" / "geometry_reference.yaml",
            {
                "image": "geometry_reference.pgm",
                "resolution": 0.05,
                "go1_mapping_role": "geometry_reference_only",
            },
        )
        return subprocess.CompletedProcess(argv, 0, "projection ok", "")

    def atomic_writer(target, payload):
        if target.name == "report.yaml":
            events.append("write_report")
        elif payload.get("status") == "complete":
            events.append("write_complete")
        elif payload.get("status") == "failed":
            events.append("write_failed")
        write_manifest_atomic(target, payload)

    return service_caller, command_runner, atomic_writer


def test_finalizer_runs_strict_order_and_completes_only_after_valid_report(tmp_path):
    session = _mapping_session(tmp_path)
    events = []
    service_caller, command_runner, atomic_writer = (
        _successful_finalizer_dependencies(session, events)
    )

    exit_code = finalize_session(
        session_dir=session,
        allowed_root=session.parent,
        sensor_height_m=0.75,
        service_caller=service_caller,
        command_runner=command_runner,
        pcd_to_grid_executable="installed-pcd-to-grid",
        atomic_writer=atomic_writer,
    )

    assert exit_code == 0
    assert events == [
        "flush",
        "save_map",
        "serialize_map",
        "pcd_to_grid",
        "write_report",
        "write_complete",
    ]
    report = yaml.safe_load(
        (session / "validation" / "report.yaml").read_text(encoding="utf-8")
    )
    manifest = yaml.safe_load(
        (session / "validation" / "session_manifest.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert report["complete"] is True
    assert manifest["status"] == "complete"
    assert manifest["custom_field"] == {"must": "survive"}
    assert "failed_step" not in manifest
    assert "error" not in manifest


@pytest.mark.parametrize(
    ("failure", "expected_events", "failed_step"),
    [
        ("flush", ["flush", "write_failed"], "writer_flush"),
        (
            "save_map",
            ["flush", "save_map", "write_failed"],
            "slam_save_map",
        ),
        (
            "serialize_map",
            ["flush", "save_map", "serialize_map", "write_failed"],
            "slam_serialize_map",
        ),
        (
            "pcd_to_grid",
            [
                "flush",
                "save_map",
                "serialize_map",
                "pcd_to_grid",
                "write_failed",
            ],
            "pcd_to_grid",
        ),
        (
            "validation_report",
            [
                "flush",
                "save_map",
                "serialize_map",
                "pcd_to_grid",
                "write_report",
                "write_failed",
            ],
            "validation_report",
        ),
    ],
)
def test_finalizer_short_circuits_and_preserves_manifest_fields(
    tmp_path, failure, expected_events, failed_step
):
    session = _mapping_session(tmp_path)
    events = []
    service_caller, command_runner, atomic_writer = (
        _successful_finalizer_dependencies(session, events)
    )

    good_service = service_caller
    good_runner = command_runner

    def failing_service(service_name, target, timeout_sec):
        result = good_service(service_name, target, timeout_sec)
        stage = {
            "/pcd_chunk_writer/flush": "flush",
            "/slam_toolbox/save_map": "save_map",
            "/slam_toolbox/serialize_map": "serialize_map",
        }[service_name]
        if failure == stage:
            raise RuntimeError(f"simulated {stage} failure")
        return result

    def failing_runner(argv, **kwargs):
        result = good_runner(argv, **kwargs)
        if failure == "pcd_to_grid":
            return subprocess.CompletedProcess(argv, 7, "partial", "projection failed")
        if failure == "validation_report":
            (session / "pcd2d" / "geometry_reference.yaml").write_text(
                yaml.safe_dump(
                    {
                        "image": "geometry_reference.pgm",
                        "resolution": 0.05,
                        "go1_mapping_role": "wrong",
                    }
                ),
                encoding="utf-8",
            )
        return result

    exit_code = finalize_session(
        session_dir=session,
        allowed_root=session.parent,
        sensor_height_m=0.75,
        service_caller=failing_service,
        command_runner=failing_runner,
        pcd_to_grid_executable="installed-pcd-to-grid",
        atomic_writer=atomic_writer,
    )

    assert exit_code != 0
    assert events == expected_events
    manifest = yaml.safe_load(
        (session / "validation" / "session_manifest.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["status"] == "failed"
    assert manifest["failed_step"] == failed_step
    assert "simulated" in manifest["error"] or "projection failed" in manifest["error"] or "validation" in manifest["error"]
    assert manifest["session_id"] == session.name
    assert manifest["ros_domain_id"] == 100
    assert manifest["custom_field"] == {"must": "survive"}
    assert "write_complete" not in events


def test_finalizer_rejects_non_running_manifest_before_external_actions(tmp_path):
    session = _mapping_session(tmp_path)
    manifest_path = session / "validation" / "session_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["status"] = "complete"
    write_manifest_atomic(manifest_path, manifest)
    events = []
    service_caller, command_runner, atomic_writer = (
        _successful_finalizer_dependencies(session, events)
    )

    exit_code = finalize_session(
        session_dir=session,
        allowed_root=session.parent,
        sensor_height_m=0.75,
        service_caller=service_caller,
        command_runner=command_runner,
        pcd_to_grid_executable="installed-pcd-to-grid",
        atomic_writer=atomic_writer,
    )

    assert exit_code != 0
    assert events == ["write_failed"]
    failed = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    assert failed["status"] == "failed"
    assert failed["failed_step"] == "preflight"


def test_atomic_writer_preserves_previous_valid_file_when_replace_fails(
    tmp_path, monkeypatch
):
    import go1_mapping.manifest as manifest_module

    target = tmp_path / "report.yaml"
    target.write_text("sentinel: true\n", encoding="utf-8")

    def fail_replace(source, destination):
        raise OSError("simulated atomic replace failure")

    monkeypatch.setattr(manifest_module.os, "replace", fail_replace)
    with pytest.raises(OSError, match="replace failure"):
        write_manifest_atomic(target, {"complete": True})

    assert yaml.safe_load(target.read_text(encoding="utf-8")) == {
        "sentinel": True
    }
    assert not list(tmp_path.glob(".report.yaml.*.tmp"))

def test_finalizer_refuses_preexisting_final_artifact_without_overwriting(tmp_path):
    session = _mapping_session(tmp_path)
    existing = session / "slam_toolbox" / "hanyang_9f.pgm"
    existing.write_bytes(b"irreplaceable-existing-map")
    events = []
    service_caller, command_runner, atomic_writer = (
        _successful_finalizer_dependencies(session, events)
    )

    exit_code = finalize_session(
        session_dir=session,
        allowed_root=session.parent,
        sensor_height_m=0.75,
        service_caller=service_caller,
        command_runner=command_runner,
        pcd_to_grid_executable="installed-pcd-to-grid",
        atomic_writer=atomic_writer,
    )

    assert exit_code != 0
    assert events == ["write_failed"]
    assert existing.read_bytes() == b"irreplaceable-existing-map"
    manifest = yaml.safe_load(
        (session / "validation" / "session_manifest.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["failed_step"] == "preflight"


def test_finalizer_rejects_session_outside_allowed_root_before_actions(tmp_path):
    session = _mapping_session(tmp_path)
    different_root = tmp_path / "different-allowed-root"
    different_root.mkdir()
    events = []
    service_caller, command_runner, atomic_writer = (
        _successful_finalizer_dependencies(session, events)
    )

    exit_code = finalize_session(
        session_dir=session,
        allowed_root=different_root,
        sensor_height_m=0.75,
        service_caller=service_caller,
        command_runner=command_runner,
        pcd_to_grid_executable="installed-pcd-to-grid",
        atomic_writer=atomic_writer,
    )

    assert exit_code != 0
    assert events == []
    manifest = yaml.safe_load(
        (session / "validation" / "session_manifest.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["status"] == "running"

def test_build_report_rejects_slam_image_symlink_outside_yaml_directory(tmp_path):
    session = _valid_session(tmp_path)
    image = session / "slam_toolbox" / "hanyang_9f.pgm"
    target = session / "pcd2d" / "geometry_reference.pgm"
    image.unlink()
    try:
        image.symlink_to(target)
    except OSError as error:
        pytest.skip(f"symlink creation unavailable: {error}")

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["slam_map"] is False
    assert any("YAML directory" in error for error in report["errors"])


@pytest.mark.parametrize(
    ("relative", "needs_return_check"),
    [
        ("slam_toolbox/hanyang_9f.yaml", False),
        ("validation/health.yaml", True),
    ],
)
def test_build_report_contains_invalid_utf8_yaml_as_failed_check(
    tmp_path, relative, needs_return_check
):
    session = _valid_session(tmp_path)
    (session / relative).write_bytes(b"\xff\xfe\x00")

    report = build_report(session)

    assert report["complete"] is False
    assert report["needs_return_check"] is needs_return_check
    assert report["errors"]


def test_build_report_treats_broken_guard_failure_symlink_as_failure(tmp_path):
    session = _valid_session(tmp_path)
    guard_failure = session / "validation" / "guard_failure.yaml"
    try:
        guard_failure.symlink_to(tmp_path / "missing-guard-target.yaml")
    except OSError as error:
        pytest.skip(f"symlink creation unavailable: {error}")

    report = build_report(session)

    assert report["complete"] is False
    assert report["checks"]["guard"] is False


@pytest.mark.parametrize("malformation", ["orientation_list", "huge_rate"])
def test_build_report_contains_malformed_health_runtime_errors(
    tmp_path, malformation
):
    session = _valid_session(tmp_path)
    health_path = session / "validation" / "health.yaml"
    payload = yaml.safe_load(health_path.read_text(encoding="utf-8"))
    if malformation == "orientation_list":
        payload["latest_odometry_pose"]["orientation"] = []
    else:
        payload["rates_hz"]["lidar"] = 10**400
    _write_yaml(health_path, payload)

    report = build_report(session)

    assert report["complete"] is False
    assert report["needs_return_check"] is True
    assert report["checks"]["health"] is False or report["checks"]["return"] is False
    assert report["errors"]
