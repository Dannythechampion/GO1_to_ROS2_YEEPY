"""Executable contract checks for the safe pose-graph diagnostics workflow."""

from __future__ import annotations

import ast
import importlib.util
import os
import re
import shlex
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import pytest


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "migration" / "verify_posegraph_navigation.sh"
LAUNCH = ROOT / "packages" / "omx_navigation" / "launch" / "go1_posegraph_navigation.launch.py"
STAGE = ROOT / "migration" / "stage_local_ros2_packages.sh"
FIELD = ROOT / "migration" / "jetson_field_deploy.sh"
ROOT_README = ROOT / "README.md"
PACKAGE_README = ROOT / "packages" / "omx_navigation" / "README.md"
DRIVER_README = ROOT / "packages" / "go1_driver" / "README.md"
MIGRATION_README = ROOT / "migration" / "README.md"
END_TO_END_README = ROOT / "docs" / "GO1_NAV2_END_TO_END.md"
BASE_TOPICS = (
    "/scan", "/Odometry", "/map", "/slam_localization/pose",
    "/localization_supervisor/status", "/localization_supervisor/ready",
    "/cmd_vel_nav", "/cmd_vel",
)
BAG_TOPICS = (
    "/scan", "/Odometry", "/tf", "/tf_static", "/initialpose", "/slam_localization/pose",
    "/localization_supervisor/status", "/localization_supervisor/ready", "/cmd_vel_nav", "/cmd_vel",
)


def _bash():
    candidate = shutil.which("bash")
    if candidate is None:
        pytest.skip("bash is unavailable")
    probe = subprocess.run([candidate, "--version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if probe.returncode != 0:
        pytest.skip("usable bash is unavailable")
    return candidate


def _shell_array(text: str, name: str) -> tuple[str, ...]:
    match = re.search(rf"(?ms)^{re.escape(name)}=\(\s*(.*?)^\)", text)
    assert match, f"{name} shell array is missing"
    return tuple(shlex.split(match.group(1)))


def _launch_module():
    spec = importlib.util.spec_from_file_location("posegraph_diagnostics_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def _fake_ros_environment(tmp_path: Path) -> dict[str, str]:
    setup = tmp_path / "opt" / "ros" / "humble" / "setup.bash"
    setup.parent.mkdir(parents=True)
    setup.write_text("# fake ROS setup\n", encoding="utf-8")
    workspace = tmp_path / "ws"
    install_setup = workspace / "install" / "setup.bash"
    install_setup.parent.mkdir(parents=True)
    install_setup.write_text("# fake workspace setup\n", encoding="utf-8")
    omx_prefix = tmp_path / "omx-prefix"
    graph_base = (
        omx_prefix / "share" / "omx_navigation" / "maps" / "hanyang_9f"
        / "20260728_204825" / "slam_toolbox" / "hanyang_9f"
    )
    graph_base.parent.mkdir(parents=True)
    for suffix in (".posegraph", ".data"):
        (Path(str(graph_base) + suffix)).write_text("fake graph artifact\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _write_executable(
        bin_dir / "timeout",
        """#!/usr/bin/env bash
set -euo pipefail
shift
if [[ "${FAKE_TIMEOUT_MODE:-}" == "echo" && "$*" == *"/localization_supervisor/status"* ]]; then
  exit 124
fi
exec "$@"
""",
    )
    _write_executable(
        bin_dir / "ros2",
        """#!/usr/bin/env bash
set -euo pipefail
case "$1 $2" in
  "pkg prefix")
    if [[ "$3" == "omx_navigation" ]]; then
      printf '%s\\n' "${FAKE_OMX_PREFIX}"
    else
      printf '/fake/%s\\n' "$3"
    fi
    ;;
  "topic list") cat <<'TOPICS'
/scan
/Odometry
/map
/slam_localization/pose
/localization_supervisor/status
/localization_supervisor/ready
/cmd_vel_nav
/cmd_vel
TOPICS
    ;;
  "topic hz") printf 'average rate: 10.0\\n' ;;
  "topic echo")
    for arg in "$@"; do
      [[ "$arg" != "-n" ]] || exit 65
    done
    [[ " $* " == *" --once "* ]] || exit 66
    if [[ "${FAKE_FAIL_SLAM_POSE_ECHO:-0}" == 1 && " $* " == *" /slam_localization/pose "* ]]; then
      printf 'delayed one-shot pose is intentionally unavailable\n' >&2
      exit 67
    fi
    for arg in "$@"; do
      case "$arg" in
        /localization_supervisor/ready) printf '\\n%s\\n---\\n\\n' "${FAKE_READY:-true}"; exit 0 ;;
        /localization_supervisor/status) printf '\\n{"state":"%s","error":"%s"}\\n---\\n\\n' "${FAKE_STATE:-READY}" "${FAKE_ERROR:-NONE}"; exit 0 ;;
      esac
    done
    printf 'message\\n'
    ;;
  "node list")
    [[ "${FAKE_NODE_LIST_FAIL:-0}" != 1 ]] || exit 23
    printf '%s\\n' "${FAKE_NODES:-/map_server}"
    ;;
  "run tf2_ros") printf 'Translation: 0.0\\n' ;;
  "lifecycle get") printf 'active [3]\\n' ;;
  "param get") printf 'Boolean value is: %s\\n' "${FAKE_ARM:-False}" ;;
  *) printf 'unexpected ros2 invocation: %s\\n' "$*" >&2; exit 64 ;;
esac
""",
    )
    environment = os.environ.copy()
    environment.update({
        "PATH": str(bin_dir) + os.pathsep + environment["PATH"],
        "ROS_SETUP_FILE": str(setup),
        "GO1_ROS2_WS": str(workspace),
        "FAKE_OMX_PREFIX": str(omx_prefix),
    })
    return environment


def _run_verifier(tmp_path: Path, mode: str, **overrides) -> subprocess.CompletedProcess[str]:
    bash = _bash()
    environment = _fake_ros_environment(tmp_path)
    environment.update({key: str(value) for key, value in overrides.items()})
    return subprocess.run([bash, str(SCRIPT), mode], cwd=ROOT, env=environment, text=True, capture_output=True)


def test_verifier_has_strict_syntax_and_exact_declared_topics():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    assert _shell_array(text, "required_topics") == BASE_TOPICS
    assert _shell_array(text, "ready_nodes") == (
        "/map_server", "/controller_server", "/smoother_server", "/planner_server",
        "/behavior_server", "/bt_navigator", "/waypoint_follower", "/velocity_smoother",
    )
    assert "strip_ros_separator()" in text
    assert "topic echo -n" not in text
    assert "--once" in text
    subprocess.run([_bash(), "-n", str(SCRIPT)], check=True)


def test_verifier_executes_preflight_and_ready_with_fake_ros():
    _bash()
    with TemporaryDirectory() as temp_dir:
        tmp_path = Path(temp_dir)
        assert _run_verifier(tmp_path / "preflight", "preflight").returncode == 0
        assert _run_verifier(tmp_path / "ready", "ready").returncode == 0


def test_ready_verifier_uses_supervisor_evidence_after_the_one_shot_pose():
    """READY/NONE and ready=true prove the prior one-shot handshake completed."""
    _bash()
    with TemporaryDirectory() as temp_dir:
        result = _run_verifier(
            Path(temp_dir), "ready", FAKE_FAIL_SLAM_POSE_ECHO="1"
        )
        assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({"FAKE_READY": "false"}, "ready 토픽"),
        ({"FAKE_STATE": "DEGRADED", "FAKE_ERROR": "LOW_OVERLAP"}, "상태 토픽"),
        ({"FAKE_NODE_LIST_FAIL": "1"}, "노드 목록"),
        ({"FAKE_NODES": "/robot/amcl"}, "AMCL"),
        ({"FAKE_TIMEOUT_MODE": "echo"}, "상태 토픽"),
        ({"FAKE_OMX_PREFIX": "/missing/omx-prefix"}, "posegraph artifact"),
    ],
)
def test_ready_verifier_fails_closed_for_bad_runtime_values(overrides, expected):
    _bash()
    with TemporaryDirectory() as temp_dir:
        result = _run_verifier(Path(temp_dir), "ready", **overrides)
        assert result.returncode != 0
        assert "FAIL:" in result.stderr
        assert expected in result.stderr


def test_launch_recording_plan_has_no_side_effect_when_disabled_and_one_session_when_enabled():
    launch = _launch_module()
    root = Path("/diagnostics-will-not-be-created")
    with patch.object(launch.Path, "mkdir") as make_directory:
        assert launch.prepare_recording_session(False, False, str(root), datetime(2026, 8, 12, tzinfo=timezone.utc), 42) is None
        session_dir, topics = launch.prepare_recording_session(
            True, False, str(root), datetime(2026, 8, 12, 1, 2, 3, tzinfo=timezone.utc), 42
        )
        assert session_dir == root / "posegraph_20260812T010203Z_42"
        make_directory.assert_called_once_with(parents=True, exist_ok=False)
        assert topics == BAG_TOPICS
        _cloud_session, cloud_topics = launch.prepare_recording_session(
            True, True, "/cloud-will-not-be-created", datetime(2026, 8, 12, tzinfo=timezone.utc), 9
        )
        assert cloud_topics == BAG_TOPICS + ("/cloud_registered_body",)


def test_launch_has_exact_diagnostics_root_and_safe_record_action():
    text = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(text)
    imported_actions = {
        alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        and node.module == "launch.actions" for alias in node.names
    }
    assert {"OpaqueFunction", "ExecuteProcess"} <= imported_actions
    assert 'DeclareLaunchArgument("diagnostics_root", default_value="/mnt/t500/localization_logs")' in text
    assert "cmd=[\"ros2\", \"bag\", \"record\"" in text


def test_stage_copies_posegraph_runtime_files_and_maps():
    text = STAGE.read_text(encoding="utf-8")
    for entry in ("package.xml", "omx_navigation", "config", "launch", "maps", "test"):
        assert entry in text
    assert "verify_posegraph_navigation.sh" in text
    assert "jetson_field_deploy.sh" in text
    assert FIELD.is_file()


def test_field_runner_has_fail_closed_jetson_contract():
    text = FIELD.read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    for value in (
        "aarch64", 'VERSION_ID="22.04"', "ROS_DISTRO", "humble",
        "robot_interface", "ldd", "livox_ros_driver2", "fast_lio",
        "pointcloud_to_laserscan", ".posegraph", ".data",
        "GO1_ARMED_AND_ESTOP_READY", "arm:=false", "arm:=true",
        "start_go1_driver:=true", "record_localization:=true",
    ):
        assert value in text
    assert "verify_nonzero_test_results" in text
    subprocess.run([_bash(), "-n", str(FIELD)], check=True)


def test_field_runner_is_executable_in_a_fresh_linux_clone():
    mode = subprocess.run(
        ["git", "ls-files", "--stage", str(FIELD.relative_to(ROOT))],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    ).stdout.split()[0]
    assert mode == "100755"


def test_field_runner_rejects_bad_armed_token_before_preflight():
    bash = _bash()
    with TemporaryDirectory() as temp_dir:
        result = subprocess.run(
            [bash, str(FIELD), "armed", "WRONG_TOKEN", str(Path(temp_dir) / "ws")],
            cwd=ROOT,
            text=True,
            capture_output=True,
        )
        assert result.returncode != 0
        assert "GO1_ARMED_AND_ESTOP_READY" in result.stderr
        assert "not implemented" not in result.stderr


def test_field_live_check_accepts_tf2_echo_timeout_after_valid_transform():
    bash = _bash()
    with TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bin_dir = root / "bin"
        bin_dir.mkdir()
        _write_executable(
            bin_dir / "timeout",
            """#!/usr/bin/env bash
shift
if [[ "$*" == *"tf2_echo"* ]]; then
  printf 'At time 1.0\nTranslation: [0.0, 0.0, 0.0]\n'
  exit 124
fi
exec "$@"
""",
        )
        _write_executable(bin_dir / "ros2", "#!/usr/bin/env bash\nexit 0\n")
        environment = os.environ.copy()
        environment["PATH"] = str(bin_dir) + os.pathsep + environment["PATH"]
        result = subprocess.run(
            [bash, "-c", 'source "$1"; verify_live_inputs', "field-test", str(FIELD)],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
        )
        assert result.returncode == 0, result.stderr
        assert "body TF inputs" in result.stdout


def test_korean_field_runbooks_cover_the_safe_operating_sequence():
    for document in (ROOT_README, PACKAGE_README):
        text = document.read_text(encoding="utf-8")
        for value in (
            "jetson_field_deploy.sh stage",
            "jetson_field_deploy.sh build",
            "jetson_field_deploy.sh preflight",
            "jetson_field_deploy.sh dry-run",
            "jetson_field_deploy.sh armed GO1_ARMED_AND_ESTOP_READY",
            "e-stop", "0.3 m", "READY", "Ctrl-C", "arm:=false",
        ):
            assert value in text


def test_old_runbooks_cannot_bypass_the_canonical_armed_runner():
    canonical = (
        "jetson_field_deploy.sh armed GO1_ARMED_AND_ESTOP_READY"
    )
    for document in (MIGRATION_README, END_TO_END_README, DRIVER_README):
        text = document.read_text(encoding="utf-8")
        assert canonical in text
        assert "ros2 launch go1_driver go1_driver.launch.py arm:=true" not in text
        assert not re.search(
            r"ros2 launch omx_navigation go1_existing_map\.launch\.py"
            r"[\s\S]{0,240}?arm:=true",
            text,
        )
    root_text = ROOT_README.read_text(encoding="utf-8")
    assert "실제 동작으로 전환할 수 있는 launch는 이 문서에 없습니다" not in root_text
    assert "현재 pose-graph launch의 `arm` 값은 반드시 `false`" not in root_text
