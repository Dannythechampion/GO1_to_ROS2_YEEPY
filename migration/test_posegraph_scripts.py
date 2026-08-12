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
BASE_TOPICS = (
    "/scan", "/Odometry", "/map", "/slam_toolbox/pose",
    "/localization_supervisor/status", "/localization_supervisor/ready",
    "/cmd_vel_nav", "/cmd_vel",
)
BAG_TOPICS = (
    "/scan", "/Odometry", "/tf", "/tf_static", "/initialpose", "/slam_toolbox/pose",
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
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _write_executable(bin_dir / "timeout", "#!/usr/bin/env bash\nshift\nexec \"$@\"\n")
    _write_executable(
        bin_dir / "ros2",
        """#!/usr/bin/env bash
set -euo pipefail
case "$1 $2" in
  "pkg prefix") printf '/fake/%s\\n' "$3" ;;
  "topic list") cat <<'TOPICS'
/scan
/Odometry
/map
/slam_toolbox/pose
/localization_supervisor/status
/localization_supervisor/ready
/cmd_vel_nav
/cmd_vel
TOPICS
    ;;
  "topic hz") printf 'average rate: 10.0\\n' ;;
  "topic echo")
    for arg in "$@"; do
      case "$arg" in
        /localization_supervisor/ready) printf '%s\\n' "${FAKE_READY:-true}"; exit 0 ;;
        /localization_supervisor/status) printf '{"state":"%s","error":"%s"}\\n' "${FAKE_STATE:-READY}" "${FAKE_ERROR:-NONE}"; exit 0 ;;
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
    subprocess.run([_bash(), "-n", str(SCRIPT)], check=True)


def test_verifier_executes_preflight_and_ready_with_fake_ros():
    _bash()
    with TemporaryDirectory() as temp_dir:
        tmp_path = Path(temp_dir)
        assert _run_verifier(tmp_path / "preflight", "preflight").returncode == 0
        assert _run_verifier(tmp_path / "ready", "ready").returncode == 0


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({"FAKE_READY": "false"}, "ready 토픽"),
        ({"FAKE_STATE": "DEGRADED", "FAKE_ERROR": "LOW_OVERLAP"}, "상태 토픽"),
        ({"FAKE_NODE_LIST_FAIL": "1"}, "노드 목록"),
        ({"FAKE_NODES": "/robot/amcl"}, "AMCL"),
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
    for entry in ("package.xml", "omx_navigation", "config", "launch", "maps"):
        assert entry in text
    assert "verify_posegraph_navigation.sh" in text
