from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "mapping_session.launch.py"
TEXT = LAUNCH.read_text(encoding="utf-8") if LAUNCH.exists() else ""


def test_launch_declares_only_mapping_session_controls():
    for argument in (
        "session_root",
        "session_id",
        "ros_domain_id",
        "start_livox",
        "start_fast_lio",
    ):
        assert f'DeclareLaunchArgument("{argument}"' in TEXT

    for forbidden in ("go1_driver", "unitree_go", "nav2_", "amcl"):
        assert forbidden not in TEXT.lower()


def test_launch_stages_livox_fast_lio_and_mapping_nodes():
    assert "msg_MID360_launch.py" in TEXT
    assert "mapping.launch.py" in TEXT
    assert '"config_path": mapping_config_dir' in TEXT
    assert '"config_file": "fast_lio_mapping_safe.yaml"' in TEXT
    assert '"rviz": "false"' in TEXT
    assert "TimerAction(period=3.0" in TEXT
    assert "TimerAction(period=8.0" in TEXT
    assert '("cloud_in", "/cloud_registered_body")' in TEXT
    assert '("scan", "/scan")' in TEXT
    assert '("map", "/map_slam")' in TEXT


def test_launch_creates_atomic_running_manifest_and_configures_session_outputs():
    manifest = (PACKAGE / "go1_mapping" / "manifest.py").read_text(encoding="utf-8")
    assert "build_running_manifest" in manifest
    for value in (
        '"status": "running"',
        '"camera_init"',
        '"body"',
        '"map_slam"',
        '"started_at_utc"',
    ):
        assert value in manifest

    assert "create_session(" in TEXT
    assert "write_manifest_atomic(" in TEXT
    assert '"allowed_root": str(resolved_session_root)' in TEXT
    assert '"output_dir": str(paths.pcd)' in TEXT
    assert "mapping_session.yaml" in TEXT


def test_bag_allowlist_storage_and_critical_exit_contract_are_exact():
    expected_topics = (
        '"/livox/lidar", "/livox/imu", "/Odometry", "/tf", "/tf_static"'
    )
    assert expected_topics in TEXT
    assert '"/cloud_registered"' not in TEXT.split("bag_command = [", 1)[1].split("]", 1)[0]
    assert '"--max-bag-size", "4294967296"' in TEXT
    assert '"--compression-mode", "file"' in TEXT
    assert '"--compression-format", "zstd"' in TEXT
    assert "OnProcessExit" in TEXT
    assert "Shutdown(" in TEXT
    assert "event.returncode" in TEXT
    assert "if returncode == 0:" in TEXT
