from pathlib import Path
import re

import pytest
import yaml

import go1_mapping.manifest as manifest
from go1_mapping.manifest import create_session, write_manifest_atomic


def test_create_session_builds_complete_tree(tmp_path):
    paths = create_session(tmp_path, "20260724_090000")

    assert paths.root == tmp_path / "20260724_090000"
    assert paths.bag.is_dir()
    assert paths.pcd.is_dir()
    assert paths.slam.is_dir()
    assert paths.pcd2d.is_dir()
    assert paths.validation.is_dir()


def test_create_session_generates_timestamp_id(tmp_path):
    paths = create_session(tmp_path, "")

    assert re.fullmatch(r"\d{8}_\d{6}", paths.root.name)


def test_create_session_refuses_existing_or_unsafe_id(tmp_path):
    create_session(tmp_path, "20260724_090000")

    for value in ("20260724_090000", "../escape", "contains space"):
        try:
            create_session(tmp_path, value)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted unsafe session id: {value}")


def test_manifest_write_is_parseable_and_has_no_partial(tmp_path):
    target = tmp_path / "manifest.yaml"

    write_manifest_atomic(target, {"session_id": "20260724_090000"})

    assert yaml.safe_load(target.read_text())["session_id"] == "20260724_090000"
    assert not (tmp_path / "manifest.yaml.partial").exists()


def test_create_session_rolls_back_root_after_child_creation_failure(
    tmp_path, monkeypatch
):
    session_id = "20260724_090000"
    root = tmp_path / session_id
    original_mkdir = Path.mkdir

    def fail_bag_mkdir(path, *args, **kwargs):
        if path == root / "bag":
            raise OSError("simulated child directory failure")
        return original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", fail_bag_mkdir)
    with pytest.raises(OSError, match="simulated child directory failure"):
        create_session(tmp_path, session_id)

    assert not root.exists()

    monkeypatch.setattr(Path, "mkdir", original_mkdir)
    paths = create_session(tmp_path, session_id)
    assert paths.root.is_dir()


def test_manifest_write_failure_removes_unique_temp_file(tmp_path, monkeypatch):
    target = tmp_path / "manifest.yaml"

    def fail_replace(source, destination):
        raise OSError("simulated replace failure")

    monkeypatch.setattr(manifest.os, "replace", fail_replace)
    with pytest.raises(OSError, match="simulated replace failure"):
        write_manifest_atomic(target, {"session_id": "20260724_090000"})

    assert not list(tmp_path.glob(".manifest.yaml.*.tmp"))
    assert not (tmp_path / "manifest.yaml.partial").exists()