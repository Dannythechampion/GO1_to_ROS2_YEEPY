from pathlib import Path
import re

import yaml

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
