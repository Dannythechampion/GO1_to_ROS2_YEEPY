from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import os
import re
import shutil
import tempfile

import yaml


SESSION_ID = re.compile(r"^\d{8}_\d{6}$")


@dataclass(frozen=True)
class SessionPaths:
    root: Path
    bag: Path
    pcd: Path
    slam: Path
    pcd2d: Path
    validation: Path


def create_session(session_root: Path, session_id: str) -> SessionPaths:
    resolved_root = Path(session_root).resolve()
    value = session_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    if not SESSION_ID.fullmatch(value):
        raise ValueError(f"invalid session id: {value}")

    root = resolved_root / value
    try:
        root.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        raise ValueError(f"session already exists: {root}") from exc

    paths = SessionPaths(
        root=root,
        bag=root / "bag",
        pcd=root / "pcd",
        slam=root / "slam_toolbox",
        pcd2d=root / "pcd2d",
        validation=root / "validation",
    )
    try:
        for directory in paths.__dict__.values():
            if directory != root:
                directory.mkdir()
    except Exception:
        shutil.rmtree(root)
        raise
    return paths


def write_manifest_atomic(target: Path, data: dict) -> None:
    target = Path(target)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as file:
            temporary = Path(file.name)
            yaml.safe_dump(data, file, sort_keys=True, allow_unicode=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)