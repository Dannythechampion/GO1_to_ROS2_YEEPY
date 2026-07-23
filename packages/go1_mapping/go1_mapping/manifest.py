from dataclasses import dataclass
from datetime import datetime, timezone
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


def build_running_manifest(
    session_id: str,
    ros_domain_id: int,
    started_at_utc: datetime | None = None,
) -> dict:
    started = started_at_utc or datetime.now(timezone.utc)
    if started.tzinfo is None or started.utcoffset() is None:
        raise ValueError("started_at_utc must be timezone-aware")
    started = started.astimezone(timezone.utc)
    return {
        "session_id": session_id,
        "status": "running",
        "ros_domain_id": int(ros_domain_id),
        "started_at_utc": started.isoformat().replace("+00:00", "Z"),
        "frames": {
            "odom": "camera_init",
            "base": "body",
            "map": "map_slam",
        },
        "topics": {
            "inputs": ["/livox/lidar", "/livox/imu"],
            "outputs": [
                "/Odometry",
                "/cloud_registered",
                "/cloud_registered_body",
                "/scan",
                "/map_slam",
            ],
        },
    }


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
    except Exception as creation_error:
        try:
            shutil.rmtree(root)
        except Exception as cleanup_error:
            raise RuntimeError(
                f"session rollback failed for {root}; partial session may remain: "
                f"{cleanup_error}"
            ) from creation_error
        raise
    return paths


def write_manifest_atomic(target: Path, data: dict) -> None:
    target = Path(target)
    temporary: Path | None = None
    operation_error: Exception | None = None
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
    except Exception as error:
        operation_error = error

    if temporary is not None:
        try:
            temporary.unlink(missing_ok=True)
        except FileNotFoundError:
            pass
        except Exception as cleanup_error:
            if operation_error is not None:
                raise RuntimeError(
                    f"manifest temporary cleanup failed for {target}; "
                    f"partial manifest state may remain: {cleanup_error}"
                ) from operation_error
            raise RuntimeError(
                f"manifest temporary cleanup failed for {target}; "
                f"partial manifest state may remain: {cleanup_error}"
            ) from cleanup_error

    if operation_error is not None:
        raise operation_error