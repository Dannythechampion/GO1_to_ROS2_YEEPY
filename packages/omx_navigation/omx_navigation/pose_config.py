"""Runtime pose files shared by commissioning and navigation nodes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import os
from pathlib import Path
import tempfile
from typing import Optional, Union

import yaml


class PoseConfigurationError(ValueError):
    """A configured pose file is absent or unsafe to use."""


class PoseFileState(Enum):
    UNCONFIGURED = "UNCONFIGURED"
    CONFIGURED = "CONFIGURED"


def normalize_angle(angle: float) -> float:
    if not math.isfinite(angle):
        raise PoseConfigurationError("pose values must be finite")
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class PlanarPose:
    frame_id: str
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if self.frame_id != "map":
            raise PoseConfigurationError("frame_id must be map")
        values = (float(self.x), float(self.y), float(self.yaw))
        if not all(math.isfinite(value) for value in values):
            raise PoseConfigurationError("pose values must be finite")
        object.__setattr__(self, "x", values[0])
        object.__setattr__(self, "y", values[1])
        object.__setattr__(self, "yaw", normalize_angle(values[2]))


@dataclass(frozen=True)
class PoseLoadResult:
    state: PoseFileState
    pose: Optional[PlanarPose]
    path: str


def load_planar_pose(
    path: Union[str, os.PathLike[str]], expected_kind: str
) -> PoseLoadResult:
    path_text = os.fspath(path) if path is not None else ""
    if not path_text.strip():
        return PoseLoadResult(PoseFileState.UNCONFIGURED, None, "")

    pose_path = Path(path_text)
    if not pose_path.is_file():
        raise PoseConfigurationError(f"configured pose file does not exist: {pose_path}")
    try:
        document = yaml.safe_load(pose_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise PoseConfigurationError(f"invalid pose YAML: {exc}") from exc
    if not isinstance(document, dict):
        raise PoseConfigurationError("pose YAML must contain a mapping")
    if document.get("schema_version") != 1:
        raise PoseConfigurationError("unsupported pose schema_version")
    if document.get("kind") != expected_kind:
        raise PoseConfigurationError(
            f"pose kind must be {expected_kind}, got {document.get('kind')!r}"
        )
    missing = [name for name in ("frame_id", "x", "y", "yaw") if name not in document]
    if missing:
        raise PoseConfigurationError(f"pose fields missing: {', '.join(missing)}")
    try:
        pose = PlanarPose(
            frame_id=str(document["frame_id"]),
            x=float(document["x"]),
            y=float(document["y"]),
            yaw=float(document["yaw"]),
        )
    except (TypeError, ValueError) as exc:
        if isinstance(exc, PoseConfigurationError):
            raise
        raise PoseConfigurationError(f"pose values are invalid: {exc}") from exc
    return PoseLoadResult(PoseFileState.CONFIGURED, pose, str(pose_path))


def write_planar_pose_atomic(
    path: Union[str, os.PathLike[str]],
    pose: PlanarPose,
    *,
    kind: str,
    overwrite: bool = False,
) -> None:
    if kind not in {"start", "destination"}:
        raise PoseConfigurationError("kind must be start or destination")
    output = Path(path)
    if not os.fspath(path).strip():
        raise PoseConfigurationError("an explicit output path is required")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() and not overwrite:
        raise FileExistsError(f"refusing to overwrite pose file: {output}")

    document = {
        "schema_version": 1,
        "kind": kind,
        "frame_id": pose.frame_id,
        "x": pose.x,
        "y": pose.y,
        "yaw": pose.yaw,
    }
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        if output.exists() and not overwrite:
            raise FileExistsError(f"refusing to overwrite pose file: {output}")
        os.replace(temporary_name, output)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
