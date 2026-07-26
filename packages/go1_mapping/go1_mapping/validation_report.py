#!/usr/bin/env python3
"""Validate and inventory artifacts produced by a mapping session."""
from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
from typing import Any

import yaml


SLAM_RESOLUTION_M = 0.05
SLAM_RESOLUTION_TOLERANCE_M = 1.0e-9
MIN_RATES_HZ = {"lidar": 8.0, "imu": 100.0, "odom": 8.0}
MAX_GAP_SEC = 1.0
MAX_RETURN_DISTANCE_M = 0.5
MAX_RETURN_YAW_RAD = math.radians(10.0)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_mapping(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as source:
        payload = yaml.safe_load(source)
    if not isinstance(payload, dict):
        raise ValueError(f"{path.name} must contain a YAML mapping")
    return payload


def _contained_file(root: Path, path: Path) -> Path:
    canonical_root = root.resolve(strict=True)
    canonical_path = path.resolve(strict=True)
    try:
        contained = os.path.commonpath(
            (str(canonical_root), str(canonical_path))
        ) == str(canonical_root)
    except ValueError:
        contained = False
    if not contained:
        raise ValueError(f"artifact escapes session directory: {path}")
    if not canonical_path.is_file():
        raise ValueError(f"artifact is not a regular file: {path}")
    return canonical_path


def _yaml_image(session: Path, yaml_path: Path, payload: dict[str, Any]) -> Path:
    image = payload.get("image")
    if not isinstance(image, str) or not image:
        raise ValueError(f"{yaml_path.name} image must be a relative path")
    image_path = Path(image)
    if image_path.is_absolute():
        raise ValueError(f"{yaml_path.name} image must be a relative path")
    yaml_directory = yaml_path.parent.resolve(strict=True)
    canonical_session = session.resolve(strict=True)
    try:
        directory_is_safe = os.path.commonpath(
            (str(canonical_session), str(yaml_directory))
        ) == str(canonical_session)
    except ValueError:
        directory_is_safe = False
    if not directory_is_safe:
        raise ValueError(f"{yaml_path.name} escapes the session directory")
    candidate = (yaml_directory / image_path).resolve(strict=True)
    try:
        image_is_safe = os.path.commonpath(
            (str(yaml_directory), str(candidate))
        ) == str(yaml_directory)
    except ValueError:
        image_is_safe = False
    if not image_is_safe:
        raise ValueError(f"{yaml_path.name} image escapes its YAML directory")
    if not candidate.is_file():
        raise ValueError(f"{yaml_path.name} image is not a regular file")
    return candidate


def _next_pgm_token(data: bytes, offset: int) -> tuple[bytes, int]:
    length = len(data)
    while offset < length:
        if data[offset] in b" \t\r\n\v\f":
            offset += 1
            continue
        if data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            if newline < 0:
                raise ValueError("unterminated PGM comment")
            offset = newline + 1
            continue
        break
    if offset >= length:
        raise EOFError
    start = offset
    while offset < length and data[offset] not in b" \t\r\n\v\f#":
        offset += 1
    return data[start:offset], offset


def _parse_positive_integer(token: bytes, field: str) -> int:
    try:
        value = int(token.decode("ascii"))
    except (UnicodeDecodeError, ValueError) as error:
        raise ValueError(f"invalid PGM {field}") from error
    if value <= 0:
        raise ValueError(f"PGM {field} must be positive")
    return value


def _validate_pgm(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    try:
        magic, offset = _next_pgm_token(data, 0)
        width_token, offset = _next_pgm_token(data, offset)
        height_token, offset = _next_pgm_token(data, offset)
        maxval_token, offset = _next_pgm_token(data, offset)
    except EOFError as error:
        raise ValueError("truncated PGM header") from error
    if magic not in (b"P2", b"P5"):
        raise ValueError("PGM magic must be P2 or P5")
    width = _parse_positive_integer(width_token, "width")
    height = _parse_positive_integer(height_token, "height")
    maxval = _parse_positive_integer(maxval_token, "maxval")
    if maxval > 65535:
        raise ValueError("PGM maxval exceeds 65535")
    pixel_count = width * height
    if magic == b"P2":
        values = []
        while True:
            try:
                token, offset = _next_pgm_token(data, offset)
            except EOFError:
                break
            values.append(_parse_positive_integer(token, "pixel") if token != b"0" else 0)
        if len(values) != pixel_count or any(value > maxval for value in values):
            raise ValueError("PGM P2 payload does not match dimensions")
    else:
        if offset >= len(data) or data[offset] not in b" \t\r\n\v\f":
            raise ValueError("PGM P5 header is missing payload separator")
        if data[offset : offset + 2] == b"\r\n":
            offset += 2
        else:
            offset += 1
        bytes_per_pixel = 1 if maxval < 256 else 2
        payload = data[offset:]
        if len(payload) != pixel_count * bytes_per_pixel:
            raise ValueError("PGM P5 payload does not match dimensions")
        if bytes_per_pixel == 1:
            samples = payload
        else:
            samples = (
                int.from_bytes(payload[index : index + 2], "big")
                for index in range(0, len(payload), 2)
            )
        if any(sample > maxval for sample in samples):
            raise ValueError("PGM P5 payload contains a sample above maxval")
    return width, height


def _yaw(pose: dict[str, Any]) -> float:
    orientation = pose["orientation"]
    values = tuple(
        float(orientation.get(name, 0.0)) for name in ("x", "y", "z", "w")
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0.0:
        raise ValueError("pose quaternion must be nonzero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _return_error(first: dict[str, Any], latest: dict[str, Any]) -> tuple[float, float]:
    first_position = first["position"]
    latest_position = latest["position"]
    dx = float(latest_position["x"]) - float(first_position["x"])
    dy = float(latest_position["y"]) - float(first_position["y"])
    yaw_delta = _yaw(latest) - _yaw(first)
    wrapped_yaw = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
    values = (dx, dy, wrapped_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("return pose values must be finite")
    return math.hypot(dx, dy), wrapped_yaw


def build_report(session_dir: Path) -> dict[str, Any]:
    session = Path(session_dir).resolve(strict=True)
    errors: list[str] = []
    checks: dict[str, bool] = {}
    artifacts: dict[str, dict[str, Any]] = {}

    def record(relative: str) -> Path | None:
        path = session / relative
        try:
            canonical = _contained_file(session, path)
            size = canonical.stat().st_size
            if size <= 0:
                raise ValueError(f"artifact is empty: {relative}")
            artifacts[relative] = {
                "size_bytes": size,
                "sha256": sha256_file(canonical),
            }
            return canonical
        except (OSError, ValueError) as error:
            errors.append(str(error))
            return None

    chunks = sorted((session / "pcd").glob("chunk_*.pcd"))
    chunk_paths = []
    for chunk in chunks:
        relative = chunk.relative_to(session).as_posix()
        recorded = record(relative)
        if recorded is not None:
            chunk_paths.append(recorded)
    checks["pcd_chunks"] = bool(chunk_paths) and len(chunk_paths) == len(chunks)
    if not chunks:
        errors.append("no chunk_*.pcd artifacts found")

    required = [
        "pcd/merged.pcd",
        "slam_toolbox/hanyang_9f.pgm",
        "slam_toolbox/hanyang_9f.yaml",
        "slam_toolbox/hanyang_9f.posegraph",
        "slam_toolbox/hanyang_9f.data",
        "pcd2d/geometry_reference.pgm",
        "pcd2d/geometry_reference.yaml",
        "validation/health.yaml",
    ]
    paths = {relative: record(relative) for relative in required}
    checks["required_artifacts"] = all(paths.values())

    slam_ok = False
    slam_resolution_ok = False
    slam_details: dict[str, Any] = {"ok": False}
    slam_yaml_path = paths["slam_toolbox/hanyang_9f.yaml"]
    if slam_yaml_path is not None:
        try:
            slam_yaml = _safe_mapping(slam_yaml_path)
            slam_image = _yaml_image(session, slam_yaml_path, slam_yaml)
            if slam_image != paths["slam_toolbox/hanyang_9f.pgm"]:
                raise ValueError("slam YAML image does not reference hanyang_9f.pgm")
            width, height = _validate_pgm(slam_image)
            resolution = float(slam_yaml["resolution"])
            slam_resolution_ok = (
                math.isfinite(resolution)
                and abs(resolution - SLAM_RESOLUTION_M)
                <= SLAM_RESOLUTION_TOLERANCE_M
            )
            if not slam_resolution_ok:
                raise ValueError(
                    f"slam resolution {resolution} is not {SLAM_RESOLUTION_M}"
                )
            slam_ok = True
            slam_details.update(
                {
                    "ok": True,
                    "resolution_m": resolution,
                    "width": width,
                    "height": height,
                }
            )
        except (
            KeyError,
            OSError,
            OverflowError,
            TypeError,
            ValueError,
            yaml.YAMLError,
        ) as error:
            errors.append(f"slam map: {error}")
    checks["slam_resolution"] = slam_resolution_ok
    checks["slam_map"] = slam_ok

    reference_ok = False
    reference_yaml_path = paths["pcd2d/geometry_reference.yaml"]
    if reference_yaml_path is not None:
        try:
            reference_yaml = _safe_mapping(reference_yaml_path)
            if (
                reference_yaml.get("go1_mapping_role")
                != "geometry_reference_only"
            ):
                raise ValueError("geometry reference YAML has an invalid role")
            reference_image = _yaml_image(
                session, reference_yaml_path, reference_yaml
            )
            if reference_image != paths["pcd2d/geometry_reference.pgm"]:
                raise ValueError(
                    "geometry reference YAML image does not reference "
                    "geometry_reference.pgm"
                )
            _validate_pgm(reference_image)
            reference_ok = True
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            errors.append(f"geometry reference: {error}")
    checks["geometry_reference"] = reference_ok

    guard_failure_path = session / "validation" / "guard_failure.yaml"
    guard_ok = not (
        guard_failure_path.exists() or guard_failure_path.is_symlink()
    )
    checks["guard"] = guard_ok
    if not guard_ok:
        errors.append("validation/guard_failure.yaml exists")

    health_ok = False
    return_ok = False
    needs_return_check = False
    health_details: dict[str, Any] = {"ok": False}
    return_details: dict[str, Any] = {"ok": False}
    health_path = paths["validation/health.yaml"]
    if health_path is not None:
        try:
            health = _safe_mapping(health_path)
            rates = health["rates_hz"]
            gaps = health["max_gaps_sec"]
            health_ok = all(
                math.isfinite(float(rates[name]))
                and float(rates[name]) >= minimum
                and math.isfinite(float(gaps[name]))
                and 0.0 <= float(gaps[name]) <= MAX_GAP_SEC
                for name, minimum in MIN_RATES_HZ.items()
            )
            if health.get("reasons") or health.get("write_errors"):
                health_ok = False
            if not health_ok:
                errors.append("health rate, gap, or guard reason check failed")
            health_details = {
                "ok": health_ok,
                "rates_hz": rates,
                "max_gaps_sec": gaps,
            }
            first = health.get("first_stable_odometry_pose")
            latest = health.get("latest_odometry_pose")
            if first is None or latest is None:
                needs_return_check = True
                errors.append("stable/latest return poses are missing")
            else:
                distance, yaw_error = _return_error(first, latest)
                return_ok = (
                    distance <= MAX_RETURN_DISTANCE_M
                    and yaw_error <= MAX_RETURN_YAW_RAD
                )
                if not return_ok:
                    errors.append("return distance or yaw exceeds threshold")
                return_details = {
                    "ok": return_ok,
                    "xy_m": distance,
                    "yaw_rad": yaw_error,
                    "yaw_deg": math.degrees(yaw_error),
                }
        except (
            AttributeError,
            KeyError,
            OverflowError,
            TypeError,
            ValueError,
            yaml.YAMLError,
        ) as error:
            needs_return_check = True
            errors.append(f"health: {error}")
    else:
        needs_return_check = True
    checks["health"] = health_ok
    checks["return"] = return_ok

    chunk_digests = [
        artifact["sha256"]
        for relative, artifact in artifacts.items()
        if relative.startswith("pcd/chunk_") and relative.endswith(".pcd")
    ]
    duplicate_hashes = sorted(
        digest for digest in set(chunk_digests) if chunk_digests.count(digest) > 1
    )
    checks["unique_chunks"] = not duplicate_hashes
    if duplicate_hashes:
        errors.append("duplicate PCD chunk SHA256 detected")

    complete = all(checks.values()) and not needs_return_check
    return {
        "complete": complete,
        "needs_return_check": needs_return_check,
        "checks": checks,
        "slam_map": slam_details,
        "pcd_chunks": {
            "ok": checks["pcd_chunks"] and checks["unique_chunks"],
            "count": len(chunk_paths),
            "duplicate_sha256": duplicate_hashes,
        },
        "health": health_details,
        "return_check": return_details,
        "artifacts": artifacts,
        "errors": errors,
    }


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir", type=Path)
    arguments = parser.parse_args()
    report = build_report(arguments.session_dir)
    print(yaml.safe_dump(report, sort_keys=True), end="")
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
