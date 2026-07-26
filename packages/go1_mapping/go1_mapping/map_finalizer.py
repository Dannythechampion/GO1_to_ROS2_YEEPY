#!/usr/bin/env python3
"""Finalize one live mapping session without deleting partial artifacts."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Callable

import yaml

from go1_mapping.manifest import (
    SESSION_ID,
    load_manifest,
    manifest_complete,
    manifest_failed,
    write_yaml_atomic,
)
from go1_mapping.validation_report import build_report


SERVICE_TIMEOUT_SEC = 10.0
PROJECTION_TIMEOUT_SEC = 600.0

ServiceCaller = Callable[[str, Path | None, float], bool]
CommandRunner = Callable[..., subprocess.CompletedProcess]
AtomicWriter = Callable[[Path, dict], None]


def _is_contained(root: Path, candidate: Path) -> bool:
    try:
        return os.path.commonpath((str(root), str(candidate))) == str(root)
    except ValueError:
        return False


def _canonical_session(
    session_dir: Path,
    allowed_root: Path,
    manifest: dict[str, Any] | None = None,
) -> Path:
    root = Path(allowed_root).resolve(strict=True)
    session = Path(session_dir).resolve(strict=True)
    if session == root or not _is_contained(root, session):
        raise ValueError(
            f"session directory is not a child of allowed root {root}: {session}"
        )
    if not session.is_dir():
        raise ValueError(f"session directory is not a directory: {session}")
    if not SESSION_ID.fullmatch(session.name):
        raise ValueError(f"invalid session directory name: {session.name}")
    if manifest is not None and manifest.get("session_id") != session.name:
        raise ValueError(
            "session manifest session_id must match the directory name"
        )
    for relative in ("pcd", "slam_toolbox", "pcd2d", "validation"):
        directory = (session / relative).resolve(strict=True)
        if not _is_contained(session, directory) or not directory.is_dir():
            raise ValueError(
                f"session directory component is unsafe: {relative}"
            )
    return session


def _require_nonempty_contained(session: Path, relative: str) -> Path:
    path = session / relative
    canonical = path.resolve(strict=True)
    if not _is_contained(session, canonical):
        raise ValueError(f"artifact escapes session directory: {relative}")
    if not canonical.is_file() or canonical.stat().st_size <= 0:
        raise ValueError(f"required artifact is empty or not regular: {relative}")
    return canonical


def _validate_pcd_chunks(session: Path) -> None:
    pcd_directory = (session / "pcd").resolve(strict=True)
    chunks = sorted((session / "pcd").glob("chunk_*.pcd"))
    if not chunks:
        raise ValueError("no chunk_*.pcd artifacts found after writer flush")
    for chunk in chunks:
        canonical = chunk.resolve(strict=True)
        if not _is_contained(pcd_directory, canonical):
            raise ValueError(f"PCD chunk escapes pcd directory: {chunk.name}")
        if not canonical.is_file() or canonical.stat().st_size <= 0:
            raise ValueError(
                f"PCD chunk is empty or not a regular file: {chunk.name}"
            )


def _projection_command(
    executable: str,
    session: Path,
    sensor_height_m: float,
) -> list[str]:
    return [
        executable,
        "--input-dir",
        str(session / "pcd"),
        "--output-pcd",
        str(session / "pcd" / "merged.pcd"),
        "--output-map",
        str(session / "pcd2d" / "geometry_reference"),
        "--sensor-height-m",
        format(sensor_height_m, ".17g"),
    ]


def finalize_session(
    *,
    session_dir: Path,
    allowed_root: Path,
    sensor_height_m: float,
    service_caller: ServiceCaller,
    command_runner: CommandRunner = subprocess.run,
    pcd_to_grid_executable: str,
    atomic_writer: AtomicWriter = write_yaml_atomic,
) -> int:
    """Run the finalization state machine and return a process exit code."""
    step = "preflight"
    manifest_path: Path | None = None
    original_manifest: dict[str, Any] | None = None
    session: Path | None = None

    try:
        height = float(sensor_height_m)
        if not math.isfinite(height) or height <= 0.0:
            raise ValueError("sensor height must be finite and greater than zero")
        if not pcd_to_grid_executable:
            raise ValueError("pcd_to_grid executable is required")

        session = _canonical_session(session_dir, allowed_root)
        manifest_path = session / "validation" / "session_manifest.yaml"
        manifest_file = manifest_path.resolve(strict=True)
        if not _is_contained(session, manifest_file) or not manifest_file.is_file():
            raise ValueError("session manifest is unsafe or not a regular file")
        original_manifest = load_manifest(manifest_file)
        session = _canonical_session(session, allowed_root, original_manifest)
        if original_manifest.get("status") != "running":
            raise ValueError("session manifest status must be running")
        for relative in (
            "pcd/merged.pcd",
            "slam_toolbox/hanyang_9f.pgm",
            "slam_toolbox/hanyang_9f.yaml",
            "slam_toolbox/hanyang_9f.posegraph",
            "slam_toolbox/hanyang_9f.data",
            "pcd2d/geometry_reference.pgm",
            "pcd2d/geometry_reference.yaml",
        ):
            output = session / relative
            if output.exists() or output.is_symlink():
                raise FileExistsError(
                    f"refusing to overwrite existing final artifact: {relative}"
                )

        step = "writer_flush"
        if not service_caller(
            "/pcd_chunk_writer/flush", None, SERVICE_TIMEOUT_SEC
        ):
            raise RuntimeError("writer flush returned an unsuccessful response")

        step = "pcd_chunks"
        _validate_pcd_chunks(session)

        step = "slam_save_map"
        slam_prefix = session / "slam_toolbox" / "hanyang_9f"
        if not service_caller(
            "/slam_toolbox/save_map", slam_prefix, SERVICE_TIMEOUT_SEC
        ):
            raise RuntimeError("slam_toolbox save_map returned failure")
        _require_nonempty_contained(
            session, "slam_toolbox/hanyang_9f.pgm"
        )
        _require_nonempty_contained(
            session, "slam_toolbox/hanyang_9f.yaml"
        )

        step = "slam_serialize_map"
        if not service_caller(
            "/slam_toolbox/serialize_map", slam_prefix, SERVICE_TIMEOUT_SEC
        ):
            raise RuntimeError("slam_toolbox serialize_map returned failure")
        _require_nonempty_contained(
            session, "slam_toolbox/hanyang_9f.posegraph"
        )
        _require_nonempty_contained(
            session, "slam_toolbox/hanyang_9f.data"
        )

        step = "pcd_to_grid"
        command = _projection_command(
            pcd_to_grid_executable, session, height
        )
        result = command_runner(
            command,
            shell=False,
            capture_output=True,
            text=True,
            timeout=PROJECTION_TIMEOUT_SEC,
            check=False,
        )
        stdout = result.stdout if isinstance(result.stdout, str) else ""
        stderr = result.stderr if isinstance(result.stderr, str) else ""
        if result.returncode != 0:
            detail = "; ".join(
                part for part in (stdout.strip(), stderr.strip()) if part
            )
            raise RuntimeError(
                f"pcd_to_grid exited with code {result.returncode}"
                + (f": {detail}" if detail else "")
            )
        for relative in (
            "pcd/merged.pcd",
            "pcd2d/geometry_reference.pgm",
            "pcd2d/geometry_reference.yaml",
        ):
            _require_nonempty_contained(session, relative)

        step = "validation_report"
        report = build_report(session)
        atomic_writer(session / "validation" / "report.yaml", report)
        if not report.get("complete"):
            details = report.get("errors")
            suffix = (
                ": " + "; ".join(str(item) for item in details)
                if isinstance(details, list) and details
                else ""
            )
            raise RuntimeError(f"validation report is incomplete{suffix}")

        step = "manifest_complete"
        atomic_writer(manifest_path, manifest_complete(original_manifest))
        return 0
    except Exception as error:
        if manifest_path is not None and original_manifest is not None:
            try:
                atomic_writer(
                    manifest_path,
                    manifest_failed(original_manifest, step, str(error)),
                )
            except Exception as write_error:
                print(
                    f"map_finalizer: {error}; failed to record failure: "
                    f"{write_error}",
                    file=sys.stderr,
                )
                return 2
        else:
            print(f"map_finalizer: {error}", file=sys.stderr)
        return 1


class RosServiceCaller:
    """Synchronous, deadline-bound wrapper around the three ROS services."""

    def __init__(self, node: Any, rclpy_module: Any):
        self.node = node
        self.rclpy = rclpy_module

    def __call__(
        self, service_name: str, target: Path | None, timeout_sec: float
    ) -> bool:
        from slam_toolbox.srv import SaveMap, SerializePoseGraph
        from std_srvs.srv import Trigger

        if service_name == "/pcd_chunk_writer/flush":
            service_type = Trigger
            request = Trigger.Request()
        elif service_name == "/slam_toolbox/save_map":
            service_type = SaveMap
            request = SaveMap.Request()
            request.name.data = str(target)
        elif service_name == "/slam_toolbox/serialize_map":
            service_type = SerializePoseGraph
            request = SerializePoseGraph.Request()
            request.filename = str(target)
        else:
            raise ValueError(f"unsupported finalizer service: {service_name}")

        started = time.monotonic()
        client = self.node.create_client(service_type, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise TimeoutError(f"service unavailable: {service_name}")
            remaining = timeout_sec - (time.monotonic() - started)
            if remaining <= 0.0:
                raise TimeoutError(f"service deadline exceeded: {service_name}")
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=remaining
            )
            if not future.done():
                raise TimeoutError(f"service response timed out: {service_name}")
            if future.exception() is not None:
                raise RuntimeError(
                    f"service call failed: {service_name}: "
                    f"{future.exception()}"
                )
            response = future.result()
            if response is None:
                raise RuntimeError(
                    f"service returned no response: {service_name}"
                )
            if service_name == "/pcd_chunk_writer/flush":
                if not response.success:
                    raise RuntimeError(
                        f"writer flush failed: {response.message}"
                    )
            elif int(response.result) != int(response.RESULT_SUCCESS):
                raise RuntimeError(
                    f"{service_name} failed with result {response.result}"
                )
            return True
        finally:
            self.node.destroy_client(client)


def _installed_paths() -> tuple[Path, str]:
    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )

    share = Path(get_package_share_directory("go1_mapping"))
    config_path = share / "config" / "mapping_session.yaml"
    with config_path.open("r", encoding="utf-8") as source:
        config = yaml.safe_load(source)
    if not isinstance(config, dict) or not isinstance(
        config.get("mapping_session"), dict
    ):
        raise ValueError("mapping_session.yaml has an invalid root")
    root_value = config["mapping_session"].get("session_root")
    if not isinstance(root_value, str) or not root_value:
        raise ValueError("mapping_session.session_root is required")
    prefix = Path(get_package_prefix("go1_mapping"))
    executable = prefix / "lib" / "go1_mapping" / "pcd_to_grid"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise FileNotFoundError(f"installed pcd_to_grid not found: {executable}")
    return Path(root_value), str(executable)


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-dir", required=True, type=Path)
    parser.add_argument("--sensor-height-m", required=True, type=float)
    arguments = parser.parse_args(args)

    try:
        allowed_root, executable = _installed_paths()
        import rclpy

        rclpy.init(args=[])
        node = rclpy.create_node("map_finalizer")
        try:
            return finalize_session(
                session_dir=arguments.session_dir,
                allowed_root=allowed_root,
                sensor_height_m=arguments.sensor_height_m,
                service_caller=RosServiceCaller(node, rclpy),
                pcd_to_grid_executable=executable,
            )
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception as error:
        print(f"map_finalizer: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
