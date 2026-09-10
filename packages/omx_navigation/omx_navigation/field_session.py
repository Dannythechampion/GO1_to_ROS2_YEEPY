"""Field-session metadata helpers shared by launch and the recorder node."""

from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def resolve_system_capture_specs(session_dir: Path, which=shutil.which):
    """Return available host loggers as argv lists, never shell commands."""
    session_dir = Path(session_dir)
    specs = []
    tegrastats = which("tegrastats")
    if tegrastats:
        specs.append(
            ("tegrastats", [tegrastats, "--interval", "1000"], session_dir / "tegrastats.log")
        )
    journalctl = which("journalctl")
    if journalctl:
        specs.append(
            (
                "kernel",
                [journalctl, "--kernel", "--follow", "--output=short-precise"],
                session_dir / "kernel.log",
            )
        )
    return tuple(specs)


class SystemLogRecorder:
    """Own background host loggers and their output files."""

    def __init__(
        self,
        session_dir: Path,
        specs=None,
        required_labels=("tegrastats", "kernel"),
    ) -> None:
        self._specs = tuple(
            resolve_system_capture_specs(session_dir) if specs is None else specs
        )
        available = {label for label, _command, _path in self._specs}
        missing = sorted(set(required_labels) - available)
        if missing:
            raise RuntimeError(
                "required system loggers are unavailable: " + ", ".join(missing)
            )
        self._processes = []
        self._streams = []
        self._started_labels = []

    @property
    def labels(self) -> list[str]:
        return list(self._started_labels)

    def start(self) -> None:
        for label, command, path in self._specs:
            stream = path.open("a", encoding="utf-8", buffering=1)
            try:
                process = subprocess.Popen(
                    command,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            except OSError as error:
                stream.write(f"capture failed to start: {error}\n")
                stream.close()
                self.stop()
                raise RuntimeError(f"{label} failed to start: {error}") from error
            self._streams.append(stream)
            self._processes.append(process)
            health_deadline = time.monotonic() + 0.5
            while time.monotonic() < health_deadline:
                return_code = process.poll()
                if return_code is not None:
                    self.stop()
                    raise RuntimeError(
                        f"{label} exited during startup with status {return_code}"
                    )
                time.sleep(0.02)
            self._started_labels.append(label)

    def stop(self) -> None:
        processes, self._processes = self._processes, []
        streams, self._streams = self._streams, []
        self._started_labels = []
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        for stream in streams:
            stream.close()


def build_session_metadata(
    operating_mode: str,
    session_id: str,
    utc_now: str | None = None,
    hostname: str | None = None,
    kernel_release: str | None = None,
    machine: str | None = None,
    launch_inputs: dict[str, str] | None = None,
    recorded_topics: list[str] | None = None,
    git_commit: str | None = None,
) -> dict[str, Any]:
    metadata = {
        "schema_version": 1,
        "session_id": session_id,
        "started_at_utc": utc_now or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "operating_mode": operating_mode,
        "host": {
            "hostname": hostname or platform.node(),
            "kernel_release": kernel_release or platform.release(),
            "machine": machine or platform.machine(),
        },
        "ros": {
            "distro": os.environ.get("ROS_DISTRO", ""),
            "domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        },
    }
    if launch_inputs is not None:
        metadata["launch_inputs"] = launch_inputs
    if recorded_topics is not None:
        metadata["recorded_topics"] = recorded_topics
    if git_commit is not None:
        metadata["software"] = {"git_commit": git_commit}
    return metadata


def write_session_metadata(session_dir: Path, metadata: dict[str, Any]) -> Path:
    target = Path(session_dir) / "session_metadata.json"
    temporary = target.with_name(target.name + ".tmp")
    temporary.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(target)
    return target
