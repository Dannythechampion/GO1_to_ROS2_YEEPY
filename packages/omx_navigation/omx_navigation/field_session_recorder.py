"""ROS node that owns metadata and host telemetry for one field session."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover - compatibility with older rclpy.
    class ExternalShutdownException(Exception):
        pass

from .field_session import (
    SystemLogRecorder,
    build_session_metadata,
    write_session_metadata,
)


class FieldSessionRecorder(Node):
    def __init__(self) -> None:
        super().__init__("field_session_recorder")
        self.declare_parameter("session_dir", "")
        self.declare_parameter("operating_mode", "DRY-RUN")
        self.declare_parameter("map", "")
        self.declare_parameter("posegraph", "")
        self.declare_parameter("nav2_params_file", "")
        self.declare_parameter("slam_params_file", "")
        self.declare_parameter("scan_params_file", "")
        self.declare_parameter("search_radius", "")
        self.declare_parameter("recorded_topics", [""])
        self.declare_parameter("git_commit", "unknown")
        session_dir = Path(str(self.get_parameter("session_dir").value))
        if not str(session_dir) or str(session_dir) == ".":
            raise RuntimeError("session_dir must be nonempty")

        self._system_logs = SystemLogRecorder(session_dir)
        self._system_logs.start()
        metadata = build_session_metadata(
            operating_mode=str(self.get_parameter("operating_mode").value),
            session_id=session_dir.name,
            launch_inputs={
                name: str(self.get_parameter(name).value)
                for name in (
                    "map",
                    "posegraph",
                    "nav2_params_file",
                    "slam_params_file",
                    "scan_params_file",
                    "search_radius",
                )
            },
            recorded_topics=[
                str(topic)
                for topic in self.get_parameter("recorded_topics").value
                if str(topic)
            ],
            git_commit=str(self.get_parameter("git_commit").value),
        )
        metadata["system_capture"] = self._system_logs.labels
        write_session_metadata(session_dir, metadata)
        self.get_logger().info(f"Recording field-session host logs in {session_dir}")

    def destroy_node(self):
        self._system_logs.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FieldSessionRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
