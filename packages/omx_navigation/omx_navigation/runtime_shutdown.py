"""Shared helpers for clean ROS 2 process shutdown."""

from __future__ import annotations

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # Allows the pure-Python test suite to run without ROS installed.
    class ExternalShutdownException(Exception):
        """Fallback matching rclpy's normal external-shutdown signal."""


NORMAL_SHUTDOWN_EXCEPTIONS = (KeyboardInterrupt, ExternalShutdownException)


def shutdown_context(rclpy_module) -> None:
    """Shut down only a live context; older test doubles may not expose ``ok``."""

    is_live = getattr(rclpy_module, "ok", lambda: True)
    if is_live():
        rclpy_module.shutdown()
