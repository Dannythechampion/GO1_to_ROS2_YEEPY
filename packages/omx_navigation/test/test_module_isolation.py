"""The isolation contract conftest.py provides, checked in execution order."""

import importlib
import sys
from types import ModuleType

_first_binding = {}


def test_a_stubbed_import_binds_to_its_stub():
    # A careless test: stubs rclpy without monkeypatch and imports a module
    # that binds rclpy's shutdown exception at import time.
    stub = ModuleType("rclpy")
    executors = ModuleType("rclpy.executors")

    class StubShutdown(Exception):
        pass

    executors.ExternalShutdownException = StubShutdown
    sys.modules["rclpy"] = stub
    sys.modules["rclpy.executors"] = executors
    sys.modules.pop("omx_navigation.runtime_shutdown", None)
    module = importlib.import_module("omx_navigation.runtime_shutdown")
    assert module.NORMAL_SHUTDOWN_EXCEPTIONS[1] is StubShutdown
    _first_binding["stub"] = StubShutdown


def test_b_the_next_test_does_not_inherit_that_binding():
    assert "stub" in _first_binding
    # The stub itself is gone (a real rclpy, which has a __file__, may stay).
    leftover = sys.modules.get("rclpy.executors")
    assert leftover is None or getattr(leftover, "__file__", None)
    module = importlib.import_module("omx_navigation.runtime_shutdown")
    assert module.NORMAL_SHUTDOWN_EXCEPTIONS[1] is not _first_binding["stub"]
