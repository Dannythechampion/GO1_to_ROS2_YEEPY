"""Keep one test module's ROS stubs from leaking into the next.

Several tests replace `rclpy` and message packages in `sys.modules` and then
import a node module. Any shared module first imported during such a test --
`omx_navigation.runtime_shutdown` binds rclpy's shutdown exception at import
time -- stays cached against those stubs for every later test. On the Jetson,
where the real ROS packages are importable, the suite passed file by file and
failed with 22 failures when run as one suite (2026-09-18).

After each test, every project module and every stub first imported during it
is dropped, and anything it replaced is put back. Real ROS modules (they have a
`__file__`) are left alone: re-importing native extensions is not safe.
"""

import sys

import pytest

_PROJECT = ("omx_navigation", "go1_driver")
_ROS = (
    "rclpy", "geometry_msgs", "std_msgs", "nav_msgs", "sensor_msgs", "tf2_msgs", "tf2_ros",
    "action_msgs", "nav2_msgs", "visualization_msgs", "builtin_interfaces",
)


def _tracked(name):
    return name.split(".", 1)[0] in _PROJECT + _ROS


def _disposable(name, module):
    top = name.split(".", 1)[0]
    return top in _PROJECT or (top in _ROS and not getattr(module, "__file__", None))


@pytest.fixture(autouse=True)
def _isolate_stubbed_modules():
    before = {name: module for name, module in sys.modules.items() if _tracked(name)}
    yield
    for name, module in list(sys.modules.items()):
        if _tracked(name) and name not in before and _disposable(name, module):
            del sys.modules[name]
    sys.modules.update(before)
