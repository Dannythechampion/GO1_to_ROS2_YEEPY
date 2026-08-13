from omx_navigation.motion_gate import gate_status_line
from omx_navigation.motion_gate_core import GateResult, VelocityCommand


def test_gate_status_line_reports_permission_and_reason():
    assert gate_status_line(
        GateResult(VelocityCommand.zero(), False, "emergency stop is pressed")
    ) == "enabled=false reason=emergency stop is pressed"


def test_gate_status_line_reports_enabled_without_claiming_motion():
    assert gate_status_line(
        GateResult(VelocityCommand.zero(), True, "waiting for navigation command")
    ) == "enabled=true reason=waiting for navigation command"
