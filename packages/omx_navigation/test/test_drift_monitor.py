import csv
import math
from pathlib import Path

import pytest

from omx_navigation.drift_monitor import (
    DriftAction,
    DriftCheck,
    DriftMonitor,
    DriftPolicy,
)
from omx_navigation.pose_tracking import compose_pose, invert_pose
from omx_navigation.scan_map_quality import Pose2D


FIELD_CHECKS = Path(__file__).parent / "data" / "drift_checks_20260918.csv"
# In the fixture, t=0 is the first READY check of the 2026-09-18 armed run
# (session t+64 s). Nav2 drove well until the far end of the corridor, which
# the robot reached at session t+294 s = fixture t+230 s.
GOOD_TRACKING_END = 230.0


def check(stamp, gap, *, tracked=Pose2D(1.0, 2.0, 0.3), dx=-0.25, dy=0.10, dyaw=math.radians(4.0),
          overlap=0.85, camera=Pose2D(0.0, 0.0, 0.0)):
    refined = Pose2D(tracked.x + dx, tracked.y + dy, tracked.yaw + dyaw)
    return DriftCheck(stamp, gap, tracked, refined, overlap, camera)


def field_checks():
    with FIELD_CHECKS.open(encoding="utf-8") as handle:
        return [
            DriftCheck(
                float(row["t"]),
                float(row["gap"]),
                Pose2D(float(row["tracked_x"]), float(row["tracked_y"]), float(row["tracked_yaw"])),
                Pose2D(float(row["refined_x"]), float(row["refined_y"]), float(row["refined_yaw"])),
                float(row["refined_overlap"]),
                Pose2D(float(row["camera_x"]), float(row["camera_y"]), float(row["camera_yaw"])),
            )
            for row in csv.DictReader(handle)
        ]


def test_small_gaps_never_count_as_drift():
    monitor = DriftMonitor()
    for stamp in range(20):
        decision = monitor.observe(check(float(stamp), 0.079))
        assert decision.action is DriftAction.NONE
        assert decision.drifting is False


def test_consistent_drift_is_corrected_after_the_required_checks():
    monitor = DriftMonitor()
    assert monitor.observe(check(0.0, 0.2)).action is DriftAction.NONE
    assert monitor.observe(check(2.0, 0.2)).consecutive == 2
    decision = monitor.observe(check(4.0, 0.2))

    assert decision.action is DriftAction.CORRECT
    assert decision.correction.x == pytest.approx(0.75)
    assert decision.correction.y == pytest.approx(2.10)
    assert decision.correction.yaw == pytest.approx(0.3 + math.radians(4.0))
    assert monitor.corrections == 1


def test_one_quiet_check_restarts_the_evidence():
    monitor = DriftMonitor()
    monitor.observe(check(0.0, 0.2))
    monitor.observe(check(2.0, 0.2))
    monitor.observe(check(4.0, 0.01))
    decision = monitor.observe(check(6.0, 0.2))
    assert decision.consecutive == 1
    assert decision.action is DriftAction.NONE


def test_heading_disagreement_blocks_correction_and_escalates_later():
    monitor = DriftMonitor(DriftPolicy(escalate_after=12.0))
    yaws = (0.0, 5.0, -3.0, 4.0, -2.0, 5.0, -4.0, 3.0)
    decisions = [
        monitor.observe(check(2.0 * index, 0.2, dyaw=math.radians(yaw)))
        for index, yaw in enumerate(yaws)
    ]
    assert all(item.action is not DriftAction.CORRECT for item in decisions)
    assert decisions[6].action is DriftAction.ESCALATE
    assert decisions[6].reason == "drift direction is not consistent"
    assert monitor.escalated is True


def test_translation_spread_along_a_corridor_does_not_block_correction():
    # Along the corridor axis the refined position wanders by tenths of a metre
    # while heading repeats; that spread is expected and must not block.
    monitor = DriftMonitor()
    for stamp, dy in ((0.0, -0.20), (2.0, 0.20), (4.0, 0.05)):
        decision = monitor.observe(check(stamp, 0.2, dy=dy))
    assert decision.action is DriftAction.CORRECT
    # The correction takes the median, not the last sample, along that axis.
    assert decision.correction.y == pytest.approx(2.05)


def test_correction_uses_the_median_map_to_odom_transform_while_moving():
    # The robot moves between checks; the drifted map->odom transform does not.
    error = Pose2D(-0.2, 0.1, math.radians(4.0))
    true_map_camera = Pose2D(3.0, -1.0, 0.5)
    drifted_map_camera = compose_pose(invert_pose(error), true_map_camera)
    monitor = DriftMonitor()
    for stamp, camera in ((0.0, Pose2D(0.0, 0.0, 0.0)), (2.0, Pose2D(0.4, 0.1, 0.2)), (4.0, Pose2D(0.9, 0.3, 0.5))):
        tracked = compose_pose(drifted_map_camera, camera)
        refined = compose_pose(true_map_camera, camera)
        decision = monitor.observe(DriftCheck(stamp, 0.2, tracked, refined, 0.85, camera))
    expected = compose_pose(true_map_camera, Pose2D(0.9, 0.3, 0.5))
    assert decision.action is DriftAction.CORRECT
    assert decision.correction.x == pytest.approx(expected.x)
    assert decision.correction.y == pytest.approx(expected.y)
    assert decision.correction.yaw == pytest.approx(expected.yaw)


def test_poor_corrected_match_is_not_pushed():
    monitor = DriftMonitor(DriftPolicy(escalate_after=6.0))
    decisions = [monitor.observe(check(2.0 * index, 0.2, overlap=0.30)) for index in range(5)]
    assert all(item.action is not DriftAction.CORRECT for item in decisions)
    assert decisions[3].action is DriftAction.ESCALATE
    assert decisions[3].reason == "corrected pose is not a good match"


def test_corrections_are_rate_limited_and_capped_before_escalating():
    monitor = DriftMonitor(DriftPolicy(correction_interval=10.0, max_corrections=2, escalate_after=12.0))
    actions = {}
    for index in range(30):
        decision = monitor.observe(check(2.0 * index, 0.2))
        actions.setdefault(decision.action, []).append(2.0 * index)
    corrections = actions[DriftAction.CORRECT]
    assert corrections[0] == 4.0
    assert all(later - earlier >= 10.0 for earlier, later in zip(corrections, corrections[1:]))
    assert len([stamp for stamp in corrections if stamp < 60.0]) == 2
    # Escalation waits `escalate_after` from the last correction, not from the
    # first drifted check, so a correction always gets time to work.
    assert actions[DriftAction.ESCALATE][0] >= corrections[1] + 12.0


def test_disabled_correction_escalates_persistent_drift():
    monitor = DriftMonitor(DriftPolicy(auto_correct=False, escalate_after=12.0))
    decisions = [monitor.observe(check(2.0 * index, 0.2)) for index in range(8)]
    assert all(item.action is not DriftAction.CORRECT for item in decisions)
    assert decisions[6].action is DriftAction.ESCALATE
    assert decisions[6].reason == "automatic correction disabled"


def test_cleared_drift_lifts_escalation_and_reset_forgets_everything():
    monitor = DriftMonitor(DriftPolicy(auto_correct=False, escalate_after=4.0))
    for index in range(4):
        monitor.observe(check(2.0 * index, 0.2))
    assert monitor.escalated is True
    monitor.observe(check(10.0, 0.01))
    assert monitor.escalated is False

    monitor = DriftMonitor()
    for index in range(3):
        monitor.observe(check(2.0 * index, 0.2))
    assert monitor.corrections == 1
    monitor.reset()
    assert monitor.corrections == 0
    assert monitor.observe(check(10.0, 0.2)).consecutive == 1


def test_median_heading_does_not_straddle_the_pi_seam():
    monitor = DriftMonitor()
    tracked = Pose2D(0.0, 0.0, math.pi - math.radians(1.0))
    for stamp, dyaw in ((0.0, 1.5), (2.0, 2.0), (4.0, 2.5)):
        decision = monitor.observe(check(stamp, 0.2, tracked=tracked, dx=0.0, dy=0.0, dyaw=math.radians(dyaw)))
    assert decision.action is DriftAction.CORRECT
    assert math.degrees(decision.correction.yaw) == pytest.approx(-179.0)


@pytest.mark.parametrize("field", ("gap", "stamp", "refined_overlap"))
def test_non_finite_checks_are_rejected(field):
    values = {"stamp": 0.0, "gap": 0.2, "refined_overlap": 0.8}
    values[field] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        DriftMonitor().observe(DriftCheck(
            values["stamp"], values["gap"], Pose2D(0, 0, 0), Pose2D(0, 0, 0), values["refined_overlap"], Pose2D(0, 0, 0),
        ))


@pytest.mark.parametrize("override", (
    {"gap_threshold": 0.0}, {"agreement_yaw": -1.0}, {"escalate_after": float("inf")},
    {"required_checks": 0}, {"max_corrections": True}, {"min_corrected_overlap": 1.5},
))
def test_policy_rejects_invalid_values(override):
    with pytest.raises(ValueError):
        DriftPolicy(**override)


def test_field_run_stays_quiet_while_tracking_was_good_and_corrects_the_drift():
    checks = field_checks()
    monitor = DriftMonitor()
    decisions = [(item, monitor.observe(item)) for item in checks]

    good = [decision for item, decision in decisions if item.stamp < GOOD_TRACKING_END]
    assert len(good) > 100
    assert not any(decision.drifting for decision in good)
    assert max(item.gap for item in checks if item.stamp < GOOD_TRACKING_END) < 0.06

    corrections = [(item, decision) for item, decision in decisions if decision.action is DriftAction.CORRECT]
    first_item, first = corrections[0]
    assert GOOD_TRACKING_END < first_item.stamp < GOOD_TRACKING_END + 60.0
    # The drift was a heading error of a few degrees (measured +3 to +5 deg).
    heading_fix = math.degrees(first.correction.yaw - first_item.tracked.yaw)
    assert 2.0 < heading_fix < 6.0
    assert all(item.refined_overlap >= DriftPolicy().min_corrected_overlap for item, _ in corrections)


def test_field_run_without_correction_escalates_after_the_drift_starts():
    checks = field_checks()
    monitor = DriftMonitor(DriftPolicy(auto_correct=False))
    escalations = [item.stamp for item in checks if monitor.observe(item).action is DriftAction.ESCALATE]
    assert escalations
    assert escalations[0] > GOOD_TRACKING_END
