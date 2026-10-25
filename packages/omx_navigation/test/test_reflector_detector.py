import math

import pytest

from omx_navigation.reflector_detector import (
    ReflectorPolicy,
    detect_dock,
)


def strip(cx, cy, n=25, bright=240.0, z=0.10, span=0.04, axis=(0.0, 1.0)):
    """A short run of points standing in for one tape strip."""
    out = []
    for i in range(n):
        t = (i / (n - 1) - 0.5) * span
        out.append((cx + axis[0] * t, cy + axis[1] * t, z, bright))
    return out


def dock_cloud(distance=0.82, separation=0.30, **kwargs):
    """Two strips abreast at `distance`, facing the sensor."""
    return strip(distance, +separation / 2, **kwargs) + strip(distance, -separation / 2, **kwargs)


def clutter(n=400, bright=60.0):
    """Ordinary scene returns: plenty of points, none near tape intensity."""
    out = []
    for i in range(n):
        angle = -math.pi + 2 * math.pi * i / n
        r = 1.0 + 0.3 * math.sin(i)
        out.append((r * math.cos(angle), r * math.sin(angle), 0.1, bright))
    return out


def test_finds_the_dock_among_ordinary_clutter():
    found = detect_dock(dock_cloud() + clutter())
    assert found is not None
    assert found.x == pytest.approx(0.82, abs=0.01)
    assert found.y == pytest.approx(0.0, abs=0.01)
    assert found.separation == pytest.approx(0.30, abs=0.01)
    assert found.points >= 2 * ReflectorPolicy().min_points_per_strip


def test_yaw_points_back_toward_the_sensor():
    """The reported heading is the one to hold on approach."""
    found = detect_dock(dock_cloud())
    assert found is not None
    assert abs(_gap(found.yaw, math.pi)) < math.radians(5)


def test_dock_off_to_one_side_is_located_correctly():
    cloud = [(x + 0.0, y + 0.45, z, r) for x, y, z, r in dock_cloud(distance=0.70)]
    found = detect_dock(cloud)
    assert found is not None
    assert found.x == pytest.approx(0.70, abs=0.01)
    assert found.y == pytest.approx(0.45, abs=0.01)


def test_ordinary_scene_alone_detects_nothing():
    """0.018% of a real scene reached 180; none of it should look like a dock."""
    assert detect_dock(clutter(n=2000, bright=60.0)) is None
    assert detect_dock(clutter(n=2000, bright=150.0)) is None


def test_single_strip_is_rejected():
    """Half a dock means half of it is occluded; guessing would be worse."""
    assert detect_dock(strip(0.82, 0.0)) is None


def test_wrong_separation_is_rejected():
    """A shiny thing that is not the dock fails the known-gap test."""
    assert detect_dock(dock_cloud(separation=0.60)) is None
    assert detect_dock(dock_cloud(separation=0.10)) is None


def test_three_bright_clusters_are_rejected():
    cloud = dock_cloud() + strip(0.82, 0.9)
    assert detect_dock(cloud) is None


def test_strips_outside_the_height_band_are_ignored():
    assert detect_dock(dock_cloud(z=0.60)) is None
    assert detect_dock(dock_cloud(z=-0.30)) is None


def test_strips_outside_the_range_window_are_ignored():
    assert detect_dock(dock_cloud(distance=0.10)) is None
    assert detect_dock(dock_cloud(distance=3.00)) is None


def test_dim_strips_are_ignored():
    assert detect_dock(dock_cloud(bright=170.0)) is None
    assert detect_dock(dock_cloud(bright=185.0)) is not None


def test_too_few_points_per_strip_is_rejected():
    assert detect_dock(dock_cloud(n=5)) is None


def test_policy_rejects_impossible_values():
    with pytest.raises(ValueError):
        ReflectorPolicy(band_lo=0.3, band_hi=0.1)
    with pytest.raises(ValueError):
        ReflectorPolicy(range_min=2.0, range_max=1.0)
    with pytest.raises(ValueError):
        ReflectorPolicy(strip_separation=0.0)
    with pytest.raises(ValueError):
        ReflectorPolicy(min_points_per_strip=1)


def _gap(first, second):
    return (first - second + math.pi) % (2.0 * math.pi) - math.pi


def test_a_stray_point_between_strip_points_does_not_split_the_strip():
    """Clusters join by distance, not by order of bearing."""
    cloud = dock_cloud()
    # A lone bright return further out, at a bearing inside one strip.
    cloud.append((1.60, 0.15 * 1.60 / 0.82, 0.10, 240.0))
    found = detect_dock(cloud)
    assert found is not None
    assert found.separation == pytest.approx(0.30, abs=0.01)


def test_bearing_window_keeps_bright_things_outside_it_out_of_the_pair():
    policy = ReflectorPolicy(bearing_min=0.0, bearing_max=math.radians(45.0))
    behind = strip(-1.0, 0.0)
    assert detect_dock(dock_cloud() + behind) is None
    assert detect_dock(dock_cloud() + behind, policy) is not None


def test_bearing_window_must_be_ordered():
    with pytest.raises(ValueError):
        ReflectorPolicy(bearing_min=1.0, bearing_max=0.5)
    with pytest.raises(ValueError):
        ReflectorPolicy(bearing_max=4.0)
