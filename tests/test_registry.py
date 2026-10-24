from __future__ import annotations

import math

import pytest

from rswaudit import registry
from rswaudit.evidence import fmt


def test_normalize_strips_separators_and_case():
    assert registry.normalize("Wheel-Radius") == "wheelradius"
    assert registry.normalize("wheel_radius") == "wheelradius"
    assert registry.normalize("WHEEL RADIUS") == "wheelradius"


@pytest.mark.parametrize("raw,expected", [
    ("wheel_radius", "wheel_radius"),
    ("WHEEL_RADIUS", "wheel_radius"),
    ("wheel_diameter", "wheel_radius"),
    ("ticks_per_rev", "ticks_per_rev"),
    ("encoder_cpr", "ticks_per_rev"),
    ("wheel_seperation", "wheel_separation"),
    ("max_vel_theta", "max_angular_vel"),
    ("controller_frequency", "control_rate"),
    ("kp", "pid_kp"),
])
def test_aliases_resolve(raw, expected):
    hit = registry.resolve(raw)
    assert hit is not None
    assert hit[0] == expected


def test_unknown_name_is_not_guessed():
    assert registry.resolve("banana_count") is None
    assert registry.resolve("") is None


def test_wheel_diameter_converts_to_radius():
    canonical, factor, tier = registry.resolve("wheel_diameter")
    assert canonical == "wheel_radius"
    assert factor == 0.5


def test_weak_alias_is_flagged():
    _canonical, _factor, tier = registry.resolve("radius")
    assert tier == 1
    _canonical, _factor, tier = registry.resolve("wheel_radius")
    assert tier == 0


def test_unit_scale_detection():
    assert registry.guess_unit_scale(500.0, 0.5) == 1000.0
    assert registry.guess_unit_scale(0.32, 320.0) == 1000.0
    assert registry.guess_unit_scale(0.0508, 0.05) is None
    assert registry.guess_unit_scale(0.0, 1.0) is None


def test_sign_and_cluster_helpers():
    assert registry.guess_sign(0.3, -0.3)
    assert not registry.guess_sign(0.3, 0.3)
    assert len(registry.cluster([0.1, 0.10001, 0.2])) == 2
    assert registry.values_agree(0.05, 0.05001, 1e-3)


def test_fmt_does_not_eat_integer_digits():
    assert fmt(20.0, 0) == "20"
    assert fmt(50.0) == "50"
    assert fmt(0.0508) == "0.0508"
    assert fmt(0.0) == "0"
    assert fmt(None) == "?"
