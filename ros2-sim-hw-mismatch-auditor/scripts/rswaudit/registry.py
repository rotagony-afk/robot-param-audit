"""Canonical parameter registry.

The same physical quantity is spelled differently in URDF, ROS 2 YAML, launch
files and C++/Python sources: wheel_radius / wheel_r / r_wheel / radius,
ticks_per_rev / cpr / encoder_resolution / ppr. Without a canonical name the
auditor could not tell that two literals describe one physical object, and
every mismatch would go unnoticed.

Each canonical entry declares:
  unit      -- the SI-ish unit all values are normalised to
  scoped    -- True when the value is per-link/per-joint (mass, PID gains),
               False when it is a whole-robot quantity (radius, track)
  aliases   -- (alias, factor, tier); value_canonical = value_raw * factor
               tier 0 = unambiguous alias, tier 1 = weak/ambiguous alias
"""

from __future__ import annotations

import math
import re

_STRIP = re.compile(r"[^a-z0-9]+")

# Ratios that almost always mean "one side used a different unit".
UNIT_RATIOS = (10.0, 100.0, 1000.0, 2.54, 25.4, 0.3048)


def normalize(name: str) -> str:
    """wheel_radius, WheelRadius, wheel-radius -> wheelradius"""
    return _STRIP.sub("", (name or "").lower())


SPECS: dict[str, dict] = {
    "wheel_radius": {
        "unit": "m", "category": "geometry", "scoped": False, "label": "驱动轮半径",
        "aliases": [
            ("wheel_radius", 1.0, 0), ("wheel_r", 1.0, 0), ("wheelrad", 1.0, 0),
            ("r_wheel", 1.0, 0), ("radius_wheel", 1.0, 0), ("drive_wheel_radius", 1.0, 0),
            ("wheel_diameter", 0.5, 0), ("wheel_dia", 0.5, 0), ("dia_wheel", 0.5, 0),
            ("radius", 1.0, 1),
        ],
    },
    "wheel_separation": {
        "unit": "m", "category": "geometry", "scoped": False, "label": "左右轮距",
        "aliases": [
            ("wheel_separation", 1.0, 0), ("wheel_seperation", 1.0, 0), ("wheel_sep", 1.0, 0),
            ("separation", 1.0, 1), ("track_width", 1.0, 0), ("track", 1.0, 1),
            ("wheel_base", 1.0, 0), ("wheelbase", 1.0, 0), ("wheel_distance", 1.0, 0),
            ("axis_distance", 1.0, 0), ("wheel_track", 1.0, 0), ("base_width", 1.0, 0),
            ("robot_width", 1.0, 0),
        ],
    },
    "ticks_per_rev": {
        "unit": "count/rev", "category": "encoder", "scoped": False, "label": "编码器每转计数",
        "aliases": [
            ("ticks_per_rev", 1.0, 0), ("ticks_per_revolution", 1.0, 0), ("tpr", 1.0, 0),
            ("cpr", 1.0, 0), ("encoder_cpr", 1.0, 0), ("counts_per_rev", 1.0, 0),
            ("encoder_resolution", 1.0, 0), ("encoder_ticks", 1.0, 0), ("ppr", 1.0, 0),
            ("pulses_per_rev", 1.0, 0), ("encoder_counts", 1.0, 0), ("ticks", 1.0, 1),
        ],
    },
    "gear_ratio": {
        "unit": "ratio", "category": "encoder", "scoped": False, "label": "电机减速比",
        "aliases": [
            ("gear_ratio", 1.0, 0), ("gearratio", 1.0, 0), ("gear_reduction", 1.0, 0),
            ("reduction", 1.0, 0), ("reduction_ratio", 1.0, 0), ("motor_gear_ratio", 1.0, 0),
        ],
    },
    "max_linear_vel": {
        "unit": "m/s", "category": "limit", "scoped": False, "label": "最大线速度",
        "aliases": [
            ("max_linear_vel", 1.0, 0), ("max_linear_velocity", 1.0, 0),
            ("max_vel_x", 1.0, 0), ("max_lin_vel", 1.0, 0), ("max_linear", 1.0, 0),
            ("linear_max_vel", 1.0, 0), ("max_speed", 1.0, 1),
        ],
    },
    "max_angular_vel": {
        "unit": "rad/s", "category": "limit", "scoped": False, "label": "最大角速度",
        "aliases": [
            ("max_angular_vel", 1.0, 0), ("max_angular_velocity", 1.0, 0),
            ("max_vel_theta", 1.0, 0), ("max_rot_vel", 1.0, 0), ("max_ang_vel", 1.0, 0),
            ("max_angular", 1.0, 0),
        ],
    },
    "max_wheel_vel": {
        "unit": "rad/s", "category": "limit", "scoped": False, "label": "单轮最大角速度",
        "aliases": [
            ("max_wheel_vel", 1.0, 0), ("max_wheel_velocity", 1.0, 0),
            ("max_wheel_speed", 1.0, 0), ("wheel_max_vel", 1.0, 0),
            ("max_wheel_omega", 1.0, 0), ("max_wheel_angular_vel", 1.0, 0),
        ],
    },
    "control_rate": {
        "unit": "Hz", "category": "timing", "scoped": False, "label": "控制/发布频率",
        "aliases": [
            ("control_rate", 1.0, 0), ("control_frequency", 1.0, 0),
            ("controller_frequency", 1.0, 0), ("loop_rate", 1.0, 0),
            ("publish_rate", 1.0, 0), ("odom_publish_rate", 1.0, 0),
            ("control_loop_rate", 1.0, 0), ("update_rate", 1.0, 1),
        ],
    },
    "sim_update_rate": {
        "unit": "Hz", "category": "timing", "scoped": False, "label": "仿真步进频率",
        "aliases": [
            ("sim_update_rate", 1.0, 0), ("physics_rate", 1.0, 0),
            ("real_time_update_rate", 1.0, 0), ("gazebo_update_rate", 1.0, 0),
            ("sim_rate", 1.0, 0),
        ],
    },
    "pid_kp": {
        "unit": "", "category": "control", "scoped": True, "label": "PID 比例增益",
        "aliases": [("kp", 1.0, 1), ("k_p", 1.0, 0), ("pid_kp", 1.0, 0), ("p_gain", 1.0, 0)],
    },
    "pid_ki": {
        "unit": "", "category": "control", "scoped": True, "label": "PID 积分增益",
        "aliases": [("ki", 1.0, 1), ("k_i", 1.0, 0), ("pid_ki", 1.0, 0), ("i_gain", 1.0, 0)],
    },
    "pid_kd": {
        "unit": "", "category": "control", "scoped": True, "label": "PID 微分增益",
        "aliases": [("kd", 1.0, 1), ("k_d", 1.0, 0), ("pid_kd", 1.0, 0), ("d_gain", 1.0, 0)],
    },
    "mass": {
        "unit": "kg", "category": "dynamic", "scoped": True, "label": "连杆质量",
        "aliases": [("mass", 1.0, 0), ("link_mass", 1.0, 0), ("weight", 1.0, 1)],
    },
    "friction_mu": {
        "unit": "", "category": "dynamic", "scoped": True, "label": "轮胎摩擦系数",
        "aliases": [("mu", 1.0, 1), ("mu1", 1.0, 0), ("mu2", 1.0, 0),
                    ("friction", 1.0, 0), ("friction_coefficient", 1.0, 0)],
    },
}

_ALIAS_INDEX: dict[str, tuple[str, float, int]] = {}
for _canon, _spec in SPECS.items():
    for _alias, _factor, _tier in _spec["aliases"]:
        _ALIAS_INDEX[normalize(_alias)] = (_canon, _factor, _tier)
# The canonical key itself is always a valid alias.
for _canon in SPECS:
    _ALIAS_INDEX.setdefault(normalize(_canon), (_canon, 1.0, 0))


def resolve(raw_name: str) -> tuple[str, float, int] | None:
    """Map an arbitrary parameter name to (canonical, factor, tier)."""
    key = normalize(raw_name)
    if key in _ALIAS_INDEX:
        return _ALIAS_INDEX[key]
    # Trailing qualifiers such as wheel_radius_left, kp_linear, mu1_wheel.
    for suffix in ("left", "right", "front", "rear", "fl", "fr", "rl", "rr",
                   "linear", "angular", "x", "y", "z", "motor", "wheel"):
        if key.endswith(suffix) and len(key) > len(suffix) + 2:
            base = key[: -len(suffix)]
            if base in _ALIAS_INDEX:
                return _ALIAS_INDEX[base]
    return None


def unit_of(canonical: str) -> str:
    return SPECS.get(canonical, {}).get("unit", "")


def label_of(canonical: str) -> str:
    return SPECS.get(canonical, {}).get("label", canonical)


def is_scoped(canonical: str) -> bool:
    return bool(SPECS.get(canonical, {}).get("scoped", False))


def rel_diff(a: float, b: float) -> float:
    if a == b:
        return 0.0
    scale = max(abs(a), abs(b))
    if scale == 0.0:
        return 0.0
    return abs(a - b) / scale


def values_agree(a: float, b: float, rel_tol: float = 1e-3) -> bool:
    return rel_diff(a, b) <= rel_tol


def guess_unit_scale(a: float, b: float, rel_tol: float = 0.01) -> float | None:
    """Return the unit ratio when a/b looks like a metric/imperial slip."""
    if a == 0 or b == 0:
        return None
    ratio = max(abs(a), abs(b)) / min(abs(a), abs(b))
    for candidate in UNIT_RATIOS:
        if abs(ratio / candidate - 1.0) <= rel_tol:
            return candidate
    return None


def cluster(values: list[float], rel_tol: float = 1e-3) -> list[list[float]]:
    """Group nearly-equal numbers together (order preserving)."""
    groups: list[list[float]] = []
    for value in values:
        for group in groups:
            if values_agree(group[0], value, rel_tol):
                group.append(value)
                break
        else:
            groups.append([value])
    return groups


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def guess_sign(a: float, b: float, rel_tol: float = 1e-3) -> bool:
    """True when a and b are equal in magnitude but opposite in sign."""
    return a != 0 and b != 0 and values_agree(abs(a), abs(b), rel_tol) and (a > 0) != (b > 0)
