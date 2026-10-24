"""Tests for the chassis-agnostic layer: mesh, model import, kinematics,
simulation, identification and the App's security surface."""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path

import pytest

from rswaudit import kinematics as kin
from rswaudit import modelspec, simulate
from rswaudit.mesh import load_mesh
from rswaudit.model_rules import run_model_rules

ROOT = Path(__file__).resolve().parents[1]
UNI = ROOT / "examples" / "universal"


def write_box_stl(path: Path, sx: float, sy: float, sz: float) -> None:
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
         (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    with path.open("wb") as fh:
        fh.write(b"box".ljust(80, b"\x00"))
        fh.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            fh.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for idx in (a, b, c):
                fh.write(struct.pack("<3f", *v[idx]))
            fh.write(struct.pack("<H", 0))


# --------------------------------------------------------------------------- #
# mesh
# --------------------------------------------------------------------------- #

def test_mesh_volume_and_inertia_match_analytic_box(tmp_path: Path):
    path = tmp_path / "box.stl"
    write_box_stl(path, 2.0, 1.0, 3.0)
    info = load_mesh(path)
    assert info.triangles == 12
    assert info.volume == pytest.approx(6.0, rel=1e-3)
    assert info.closed
    mass, com, tensor = info.inertia(1.0)
    assert mass == pytest.approx(6.0, rel=1e-3)
    assert com[0] == pytest.approx(0.0, abs=1e-6)
    assert tensor["ixx"] == pytest.approx(6.0 * (1 ** 2 + 3 ** 2) / 12, rel=1e-3)
    assert tensor["iyy"] == pytest.approx(6.0 * (2 ** 2 + 3 ** 2) / 12, rel=1e-3)
    assert tensor["izz"] == pytest.approx(6.0 * (2 ** 2 + 1 ** 2) / 12, rel=1e-3)


def test_mesh_obj_is_supported(tmp_path: Path):
    path = tmp_path / "tri.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                    "f 1 2 3\nf 1 2 4\nf 2 3 4\nf 1 3 4\n", encoding="utf-8")
    info = load_mesh(path)
    assert info.triangles == 4
    assert info.volume == pytest.approx(1.0 / 6.0, rel=2e-2)


# --------------------------------------------------------------------------- #
# model import
# --------------------------------------------------------------------------- #

def test_urdf_import_finds_wheels_and_drive():
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    assert spec.drive == "differential"
    assert len(spec.wheels) == 2
    assert spec.wheel_radius == pytest.approx(0.0508)
    assert spec.track == pytest.approx(0.32)
    # the planted defect: one wheel spins about z, which is recorded, not hidden
    assert any("竖直" in item for item in spec.unresolved)


def test_mjcf_import():
    spec = modelspec.load_model(UNI / "demo_robot.mjcf.xml")
    assert spec.drive == "differential"
    assert len(spec.wheels) == 2
    positions = sorted(round(w.y, 3) for w in spec.wheels)
    assert positions == [-0.16, 0.16]
    assert spec.wheel_radius == pytest.approx(0.0508)


def test_json_spec_import_and_drive_detection():
    spec = modelspec.load_model(UNI / "mecanum_bot.json")
    assert spec.drive == "mecanum"
    assert len(spec.wheels) == 4
    assert spec.reality["wheel_radius"] == pytest.approx(0.0752)
    assert abs(spec.wheels[0].roller_deg) == pytest.approx(45.0)


def test_sdf_import(tmp_path: Path):
    sdf = """<sdf version="1.6"><model name="cart">
      <link name="base"><inertial><mass value="3.0"/></inertial></link>
      <link name="left_wheel"><collision><geometry><cylinder>
        <radius>0.06</radius><length>0.02</length></cylinder></geometry></collision></link>
      <joint name="left_wheel_joint" type="revolute">
        <parent>base</parent><child>left_wheel</child>
        <axis><xyz>0 1 0</xyz></axis><pose>0 0.13 0.06 0 0 0</pose>
      </joint>
    </model></sdf>"""
    path = tmp_path / "cart.sdf"
    path.write_text(sdf, encoding="utf-8")
    spec = modelspec.load_model(path)
    assert len(spec.wheels) == 1
    assert spec.wheels[0].radius == pytest.approx(0.06)
    assert spec.wheels[0].y == pytest.approx(0.13)


# --------------------------------------------------------------------------- #
# kinematics
# --------------------------------------------------------------------------- #

def test_differential_cannot_strafe():
    spec = modelspec.load_model(UNI / "mecanum_no_rollers.json")
    assert not kin.capability_matrix(spec)["can_strafe"]
    result = kin.check_twist(spec, kin.Twist(0.0, 0.2, 0.0))
    assert result.lateral_residual == pytest.approx(0.2, abs=1e-6)
    assert not result.ok


def test_mecanum_can_strafe():
    spec = modelspec.load_model(UNI / "mecanum_bot.json")
    assert kin.capability_matrix(spec)["can_strafe"]
    result = kin.check_twist(spec, kin.Twist(0.0, 0.2, 0.0))
    assert result.ok
    # 45 degree rollers: |omega| = cos(45) * v / r
    expected = 0.2 * math.cos(math.radians(45)) / 0.075
    assert all(abs(abs(w) - expected) < 1e-6 for w in result.wheel_omegas)


def test_ackermann_steering_angles_match_geometry():
    spec = modelspec.load_model(UNI / "ackermann_bot.json")
    result = kin.check_twist(spec, kin.Twist(1.5, 0.0, 0.4))
    assert result.ok, result.problems
    # body origin must be projected so the rear axle carries the ICR
    assert result.twist.vy == pytest.approx(0.525 * 0.4, abs=1e-6)
    angles = sorted(abs(v) for v in result.steering.values())
    outer, inner = angles[0], angles[-1]
    # textbook Ackermann: atan(L/(R -/+ track/2)) with L=1.05, R=3.75, track=1.2
    assert inner == pytest.approx(math.degrees(math.atan(1.05 / (3.75 - 0.6))), abs=0.2)
    assert outer == pytest.approx(math.degrees(math.atan(1.05 / (3.75 + 0.6))), abs=0.2)
    assert inner > outer


def test_diff_drive_wheel_speeds_are_textbook():
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    result = kin.check_twist(spec, kin.Twist(0.2, 0.0, 1.0))
    left, right = result.wheel_omegas
    assert left == pytest.approx((0.2 - 1.0 * 0.16) / 0.0508, rel=1e-6)
    assert right == pytest.approx((0.2 + 1.0 * 0.16) / 0.0508, rel=1e-6)


# --------------------------------------------------------------------------- #
# simulation and identification
# --------------------------------------------------------------------------- #

def test_simulation_reproduces_odometry_scale_error():
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    # physical radius 0.0508, firmware uses 0.05
    spec.reality["wheel_radius"] = 0.0508
    profile = simulate.profile_step(5.0, 0.2, 0.0, dt=0.02)
    result = simulate.simulate(spec, profile, 0.02, radius_used=0.05)
    assert result.distance == pytest.approx(0.2 * 4.98, rel=0.01)
    assert result.odom_distance == pytest.approx(
        result.distance * 0.05 / 0.0508, rel=0.01)


def test_identification_finds_radius_from_straight_run():
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    profile = simulate.profile_step(5.0, 0.2, 0.0, dt=0.02)
    # measured: truth 0.995 m, odometry 0.9793 m -> radius ratio 1.016
    measured = {"distance": 0.995}
    findings, _summary = run_model_rules(
        spec, twist=kin.Twist(0.2, 0.0, 0.0), profile=profile,
        measured=measured, dt=0.02, measured_odom=0.9793, radius_used=0.05)
    ident = [f for f in findings if f.rule == "M10"]
    assert ident, [f.rule for f in findings]
    assert ident[0].metrics["radius_true"] == pytest.approx(0.0508, rel=2e-3)


def test_identification_finds_track_from_turn_run():
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    profile = simulate.profile_step(4.0, 0.0, 0.5, dt=0.02)
    findings, _summary = run_model_rules(
        spec, twist=kin.Twist(0.0, 0.0, 0.5), profile=profile,
        measured={"yaw": 2.0}, dt=0.02, measured_odom_yaw=2.1333,
        track_used=0.30)
    ident = [f for f in findings if f.rule == "M10"]
    assert ident
    assert ident[0].metrics["track_true"] == pytest.approx(0.32, rel=3e-3)


def test_mesh_rule_flags_mass_mismatch():
    spec = modelspec.load_model(UNI / "mesh_bot.json")
    mesh = load_mesh(UNI / "chassis.stl")
    findings, _extra = run_model_rules(spec, mesh=mesh, density=1200.0)
    mesh_rules = [f for f in findings if f.rule == "M07"]
    assert any("质量" in f.title for f in mesh_rules)
    mass_rule = [f for f in mesh_rules if "质量" in f.title][0]
    assert mass_rule.metrics["mesh_mass"] == pytest.approx(7.2, rel=1e-2)
    assert mass_rule.metrics["declared_mass"] == pytest.approx(2.4)


def test_capability_rule_flags_infeasible_lateral_command():
    spec = modelspec.load_model(UNI / "mecanum_no_rollers.json")
    findings, _extra = run_model_rules(spec, twist=kin.Twist(0.0, 0.2, 0.0))
    assert any(f.rule == "M02" and "横向" in f.title for f in findings)


# --------------------------------------------------------------------------- #
# app security surface
# --------------------------------------------------------------------------- #

def test_app_safe_name_and_session_filter():
    from rswaudit.app import Handler, _safe_name

    assert _safe_name("../../etc/passwd") == "passwd"
    assert _safe_name("a b.urdf") == "a_b.urdf"
    empty = _safe_name("")
    assert empty.isalnum()

    handler = Handler.__new__(Handler)
    assert handler._session({"s": ["a1b2c3"]}) == "a1b2c3"
    assert handler._session({"s": ["../evil"]}) == "default"
    assert handler._session({"s": ["x" * 40]}) == "default"
    assert handler._session({}) == "default"
