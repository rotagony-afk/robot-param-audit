"""Tests for arms, rotors, lidars and the Simulink generator."""

from __future__ import annotations

import math
import json
from pathlib import Path

import pytest

from rswaudit import modelspec, simulink_gen
from rswaudit.mechanism_rules import rule_arm, rule_lidar, rule_rotor, run_mechanism_rules
from rswaudit.mechanisms import ArmJoint, ArmSpec, LidarSpec, RotorSpec

ROOT = Path(__file__).resolve().parents[1]
MECH = ROOT / "examples" / "mechanisms"


# --------------------------------------------------------------------------- #
# arm
# --------------------------------------------------------------------------- #

def planar_arm() -> ArmSpec:
    return ArmSpec("planar2", joints=[
        ArmJoint("j1", (0, 0, 1), (0, 0, 0), effort=20.0, velocity=1.0),
        ArmJoint("j2", (0, 0, 1), (0.5, 0, 0), effort=10.0, velocity=1.0),
    ], tip_offset=(0.4, 0, 0))


def test_arm_forward_kinematics():
    arm = planar_arm()
    tip = arm.forward([0.0, 0.0])
    assert tip[0][3] == pytest.approx(0.9, abs=1e-9)
    assert tip[1][3] == pytest.approx(0.0, abs=1e-9)
    tip = arm.forward([math.pi / 2, 0.0])
    assert tip[0][3] == pytest.approx(0.0, abs=1e-9)
    assert tip[1][3] == pytest.approx(0.9, abs=1e-9)


def test_arm_jacobian_matches_finite_differences():
    spec = modelspec.load_model(MECH / "arm6.json")
    arm = spec.arm
    q = [0.4, -0.5, 0.9, 0.3, -0.6, 1.1]
    jac = arm.jacobian(q)
    base = arm.forward(q)
    eps = 1e-6
    for index in range(arm.dof):
        moved = list(q)
        moved[index] += eps
        tip = arm.forward(moved)
        for row in range(3):
            numeric = (tip[row][3] - base[row][3]) / eps
            assert jac[row][index] == pytest.approx(numeric, abs=1e-4)


def test_arm_six_dof_zero_pose():
    spec = modelspec.load_model(MECH / "arm6.json")
    arm = spec.arm
    assert arm.dof == 6
    tip = arm.forward([0.0] * 6)
    assert tip[0][3] == pytest.approx(0.71, abs=1e-6)
    assert tip[2][3] == pytest.approx(0.31, abs=1e-6)


def test_arm_ik_reachable_and_not():
    spec = modelspec.load_model(MECH / "arm6.json")
    arm = spec.arm
    reachable = arm.inverse((0.45, 0.0, 0.30))
    assert reachable["residual"] < 1e-3
    far = arm.inverse((1.4, 0.0, 1.4))
    assert far["residual"] > 0.1


def test_arm_static_torque_scales_with_payload():
    spec = modelspec.load_model(MECH / "arm6.json")
    arm = spec.arm
    q = [0.0] * 6
    tau = arm.static_torque(q, (0.0, 0.0, -9.81 * 2.0))
    # shoulder carries the whole overhang: F * x, about 14 N*m
    assert tau[1] == pytest.approx(9.81 * 2.0 * 0.71, rel=0.05)
    assert tau[0] == pytest.approx(0.0, abs=1e-9)   # yaw joint sees no moment


def test_arm_payload_torque_rule_fires():
    spec = modelspec.load_model(MECH / "arm6.json")
    findings, info = rule_arm(spec.arm, (0.45, 0.0, 0.30), 2.0, "arm6.json")
    assert info["residual"] < 1e-3
    over = [f for f in findings if f.rule == "A03"]
    assert over, [(f.rule, f.title) for f in findings]
    assert "j5_pitch" in over[0].title


def test_arm_unreachable_rule_fires():
    spec = modelspec.load_model(MECH / "arm6.json")
    findings, _info = rule_arm(spec.arm, (1.4, 0.0, 1.4), 0.0, "arm6.json")
    assert any(f.rule == "A01" for f in findings)


# --------------------------------------------------------------------------- #
# rotor
# --------------------------------------------------------------------------- #

def test_rotor_thrust_torque_power():
    rotor = RotorSpec(diameter_m=0.254, count=4, kT=1.2e-5, kQ=2.0e-7, max_rpm=9000)
    omega = 9000 * 2 * math.pi / 60
    assert rotor.thrust(9000) == pytest.approx(1.2e-5 * omega ** 2, rel=1e-9)
    assert rotor.torque(9000) == pytest.approx(2.0e-7 * omega ** 2, rel=1e-9)
    assert rotor.power(9000) == pytest.approx(rotor.torque(9000) * omega, rel=1e-9)
    assert rotor.thrust(9000) == pytest.approx(10.66, abs=0.02)


def test_rotor_hover_rpm():
    rotor = RotorSpec(kT=1.2e-5, count=4, total_mass_kg=1.5)
    rpm = rotor.hover_rpm()
    assert rpm == pytest.approx(5287.0, abs=20.0)


def test_rotor_declared_thrust_mismatch():
    spec = modelspec.load_model(MECH / "quad_x.json")
    findings, info = rule_rotor(spec.rotor, "quad_x.json")
    assert info["max_thrust_per_rotor"] == pytest.approx(10.66, abs=0.02)
    titles = [f.title for f in findings if f.rule == "P01"]
    assert titles and "24%" in titles[0]


def test_rotor_cannot_hover():
    rotor = RotorSpec(kT=5e-6, count=4, max_rpm=5000, total_mass_kg=5.0)
    findings, _info = rule_rotor(rotor, "x.json")
    assert any(f.rule == "P03" for f in findings)


# --------------------------------------------------------------------------- #
# lidar
# --------------------------------------------------------------------------- #

def test_lidar_points_per_second_and_blind_radius():
    lidar = LidarSpec(fov_h_deg=360, az_res_deg=0.2, vert_lines=16, rate_hz=10,
                      mount_height_m=0.4, fov_v_deg=30, mount_pitch_deg=0)
    assert lidar.points_per_second() == pytest.approx(288000.0)
    assert lidar.ground_blind_radius() == pytest.approx(1.4928, abs=1e-3)


def test_lidar_rules_fire_on_inconsistent_spec():
    spec = modelspec.load_model(MECH / "lidar360.json")
    findings, info = rule_lidar(spec.lidar, "lidar360.json")
    rules = [f.rule for f in findings]
    assert "L01" in rules
    assert "L02" in rules
    assert info["points_per_second"] == pytest.approx(288000.0)


# --------------------------------------------------------------------------- #
# simulink generation
# --------------------------------------------------------------------------- #

def test_simulink_model_for_wheels_is_valid():
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    model = simulink_gen.build_model(spec, command=0.3, used_radius=0.05)
    assert model is not None and model.kind == "wheeled"
    assert simulink_gen.validate(model) == []
    text = simulink_gen.emit_m(model)
    assert text.startswith("function build_")
    assert text.count("add_block(") == len(model.blocks)
    assert text.count("add_line(") == len(model.lines)
    assert "save_system" in text
    mdl = simulink_gen.emit_mdl(model)
    assert mdl.startswith("Model {") and "Block {" in mdl


def test_simulink_model_for_arm_has_one_branch_per_joint():
    spec = modelspec.load_model(MECH / "arm6.json")
    model = simulink_gen.build_model(spec)
    assert model is not None and model.kind == "arm"
    assert simulink_gen.validate(model) == []
    assert len([b for b in model.blocks if b.kind == "sat"]) == 6
    assert len([b for b in model.blocks if b.kind == "scope"]) == 6


def test_simulink_model_for_rotor():
    spec = modelspec.load_model(MECH / "quad_x.json")
    model = simulink_gen.build_model(spec)
    assert model is not None and model.kind == "rotor"
    assert simulink_gen.validate(model) == []
    maps = [b for b in model.blocks if b.kind == "fcn"]
    assert maps and all("u(1)" in b.params["Expr"] for b in maps)
    assert any(b.params["Expr"].startswith("0.000012") for b in maps)
    assert any(b.params["Expr"].startswith("0.0000002") for b in maps)


def test_simulink_validation_catches_dangling_lines():
    model = simulink_gen.SlModel(name="bad", kind="wheeled")
    model.add("const", "cmd", 0, 0, Value="1")
    model.link("cmd", "missing_block")
    problems = simulink_gen.validate(model)
    assert any("不存在" in p for p in problems)


def test_simulink_bundle_writes_four_files(tmp_path: Path):
    spec = modelspec.load_model(
        ROOT / "examples" / "demo_robot" / "description" / "urdf" / "demo_robot.urdf")
    model = simulink_gen.build_model(spec, command=0.3, used_radius=0.05)
    written = simulink_gen.write_bundle(model, spec, tmp_path)
    assert written["valid"] == "1"
    for key in ("matlab_builder", "mdl", "params", "readme"):
        assert Path(written[key]).exists()
    params = Path(written["params"]).read_text(encoding="utf-8")
    assert "r_true" in params and "r_used" in params
    readme = Path(written["readme"]).read_text(encoding="utf-8")
    assert "无法启动 MATLAB" in readme      # the limitation is stated, not hidden


def test_mechanism_dispatch_covers_all_three():
    arm = modelspec.load_model(MECH / "arm6.json")
    rotor = modelspec.load_model(MECH / "quad_x.json")
    lidar = modelspec.load_model(MECH / "lidar360.json")
    findings, extra = run_mechanism_rules(arm.arm, rotor.rotor, lidar.lidar,
                                          target=(0.45, 0.0, 0.30), payload_kg=2.0)
    rules = {f.rule for f in findings}
    assert {"A03", "P01", "L01"}.issubset(rules)
    assert set(extra) == {"arm", "rotor", "lidar"}
