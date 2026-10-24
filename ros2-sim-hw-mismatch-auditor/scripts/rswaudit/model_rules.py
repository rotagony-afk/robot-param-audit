"""Model-level rules (M01-M10).

These are the checks that make the tool chassis-agnostic. They run on a
ModelSpec (imported from URDF / MJCF / SDF / JSON) plus optional measured
reality, a mesh file and a run log, and they never assume differential drive.
"""

from __future__ import annotations

import math
from typing import Any

from . import registry
from .evidence import Evidence, Finding, Proposal, STATUS_CONFIRMED, STATUS_SUSPECTED, fmt
from .kinematics import (Twist, ackermann_inner_outer, capability_matrix, check_twist,
                         ideal_steering, max_speed)
from .mesh import MeshInfo
from .modelspec import ModelSpec, effective_friction, effective_mass, effective_radius
from .simulate import Profile, SimResult, identify, simulate


def _ev(model: ModelSpec, wheel_name: str = "", field_name: str = "",
        fallback: str = "") -> Evidence:
    for wheel in model.wheels:
        if wheel_name and wheel.name != wheel_name:
            continue
        evidence = wheel.evidence.get(field_name) if wheel.evidence else None
        if evidence is not None:
            return evidence
    if fallback:
        return Evidence(model.source, None, None, "", fallback)
    return Evidence(model.source, None, None, "", "model " + model.name)


# --------------------------------------------------------------------------- #
# M01 - the model must state usable numbers
# --------------------------------------------------------------------------- #

def rule_model_sanity(model: ModelSpec) -> list[Finding]:
    findings: list[Finding] = []
    problems: list[str] = []
    has_mechanism = (model.arm is not None or model.rotor is not None
                     or model.lidar is not None)
    if model.wheels:
        bad_radius = [w.name for w in model.wheels if w.radius <= 0]
        if bad_radius:
            problems.append("轮半径为 0 或缺失: " + ", ".join(bad_radius))
    elif not has_mechanism:
        problems.append("没有识别到任何机构（轮子 / 机械臂 / 螺旋桨 / 激光雷达）")
    if (model.wheels or model.arm is not None) and model.total_mass <= 0:
        problems.append("整机质量为 0，动力学检查无意义")
    if problems:
        findings.append(Finding(
            rule="M01", severity="HIGH", title="模型参数不完整，无法做动力学验证",
            detail="；".join(problems) + "。请补齐后再运行，否则后续结论会失真。",
            evidence=[Evidence(model.source, None, None, "", "model " + model.name)],
            confidence="HIGH", status=STATUS_CONFIRMED, tags=["model"]))
    return findings


# --------------------------------------------------------------------------- #
# M02 - can this chassis physically do the commanded motion?
# --------------------------------------------------------------------------- #

def rule_capability(model: ModelSpec, twist: Twist) -> list[Finding]:
    findings: list[Finding] = []
    caps = capability_matrix(model)
    result = check_twist(model, twist)
    steered = any(w.steerable for w in model.wheels)
    if result.lateral_adjust > 0.01 and steered:
        findings.append(Finding(
            rule="M11", severity="INFO",
            title=("转向几何修正：车身原点需额外 %.3f m/s 侧向速度" % result.lateral_adjust),
            detail=("该底盘是前轮转向结构，转弯时车身原点相对后轴必须带侧向速度分量；"
                    "工具已按固定轮约束把命令投影到可行解（"
                    + fmt(twist.vx, 3) + " / " + fmt(twist.vy, 3) + " -> "
                    + fmt(result.twist.vx, 3) + " / " + fmt(result.twist.vy, 3)
                    + " m/s），后续限速、力矩与转向角都基于修正后的命令计算。"),
            evidence=[Evidence(model.source)], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["steering"],
            metrics={"adjust": result.lateral_adjust}))
    if result.lateral_residual > 0.01 and not steered:
        findings.append(Finding(
            rule="M02", severity="HIGH",
            title=("该底盘无法完成横向运动（残留侧滑 %.3f m/s）" % result.lateral_residual),
            detail=("模型是「" + caps["drive"] + "」，共 " + str(caps["wheels"])
                    + " 个轮子，存在 " + str(caps["lateral_constraints"])
                    + " 条不可侧滑约束，平面自由度只有 " + str(caps["dof"])
                    + "。命令中 " + fmt(twist.vx, 3) + " / " + fmt(twist.vy, 3)
                    + " m/s 的线速度与 " + fmt(twist.w, 3) + " rad/s 角速度组合里，"
                      "有 " + fmt(result.lateral_residual, 4)
                    + " m/s 的侧向分量无处可去——轮子只能打滑，指令中的这部分运动不会发生。"),
            evidence=[_ev(model, "", "radius"),
                      Evidence(model.source, None, None, "",
                               "wheels: " + ", ".join(
                                   w.name + "(" + fmt(w.x, 3) + "," + fmt(w.y, 3)
                                   + ")" for w in model.wheels))],
            confidence="HIGH", status=STATUS_CONFIRMED,
            tags=["capability", "kinematics"],
            metrics={"lateral_residual": result.lateral_residual, **caps}))
    if result.steering and any(abs(v) > result.steering_limit for v in result.steering.values()):
        worst = max(result.steering.items(), key=lambda kv: abs(kv[1]))
        findings.append(Finding(
            rule="M02", severity="MEDIUM", title="转向角需求超出机械限位",
            detail=("该命令要求 " + worst[0] + " 转到 " + fmt(worst[1], 1)
                    + "°，超过限定 " + fmt(result.steering_limit, 0)
                    + "°。实际会绕更大的半径转，轨迹与预期不同。"),
            evidence=[_ev(model, worst[0], "position")],
            confidence="MEDIUM", status=STATUS_CONFIRMED, tags=["capability"]))
    return findings


# --------------------------------------------------------------------------- #
# M03 / M04 / M05 - speed, traction and torque limits
# --------------------------------------------------------------------------- #

def rule_limits(model: ModelSpec, twist: Twist) -> list[Finding]:
    result = check_twist(model, twist, acceleration=0.0)
    findings: list[Finding] = []
    if result.saturated:
        worst = max(abs(v) / (w.max_omega or 1.0)
                    for v, w in zip(result.wheel_omegas, model.wheels) if w.max_omega > 0)
        findings.append(Finding(
            rule="M03", severity="HIGH",
            title=("轮速超限 %.2f 倍：%s" % (worst, ", ".join(result.saturated))),
            detail=("命令需要这些轮子转到 " + ", ".join(
                w.name + "=" + fmt(v, 2) for w, v in zip(model.wheels, result.wheel_omegas)
                if w.name in result.saturated)
                + " rad/s，而电机上限是 "
                + ", ".join(w.name + "=" + fmt(w.max_omega, 2)
                            for w in model.wheels if w.name in result.saturated)
                + " rad/s。实际速度会被封顶，机器人比命令慢。"),
            evidence=[_ev(model, name, "radius") for name in result.saturated[:4]],
            confidence="HIGH", status=STATUS_CONFIRMED, tags=["limit", "speed"],
            metrics={"saturation": result.saturation}))
    if result.traction_margin > 1.0:
        findings.append(Finding(
            rule="M04", severity="HIGH",
            title=("附着不足：需要 %.2f 倍的摩擦极限" % result.traction_margin),
            detail=("按整备质量 " + fmt(effective_mass(model), 2) + " kg、摩擦系数 "
                    + fmt(effective_friction(model), 2) + " 计算，可用牵引力是 "
                    + fmt(sum(result.traction_limit), 1) + " N，而命令所需约 "
                    + fmt(sum(abs(f) for f in result.required_force), 1)
                    + " N。超出后轮子空转，位移小于命令值——这正是“里程计看着正常、实车走不到”的常见原因。"),
            evidence=[_ev(model, "", "radius")],
            confidence="MEDIUM", status=STATUS_CONFIRMED, tags=["traction", "slip"],
            metrics={"traction_margin": result.traction_margin,
                     "friction": effective_friction(model)}))
    if result.torque_margin > 1.0:
        findings.append(Finding(
            rule="M05", severity="MEDIUM",
            title=("力矩不足：需要 %.2f 倍的电机能力" % result.torque_margin),
            detail=("按电机扭矩-转速线性特性估算，命令需要的力矩超过当前转速下的可用力矩 "
                    + fmt(result.torque_margin, 2) + " 倍。表现是加速变慢、高速段上不去。"),
            evidence=[_ev(model, "", "radius")],
            confidence="MEDIUM", status=STATUS_CONFIRMED, tags=["torque"]))
    return findings


# --------------------------------------------------------------------------- #
# M06 - Ackermann geometry consistency
# --------------------------------------------------------------------------- #

def rule_ackermann(model: ModelSpec) -> list[Finding]:
    if model.drive != "ackermann":
        return []
    if model.wheelbase <= 0 or model.track <= 0:
        return [Finding(
            rule="M06", severity="MEDIUM", title="阿克曼底盘缺少轴距/轮距，无法核对转向几何",
            detail=("模型声明为阿克曼转向，但 wheelbase=" + fmt(model.wheelbase, 3)
                    + " m、track=" + fmt(model.track, 3)
                    + " m。请补充尺寸，否则内外轮转角关系无法验证。"),
            evidence=[Evidence(model.source)], confidence="HIGH",
            status=STATUS_SUSPECTED, tags=["ackermann"])]
    # the ideal inner/outer split must satisfy the Ackermann condition
    for curvature in (0.5, 1.0, 2.0):
        angles = ackermann_inner_outer(model, curvature)
        if angles["inner"] - angles["outer"] > 40.0:
            return [Finding(
                rule="M06", severity="LOW",
                title="阿克曼转向几何在小半径下差异过大",
                detail=("曲率 " + fmt(curvature, 2) + " 时内轮 "
                        + fmt(angles["inner"], 1) + "°、外轮 " + fmt(angles["outer"], 1)
                        + "°，差值超过 40°；请确认轴距/轮距是否写反。"),
                evidence=[Evidence(model.source)], confidence="LOW",
                status=STATUS_SUSPECTED, tags=["ackermann"])]
    return []


# --------------------------------------------------------------------------- #
# M07 - 3D mesh mass properties vs the number written in the model
# --------------------------------------------------------------------------- #

def rule_mesh(model: ModelSpec, mesh: MeshInfo, density: float,
              tol: float = 0.2) -> list[Finding]:
    findings: list[Finding] = []
    mass_mesh, _com, tensor = mesh.inertia(density)
    if mesh.volume <= 0:
        return [Finding(
            rule="M07", severity="MEDIUM", title="网格体积为 0，无法反推质量特性",
            detail=("解析到 " + str(mesh.triangles) + " 个三角面，但有符号体积为 0。"
                    "常见原因是网格不是封闭体（只有外壳）或法向混乱。"),
            evidence=[Evidence(mesh.path)], confidence="HIGH",
            status=STATUS_SUSPECTED, tags=["mesh"])]
    declared = model.mass
    if declared > 0 and abs(mass_mesh - declared) / declared > tol:
        implied_density = declared / mesh.volume if mesh.volume else 0.0
        findings.append(Finding(
            rule="M07", severity="HIGH",
            title=("网格反推质量 %.3f kg 与模型声明 %.3f kg 相差 %.1f%%"
                   % (mass_mesh, declared, abs(mass_mesh - declared) / declared * 100.0)),
            detail=("按密度 " + fmt(density, 1) + " kg/m³、体积 "
                    + fmt(mesh.volume * 1000.0, 2) + " L 计算，这个网格应当是 "
                    + fmt(mass_mesh, 3) + " kg；模型里写的是 " + fmt(declared, 3)
                    + " kg。若要让两者一致，密度应当是 " + fmt(implied_density, 1)
                    + " kg/m³。质量错 2 倍以上会直接改变加速、附着与制动距离。"),
            evidence=[Evidence(mesh.path, None, None,
                               "triangles=" + str(mesh.triangles)
                               + " volume=" + fmt(mesh.volume, 6) + " m3"),
                      Evidence(model.source, None, None, "", "declared mass")],
            confidence="HIGH", status=STATUS_CONFIRMED, tags=["mesh", "mass"],
            metrics={"mesh_mass": mass_mesh, "declared_mass": declared,
                     "volume": mesh.volume, "density": density}))
    izz = tensor.get("izz", 0.0)
    if model.declared_inertia_zz and izz > 0:
        rel = abs(izz - model.declared_inertia_zz) / model.declared_inertia_zz
        if rel > 0.35:
            findings.append(Finding(
                rule="M07", severity="MEDIUM",
                title=("网格反推偏航惯量 %.4f 与模型声明 %.4f 相差 %.0f%%"
                       % (izz, model.declared_inertia_zz, rel * 100.0)),
                detail="偏航惯量差太多会让转向响应、超调与稳定时间全部失真。",
                evidence=[Evidence(mesh.path), Evidence(model.source)],
                confidence="MEDIUM", status=STATUS_CONFIRMED, tags=["mesh", "inertia"]))
    # wheels must sit inside the modelled footprint
    if model.wheels and mesh.bbox_max:
        half_x = max(abs(mesh.bbox_min[0]), abs(mesh.bbox_max[0]))
        half_y = max(abs(mesh.bbox_min[1]), abs(mesh.bbox_max[1]))
        outside = [w.name for w in model.wheels
                   if abs(w.x) > half_x * 1.6 + 0.05 or abs(w.y) > half_y * 1.6 + 0.05]
        if outside:
            findings.append(Finding(
                rule="M07", severity="MEDIUM", title="轮子位置远远超出网格包围盒",
                detail=(", ".join(outside) + " 的安装位置超过网格包围盒 ±"
                        + fmt(half_x, 3) + " / ±" + fmt(half_y, 3)
                        + " m。可能是坐标系原点不同（CAD 与 URDF 对不上），"
                          "这会让轮距、轴距、转动惯量全部错位。"),
                evidence=[Evidence(mesh.path), Evidence(model.source)],
                confidence="MEDIUM", status=STATUS_SUSPECTED, tags=["mesh", "geometry"]))
    if not mesh.closed:
        findings.append(Finding(
            rule="M07", severity="LOW", title="网格不封闭（存在边界边）",
            detail=("约 " + fmt(mesh.duplicate_vertices * 100.0, 1)
                    + "% 的边只被一个三角面使用。体积与惯量仍能算，但结果可能偏小；"
                      "建议导出时合并顶点或补洞。"),
            evidence=[Evidence(mesh.path)], confidence="HIGH",
            status=STATUS_SUSPECTED, tags=["mesh"]))
    return findings


# --------------------------------------------------------------------------- #
# M08 - measured reality vs model numbers
# --------------------------------------------------------------------------- #

def rule_reality(model: ModelSpec, tol: float = 0.01) -> list[Finding]:
    findings: list[Finding] = []
    if not model.reality:
        return findings
    checks = [
        ("wheel_radius", effective_radius(model) if "wheel_radius" in model.reality
         else model.wheel_radius, "轮半径", "m"),
        ("mass", model.mass, "整机质量", "kg"),
        ("friction", model.friction, "摩擦系数", ""),
    ]
    items = []
    for key, model_value, label, unit in checks:
        if key not in model.reality:
            continue
        real_value = model.reality[key]
        if model_value <= 0:
            continue
        rel = abs(real_value - model_value) / model_value
        items.append((key, label, unit, model_value, real_value, rel))
    for key, label, unit, model_value, real_value, rel in items:
        if rel <= tol:
            continue
        severity = "HIGH" if rel > 0.02 else "MEDIUM"
        note = ""
        if key == "wheel_radius":
            note = ("里程计会用模型值 " + fmt(model_value, 6) + " 换算，实际是 "
                    + fmt(real_value, 6) + "，因此里程计会系统性偏 "
                    + fmt((model_value / real_value - 1.0) * 100.0, 2) + "%。")
        findings.append(Finding(
            rule="M08", severity=severity,
            title=(label + "模型值 " + fmt(model_value, 6) + (" " + unit if unit else "")
                   + " 与实测 " + fmt(real_value, 6) + " 相差 "
                   + fmt(rel * 100.0, 2) + "%"),
            detail=("模型里写的是 " + fmt(model_value, 6) + "，实测是 "
                    + fmt(real_value, 6) + "。" + note
                    + "这是最容易解释“仿真对、实物不对”的一类差异。"),
            evidence=[Evidence(model.source, None, None, "", "model " + label)],
            confidence="HIGH", status=STATUS_CONFIRMED,
            tags=["reality", key],
            metrics={"model": model_value, "measured": real_value, "rel": rel}))
    return findings


# --------------------------------------------------------------------------- #
# M09 - does the simulation reproduce the measured run?
# --------------------------------------------------------------------------- #

def rule_fit(model: ModelSpec, profile: Profile, measured: dict, dt: float,
             sim: SimResult | None = None, tol: float = 0.03,
             measured_odom: float | None = None,
             radius_used: float | None = None) -> tuple[list[Finding], dict]:
    findings: list[Finding] = []
    sim = sim or simulate(model, profile, dt, radius_used=radius_used)
    distance = measured.get("distance")
    yaw = measured.get("yaw")
    if distance is None and yaw is None:
        return findings, {"skipped": "没有可对比的实测位移/转角"}

    rel_dist = abs(sim.distance - distance) / distance if distance else 0.0
    rel_yaw = abs(sim.yaw - yaw) / abs(yaw) if yaw else 0.0
    metrics = {
        "sim_distance": sim.distance, "sim_yaw": sim.yaw,
        "measured_distance": distance, "measured_yaw": yaw,
        "rel_distance": rel_dist, "rel_yaw": rel_yaw,
        "saturation_fraction": sim.saturation_fraction,
        "slip_fraction": sim.slip_fraction,
        "sim_odom": sim.odom_distance,
        "radius_true": sim.radius_true, "radius_used": sim.radius_used,
    }
    odom_note = ""
    if measured_odom and measured_odom > 0:
        odom_err = sim.odom_distance - measured_odom
        metrics["measured_odom"] = measured_odom
        metrics["odom_rel"] = abs(odom_err) / measured_odom
        if abs(odom_err) / measured_odom <= max(tol, 0.02):
            odom_note = ("  另外，按固件轮半径 " + fmt(sim.radius_used, 6)
                         + " m 推出来的里程计是 " + fmt(sim.odom_distance, 4)
                         + " m，实测里程计是 " + fmt(measured_odom, 4)
                         + " m，相差 " + fmt(abs(odom_err) / measured_odom * 100.0, 2)
                         + "%。也就是说：模型的物理参数是对的，但固件换算用的轮半径与实物 "
                         + fmt(sim.radius_true, 6) + " m 不同，这条里程计误差可以被完整解释。")
    compared: list[str] = []
    if distance:
        compared.append("位移差 " + fmt(rel_dist * 100.0, 2) + "%")
    if yaw:
        compared.append("转角差 " + fmt(rel_yaw * 100.0, 2) + "%")
    sim_text = ("用模型参数前向仿真得到 " + fmt(sim.distance, 4) + " m / "
                + fmt(sim.yaw, 4) + " rad")
    measured_text = ("实测 " + (fmt(distance, 4) if distance else "?") + " m / "
                     + (fmt(yaw, 4) if yaw else "?") + " rad")
    ok = ((not distance or rel_dist <= tol)
          and (not yaw or rel_yaw <= max(tol, 0.05)))
    if ok:
        findings.append(Finding(
            rule="M09", severity="INFO",
            title="仿真与实测一致（" + "，".join(compared) + "）",
            detail=(sim_text + "，" + measured_text + "。模型可以解释这次运行。"
                    + odom_note),
            evidence=[Evidence(model.source)], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["fit", "pass"], metrics=metrics))
        return findings, metrics

    findings.append(Finding(
        rule="M09", severity="HIGH",
        title="仿真与实测不符：" + "，".join(compared),
        detail=(sim_text + "，" + measured_text + "。这说明模型里至少有一个物理参数与实物"
                "不符——这本身就是一个可复现的验证结论；下一步用参数拟合定位到具体是哪一个。"
                + odom_note),
        evidence=[Evidence(model.source)], confidence="HIGH",
        status=STATUS_CONFIRMED, tags=["fit"], metrics=metrics))
    return findings, metrics


def rule_identify(model: ModelSpec, profile: Profile, measured: dict,
                  dt: float, measured_odom: float | None = None,
                  measured_odom_yaw: float | None = None,
                  radius_used: float | None = None,
                  track_used: float | None = None,
                  fit_ok: bool = False
                  ) -> tuple[list[Finding], dict]:
    labels = {"wheel_radius": "轮半径", "wheel_separation": "轮距",
              "latency": "控制延迟"}
    findings: list[Finding] = []
    direct: dict[str, Any] = {}

    # 1) direct measurement: truth/odometry is exactly the parameter ratio
    truth_distance = measured.get("distance")
    if truth_distance and measured_odom and measured_odom > 0:
        ratio = truth_distance / measured_odom
        r_used = radius_used if radius_used is not None else model.wheel_radius
        r_true = r_used * ratio
        direct["radius"] = {"ratio": ratio, "used": r_used, "true": r_true}
        if abs(ratio - 1.0) > 0.01:
            findings.append(Finding(
                rule="M10", severity="HIGH",
                title=("根因收敛：轮半径应为 " + fmt(r_true, 6) + " m（当前用 "
                       + fmt(r_used, 6) + " m 换算）"),
                detail=("实测真值位移 " + fmt(truth_distance, 4) + " m，里程计报告 "
                        + fmt(measured_odom, 4) + " m，比值 "
                        + fmt(ratio, 6) + "。里程计是用轮半径换算的，因此实物轮半径 = "
                        + fmt(r_used, 6) + " × " + fmt(ratio, 6) + " = "
                        + fmt(r_true, 6) + " m。这是直接比值，不依赖拟合，置信度高。"),
                evidence=[Evidence(model.source)], confidence="HIGH",
                status=STATUS_CONFIRMED, tags=["identification", "radius"],
                metrics={"ratio": ratio, "radius_used": r_used, "radius_true": r_true}))

    truth_yaw = measured.get("yaw")
    if truth_yaw and measured_odom_yaw and abs(measured_odom_yaw) > 1e-6:
        ratio_yaw = truth_yaw / measured_odom_yaw
        l_used = track_used if track_used is not None else (model.track or 0.0)
        l_true = l_used / ratio_yaw if ratio_yaw else 0.0
        direct["track"] = {"ratio": ratio_yaw, "used": l_used, "true": l_true}
        if abs(ratio_yaw - 1.0) > 0.01 and l_true > 0:
            findings.append(Finding(
                rule="M10", severity="HIGH",
                title=("根因收敛：轮距应为 " + fmt(l_true, 4) + " m（当前用 "
                       + fmt(l_used, 4) + " m 换算）"),
                detail=("实测真值转角 " + fmt(truth_yaw, 4) + " rad，里程计报告 "
                        + fmt(measured_odom_yaw, 4) + " rad，比值 "
                        + fmt(ratio_yaw, 6) + "。差速模型里 omega = (v_r - v_l)/L，"
                          "所以实物轮距 = " + fmt(l_used, 4) + " / " + fmt(ratio_yaw, 6)
                        + " = " + fmt(l_true, 4) + " m。同样是直接比值，不依赖拟合。"),
                evidence=[Evidence(model.source)], confidence="HIGH",
                status=STATUS_CONFIRMED, tags=["identification", "track"],
                metrics={"ratio": ratio_yaw, "track_used": l_used, "track_true": l_true}))

    if fit_ok:
        # the simulation already reproduces the run, so there is nothing left for
        # a sensitivity fit to explain; only the direct ratios above are reported
        return findings, {"direct": direct, "skipped": "仿真与实测已一致"}

    # 2) sensitivity fit for anything the ratios cannot explain
    result = identify(model, profile, measured, dt)
    plausible: dict = result["plausible"]
    for key in direct:
        plausible.pop("wheel_radius" if key == "radius" else "wheel_separation", None)
    summary = {
        "components": result["components"],
        "plausible": plausible,
        "deltas": result["deltas"],
        "base_residual": result["base_residual"],
        "identifiable": result["identifiable"],
        "direct": direct,
    }
    if not plausible:
        return findings, summary

    if len(plausible) == 1:
        key, delta = next(iter(plausible.items()))
        if key == "wheel_radius":
            action = ("把轮半径从 " + fmt(model.wheel_radius, 6) + " 改为 "
                      + fmt(model.wheel_radius * (1.0 + delta), 6) + " m")
        elif key == "wheel_separation":
            action = ("把轮距从 " + fmt(model.track, 4) + " 改为 "
                      + fmt((model.track or 0.0) * (1.0 + delta), 4) + " m")
        else:
            action = "在控制链路中加入约 " + fmt(abs(delta) * 1000.0, 0) + " ms 延迟"
        findings.append(Finding(
            rule="M10", severity="HIGH",
            title=("根因收敛：" + labels.get(key, key) + "应修正 "
                   + fmt(delta * 100.0, 2) + "%"),
            detail=("逐个参数单独试算后，只有 " + labels.get(key, key)
                    + " 能在物理上解释这次实测偏差，" + action
                    + "，仿真即可与实测吻合。结论来自前向仿真 + 数值灵敏度，"
                      "属于参数级验证，不是接触动力学仿真。"),
            evidence=[Evidence(model.source)], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["identification"], metrics=summary))
        return findings, summary

    candidates = "；".join(labels.get(k, k) + " " + fmt(v * 100.0, 1) + "%"
                          for k, v in plausible.items())
    findings.append(Finding(
        rule="M10", severity="MEDIUM", title="多个参数都能解释这次偏差，拒绝给出单一根因",
        detail=("单独调整下列任一参数都能让仿真对上实测：" + candidates
                + "。当前命令下它们的影响接近共线，仅凭这一次运行无法区分；"
                  "请再提供一段不同曲率或不同速度的行程后再拟合。"),
        evidence=[Evidence(model.source)], confidence="HIGH",
        status="unverifiable", tags=["identification", "ambiguous"], metrics=summary))
    return findings, summary


def run_model_rules(model: ModelSpec, twist: Twist | None = None,
                    mesh: MeshInfo | None = None, density: float | None = None,
                    profile: Profile | None = None,
                    measured: dict | None = None,
                    dt: float = 0.02, measured_odom: float | None = None,
                    measured_odom_yaw: float | None = None,
                    radius_used: float | None = None,
                    track_used: float | None = None
                    ) -> tuple[list[Finding], dict[str, Any]]:
    twist = twist or Twist(0.3, 0.0, 0.0)
    findings: list[Finding] = []
    extra: dict[str, Any] = {"capability": capability_matrix(model)}
    findings.extend(rule_model_sanity(model))
    findings.extend(rule_capability(model, twist))
    findings.extend(rule_limits(model, twist))
    findings.extend(rule_ackermann(model))
    findings.extend(rule_reality(model))
    if mesh is not None and density:
        findings.extend(rule_mesh(model, mesh, density))
    if profile is not None and measured is not None:
        fit_findings, fit_metrics = rule_fit(
            model, profile, measured, dt,
            measured_odom=measured_odom, radius_used=radius_used)
        findings.extend(fit_findings)
        extra["fit"] = fit_metrics
        ident_findings, ident_metrics = rule_identify(
            model, profile, measured, dt, measured_odom=measured_odom,
            measured_odom_yaw=measured_odom_yaw, radius_used=radius_used,
            track_used=track_used,
            fit_ok=bool(fit_findings) and fit_findings[0].severity == "INFO")
        findings.extend(ident_findings)
        extra["identify"] = ident_metrics
    return findings, extra
