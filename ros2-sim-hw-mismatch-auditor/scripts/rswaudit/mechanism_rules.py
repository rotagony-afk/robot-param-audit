"""Rules for the non-wheeled mechanisms (A / P / L series)."""

from __future__ import annotations

import math
from typing import Any

from .evidence import Evidence, Finding, STATUS_CONFIRMED, STATUS_SUSPECTED, fmt
from .mechanisms import ArmSpec, LidarSpec, RotorSpec


def _ev(source: str, locator: str) -> Evidence:
    return Evidence(source, None, None, "", locator)


# --------------------------------------------------------------------------- #
# A: serial arm
# --------------------------------------------------------------------------- #

def rule_arm(arm: ArmSpec, target: tuple[float, float, float] | None,
             payload_kg: float, source: str) -> tuple[list[Finding], dict]:
    findings: list[Finding] = []
    info: dict[str, Any] = {"dof": arm.dof}
    if arm.dof == 0:
        return [Finding(
            rule="A01", severity="HIGH", title="机械臂没有任何关节",
            detail="模型里没有解析到关节，无法做运动学校验。",
            evidence=[_ev(source, "arm")], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["arm"])], info

    if target is None:
        # the zero configuration is always within the joint limits, so the tip
        # position it produces is a reachable default; real checks should pass
        # an explicit --target
        zero = arm.forward([0.0] * arm.dof)
        target = (zero[0][3], zero[1][3], zero[2][3])
        info["assumed_target"] = list(target)
    target = tuple(float(v) for v in target)

    solution = arm.inverse(target)
    residual = solution["residual"]
    info.update({"target": target, "residual": residual,
                 "q": solution["q"], "at_limit": solution["at_limit"]})

    if residual > 1e-3:
        findings.append(Finding(
            rule="A01", severity="HIGH",
            title=("目标点不可达（末端残差 %.1f mm）" % (residual * 1000.0)),
            detail=("在关节限位内做阻尼最小二乘逆解，末端最多只能到距离目标 "
                    + fmt(residual * 1000.0, 1) + " mm 的位置（目标 "
                    + ", ".join(fmt(v, 3) for v in target) + " m）。"
                    "常见原因：臂长不够、关节限位太窄，或模型里某段连杆长度写错。"
                    + (" 触到限位的关节：" + ", ".join(solution["at_limit"])
                       if solution["at_limit"] else "")),
            evidence=[_ev(source, "arm target " + str(target))],
            confidence="HIGH", status=STATUS_CONFIRMED, tags=["arm", "reach"],
            metrics={"residual_mm": residual * 1000.0}))

    manipulability = arm.manipulability(solution["q"])
    info["manipulability"] = manipulability
    if manipulability < 1e-3 and residual <= 1e-3:
        findings.append(Finding(
            rule="A02", severity="MEDIUM", title="解位于或接近奇异位形",
            detail=("在该位形下位置雅可比的可操作度只有 " + fmt(manipulability, 6)
                    + "（< 1e-3）。此时末端某个方向几乎动不了，逆解对误差极其敏感，"
                      "实机会表现为关节突然大幅转动。建议换一个构型或增加冗余自由度。"),
            evidence=[_ev(source, "arm")], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["arm", "singularity"],
            metrics={"manipulability": manipulability}))

    force = (0.0, 0.0, -payload_kg * 9.81) if payload_kg > 0 else (0.0, 0.0, 0.0)
    torques = arm.static_torque(solution["q"], force)
    info["static_torque"] = torques
    if payload_kg > 0:
        over = []
        for joint, torque in zip(arm.joints, torques):
            if joint.effort > 0 and abs(torque) > joint.effort:
                over.append((joint.name, torque, joint.effort))
        if over:
            findings.append(Finding(
                rule="A03", severity="HIGH",
                title=("负载 %.2f kg 时关节力矩超限（%s）"
                       % (payload_kg, ", ".join(item[0] for item in over))),
                detail=("只算重力静载，需要的关节力矩就已经超过额定："
                        + "；".join(item[0] + " 需要 " + fmt(item[1], 2) + " N·m / 额定 "
                                    + fmt(item[2], 2) + " N·m" for item in over)
                        + "。实际还要加上加减速动态力矩，超限会更严重。"),
                evidence=[_ev(source, "arm payload")],
                confidence="HIGH", status=STATUS_CONFIRMED, tags=["arm", "torque"],
                metrics={"torques": torques, "payload_kg": payload_kg}))
        else:
            ratios = [joint.effort / abs(torque)
                      for joint, torque in zip(arm.joints, torques)
                      if joint.effort > 0 and abs(torque) > 1e-9]
            info["torque_margin"] = min(ratios) if ratios else 0.0

    if arm.dof < 3:
        findings.append(Finding(
            rule="A04", severity="LOW", title="自由度少于 3，无法覆盖三维工作空间",
            detail=("只有 " + str(arm.dof) + " 个关节，末端点只能在子空间内移动，"
                    "任何三维目标都会有结构性的不可达。"),
            evidence=[_ev(source, "arm")], confidence="HIGH",
            status=STATUS_SUSPECTED, tags=["arm"]))
    return findings, info


# --------------------------------------------------------------------------- #
# P: rotor / propeller
# --------------------------------------------------------------------------- #

def rule_rotor(rotor: RotorSpec, source: str) -> tuple[list[Finding], dict]:
    findings: list[Finding] = []
    info: dict[str, Any] = {}
    if rotor.kT <= 0 or rotor.max_rpm <= 0:
        return [Finding(
            rule="P01", severity="MEDIUM", title="缺少螺旋桨系数或最大转速",
            detail="没有 kT（推力系数）或 max_rpm，无法计算推力与功率。请补充，"
                   "这两个数通常来自桨厂家的实测曲线。",
            evidence=[_ev(source, "rotor")], confidence="HIGH",
            status=STATUS_SUSPECTED, tags=["rotor"])], info

    max_thrust = rotor.thrust(rotor.max_rpm)
    max_torque = rotor.torque(rotor.max_rpm)
    max_power = rotor.power(rotor.max_rpm)
    info.update({"max_thrust_per_rotor": max_thrust,
                 "max_torque": max_torque, "max_power": max_power,
                 "total_thrust": max_thrust * rotor.count})
    if rotor.declared_max_thrust > 0:
        rel = abs(max_thrust - rotor.declared_max_thrust) / rotor.declared_max_thrust
        info["declared_thrust_rel"] = rel
        if rel > 0.10:
            findings.append(Finding(
                rule="P01", severity="HIGH",
                title=("声明最大推力 %.2f N 与 kT/转速算出的 %.2f N 相差 %.0f%%"
                       % (rotor.declared_max_thrust, max_thrust, rel * 100.0)),
                detail=("T = kT·ω²，按 kT = " + fmt(rotor.kT, 8) + "、max_rpm = "
                        + fmt(rotor.max_rpm, 0) + " 算出来单桨最大推力是 "
                        + fmt(max_thrust, 3) + " N，而规格里写的是 "
                        + fmt(rotor.declared_max_thrust, 3) + " N。"
                        "两者不一致时，飞控里按声明值做的推力分配会整体偏差。"),
                evidence=[_ev(source, "rotor thrust")], confidence="HIGH",
                status=STATUS_CONFIRMED, tags=["rotor", "thrust"],
                metrics={"computed": max_thrust,
                         "declared": rotor.declared_max_thrust}))
    if rotor.motor_torque_limit > 0 and max_torque > rotor.motor_torque_limit:
        findings.append(Finding(
            rule="P02", severity="HIGH",
            title=("满转所需扭矩 %.3f N·m 超过电机上限 %.3f N·m"
                   % (max_torque, rotor.motor_torque_limit)),
            detail=("Q = kQ·ω²，满转时每桨需要 " + fmt(max_torque, 4)
                    + " N·m，电机只能给 " + fmt(rotor.motor_torque_limit, 4)
                    + " N·m，实际转速到不了 max_rpm，真实最大推力低于计算值。"),
            evidence=[_ev(source, "rotor torque")], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["rotor", "torque"]))

    if rotor.total_mass_kg > 0 and rotor.count > 0:
        weight = rotor.total_mass_kg * rotor.gravity
        total_max = max_thrust * rotor.count
        info["weight"] = weight
        info["thrust_to_weight"] = total_max / weight if weight else 0.0
        if total_max < weight:
            findings.append(Finding(
                rule="P03", severity="HIGH", title="满油门推力不足以悬停",
                detail=("整机 " + fmt(rotor.total_mass_kg, 2) + " kg 需要 "
                        + fmt(weight, 1) + " N，而 " + str(rotor.count) + " 桨满转只有 "
                        + fmt(total_max, 1) + " N（推重比 "
                        + fmt(total_max / weight, 2) + "）。这架机器根本飞不起来，"
                          "参数或桨数一定有一处写错。"),
                evidence=[_ev(source, "rotor hover")], confidence="HIGH",
                status=STATUS_CONFIRMED, tags=["rotor", "hover"]))
        else:
            hover = rotor.hover_rpm()
            info["hover_rpm"] = hover
    loading = rotor.disk_loading(rotor.max_rpm)
    info["disk_loading"] = loading
    if loading > 400.0:
        findings.append(Finding(
            rule="P04", severity="LOW", title="桨盘载荷偏高，效率不佳",
            detail=("满载桨盘载荷 " + fmt(loading, 0) + " N/m²（多旋翼通常 100–300）。"
                    "同样的拉力需要更大功率与更高转速，续航会明显变差。"),
            evidence=[_ev(source, "rotor disk")], confidence="MEDIUM",
            status=STATUS_SUSPECTED, tags=["rotor", "efficiency"]))
    return findings, info


# --------------------------------------------------------------------------- #
# L: lidar
# --------------------------------------------------------------------------- #

def rule_lidar(lidar: LidarSpec, source: str) -> tuple[list[Finding], dict]:
    findings: list[Finding] = []
    pps = lidar.points_per_second()
    blind = lidar.ground_blind_radius()
    info: dict[str, Any] = {"points_per_second": pps,
                            "blind_radius": blind,
                            "spot_at_max": lidar.spot_size(lidar.range_max_m)}
    if pps > 0 and lidar.declared_points_per_s > 0:
        rel = abs(pps - lidar.declared_points_per_s) / lidar.declared_points_per_s
        info["declared_rel"] = rel
        if rel > 0.10:
            findings.append(Finding(
                rule="L01", severity="HIGH",
                title=("声明 %s 点/秒与实际扫描几何不符（应为 %s）"
                       % (fmt(lidar.declared_points_per_s, 0), fmt(pps, 0))),
                detail=("按水平视场 " + fmt(lidar.fov_h_deg, 1) + "°、角分辨率 "
                        + fmt(lidar.az_res_deg, 3) + "°、竖直 " + str(lidar.vert_lines)
                        + " 线、帧率 " + fmt(lidar.rate_hz, 1) + " Hz 计算，每秒点是 "
                        + fmt(pps, 0) + "，而规格写的是 "
                        + fmt(lidar.declared_points_per_s, 0)
                        + "。这直接影响带宽估算与点云密度规划（建图、避障算法都按这个数配）。"),
                evidence=[_ev(source, "lidar points")], confidence="HIGH",
                status=STATUS_CONFIRMED, tags=["lidar", "spec"],
                metrics={"computed": pps,
                         "declared": lidar.declared_points_per_s}))
    if blind is not None and blind > 0.4:
        findings.append(Finding(
            rule="L02", severity="MEDIUM",
            title=("安装高度 %.2f m 时地面盲区半径 %.2f m"
                   % (lidar.mount_height_m, blind)),
            detail=("最低一条光束与水平面夹角 "
                    + fmt(-(lidar.mount_pitch_deg) - lidar.fov_v_deg / 2.0, 1)
                    + "°，在高度 " + fmt(lidar.mount_height_m, 2) + " m 处落地，"
                      "以机身为中心 " + fmt(blind, 2) + " m 半径内看不到地面。"
                      "移动机器人要确认这个盲区小于最小避障距离，否则贴地障碍会漏检。"),
            evidence=[_ev(source, "lidar mount")], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["lidar", "mount"],
            metrics={"blind_radius": blind}))
    if lidar.accuracy_m > 0 and lidar.range_resolution_m > 0 \
            and lidar.accuracy_m < lidar.range_resolution_m:
        findings.append(Finding(
            rule="L03", severity="LOW", title="精度优于量程分辨率，规格自相矛盾",
            detail=("标注精度 " + fmt(lidar.accuracy_m, 4) + " m 小于量程分辨率 "
                    + fmt(lidar.range_resolution_m, 4)
                    + " m。分辨率是最小可分辨步长，精度不可能比它更细，建议核对数据手册。"),
            evidence=[_ev(source, "lidar accuracy")], confidence="MEDIUM",
            status=STATUS_SUSPECTED, tags=["lidar", "spec"]))
    spot = lidar.spot_size(lidar.range_max_m)
    info["spot_ratio"] = spot / lidar.range_max_m if lidar.range_max_m else 0.0
    return findings, info


# --------------------------------------------------------------------------- #

def run_mechanism_rules(arm: ArmSpec | None, rotor: RotorSpec | None,
                        lidar: LidarSpec | None,
                        target: tuple[float, float, float] | None = None,
                        payload_kg: float = 0.0,
                        source: str = "model") -> tuple[list[Finding], dict]:
    findings: list[Finding] = []
    extra: dict[str, Any] = {}
    if arm is not None:
        arm_findings, arm_info = rule_arm(
            arm, target, payload_kg if payload_kg else arm.payload_kg, source)
        findings.extend(arm_findings)
        extra["arm"] = arm_info
    if rotor is not None:
        rotor_findings, rotor_info = rule_rotor(rotor, source)
        findings.extend(rotor_findings)
        extra["rotor"] = rotor_info
    if lidar is not None:
        lidar_findings, lidar_info = rule_lidar(lidar, source)
        findings.extend(lidar_findings)
        extra["lidar"] = lidar_info
    return findings, extra
