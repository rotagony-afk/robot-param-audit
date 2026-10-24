"""Static inconsistency rules (S01-S10).

Each rule is a pure function over what the scanners actually read. A rule may
only report what it can point at: every finding carries the exact facts (with
file, line, column) that produced it. When the files alone cannot tell which
value is the physical truth, the rule says so instead of guessing.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

from . import registry
from .evidence import (Evidence, Fact, Finding, Proposal, fmt,
                       STATUS_CONFIRMED, STATUS_SUSPECTED)
from .scanners_static import RobotModel, YamlEntry

SOURCE_PRIORITY = {"yaml": 0, "launch": 1, "source": 2, "urdf": 3, "sdf": 3}


def group_facts(facts: Iterable[Fact], rel_tol: float = 1e-3) -> dict[tuple[str, str], list[Fact]]:
    groups: dict[tuple[str, str], list[Fact]] = {}
    for fact in facts:
        if not fact.is_number:
            continue
        scope = fact.scope if registry.is_scoped(fact.canonical) else ""
        groups.setdefault((fact.canonical, scope), []).append(fact)
    return groups


def cluster_facts(facts: list[Fact], rel_tol: float = 1e-3) -> list[list[Fact]]:
    groups: list[list[Fact]] = []
    for fact in facts:
        for group in groups:
            if registry.values_agree(group[0].value, fact.value, rel_tol):
                group.append(fact)
                break
        else:
            groups.append([fact])
    return groups


def describe(facts: list[Fact]) -> str:
    parts = []
    for fact in sorted(facts, key=lambda f: (SOURCE_PRIORITY.get(f.source, 9), f.evidence.path)):
        unit = (" " + fact.unit) if fact.unit else ""
        parts.append(f"{fact.source} {fmt(fact.value)}{unit} @ {fact.evidence.location}")
    return " / ".join(parts)


def sources_of(facts: list[Fact]) -> set[str]:
    return {f.source for f in facts}


def unique_value(facts: list[Fact], rel_tol: float = 1e-3) -> float | None:
    """Return the value only when all sources agree; otherwise None."""
    numbers = [f.value for f in facts if f.is_number]
    if not numbers:
        return None
    clusters = registry.cluster(numbers, rel_tol)
    if len(clusters) > 1:
        return None
    return clusters[0][0]


# --------------------------------------------------------------------------- #
# S01 / S02 - one quantity, several different numbers
# --------------------------------------------------------------------------- #

def rule_conflicts(facts: list[Fact], rel_tol: float) -> list[Finding]:
    findings: list[Finding] = []
    for (canonical, scope), group in group_facts(facts, rel_tol).items():
        if len(group) < 2:
            continue
        clusters = cluster_facts(group, rel_tol)
        if len(clusters) < 2:
            continue
        means = [registry.mean([f.value for f in c]) for c in clusters]
        unit_ratio = None
        for i in range(len(means)):
            for j in range(i + 1, len(means)):
                guess = registry.guess_unit_scale(means[i], means[j])
                if guess:
                    unit_ratio = guess
                    break
            if unit_ratio:
                break
        label = registry.label_of(canonical)
        unit = registry.unit_of(canonical)
        evidence = [f.evidence for f in group]
        srcs = sources_of(group)
        numeric = [abs(v) for v in means if v]
        rel = 0.0
        if len(numeric) > 1:
            rel = abs(max(numeric) - min(numeric)) / max(numeric)
        if unit_ratio:
            finding = Finding(
                rule="S02", severity="CRITICAL",
                title=f"疑似单位不一致：{label}（比值 {fmt(unit_ratio, 4)}）",
                detail=(f"{label} 在不同来源相差约 {fmt(unit_ratio, 4)} 倍，"
                        f"最可能是米/毫米/厘米混用。当前取值：" + describe(group)
                        + "。只要有一处是按错误单位写入，里程计与限速都会被整体放大或缩小。"
                          "工具无法判断哪一份配置会被真正加载，因此标记为待确认——"
                          "请确认这份文件是否仍在编译/启动链路中。"),
                evidence=evidence, confidence="HIGH", status=STATUS_SUSPECTED,
                tags=["unit", canonical],
                metrics={"canonical": canonical, "unit_ratio": unit_ratio,
                         "values": [fmt(v) for v in means]},
            )
            findings.append(finding)
            continue
        sign_only = len(clusters) == 2 and registry.guess_sign(
            clusters[0][0].value, clusters[1][0].value, rel_tol)
        severity = "HIGH" if len(srcs) > 1 or sign_only else "MEDIUM"
        extra = "（两者仅符号相反，通常是正负号或安装方向约定不同）" if sign_only else ""
        confidence = "HIGH" if all(f.confidence == "HIGH" for f in group) else "MEDIUM"
        findings.append(Finding(
            rule="S01", severity=severity,
            title=f"{label} 多处定义且数值不一致" + ("（2 组）" if len(clusters) == 2 else f"（{len(clusters)} 组）"),
            detail=(f"{label} 共被读取到 {len(group)} 次，分成 {len(clusters)} 个不同的值，"
                    f"最大相对差 {fmt(rel * 100, 3)}%。" + extra + " 当前取值：" + describe(group)
                    + "。仅凭静态文件无法判定哪一处对应实物，需结合运行日志反推（见 C 类结论）。"),
            evidence=evidence, confidence=confidence, status=STATUS_CONFIRMED,
            tags=["conflict", canonical],
            metrics={"canonical": canonical, "rel_diff": rel,
                     "values": [fmt(v) for v in means], "sources": sorted(srcs)},
        ))
    return findings


# --------------------------------------------------------------------------- #
# S03 - values that could not be evaluated statically
# --------------------------------------------------------------------------- #

def rule_unresolved(facts: list[Fact]) -> list[Finding]:
    unresolved = [f for f in facts if f.unresolved]
    if not unresolved:
        return []
    lines = []
    for fact in unresolved[:12]:
        lines.append(f"{fact.source} {fact.raw_name}={fact.literal} @ {fact.evidence.location}")
    return [Finding(
        rule="S03", severity="INFO",
        title=f"{len(unresolved)} 个参数是表达式，静态阶段无法求值",
        detail=("以下参数不是字面量（例如 xacro/launch 变量替换），本次比对跳过它们，"
                "因此这些参数不参与任何数值结论：" + "；".join(lines)),
        evidence=[f.evidence for f in unresolved[:12]],
        confidence="HIGH", status="unverifiable", tags=["unresolved"],
        metrics={"count": len(unresolved)},
    )]


# --------------------------------------------------------------------------- #
# S04 - wheel separation declared vs. geometry implied by joint origins
# --------------------------------------------------------------------------- #

def rule_separation_vs_joints(facts: list[Fact], robots: list[RobotModel],
                              rel_tol: float) -> list[Finding]:
    findings: list[Finding] = []
    yaml_sep = [f for f in facts if f.canonical == "wheel_separation" and f.is_number]
    if not yaml_sep:
        return findings
    declared = unique_value(yaml_sep, rel_tol)
    for model in robots:
        wheels: list[tuple[str, float, Evidence]] = []
        for name, joint in model.joints.items():
            if "wheel" not in name.lower():
                continue
            origin = joint.get("origin")
            if not origin or len(origin) < 2:
                continue
            wheels.append((name, origin[1], joint.get("origin_ev")))
        if len(wheels) < 2:
            continue
        ys = [w[1] for w in wheels]
        if min(ys) >= 0 or max(ys) <= 0:
            continue
        implied = max(ys) - min(ys)
        if declared is None:
            base = max(yaml_sep, key=lambda f: abs(f.value))
            declared_value = base.value
            ambiguous = True
        else:
            declared_value = declared
            ambiguous = False
        if registry.values_agree(implied, declared_value, rel_tol):
            continue
        rel = registry.rel_diff(implied, declared_value)
        evidence = [f.evidence for f in yaml_sep] + [w[2] for w in wheels if w[2]]
        findings.append(Finding(
            rule="S04", severity="HIGH",
            title=(f"{registry.label_of('wheel_separation')}与 URDF 轮子安装位置不一致"
                   f"（声明 {fmt(declared_value)} m / 几何 {fmt(implied)} m）"),
            detail=(f"配置里 wheel_separation = {fmt(declared_value)} m，"
                    f"但 URDF 中左右轮 joint origin 的 y 分量为 "
                    + ", ".join(f"{w[0]}={fmt(w[1])}" for w in wheels)
                    + f"，两者相差 {fmt(rel * 100, 3)}%。差速转向角速度 = (v_r - v_l) / wheel_separation，"
                      "该值偏小会让机器人转身角度偏大。"),
            evidence=evidence,
            confidence="MEDIUM" if ambiguous else "HIGH",
            status=STATUS_CONFIRMED if not ambiguous else STATUS_SUSPECTED,
            tags=["conflict", "wheel_separation", "geometry"],
            metrics={"declared": declared_value, "implied": implied, "rel_diff": rel},
        ))
    return findings


# --------------------------------------------------------------------------- #
# S05 - wheel joints must share a rotation-axis direction
# --------------------------------------------------------------------------- #

_AXIS_LABELS = {(0.0, 0.0, 1.0): "绕 z 轴", (0.0, 1.0, 0.0): "绕 y 轴", (1.0, 0.0, 0.0): "绕 x 轴"}


def _axis_key(axis: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(round(abs(v), 6) for v in axis[:3])


def rule_wheel_axis(robots: list[RobotModel]) -> list[Finding]:
    findings: list[Finding] = []
    for model in robots:
        wheels = []
        for name, joint in model.joints.items():
            if "wheel" not in name.lower():
                continue
            axis = joint.get("axis")
            if not axis:
                continue
            wheels.append((name, _axis_key(axis), joint.get("axis_ev"), joint.get("type", "")))
        if len(wheels) < 2:
            continue
        keys = {w[1] for w in wheels}
        evidence = [w[2] for w in wheels if w[2]]
        if len(keys) > 1:
            desc = ", ".join(f"{w[0]}={list(w[1])}" for w in wheels)
            findings.append(Finding(
                rule="S05", severity="HIGH",
                title="轮子 joint 的旋转轴方向不一致",
                detail=("差速驱动要求所有驱动轮绕同一条轴旋转，否则同一速度指令下轮子转向不同。"
                        f"当前 URDF 中：{desc}。"),
                evidence=evidence, confidence="HIGH", status=STATUS_CONFIRMED,
                tags=["axis", "urdf"], metrics={"axes": [list(k) for k in keys]},
            ))
            continue
        key = next(iter(keys))
        if key[0] >= 0.99:
            findings.append(Finding(
                rule="S05", severity="MEDIUM",
                title="轮子 joint 的旋转轴与前进方向平行",
                detail=("驱动轮 joint 的 axis 为 x 方向，与机器人前进方向重合；"
                        "若轮子按此轴转动，切向速度会指向车体左右而不是前后。请确认 axis 是否写错。"),
                evidence=evidence, confidence="MEDIUM", status=STATUS_SUSPECTED,
                tags=["axis", "urdf"], metrics={"axis": list(key)},
            ))
    return findings


# --------------------------------------------------------------------------- #
# S06 - inertial sanity (a zero mass makes Gazebo produce NaNs)
# --------------------------------------------------------------------------- #

def rule_inertia(robots: list[RobotModel]) -> list[Finding]:
    findings: list[Finding] = []
    for model in robots:
        missing: list[Evidence] = []
        zero_inertia: list[Evidence] = []
        bad_mass: list[Evidence] = []
        for name, link in model.links.items():
            if not name or not isinstance(link, dict):
                continue
            mass = link.get("mass")
            inertia = link.get("inertia")
            if mass is not None and mass <= 0:
                bad_mass.append(link.get("mass_ev") or Evidence(model.path))
            elif mass is None and link.get("inertia_ev") is None:
                line = link.get("line")
                missing.append(Evidence(model.path, line, None, "",
                                        "link '" + name + "' 没有 inertial 块"))
            if inertia and all(v == 0 for v in inertia):
                zero_inertia.append(link.get("inertia_ev") or Evidence(model.path))
        if bad_mass:
            findings.append(Finding(
                rule="S06", severity="HIGH", title="连杆质量为 0 或负数",
                detail="质量为 0 的连杆在 Gazebo 中会产生 NaN 或直接被丢弃，仿真随之发散。",
                evidence=bad_mass, confidence="HIGH", status=STATUS_CONFIRMED,
                tags=["inertial", "urdf"],
            ))
        if zero_inertia:
            findings.append(Finding(
                rule="S06", severity="HIGH", title="转动惯量三个主轴全为 0",
                detail=("惯量矩阵全 0 会让物理引擎求逆失败，表现为连杆乱飞或仿真报 singular。"
                        "至少给 ixx/iyy/izz 一个非零值。"),
                evidence=zero_inertia, confidence="HIGH", status=STATUS_CONFIRMED,
                tags=["inertial", "urdf"],
            ))
        if missing:
            findings.append(Finding(
                rule="S06", severity="MEDIUM", title=f"{len(missing)} 个连杆没有惯性参数",
                detail=("这些连杆没有 inertial 块，Gazebo 会套用默认质量与零惯量，"
                        "与设计意图不符且容易引发不稳定。"),
                evidence=missing[:8], confidence="HIGH", status=STATUS_SUSPECTED,
                tags=["inertial", "urdf"],
            ))
    return findings


# --------------------------------------------------------------------------- #
# S07 -控制频率 vs 仿真插件更新频率
# --------------------------------------------------------------------------- #

def rule_rates(facts: list[Fact], rel_tol: float) -> list[Finding]:
    control = [f for f in facts if f.canonical == "control_rate" and f.is_number]
    sim = [f for f in facts if f.canonical == "sim_update_rate" and f.is_number]
    if not control or not sim:
        return []
    ctrl = unique_value(control, rel_tol)
    simv = unique_value(sim, rel_tol)
    if ctrl is None or simv is None or simv <= 0:
        return []
    if ctrl <= simv * 1.01:
        return []
    ratio = ctrl / simv
    integral = abs(ratio - round(ratio)) < 0.05
    return [Finding(
        rule="S07", severity="HIGH" if not integral else "MEDIUM",
        title=f"控制频率 {fmt(ctrl, 4)} Hz 高于仿真更新频率 {fmt(simv, 4)} Hz",
        detail=(f"控制器每秒下发 {fmt(ctrl, 4)} 次指令，而 Gazebo 侧每 {fmt(simv, 4)} Hz 才更新一次，"
                f"频率比 {fmt(ratio, 4)}{'不是整数' if not integral else '为整数'}。"
                "多出来的指令会被丢弃或合并，仿真中看到的行为与实际控制器不同，"
                "这也常是“仿真里好好的、上实物就抖”的根源之一。"),
        evidence=[f.evidence for f in control] + [f.evidence for f in sim],
        confidence="MEDIUM", status=STATUS_CONFIRMED,
        tags=["timing", "rate"], metrics={"control_rate": ctrl, "sim_rate": simv, "ratio": ratio},
    )]


# --------------------------------------------------------------------------- #
# S08 - declared max speed cannot be reached by the wheels
# --------------------------------------------------------------------------- #

def rule_reachability(facts: list[Fact], rel_tol: float) -> list[Finding]:
    lin = [f for f in facts if f.canonical == "max_linear_vel" and f.is_number]
    wheel = [f for f in facts if f.canonical == "max_wheel_vel" and f.is_number]
    radius = [f for f in facts if f.canonical == "wheel_radius" and f.is_number]
    if not lin:
        return []
    max_lin = unique_value(lin, rel_tol)
    max_wheel = unique_value(wheel, rel_tol)
    rad = unique_value(radius, rel_tol)
    if max_lin is None or max_wheel is None or rad is None or max_wheel <= 0 or rad <= 0:
        return []
    reachable = max_wheel * rad
    if max_lin <= reachable * 1.01:
        return []
    return [Finding(
        rule="S08", severity="MEDIUM",
        title=f"限速 {fmt(max_lin)} m/s 超过轮子实际能力 {fmt(reachable)} m/s",
        detail=(f"max_linear_vel = {fmt(max_lin)} m/s，但单轮上限 {fmt(max_wheel)} rad/s、"
                f"轮半径 {fmt(rad)} m 只能跑到 {fmt(reachable)} m/s。"
                "超出部分永远达不到，导航/控制器会长期处于饱和状态。"),
        evidence=[f.evidence for f in lin] + [f.evidence for f in wheel] + [f.evidence for f in radius],
        confidence="HIGH", status=STATUS_CONFIRMED,
        tags=["limit"], metrics={"max_linear_vel": max_lin, "reachable": reachable},
    )]


# --------------------------------------------------------------------------- #
# S09 - launch passes a numeric parameter nobody declares
# --------------------------------------------------------------------------- #

def rule_launch_unknown(facts: list[Fact], yaml_entries: list[tuple[str, list[YamlEntry], str]],
                        launch_info: list[tuple[str, dict]]) -> list[Finding]:
    declared: set[str] = set()
    for _path, entries, _text in yaml_entries:
        for entry in entries:
            declared.add(entry.key.lower())
    if not declared:
        return []
    findings: list[Finding] = []
    for path, info in launch_info:
        for param in info.get("params", []):
            if param.get("canonical") is not None:
                continue
            token = str(param.get("value", ""))
            try:
                float(token)
            except ValueError:
                continue
            if param["name"].lower() in declared:
                continue
            findings.append(Finding(
                rule="S09", severity="LOW",
                title=f"launch 传入的参数 {param['name']} 在配置文件中找不到",
                detail=(f"{path} 向节点传入了数值参数 {param['name']}={token}，"
                        "但没有任何 YAML 配置声明同名键。可能是拼写错误，也可能是只在 launch 里临时覆盖；"
                        "请人工确认，工具不会替你判断。"),
                evidence=[param["evidence"]],
                confidence="LOW", status=STATUS_SUSPECTED, tags=["launch"],
            ))
    return findings


# --------------------------------------------------------------------------- #
# S10 - duplicated keys inside one YAML scope (last one silently wins)
# --------------------------------------------------------------------------- #

def rule_yaml_duplicates(yaml_entries: list[tuple[str, list[YamlEntry], str]]) -> list[Finding]:
    findings: list[Finding] = []
    for path, entries, _text in yaml_entries:
        seen: dict[tuple[tuple[str, ...], str], list[YamlEntry]] = {}
        for entry in entries:
            seen.setdefault((entry.path, entry.key.lower()), []).append(entry)
        for (_path, key), group in seen.items():
            if len(group) < 2:
                continue
            findings.append(Finding(
                rule="S10", severity="MEDIUM",
                title=f"同一 YAML 作用域内重复定义 {key}",
                detail=(f"{path} 中 {key} 出现 {len(group)} 次（第 "
                        + ", ".join(str(e.line) for e in group)
                        + " 行）。加载器通常只保留最后一次，前面的值被静默丢弃，"
                          "改错了地方也不会报错。"),
                evidence=[Evidence(path, e.line, e.col, "", "yaml " + key) for e in group],
                confidence="HIGH", status=STATUS_CONFIRMED, tags=["yaml"],
            ))
    return findings


# --------------------------------------------------------------------------- #

def run_static(facts: list[Fact], robots: list[RobotModel],
               yaml_entries: list[tuple[str, list[YamlEntry], str]],
               launch_info: list[tuple[str, dict]],
               rel_tol: float = 1e-3) -> list[Finding]:
    findings: list[Finding] = []
    findings.extend(rule_conflicts(facts, rel_tol))
    findings.extend(rule_unresolved(facts))
    findings.extend(rule_separation_vs_joints(facts, robots, rel_tol))
    findings.extend(rule_wheel_axis(robots))
    findings.extend(rule_inertia(robots))
    findings.extend(rule_rates(facts, rel_tol))
    findings.extend(rule_reachability(facts, rel_tol))
    findings.extend(rule_launch_unknown(facts, yaml_entries, launch_info))
    findings.extend(rule_yaml_duplicates(yaml_entries))
    return findings
