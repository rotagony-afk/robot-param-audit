"""Cross-stage correlation: runtime measurement + static conflict -> root cause.

This is the step that turns two weak signals into one actionable, reviewable
edit proposal:

  R01 says "odometry reports 1.6% less distance than truth"
  S01 says "wheel_radius is 0.05 in YAML but 0.0508 in URDF"
  -> 0.05 * 1.016 = 0.0508 exactly matches the URDF geometry
  -> propose: change the YAML literal to 0.0508 (and show the diff)

The correlation refuses to decide when the evidence is ambiguous: if the same
scale error could be explained by two parameters and no third source breaks the
tie, the finding is reported as unverifiable with both candidate corrections.
"""

from __future__ import annotations

import math
from typing import Any, Iterable

from . import registry
from .evidence import (Evidence, Fact, Finding, Proposal, fmt, STATUS_CONFIRMED,
                       STATUS_SUSPECTED, STATUS_UNVERIFIABLE)
from .scanners_static import RobotModel

CONFIG_PRIORITY = {"yaml": 0, "launch": 1, "source": 2, "urdf": 3, "sdf": 3}
# A correction recovered from noisy logs will not hit a static literal exactly,
# so corroboration uses a deliberately looser tolerance than conflict detection.
MATCH_TOL = 5e-3


def _numbers(facts: Iterable[Fact], canonical: str) -> list[Fact]:
    return [f for f in facts if f.canonical == canonical and f.is_number]


def _pick(value: float) -> str:
    """Format a corrected value without losing significant digits."""
    if value != 0 and abs(value) < 1e-4:
        return "%.6g" % value
    return fmt(value, 8)


def _clusters(facts: list[Fact], rel_tol: float) -> list[list[Fact]]:
    groups: list[list[Fact]] = []
    for fact in facts:
        for group in groups:
            if registry.values_agree(group[0].value, fact.value, rel_tol):
                group.append(fact)
                break
        else:
            groups.append([fact])
    return groups


def _config_fact(facts: list[Fact], rel_tol: float) -> Fact | None:
    """The value the running node most likely loads (config files beat geometry)."""
    if not facts:
        return None
    ordered = sorted(facts, key=lambda f: (CONFIG_PRIORITY.get(f.source, 9), f.evidence.path,
                                          f.evidence.line or 0))
    return ordered[0]


def _match_elsewhere(facts: list[Fact], target: float, skip: Fact, rel_tol: float) -> Fact | None:
    for fact in facts:
        if fact is skip:
            continue
        if fact.evidence.path == skip.evidence.path and fact.evidence.line == skip.evidence.line:
            continue
        if registry.values_agree(fact.value, target, rel_tol):
            return fact
    return None


def _implied_separation(robots: list[RobotModel]) -> tuple[float, list[Evidence]] | None:
    for model in robots:
        wheels = []
        for name, joint in model.joints.items():
            if "wheel" not in name.lower():
                continue
            origin = joint.get("origin")
            if not origin or len(origin) < 2:
                continue
            wheels.append((name, origin[1], joint.get("origin_ev")))
        ys = [w[1] for w in wheels]
        if len(wheels) < 2 or min(ys) >= 0 or max(ys) <= 0:
            continue
        evidence = [w[2] for w in wheels if w[2]]
        return max(ys) - min(ys), evidence
    return None


def _proposals_for(facts: list[Fact], used_value: float, new_value: float,
                   reference: Fact | None, rel_tol: float, reason: str) -> list[Proposal]:
    proposals: list[Proposal] = []
    for fact in facts:
        if not registry.values_agree(fact.value, used_value, rel_tol):
            continue
        if reference is not None and fact is reference:
            continue
        if fact.factor:
            raw_new = new_value / fact.factor
        else:
            raw_new = new_value
        new_text = _pick(raw_new)
        if new_text == fact.literal.strip().strip("\"'"):
            continue
        proposals.append(Proposal(fact.evidence, fact.literal, new_text, reason))
    return proposals


def correlate_linear(facts: list[Fact], finding: Finding, rel_tol: float) -> Finding | None:
    ratio = finding.metrics.get("ratio")
    if not ratio or ratio <= 0:
        return None
    candidates: list[dict[str, Any]] = []
    specs = (
        ("wheel_radius", _numbers(facts, "wheel_radius"),
         lambda v: v * ratio, "r_true = r_used * (真值/里程计)"),
        ("ticks_per_rev", _numbers(facts, "ticks_per_rev"),
         lambda v: v / ratio, "N_true = N_used / (真值/里程计)"),
        ("gear_ratio", _numbers(facts, "gear_ratio"),
         lambda v: v * ratio, "gear_true = gear_used * (真值/里程计)"),
    )
    for canonical, facts_c, transform, formula in specs:
        if not facts_c:
            continue
        base = _config_fact(facts_c, rel_tol)
        if base is None or not base.value:
            continue
        new_value = transform(base.value)
        match = _match_elsewhere(facts_c, new_value, base, max(rel_tol, MATCH_TOL))
        candidates.append({
            "canonical": canonical, "facts": facts_c, "base": base,
            "new_value": new_value, "match": match, "formula": formula,
        })
    if not candidates:
        return Finding(
            rule="C01", severity="INFO",
            title="检测到线位移尺度偏差，但没有可用于修正的参数",
            detail=("运行日志显示尺度偏差 " + fmt((ratio - 1) * 100, 2) + "%，"
                    "但工程里没有读取到 wheel_radius / ticks_per_rev / gear_ratio，"
                    "无法计算修正值。请确认这些参数是否通过其他方式（如参数服务器、环境变量）注入。"),
            evidence=finding.evidence, confidence="HIGH",
            status=STATUS_UNVERIFIABLE, tags=["correlation"])
    matched = [c for c in candidates if c["match"]]
    if len(matched) >= 1:
        chosen = matched[0]
        # Snap the estimate onto the corroborating literal: the static file is a
        # cleaner statement of the intended value than a noisy log integration.
        new_value = chosen["match"].value
        reason = ("按 " + chosen["formula"] + " 反推得到 " + _pick(chosen["new_value"])
                  + "，与 " + chosen["match"].evidence.location + " 中的 "
                  + fmt(chosen["match"].value) + " 一致，采用该值")
        proposals = _proposals_for(chosen["facts"], chosen["base"].value, new_value,
                                   chosen["match"], rel_tol, reason)
        others = "、".join(registry.label_of(c["canonical"]) for c in candidates
                          if c is not chosen)
        detail = (
            "运行数据给出的尺度比值（真值/里程计）= " + fmt(ratio, 6) + "，"
            "即里程计每报告 1 m，实际走了 " + fmt(ratio, 6) + " m。"
            "把这一比值代入各候选参数：" + "；".join(
                f"{registry.label_of(c['canonical'])} {fmt(c['base'].value)} -> {_pick(c['new_value'])}"
                + ("（与静态文件中的 " + fmt(c["match"].value) + " 一致）" if c["match"] else "")
                for c in candidates
            ) + "。"
        )
        if others:
            detail += ("其中 " + others + " 在数学上同样能解释该误差，但没有任何静态文件支持它，"
                       "因此判定为次要可能。")
        detail += " 修改建议来自两路证据的交叉验证，请你确认轮子实物尺寸后再应用。"
        return Finding(
            rule="C01", severity="HIGH",
            title=("根因收敛：" + registry.label_of(chosen["canonical"]) + " 应为 "
                   + _pick(new_value) + "（当前 " + fmt(chosen["base"].value) + "）"),
            detail=detail,
            evidence=[chosen["base"].evidence, chosen["match"].evidence] + list(finding.evidence),
            confidence="HIGH", status=STATUS_CONFIRMED,
            proposals=proposals, tags=["correlation", "root-cause"],
            metrics={"canonical": chosen["canonical"], "ratio": ratio,
                     "old": chosen["base"].value, "new": new_value,
                     "candidates": [c["canonical"] for c in candidates]},
        )
    if len(candidates) == 1:
        chosen = candidates[0]
        proposals = _proposals_for(chosen["facts"], chosen["base"].value,
                                   chosen["new_value"], None, rel_tol,
                                   "按 " + chosen["formula"] + " 反推（无第二来源可交叉验证）")
        return Finding(
            rule="C01", severity="MEDIUM",
            title=("建议校正 " + registry.label_of(chosen["canonical"]) + ": "
                   + fmt(chosen["base"].value) + " -> " + _pick(chosen["new_value"])),
            detail=("只有一处参数能解释该尺度误差：" + registry.label_of(chosen["canonical"])
                    + "。按 " + chosen["formula"] + " 得到的修正值是 " + _pick(chosen["new_value"])
                    + "，但工程里只有 " + chosen["base"].evidence.location
                    + " 一个来源，缺少独立交叉验证，因此置信度只能给到中。"),
            evidence=[chosen["base"].evidence], confidence="MEDIUM",
            status=STATUS_SUSPECTED, proposals=proposals,
            tags=["correlation", "root-cause"], metrics={"canonical": chosen["canonical"]},
        )
    return Finding(
        rule="C01", severity="MEDIUM",
        title="尺度误差有多个等价解释，无法唯一确定根因",
        detail=("运行数据只给出一个尺度比值 " + fmt(ratio, 6) + "，而下列参数在运动学中彼此等价，"
                "都能产生同样的误差：" + "；".join(
                    registry.label_of(c["canonical"]) + " " + fmt(c["base"].value)
                    + " -> " + _pick(c["new_value"]) for c in candidates)
                + "。静态文件也没有提供任何一处支持其中某个修正值。"
                  "请补充轮子实测直径或编码器规格后再判定；工具不会替你猜。"),
        evidence=[c["base"].evidence for c in candidates], confidence="HIGH",
        status=STATUS_UNVERIFIABLE, tags=["correlation", "ambiguous"],
        metrics={"ratio": ratio, "candidates": [
            {"canonical": c["canonical"], "old": c["base"].value, "new": c["new_value"]}
            for c in candidates]},
    )


def correlate_angular(facts: list[Fact], robots: list[RobotModel], finding: Finding,
                      rel_tol: float) -> Finding | None:
    ratio = finding.metrics.get("ratio")
    if not ratio or ratio <= 0:
        return None
    sep_facts = _numbers(facts, "wheel_separation")
    if not sep_facts:
        return Finding(
            rule="C02", severity="INFO",
            title="检测到转角尺度偏差，但没有读取到 wheel_separation",
            detail="运行日志显示转角偏差 " + fmt((ratio - 1) * 100, 2) + "%，但工程里没有轮距参数，无法给出修正值。",
            evidence=finding.evidence, confidence="HIGH",
            status=STATUS_UNVERIFIABLE, tags=["correlation"])
    base = _config_fact(sep_facts, rel_tol)
    if base is None or not base.value:
        return None
    new_value = base.value / ratio
    implied = _implied_separation(robots)
    match = None
    if implied and registry.values_agree(implied[0], new_value, max(rel_tol, MATCH_TOL)):
        match = Fact("wheel_separation", "joint origin distance", implied[0],
                     _pick(implied[0]), "m", "", "urdf", implied[1][0],
                     note="由左右轮 joint origin 推导")
    if match is None:
        match = _match_elsewhere(sep_facts, new_value, base, max(rel_tol, MATCH_TOL))
    if match is not None:
        new_value = match.value
        reason = ("按 L_true = L_used / (真值/里程计) 反推，并与 " + match.evidence.location
                  + " 一致，采用该值")
        proposals = _proposals_for(sep_facts, base.value, new_value, match, rel_tol, reason)
        return Finding(
            rule="C02", severity="HIGH",
            title=("根因收敛：" + registry.label_of("wheel_separation") + " 应为 "
                   + _pick(new_value) + "（当前 " + fmt(base.value) + "）"),
            detail=("转角比值（真值/里程计）= " + fmt(ratio, 6) + "。差速模型下 ω = (v_r - v_l) / L，"
                    "因此 L_true = L_used / 比值 = " + fmt(base.value) + " / " + fmt(ratio, 6)
                    + " = " + _pick(new_value) + "，与 " + match.evidence.location
                    + " 中的 " + fmt(match.value) + " 吻合。"),
            evidence=[base.evidence, match.evidence] + list(finding.evidence),
            confidence="HIGH", status=STATUS_CONFIRMED,
            proposals=proposals, tags=["correlation", "root-cause"],
            metrics={"canonical": "wheel_separation", "ratio": ratio,
                     "old": base.value, "new": new_value},
        )
    proposals = _proposals_for(sep_facts, base.value, new_value, None, rel_tol,
                               "按 L_true = L_used / 比值 反推（无第二来源可交叉验证）")
    return Finding(
        rule="C02", severity="MEDIUM",
        title=("建议校正 " + registry.label_of("wheel_separation") + ": "
               + fmt(base.value) + " -> " + _pick(new_value)),
        detail=("工程里没有第二处轮距来源可用于交叉验证。按转角比值反推的值是 "
                + _pick(new_value) + "，请用卷尺量一次左右轮中心距后再确认。"),
        evidence=[base.evidence], confidence="MEDIUM", status=STATUS_SUSPECTED,
        proposals=proposals, tags=["correlation", "root-cause"],
        metrics={"canonical": "wheel_separation", "new": new_value},
    )


def correlate(facts: list[Fact], robots: list[RobotModel], findings: list[Finding],
              rel_tol: float = 1e-3) -> list[Finding]:
    out: list[Finding] = []
    for finding in findings:
        if finding.rule == "R01" and finding.severity in ("HIGH", "CRITICAL"):
            extra = correlate_linear(facts, finding, rel_tol)
            if extra:
                out.append(extra)
        elif finding.rule == "R02" and finding.severity in ("HIGH", "CRITICAL"):
            extra = correlate_angular(facts, robots, finding, rel_tol)
            if extra:
                out.append(extra)
    return out
