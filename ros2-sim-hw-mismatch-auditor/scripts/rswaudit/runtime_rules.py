"""Runtime rules (R01-R08) over a recorded log.

Every number in a finding is computed from the log columns that were actually
present. If a needed column is missing the rule reports "unverifiable" with the
reason, which is how the tool avoids the classic failure mode of an AI assistant
claiming "the odometry is 1.5% off" without ever seeing ground truth.
"""

from __future__ import annotations

import math
from typing import Any

from . import registry
from .evidence import Evidence, Finding, fmt, STATUS_CONFIRMED, STATUS_SUSPECTED
from .scanners_log import (LogTable, diff_stats, finite_pairs, integrate, path_length,
                           unwrap)

SCALE_TOL = 0.01          # 1% scale error before we call it a mismatch
JITTER_WARN = 0.20
JITTER_BAD = 0.50


_EVIDENCE_ROLES = ("t", "cmd_lin", "cmd_ang", "odom_lin", "odom_ang",
                   "odom_yaw", "truth_x", "truth_y", "truth_yaw")


def _ev(table: LogTable, idx: int, note: str = "") -> Evidence:
    """Evidence pointing at one log row, quoting the values on that row."""
    lines = getattr(table, "row_lines", None) or []
    line = lines[idx] if 0 <= idx < len(lines) else None
    parts = []
    for role in _EVIDENCE_ROLES:
        if not table.has(role):
            continue
        column = table.columns[role]
        if idx < len(column):
            parts.append(role + "=" + fmt(column[idx], 5))
        if len(parts) >= 4:
            break
    excerpt = " ".join(parts)
    locator = "第 " + str(idx + 1) + " 个样本"
    if note:
        locator += "（" + note + "）"
    return Evidence(table.path, line, None, excerpt, locator)


def _truth_distance(table: LogTable) -> tuple[float, str] | None:
    if table.has("truth_x") and table.has("truth_y"):
        total, used = path_length(table.col("truth_x"), table.col("truth_y"))
        if used > 0:
            return total, "由 truth_x/truth_y 轨迹累积"
    if table.has("truth_lin"):
        t, _ok = table.time_axis()
        total, used, _skipped = integrate(t, table.col("truth_lin"))
        if used > 0:
            return abs(total), "由 truth_lin 积分"
    return None


def _truth_yaw(table: LogTable) -> tuple[float, str] | None:
    if table.has("truth_yaw"):
        angles = unwrap(table.col("truth_yaw"))
        finite = [a for a in angles if a == a]
        if len(finite) >= 2:
            return abs(finite[-1] - finite[0]), "由 truth_yaw 首尾差值"
    return None


# --------------------------------------------------------------------------- #
# R01 - linear odometry scale
# --------------------------------------------------------------------------- #

def rule_linear_scale(table: LogTable, expect_distance: float | None,
                      scale_tol: float = SCALE_TOL) -> list[Finding]:
    if not table.has("odom_lin"):
        return [Finding(
            rule="R01", severity="INFO", title="缺少里程计线速度列，无法核对线位移尺度",
            detail="日志里没有 odom_lin（或同义列）。请导出 odom 的 vx 后再跑一次。",
            evidence=[Evidence(table.path)], confidence="HIGH", status="unverifiable",
            tags=["odom", "missing-input"])]

    t, time_ok = table.time_axis()
    odom_total, used, skipped = integrate(t, table.col("odom_lin"))
    odom_total = abs(odom_total)
    truth = _truth_distance(table)
    source = ""
    if expect_distance is not None:
        truth_value = abs(expect_distance)
        source = "由 --expect-distance 提供（实测直线距离）"
    elif truth:
        truth_value, source = truth
    else:
        return [Finding(
            rule="R01", severity="INFO",
            title="日志中没有真值位移，里程计绝对尺度无法判定",
            detail=("没有 truth_x/truth_y、truth_lin 或 --expect-distance，"
                    "只能确认里程计自己积出来的距离是 " + fmt(odom_total, 4) + " m，"
                    "无法判断它对不对。补一个外部真值（卷尺实测、动捕、仿真真值话题）即可判定。"),
            evidence=[_ev(table, 0, "odom_lin 有效积分段 " + str(used) + " 段")],
            confidence="HIGH", status="unverifiable", tags=["odom"],
            metrics={"odom_distance": odom_total, "used_segments": used},
        )]
    if odom_total <= 1e-6:
        return [Finding(
            rule="R01", severity="INFO", title="里程计线速度全程为 0",
            detail="odom_lin 积分为 0，可能机器人没有移动，或里程计话题没有被记录。",
            evidence=[_ev(table, 0, "odom_lin integrated to 0")],
            confidence="HIGH", status="unverifiable", tags=["odom"])]

    ratio = truth_value / odom_total
    metrics: dict[str, Any] = {
        "odom_distance": odom_total, "truth_distance": truth_value,
        "ratio": ratio, "truth_source": source, "used_segments": used,
        "skipped_segments": skipped, "time_axis_ok": time_ok,
    }
    if registry.rel_diff(ratio, 1.0) <= scale_tol:
        return [Finding(
            rule="R01", severity="INFO",
            title=f"线位移尺度正常（真值 {fmt(truth_value, 4)} m / 里程计 {fmt(odom_total, 4)} m）",
            detail=(f"偏差 {fmt((ratio - 1) * 100, 3)}%，在 {fmt(scale_tol * 100, 0)}% 容差内。"
                    f"真值来源：{source}。"),
            evidence=[_ev(table, 0, "truth=" + fmt(truth_value) + " odom=" + fmt(odom_total))],
            confidence="HIGH" if time_ok else "MEDIUM", status=STATUS_CONFIRMED,
            tags=["odom", "pass"], metrics=metrics)]

    return [Finding(
        rule="R01", severity="HIGH",
        title=f"里程计线位移偏差 {fmt((ratio - 1) * 100, 2)}%（真值 {fmt(truth_value, 4)} m / 里程计 {fmt(odom_total, 4)} m）",
        detail=(f"真值 {fmt(truth_value, 4)} m，里程计积分 {fmt(odom_total, 4)} m，比值 {fmt(ratio, 6)}。"
                f"真值来源：{source}。"
                "这个比值只说明“尺度错了”，不能直接说明是哪个参数错了：轮半径、每转计数、减速比三者"
                "在运动学里等价。下面给出的修正候选需要与静态冲突交叉验证后才能定案。"),
        evidence=[_ev(table, 0, "first odom sample"), _ev(table, table.n - 1, "last odom sample")],
        confidence="HIGH" if time_ok else "MEDIUM", status=STATUS_CONFIRMED,
        tags=["odom", "scale", "linear"], metrics=metrics)]


# --------------------------------------------------------------------------- #
# R02 - angular odometry scale
# --------------------------------------------------------------------------- #

def rule_angular_scale(table: LogTable, expect_yaw: float | None,
                       scale_tol: float = SCALE_TOL) -> list[Finding]:
    have_rate = table.has("odom_ang")
    have_pose = table.has("odom_yaw")
    if not have_rate and not have_pose:
        return [Finding(
            rule="R02", severity="INFO", title="缺少里程计角速度/姿态列，无法核对转角尺度",
            detail="日志里既没有 odom_ang 也没有 odom_yaw。",
            evidence=[Evidence(table.path)], confidence="HIGH", status="unverifiable",
            tags=["odom", "missing-input"])]

    if have_pose:
        angles = unwrap(table.col("odom_yaw"))
        finite = [a for a in angles if a == a]
        odom_yaw = abs(finite[-1] - finite[0]) if len(finite) >= 2 else 0.0
    else:
        t, _ok = table.time_axis()
        odom_yaw = abs(integrate(t, table.col("odom_ang"))[0])

    truth = _truth_yaw(table)
    if expect_yaw is not None:
        truth_value, source = abs(expect_yaw), "由 --expect-yaw 提供（实测转角）"
    elif truth:
        truth_value, source = truth
    else:
        return [Finding(
            rule="R02", severity="INFO", title="日志中没有真值转角，角尺度无法判定",
            detail=("里程计累计转角 " + fmt(odom_yaw, 4) + " rad，但没有 truth_yaw 或 --expect-yaw，"
                    "无法判断轮距是否正确。"),
            evidence=[_ev(table, max(table.n - 1, 0), "odom yaw " + fmt(odom_yaw))],
            confidence="HIGH", status="unverifiable", tags=["odom"],
            metrics={"odom_yaw": odom_yaw})]
    if odom_yaw <= 1e-4:
        return [Finding(
            rule="R02", severity="INFO", title="里程计转角几乎为 0，角尺度无法判定",
            detail="机器人可能没有转向，或 odom_yaw/odom_ang 没有被记录。",
            evidence=[_ev(table, 0, "odom yaw ~ 0")], confidence="HIGH",
            status="unverifiable", tags=["odom"])]

    ratio = truth_value / odom_yaw
    metrics = {"odom_yaw": odom_yaw, "truth_yaw": truth_value, "ratio": ratio,
               "truth_source": source}
    if registry.rel_diff(ratio, 1.0) <= scale_tol:
        return [Finding(
            rule="R02", severity="INFO",
            title=f"转角尺度正常（真值 {fmt(truth_value, 4)} rad / 里程计 {fmt(odom_yaw, 4)} rad）",
            detail=f"偏差 {fmt((ratio - 1) * 100, 3)}%，在容差内。真值来源：{source}。",
            evidence=[_ev(table, max(table.n - 1, 0), "yaw comparison")],
            confidence="HIGH", status=STATUS_CONFIRMED, tags=["odom", "pass"], metrics=metrics)]
    return [Finding(
        rule="R02", severity="HIGH",
        title=f"里程计转角偏差 {fmt((ratio - 1) * 100, 2)}%（真值 {fmt(truth_value, 4)} rad / 里程计 {fmt(odom_yaw, 4)} rad）",
        detail=(f"真值 {fmt(truth_value, 4)} rad，里程计 {fmt(odom_yaw, 4)} rad，比值 {fmt(ratio, 6)}。"
                "差速模型中角速度 = (v_r - v_l) / wheel_separation，"
                "比值偏离 1 直接指向轮距（或左右轮尺度不一致）。"),
        evidence=[_ev(table, 0, "first sample"), _ev(table, table.n - 1, "last sample")],
        confidence="HIGH", status=STATUS_CONFIRMED,
        tags=["odom", "scale", "angular"], metrics=metrics)]


# --------------------------------------------------------------------------- #
# R03 - control period jitter
# --------------------------------------------------------------------------- #

def rule_dt_jitter(table: LogTable) -> list[Finding]:
    t, time_ok = table.time_axis()
    stats = diff_stats(t)
    if not time_ok:
        return [Finding(
            rule="R03", severity="INFO", title="没有可信时间列，无法评估控制周期抖动",
            detail="日志缺时间戳或时间戳是合成出来的，周期类指标不可信。",
            evidence=[Evidence(table.path)], confidence="HIGH", status="unverifiable",
            tags=["timing"])]
    if stats.get("n", 0) < 5 or not stats.get("median"):
        return [Finding(
            rule="R03", severity="INFO", title="样本太少，无法评估控制周期抖动",
            detail="有效时间差少于 5 段。", evidence=[Evidence(table.path)],
            confidence="HIGH", status="unverifiable", tags=["timing"])]
    jitter = stats["jitter_frac"]
    median = stats["median"]
    metrics = {k: (float(v) if isinstance(v, (int, float)) else v) for k, v in stats.items()}
    metrics["median_hz"] = 1.0 / median if median else None
    if jitter < JITTER_WARN:
        return [Finding(
            rule="R03", severity="INFO",
            title=f"控制周期稳定（中位 {fmt(median * 1000, 3)} ms ≈ {fmt(1 / median, 3)} Hz）",
            detail=f"p95 相对中位抖动 {fmt(jitter * 100, 2)}%，低于 {fmt(JITTER_WARN * 100, 0)}% 门限。",
            evidence=[_ev(table, 1, "period statistics")],
            confidence="HIGH", status=STATUS_CONFIRMED, tags=["timing", "pass"],
            metrics=metrics)]
    severity = "HIGH" if jitter >= JITTER_BAD else "MEDIUM"
    return [Finding(
        rule="R03", severity=severity,
        title=f"控制周期抖动 {fmt(jitter * 100, 1)}%（中位 {fmt(median * 1000, 3)} ms，最大 {fmt(stats['max'] * 1000, 3)} ms）",
        detail=("时间戳显示控制/记录循环周期不稳定。p95 周期比中位数长 "
                f"{fmt((stats['p95'] - median) * 1000, 3)} ms。"
                "周期抖动会让 PID 的微分项和积分项都算错，表现为同一组参数在实机上时好时坏。"
                "请检查是否与 ROS 通信、仿真步进或调度优先级耦合。"),
        evidence=[_ev(table, 1, "period statistics")],
        confidence="HIGH", status=STATUS_CONFIRMED, tags=["timing"], metrics=metrics)]


# --------------------------------------------------------------------------- #
# R04 - command-to-response latency
# --------------------------------------------------------------------------- #

def rule_latency(table: LogTable, max_lag: int = 60) -> list[Finding]:
    if not (table.has("cmd_lin") and table.has("odom_lin")):
        return [Finding(
            rule="R04", severity="INFO", title="缺少指令或反馈列，无法测量响应延迟",
            detail="需要同时有 cmd_lin 与 odom_lin。", evidence=[Evidence(table.path)],
            confidence="HIGH", status="unverifiable", tags=["latency"])]
    cmd, odom = table.col("cmd_lin"), table.col("odom_lin")
    pairs = finite_pairs(cmd, odom)
    if len(pairs) < 40:
        return [Finding(
            rule="R04", severity="INFO", title="样本太少，无法测量响应延迟",
            detail=f"有效样本 {len(pairs)} < 40。", evidence=[Evidence(table.path)],
            confidence="HIGH", status="unverifiable", tags=["latency"])]
    xs = [p[0] for p in pairs]
    ys = [p[1] for p in pairs]
    span = max(xs) - min(xs)
    if span < 0.05:
        return [Finding(
            rule="R04", severity="INFO", title="指令几乎是常量，无法测量响应延迟",
            detail="cmd_lin 变化范围小于 0.05 m/s，需要阶跃或变化指令才能测延迟。",
            evidence=[Evidence(table.path)], confidence="HIGH", status="unverifiable",
            tags=["latency"])]
    n = len(xs)
    limit = max(1, min(max_lag, n // 4))
    best_lag, best_score = 0, -2.0
    for lag in range(0, limit + 1):
        a = xs[: n - lag]
        b = ys[lag:]
        if len(a) < 10:
            break
        ma = sum(a) / len(a)
        mb = sum(b) / len(b)
        num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
        da = math.sqrt(sum((x - ma) ** 2 for x in a))
        db = math.sqrt(sum((y - mb) ** 2 for y in b))
        score = num / (da * db) if da and db else 0.0
        if score > best_score:
            best_lag, best_score = lag, score
    t, time_ok = table.time_axis()
    stats = diff_stats(t)
    dt = stats.get("median") or 0.0
    lag_s = best_lag * dt if time_ok else None
    metrics = {"lag_samples": best_lag, "lag_s": lag_s, "corr": best_score,
               "dt": dt, "time_axis_ok": time_ok}
    if not time_ok or lag_s is None:
        return [Finding(
            rule="R04", severity="INFO", title="存在信号延迟但时间轴不可信",
            detail=f"互相关峰值出现在 {best_lag} 个样本之后（相关系数 {fmt(best_score, 3)}），"
                   "但缺少可信时间戳，无法换算成秒。",
            evidence=[_ev(table, 0, "cross-correlation")], confidence="LOW",
            status="unverifiable", tags=["latency"], metrics=metrics)]
    one_cycle = dt
    if lag_s <= max(1.5 * one_cycle, 0.03):
        return [Finding(
            rule="R04", severity="INFO",
            title=f"指令到反馈延迟在 1 个控制周期内（约 {fmt(lag_s * 1000, 2)} ms）",
            detail=f"互相关峰值出现在 {best_lag} 个样本（{fmt(lag_s * 1000, 2)} ms，相关系数 {fmt(best_score, 3)}）。",
            evidence=[_ev(table, best_lag, "cross-correlation peak")],
            confidence="MEDIUM", status=STATUS_CONFIRMED, tags=["latency", "pass"],
            metrics=metrics)]
    severity = "HIGH" if lag_s > 0.15 else "MEDIUM"
    return [Finding(
        rule="R04", severity=severity,
        title=f"指令到反馈存在约 {fmt(lag_s * 1000, 1)} ms 延迟（{best_lag} 个周期）",
        detail=("cmd_lin 与 odom_lin 的互相关峰值出现在 " + str(best_lag) + " 个样本之后，"
                f"按中位周期 {fmt(dt * 1000, 3)} ms 折算约 {fmt(lag_s * 1000, 1)} ms"
                f"（相关系数 {fmt(best_score, 3)}）。延迟在闭环里等效于相位滞后，"
                "会在 PID 增益偏高时直接引起振荡，这也解释“仿真不抖实机抖”。"),
        evidence=[_ev(table, 0, "cross-correlation"),
                  _ev(table, min(best_lag, table.n - 1), "lagged response")],
        confidence="MEDIUM", status=STATUS_CONFIRMED, tags=["latency"], metrics=metrics)]


# --------------------------------------------------------------------------- #
# R05 - command clipping
# --------------------------------------------------------------------------- #

def rule_saturation(table: LogTable) -> list[Finding]:
    if not table.has("cmd_lin"):
        return []
    values = [v for v in table.col("cmd_lin") if v == v]
    if len(values) < 20:
        return []
    peak = max(abs(v) for v in values)
    if peak <= 0:
        return []
    non_zero = sorted(abs(v) for v in values if abs(v) > 1e-9)
    if not non_zero:
        return []
    typical = non_zero[len(non_zero) // 2]
    # A run that only ever commands one speed is a constant-speed test, not a
    # saturated loop. Only flag a plateau that clearly stands above the rest.
    if peak <= typical * 1.5:
        return []
    at_peak = sum(1 for v in values if abs(abs(v) - peak) <= max(peak * 1e-3, 1e-9))
    frac = at_peak / len(values)
    if frac < 0.05:
        return []
    first = next(i for i, v in enumerate(table.col("cmd_lin")) if v == v and abs(abs(v) - peak) <= peak * 1e-3)
    return [Finding(
        rule="R05", severity="MEDIUM",
        title=f"指令疑似被限幅：{fmt(frac * 100, 1)}% 的样本正好停在 {fmt(peak)} m/s",
        detail=("指令大量堆积在同一个极值上，说明要么外部限速把指令削平了，"
                "要么控制器输出饱和。饱和期间闭环失去调节能力，"
                "此时调 PID 是没有意义的。"),
        evidence=[_ev(table, first, "first sample at peak " + fmt(peak))],
        confidence="MEDIUM", status=STATUS_SUSPECTED, tags=["limit", "saturation"],
        metrics={"peak": peak, "fraction": frac})]


# --------------------------------------------------------------------------- #
# R06 - oscillation / overshoot on the last stable segment
# --------------------------------------------------------------------------- #

def rule_oscillation(table: LogTable) -> list[Finding]:
    if not (table.has("cmd_lin") and table.has("odom_lin")):
        return [Finding(
            rule="R06", severity="INFO", title="缺少指令或反馈列，无法评估超调与振荡",
            detail="需要同时有 cmd_lin 与 odom_lin。", evidence=[Evidence(table.path)],
            confidence="HIGH", status="unverifiable", tags=["pid"])]
    cmd, odom = table.col("cmd_lin"), table.col("odom_lin")
    n = min(len(cmd), len(odom))
    segments: list[tuple[int, int, float]] = []
    start = 0
    for idx in range(1, n):
        if cmd[idx] != cmd[idx - 1] and abs(cmd[idx] - cmd[idx - 1]) > 1e-12:
            segments.append((start, idx, cmd[start]))
            start = idx
    segments.append((start, n, cmd[start]))
    usable = [(s, e, v) for s, e, v in segments if e - s >= 12 and abs(v) > 0.05]
    if not usable:
        return [Finding(
            rule="R06", severity="INFO", title="没有可分析的恒定指令段",
            detail="日志里缺少长度足够的恒速段（至少 12 个样本且速度大于 0.05 m/s），"
                   "超调与振荡只能在阶跃后评估。",
            evidence=[Evidence(table.path)], confidence="HIGH", status="unverifiable",
            tags=["pid"])]
    seg_start, seg_end, setpoint = max(usable, key=lambda s: s[1] - s[0])
    keep_from = seg_start + int(0.3 * (seg_end - seg_start))
    window = [odom[i] for i in range(keep_from, seg_end) if odom[i] == odom[i]]
    if len(window) < 6:
        return []
    sign = 1.0 if setpoint > 0 else -1.0
    peak = max(v * sign for v in window)
    overshoot = (peak - abs(setpoint)) / abs(setpoint)
    deadband = abs(setpoint) * 0.02
    crossings = 0
    prev = 0
    for v in window:
        err = (v - setpoint) * sign
        if err > deadband:
            cur = 1
        elif err < -deadband:
            cur = -1
        else:
            continue
        if prev and cur != prev:
            crossings += 1
        prev = cur
    t, time_ok = table.time_axis()
    duration = 0.0
    if time_ok and seg_end - 1 < len(t):
        duration = t[seg_end - 1] - t[keep_from]
    freq = crossings / (2 * duration) if duration > 0 else None
    metrics = {"setpoint": setpoint, "overshoot_pct": overshoot * 100,
               "crossings": crossings, "freq_hz": freq, "segment_samples": seg_end - seg_start}
    evidence = [_ev(table, keep_from, "setpoint " + fmt(setpoint)),
                _ev(table, min(seg_end - 1, table.n - 1), "response tail")]
    if crossings >= 3 or overshoot > 0.10:
        parts = []
        if overshoot > 0.10:
            parts.append(f"超调 {fmt(overshoot * 100, 1)}%")
        if crossings >= 3:
            parts.append(f"稳态附近穿越 {crossings} 次")
        detail = ("恒速段 setpoint = " + fmt(setpoint) + " m/s 的响应中，" + "、".join(parts) + "。")
        if freq:
            detail += f"估计振荡频率约 {fmt(freq, 3)} Hz（周期 {fmt(1 / freq, 4)} s）。"
        detail += ("控制器振荡通常来自增益过高、反馈延迟或采样周期抖动；"
                   "工具只负责指出现象与量化指标，具体改哪个增益需要你结合 R03/R04 一起判断。")
        return [Finding(
            rule="R06", severity="MEDIUM" if overshoot <= 0.30 and crossings < 8 else "HIGH",
            title="速度环存在超调/振荡：" + "、".join(parts),
            detail=detail, evidence=evidence, confidence="MEDIUM",
            status=STATUS_CONFIRMED, tags=["pid", "oscillation"], metrics=metrics)]
    return [Finding(
        rule="R06", severity="INFO",
        title=f"速度环响应平稳（超调 {fmt(max(overshoot, 0) * 100, 1)}%，穿越 {crossings} 次）",
        detail=f"恒速段 setpoint = {fmt(setpoint)} m/s，未见明显振荡。",
        evidence=evidence, confidence="MEDIUM", status=STATUS_CONFIRMED,
        tags=["pid", "pass"], metrics=metrics)]


# --------------------------------------------------------------------------- #
# R07 - feedback stuck at zero while commands keep coming
# --------------------------------------------------------------------------- #

def rule_stuck(table: LogTable) -> list[Finding]:
    if not (table.has("cmd_lin") and table.has("odom_lin")):
        return []
    cmd, odom = table.col("cmd_lin"), table.col("odom_lin")
    t, time_ok = table.time_axis()
    if not time_ok:
        return []
    run_start = None
    best: tuple[int, int] | None = None
    for idx in range(min(len(cmd), len(odom), len(t))):
        stuck = cmd[idx] == cmd[idx] and odom[idx] == odom[idx] and abs(cmd[idx]) > 0.05 and abs(odom[idx]) < 0.005
        if stuck and run_start is None:
            run_start = idx
        elif not stuck and run_start is not None:
            if t[idx - 1] - t[run_start] >= 0.5:
                if best is None or idx - 1 - run_start > best[1] - best[0]:
                    best = (run_start, idx - 1)
            run_start = None
    if best is None:
        return []
    start, end = best
    duration = t[end] - t[start]
    return [Finding(
        rule="R07", severity="HIGH",
        title=f"反馈长时间停在 0：{fmt(duration, 3)} s 内有指令但无响应",
        detail=(f"从 t={fmt(t[start], 3)} s 到 t={fmt(t[end], 3)} s，cmd_lin 保持 "
                f"{fmt(cmd[start], 3)} m/s 左右，而 odom_lin 恒为 0。"
                "典型原因：编码器线序/中断没接上、话题没发布、里程计被重置、"
                "或指令被 publish 到了没人订阅的命名空间。"),
        evidence=[_ev(table, start, "stuck start"),
                  _ev(table, end, "stuck end")],
        confidence="HIGH", status=STATUS_CONFIRMED, tags=["odom", "dead-signal"],
        metrics={"duration_s": duration, "cmd": cmd[start]})]


# --------------------------------------------------------------------------- #
# R08 - time axis sanity
# --------------------------------------------------------------------------- #

def rule_time_sanity(table: LogTable) -> list[Finding]:
    t, time_ok = table.time_axis()
    if not time_ok or len(t) < 3:
        return []
    nan_count = sum(1 for v in t if v != v)
    non_monotonic = sum(1 for i in range(1, len(t)) if t[i] == t[i] and t[i - 1] == t[i - 1] and t[i] <= t[i - 1])
    total_nan = 0
    for name, column in table.raw_columns.items():
        total_nan += sum(1 for v in column if v != v)
    if nan_count == 0 and non_monotonic <= 1 and total_nan == 0:
        return [Finding(
            rule="R08", severity="INFO", title="时间轴与数值列完整",
            detail=f"共 {table.n} 个样本，无 NaN，时间戳单调递增。",
            evidence=[Evidence(table.path)], confidence="HIGH",
            status=STATUS_CONFIRMED, tags=["data", "pass"])]
    problems = []
    if nan_count:
        problems.append(f"时间列有 {nan_count} 个 NaN")
    if non_monotonic > 1:
        problems.append(f"{non_monotonic} 处时间戳未递增")
    if total_nan:
        problems.append(f"数值列共 {total_nan} 个 NaN")
    return [Finding(
        rule="R08", severity="MEDIUM" if non_monotonic > 1 else "LOW",
        title="日志数据质量存在问题：" + "；".join(problems),
        detail=("存在无效行会让积分与频率统计失真。规则已跳过这些样本，"
                "但仍建议修正导出脚本后重跑，否则结论只能算“尽力而为”。"),
        evidence=[Evidence(table.path)], confidence="HIGH",
        status=STATUS_SUSPECTED, tags=["data"],
        metrics={"nan_time": nan_count, "non_monotonic": non_monotonic, "nan_values": total_nan})]


def run_runtime(table: LogTable, expect_distance: float | None = None,
                expect_yaw: float | None = None,
                scale_tol: float = SCALE_TOL) -> list[Finding]:
    findings: list[Finding] = []
    findings.extend(rule_time_sanity(table))
    findings.extend(rule_linear_scale(table, expect_distance, scale_tol))
    findings.extend(rule_angular_scale(table, expect_yaw, scale_tol))
    findings.extend(rule_dt_jitter(table))
    findings.extend(rule_latency(table))
    findings.extend(rule_saturation(table))
    findings.extend(rule_oscillation(table))
    findings.extend(rule_stuck(table))
    return findings
