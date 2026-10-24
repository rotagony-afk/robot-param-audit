"""Runtime log reader (CSV / key=value / ROS 2 console lines).

The auditor only ever reasons about numbers that were actually present in the
log. Columns that are missing stay missing: the corresponding check reports
"unverifiable" instead of inventing a ground truth.
"""

from __future__ import annotations

import csv
import io
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# role -> accepted column spellings (compared after normalisation)
ROLE_ALIASES: dict[str, list[str]] = {
    "t": ["t", "time", "stamp", "timestamp", "sec", "t_sec", "time_s", "elapsed",
          "ros_time", "sim_time", "wall_time"],
    "cmd_lin": ["cmd_lin", "cmd_linear", "cmd_v", "v_cmd", "linear_cmd", "lin_cmd",
                "cmd_vel_x", "v_ref", "target_lin", "target_v", "cmd_speed",
                "desired_lin", "linear_velocity_cmd"],
    "cmd_ang": ["cmd_ang", "cmd_angular", "w_cmd", "omega_cmd", "angular_cmd",
                "cmd_vel_theta", "w_ref", "target_ang", "desired_ang", "yaw_rate_cmd"],
    "cmd_lat": ["cmd_lat", "cmd_lin_y", "cmd_vel_y", "vy_cmd", "lateral_cmd",
                "strafe_cmd"],
    "odom_lin": ["odom_lin", "odom_linear", "odom_v", "v_odom", "linear_odom",
                 "odom_vel_x", "v_meas", "measured_lin", "v_actual", "speed_meas"],
    "odom_ang": ["odom_ang", "odom_angular", "w_odom", "omega_odom", "odom_vel_theta",
                 "w_meas", "measured_ang", "yaw_rate_meas"],
    "odom_x": ["odom_x", "x_odom", "pos_x"],
    "odom_y": ["odom_y", "y_odom", "pos_y"],
    "odom_yaw": ["odom_yaw", "yaw_odom", "theta_odom", "odom_theta"],
    "truth_lin": ["truth_lin", "true_v", "gt_v", "ground_truth_v", "mocap_lin",
                  "v_true", "reference_v", "ref_v"],
    "truth_x": ["truth_x", "gt_x", "true_x", "mocap_x", "reference_x", "ref_x", "gt_pos_x"],
    "truth_y": ["truth_y", "gt_y", "true_y", "mocap_y", "reference_y", "ref_y", "gt_pos_y"],
    "truth_yaw": ["truth_yaw", "gt_yaw", "true_yaw", "mocap_yaw", "reference_yaw",
                  "ref_yaw", "gt_theta"],
    "left_enc": ["left_enc", "enc_left", "left_ticks", "tick_l", "enc_l", "left_encoder"],
    "right_enc": ["right_enc", "enc_right", "right_ticks", "tick_r", "enc_r", "right_encoder"],
    "pwm": ["pwm", "duty", "u", "ctrl_out", "motor_cmd"],
    "current": ["current", "curr", "amps", "i_motor"],
    "dt": ["dt", "period", "loop_dt", "cycle_time", "step"],
}

_ROLE_INDEX: dict[str, str] = {}
for _role, _names in ROLE_ALIASES.items():
    for _name in _names:
        _ROLE_INDEX[_name] = _role

_NAN = float("nan")
_DELIMS = [",", ";", "\t", "|"]


def _norm(name: str) -> str:
    return "".join(ch for ch in name.strip().lower() if ch.isalnum() or ch == "_")


def resolve_columns(names: list[str]) -> tuple[dict[str, str], list[str]]:
    roles: dict[str, str] = {}
    unmapped: list[str] = []
    for name in names:
        key = _norm(name)
        role = _ROLE_INDEX.get(key)
        if role is None:
            unmapped.append(name)
        elif role not in roles:
            roles[role] = name
    return roles, unmapped


def _to_float(token: str) -> float:
    tok = (token or "").strip().strip("\"'")
    if not tok:
        return _NAN
    low = tok.lower()
    if low in ("nan", "na", "n/a", "-"):
        return _NAN
    if low in ("inf", "+inf", "infinity"):
        return float("inf")
    if low in ("-inf", "-infinity"):
        return float("-inf")
    try:
        return float(tok)
    except ValueError:
        return _NAN


@dataclass
class LogTable:
    path: str
    fmt: str
    columns: dict[str, list[float]] = field(default_factory=dict)
    raw_columns: dict[str, list[float]] = field(default_factory=dict)
    roles: dict[str, str] = field(default_factory=dict)
    unmapped: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    header_line: int = 1
    synthetic_time: bool = False
    row_lines: list[int] = field(default_factory=list)

    @property
    def n(self) -> int:
        if not self.columns:
            return 0
        return len(next(iter(self.columns.values())))

    def has(self, role: str) -> bool:
        return role in self.columns and bool(self.columns[role])

    def col(self, role: str) -> list[float] | None:
        return self.columns.get(role)

    def time_axis(self) -> tuple[list[float], bool]:
        """Return the time axis and whether it is trustworthy."""
        if self.has("t"):
            return list(self.columns["t"]), not self.synthetic_time
        return [float(i) for i in range(self.n)], False


def _sniff(first_lines: list[str]) -> str:
    for line in first_lines:
        best = ""
        best_count = 0
        for delim in _DELIMS:
            count = line.count(delim)
            if count > best_count:
                best, best_count = delim, count
        if best_count >= 1:
            return "csv"
    return "kv"


def _split_csv(line: str, delim: str) -> list[str]:
    reader = csv.reader(io.StringIO(line), delimiter=delim)
    for row in reader:
        return row
    return []


_KV_RE = re.compile(
    r"([A-Za-z_][\w/]*)\s*[=:]\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|NaN|inf|-inf)")
_STAMP_RE = re.compile(r"^\[?\s*([0-9]+\.[0-9]+)\s*\]?")


def parse_log_text(text: str, relpath: str) -> LogTable:
    lines = text.splitlines()
    content: list[tuple[int, str]] = []
    for lineno, raw in enumerate(lines, start=1):
        stripped = raw.strip()
        if not stripped:
            continue
        if stripped.startswith("#") or stripped.startswith("//"):
            continue
        content.append((lineno, raw))
    table = LogTable(path=relpath, fmt="empty")
    if not content:
        table.warnings.append("日志为空")
        return table

    head = [line for _, line in content[:5]]
    fmt = _sniff(head)
    table.fmt = fmt

    if fmt == "csv":
        header_line, header_raw = content[0]
        table.header_line = header_line
        delim = max(_DELIMS, key=lambda d: header_raw.count(d))
        names = [cell.strip() for cell in _split_csv(header_raw, delim)]
        roles, unmapped = resolve_columns(names)
        table.roles = roles
        table.unmapped = unmapped
        columns = {name: [] for name in names}
        for lineno, raw in content[1:]:
            table.row_lines.append(lineno)
            cells = _split_csv(raw, delim)
            for idx, name in enumerate(names):
                value = cells[idx] if idx < len(cells) else ""
                columns[name].append(_to_float(value))
        table.raw_columns = columns
        for role, name in roles.items():
            table.columns[role] = columns[name]
        if "t" not in roles:
            table.warnings.append("未找到时间列，按行号合成时间轴（频率类指标不可信）")
            table.synthetic_time = True
            table.columns["t"] = [float(i) for i in range(len(next(iter(columns.values()), [])))]
    else:
        rows: list[dict[str, float]] = []
        order: list[str] = []
        for lineno, raw in content:
            found = _KV_RE.findall(raw)
            if not found:
                continue
            row: dict[str, float] = {}
            for key, value in found:
                key = _norm(key)
                if key not in row:
                    row[key] = _to_float(value)
                    if key not in order:
                        order.append(key)
            if "t" not in row:
                stamp = _STAMP_RE.match(raw.strip())
                if stamp:
                    row["t"] = float(stamp.group(1))
                    if "t" not in order:
                        order.insert(0, "t")
            rows.append(row)
            table.row_lines.append(lineno)
        roles, unmapped = resolve_columns(order)
        table.roles = roles
        table.unmapped = [name for name in unmapped][:20]
        columns = {name: [] for name in order}
        for row in rows:
            for name in order:
                columns[name].append(row.get(name, _NAN))
        table.raw_columns = columns
        for role, name in roles.items():
            table.columns[role] = columns[name]
        if "t" not in roles:
            table.warnings.append("未找到时间列，按行号合成时间轴（频率类指标不可信）")
            table.synthetic_time = True
            table.columns["t"] = [float(i) for i in range(len(rows))]
    if table.n < 2:
        table.warnings.append("有效数据点少于 2 行，无法做任何统计")
    return table


def load_log(path: str | Path) -> LogTable:
    full = Path(path)
    text = full.read_text(encoding="utf-8", errors="replace")
    return parse_log_text(text, full.name)


# --------------------------------------------------------------------------- #
# small numeric helpers used by the runtime rules
# --------------------------------------------------------------------------- #

def finite_pairs(xs: list[float], ys: list[float]) -> list[tuple[float, float]]:
    out = []
    for x, y in zip(xs, ys):
        if x == x and y == y and abs(x) != float("inf") and abs(y) != float("inf"):
            out.append((x, y))
    return out


def integrate(t: list[float], y: list[float]) -> tuple[float, int, int]:
    """Trapezoid integral, skipping non-finite or non-positive dt."""
    total = 0.0
    used = 0
    skipped = 0
    for idx in range(1, min(len(t), len(y))):
        dt = t[idx] - t[idx - 1]
        if not (dt > 0) or dt > 5.0 or y[idx] != y[idx] or y[idx - 1] != y[idx - 1]:
            skipped += 1
            continue
        total += 0.5 * (y[idx] + y[idx - 1]) * dt
        used += 1
    return total, used, skipped


def path_length(xs: list[float], ys: list[float]) -> tuple[float, int]:
    total = 0.0
    used = 0
    for idx in range(1, min(len(xs), len(ys))):
        dx = xs[idx] - xs[idx - 1]
        dy = ys[idx] - ys[idx - 1]
        if any(v != v for v in (dx, dy)):
            continue
        seg = math.hypot(dx, dy)
        if seg > 1.0:      # a 1 m jump in one sample is a glitch, not motion
            continue
        total += seg
        used += 1
    return total, used


def unwrap(angles: list[float]) -> list[float]:
    out: list[float] = []
    prev = 0.0
    for idx, ang in enumerate(angles):
        if ang != ang:
            out.append(_NAN)
            continue
        if idx == 0:
            out.append(ang)
            prev = ang
            continue
        delta = ang - prev
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        out.append(out[-1] + delta)
        prev = ang
    return out


def diff_stats(t: list[float]) -> dict[str, Any]:
    diffs = []
    for idx in range(1, len(t)):
        dt = t[idx] - t[idx - 1]
        if dt == dt and dt > 0:
            diffs.append(dt)
    if not diffs:
        return {"n": 0}
    ordered = sorted(diffs)
    median = ordered[len(ordered) // 2]
    mean = sum(diffs) / len(diffs)
    p95 = ordered[int(0.95 * (len(ordered) - 1))]
    return {
        "n": len(diffs),
        "median": median,
        "mean": mean,
        "p95": p95,
        "max": ordered[-1],
        "min": ordered[0],
        "jitter_frac": (p95 - median) / median if median else float("nan"),
    }
