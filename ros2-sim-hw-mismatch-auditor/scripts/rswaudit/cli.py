"""Command line interface.

Exit codes
  0  ran successfully, no CRITICAL/HIGH finding
  1  usage error / unreadable input
  2  ran successfully, at least one CRITICAL/HIGH finding (CI friendly)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Sequence

from . import __tool__, __version__
from .correlate import correlate
from .kinematics import Twist
from .mesh import load_mesh
from .mechanism_rules import run_mechanism_rules
from .model_rules import run_model_rules
from .modelspec import apply_reality, load_model
from .simulink_gen import build_model as build_simulink_model
from .simulink_gen import write_bundle as write_simulink_bundle
from .patcher import apply_patches, build_patches
from .report import (AuditResult, collect_inputs, console_summary, exit_code,
                     write_outputs)
from .runtime_rules import run_runtime
from .scanners_log import load_log, parse_log_text
from .simulate import DEFAULT_DT, Profile, profile_from_table
from .scanners_static import collect_static
from .static_rules import run_static
from .trace import Trace

STATIC_RULES = ["S01", "S02", "S03", "S04", "S05", "S06", "S07", "S08", "S09", "S10"]
RUNTIME_RULES = ["R01", "R02", "R03", "R04", "R05", "R06", "R07", "R08"]


def _rel(root: Path, path: str) -> str:
    try:
        return Path(path).resolve().relative_to(root.resolve()).as_posix()
    except (ValueError, OSError):
        return Path(path).name


def _log_rules(trace: Trace, stage: str, findings: list, rule_ids: Sequence[str]) -> None:
    counts: dict[str, int] = {rule: 0 for rule in rule_ids}
    for finding in findings:
        counts[finding.rule] = counts.get(finding.rule, 0) + 1
    for rule, count in counts.items():
        trace.event(stage, "rule", rule=rule, findings=count)


def run_audit(project: str = ".", log: str | None = None, out: str | None = None,
              apply: bool = False, tolerance: float = 1e-3,
              expect_distance: float | None = None, expect_yaw: float | None = None,
              quiet: bool = False, command: str | None = None,
              scale_tol: float = 0.01) -> tuple[AuditResult, dict[str, str]]:
    root = Path(project).resolve()
    if not root.exists():
        raise FileNotFoundError("工程目录不存在: " + str(root))
    out_dir = Path(out).resolve() if out else Path.cwd() / "audit-out"
    out_dir.mkdir(parents=True, exist_ok=True)
    trace = Trace(out_dir / "trace.jsonl")
    trace.event("init", "start", tool=__tool__, version=__version__,
                project=str(root), out=str(out_dir))
    trace.event("init", "options", tolerance=tolerance, scale_tol=scale_tol,
                expect_distance=expect_distance, expect_yaw=expect_yaw, apply=apply)
    if command:
        trace.event("init", "command", command=command)

    # ---- static ---------------------------------------------------------- #
    inputs = collect_static(root)
    for rel in inputs.files:
        trace.event("static", "scan-file", file=rel, kind=inputs.labels.get(rel, "?"))
    trace.event("static", "scan-done", files=len(inputs.files), facts=len(inputs.facts),
                skipped=len(inputs.skipped))
    for item in inputs.skipped:
        trace.event("static", "skip", reason=item)
    static_findings = run_static(inputs.facts, inputs.robots, inputs.yaml_entries,
                                 inputs.launch_info, rel_tol=tolerance)
    _log_rules(trace, "static", static_findings, STATIC_RULES)

    # ---- runtime --------------------------------------------------------- #
    runtime_findings: list = []
    log_summaries: list[dict[str, Any]] = []
    log_rel = ""
    if log:
        log_path = Path(log)
        if not log_path.is_absolute():
            log_path = (root / log) if (root / log).exists() else Path.cwd() / log
        if not log_path.exists():
            raise FileNotFoundError("日志文件不存在: " + str(log_path))
        table = load_log(log_path)
        log_rel = _rel(root, str(log_path))
        log_summaries.append({
            "path": log_rel, "format": table.fmt, "rows": table.n,
            "roles": sorted(table.roles.keys()), "unmapped": table.unmapped[:20],
            "warnings": table.warnings, "header_line": table.header_line,
            "columns": {name: len(values) for name, values in table.raw_columns.items()},
        })
        trace.event("runtime", "log-loaded", file=log_rel, format=table.fmt,
                    rows=table.n, roles=sorted(table.roles.keys()),
                    unmapped=table.unmapped[:10], warnings=table.warnings)
        runtime_findings = run_runtime(table, expect_distance=expect_distance,
                                       expect_yaw=expect_yaw, scale_tol=scale_tol)
        _log_rules(trace, "runtime", runtime_findings, RUNTIME_RULES)

    # ---- correlation ----------------------------------------------------- #
    correlation_findings = correlate(inputs.facts, inputs.robots,
                                     runtime_findings + static_findings,
                                     rel_tol=tolerance)
    for finding in correlation_findings:
        trace.event("correlate", "linked", rule=finding.rule,
                    severity=finding.severity, status=finding.status,
                    proposals=len(finding.proposals))

    # ---- patches --------------------------------------------------------- #
    all_findings = static_findings + runtime_findings + correlation_findings
    patches = build_patches(all_findings, root)
    for patch in patches:
        trace.event("patch", "built", file=patch.path,
                    proposals=len(patch.proposals), hunks=patch.diff.count("@@"))

    result = AuditResult(
        tool=__tool__, version=__version__, root=str(root),
        command=command or "python scripts/audit.py",
        options={"tolerance": tolerance, "scale_tol": scale_tol,
                 "expect_distance": expect_distance, "expect_yaw": expect_yaw,
                 "apply": apply, "log": log_rel or None},
        inputs=collect_inputs(root, inputs.files, inputs.labels,
                              extra=[log_rel] if log_rel else []),
        static_findings=static_findings,
        runtime_findings=runtime_findings,
        correlation_findings=correlation_findings,
        log_summaries=log_summaries,
        skipped=inputs.skipped,
        patches=patches,
        trace_jsonl=str(trace.path),
    )

    written = write_outputs(result, out_dir, trace.to_markdown(), len(trace.events))
    trace.event("report", "written", **{k: str(v) for k, v in written.items()})
    if result.patch_files:
        trace.event("patch", "files", files=result.patch_files)

    if apply and patches:
        notes = apply_patches(patches, root)
        result.notes.extend("apply: " + note for note in notes)
        trace.event("patch", "applied", notes=notes)
        write_outputs(result, out_dir, trace.to_markdown(), len(trace.events))

    if not quiet:
        print(console_summary(result))
        print("")
        print("报告: " + str(Path(written["report"])))
        print("机器可读结论: " + str(Path(written["findings"])))
        print("调试痕迹: " + str(Path(written["trace_jsonl"])))
        if result.patch_files:
            print("补丁: " + str(Path(written.get("patches", out_dir / "patches"))))
    return result, written


def _resolve(path: str | None, base: Path) -> Path | None:
    if not path:
        return None
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    if (base / candidate).exists():
        return base / candidate
    return Path.cwd() / candidate


def _truth_from_table(table, expect_distance, expect_yaw):
    """Measured distance / yaw to validate the model against."""
    from .scanners_log import integrate, path_length, unwrap
    distance = None
    yaw = None
    if expect_distance is not None:
        distance = abs(expect_distance)
    elif table.has("truth_x") and table.has("truth_y"):
        value, used = path_length(table.col("truth_x"), table.col("truth_y"))
        if used:
            distance = value
    elif table.has("truth_lin"):
        t, _ok = table.time_axis()
        value, used, _skipped = integrate(t, table.col("truth_lin"))
        if used:
            distance = abs(value)
    if expect_yaw is not None:
        yaw = abs(expect_yaw)
    elif table.has("truth_yaw"):
        angles = [a for a in unwrap(table.col("truth_yaw")) if a == a]
        if len(angles) >= 2:
            yaw = abs(angles[-1] - angles[0])
    return distance, yaw


def run_model_audit(model_path: str, project: str | None = None,
                    log: str | None = None, out: str | None = None,
                    reality: dict[str, float] | None = None,
                    mesh_path: str | None = None, density: float | None = None,
                    speed: float = 0.3, yaw_rate: float = 0.0,
                    lateral: float = 0.0, tolerance: float = 1e-3,
                    used_radius: float | None = None,
                    used_track: float | None = None,
                    target: tuple | None = None, payload: float = 0.0,
                    simulink: str | None = None,
                    expect_distance: float | None = None,
                    expect_yaw: float | None = None,
                    quiet: bool = False, command: str | None = None
                    ) -> tuple[AuditResult, dict[str, str]]:
    """Model-driven audit: any chassis description + optional run log."""
    base = Path(project).resolve() if project else Path.cwd()
    out_dir = Path(out).resolve() if out else Path.cwd() / "audit-out"
    out_dir.mkdir(parents=True, exist_ok=True)
    trace = Trace(out_dir / "trace.jsonl")
    trace.event("init", "start", tool=__tool__, version=__version__,
                mode="model", model=str(model_path))
    if command:
        trace.event("init", "command", command=command)

    spec = load_model(model_path)
    trace.event("model", "loaded", file=spec.source, drive=spec.drive,
                wheels=len(spec.wheels), mass=spec.mass,
                notes=spec.notes, unresolved=spec.unresolved)
    applied = apply_reality(spec, reality or {})
    if applied:
        trace.event("model", "reality-applied", keys=applied, values=spec.reality)
    mesh = None
    if mesh_path:
        mesh = load_mesh(mesh_path)
        trace.event("mesh", "loaded", file=mesh.path, triangles=mesh.triangles,
                    volume=mesh.volume, closed=mesh.closed,
                    warnings=mesh.warnings)

    twist = Twist(speed, lateral, yaw_rate)
    trace.event("model", "command-twist", vx=speed, vy=lateral, w=yaw_rate)

    # optional: also audit a project tree, so users can point at a whole package
    static_findings: list = []
    inputs_static = None
    if project:
        inputs_static = collect_static(base)
        for rel in inputs_static.files:
            trace.event("static", "scan-file", file=rel,
                        kind=inputs_static.labels.get(rel, "?"))
        static_findings.extend(run_static(inputs_static.facts, inputs_static.robots,
                                          inputs_static.yaml_entries,
                                          inputs_static.launch_info, rel_tol=tolerance))
        _log_rules(trace, "static", static_findings, STATIC_RULES)

    runtime_findings: list = []
    log_summaries: list[dict] = []
    log_rel = ""
    table = None
    if log:
        log_path = _resolve(log, base)
        if log_path is None or not log_path.exists():
            raise FileNotFoundError("日志文件不存在: " + str(log))
        table = load_log(log_path)
        log_rel = _rel(base, str(log_path))
        log_summaries.append({
            "path": log_rel, "format": table.fmt, "rows": table.n,
            "roles": sorted(table.roles.keys()), "unmapped": table.unmapped[:20],
            "warnings": table.warnings, "header_line": table.header_line,
            "columns": {name: len(values) for name, values in table.raw_columns.items()},
        })
        trace.event("runtime", "log-loaded", file=log_rel, format=table.fmt,
                    rows=table.n, roles=sorted(table.roles.keys()))
        runtime_findings = run_runtime(table, expect_distance=expect_distance,
                                       expect_yaw=expect_yaw)
        _log_rules(trace, "runtime", runtime_findings, RUNTIME_RULES)

    profile = None
    measured = None
    if table is not None and (table.has("cmd_lin") or table.has("cmd_ang")
                             or table.has("cmd_lat")):
        profile = profile_from_table(table)
        distance, yaw = _truth_from_table(table, expect_distance, expect_yaw)
        measured = {}
        if distance is not None:
            measured["distance"] = distance
        if yaw is not None:
            measured["yaw"] = yaw
        measured = measured or None
        if measured:
            trace.event("model", "measured", **{k: v for k, v in measured.items()})

    measured_odom = None
    measured_odom_yaw = None
    if table is not None:
        from .scanners_log import integrate, unwrap
        times, _ok = table.time_axis()
        if table.has("odom_lin"):
            value, used, _skipped = integrate(times, table.col("odom_lin"))
            if used:
                measured_odom = abs(value)
        if table.has("odom_yaw"):
            angles = [a for a in unwrap(table.col("odom_yaw")) if a == a]
            if len(angles) >= 2:
                measured_odom_yaw = abs(angles[-1] - angles[0])
        elif table.has("odom_ang"):
            value, used, _skipped = integrate(times, table.col("odom_ang"))
            if used:
                measured_odom_yaw = abs(value)
        trace.event("model", "measured-odom", distance=measured_odom or 0.0,
                    yaw=measured_odom_yaw or 0.0)

    model_findings, extra = run_model_rules(
        spec, twist=twist, mesh=mesh, density=density, profile=profile,
        measured=measured, dt=DEFAULT_DT, measured_odom=measured_odom,
        measured_odom_yaw=measured_odom_yaw, radius_used=used_radius,
        track_used=used_track)
    trace.event("model", "capability", **extra.get("capability", {}))
    for finding in model_findings:
        trace.event("model", "rule", rule=finding.rule, severity=finding.severity,
                    status=finding.status)

    # arms / rotors / lidars are checked by their own rule family
    mech_findings, mech_extra = run_mechanism_rules(
        spec.arm, spec.rotor, spec.lidar, target=target, payload_kg=payload,
        source=spec.source)
    model_findings.extend(mech_findings)
    for kind, values in mech_extra.items():
        summary = {k: v for k, v in values.items()
                   if isinstance(v, (int, float, str, bool))}
        trace.event("mechanism", "checked", kind=kind, **summary)
    for finding in mech_findings:
        trace.event("mechanism", "rule", rule=finding.rule,
                    severity=finding.severity, status=finding.status)

    # M09/M10 are the "verify by simulation" conclusions
    runtime_rule_ids = {"M09", "M10"}
    mechanism_kinds = [name for name, value in (("arm", spec.arm),
                                                ("rotor", spec.rotor),
                                                ("lidar", spec.lidar)) if value]
    static_out = [f for f in model_findings if f.rule not in runtime_rule_ids]
    correlation_out = [f for f in model_findings if f.rule in runtime_rule_ids]

    result = AuditResult(
        tool=__tool__, version=__version__, root=str(base),
        command=command or ("python scripts/audit.py model --model " + str(model_path)),
        options={"mode": "model", "model": spec.source, "drive": spec.drive,
                 "twist": {"vx": speed, "vy": lateral, "w": yaw_rate},
                 "reality": dict(spec.reality), "density": density,
                 "tolerance": tolerance},
        inputs=collect_inputs(base, (inputs_static.files if inputs_static else []),
                              inputs_static.labels if inputs_static else {},
                              extra=[rel for rel in
                                     [spec.source, log_rel,
                                      Path(mesh_path).name if mesh_path else None]
                                     if rel]),
        static_findings=static_out + static_findings,
        runtime_findings=runtime_findings,
        correlation_findings=correlation_out,
        log_summaries=log_summaries,
        skipped=(inputs_static.skipped if inputs_static else []),
        trace_jsonl=str(trace.path),
        notes=list(spec.notes) + list(spec.unresolved),
    )
    if spec.wheels:
        result.notes.append("底盘类型: " + spec.drive + "；轮子: "
                            + ", ".join(w.name for w in spec.wheels))
    if mechanism_kinds:
        result.notes.append("本次检查的机构: " + ", ".join(mechanism_kinds))
    result.notes.append("整备质量 %.3f kg，偏航惯量 %.4f kg*m^2，轮半径 %.4f m"
                        % (spec.mass, spec.inertia_zz, spec.wheel_radius))
    if table is None:
        result.notes.append("本次没有运行日志，M09/M10（仿真对比实测）未执行")

    patches = build_patches(result.all_findings(), base)
    result.patches = patches
    if patches:
        trace.event("patch", "built", files=[p.path for p in patches])

    if simulink:
        sl_model = build_simulink_model(spec, command=speed, used_radius=used_radius)
        if sl_model is None:
            result.notes.append("该模型没有可仿真的动态（只有传感器规格），"
                                "未生成 Simulink 文件")
            trace.event("simulink", "skipped", reason="no-dynamics")
        else:
            bundle = write_simulink_bundle(sl_model, spec, simulink, result)
            trace.event("simulink", "written", **bundle)
            result.notes.append(
                "Simulink: " + bundle["matlab_builder"]
                + "（在 MATLAB 里运行它即可生成 " + sl_model.name + ".slx）")
            if bundle["valid"] != "1":
                result.notes.append("Simulink 离线结构自检发现问题: "
                                    + bundle["problems"])
    written = write_outputs(result, out_dir, trace.to_markdown(), len(trace.events))
    trace.event("report", "written", **{k: str(v) for k, v in written.items()})
    if not quiet:
        print(console_summary(result))
        print("")
        for key, value in written.items():
            print(key + ": " + str(value))
    return result, written


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="audit.py",
        description="机器人仿真实机参数错配诊断器：比对 URDF/YAML/launch/源码，"
                    "并结合运行日志定位参数错配根因。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python scripts/audit.py static --project ./demo_robot\n"
               "  python scripts/audit.py full --project ./demo_robot "
               "--log ./demo_robot/logs/run_straight.csv\n",
    )
    parser.add_argument("--version", action="version", version=__tool__ + " " + __version__)
    sub = parser.add_subparsers(dest="command")

    def add_common(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("--project", "-p", default=".", help="ROS 2 工程根目录（默认当前目录）")
        sp.add_argument("--log", "-l", default=None, help="运行日志文件（CSV 或 key=value 文本）")
        sp.add_argument("--out", "-o", default=None, help="产物输出目录（默认 ./audit-out）")
        sp.add_argument("--tolerance", type=float, default=1e-3,
                        help="静态数值相等判定容差，默认 1e-3（相对值）")
        sp.add_argument("--scale-tolerance", type=float, default=0.01,
                        help="运行尺度偏差容差，默认 0.01（1%%）")
        sp.add_argument("--expect-distance", type=float, default=None,
                        help="实测直线位移（米），当日志没有真值列时使用")
        sp.add_argument("--expect-yaw", type=float, default=None,
                        help="实测转角（弧度），当日志没有真值列时使用")
        sp.add_argument("--apply", action="store_true",
                        help="把高置信度建议直接写入文件（会生成 .bak 备份）")
        sp.add_argument("--quiet", "-q", action="store_true", help="只写文件，不打印摘要")

    add_common(sub.add_parser("static", help="只做静态一致性检查"))
    add_common(sub.add_parser("runtime", help="只做运行日志诊断"))
    add_common(sub.add_parser("full", help="静态 + 运行 + 根因关联（推荐）"))

    model_sp = sub.add_parser("model", help="按模型规格做通用验证（差速/阿克曼/麦轮/全向/滑移）")
    model_sp.add_argument("--model", "-m", required=True,
                          help="URDF/Xacro、MJCF、SDF 或 JSON 模型文件")
    model_sp.add_argument("--reality", "-r", default=None,
                          help="实测参数：JSON/YAML 文件，或 wheel_radius=0.0508,mass=12")
    model_sp.add_argument("--mesh", default=None, help="3D 网格（.stl/.obj），反推质量与惯量")
    model_sp.add_argument("--density", type=float, default=None,
                          help="网格材料密度 kg/m3（默认 1000）")
    model_sp.add_argument("--project", "-p", default=None,
                          help="同时审计一个工程目录（可选）")
    model_sp.add_argument("--log", "-l", default=None, help="实测日志（用于仿真对比与拟合）")
    model_sp.add_argument("--speed", type=float, default=0.3, help="待验证的命令线速度 m/s")
    model_sp.add_argument("--yaw-rate", type=float, default=0.0, help="待验证的命令角速度 rad/s")
    model_sp.add_argument("--lateral", type=float, default=0.0,
                          help="待验证的命令横向速度 m/s（麦轮/全向）")
    model_sp.add_argument("--out", "-o", default=None)
    model_sp.add_argument("--tolerance", type=float, default=1e-3)
    model_sp.add_argument("--expect-distance", type=float, default=None)
    model_sp.add_argument("--expect-yaw", type=float, default=None)
    model_sp.add_argument("--used-radius", type=float, default=None,
                          help="固件/里程计里实际使用的轮半径（默认取模型值）")
    model_sp.add_argument("--used-track", type=float, default=None,
                          help="固件/里程计里实际使用的轮距（默认取模型值）")
    model_sp.add_argument("--target", default=None,
                          help="机械臂末端目标点 x,y,z（米），默认取可达空间内一点")
    model_sp.add_argument("--payload", type=float, default=0.0,
                          help="机械臂末端负载（kg），用于静态力矩校验")
    model_sp.add_argument("--simulink", default=None,
                          help="导出 Simulink 模型包到该目录（.m 构建脚本 + .mdl + params.m）")
    model_sp.add_argument("--quiet", "-q", action="store_true")

    app_sp = sub.add_parser("app", help="启动本地 Web App（零依赖）")
    app_sp.add_argument("--port", type=int, default=8765)
    app_sp.add_argument("--host", default="127.0.0.1")
    app_sp.add_argument("--no-browser", action="store_true")

    add_common(parser)
    return parser


def _compose_command(args: argparse.Namespace) -> str:
    parts = ["python scripts/audit.py", args.command or "full"]
    if args.project:
        parts.append("--project " + args.project)
    if args.log:
        parts.append("--log " + args.log)
    if args.apply:
        parts.append("--apply")
    if args.expect_distance is not None:
        parts.append("--expect-distance " + str(args.expect_distance))
    if args.expect_yaw is not None:
        parts.append("--expect-yaw " + str(args.expect_yaw))
    return " ".join(parts)


def parse_reality(value: str) -> dict[str, float]:
    """Read measured parameters from a JSON/YAML file or an inline list."""
    path = Path(value)
    if path.exists():
        text = path.read_text(encoding="utf-8", errors="replace")
        if path.suffix.lower() == ".json":
            data = json.loads(text)
            return {str(k): float(v) for k, v in data.items()
                    if isinstance(v, (int, float))}
        from .scanners_static import parse_number, scan_yaml
        result: dict[str, float] = {}
        for entry in scan_yaml(text, path.name):
            number = parse_number(entry.literal)
            if number is not None:
                result[entry.key] = number
        return result
    result: dict[str, float] = {}
    for chunk in value.replace(";", ",").split(","):
        if "=" not in chunk:
            continue
        key, _, raw = chunk.partition("=")
        try:
            result[key.strip()] = float(raw)
        except ValueError:
            continue
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    command = args.command or "full"
    if command == "runtime" and not args.log:
        parser.error("runtime 子命令必须提供 --log")
    if command == "app":
        from .app import serve
        return serve(host=args.host, port=args.port,
                     open_browser=not args.no_browser)
    if command == "model":
        try:
            reality = parse_reality(args.reality) if args.reality else {}
            density = args.density
            if density is None and args.mesh:
                density = 1000.0
            target = None
            if args.target:
                parts = [float(v) for v in str(args.target).replace(";", ",").split(",")
                         if v.strip()]
                if len(parts) == 3:
                    target = tuple(parts)
            cmd = ("python scripts/audit.py model --model " + args.model
                   + (" --log " + args.log if args.log else "")
                   + (" --mesh " + args.mesh if args.mesh else "")
                   + (" --simulink " + args.simulink if args.simulink else ""))
            result, _written = run_model_audit(
                model_path=args.model, project=args.project, log=args.log,
                out=args.out, reality=reality, mesh_path=args.mesh,
                density=density, speed=args.speed, yaw_rate=args.yaw_rate,
                lateral=args.lateral, tolerance=args.tolerance,
                expect_distance=args.expect_distance, expect_yaw=args.expect_yaw,
                used_radius=args.used_radius, used_track=args.used_track,
                target=target, payload=args.payload, simulink=args.simulink,
                quiet=args.quiet, command=cmd)
        except (FileNotFoundError, ValueError) as exc:
            print("错误: " + str(exc), file=sys.stderr)
            return 1
        return exit_code(result)
    try:
        result, _written = run_audit(
            project=args.project, log=args.log, out=args.out, apply=args.apply,
            tolerance=args.tolerance, expect_distance=args.expect_distance,
            expect_yaw=args.expect_yaw, quiet=args.quiet,
            command=_compose_command(args), scale_tol=args.scale_tolerance,
        )
    except FileNotFoundError as exc:
        print("错误: " + str(exc), file=sys.stderr)
        return 1
    return exit_code(result)


if __name__ == "__main__":
    raise SystemExit(main())
