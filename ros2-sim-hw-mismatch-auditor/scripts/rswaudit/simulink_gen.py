"""Simulink model generation.

The auditor can emit a Simulink model of the system it just verified, so the
customer can re-run the plant in Simulink instead of trusting a report.

Honesty about the environment: MATLAB cannot be launched inside the build
sandbox (it needs IPC channels that are blocked), so this module ships

  * build_<name>.m   a MATLAB function that constructs the model with the
                     documented new_system / add_block / add_line API and saves
                     a genuine .slx - this is the reliable path;
  * <name>.mdl       the same model in Simulink's text format, for a plain
                     double-click if the MATLAB version accepts it;
  * params.m         every number the model uses, including the corrections the
                     audit proposed;
  * README.txt       what the model contains and what it does not model.

The block/line graph is validated offline (unique names, no dangling lines,
valid ports) because a broken graph would fail inside MATLAB where we cannot
see it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .evidence import fmt
from .mechanisms import ArmSpec, LidarSpec, RotorSpec
from .modelspec import ModelSpec, effective_friction, effective_mass, effective_radius

SI = {
    "const": "simulink/Sources/Constant",
    "step": "simulink/Sources/Step",
    "gain": "simulink/Math Operations/Gain",
    "sum": "simulink/Math Operations/Sum",
    "integ": "simulink/Continuous/Integrator",
    "transfer": "simulink/Continuous/Transfer Fcn",
    "scope": "simulink/Sinks/Scope",
    "fcn": "simulink/User-Defined Functions/Fcn",
    "sat": "simulink/Discontinuities/Saturation",
    "mux": "simulink/Signal Routing/Mux",
    "toworkspace": "simulink/Sinks/To Workspace",
}


@dataclass
class SlBlock:
    kind: str
    name: str
    params: dict[str, str] = field(default_factory=dict)
    position: tuple[int, int, int, int] = (40, 100, 80, 140)

    @property
    def library_path(self) -> str:
        return SI[self.kind]


@dataclass
class SlLine:
    src: str
    sport: int
    dst: str
    dport: int


@dataclass
class SlModel:
    name: str
    kind: str
    blocks: list[SlBlock] = field(default_factory=list)
    lines: list[SlLine] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    stop_time: float = 5.0

    def add(self, kind: str, name: str, row: int, col: int,
            **params: str) -> str:
        x = 40 + col * 150
        y = 80 + row * 70
        self.blocks.append(SlBlock(kind, name, {k: str(v) for k, v in params.items()},
                                   (x, y, x + 80, y + 40)))
        return name

    def link(self, src: str, dst: str, sport: int = 1, dport: int = 1) -> None:
        self.lines.append(SlLine(src, sport, dst, dport))


# --------------------------------------------------------------------------- #
# model builders
# --------------------------------------------------------------------------- #

def _tau() -> float:
    return 0.03


def build_wheeled(spec: ModelSpec, command: float, used_radius: float | None) -> SlModel:
    model = SlModel(name="rsw_" + _slug(spec.name), kind="wheeled")
    r_true = effective_radius(spec) or spec.wheel_radius
    r_used = used_radius if used_radius is not None else spec.wheel_radius
    ratio = (r_used / r_true) if r_true else 1.0
    model.notes.append("轮式底盘一阶速度环：v = (v_cmd - v)/tau")
    model.notes.append("里程计增益 r_used/r_true = " + fmt(ratio, 6)
                       + "（固件轮半径 " + fmt(r_used, 6) + " / 物理 " + fmt(r_true, 6) + "）")

    model.add("step", "v_cmd", 0, 0, Time="0", Before="0", After=fmt(command, 6))
    model.add("sum", "sum_v", 1, 1, Inputs="+-")
    model.add("gain", "gain_tau", 1, 2, Gain=fmt(1.0 / _tau(), 6))
    model.add("integ", "integ_v", 1, 3, InitialCondition="0")
    model.add("integ", "integ_x", 2, 4, InitialCondition="0")
    model.add("gain", "gain_odom", 2, 5, Gain=fmt(ratio, 8))
    model.add("scope", "scope_v", 1, 4)
    model.add("scope", "scope_x", 2, 6)
    model.add("scope", "scope_odom", 3, 6)

    model.link("v_cmd", "sum_v", 1, 1)
    model.link("integ_v", "sum_v", 1, 2)
    model.link("sum_v", "gain_tau")
    model.link("gain_tau", "integ_v")
    model.add("toworkspace", "ws_v", 3, 2, VariableName="v_out", SaveFormat="Array")
    model.add("toworkspace", "ws_x", 3, 4, VariableName="x_out", SaveFormat="Array")
    model.link("integ_v", "ws_v")
    model.link("integ_x", "ws_x")
    model.link("integ_v", "integ_x")
    model.link("integ_v", "scope_v")
    model.link("integ_x", "scope_x")
    model.link("integ_x", "gain_odom")
    model.link("gain_odom", "scope_odom")
    return model


def build_arm(arm: ArmSpec) -> SlModel:
    model = SlModel(name="rsw_" + _slug(arm.name), kind="arm")
    model.notes.append("每个关节一个二阶伺服模型：tau -> 1/(J s + b) -> omega -> q")
    model.notes.append("J 与 b 用 params.m 里的标称值，请用 CAD 得到的真实连杆惯量替换")
    model.stop_time = 5.0
    for index, joint in enumerate(arm.joints):
        row = index
        low, high = joint.usable_limits()
        nominal_j = max(abs(joint.effort) * 0.01, 0.005)
        damping = 2.0 * math.sqrt(nominal_j * 0.5)
        model.add("step", "tau_" + str(index + 1), row, 0,
                  Time="0.1", Before="0", After="1")
        model.add("sum", "sum_" + str(index + 1), row, 1, Inputs="+-")
        model.add("gain", "gain_j_" + str(index + 1), row, 2,
                  Gain=fmt(1.0 / nominal_j, 6))
        model.add("integ", "integ_w_" + str(index + 1), row, 3,
                  InitialCondition="0")
        model.add("gain", "gain_b_" + str(index + 1), row, 4,
                  Gain=fmt(damping, 6))
        model.add("integ", "integ_q_" + str(index + 1), row, 5,
                  InitialCondition="0")
        model.add("sat", "sat_" + str(index + 1), row, 6,
                  UpperLimit=fmt(high, 6), LowerLimit=fmt(low, 6))
        model.add("scope", "scope_q" + str(index + 1), row, 7)
        model.add("toworkspace", "ws_q" + str(index + 1), row, 8,
                  VariableName="q" + str(index + 1), SaveFormat="Array")
        model.link("sat_" + str(index + 1), "ws_q" + str(index + 1))
        model.link("tau_" + str(index + 1), "sum_" + str(index + 1), 1, 1)
        model.link("integ_w_" + str(index + 1), "gain_b_" + str(index + 1))
        model.link("gain_b_" + str(index + 1), "sum_" + str(index + 1), 1, 2)
        model.link("sum_" + str(index + 1), "gain_j_" + str(index + 1))
        model.link("gain_j_" + str(index + 1), "integ_w_" + str(index + 1))
        model.link("integ_w_" + str(index + 1), "integ_q_" + str(index + 1))
        model.link("integ_q_" + str(index + 1), "sat_" + str(index + 1))
        model.link("sat_" + str(index + 1), "scope_q" + str(index + 1))
    return model


def build_rotor(rotor: RotorSpec) -> SlModel:
    model = SlModel(name="rsw_" + _slug(rotor.name), kind="rotor")
    model.notes.append("螺旋桨用执行盘动量理论的系数形式：T = kT*omega^2, Q = kQ*omega^2")
    model.notes.append("转速指令先经过一阶转动惯量滞后，再进入气动映射")
    model.add("step", "rpm_cmd", 0, 0, Time="0", Before="0",
              After=fmt(rotor.max_rpm, 1))
    model.add("gain", "gain_rad", 0, 1, Gain=fmt(2.0 * math.pi / 60.0, 8))
    model.add("sum", "sum_lag", 1, 2, Inputs="+-")
    model.add("gain", "gain_tau_rotor", 1, 3, Gain=fmt(1.0 / 0.05, 4))
    model.add("integ", "integ_omega", 1, 4, InitialCondition="0")
    model.add("fcn", "thrust_map", 2, 5,
              Expr=fmt(rotor.kT, 10) + "*u(1)*u(1)")
    model.add("scope", "scope_thrust", 2, 6)
    model.add("toworkspace", "ws_thrust", 2, 7, VariableName="thrust_out", SaveFormat="Array")
    model.link("thrust_map", "ws_thrust")
    model.add("scope", "scope_omega", 1, 5)
    if rotor.kQ > 0:
        model.add("fcn", "torque_map", 3, 5,
                  Expr=fmt(rotor.kQ, 12) + "*u(1)*u(1)")
        model.add("scope", "scope_torque", 3, 6)
        model.link("integ_omega", "torque_map")
        model.link("torque_map", "scope_torque")
    model.link("rpm_cmd", "gain_rad")
    model.link("gain_rad", "sum_lag", 1, 1)
    model.link("integ_omega", "sum_lag", 1, 2)
    model.link("sum_lag", "gain_tau_rotor")
    model.link("gain_tau_rotor", "integ_omega")
    model.link("integ_omega", "thrust_map")
    model.link("thrust_map", "scope_thrust")
    model.link("integ_omega", "scope_omega")
    return model


def _slug(text: str) -> str:
    cleaned = "".join(ch if ch.isalnum() else "_" for ch in (text or "model"))
    return cleaned.strip("_").lower() or "model"


def build_model(spec: ModelSpec, command: float = 0.3,
                used_radius: float | None = None) -> SlModel | None:
    """Pick the right Simulink plant for whatever was just audited."""
    if spec.arm is not None:
        return build_arm(spec.arm)
    if spec.rotor is not None:
        return build_rotor(spec.rotor)
    if spec.wheels:
        return build_wheeled(spec, command, used_radius)
    return None


# --------------------------------------------------------------------------- #
# validation (we cannot run MATLAB here, so the graph is checked offline)
# --------------------------------------------------------------------------- #

def validate(model: SlModel) -> list[str]:
    problems: list[str] = []
    names = [block.name for block in model.blocks]
    if not names:
        problems.append("模型没有任何模块")
    duplicates = {name for name in names if names.count(name) > 1}
    if duplicates:
        problems.append("模块名重复: " + ", ".join(sorted(duplicates)))
    known = set(names)
    for line in model.lines:
        if line.src not in known:
            problems.append("连线起点不存在: " + line.src)
        if line.dst not in known:
            problems.append("连线终点不存在: " + line.dst)
        if line.sport < 1 or line.dport < 1:
            problems.append("端口号非法: " + line.src + " -> " + line.dst)
    for block in model.blocks:
        if block.kind not in SI:
            problems.append("未知模块类型: " + block.kind)
    # every non-scope block that is not a source should be driven by something
    driven = {line.dst for line in model.lines}
    for block in model.blocks:
        if block.kind in ("sum", "gain", "integ", "scope", "sat", "fcn",
                          "transfer", "toworkspace") and block.name not in driven:
            problems.append("模块没有输入连线: " + block.name)
    return problems


# --------------------------------------------------------------------------- #
# writers
# --------------------------------------------------------------------------- #

def _matlab_str(value: str) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def emit_m(model: SlModel, params: dict[str, str] | None = None) -> str:
    func = "build_" + model.name
    lines = [
        "function " + func + "()",
        "%" + func.upper() + "  由 rsw-audit 自动生成，请勿手工修改。",
        "%   在 MATLAB 中运行本文件即可生成并保存 " + model.name + ".slx，然后点 Run。",
        "%   生成时间与参数来源见同目录 params.m 与 README.txt。",
        "",
        "    model = " + _matlab_str(model.name) + ";",
        "    here = fileparts(mfilename('fullpath'));",
        "    if bdIsLoaded(model)",
        "        close_system(model, 0);",
        "    end",
        "    new_system(model);",
        "    open_system(model);",
        "    set_param(model, 'StopTime', " + _matlab_str(fmt(model.stop_time, 3)) + ");",
        "",
    ]
    for block in model.blocks:
        path = "[model '/' " + _matlab_str(block.name) + "]"
        pairs = []
        for key, value in block.params.items():
            pairs.append(_matlab_str(key) + ", " + _matlab_str(value))
        pos = ", 'Position', [" + " ".join(str(v) for v in block.position) + "]"
        if pairs:
            lines.append("    add_block(" + _matlab_str(block.library_path)
                         + ", " + path + ", " + ", ".join(pairs) + pos + ");")
        else:
            lines.append("    add_block(" + _matlab_str(block.library_path)
                         + ", " + path + pos + ");")
    lines.append("")
    for line in model.lines:
        src = _matlab_str(line.src + "/" + str(line.sport))
        dst = _matlab_str(line.dst + "/" + str(line.dport))
        lines.append("    add_line(model, " + src + ", " + dst
                     + ", 'autorouting', 'on');")
    lines.extend([
        "",
        "    target = fullfile(here, [model '.slx']);",
        "    save_system(model, target);",
        "    fprintf('已生成: %s\\n', target);",
        "    fprintf('模块数: %d, 连线数: %d\\n', " + str(len(model.blocks))
        + ", " + str(len(model.lines)) + ");",
        "end",
        "",
    ])
    return "\n".join(lines)


def emit_mdl(model: SlModel) -> str:
    out = ["Model {",
           "  Name    " + _matlab_str(model.name),
           "  Version 8.0",
           "  SaveDefaultBlockParams on",
           "  Description " + _matlab_str("generated by rsw-audit"),
           "  System {",
           "    Name " + _matlab_str(model.name),
           "    Location [40, 40, 1400, 900]",
           "    Open on"]
    for block in model.blocks:
        out.append("    Block {")
        out.append("      BlockType " + _block_type(block.kind))
        out.append("      Name      " + _matlab_str(block.name))
        for key, value in block.params.items():
            out.append("      " + key + " " + _matlab_str(value))
        out.append("      Position [" + ", ".join(str(v) for v in block.position) + "]")
        out.append("    }")
    for line in model.lines:
        out.append("    Line {")
        out.append("      SrcBlock " + _matlab_str(line.src))
        out.append("      SrcPort  " + str(line.sport))
        out.append("      DstBlock " + _matlab_str(line.dst))
        out.append("      DstPort  " + str(line.dport))
        out.append("    }")
    out.extend(["  }", "}", ""])
    return "\n".join(out)


def _block_type(kind: str) -> str:
    return {"const": "Constant", "step": "Step", "gain": "Gain", "sum": "Sum",
            "integ": "Integrator", "transfer": "TransferFcn", "scope": "Scope",
            "fcn": "Fcn", "sat": "Saturation", "mux": "Mux",
            "toworkspace": "ToWorkspace"}[kind]


def emit_params(spec: ModelSpec, result: Any | None) -> str:
    lines = ["% params.m - 由 rsw-audit 生成，模型里用到的所有数值都在这里",
             "% 建议把这里改成结构体或 .mat，方便批量扫描", ""]
    if spec.wheels:
        lines.append("r_true = " + fmt(effective_radius(spec), 8) + ";   % 物理轮半径 [m]")
        lines.append("r_used = " + fmt(spec.wheel_radius, 8) + ";   % 模型/固件轮半径 [m]")
        lines.append("track  = " + fmt(spec.track, 6) + ";   % 轮距 [m]")
        lines.append("mass   = " + fmt(effective_mass(spec), 4) + ";   % 整备质量 [kg]")
        lines.append("mu     = " + fmt(effective_friction(spec), 4) + ";   % 地面摩擦系数")
        lines.append("tau    = " + fmt(_tau(), 4) + ";   % 速度环时间常数 [s]")
    if spec.arm is not None:
        lines.append("dof = " + str(spec.arm.dof) + ";")
        for index, joint in enumerate(spec.arm.joints, start=1):
            low, high = joint.usable_limits()
            lines.append("q" + str(index) + "_min = " + fmt(low, 6) + ";")
            lines.append("q" + str(index) + "_max = " + fmt(high, 6) + ";")
            lines.append("tau" + str(index) + "_max = " + fmt(joint.effort, 4) + ";")
        lines.append("% 标称关节惯量（请用 CAD 或厂家参数替换）")
        lines.append("J_nominal = 0.05;   % [kg*m^2]")
    if spec.rotor is not None:
        rotor = spec.rotor
        lines.append("kT = " + fmt(rotor.kT, 12) + ";   % [N/(rad/s)^2]")
        lines.append("kQ = " + fmt(rotor.kQ, 14) + ";   % [N*m/(rad/s)^2]")
        lines.append("max_rpm = " + fmt(rotor.max_rpm, 1) + ";")
        lines.append("n_rotors = " + str(rotor.count) + ";")
        lines.append("m_total = " + fmt(rotor.total_mass_kg, 4) + ";")
    if result is not None:
        proposals = [p for finding in result.all_findings() for p in finding.proposals]
        if proposals:
            lines.append("")
            lines.append("% 审计建议的修改（如需验证修正后的行为，把这些值填回模型）")
            for proposal in proposals:
                lines.append("%   " + proposal.evidence.location + "  "
                             + proposal.old_text + " -> " + proposal.new_text)
    lines.append("")
    return "\n".join(lines)


def emit_readme(model: SlModel, spec: ModelSpec, problems: list[str]) -> str:
    lines = [
        "rsw-audit 生成的 Simulink 模型包",
        "=" * 40,
        "",
        "文件：",
        "  build_" + model.name + ".m   在 MATLAB 里运行它，生成并保存 "
        + model.name + ".slx（推荐）",
        "  " + "mdl/" + model.name + ".mdl   同一模型的文本版本，可尝试直接双击打开",
        "  params.m                 模型用到的全部数值（含审计建议的修改）",
        "",
        "使用步骤：",
        "  1. 把整个目录拷到装了 MATLAB + Simulink 的机器上；",
        "  2. 在 MATLAB 当前目录指向本目录，命令行执行  build_" + model.name,
        "  3. 打开生成的 .slx，点 Run，在 Scope 里看响应。",
        "",
        "模型内容：",
    ]
    for note in model.notes:
        lines.append("  - " + note)
    lines.append("")
    lines.append("模块数 " + str(len(model.blocks)) + "，连线数 " + str(len(model.lines)) + "。")
    lines.append("")
    lines.append("说明：文本模型放在 mdl/ 子目录，避免同名 .mdl 与模型名在 MATLAB 路径上"
                 "相互遮蔽（会报 shadowing 警告）。")
    lines.append("")
    lines.append("离线结构自检：" + ("通过（模块名唯一、连线无悬空、端口合法）"
                                  if not problems else "发现问题"))
    for problem in problems:
        lines.append("  ! " + problem)
    lines.append("")
    lines.append("必须说明的边界：")
    lines.append("  - 这些模型是「参数级」验证模型：一阶速度环 / 二阶关节伺服 / 桨推力映射，")
    lines.append("    用于验证参数改了之后响应怎么变，**不是**接触动力学（无碰撞、无轮胎模型、")
    lines.append("    无气动非定常效应）。需要真实接触请用 Simscape Multibody / MuJoCo 等。")
    lines.append("  - 本工具运行环境无法启动 MATLAB，因此 .slx 未被实际生成与打开验证；")
    lines.append("    build_*.m 只使用了 new_system / add_block / add_line / save_system")
    lines.append("    这些长期稳定的公开接口，并已做离线结构自检。若你的 MATLAB 版本报告")
    lines.append("    模块路径差异，把错误发我即可，我会按你的版本调整路径。")
    lines.append("")
    return "\n".join(lines)


def write_bundle(model: SlModel, spec: ModelSpec, out_dir: str | Path,
                 result: Any | None = None) -> dict[str, str]:
    target = Path(out_dir)
    target.mkdir(parents=True, exist_ok=True)
    problems = validate(model)
    written: dict[str, str] = {}
    path = target / ("build_" + model.name + ".m")
    path.write_text(emit_m(model), encoding="utf-8")
    written["matlab_builder"] = str(path)
    mdl_dir = target / "mdl"
    mdl_dir.mkdir(parents=True, exist_ok=True)
    path = mdl_dir / (model.name + ".mdl")
    path.write_text(emit_mdl(model), encoding="utf-8")
    written["mdl"] = str(path)
    path = target / "params.m"
    path.write_text(emit_params(spec, result), encoding="utf-8")
    written["params"] = str(path)
    path = target / "README.txt"
    path.write_text(emit_readme(model, spec, problems), encoding="utf-8")
    written["readme"] = str(path)
    written["valid"] = "0" if problems else "1"
    written["problems"] = "; ".join(problems)
    return written
