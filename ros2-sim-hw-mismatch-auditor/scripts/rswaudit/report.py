"""Report rendering: human Markdown, machine JSON, trace Markdown, patches.

The report is intentionally blunt about what was *not* verified. Every section
that says "已确认" is backed by a file/line citation or a numeric measurement;
everything else is collected in the "未验证 / 无法判定" section.

NOTE ON FORMATTING: this module writes Markdown by hand. The literal backtick
character is produced through the BT constant so that the source file stays
readable inside tools that treat backticks as delimiters.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from .evidence import Finding, fmt, sev_rank
from .patcher import FilePatch, write_patches

BT = chr(96)
FENCE = BT * 3

SEV_BADGE = {
    "CRITICAL": "[严重]",
    "HIGH": "[高]",
    "MEDIUM": "[中]",
    "LOW": "[低]",
    "INFO": "[信息]",
}
STATUS_LABEL = {
    "confirmed": "已确认",
    "suspected": "待确认",
    "unverifiable": "无法判定",
}


def code(text: str) -> str:
    return BT + str(text) + BT


def fence(text: str) -> str:
    return "\n" + FENCE + "\n" + text + "\n" + FENCE + "\n"


@dataclass
class InputFile:
    path: str
    kind: str
    size: int
    sha256: str


@dataclass
class AuditResult:
    tool: str
    version: str
    root: str
    command: str
    generated_at: str = field(default_factory=lambda: datetime.now(timezone.utc)
                              .astimezone().strftime("%Y-%m-%d %H:%M:%S %z"))
    options: dict[str, Any] = field(default_factory=dict)
    inputs: list[InputFile] = field(default_factory=list)
    static_findings: list[Finding] = field(default_factory=list)
    runtime_findings: list[Finding] = field(default_factory=list)
    correlation_findings: list[Finding] = field(default_factory=list)
    log_summaries: list[dict[str, Any]] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    patches: list[FilePatch] = field(default_factory=list)
    patch_files: list[str] = field(default_factory=list)
    trace_jsonl: str = ""
    trace_md: str = ""
    notes: list[str] = field(default_factory=list)

    def all_findings(self) -> list[Finding]:
        everything = (list(self.static_findings) + list(self.runtime_findings)
                      + list(self.correlation_findings))
        return sorted(everything, key=lambda f: (sev_rank(f.severity), f.rule))

    def counts(self) -> dict[str, int]:
        counts = {"CRITICAL": 0, "HIGH": 0, "MEDIUM": 0, "LOW": 0, "INFO": 0}
        for finding in self.all_findings():
            counts[finding.severity] = counts.get(finding.severity, 0) + 1
        return counts

    def blocking(self) -> list[Finding]:
        return [f for f in self.all_findings() if f.severity in ("CRITICAL", "HIGH")]

    def boundaries(self) -> list[str]:
        notes = [
            "只判断“文件里写了什么”，不判断“实物到底是什么”。轮子真实直径、电机实际转速仍需人工测量。",
            "没有真值（truth 列或 --expect-distance / --expect-yaw）时，绝对尺度一律判为无法判定，不会给出偏差百分比。",
            "表达式参数（xacro / launch 变量替换）不参与数值比对，最多在 S03 中列出。",
            "不支持 rosbag2 的 CDR 反序列化；如需分析 bag，请先导出为 CSV 或逐行文本。",
            "工具不会自动改代码；--apply 只在显式传入时生效，并生成 .bak 备份。",
        ]
        if not any(item.kind == "log" for item in self.inputs):
            notes.append("本次没有输入任何运行日志，R 类规则全部未执行。")
        return notes


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as fh:
            for chunk in iter(lambda: fh.read(65536), b""):
                digest.update(chunk)
    except OSError:
        return "unreadable"
    return digest.hexdigest()[:16]


def collect_inputs(root: str | Path, files: Iterable[str], kinds: dict[str, str],
                   extra: Iterable[str] = ()) -> list[InputFile]:
    root_path = Path(root)
    out: list[InputFile] = []
    seen: set[str] = set()
    for rel in list(files) + list(extra):
        if rel in seen:
            continue
        seen.add(rel)
        full = root_path / rel
        if not full.exists():
            full = Path(rel)
        size = full.stat().st_size if full.exists() else 0
        out.append(InputFile(rel, kinds.get(rel, "log"), size, sha256_of(full)))
    return out


def _evidence_block(evidence: list) -> str:
    if not evidence:
        return ""
    chunks = ["**证据**", ""]
    for item in evidence:
        head = "- " + code(item.location)
        if item.locator:
            head += "  (" + item.locator + ")"
        chunks.append(head)
        if item.excerpt:
            chunks.append("")
            chunks.append("      " + item.excerpt)
            chunks.append("")
    return "\n".join(chunks) + "\n"


def _metrics_block(metrics: dict[str, Any]) -> str:
    if not metrics:
        return ""
    rows = []
    for key, value in metrics.items():
        if isinstance(value, float):
            text = fmt(value, 6)
        elif isinstance(value, (list, dict)):
            text = json.dumps(value, ensure_ascii=False)
        else:
            text = str(value)
        rows.append("| " + key + " | " + text.replace("|", "\\|") + " |")
    return "**指标**\n\n| 名称 | 值 |\n|---|---|\n" + "\n".join(rows) + "\n"


def render_finding(finding: Finding, heading: str = "###") -> str:
    badge = SEV_BADGE.get(finding.severity, finding.severity)
    status = STATUS_LABEL.get(finding.status, finding.status)
    parts = [heading + " " + badge + " " + finding.rule + " · " + finding.title, ""]
    line = "置信度 **" + finding.confidence + "** · 状态 **" + status + "**"
    if finding.tags:
        line += " · 标签 " + ", ".join(finding.tags)
    parts.extend([line, "", finding.detail, ""])
    block = _evidence_block(finding.evidence)
    if block:
        parts.append(block)
    metrics = _metrics_block(finding.metrics)
    if metrics:
        parts.append(metrics)
    if finding.proposals:
        parts.append("**建议修改（默认不写入磁盘）**")
        parts.append("")
        for proposal in finding.proposals:
            parts.append("- " + code(proposal.evidence.location) + "  " + code(proposal.old_text)
                         + " -> " + code(proposal.new_text) + "  — " + proposal.reason)
        parts.append("")
    return "\n".join(parts).rstrip() + "\n"


def _table(header: list[str], rows: list[list[str]]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    for row in rows:
        cells = []
        for cell in row:
            cells.append(str(cell).replace("|", "\\|").replace("\n", " "))
        out.append("| " + " | ".join(cells) + " |")
    return "\n".join(out) + "\n"


def render_summary_table(findings: list[Finding]) -> str:
    rows = []
    for finding in findings:
        rows.append([
            finding.rule,
            SEV_BADGE.get(finding.severity, finding.severity),
            finding.confidence,
            STATUS_LABEL.get(finding.status, finding.status),
            finding.title,
            finding.evidence[0].location if finding.evidence else "-",
        ])
    if not rows:
        return "_没有任何发现。_\n"
    return _table(["规则", "级别", "置信度", "状态", "结论", "首个证据位置"], rows)


def render_report(result: AuditResult) -> str:
    counts = result.counts()
    blocking = result.blocking()
    lines: list[str] = []
    lines.append("# 机器人仿真实机参数错配诊断报告")
    lines.append("")
    lines.append(_table(["项目", "值"], [
        ["工具", result.tool + " v" + result.version],
        ["生成时间", result.generated_at],
        ["工程根目录", result.root],
        ["执行命令", code(result.command)],
        ["读取文件数", str(len(result.inputs))],
        ["跳过文件数", str(len(result.skipped))],
    ]))
    lines.append("")

    lines.append("## 0. 结论摘要")
    lines.append("")
    lines.append("| 级别 | 数量 |\n|---|---|")
    for sev in ("CRITICAL", "HIGH", "MEDIUM", "LOW", "INFO"):
        lines.append("| " + SEV_BADGE[sev] + " " + sev + " | " + str(counts.get(sev, 0)) + " |")
    lines.append("")
    if blocking:
        lines.append("**需要先处理的问题（" + str(len(blocking)) + " 项）：**")
        lines.append("")
        lines.append(render_summary_table(blocking))
    else:
        lines.append("没有 CRITICAL / HIGH 级别的问题。")
        lines.append("")

    lines.append("## 1. 输入清单与指纹")
    lines.append("")
    rows = [[code(item.path), item.kind, str(item.size), code(item.sha256)]
            for item in result.inputs]
    if rows:
        lines.append(_table(["文件", "类型", "字节", "sha256(前16)"], rows))
    else:
        lines.append("_没有读到任何文件。_\n")
    if result.skipped:
        lines.append("**跳过的文件 / 解析失败（未参与任何结论）**")
        lines.append("")
        for item in result.skipped:
            lines.append("- " + item)
        lines.append("")
    if result.log_summaries:
        lines.append("### 运行日志概览")
        lines.append("")
        for summary in result.log_summaries:
            roles = ", ".join(summary.get("roles") or []) or "无可识别列"
            lines.append("- " + code(summary["path"]) + " · 格式 " + str(summary["format"])
                         + " · " + str(summary["rows"]) + " 行 · 识别列 " + roles)
            for warning in summary.get("warnings") or []:
                lines.append("  - 警告：" + warning)
        lines.append("")

    lines.append("## 2. 静态一致性检查（S 类）")
    lines.append("")
    static_real = [f for f in result.static_findings if f.status != "unverifiable"]
    if static_real:
        for finding in sorted(static_real, key=lambda f: (sev_rank(f.severity), f.rule)):
            lines.append(render_finding(finding))
    else:
        lines.append("_静态文件之间没有发现数值冲突。_\n")

    lines.append("## 3. 运行数据诊断（R 类）")
    lines.append("")
    runtime_real = [f for f in result.runtime_findings if f.status != "unverifiable"]
    if runtime_real:
        for finding in sorted(runtime_real, key=lambda f: (sev_rank(f.severity), f.rule)):
            lines.append(render_finding(finding))
    else:
        lines.append("_没有提供运行日志，或日志中没有可用的数值结论。_\n")

    lines.append("## 4. 根因关联与修改建议（C 类）")
    lines.append("")
    if result.correlation_findings:
        for finding in sorted(result.correlation_findings,
                             key=lambda f: (sev_rank(f.severity), f.rule)):
            lines.append(render_finding(finding))
    else:
        lines.append("_没有同时拿到“运行偏差”和“静态冲突”，因此没有可收敛的根因。_\n")

    unverified = [f for f in result.all_findings() if f.status == "unverifiable"]
    lines.append("## 5. 未验证 / 无法判定的内容（请勿当成结论）")
    lines.append("")
    if unverified:
        for finding in unverified:
            lines.append(render_finding(finding, heading="####"))
    else:
        lines.append("_本次所有检查项都给出了可判定的结论。_\n")
    lines.append("**工具本身的能力边界**：")
    lines.append("")
    for note in result.boundaries():
        lines.append("- " + note)
    lines.append("")

    passes = [f for f in result.all_findings() if "pass" in f.tags]
    lines.append("## 6. 已确认通过的检查项")
    lines.append("")
    if passes:
        for finding in passes:
            lines.append("- " + finding.rule + " " + finding.title)
    else:
        lines.append("_无。_")
    lines.append("")

    lines.append("## 7. 复现与下一步")
    lines.append("")
    lines.append(fence(
        "# 完整复现（静态 + 运行）\n"
        + result.command + "\n\n"
        "# 只跑静态检查\n"
        "python scripts/audit.py static --project .\n\n"
        "# 确认无误后再落盘修改（会生成 .bak 备份）\n"
        "python scripts/audit.py full --project . --log logs/run_straight.csv --apply"))
    lines.append("")
    if result.patch_files:
        lines.append("**已生成的补丁文件**（人工审阅后可用 git apply 或手工修改）：")
        lines.append("")
        for name in result.patch_files:
            lines.append("- " + code("patches/" + name))
        lines.append("")
    lines.append("**调试痕迹**：" + code(Path(result.trace_jsonl).name)
                 + "（逐条 JSONL，记录阶段、动作、读取的文件与判定理由）、"
                 + code(Path(result.trace_md).name) + "（同一份内容的人类可读版）。")
    lines.append("")
    if result.notes:
        lines.append("**本次运行的补充说明**")
        lines.append("")
        for note in result.notes:
            lines.append("- " + note)
        lines.append("")
    return "\n".join(lines)


def render_trace_markdown(result: AuditResult, trace_markdown: str, events: int = 0) -> str:
    head = ["# 调试痕迹", "",
            "每一行对应工具执行过程中的一个动作，按时间顺序记录，可用于复现同一次判定。", "",
            _table(["项目", "值"], [
                ["生成时间", result.generated_at],
                ["命令", code(result.command)],
                ["事件数", str(events)],
            ]), ""]
    return "\n".join(head) + trace_markdown


def findings_json(result: AuditResult) -> dict[str, Any]:
    return {
        "tool": result.tool,
        "version": result.version,
        "generated_at": result.generated_at,
        "root": result.root,
        "command": result.command,
        "options": result.options,
        "summary": result.counts(),
        "blocking_count": len(result.blocking()),
        "inputs": [
            {"path": item.path, "kind": item.kind, "size": item.size, "sha256": item.sha256}
            for item in result.inputs
        ],
        "skipped": result.skipped,
        "logs": result.log_summaries,
        "findings": [finding.to_dict() for finding in result.all_findings()],
        "patches": [{"path": patch.path, "applied": patch.applied,
                     "backup": patch.backup, "proposals": len(patch.proposals)}
                    for patch in result.patches],
        "boundaries": result.boundaries(),
    }


def write_outputs(result: AuditResult, out_dir: str | Path, trace_markdown: str,
                  events: int = 0) -> dict[str, str]:
    target = Path(out_dir)
    target.mkdir(parents=True, exist_ok=True)
    written: dict[str, str] = {}
    if result.patches:
        result.patch_files = write_patches(result.patches, target)
        written["patches"] = str(target / "patches")
    report_path = target / "report.md"
    report_path.write_text(render_report(result), encoding="utf-8")
    written["report"] = str(report_path)
    json_path = target / "findings.json"
    json_path.write_text(json.dumps(findings_json(result), ensure_ascii=False, indent=2),
                         encoding="utf-8")
    written["findings"] = str(json_path)
    trace_path = target / "trace.md"
    trace_path.write_text(render_trace_markdown(result, trace_markdown, events),
                          encoding="utf-8")
    written["trace_md"] = str(trace_path)
    written["trace_jsonl"] = result.trace_jsonl
    return written


def exit_code(result: AuditResult) -> int:
    return 2 if result.blocking() else 0


def console_summary(result: AuditResult) -> str:
    counts = result.counts()
    lines = []
    lines.append("=" * 78)
    lines.append("机器人仿真实机参数错配诊断 · " + result.tool + " v" + result.version)
    lines.append("工程: " + result.root)
    lines.append("统计: 严重 %d | 高 %d | 中 %d | 低 %d | 信息 %d"
                 % (counts["CRITICAL"], counts["HIGH"], counts["MEDIUM"],
                    counts["LOW"], counts["INFO"]))
    lines.append("=" * 78)
    shown = [f for f in result.all_findings() if sev_rank(f.severity) <= 2]
    if not shown:
        lines.append("没有发现严重 / 高 / 中级问题。")
    for finding in shown:
        lines.append("")
        lines.append(SEV_BADGE.get(finding.severity, finding.severity) + " "
                     + finding.rule + " [" + finding.confidence + "/"
                     + STATUS_LABEL.get(finding.status, finding.status) + "] "
                     + finding.title)
        if finding.evidence:
            lines.append("    首证据: " + finding.evidence[0].location)
        for proposal in finding.proposals:
            lines.append("    建议: " + proposal.evidence.location + "  "
                         + proposal.old_text + " -> " + proposal.new_text)
    return "\n".join(lines)
