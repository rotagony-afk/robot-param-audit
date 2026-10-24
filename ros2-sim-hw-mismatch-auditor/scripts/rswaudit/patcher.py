"""Patch proposal / optional application.

The auditor never edits your project by default. It renders a unified diff per
file into submission-ready .patch files, and only writes changes when the caller
explicitly passes --apply, in which case the original file is backed up next to
it as <name>.bak so the change is reversible.
"""

from __future__ import annotations

import difflib
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

from .evidence import Finding, Proposal


@dataclass
class FilePatch:
    path: str
    diff: str = ""
    applied: bool = False
    backup: str = ""
    proposals: list[Proposal] = field(default_factory=list)


def _read(root: Path, rel: str) -> str | None:
    full = root / rel
    if not full.exists():
        return None
    return full.read_text(encoding="utf-8", errors="replace")


def _replace_in_line(line: str, proposal: Proposal) -> str:
    old = proposal.old_text
    new = proposal.new_text
    if not old:
        return line
    col = proposal.evidence.col
    if col is not None:
        start = max(col - 1, 0)
        idx = line.find(old, start)
        if idx != -1 and idx <= start + 2:
            return line[:idx] + new + line[idx + len(old):]
    idx = line.find(old)
    if idx == -1:
        return line
    return line[:idx] + new + line[idx + len(old):]


def _apply_proposals(text: str, proposals: list[Proposal]) -> tuple[str, list[str]]:
    lines = text.splitlines(keepends=True)
    notes: list[str] = []
    for proposal in sorted(proposals, key=lambda p: -(p.evidence.line or 0)):
        lineno = proposal.evidence.line
        if not lineno or lineno > len(lines):
            notes.append("跳过（行号超出文件）：" + proposal.evidence.location)
            continue
        idx = lineno - 1
        original = lines[idx]
        updated = _replace_in_line(original, proposal)
        if updated == original:
            notes.append("跳过（该行未找到原字面量）：" + proposal.evidence.location)
            continue
        lines[idx] = updated
        notes.append("已改：" + proposal.evidence.location + " " + proposal.old_text
                     + " -> " + proposal.new_text)
    return "".join(lines), notes


def build_patches(findings: Iterable[Finding], root: str | Path) -> list[FilePatch]:
    root_path = Path(root)
    grouped: dict[str, list[Proposal]] = {}
    for finding in findings:
        for proposal in finding.proposals:
            grouped.setdefault(proposal.evidence.path, []).append(proposal)
    patches: list[FilePatch] = []
    for rel, proposals in sorted(grouped.items()):
        original = _read(root_path, rel)
        if original is None:
            continue
        updated, notes = _apply_proposals(original, proposals)
        diff = "".join(difflib.unified_diff(
            original.splitlines(keepends=True),
            updated.splitlines(keepends=True),
            fromfile="a/" + rel, tofile="b/" + rel, n=2,
        ))
        patches.append(FilePatch(path=rel, diff=diff, proposals=proposals))
    return patches


def write_patches(patches: list[FilePatch], out_dir: str | Path) -> list[str]:
    target = Path(out_dir) / "patches"
    target.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    for patch in patches:
        name = patch.path.replace("/", "__").replace("\\", "__") + ".patch"
        full = target / name
        body = patch.diff or "（该文件不需要修改）\n"
        full.write_text(
            "# auto-generated patch proposal, review before applying\n"
            "# target: " + patch.path + "\n"
            + body, encoding="utf-8")
        written.append(name)
    return written


def apply_patches(patches: list[FilePatch], root: str | Path) -> list[str]:
    root_path = Path(root)
    notes: list[str] = []
    for patch in patches:
        if not patch.diff.strip():
            continue
        full = root_path / patch.path
        if not full.exists():
            notes.append("文件不存在：" + patch.path)
            continue
        original = full.read_text(encoding="utf-8", errors="replace")
        updated, local_notes = _apply_proposals(original, patch.proposals)
        notes.extend(patch.path + ": " + note for note in local_notes)
        if updated == original:
            continue
        backup = full.with_suffix(full.suffix + ".bak")
        shutil.copyfile(full, backup)
        full.write_text(updated, encoding="utf-8")
        patch.applied = True
        patch.backup = backup.name
    return notes
