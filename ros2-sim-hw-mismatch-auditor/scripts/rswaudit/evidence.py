"""Value objects shared by every extractor, rule and reporter.

Nothing in this module touches the file system. Every fact and every finding
carries an Evidence object pointing at the exact file, line and (where the file
is column-addressable) column it came from, so that a reader of the report can
verify each claim by opening the file. This is the mechanism that keeps the tool
from inventing parameters that do not exist.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable

SEVERITY_ORDER = {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3, "INFO": 4}
CONFIDENCE_ORDER = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
# Status tells the reader how much is actually proven.
STATUS_CONFIRMED = "confirmed"        # proven by files/logs that were read
STATUS_SUSPECTED = "suspected"        # plausible, needs a human look
STATUS_UNVERIFIABLE = "unverifiable"  # cannot be decided from the inputs


def sev_rank(severity: str) -> int:
    return SEVERITY_ORDER.get((severity or "").upper(), 9)


def conf_rank(confidence: str) -> int:
    return CONFIDENCE_ORDER.get((confidence or "").upper(), 9)


def fmt(value: float | int | None, digits: int = 6) -> str:
    """Format a number for humans without trailing zero noise."""
    if value is None:
        return "?"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, int):
        return str(value)
    if value != value or value in (float("inf"), float("-inf")):
        return str(value)
    s = f"{value:.{digits}f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


@dataclass
class Evidence:
    """A single, citable location in an input file."""

    path: str
    line: int | None = None
    col: int | None = None
    excerpt: str = ""
    locator: str = ""

    @property
    def location(self) -> str:
        if self.line is None:
            return self.path
        return f"{self.path}:{self.line}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": self.path,
            "line": self.line,
            "col": self.col,
            "excerpt": self.excerpt,
            "locator": self.locator,
            "location": self.location,
        }

    def render(self) -> str:
        out = self.location
        if self.locator:
            out += " (" + self.locator + ")"
        if self.excerpt:
            out += "  ->  " + self.excerpt
        return out


@dataclass
class Fact:
    """One physical/kinematic parameter read from one place."""

    canonical: str
    raw_name: str
    value: float | None
    literal: str
    unit: str
    scope: str
    source: str          # urdf | yaml | launch | source | sdf
    evidence: Evidence
    factor: float = 1.0  # multiplier applied to reach the canonical unit
    unresolved: bool = False
    confidence: str = "HIGH"
    note: str = ""

    @property
    def is_number(self) -> bool:
        return self.value is not None and not self.unresolved

    def to_dict(self) -> dict[str, Any]:
        return {
            "canonical": self.canonical,
            "raw_name": self.raw_name,
            "value": self.value,
            "literal": self.literal,
            "unit": self.unit,
            "scope": self.scope,
            "source": self.source,
            "factor": self.factor,
            "unresolved": self.unresolved,
            "confidence": self.confidence,
            "note": self.note,
            "evidence": self.evidence.to_dict(),
        }

    def render(self) -> str:
        val = self.literal if self.unresolved else fmt(self.value)
        unit = (" " + self.unit) if self.unit else ""
        return f"{self.source}:{self.raw_name}={val}{unit} @ {self.evidence.location}"


@dataclass
class Proposal:
    """A concrete, reviewable edit suggestion for one literal in one file."""

    evidence: Evidence
    old_text: str
    new_text: str
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": self.evidence.path,
            "line": self.evidence.line,
            "col": self.evidence.col,
            "old_text": self.old_text,
            "new_text": self.new_text,
            "reason": self.reason,
        }


@dataclass
class Finding:
    rule: str
    severity: str
    title: str
    detail: str
    evidence: list[Evidence] = field(default_factory=list)
    confidence: str = "MEDIUM"
    status: str = STATUS_CONFIRMED
    proposals: list[Proposal] = field(default_factory=list)
    tags: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "rule": self.rule,
            "severity": self.severity,
            "title": self.title,
            "detail": self.detail,
            "confidence": self.confidence,
            "status": self.status,
            "evidence": [e.to_dict() for e in self.evidence],
            "proposals": [p.to_dict() for p in self.proposals],
            "tags": list(self.tags),
            "metrics": dict(self.metrics),
        }

    def sort_key(self) -> tuple[int, int, str]:
        return (sev_rank(self.severity), conf_rank(self.confidence), self.rule)


def sort_findings(findings: Iterable[Finding]) -> list[Finding]:
    return sorted(findings, key=lambda f: f.sort_key())
