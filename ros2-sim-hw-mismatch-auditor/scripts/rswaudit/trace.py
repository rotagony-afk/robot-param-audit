"""Append-only debug trace.

Every stage writes one JSONL record per action: what was inspected, which rule
ran, what it concluded and why. The file is the "debugging footprint" the tool
promises: a second person can replay the run from the trace alone, and the tool
never claims a check it did not actually log.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


class Trace:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("", encoding="utf-8")
        self.seq = 0
        self.events: list[dict[str, Any]] = []
        self.started = time.time()

    def event(self, stage: str, action: str, **fields: Any) -> dict[str, Any]:
        self.seq += 1
        rec: dict[str, Any] = {
            "seq": self.seq,
            "stage": stage,
            "action": action,
            "t": round(time.time() - self.started, 4),
        }
        for key, value in fields.items():
            if value is None:
                continue
            rec[key] = value
        with self.path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        self.events.append(rec)
        return rec

    def stage(self, name: str) -> None:
        self.event(name, "stage-start")

    def to_markdown(self) -> str:
        rows = ["| # | 阶段 | 动作 | 详情 |", "|---|---|---|---|"]
        for rec in self.events:
            detail = {
                k: v
                for k, v in rec.items()
                if k not in ("seq", "stage", "action", "t")
            }
            text = json.dumps(detail, ensure_ascii=False, default=str)
            if len(text) > 220:
                text = text[:217] + "..."
            text = text.replace("|", "\\|")
            rows.append("| " + str(rec["seq"]) + " | " + rec["stage"] + " | "
                        + rec["action"] + " | " + text + " |")
        return "\n".join(rows) + "\n"
