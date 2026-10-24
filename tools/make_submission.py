#!/usr/bin/env python3
"""Build the submission package.

Only files that are actually part of the deliverable are packed: an explicit
allow-list of directories plus a deny-list of build artifacts, so nothing stray
(no .bak, no __pycache__, no local audit output, no sandbox leftovers) can end up
in the zip.

Usage:
    python tools/make_submission.py [--out ../submission]
"""

from __future__ import annotations

import argparse
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

TOP_FILES = ["README.md", "LICENSE", "pyproject.toml", "requirements.txt", ".gitignore",
             "run-app.cmd", "run-app.sh", "run-demo.cmd", "push-to-github.cmd"]
INCLUDE_DIRS = [
    "ros2-sim-hw-mismatch-auditor",
    "examples",
    "tests",
    "tools",
    "docs",
]
SKIP_DIRS = {
    "__pycache__", ".git", ".pytest_cache", "tests-work", "pytest-tmp",
    ".pytest-tmp", "_pages", "_frames", "_out-static", "_out-full",
    "audit-out", "build", "dist", ".venv", "node_modules",
}
SKIP_PREFIX = ("pytest-cache-files-", "_out-", "_frames")
SKIP_SUFFIX = (".pyc", ".pyo", ".bak", ".egg-info")
SKIP_NAMES = {".DS_Store", "Thumbs.db"}


def collect() -> list[tuple[Path, str]]:
    entries: list[tuple[Path, str]] = []
    for name in TOP_FILES:
        path = ROOT / name
        if path.exists():
            entries.append((path, name))
    for directory in INCLUDE_DIRS:
        base = ROOT / directory
        if not base.exists():
            continue
        for path in sorted(base.rglob("*")):
            if not path.is_file():
                continue
            parts = path.relative_to(ROOT).parts
            if any(part in SKIP_DIRS or part.startswith(SKIP_PREFIX) for part in parts):
                continue
            if path.suffix in SKIP_SUFFIX or path.name in SKIP_NAMES:
                continue
            if path.stat().st_size == 0 and path.suffix in (".py", ".md"):
                continue
            entries.append((path, path.relative_to(ROOT).as_posix()))
    return entries


def write_zip(target: Path, entries: list[tuple[Path, str]], prefix: str = "") -> int:
    target.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for path, arcname in entries:
            zf.write(path, prefix + arcname)
    return len(entries)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(ROOT.parent / "submission"))
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    entries = collect()
    total = write_zip(out / "project-source.zip", entries)
    print("project-source.zip: %d files, %d bytes"
          % (total, (out / "project-source.zip").stat().st_size))

    skill = [(p, a) for p, a in entries if a.startswith("ros2-sim-hw-mismatch-auditor/")]
    write_zip(out / "skill.zip", skill)
    print("skill.zip: %d files, %d bytes"
          % (len(skill), (out / "skill.zip").stat().st_size))

    for name in ("project-introduction.pdf", "demo-video.mp4"):
        source = ROOT / "docs" / name
        if source.exists():
            (out / name).write_bytes(source.read_bytes())
            print("copied %s (%d bytes)" % (name, source.stat().st_size))
    guide = ROOT / "docs" / "06-提交操作指引.md"
    if guide.exists():
        (out / "提交指引.md").write_bytes(guide.read_bytes())
        print("copied 提交指引.md (%d bytes)" % guide.stat().st_size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
