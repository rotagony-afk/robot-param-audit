"""Make the skill package importable without installing it."""

from __future__ import annotations

import shutil
import sys
import uuid
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "ros2-sim-hw-mismatch-auditor" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

TMP_ROOT = ROOT / "tests-work"


@pytest.fixture()
def tmp_path():
    """A writable scratch directory inside the project.

    pytest's own tmp_path fixture relies on the tmpdir plugin, whose session
    cleanup enumerates the temp root; that enumeration is denied in some
    sandboxed CI environments, so the suite manages its own directories.
    """
    TMP_ROOT.mkdir(parents=True, exist_ok=True)
    target = TMP_ROOT / ("case-" + uuid.uuid4().hex[:10])
    target.mkdir(parents=True, exist_ok=True)
    return target

DEMO = ROOT / "examples" / "demo_robot"
CLEAN = ROOT / "examples" / "synthetic_clean"


@pytest.fixture(scope="session")
def demo_project() -> Path:
    return DEMO


@pytest.fixture()
def demo_copy(tmp_path: Path) -> Path:
    """A writable copy of the demo project (so --apply tests never touch the repo)."""
    target = tmp_path / "demo_robot"
    shutil.copytree(DEMO, target)
    return target


@pytest.fixture()
def log_path(demo_project: Path):
    def _get(name: str) -> Path:
        return demo_project / "logs" / name
    return _get
