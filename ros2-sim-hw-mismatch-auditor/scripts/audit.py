#!/usr/bin/env python3
"""Skill entry point: run the auditor without installing the package.

Usage:
    python scripts/audit.py full --project <ros2 project> --log <run log>
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rswaudit.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
