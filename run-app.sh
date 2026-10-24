#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
export PYTHONIOENCODING=utf-8
exec python3 ros2-sim-hw-mismatch-auditor/scripts/audit.py app --port "${1:-8765}"
