from __future__ import annotations

import json
from pathlib import Path

import pytest

from rswaudit.cli import main, run_audit
from rswaudit.report import exit_code


def test_full_run_produces_all_artifacts(demo_project: Path, tmp_path: Path):
    out = tmp_path / "out"
    result, written = run_audit(project=str(demo_project),
                                log=str(demo_project / "logs" / "run_straight.csv"),
                                out=str(out), quiet=True)
    assert Path(written["report"]).exists()
    assert Path(written["findings"]).exists()
    assert (out / "trace.jsonl").exists()
    assert (out / "trace.md").exists()
    assert (out / "patches").is_dir()
    assert exit_code(result) == 2

    payload = json.loads(Path(written["findings"]).read_text(encoding="utf-8"))
    assert payload["summary"]["HIGH"] >= 5
    assert payload["blocking_count"] >= 1
    assert payload["inputs"]
    for finding in payload["findings"]:
        assert finding["evidence"], finding["rule"]
        assert finding["status"] in ("confirmed", "suspected", "unverifiable")

    trace_lines = (out / "trace.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(trace_lines) > 20
    seqs = []
    for line in trace_lines:
        record = json.loads(line)
        seqs.append(record["seq"])
        assert record["stage"] and record["action"]
    assert seqs == sorted(seqs)


def test_report_separates_verified_from_unverified(demo_project: Path, tmp_path: Path):
    out = tmp_path / "out"
    _result, written = run_audit(project=str(demo_project),
                                 log=str(demo_project / "logs" / "run_pid.csv"),
                                 out=str(out), quiet=True)
    report = Path(written["report"]).read_text(encoding="utf-8")
    assert "## 5. 未验证 / 无法判定的内容" in report
    assert "缺少真值" in report or "无法判定" in report
    assert "根因收敛" not in report        # pid log has no ground truth -> no C 结论
    assert "已确认通过的检查项" in report


def test_static_only_run_skips_runtime(demo_project: Path, tmp_path: Path):
    out = tmp_path / "out"
    result, written = run_audit(project=str(demo_project), out=str(out), quiet=True)
    assert result.runtime_findings == []
    report = Path(written["report"]).read_text(encoding="utf-8")
    assert "R 类规则全部未执行" in report


def test_clean_project_exits_zero(tmp_path: Path):
    root = Path(__file__).resolve().parents[1]
    out = tmp_path / "out"
    result, _written = run_audit(project=str(root / "examples" / "synthetic_clean"),
                                 out=str(out), quiet=True)
    assert result.all_findings() == []
    assert exit_code(result) == 0


def test_missing_log_is_a_usage_error(demo_project: Path, tmp_path: Path):
    code = main(["runtime", "--project", str(demo_project),
                 "--log", str(tmp_path / "nope.csv"), "--quiet"])
    assert code == 1


def test_apply_flag_edits_the_copy(demo_copy: Path, tmp_path: Path):
    result, _written = run_audit(project=str(demo_copy),
                                 log=str(demo_copy / "logs" / "run_straight.csv"),
                                 out=str(tmp_path / "out"), apply=True, quiet=True)
    yaml_path = demo_copy / "description" / "config" / "demo_robot_params.yaml"
    assert "wheel_radius: 0.0508" in yaml_path.read_text(encoding="utf-8")
    assert any("已改" in note for note in result.notes)
