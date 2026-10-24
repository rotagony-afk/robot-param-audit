from __future__ import annotations

from pathlib import Path

import pytest

from rswaudit.scanners_static import collect_static
from rswaudit.static_rules import run_static


def analyse(project: Path):
    inputs = collect_static(project)
    findings = run_static(inputs.facts, inputs.robots, inputs.yaml_entries,
                          inputs.launch_info)
    return inputs, findings


def rules(findings) -> list[str]:
    return [f.rule for f in findings]


@pytest.fixture(scope="module")
def demo(demo_project: Path):
    return analyse(demo_project)


def test_demo_project_finds_every_planted_defect(demo):
    _inputs, findings = demo
    found = set(rules(findings))
    expected = {"S01", "S02", "S04", "S05", "S06", "S07", "S09", "S10"}
    assert expected.issubset(found), "missing: " + str(expected - found)


def test_clean_project_is_silent():
    _inputs, findings = analyse(Path(__file__).resolve().parents[1]
                                / "examples" / "synthetic_clean")
    assert findings == []


def test_unit_mismatch_is_critical_and_suspected(demo):
    _inputs, findings = demo
    unit = [f for f in findings if f.rule == "S02"]
    assert len(unit) == 1
    assert unit[0].severity == "CRITICAL"
    assert unit[0].status == "suspected"
    assert unit[0].metrics["unit_ratio"] == pytest.approx(1000.0)


def test_wheel_radius_conflict_lists_all_sources(demo):
    _inputs, findings = demo
    conflict = [f for f in findings
                if f.rule == "S01" and f.metrics.get("canonical") == "wheel_radius"]
    assert len(conflict) == 1
    sources = conflict[0].metrics["sources"]
    assert sources == ["source", "urdf", "yaml"]
    assert conflict[0].metrics["rel_diff"] == pytest.approx(0.015748, abs=1e-5)
    locations = {e.location for e in conflict[0].evidence}
    assert "src/diff_drive_controller.cpp:10" in locations
    assert "description/urdf/demo_robot.urdf:38" in locations


def test_wheel_axis_conflict_is_reported(demo):
    _inputs, findings = demo
    axis = [f for f in findings if f.rule == "S05"]
    assert axis and axis[0].severity == "HIGH"
    assert len(axis[0].metrics["axes"]) == 2


def test_inertia_defects(demo):
    _inputs, findings = demo
    inertia = [f for f in findings if f.rule == "S06"]
    assert len(inertia) == 2
    severities = sorted(f.severity for f in inertia)
    assert severities == ["HIGH", "MEDIUM"]


def test_control_rate_vs_sim_rate(demo):
    _inputs, findings = demo
    rates = [f for f in findings if f.rule == "S07"]
    assert len(rates) == 1
    assert rates[0].metrics["control_rate"] == pytest.approx(50.0)
    assert rates[0].metrics["sim_rate"] == pytest.approx(20.0)


def test_duplicate_yaml_key(demo):
    _inputs, findings = demo
    dup = [f for f in findings if f.rule == "S10"]
    assert len(dup) == 1
    assert [e.line for e in dup[0].evidence] == [7, 15]


def test_every_evidence_line_points_at_the_quoted_text(demo, demo_project: Path):
    _inputs, findings = demo
    checked = 0
    for finding in findings:
        for evidence in finding.evidence:
            if not evidence.excerpt or not evidence.line:
                continue
            full = demo_project / evidence.path
            assert full.exists(), evidence.path
            lines = full.read_text(encoding="utf-8").splitlines()
            assert 1 <= evidence.line <= len(lines)
            assert lines[evidence.line - 1].strip() == evidence.excerpt, (
                evidence.path + ":" + str(evidence.line))
            checked += 1
    assert checked >= 20


def test_no_finding_without_evidence(demo):
    _inputs, findings = demo
    for finding in findings:
        assert finding.evidence, finding.rule + " has no evidence"
        for evidence in finding.evidence:
            assert evidence.path
