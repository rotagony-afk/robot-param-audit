from __future__ import annotations

import pytest

from rswaudit.runtime_rules import run_runtime
from rswaudit.scanners_log import load_log


def by_rule(findings, rule):
    return [f for f in findings if f.rule == rule]


def test_straight_run_linear_scale(log_path):
    table = load_log(log_path("run_straight.csv"))
    findings = run_runtime(table)
    r01 = by_rule(findings, "R01")[0]
    assert r01.severity == "HIGH"
    assert r01.status == "confirmed"
    assert r01.metrics["ratio"] == pytest.approx(1.016, abs=2e-3)
    assert r01.metrics["odom_distance"] == pytest.approx(0.9793, abs=1e-3)
    assert r01.metrics["truth_distance"] == pytest.approx(0.995, abs=1e-3)


def test_straight_run_has_no_false_positive_saturation(log_path):
    table = load_log(log_path("run_straight.csv"))
    findings = run_runtime(table)
    # a constant-speed test is not a saturated loop
    assert by_rule(findings, "R05") == []


def test_turn_run_angular_scale(log_path):
    table = load_log(log_path("run_turn.csv"))
    findings = run_runtime(table)
    r02 = by_rule(findings, "R02")[0]
    assert r02.severity == "HIGH"
    assert r02.metrics["ratio"] == pytest.approx(0.9375, abs=2e-3)
    assert r02.metrics["truth_source"].startswith("由 truth_yaw")


def test_log_without_ground_truth_refuses_to_quantify(log_path):
    table = load_log(log_path("run_pid.csv"))
    findings = run_runtime(table)
    r01 = by_rule(findings, "R01")[0]
    assert r01.status == "unverifiable"
    assert r01.severity == "INFO"
    assert "ratio" not in r01.metrics
    r02 = by_rule(findings, "R02")[0]
    assert r02.status == "unverifiable"


def test_pid_log_triggers_timing_and_control_rules(log_path):
    table = load_log(log_path("run_pid.csv"))
    findings = run_runtime(table)
    found = {f.rule for f in findings}
    assert {"R03", "R04", "R05", "R06", "R07"}.issubset(found)
    jitter = by_rule(findings, "R03")[0]
    assert jitter.severity in ("HIGH", "MEDIUM")
    assert jitter.metrics["jitter_frac"] > 0.2
    stuck = by_rule(findings, "R07")[0]
    assert stuck.metrics["duration_s"] >= 0.5


def test_expect_distance_replaces_missing_truth(tmp_path):
    log = tmp_path / "straight.csv"
    log.write_text("t,cmd_lin,odom_lin\n"
                   "0.0,0.2,0.19685\n"
                   "1.0,0.2,0.19685\n"
                   "2.0,0.2,0.19685\n", encoding="utf-8")
    table = load_log(log)
    findings = run_runtime(table, expect_distance=1.0)
    r01 = by_rule(findings, "R01")[0]
    assert r01.status == "confirmed"
    assert r01.metrics["truth_source"].startswith("由 --expect-distance")
    assert r01.metrics["ratio"] == pytest.approx(1.0 / 0.3937, abs=1e-3)


def test_too_few_samples_is_unverifiable(tmp_path):
    log = tmp_path / "tiny.csv"
    log.write_text("t,cmd_lin,odom_lin\n0.0,0.1,0.1\n", encoding="utf-8")
    table = load_log(log)
    assert any(w for w in table.warnings)
    findings = run_runtime(table)
    assert findings  # nothing crashes, it just refuses to conclude
