from __future__ import annotations

from pathlib import Path

import pytest

from rswaudit.correlate import correlate
from rswaudit.patcher import apply_patches, build_patches
from rswaudit.runtime_rules import run_runtime
from rswaudit.scanners_log import load_log
from rswaudit.scanners_static import collect_static
from rswaudit.static_rules import run_static


def prepare(project: Path, log: str):
    inputs = collect_static(project)
    static = run_static(inputs.facts, inputs.robots, inputs.yaml_entries,
                        inputs.launch_info)
    table = load_log(project / "logs" / log)
    runtime = run_runtime(table)
    cross = correlate(inputs.facts, inputs.robots, runtime + static)
    return inputs, static, runtime, cross


def test_linear_correlation_converges_on_urdf_geometry(demo_project: Path):
    _i, _s, _r, cross = prepare(demo_project, "run_straight.csv")
    c01 = [f for f in cross if f.rule == "C01"]
    assert len(c01) == 1
    finding = c01[0]
    assert finding.severity == "HIGH"
    assert finding.status == "confirmed"
    assert finding.metrics["new"] == pytest.approx(0.0508)
    assert finding.metrics["old"] == pytest.approx(0.05)
    new_values = {p.new_text for p in finding.proposals}
    assert new_values == {"0.0508"}
    assert len(finding.proposals) == 3


def test_angular_correlation_converges_on_joint_origins(demo_project: Path):
    _i, _s, _r, cross = prepare(demo_project, "run_turn.csv")
    c02 = [f for f in cross if f.rule == "C02"]
    assert len(c02) == 1
    assert c02[0].metrics["new"] == pytest.approx(0.32)
    assert len(c02[0].proposals) == 2


def test_ambiguous_scale_is_reported_as_unverifiable(tmp_path: Path):
    project = tmp_path / "ambiguous"
    (project / "config").mkdir(parents=True)
    (project / "config" / "params.yaml").write_text(
        "bot:\n  ros__parameters:\n    wheel_radius: 0.05\n    ticks_per_rev: 1024\n",
        encoding="utf-8")
    log = project / "run.csv"
    log.write_text("# no ground truth, 1.6% scale error\n"
                   "t,cmd_lin,odom_lin\n"
                   "0.0,0.2,0.19685\n1.0,0.2,0.19685\n2.0,0.2,0.19685\n",
                   encoding="utf-8")
    inputs = collect_static(project)
    table = load_log(log)
    runtime = run_runtime(table, expect_distance=0.6)
    cross = correlate(inputs.facts, inputs.robots, runtime)
    c01 = [f for f in cross if f.rule == "C01"][0]
    assert c01.status == "unverifiable"
    assert c01.proposals == []
    assert len(c01.metrics["candidates"]) == 2


def test_patch_diff_targets_the_right_lines(demo_project: Path):
    inputs, static, runtime, cross = prepare(demo_project, "run_straight.csv")
    patches = build_patches(static + runtime + cross, demo_project)
    names = {p.path for p in patches}
    assert names == {"description/config/demo_robot_params.yaml",
                     "src/diff_drive_controller.cpp"}
    yaml_patch = [p for p in patches
                  if p.path.endswith("demo_robot_params.yaml")][0]
    added = [line for line in yaml_patch.diff.splitlines() if line.startswith("+")]
    assert "+    wheel_radius: 0.0508" in added
    assert sum(1 for line in added if "wheel_radius: 0.0508" in line) == 2
    # untouched parameters must not appear as removed lines
    assert "-    wheel_separation: 0.30" not in yaml_patch.diff


def test_apply_writes_backup_and_is_idempotent(demo_copy: Path):
    inputs, static, runtime, cross = prepare(demo_copy, "run_straight.csv")
    patches = build_patches(static + runtime + cross, demo_copy)
    notes = apply_patches(patches, demo_copy)
    assert any("已改" in note for note in notes)
    yaml_path = demo_copy / "description" / "config" / "demo_robot_params.yaml"
    text = yaml_path.read_text(encoding="utf-8")
    assert "wheel_radius: 0.0508" in text
    assert yaml_path.with_suffix(".yaml.bak").exists()

    # Re-running produces no further edit for the radius.
    inputs2, static2, runtime2, cross2 = prepare(demo_copy, "run_straight.csv")
    patches2 = build_patches(static2 + runtime2 + cross2, demo_copy)
    radius_changes = [line for patch in patches2 for line in patch.diff.splitlines()
                      if line.startswith("+") and "0.0508" in line]
    assert radius_changes == []


def test_apply_respects_column_not_first_occurrence(tmp_path: Path):
    target = tmp_path / "cfg.yaml"
    target.write_text("a:\n  wheel_radius: 0.05\nb:\n  wheel_radius: 0.05\n",
                      encoding="utf-8")
    from rswaudit.evidence import Evidence, Finding, Proposal
    evidence = Evidence("cfg.yaml", 4, 17, "", "test")
    finding = Finding("C01", "HIGH", "t", "d",
                      proposals=[Proposal(evidence, "0.05", "0.0508", "test")])
    patches = build_patches([finding], tmp_path)
    apply_patches(patches, tmp_path)
    lines = target.read_text(encoding="utf-8").splitlines()
    assert lines[1] == "  wheel_radius: 0.05"
    assert lines[3] == "  wheel_radius: 0.0508"
