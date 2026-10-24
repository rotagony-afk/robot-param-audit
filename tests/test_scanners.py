from __future__ import annotations

import pytest

from rswaudit import scanners_log, scanners_static
from rswaudit.scanners_static import (extract_urdf, scan_launch, scan_source, scan_yaml,
                                     scan_xml_tags)

URDF = """<robot name="r">
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.0508"
                  length="0.03"/>
      </geometry>
    </visual>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.16 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
"""


def test_xml_scanner_reports_exact_lines_and_columns():
    tags = scan_xml_tags(URDF, "test.urdf")
    cylinder = [t for t in tags if t.name == "cylinder"][0]
    assert cylinder.attrs["radius"] == "0.0508"
    assert cylinder.attr_pos["radius"] == (5, 27)


def test_urdf_extraction_handles_multiline_tags():
    facts, model = extract_urdf(URDF, "test.urdf")
    radii = [f for f in facts if f.canonical == "wheel_radius"]
    assert len(radii) == 1
    assert radii[0].value == pytest.approx(0.0508)
    assert radii[0].evidence.line == 5
    joint = model.joints["left_wheel_joint"]
    assert joint["axis"] == (0.0, 1.0, 0.0)
    assert joint["origin"] == (0.0, 0.16, 0.1)
    assert joint["origin_ev"].line == 13


def test_urdf_expressions_stay_unresolved():
    text = URDF.replace('radius="0.0508"', 'radius="' + chr(36) + '{wheel_r}"')
    facts, _model = extract_urdf(text, "test.urdf")
    radii = [f for f in facts if f.canonical == "wheel_radius"]
    assert radii == [] or radii[0].unresolved


def test_yaml_entries_keep_columns_for_flow_sequences():
    text = "node:\n  ros__parameters:\n    radius: 0.05\n    wheels: [0.05, 0.06]\n"
    entries = scan_yaml(text, "a.yaml")
    by_key = {}
    for entry in entries:
        by_key.setdefault(entry.key, []).append(entry)
    assert by_key["radius"][0].literal == "0.05"
    assert by_key["radius"][0].col == 13
    items = by_key["wheels"]
    assert [i.literal for i in items] == ["0.05", "0.06"]
    # the second element must point at its own column, not the first one
    assert items[1].col == items[0].col + 6


def test_yaml_comments_and_quotes_are_handled():
    text = "a:\n  wheel_radius: '0.05'   # quoted\n"
    entries = scan_yaml(text, "a.yaml")
    assert entries[0].literal == "0.05"
    assert entries[0].path == ("a",)


def test_launch_python_parameters_and_args():
    text = (
        "DeclareLaunchArgument('use_sim_time', default_value='true')\n"
        "Node(parameters=[{'controller_frequency': 50.0, 'publish_tf': False}])\n"
    )
    facts, info = scan_launch(text, "x.launch.py")
    control = [f for f in facts if f.canonical == "control_rate"]
    assert control and control[0].value == pytest.approx(50.0)
    # booleans are not numbers and must not be reported as unknown parameters
    assert all(p["name"] != "publish_tf" for p in info["params"])


def test_source_constants_are_found_with_line_numbers():
    text = "#include <cmath>\nconstexpr double WHEEL_RADIUS = 0.05;\n#define TICKS_PER_REV 1024\n"
    facts = scan_source(text, "a.cpp")
    by_name = {f.canonical: f for f in facts}
    assert by_name["wheel_radius"].value == pytest.approx(0.05)
    assert by_name["wheel_radius"].evidence.line == 2
    assert by_name["ticks_per_rev"].value == pytest.approx(1024)


def test_source_scan_ignores_unrelated_variables():
    facts = scan_source("int counter = 0;\nself.foo = 3.5;\n", "a.py")
    assert facts == []


def test_csv_log_parsing_and_column_roles():
    text = "# recorded by diff_drive_node\nt,cmd_lin,odom_lin,truth_x,truth_y\n0.0,0.2,0.19,0.0,0.0\n0.1,0.2,0.19,0.02,0.0\n"
    table = scanners_log.parse_log_text(text, "log.csv")
    assert table.fmt == "csv"
    assert table.n == 2
    assert set(table.roles) == {"t", "cmd_lin", "odom_lin", "truth_x", "truth_y"}
    assert table.row_lines == [3, 4]
    assert table.col("cmd_lin") == [0.2, 0.2]


def test_kv_log_parsing():
    text = "[INFO] [12.5]: cmd_lin=0.3 odom_lin=0.29\n[INFO] [12.55]: cmd_lin=0.3 odom_lin=0.30\n"
    table = scanners_log.parse_log_text(text, "log.txt")
    assert table.fmt == "kv"
    assert table.n == 2
    assert table.col("cmd_lin") == [0.3, 0.3]
    assert table.has("t")


def test_log_without_time_column_is_marked_synthetic():
    text = "cmd_lin,odom_lin\n0.2,0.2\n0.2,0.2\n"
    table = scanners_log.parse_log_text(text, "log.csv")
    assert table.synthetic_time
    assert any("时间列" in w for w in table.warnings)
