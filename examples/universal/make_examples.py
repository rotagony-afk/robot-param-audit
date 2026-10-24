#!/usr/bin/env python3
"""Generate the chassis-agnostic example set (models, mesh, run log).

Everything here is synthesised on purpose so the examples stay small and the
expected numbers are reproducible:

  mecanum_bot.json        4 mecanum wheels, +/-45 deg rollers  -> can strafe
  mecanum_no_rollers.json same robot but roller angle left at 0 -> cannot strafe
  ackermann_bot.json      front-steered chassis                -> steering angles
  mesh_bot.json           declares 2.4 kg while the mesh says 7.2 kg
  chassis.stl             the 0.3 x 0.2 x 0.1 m chassis mesh
  demo_robot.mjcf.xml     the same differential drive, in MuJoCo format
  mecanum_log.csv         4 s of pure sideways motion with ground truth
  reality.json            measured parameters for the demo robot

Run:  python examples/universal/make_examples.py
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path

HERE = Path(__file__).resolve().parent


MECANUM = {
    "name": "mecanum_bot",
    "drive": "mecanum",
    "chassis": {"mass": 12.0, "inertia_zz": 0.36, "wheelbase": 0.44, "track": 0.36},
    "wheel_radius": 0.075,
    "motor": {"stall_torque": 1.2, "no_load_omega": 25.0},
    "max_omega": 20.0,
    "env": {"friction": 0.6, "gravity": 9.81},
    "wheels": [
        {"name": "fl", "x": 0.22, "y": 0.18, "roller_deg": 45},
        {"name": "fr", "x": 0.22, "y": -0.18, "roller_deg": -45},
        {"name": "rl", "x": -0.22, "y": 0.18, "roller_deg": -45},
        {"name": "rr", "x": -0.22, "y": -0.18, "roller_deg": 45},
    ],
    "reality": {"wheel_radius": 0.0752, "mass": 12.4, "friction": 0.55},
}

ACKERMANN = {
    "name": "ackermann_bot",
    "drive": "ackermann",
    "chassis": {"mass": 55.0, "inertia_zz": 8.0, "wheelbase": 1.05, "track": 1.2},
    "wheel_radius": 0.15,
    "motor": {"stall_torque": 12.0, "no_load_omega": 30.0},
    "max_omega": 25.0,
    "env": {"friction": 0.8, "gravity": 9.81},
    "wheels": [
        {"name": "fl", "x": 0.525, "y": 0.6, "steerable": True},
        {"name": "fr", "x": 0.525, "y": -0.6, "steerable": True},
        {"name": "rl", "x": -0.525, "y": 0.6},
        {"name": "rr", "x": -0.525, "y": -0.6},
    ],
}

MESH_BOT = {
    "name": "mesh_bot",
    "drive": "differential",
    "chassis": {"mass": 2.4, "inertia_zz": 0.02},
    "wheel_radius": 0.05,
    "max_omega": 20.0,
    "wheels": [
        {"name": "left", "x": 0.0, "y": 0.15},
        {"name": "right", "x": 0.0, "y": -0.15},
    ],
}

MJCF = """<mujoco model="demo_robot_mjcf">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base_link" pos="0 0 0.1">
      <inertial pos="0 0 0" mass="2.4" diaginertia="0.01 0.012 0.02"/>
      <geom name="chassis" type="box" size="0.12 0.14 0.05"/>
      <body name="left_wheel" pos="0 0.16 0">
        <joint name="left_wheel_joint" type="hinge" axis="0 1 0" range="-100000 100000"/>
        <inertial pos="0 0 0" mass="0.18" diaginertia="0.0001 0.0002 0.0002"/>
        <geom type="cylinder" size="0.0508 0.015" euler="90 0 0"/>
      </body>
      <body name="right_wheel" pos="0 -0.16 0">
        <joint name="right_wheel_joint" type="hinge" axis="0 1 0" range="-100000 100000"/>
        <inertial pos="0 0 0" mass="0.18" diaginertia="0.0001 0.0002 0.0002"/>
        <geom type="cylinder" size="0.0508 0.015" euler="90 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="left_wheel_joint" gear="1.5"/>
    <motor joint="right_wheel_joint" gear="1.5"/>
  </actuator>
</mujoco>
"""

REALITY = {
    "wheel_radius": 0.0508,
    "mass": 3.1,
    "friction": 0.55,
}


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")


def box_stl(path: Path, sx: float, sy: float, sz: float) -> None:
    """Binary STL of an axis-aligned box centred on the origin."""
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    v = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
         (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    with path.open("wb") as fh:
        fh.write(b"rsw-audit chassis sample".ljust(80, b"\x00"))
        fh.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            fh.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for index in (a, b, c):
                fh.write(struct.pack("<3f", *v[index]))
            fh.write(struct.pack("<H", 0))


def mecanum_log(path: Path, duration: float = 4.0, speed: float = 0.2) -> None:
    """Pure sideways motion: 0.8 m to the left, wheels rolling the whole time."""
    dt = 0.02
    rows = ["t,cmd_lat,odom_lat,truth_x,truth_y"]
    steps = int(duration / dt)
    y = 0.0
    for index in range(steps + 1):
        t = index * dt
        y += speed * dt
        rows.append("%.3f,%.4f,%.6f,0.000000,%.6f"
                    % (t, speed, speed * 0.997, y))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")


def main() -> None:
    write_json(HERE / "mecanum_bot.json", MECANUM)
    broken = json.loads(json.dumps(MECANUM))
    broken["name"] = "mecanum_no_rollers"
    for wheel in broken["wheels"]:
        wheel["roller_deg"] = 0
    broken.pop("reality", None)
    write_json(HERE / "mecanum_no_rollers.json", broken)
    write_json(HERE / "ackermann_bot.json", ACKERMANN)
    write_json(HERE / "mesh_bot.json", MESH_BOT)
    write_json(HERE / "reality.json", REALITY)
    (HERE / "demo_robot.mjcf.xml").write_text(MJCF, encoding="utf-8")
    box_stl(HERE / "chassis.stl", 0.30, 0.20, 0.10)
    mecanum_log(HERE / "mecanum_log.csv")
    for path in sorted(HERE.iterdir()):
        if path.name != Path(__file__).name:
            print("wrote", path.name, path.stat().st_size, "bytes")


if __name__ == "__main__":
    main()
