#!/usr/bin/env python3
"""Generate the sample run logs used by the demo project.

Deterministic (fixed seed) so that the numbers quoted in README and in the
competition document always match what the tests assert.

Physical setup the logs pretend to come from:
    true wheel radius      0.0508 m  (2 inch wheels, matches the URDF cylinder)
    radius used by firmware 0.05   m  (the YAML value)
    true wheel separation  0.32   m  (matches the URDF joint origins)
    separation used        0.30   m  (the YAML value)

Consequences, which the auditor is supposed to recover from the data:
    linear odometry under-reports by 0.0508/0.05     = 1.6 %
    angular odometry over-reports by 0.32/0.30       = 6.67 %
"""

from __future__ import annotations

import math
import random
from pathlib import Path

R_TRUE = 0.0508
R_USED = 0.05
SEP_TRUE = 0.32
SEP_USED = 0.30

OUT_DIR = Path(__file__).resolve().parent / "demo_robot" / "logs"


def write_csv(path: Path, header: list[str], rows: list[list[str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [",".join(header)]
    lines.extend(",".join(row) for row in rows)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_straight() -> Path:
    """5 s straight run at 0.2 m/s with ground-truth position."""
    rng = random.Random(20260101)
    dt = 0.025
    n = 200
    rows: list[list[str]] = []
    x_true = 0.0
    for index in range(n):
        t = index * dt
        # The robot is already cruising when the recording starts, so the whole
        # log is one constant interval: no integrator edge effects.
        cmd = 0.2
        x_true += cmd * dt
        odom_v = cmd * (R_USED / R_TRUE) + rng.gauss(0.0, 1e-4)
        rows.append([f"{t:.3f}", f"{cmd:.4f}", f"{odom_v:.6f}",
                     f"{x_true:.6f}", "0.000000"])
    path = OUT_DIR / "run_straight.csv"
    write_csv(path, ["t", "cmd_lin", "odom_lin", "truth_x", "truth_y"], rows)
    return path


def write_turn() -> Path:
    """5 s in-place rotation at 0.5 rad/s with ground-truth heading."""
    rng = random.Random(20260102)
    dt = 0.02
    n = 250
    rows: list[list[str]] = []
    yaw_true = 0.0
    yaw_odom = 0.0
    for index in range(n):
        t = index * dt
        cmd = 0.0 if t < 0.2 else 0.5
        yaw_true += cmd * dt
        yaw_odom += cmd * dt * (SEP_TRUE / SEP_USED)
        rows.append([f"{t:.3f}", f"{cmd:.4f}",
                     f"{cmd * (SEP_TRUE / SEP_USED):.6f}",
                     f"{yaw_odom + rng.gauss(0.0, 2e-4):.6f}",
                     f"{yaw_true:.6f}"])
    path = OUT_DIR / "run_turn.csv"
    write_csv(path, ["t", "cmd_ang", "odom_ang", "odom_yaw", "truth_yaw"], rows)
    return path


def write_pid() -> Path:
    """4.5 s closed-loop record: dead start, 25 % overshoot, clipping, jitter.

    Deliberately contains NO ground-truth column: the auditor must refuse to
    quantify an absolute scale error on this file.
    """
    rng = random.Random(20260103)
    rows: list[list[str]] = []
    t = 0.0
    while t < 4.5:
        # 10 % of the samples arrive one to 1.6 cycles late.
        dt = 0.02
        if rng.random() < 0.10:
            dt = 0.02 * rng.uniform(2.0, 2.6)
        if t < 0.6:
            cmd, odom = 0.3, 0.0
        elif t < 3.0:
            cmd = 0.3
            tau = t - 0.6 - 0.06          # 60 ms transport delay
            if tau < 0.0:
                odom = 0.0
            else:
                odom = 0.3 * (1.0 + 0.25 * math.exp(-0.6 * tau)
                              * math.cos(2.0 * math.pi * 1.5 * tau))
        elif t < 3.9:
            cmd = 0.5
            odom = 0.5 * (1.0 - math.exp(-(t - 3.0) / 0.08))
        else:
            cmd = 0.0
            odom = max(0.0, 0.5 * math.exp(-(t - 3.9) / 0.10))
        noise = 0.0005 if t < 0.6 else 0.0015
        odom += rng.gauss(0.0, noise)
        rows.append([f"{t:.4f}", f"{cmd:.4f}", f"{odom:.6f}"])
        t += dt
    path = OUT_DIR / "run_pid.csv"
    write_csv(path, ["t", "cmd_lin", "odom_lin"], rows)
    return path


def main() -> None:
    for path in (write_straight(), write_turn(), write_pid()):
        print("wrote", path)


if __name__ == "__main__":
    main()
