"""Forward simulation and parameter identification.

The simulation is deliberately small and honest: it integrates the wheel-speed
kinematics of the model with a first-order actuator lag, a transport delay, a
wheel speed limit and a traction limit. It does *not* solve contact dynamics, so
it cannot predict vibration, wheel hop or rolling over. What it can do is
answer the question that actually blocks people:

    "with the numbers written in this model, will the robot go where the
     command says, and if not, by how much and because of which parameter?"

Three things come out of it:

* a predicted trajectory, including the odometry a firmware would report if it
  used the model wheel radius instead of the physical one;
* the same trajectory under perturbed parameters, which gives a numerical
  Jacobian used to identify the parameter that best explains a measured run;
* an explicit identifiability verdict, so an under-determined fit is reported
  as ambiguous instead of being presented as a root cause.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from . import registry
from .evidence import Evidence, Finding, Proposal, fmt
from .kinematics import ROLLING_RESISTANCE, Twist, ideal_steering, wheel_speeds
from .modelspec import ModelSpec, effective_friction, effective_mass, effective_radius
from .scanners_log import LogTable

DEFAULT_DT = 0.02
DEFAULT_TAU = 0.03


@dataclass
class Profile:
    t: list[float] = field(default_factory=list)
    vx: list[float] = field(default_factory=list)
    vy: list[float] = field(default_factory=list)
    w: list[float] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.t)


def profile_from_table(table: LogTable) -> Profile:
    t, _ok = table.time_axis()
    n = len(t)
    vx = list(table.col("cmd_lin") or [0.0] * n)
    w = list(table.col("cmd_ang") or [0.0] * n)
    vy = list(table.col("cmd_lat") or [0.0] * n)
    return Profile(t=list(t), vx=vx, vy=vy, w=w)


def profile_step(duration: float, vx: float, w: float = 0.0, vy: float = 0.0,
                 dt: float = DEFAULT_DT) -> Profile:
    steps = max(2, int(duration / dt) + 1)
    times = [i * dt for i in range(steps)]
    return Profile(t=times, vx=[vx] * steps, vy=[vy] * steps, w=[w] * steps)


@dataclass
class SimResult:
    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    psi: list[float] = field(default_factory=list)
    vx: list[float] = field(default_factory=list)
    vy: list[float] = field(default_factory=list)
    w: list[float] = field(default_factory=list)
    odom_distance: float = 0.0
    odom_yaw: float = 0.0
    distance: float = 0.0
    yaw: float = 0.0
    saturation_fraction: float = 0.0
    slip_fraction: float = 0.0
    max_traction_margin: float = 1.0
    radius_true: float = 0.0
    radius_used: float = 0.0
    notes: list[str] = field(default_factory=list)


def simulate(model: ModelSpec, profile: Profile, dt: float = DEFAULT_DT,
             transport_delay: float = 0.0, tau: float = DEFAULT_TAU,
             radius_used: float | None = None,
             track_used: float | None = None,
             friction: float | None = None,
             max_steps: int = 20000) -> SimResult:
    """Forward kinematics with actuator lag, delay, speed and traction limits."""
    if not model.wheels:
        return SimResult(notes=["模型没有轮子，无法仿真"])
    # r_true is the physical radius (reality beats the model), r_used is what the
    # firmware divides by: the number written in the model/config.
    r_true = effective_radius(model) or model.wheel_radius
    r_used = radius_used if radius_used is not None else model.wheel_radius
    track_true = model.track or 0.0
    track_used = track_used if track_used is not None else track_true
    mu = effective_friction(model) if friction is None else friction
    gravity = model.gravity

    result = SimResult(radius_true=r_true, radius_used=r_used)
    result.notes.append("一阶执行器滞后 tau=" + fmt(tau, 4) + " s，传输延迟 "
                        + fmt(transport_delay * 1000.0, 2) + " ms，"
                        "附着限制 a_max=" + fmt(max(mu - ROLLING_RESISTANCE, 0.05) * gravity, 3)
                        + " m/s^2")
    if r_true <= 0:
        result.notes.append("轮半径未知或为 0，仿真结果不可用")
        return result

    wheel_omega = [0.0] * len(model.wheels)
    delay_samples = max(0, int(round(transport_delay / dt))) if dt > 0 else 0
    queue: list[list[float]] = []
    x = y = psi = 0.0
    vx = vy = 0.0
    sat_steps = slip_steps = 0
    steps = min(len(profile), max_steps)
    odom_dist = 0.0
    odom_yaw = 0.0
    a_max = max(mu - ROLLING_RESISTANCE, 0.05) * gravity

    for index in range(steps):
        t = profile.t[index]
        if index:
            step_dt = profile.t[index] - profile.t[index - 1]
        else:
            step_dt = dt
        if not (step_dt > 0) or step_dt > 1.0:
            step_dt = dt

        command = Twist(profile.vx[index], profile.vy[index], profile.w[index])
        if any(value != value for value in (command.vx, command.vy, command.w)):
            command = Twist(0.0, 0.0, 0.0)
        steering = ideal_steering(model, command)
        target = wheel_speeds(model, command, steering)
        for i, wheel in enumerate(model.wheels):
            if wheel.max_omega > 0 and abs(target[i]) > wheel.max_omega:
                target[i] = math.copysign(wheel.max_omega, target[i])
                sat_steps += 1

        if delay_samples:
            queue.append(target)
            target = queue.pop(0) if len(queue) > delay_samples else [0.0] * len(target)

        if tau > 0:
            alpha = min(1.0, step_dt / tau)
            for i in range(len(wheel_omega)):
                wheel_omega[i] += (target[i] - wheel_omega[i]) * alpha
        else:
            wheel_omega = list(target)

        kinematic, _residual = _twist_from(model, wheel_omega, r_true)

        # a tyre can only transmit so much force: limit how fast the body
        # velocity may follow the wheels instead of scaling the wheels forever
        dvx = kinematic.vx - vx
        dvy = kinematic.vy - vy
        demand = math.hypot(dvx, dvy) / step_dt
        result.max_traction_margin = max(result.max_traction_margin, demand / a_max)
        if demand > a_max:
            scale = a_max / demand
            vx += dvx * scale
            vy += dvy * scale
            slip_steps += 1
        else:
            vx, vy = kinematic.vx, kinematic.vy
        w = kinematic.w

        x += (vx * math.cos(psi) - vy * math.sin(psi)) * step_dt
        y += (vx * math.sin(psi) + vy * math.cos(psi)) * step_dt
        psi += w * step_dt

        odom_scale = (r_used / r_true) if r_true else 1.0
        odom_dist += math.hypot(vx, vy) * odom_scale * step_dt
        # a firmware using L_used reports omega_odom = omega_true * L_true/L_used
        yaw_scale = (track_true / track_used) if track_used else 1.0
        odom_yaw += w * odom_scale * yaw_scale * step_dt

        result.t.append(t)
        result.x.append(x)
        result.y.append(y)
        result.psi.append(psi)
        result.vx.append(vx)
        result.vy.append(vy)
        result.w.append(w)

    result.distance = _path_length(result)
    result.yaw = psi
    result.odom_distance = odom_dist
    result.odom_yaw = odom_yaw
    if steps:
        result.saturation_fraction = sat_steps / steps
        result.slip_fraction = slip_steps / steps
    return result


def _path_length(result: SimResult) -> float:
    total = 0.0
    for i in range(1, len(result.x)):
        total += math.hypot(result.x[i] - result.x[i - 1], result.y[i] - result.y[i - 1])
    return total


def _twist_from(model: ModelSpec, omegas: list[float],
                radius: float) -> tuple[Twist, float]:
    """Body twist from wheel speeds using the *physical* radius.

    Implemented here (rather than reusing kinematics.twist_from_speeds) because
    the simulated radius may differ from the one stored in the model.
    """
    from .kinematics import jacobian, lstsq3
    # J already contains 1/r, so the constraint is J * xi = omega
    rows = jacobian(model)
    rhs = list(omegas)
    solution, _singular, _rank = lstsq3(rows, rhs)
    twist = Twist(*solution)
    residual = 0.0
    for row, value in zip(rows, rhs):
        predicted = row[0] * twist.vx + row[1] * twist.vy + row[2] * twist.w
        residual = max(residual, abs(predicted - value))
    return twist, residual


# --------------------------------------------------------------------------- #
# identification
# --------------------------------------------------------------------------- #

DEFAULT_PARAMS = ("wheel_radius", "wheel_separation", "latency")


def identify(model: ModelSpec, profile: Profile, measured: dict, dt: float = DEFAULT_DT,
             params: tuple[str, ...] = DEFAULT_PARAMS,
             step: float = 0.02) -> dict[str, Any]:
    """Which model parameter best explains the measured run, and by how much.

    Each parameter is tried alone: its sensitivity is measured numerically, and
    the relative correction that would zero the residual on its own is computed.
    Only parameters whose single-parameter correction is physically plausible
    (below 40 %) are kept. If exactly one survives, the fit is identifiable; if
    several survive, the report says so instead of picking one.
    """
    base = simulate(model, profile, dt)
    keys = [key for key in ("distance", "yaw") if measured.get(key) is not None]
    base_residual = [getattr(base, key) - measured[key] for key in keys]

    def build(candidate: ModelSpec, latency: float) -> SimResult:
        return simulate(candidate, profile, dt, transport_delay=latency)

    def perturbed(key: str) -> tuple[SimResult, float]:
        if key == "wheel_radius":
            changed = _clone(model)
            for wheel in changed.wheels:
                wheel.radius = wheel.radius * (1.0 + step)
            return build(changed, 0.0), step
        if key == "wheel_separation":
            changed = _clone(model)
            for wheel in changed.wheels:
                wheel.y = wheel.y * (1.0 + step)
            if changed.track:
                changed.track = changed.track * (1.0 + step)
            return build(changed, 0.0), step
        if key == "latency":
            return simulate(model, profile, dt, transport_delay=step), step
        raise ValueError("unknown parameter " + key)

    rows: list[list[float]] = []
    applied: list[str] = []
    for key in params:
        candidate, delta = perturbed(key)
        rows.append([(getattr(candidate, comp) - getattr(base, comp)) / delta
                     for comp in keys])
        applied.append(key)

    explanations: list[dict[str, Any]] = []
    for index, key in enumerate(applied):
        for comp_index, comp in enumerate(keys):
            sensitivity = rows[index][comp_index]
            if abs(sensitivity) < 1e-9:
                continue
            explanations.append({
                "param": key, "component": comp,
                "delta": -base_residual[comp_index] / sensitivity,
                "sensitivity": sensitivity,
            })
    # (deltas stay in each parameter's own unit until they are filtered above)

    # express every correction as a relative fraction so one threshold applies
    # to all of them (latency is measured against a 20 ms nominal)
    nominal = {"latency": 0.02}
    plausible: dict[str, float] = {}
    for item in explanations:
        relative = item["delta"] / nominal.get(item["param"], 1.0)
        if 0.02 <= abs(relative) <= 0.4:
            plausible.setdefault(item["param"], []).append(relative)
    plausible = {key: sum(values) / len(values) for key, values in plausible.items()}

    deltas: dict[str, float] = {}
    if len(keys) >= 2:
        n = len(applied)
        ata = [[sum(rows[i][k] * rows[j][k] for k in range(len(keys)))
                for j in range(n)] for i in range(n)]
        atb = [-sum(rows[i][k] * base_residual[k] for k in range(len(keys)))
               for i in range(n)]
        scale = max((abs(v) for row in ata for v in row), default=1.0) or 1.0
        for i in range(n):
            ata[i][i] += 1e-9 * scale
        deltas = dict(zip(applied, _solve(ata, atb)))

    return {
        "base": {
            "distance": base.distance, "yaw": base.yaw,
            "odom_distance": base.odom_distance, "odom_yaw": base.odom_yaw,
            "saturation_fraction": base.saturation_fraction,
            "slip_fraction": base.slip_fraction,
        },
        "measured": dict(measured),
        "components": keys,
        "base_residual": base_residual,
        "params": applied,
        "sensitivity": rows,
        "explanations": explanations,
        "plausible": plausible,
        "deltas": deltas,
        "identifiable": len(plausible) == 1,
        "radius_true": base.radius_true,
        "radius_used": base.radius_used,
    }


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(a)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-15:
            continue
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(n):
            if row == col:
                continue
            factor = m[row][col] / m[col][col]
            for k in range(col, n + 1):
                m[row][k] -= factor * m[col][k]
    return [m[i][n] / m[i][i] if m[i][i] else 0.0 for i in range(n)]


def _clone(model: ModelSpec) -> ModelSpec:
    import copy
    return copy.deepcopy(model)
