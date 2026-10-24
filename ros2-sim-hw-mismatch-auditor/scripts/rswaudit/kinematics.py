"""Universal wheel kinematics: one Jacobian for every wheeled chassis.

The whole point of this module is that differential drive, Ackermann, mecanum,
omni and skid-steer are *not* special cases in code. They are all described by
one wheel table, and one formula covers them:

    a wheel at body position (x, y) whose no-slip direction is d
    contributes the row   1/r * [ cos d,  sin d,  -y*cos d + x*sin d ]

with d = rolling_direction + roller_angle. Setting roller angle to 0 gives an
ordinary wheel, to +/-45 a mecanum wheel, to 90 an omni wheel. Steering simply
makes the rolling direction a function of the commanded twist.

Wheels that cannot slide sideways add a lateral constraint
    -sin d * (vx - w*y) + cos d * (vy + w*x) = 0
and the residual of that constraint tells whether the chassis can physically do
the commanded motion. This is what catches "your model says differential drive
but the command asks for sideways motion".

No third-party dependency: all linear algebra is a 3x3 normal-equation solve.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from .modelspec import ModelSpec, Wheel, effective_friction, effective_mass, effective_radius

ROLLING_RESISTANCE = 0.015
MAX_STEER_DEG = 35.0


@dataclass
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0

    def scaled(self, factor: float) -> "Twist":
        return Twist(self.vx * factor, self.vy * factor, self.w * factor)

    def as_list(self) -> list[float]:
        return [self.vx, self.vy, self.w]


@dataclass
class Feasibility:
    twist: Twist
    wheel_omegas: list[float] = field(default_factory=list)
    wheel_names: list[str] = field(default_factory=list)
    lateral_residual: float = 0.0
    lateral_adjust: float = 0.0
    saturation: float = 0.0          # max |omega| / omega_max, 0 if unknown
    saturated: list[str] = field(default_factory=list)
    required_force: list[float] = field(default_factory=list)
    traction_limit: list[float] = field(default_factory=list)
    traction_margin: float = 1.0     # 1.0 = exactly at the limit
    torque_margin: float = 1.0
    steering: dict[str, float] = field(default_factory=dict)
    steering_limit: float = MAX_STEER_DEG
    problems: list[dict[str, Any]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems


# --------------------------------------------------------------------------- #
# linear algebra (3 unknowns, no numpy)
# --------------------------------------------------------------------------- #

def solve3(a: list[list[float]], b: list[float]) -> tuple[list[float], bool]:
    """Gaussian elimination with partial pivoting; returns (x, singular)."""
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    n = 3
    scale = max((abs(v) for row in a for v in row), default=1.0) or 1.0
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12 * scale:
            return [0.0, 0.0, 0.0], True
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(n):
            if row == col:
                continue
            factor = m[row][col] / m[col][col]
            if factor == 0:
                continue
            for k in range(col, n + 1):
                m[row][k] -= factor * m[col][k]
    return [m[i][n] / m[i][i] for i in range(n)], False


def lstsq3(rows: list[list[float]], rhs: list[float]) -> tuple[list[float], bool, int]:
    """Least squares for 3 unknowns; returns (x, singular, rank-ish)."""
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for row, value in zip(rows, rhs):
        for i in range(3):
            atb[i] += row[i] * value
            for j in range(3):
                ata[i][j] += row[i] * row[j]
    size = max((abs(v) for row in ata for v in row), default=1.0) or 1.0
    for i in range(3):
        ata[i][i] += 1e-9 * size     # tiny ridge keeps singular systems solvable
    solution, singular = solve3(ata, atb)
    rank = sum(1 for i in range(3) if abs(ata[i][i]) > 1e-9 * size)
    return solution, singular, rank


# --------------------------------------------------------------------------- #
# wheel geometry
# --------------------------------------------------------------------------- #

def no_slip_angle(wheel: Wheel, steering: float = 0.0) -> float:
    """Direction in which the wheel's contact point cannot slide."""
    return math.radians(wheel.alpha_deg + wheel.roller_deg + steering)


def wheel_row(wheel: Wheel, steering: float = 0.0) -> list[float]:
    if wheel.radius <= 0:
        return [0.0, 0.0, 0.0]
    angle = no_slip_angle(wheel, steering)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return [cos_a / wheel.radius, sin_a / wheel.radius,
            (-wheel.y * cos_a + wheel.x * sin_a) / wheel.radius]


def slides_sideways(wheel: Wheel) -> bool:
    """True when the wheel has a roller (or is declared slip free).

    A plain wheel cannot move sideways at all; a wheel with rollers at angle
    gamma can, which is why only plain wheels contribute a lateral constraint.
    """
    return wheel.slip_free or abs(wheel.roller_deg) > 1e-6


def lateral_row(wheel: Wheel, steering: float = 0.0) -> list[float]:
    """Constraint row that forbids sideways sliding of a fixed wheel."""
    angle = no_slip_angle(wheel, steering)
    sin_a, cos_a = math.sin(angle), math.cos(angle)
    return [-sin_a, cos_a, sin_a * wheel.y + cos_a * wheel.x]


def jacobian(model: ModelSpec, steering: dict[str, float] | None = None) -> list[list[float]]:
    steering = steering or {}
    return [wheel_row(w, steering.get(w.name, 0.0)) for w in model.wheels]


def wheel_speeds(model: ModelSpec, twist: Twist,
                 steering: dict[str, float] | None = None) -> list[float]:
    rows = jacobian(model, steering)
    values = twist.as_list()
    return [row[0] * values[0] + row[1] * values[1] + row[2] * values[2]
            for row in rows]


def twist_from_speeds(model: ModelSpec, omegas: list[float]) -> tuple[Twist, float]:
    """Best-fit body twist for measured wheel speeds, plus the fit residual."""
    # J already contains 1/r, so the constraint is J * xi = omega (rad/s)
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
# steering
# --------------------------------------------------------------------------- #

def ideal_steering(model: ModelSpec, twist: Twist) -> dict[str, float]:
    """Steering angle each steerable wheel needs for this twist (degrees)."""
    result: dict[str, float] = {}
    steerable = [w for w in model.wheels if w.steerable]
    if not steerable:
        return result
    if abs(twist.w) < 1e-6:
        angle = math.degrees(math.atan2(twist.vy, twist.vx)) if (twist.vx or twist.vy) else 0.0
        return {w.name: angle for w in steerable}
    # instantaneous centre of rotation in the body frame
    icr_x = -twist.vy / twist.w
    icr_y = twist.vx / twist.w
    for wheel in steerable:
        # the wheel rolls tangentially to its radius vector from the ICR, so the
        # rolling direction is that vector rotated by +/- 90 degrees
        rx = wheel.x - icr_x
        ry = wheel.y - icr_y
        if twist.w > 0:
            direction = (-ry, rx)
        else:
            direction = (ry, -rx)
        angle = math.degrees(math.atan2(direction[1], direction[0]))
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        result[wheel.name] = angle
    return result


def ackermann_inner_outer(model: ModelSpec, curvature: float) -> dict[str, float]:
    """Textbook Ackermann angles for a front-steered chassis (for M06 checks)."""
    wheelbase = model.wheelbase or 0.0
    track = model.track or 0.0
    if abs(curvature) < 1e-9 or wheelbase <= 0 or track <= 0:
        return {"inner": 0.0, "outer": 0.0}
    radius = 1.0 / curvature
    inner = math.degrees(math.atan(wheelbase / (radius - track / 2.0)))
    outer = math.degrees(math.atan(wheelbase / (radius + track / 2.0)))
    return {"inner": inner, "outer": outer}


# --------------------------------------------------------------------------- #
# feasibility of a commanded twist
# --------------------------------------------------------------------------- #

def check_twist(model: ModelSpec, twist: Twist,
                acceleration: float = 0.0,
                normal_split: list[float] | None = None) -> Feasibility:
    friction = effective_friction(model)
    mass = effective_mass(model)
    result = Feasibility(twist=twist)
    if not model.wheels:
        result.problems.append({"kind": "no-wheels",
                                "detail": "模型里没有识别到轮子，无法评估运动学"})
        return result

    steered = any(w.steerable for w in model.wheels)
    fixed_rows = [lateral_row(w) for w in model.wheels
                  if not slides_sideways(w) and not w.steerable]
    residual = 0.0
    for row in fixed_rows:
        residual = max(residual, abs(row[0] * twist.vx + row[1] * twist.vy
                                     + row[2] * twist.w))
    result.lateral_residual = residual

    if steered and residual > 0.01:
        # a front-steered car rotating about a point on its rear axle implies a
        # lateral velocity at the body origin. Keep vx and omega as commanded and
        # solve only for the vy the fixed wheels require.
        needed_vy = lateral_velocity_for(fixed_rows, twist)
        if needed_vy is not None:
            result.lateral_adjust = abs(needed_vy - twist.vy)
            twist = Twist(twist.vx, needed_vy, twist.w)
    elif residual > 0.01:
        result.problems.append({
            "kind": "lateral-slip", "detail":
                "该底盘存在不能横向滑动的轮子，但命令里有 " + "%.3f" % residual
                + " m/s 的侧向分量（无滚子/转向自由度无法实现）",
        })

    steering = ideal_steering(model, twist)
    result.steering = steering
    for name, angle in steering.items():
        if abs(angle) > MAX_STEER_DEG + 1e-6:
            result.problems.append({
                "kind": "steering-limit", "wheel": name,
                "detail": "需要转向 " + "%.1f" % angle + "°，超过常见上限 "
                          + "%.0f" % MAX_STEER_DEG + "°",
            })

    omegas = wheel_speeds(model, twist, steering)
    result.wheel_omegas = omegas
    result.wheel_names = [w.name for w in model.wheels]
    result.twist = twist

    # speed limits
    saturated = []
    ratio = 0.0
    for wheel, omega in zip(model.wheels, omegas):
        if wheel.max_omega > 0:
            share = abs(omega) / wheel.max_omega
            ratio = max(ratio, share)
            if share > 1.0 + 1e-6:
                saturated.append(wheel.name)
    result.saturation = ratio
    result.saturated = saturated
    if saturated:
        result.problems.append({
            "kind": "wheel-speed", "detail":
                "轮速超出电机上限：" + ", ".join(saturated) +
                "（最高 " + "%.2f" % ratio + " 倍）",
        })

    # required traction and torque: solve J^T f = wrench for the minimum-norm
    # wheel forces, exactly as a real controller would distribute the load
    resistance = ROLLING_RESISTANCE * mass * model.gravity
    speed = math.hypot(twist.vx, twist.vy)
    drag = 0.5 * 1.225 * 0.6 * speed * speed
    if speed > 1e-9:
        ux, uy = twist.vx / speed, twist.vy / speed
    else:
        ux, uy = 1.0, 0.0
    total_resistance = resistance + drag
    wrench = [mass * acceleration * ux + total_resistance * ux,
              mass * acceleration * uy + total_resistance * uy,
              0.0]
    rows = jacobian(model, steering)
    forces = _min_norm_forces(rows, wrench)
    counts = len(model.wheels)
    split = normal_split or [1.0 / counts] * counts
    result.required_force = forces
    limits = [friction * mass * model.gravity * s for s in split]
    result.traction_limit = limits
    margin = 1.0
    for force, limit in zip(forces, limits):
        if limit > 0:
            margin = max(margin, abs(force) / limit)
    result.traction_margin = margin
    if margin > 1.0:
        result.problems.append({
            "kind": "traction", "detail":
                "需求牵引力超过附着极限 " + "%.2f" % margin + " 倍 → 会打滑，"
                "位移会小于命令值",
        })

    torque_margin = 1.0
    for wheel, force, omega in zip(model.wheels, forces, omegas):
        if wheel.stall_torque <= 0:
            continue
        no_load = wheel.no_load_omega or wheel.max_omega or 1.0
        available = wheel.stall_torque * max(0.0, 1.0 - abs(omega) / no_load)
        needed = abs(force) * wheel.radius
        if available > 0:
            torque_margin = max(torque_margin, needed / available)
    result.torque_margin = torque_margin
    if torque_margin > 1.0:
        result.problems.append({
            "kind": "torque", "detail":
                "需求力矩超过电机能力 " + "%.2f" % torque_margin
                + " 倍 → 到不了这个速度",
        })
    return result


def lateral_velocity_for(rows: list[list[float]], twist: Twist) -> float | None:
    """vy that satisfies every fixed-wheel constraint, with vx and w unchanged."""
    numer = 0.0
    denom = 0.0
    for row in rows:
        a, b, c = row
        if abs(b) < 1e-9:
            continue
        numer += b * (a * twist.vx + c * twist.w)
        denom += b * b
    if denom < 1e-12:
        return None
    return -numer / denom


def project_feasible(rows: list[list[float]], twist: Twist) -> Twist:
    """Closest twist to the command that satisfies every constraint A xi = 0."""
    if not rows:
        return twist
    m = len(rows)
    gram = [[sum(rows[i][k] * rows[j][k] for k in range(3)) for j in range(m)]
            for i in range(m)]
    scale = max((abs(v) for row in gram for v in row), default=1.0) or 1.0
    for i in range(m):
        gram[i][i] += 1e-12 * scale
    values = [row[0] * twist.vx + row[1] * twist.vy + row[2] * twist.w
              for row in rows]
    correction = _solve_any(gram, values)
    adjusted = [twist.vx - sum(rows[i][0] * correction[i] for i in range(m)),
                twist.vy - sum(rows[i][1] * correction[i] for i in range(m)),
                twist.w - sum(rows[i][2] * correction[i] for i in range(m))]
    return Twist(*adjusted)


def _solve_any(a: list[list[float]], b: list[float]) -> list[float]:
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


def _min_norm_forces(rows: list[list[float]], wrench: list[float]) -> list[float]:
    """Minimum-norm wheel forces producing the given body wrench (J^T f = w)."""
    if not rows:
        return []
    gram = [[0.0] * 3 for _ in range(3)]
    for row in rows:
        for i in range(3):
            for j in range(3):
                gram[i][j] += row[i] * row[j]
    scale = max((abs(v) for row in gram for v in row), default=1.0) or 1.0
    for i in range(3):
        gram[i][i] += 1e-9 * scale
    solution, _singular = solve3(gram, wrench)
    return [row[0] * solution[0] + row[1] * solution[1] + row[2] * solution[2]
            for row in rows]


def matrix_rank(rows: list[list[float]]) -> int:
    if not rows:
        return 0
    m = [list(row) for row in rows]
    cols = len(m[0])
    scale = max((abs(v) for row in m for v in row), default=1.0) or 1.0
    rank = 0
    for col in range(cols):
        pivot = max(range(rank, len(m)), key=lambda r: abs(m[r][col]), default=None)
        if pivot is None or abs(m[pivot][col]) < 1e-9 * scale:
            continue
        m[rank], m[pivot] = m[pivot], m[rank]
        for row in range(len(m)):
            if row != rank:
                factor = m[row][col] / m[rank][col]
                for k in range(col, cols):
                    m[row][k] -= factor * m[rank][k]
        rank += 1
        if rank == len(m):
            break
    return rank


def max_speed(model: ModelSpec, direction: tuple[float, float] = (1.0, 0.0),
              limit: float = 5.0) -> float:
    """Largest |v| along a direction before a wheel hits its speed limit."""
    rows = jacobian(model)
    best = limit
    for wheel, row in zip(model.wheels, rows):
        if wheel.max_omega <= 0:
            continue
        unit = row[0] * direction[0] + row[1] * direction[1]
        if abs(unit) < 1e-9:
            continue
        best = min(best, wheel.max_omega / abs(unit))
    return best


def capability_matrix(model: ModelSpec) -> dict[str, Any]:
    """Summarise which of the three planar DoF the wheel layout can produce."""
    lateral = [lateral_row(w) for w in model.wheels
               if not slides_sideways(w) and not w.steerable]
    rank = matrix_rank(lateral)
    dof = 3 - rank
    return {
        "drive": model.drive,
        "wheels": len(model.wheels),
        "dof": dof,
        "lateral_constraints": len(lateral),
        "can_strafe": rank == 0,
        "can_rotate": rank < 3,
        "max_forward": max_speed(model, (1.0, 0.0)),
        "max_lateral": max_speed(model, (0.0, 1.0)),
    }
