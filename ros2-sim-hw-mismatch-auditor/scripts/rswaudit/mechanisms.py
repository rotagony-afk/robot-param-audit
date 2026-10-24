"""Mechanism models beyond wheeled chassis: serial arms, rotors, lidars.

Three small, exact models that answer the same question for very different
hardware: does the number written in the spec actually hold up?

* ArmSpec  - forward kinematics, geometric Jacobian, damped-least-squares IK,
             static joint torque for a payload, and a manipulability index.
* RotorSpec- actuator-disk momentum theory in coefficient form:
             T = kT*w^2, Q = kQ*w^2, P = Q*w.
* LidarSpec- scan geometry: points per second, beam spot size, ground blind
             radius for a given mounting height and pitch.

No contact dynamics and no unsteady aerodynamics are modelled; the docstrings
say so explicitly instead of implying more.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

Vec3 = tuple[float, float, float]
Mat4 = list[list[float]]


# --------------------------------------------------------------------------- #
# tiny 4x4 helpers
# --------------------------------------------------------------------------- #

def eye4() -> Mat4:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def matmul(a: Mat4, b: Mat4) -> Mat4:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def translate(x: float, y: float, z: float) -> Mat4:
    m = eye4()
    m[0][3], m[1][3], m[2][3] = x, y, z
    return m


def rot_axis(axis: Vec3, angle: float) -> Mat4:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-12:
        return eye4()
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_rpy(rpy: Vec3) -> Mat4:
    r, p, y = rpy
    return matmul(rot_axis((0.0, 0.0, 1.0), y),
                  matmul(rot_axis((0.0, 1.0, 0.0), p),
                         rot_axis((1.0, 0.0, 0.0), r)))


def pos_of(m: Mat4) -> Vec3:
    return (m[0][3], m[1][3], m[2][3])


def axis_of(m: Mat4) -> Vec3:
    """z axis of a frame (kept for callers that rotate about local z)."""
    return (m[0][2], m[1][2], m[2][2])


def rotate_vec(m: Mat4, v: Vec3) -> Vec3:
    """Rotate a vector by the rotation part of a homogeneous transform."""
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


# --------------------------------------------------------------------------- #
# serial arm
# --------------------------------------------------------------------------- #

@dataclass
class ArmJoint:
    name: str
    axis: Vec3 = (0.0, 0.0, 1.0)
    origin: Vec3 = (0.0, 0.0, 0.0)
    rpy: Vec3 = (0.0, 0.0, 0.0)
    kind: str = "revolute"          # revolute | prismatic
    lower: float = -math.pi
    upper: float = math.pi
    effort: float = 0.0             # N*m (or N for prismatic), 0 = unknown
    velocity: float = 0.0           # rad/s (or m/s)

    @property
    def unit(self) -> str:
        return "m" if self.kind == "prismatic" else "rad"

    def usable_limits(self) -> tuple[float, float]:
        low, high = self.lower, self.upper
        if self.kind == "prismatic":
            return (low, high)
        if abs(high - low) > 2.0 * math.pi + 1e-6:
            return (-math.pi, math.pi)
        return (low, high)


@dataclass
class ArmSpec:
    name: str
    joints: list[ArmJoint] = field(default_factory=list)
    tip_offset: Vec3 = (0.0, 0.0, 0.0)
    payload_kg: float = 0.0

    @property
    def dof(self) -> int:
        return len(self.joints)

    def forward(self, q: list[float]) -> Mat4:
        frame = eye4()
        for joint, value in zip(self.joints, q):
            frame = matmul(frame, translate(*joint.origin))
            frame = matmul(frame, rot_rpy(joint.rpy))
            if joint.kind == "prismatic":
                frame = matmul(frame, translate(*(axis * value for axis in joint.axis)))
            else:
                frame = matmul(frame, rot_axis(joint.axis, value))
        return matmul(frame, translate(*self.tip_offset))

    def frames(self, q: list[float]) -> list[Mat4]:
        """World transform of every joint frame, evaluated before its own motion.

        The running frame must accumulate the motion of every previous joint;
        forgetting that silently evaluates the Jacobian at q = 0.
        """
        out: list[Mat4] = []
        frame = eye4()
        for joint, value in zip(self.joints, q):
            frame = matmul(frame, translate(*joint.origin))
            frame = matmul(frame, rot_rpy(joint.rpy))
            out.append([row[:] for row in frame])
            if joint.kind == "prismatic":
                frame = matmul(frame, translate(*(axis * value for axis in joint.axis)))
            else:
                frame = matmul(frame, rot_axis(joint.axis, value))
        return out

    def jacobian(self, q: list[float]) -> list[list[float]]:
        """6 x n geometric Jacobian in the base frame."""
        frames = self.frames(q)
        tip = pos_of(self.forward(q))
        rows = [[0.0] * self.dof for _ in range(6)]
        for index, (joint, frame) in enumerate(zip(self.joints, frames)):
            # the joint axis is the local axis carried into the base frame; for a
            # joint about x or y it is NOT the z column of the frame
            direction = rotate_vec(frame, joint.axis)
            if joint.kind == "prismatic":
                for axis in range(3):
                    rows[axis][index] = direction[axis]
                continue
            # the joint axis passes through the frame position after the origin
            origin = pos_of(frame)
            rx, ry, rz = tip[0] - origin[0], tip[1] - origin[1], tip[2] - origin[2]
            rows[0][index] = direction[1] * rz - direction[2] * ry
            rows[1][index] = direction[2] * rx - direction[0] * rz
            rows[2][index] = direction[0] * ry - direction[1] * rx
            rows[3][index] = direction[0]
            rows[4][index] = direction[1]
            rows[5][index] = direction[2]
        return rows

    def manipulability(self, q: list[float]) -> float:
        jac = self.jacobian(q)
        # sqrt(det(J J^T)) via Gram determinant of the 3 position rows, which is
        # enough to detect a singular position without a linear-algebra package
        rows = jac[:3]
        gram = [[sum(rows[i][k] * rows[j][k] for k in range(self.dof)) for j in range(3)]
                for i in range(3)]
        det = (gram[0][0] * (gram[1][1] * gram[2][2] - gram[1][2] * gram[2][1])
               - gram[0][1] * (gram[1][0] * gram[2][2] - gram[1][2] * gram[2][0])
               + gram[0][2] * (gram[1][0] * gram[2][1] - gram[1][1] * gram[2][0]))
        return math.sqrt(max(det, 0.0))

    def inverse(self, target: Vec3, q0: list[float] | None = None,
                iterations: int = 200, tol: float = 1e-4) -> dict:
        """Damped least squares IK; returns q, residual and whether limits held."""
        q = list(q0) if q0 else [(low + high) / 2.0
                                 for low, high in (j.usable_limits() for j in self.joints)]
        best_q, best_res = list(q), float("inf")
        for _ in range(iterations):
            tip = pos_of(self.forward(q))
            error = [target[0] - tip[0], target[1] - tip[1], target[2] - tip[2]]
            residual = math.sqrt(sum(value * value for value in error))
            if residual < best_res:
                best_q, best_res = list(q), residual
            if residual < tol:
                break
            jac = self.jacobian(q)[:3]
            # solve (J J^T + lambda I) y = e, then dq = J^T y
            lam = 1e-3 + 1e-2 * residual
            gram = [[sum(jac[i][k] * jac[j][k] for k in range(self.dof)) for j in range(3)]
                    for i in range(3)]
            for i in range(3):
                gram[i][i] += lam
            y = _solve(gram, error)
            dq = [sum(jac[i][k] * y[i] for i in range(3)) for k in range(self.dof)]
            step = max((abs(value) for value in dq), default=0.0)
            scale = min(1.0, 0.25 / step) if step > 0.25 else 1.0
            for index, joint in enumerate(self.joints):
                low, high = joint.usable_limits()
                q[index] = min(high, max(low, q[index] + dq[index] * scale))
        clipped = []
        for index, joint in enumerate(self.joints):
            low, high = joint.usable_limits()
            if q[index] <= low + 1e-9 or q[index] >= high - 1e-9:
                clipped.append(joint.name)
        return {"q": best_q, "residual": best_res,
                "at_limit": clipped}

    def static_torque(self, q: list[float], force: Vec3) -> list[float]:
        """Joint torques that hold the given tip force (N) statically."""
        jac = self.jacobian(q)
        return [sum(jac[i][index] * force[i] for i in range(3))
                for index in range(self.dof)]


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


# --------------------------------------------------------------------------- #
# rotor (propeller)
# --------------------------------------------------------------------------- #

@dataclass
class RotorSpec:
    name: str = "rotor"
    diameter_m: float = 0.254
    count: int = 4
    kT: float = 0.0                  # N/(rad/s)^2
    kQ: float = 0.0                  # N*m/(rad/s)^2
    max_rpm: float = 0.0
    motor_torque_limit: float = 0.0  # N*m, 0 = unknown
    declared_max_thrust: float = 0.0  # N per rotor, 0 = unknown
    total_mass_kg: float = 0.0        # 0 = unknown (cannot check hover)
    gravity: float = 9.81

    def omega(self, rpm: float) -> float:
        return rpm * 2.0 * math.pi / 60.0

    def thrust(self, rpm: float) -> float:
        return self.kT * self.omega(rpm) ** 2

    def torque(self, rpm: float) -> float:
        return self.kQ * self.omega(rpm) ** 2

    def power(self, rpm: float) -> float:
        return self.torque(rpm) * self.omega(rpm)

    @property
    def disk_area(self) -> float:
        radius = self.diameter_m / 2.0
        return math.pi * radius * radius

    def disk_loading(self, rpm: float) -> float:
        area = self.disk_area * max(self.count, 1)
        return self.thrust(rpm) * max(self.count, 1) / area if area > 0 else 0.0

    def hover_rpm(self) -> float | None:
        if not self.kT or not self.total_mass_kg or not self.count:
            return None
        needed = self.total_mass_kg * self.gravity / self.count
        return math.sqrt(needed / self.kT) * 60.0 / (2.0 * math.pi)


# --------------------------------------------------------------------------- #
# lidar
# --------------------------------------------------------------------------- #

@dataclass
class LidarSpec:
    name: str = "lidar"
    fov_h_deg: float = 360.0
    fov_v_deg: float = 30.0
    az_res_deg: float = 0.2
    vert_lines: int = 16
    rate_hz: float = 10.0
    declared_points_per_s: float = 0.0
    range_min_m: float = 0.3
    range_max_m: float = 100.0
    accuracy_m: float = 0.03
    range_resolution_m: float = 0.01
    divergence_mrad: float = 3.0
    mount_height_m: float = 0.5
    mount_pitch_deg: float = 0.0        # negative = looking down

    def horizontal_samples(self) -> float:
        if self.az_res_deg <= 0:
            return 0.0
        return self.fov_h_deg / self.az_res_deg

    def points_per_second(self) -> float:
        return self.horizontal_samples() * self.vert_lines * self.rate_hz

    def spot_size(self, distance_m: float) -> float:
        return 2.0 * distance_m * math.tan(math.radians(self.divergence_mrad / 1000.0) / 2.0)

    def ground_blind_radius(self) -> float | None:
        """Radius of the cone on the floor the scanner cannot see, if any."""
        if self.mount_height_m <= 0:
            return None
        down = -self.mount_pitch_deg - self.fov_v_deg / 2.0
        if down >= -0.1:
            return None                      # the lowest beam never hits the floor
        return self.mount_height_m / math.tan(math.radians(-down))

    def max_hits_before_saturation(self, points_per_s: float) -> float:
        if points_per_s <= 0:
            return 0.0
        return points_per_s / max(self.rate_hz, 1e-9)
