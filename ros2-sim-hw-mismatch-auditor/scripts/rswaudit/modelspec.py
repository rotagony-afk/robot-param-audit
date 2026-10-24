"""Universal robot model: one description, four importers.

The auditor used to understand URDF and differential drive. This module makes it
understand any model that states the same physical facts:

    .urdf / .xacro   ROS description
    .xml             MJCF (MuJoCo) or SDF (Gazebo), detected by root element
    .json            declarative spec, the "maths model" route: bodies, joints,
                     and a wheel table with position, radius, rolling angle and
                     roller angle

Everything lands in ModelSpec, which is deliberately not a full physics model:
it is the set of numbers that decide whether the machine can do what it is told.
That is exactly the set the auditor needs to compare.

Unsupported constructs are reported in notes / unresolved instead of being
guessed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .evidence import Evidence
from .mechanisms import ArmJoint, ArmSpec, LidarSpec, RotorSpec
from .scanners_static import RobotModel, extract_urdf, line_text, scan_xml_tags

GRAVITY = 9.81


@dataclass
class Body:
    name: str
    mass: float | None = None
    com: tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia: dict[str, float] = field(default_factory=dict)
    evidence: Evidence | None = None


@dataclass
class Joint:
    name: str
    kind: str
    parent: str = ""
    child: str = ""
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lower: float = 0.0
    upper: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    evidence: Evidence | None = None


@dataclass
class Wheel:
    name: str
    x: float
    y: float
    radius: float
    alpha_deg: float = 0.0     # rolling direction in the body frame
    roller_deg: float = 0.0    # 0 = fixed wheel, +/-45 = mecanum, 90 = omni
    steerable: bool = False
    slip_free: bool = False    # can slide sideways by design (rollers/caster)
    max_omega: float = 0.0     # rad/s, 0 = unknown
    stall_torque: float = 0.0  # N*m, 0 = unknown
    no_load_omega: float = 0.0
    evidence: dict[str, Evidence] = field(default_factory=dict)

    @property
    def direction_deg(self) -> float:
        return self.alpha_deg + self.roller_deg


@dataclass
class ModelSpec:
    name: str
    source: str
    drive: str = "unknown"          # differential | ackermann | mecanum | omni | skid
    bodies: list[Body] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    wheels: list[Wheel] = field(default_factory=list)
    declared_mass: float | None = None
    declared_inertia_zz: float | None = None
    friction: float = 0.7
    gravity: float = GRAVITY
    wheelbase: float = 0.0
    track: float = 0.0
    notes: list[str] = field(default_factory=list)
    unresolved: list[str] = field(default_factory=list)
    evidence: dict[str, Evidence] = field(default_factory=dict)
    reality: dict[str, float] = field(default_factory=dict)
    arm: ArmSpec | None = None
    rotor: RotorSpec | None = None
    lidar: LidarSpec | None = None

    # -- derived quantities ------------------------------------------------- #
    @property
    def mass(self) -> float:
        if self.declared_mass:
            return self.declared_mass
        return sum(b.mass or 0.0 for b in self.bodies)

    @property
    def total_mass(self) -> float:
        return self.mass if self.mass > 0 else 1.0

    @property
    def inertia_zz(self) -> float:
        if self.declared_inertia_zz:
            return self.declared_inertia_zz
        total = 0.0
        for body in self.bodies:
            izz = body.inertia.get("izz") or 0.0
            total += izz + (body.mass or 0.0) * (body.com[0] ** 2 + body.com[1] ** 2)
        if total <= 0:
            radius = max(self.half_extent(), 0.05)
            total = 0.5 * self.total_mass * radius * radius
        return total

    def half_extent(self) -> float:
        if not self.wheels:
            return 0.1
        return max(math.hypot(w.x, w.y) for w in self.wheels)

    @property
    def wheel_radius(self) -> float:
        radii = [w.radius for w in self.wheels if w.radius > 0]
        return sum(radii) / len(radii) if radii else 0.0

    def summary(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "source": self.source,
            "drive": self.drive,
            "wheels": len(self.wheels),
            "mass": self.mass,
            "inertia_zz": self.inertia_zz,
            "wheel_radius": self.wheel_radius,
            "track": self.track,
            "wheelbase": self.wheelbase,
        }


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def _vec3(text: str, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    parts = str(text).replace(",", " ").split()
    try:
        values = [float(p) for p in parts[:3]]
    except ValueError:
        return default
    while len(values) < 3:
        values.append(0.0)
    return (values[0], values[1], values[2])


def _axis_alpha(axis: tuple[float, float, float]) -> float:
    """Rolling direction of a wheel whose spin axis is given (body frame)."""
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / norm, ay / norm, az / norm
    tx = ay
    ty = -ax
    if abs(tx) < 1e-9 and abs(ty) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(ty, tx))


def _is_vertical(axis: tuple[float, float, float]) -> bool:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    return abs(az) / norm > 0.9


def _guess_drive(wheels: list[Wheel]) -> str:
    if any(w.steerable for w in wheels):
        return "ackermann"
    if any(abs(w.roller_deg) >= 80 for w in wheels):
        return "omni"
    if any(abs(w.roller_deg) in (40, 45, 50) for w in wheels):
        return "mecanum"
    if len(wheels) >= 4:
        return "skid"
    if len(wheels) == 2:
        return "differential"
    return "unknown"


# --------------------------------------------------------------------------- #
# URDF
# --------------------------------------------------------------------------- #

def from_urdf(text: str, rel: str) -> ModelSpec:
    _facts, robot = extract_urdf(text, rel)
    stem = rel.split("/")[-1].rsplit(".", 1)[0] or "urdf_robot"
    spec = ModelSpec(name=stem, source=rel)
    for name, link in robot.links.items():
        if not name:
            continue
        inertia = link.get("inertia") or [0.0, 0.0, 0.0]
        spec.bodies.append(Body(
            name=name, mass=link.get("mass"),
            inertia={"ixx": inertia[0], "iyy": inertia[1], "izz": inertia[2]},
            evidence=link.get("mass_ev") or link.get("inertia_ev")))
    for name, joint in robot.joints.items():
        spec.joints.append(Joint(
            name=name, kind=joint.get("type", ""), parent=joint.get("parent", ""),
            child=joint.get("child", ""), axis=joint.get("axis") or (0.0, 0.0, 1.0),
            origin=joint.get("origin") or (0.0, 0.0, 0.0),
            evidence=joint.get("joint_ev")))

    for name, joint in robot.joints.items():
        child = joint.get("child", "")
        link = robot.links.get(child, {})
        radius = link.get("radius")
        kind = joint.get("type", "")
        if kind not in ("continuous", "revolute") or radius is None:
            continue
        axis = joint.get("axis") or (0.0, 1.0, 0.0)
        origin = joint.get("origin") or (0.0, 0.0, 0.0)
        if _is_vertical(axis):
            # a driving wheel link must spin about a horizontal axis; a vertical
            # axis here is a modelling defect, not a steering joint
            spec.unresolved.append(
                "轮子 joint " + name + " 的旋转轴是竖直方向（" +
                ", ".join("%g" % v for v in axis) +
                "），差速/阿克曼底盘的驱动轮应当绕水平轴旋转")
        spec.wheels.append(Wheel(
            name=name, x=origin[0], y=origin[1], radius=float(radius),
            alpha_deg=_axis_alpha(axis),
            steerable=False,
            slip_free=("caster" in name.lower()),
            evidence={"radius": link.get("radius_ev"),
                      "position": joint.get("origin_ev")},
        ))
    # URDF usually omits the inertial origin; the joint origin is the best
    # available estimate of where a link actually sits, and it is only used for
    # the yaw-inertia term, never for a correctness claim.
    for joint in spec.joints:
        for body in spec.bodies:
            if body.name == joint.child and body.com == (0.0, 0.0, 0.0)                     and joint.origin != (0.0, 0.0, 0.0):
                body.com = joint.origin
    spec.notes.append("URDF: 轮子由「连续/旋转 joint + 子连杆圆柱半径」识别；"
                      "roller 角与电机曲线无法从 URDF 得到，需用 JSON 规格补充；"
                      "连杆质心按 joint origin 近似，仅用于偏航惯量")
    return _finalize(spec)


# --------------------------------------------------------------------------- #
# MJCF / SDF
# --------------------------------------------------------------------------- #

def _rot_euler(deg: tuple[float, float, float]) -> list[list[float]]:
    x, y, z = (math.radians(v) for v in deg)
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]

    def mul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]
    return mul(rz, mul(ry, rx))


def _apply(rot: list[list[float]], vec: tuple[float, float, float]) -> tuple[float, float, float]:
    return (sum(rot[0][i] * vec[i] for i in range(3)),
            sum(rot[1][i] * vec[i] for i in range(3)),
            sum(rot[2][i] * vec[i] for i in range(3)))


def _compose(rot_a, pos_a, rot_b, pos_b):
    moved = _apply(rot_a, pos_b)
    pos = (pos_a[0] + moved[0], pos_a[1] + moved[1], pos_a[2] + moved[2])
    rot = [[sum(rot_a[i][k] * rot_b[k][j] for k in range(3)) for j in range(3)]
           for i in range(3)]
    return rot, pos


def from_mjcf(text: str, rel: str) -> ModelSpec:
    tags = scan_xml_tags(text, rel)
    name = "mjcf_model"
    for tag in tags:
        if tag.name == "mujoco" and tag.attrs.get("model"):
            name = tag.attrs["model"]
    spec = ModelSpec(name=name, source=rel)
    stack: list[dict[str, Any]] = []
    body_stack: list[str] = []

    for tag in tags:
        if tag.closing:
            if stack and tag.name in ("body", "worldbody", "mujoco", "default",
                                      "actuator", "asset", "compiler", "option"):
                stack.pop()
            if tag.name == "body" and body_stack:
                body_stack.pop()
            continue
        line = tag.line
        if tag.name == "body":
            local_pos = _vec3(tag.attrs.get("pos", "0 0 0"))
            local_rot = _rot_euler(_vec3(tag.attrs.get("euler", "0 0 0")))
            if stack:
                rot, pos = _compose(stack[-1]["rot"], stack[-1]["pos"], local_rot, local_pos)
            else:
                rot, pos = local_rot, local_pos
            body_name = tag.attrs.get("name", "body%d" % len(spec.bodies))
            stack.append({"rot": rot, "pos": pos, "name": body_name})
            body_stack.append(body_name)
            spec.bodies.append(Body(
                name=body_name, com=pos,
                evidence=Evidence(rel, line, None, line_text(text, line),
                                  "body " + body_name)))
            continue
        if tag.name in ("worldbody", "mujoco", "default", "actuator", "asset",
                        "compiler", "option", "keyframe", "contact", "sensor"):
            rot = stack[-1]["rot"] if stack else _rot_euler((0, 0, 0))
            pos = stack[-1]["pos"] if stack else (0.0, 0.0, 0.0)
            stack.append({"rot": rot, "pos": pos, "name": tag.name})
            continue
        frame = stack[-1] if stack else {"rot": _rot_euler((0, 0, 0)),
                                         "pos": (0.0, 0.0, 0.0)}
        if tag.name == "inertial" and body_stack:
            mass = float(tag.attrs.get("mass", "0") or 0)
            diag = _vec3(tag.attrs.get("diaginertia", "0 0 0"))
            for body in spec.bodies:
                if body.name == body_stack[-1]:
                    body.mass = mass
                    body.inertia = {"ixx": diag[0], "iyy": diag[1], "izz": diag[2]}
                    body.evidence = Evidence(rel, line, None, line_text(text, line),
                                             "inertial " + body.name)
        elif tag.name == "joint" and body_stack:
            axis = _vec3(tag.attrs.get("axis", "0 0 1"))
            joint_name = tag.attrs.get("name", "joint%d" % len(spec.joints))
            spec.joints.append(Joint(
                name=joint_name, kind=tag.attrs.get("type", "hinge"),
                child=body_stack[-1], axis=_apply(frame["rot"], axis),
                origin=frame["pos"],
                evidence=Evidence(rel, line, None, line_text(text, line),
                                  "joint " + joint_name)))
        elif tag.name == "geom" and body_stack:
            geom_type = tag.attrs.get("type", "")
            size = _vec3(tag.attrs.get("size", "0 0 0"))
            offset = _apply(frame["rot"], _vec3(tag.attrs.get("pos", "0 0 0")))
            world = (frame["pos"][0] + offset[0], frame["pos"][1] + offset[1],
                     frame["pos"][2] + offset[2])
            if geom_type == "cylinder":
                joint = next((j for j in reversed(spec.joints)
                              if j.child == body_stack[-1]), None)
                radius = size[0]
                if joint is not None and radius > 0:
                    spec.wheels.append(Wheel(
                        name=joint.name, x=world[0], y=world[1], radius=radius,
                        alpha_deg=_axis_alpha(joint.axis),
                        slip_free=("caster" in joint.name.lower()
                                   or "caster" in body_stack[-1].lower()),
                        evidence={
                            "radius": Evidence(rel, line, None, line_text(text, line),
                                               "geom cylinder " + body_stack[-1]),
                            "position": Evidence(rel, line, None, line_text(text, line),
                                                 "body " + body_stack[-1]),
                        }))
        elif tag.name == "option":
            parts = tag.attrs.get("gravity", "0 0 -9.81").split()
            if len(parts) == 3:
                spec.gravity = abs(float(parts[2]))
        elif tag.name == "motor":
            joint_name = tag.attrs.get("joint", "")
            gear = float(tag.attrs.get("gear", "1") or 1)
            for wheel in spec.wheels:
                if wheel.name == joint_name:
                    wheel.stall_torque = gear
        elif tag.name == "include":
            spec.unresolved.append("MJCF include 未展开: "
                                   + tag.attrs.get("file", "?"))
    spec.notes.append("MJCF: 轮子由「hinge joint + cylinder geom」识别；"
                      "include 不展开；actuator 力矩-转速曲线不完整时按未知处理")
    return _finalize(spec)


# --------------------------------------------------------------------------- #
# SDF
# --------------------------------------------------------------------------- #

def from_sdf(text: str, rel: str) -> ModelSpec:
    tags = scan_xml_tags(text, rel)
    spec = ModelSpec(name="sdf_model", source=rel)
    link_radius: dict[str, float] = {}
    link_radius_ev: dict[str, Evidence] = {}
    joints: list[dict[str, Any]] = []
    cur_link = ""
    cur_joint: dict[str, Any] = {}
    in_axis = False
    for tag in tags:
        line = tag.line
        if tag.closing:
            if tag.name == "joint" and cur_joint:
                joints.append(cur_joint)
                cur_joint = {}
            if tag.name == "axis":
                in_axis = False
            continue
        if tag.name == "model" and tag.attrs.get("name"):
            spec.name = tag.attrs["name"]
        elif tag.name == "link":
            cur_link = tag.attrs.get("name", "")
            spec.bodies.append(Body(
                name=cur_link,
                evidence=Evidence(rel, line, None, line_text(text, line),
                                  "link " + cur_link)))
        elif tag.name == "mass" and cur_link:
            value = float(tag.attrs.get("value", "0") or 0)
            for body in spec.bodies:
                if body.name == cur_link:
                    body.mass = value
                    body.evidence = Evidence(rel, line, None, line_text(text, line),
                                             "mass " + cur_link)
        elif tag.name == "inertia" and cur_link:
            for body in spec.bodies:
                if body.name == cur_link:
                    body.inertia = {k: float(tag.attrs.get(k, "0") or 0)
                                    for k in ("ixx", "iyy", "izz")}
        elif tag.name == "radius" and cur_link:
            value = (tag.text or "").strip()
            try:
                link_radius[cur_link] = float(value)
                link_radius_ev[cur_link] = Evidence(
                    rel, line, None, line_text(text, line), "cylinder radius " + cur_link)
            except ValueError:
                pass
        elif tag.name == "joint":
            cur_joint = {
                "name": tag.attrs.get("name", ""),
                "type": tag.attrs.get("type", ""),
                "evidence": Evidence(rel, line, None, line_text(text, line),
                                     "joint " + tag.attrs.get("name", "")),
            }
        elif tag.name == "parent" and cur_joint:
            cur_joint["parent"] = (tag.text or "").strip()
        elif tag.name == "child" and cur_joint:
            cur_joint["child"] = (tag.text or "").strip()
        elif tag.name == "pose" and cur_joint:
            cur_joint["pose"] = _vec3(tag.text or "0 0 0")
        elif tag.name == "axis" and cur_joint:
            in_axis = True
        elif tag.name == "xyz" and cur_joint and in_axis:
            cur_joint["axis"] = _vec3(tag.text or "0 0 1")
        elif tag.name == "gravity" and tag.text:
            spec.gravity = abs(float((tag.text or "0 0 -9.81").split()[-1]))

    for joint in joints:
        spec.joints.append(Joint(
            name=joint.get("name", ""), kind=joint.get("type", ""),
            parent=joint.get("parent", ""), child=joint.get("child", ""),
            axis=joint.get("axis", (0.0, 0.0, 1.0)),
            origin=joint.get("pose", (0.0, 0.0, 0.0)),
            evidence=joint.get("evidence")))
        child = joint.get("child", "")
        radius = link_radius.get(child)
        if radius and joint.get("type", "") in ("revolute", "continuous"):
            axis = joint.get("axis", (0.0, 1.0, 0.0))
            pose = joint.get("pose", (0.0, 0.0, 0.0))
            spec.wheels.append(Wheel(
                name=joint.get("name", ""), x=pose[0], y=pose[1], radius=radius,
                alpha_deg=_axis_alpha(axis),
                steerable=_is_vertical(axis),
                slip_free=("caster" in child.lower()),
                evidence={"radius": link_radius_ev.get(child),
                          "position": joint.get("evidence")}))
    spec.notes.append("SDF: 轮子由「revolute joint + 子连杆 cylinder 半径」识别；"
                      "嵌套 <model> 与 <include> 不展开")
    return _finalize(spec)


# --------------------------------------------------------------------------- #
# declarative JSON spec  (the "maths model" route)
# --------------------------------------------------------------------------- #

def from_json(doc: dict, rel: str) -> ModelSpec:
    spec = ModelSpec(name=str(doc.get("name", "spec_model")), source=rel,
                     drive=str(doc.get("drive", "unknown")))
    chassis = doc.get("chassis") or {}
    if "mass" in chassis:
        spec.declared_mass = float(chassis["mass"])
    elif "mass" in doc:
        spec.declared_mass = float(doc["mass"])
    if "inertia_zz" in chassis:
        spec.declared_inertia_zz = float(chassis["inertia_zz"])
    spec.wheelbase = float(doc.get("wheelbase", chassis.get("wheelbase", 0.0)) or 0.0)
    spec.track = float(doc.get("track", chassis.get("track", 0.0)) or 0.0)
    env = doc.get("env") or {}
    spec.friction = float(env.get("friction", doc.get("friction", 0.7)))
    spec.gravity = float(env.get("gravity", GRAVITY))
    default_radius = float(doc.get("wheel_radius", 0.0) or 0.0)
    motor = doc.get("motor") or doc.get("wheel_motor") or {}
    for index, item in enumerate(doc.get("wheels") or []):
        roller = float(item.get("roller_deg", 0.0))
        spec.wheels.append(Wheel(
            name=str(item.get("name", "w%d" % index)),
            x=float(item.get("x", 0.0)), y=float(item.get("y", 0.0)),
            radius=float(item.get("radius", default_radius) or 0.0),
            alpha_deg=float(item.get("rolling_deg", item.get("alpha_deg", 0.0))),
            roller_deg=roller,
            steerable=bool(item.get("steerable", False)),
            slip_free=bool(item.get("slip_free", abs(roller) > 1e-9)),
            max_omega=float(item.get("max_omega", doc.get("max_omega", 0.0)) or 0.0),
            stall_torque=float(item.get("stall_torque",
                                        motor.get("stall_torque", 0.0)) or 0.0),
            no_load_omega=float(item.get("no_load_omega",
                                         motor.get("no_load_omega", 0.0)) or 0.0),
        ))
    for body in doc.get("bodies") or []:
        spec.bodies.append(Body(
            name=str(body.get("name", "b%d" % len(spec.bodies))),
            mass=float(body["mass"]) if "mass" in body else None,
            com=tuple(body.get("com", (0.0, 0.0, 0.0))),
            inertia=dict(body.get("inertia", {})),
        ))
    for key, value in (doc.get("reality") or {}).items():
        try:
            spec.reality[str(key)] = float(value)
        except (TypeError, ValueError):
            pass

    arm_doc = doc.get("arm")
    if isinstance(arm_doc, dict):
        joints: list[ArmJoint] = []
        for index, item in enumerate(arm_doc.get("joints") or []):
            origin = item.get("origin") or (0.0, 0.0, 0.0)
            if isinstance(origin, dict):
                xyz = origin.get("xyz", (0.0, 0.0, 0.0))
                rpy = origin.get("rpy", (0.0, 0.0, 0.0))
            else:
                xyz = origin
                rpy = item.get("rpy") or (0.0, 0.0, 0.0)
            kind = str(item.get("type", "revolute"))
            # the spec uses human units: degrees for angles, metres for lengths
            conv = 1.0 if kind == "prismatic" else math.pi / 180.0
            limits = item.get("limits") or (-180.0, 180.0)
            joints.append(ArmJoint(
                name=str(item.get("name", "joint%d" % (index + 1))),
                axis=tuple(item.get("axis", (0.0, 0.0, 1.0))),
                origin=tuple(xyz),
                rpy=tuple(math.radians(float(v)) for v in rpy),
                kind=kind,
                lower=float(limits[0]) * conv, upper=float(limits[1]) * conv,
                effort=float(item.get("effort", 0.0) or 0.0),
                velocity=float(item.get("velocity", 0.0) or 0.0) * conv))
        spec.arm = ArmSpec(
            name=str(arm_doc.get("name", spec.name)),
            joints=joints,
            tip_offset=tuple(arm_doc.get("tip_offset", (0.0, 0.0, 0.0))),
            payload_kg=float(arm_doc.get("payload_kg", doc.get("payload_kg", 0.0)) or 0.0))
        spec.drive = "arm"
        spec.notes.append("机械臂：JSON 里 limits/velocity 用度（直线关节用米），"
                          "origin.rpy 用度；工具内部统一换算成弧度与米")

    rotor_doc = doc.get("rotor")
    if isinstance(rotor_doc, dict):
        spec.rotor = RotorSpec(
            name=str(rotor_doc.get("name", spec.name)),
            diameter_m=float(rotor_doc.get("diameter_m", 0.254)),
            count=int(rotor_doc.get("count", 4)),
            kT=float(rotor_doc.get("kT", 0.0)),
            kQ=float(rotor_doc.get("kQ", 0.0)),
            max_rpm=float(rotor_doc.get("max_rpm", 0.0)),
            motor_torque_limit=float(rotor_doc.get("motor_torque_limit", 0.0)),
            declared_max_thrust=float(rotor_doc.get("declared_max_thrust", 0.0)),
            total_mass_kg=float(rotor_doc.get("total_mass_kg",
                                              doc.get("mass", 0.0)) or 0.0))
        spec.drive = "rotor"
        spec.notes.append("螺旋桨：T = kT*w^2、Q = kQ*w^2，系数来自桨厂家实测曲线")

    lidar_doc = doc.get("lidar")
    if isinstance(lidar_doc, dict):
        spec.lidar = LidarSpec(
            name=str(lidar_doc.get("name", spec.name)),
            fov_h_deg=float(lidar_doc.get("fov_h_deg", 360.0)),
            fov_v_deg=float(lidar_doc.get("fov_v_deg", 30.0)),
            az_res_deg=float(lidar_doc.get("az_res_deg", 0.2)),
            vert_lines=int(lidar_doc.get("vert_lines", 16)),
            rate_hz=float(lidar_doc.get("rate_hz", 10.0)),
            declared_points_per_s=float(lidar_doc.get("declared_points_per_s", 0.0)),
            range_min_m=float(lidar_doc.get("range_min_m", 0.3)),
            range_max_m=float(lidar_doc.get("range_max_m", 100.0)),
            accuracy_m=float(lidar_doc.get("accuracy_m", 0.03)),
            range_resolution_m=float(lidar_doc.get("range_resolution_m", 0.01)),
            divergence_mrad=float(lidar_doc.get("divergence_mrad", 3.0)),
            mount_height_m=float(lidar_doc.get("mount_height_m", 0.5)),
            mount_pitch_deg=float(lidar_doc.get("mount_pitch_deg", 0.0)))
        if spec.arm is None and spec.rotor is None:
            spec.drive = "lidar"
        spec.notes.append("激光雷达：点数/秒 = 水平视场/角分辨率 × 线数 × 帧率")

    spec.notes.append("JSON 规格：数值直接来自描述文件，适合仿真模型尚未建立时的"
                      "纯数学/参数模型")
    return _finalize(spec)


# --------------------------------------------------------------------------- #
# finalise, dispatch, reality overlay
# --------------------------------------------------------------------------- #

def _finalize(spec: ModelSpec) -> ModelSpec:
    if spec.drive in ("", "auto", "unknown"):
        spec.drive = _guess_drive(spec.wheels)
    if not spec.track:
        ys = [w.y for w in spec.wheels]
        if ys:
            spec.track = max(ys) - min(ys)
    if not spec.wheelbase:
        xs = [w.x for w in spec.wheels]
        if xs:
            spec.wheelbase = max(xs) - min(xs)
    if not spec.wheels and spec.arm is None and spec.rotor is None             and spec.lidar is None:
        spec.unresolved.append("没有识别到任何机构：请确认模型是轮式底盘/机械臂/螺旋桨/"
                               "激光雷达，或改用 JSON 规格显式声明")
    return spec


def load_model(path: str | Path) -> ModelSpec:
    full = Path(path)
    if not full.exists():
        raise FileNotFoundError("模型文件不存在: " + str(full))
    suffix = full.suffix.lower()
    text = full.read_text(encoding="utf-8", errors="replace")
    rel = full.name
    if suffix in (".urdf", ".xacro"):
        return from_urdf(text, rel)
    if suffix == ".json":
        return from_json(json.loads(text), rel)
    if suffix in (".xml", ".sdf", ".world"):
        head = text[:4000].lower()
        if "<mujoco" in head:
            return from_mjcf(text, rel)
        if "<sdf" in head or "<model" in head:
            return from_sdf(text, rel)
        raise ValueError("无法判断 XML 类型（需要 mujoco 或 sdf/model 根元素）: "
                         + str(full))
    raise ValueError("不支持的模型格式: " + suffix
                     + "（支持 .urdf/.xacro/.xml(MJCF,SDF)/.json）")


def apply_reality(spec: ModelSpec, reality: dict[str, float]) -> list[str]:
    """Record measured values without destroying the model's own numbers."""
    applied: list[str] = []
    for key, value in (reality or {}).items():
        try:
            spec.reality[str(key)] = float(value)
            applied.append(str(key))
        except (TypeError, ValueError):
            continue
    return applied


def effective_radius(spec: ModelSpec) -> float:
    if "wheel_radius" in spec.reality:
        return spec.reality["wheel_radius"]
    return spec.wheel_radius


def effective_mass(spec: ModelSpec) -> float:
    if "mass" in spec.reality:
        return spec.reality["mass"]
    return spec.total_mass


def effective_friction(spec: ModelSpec) -> float:
    if "friction" in spec.reality:
        return spec.reality["friction"]
    return spec.friction
