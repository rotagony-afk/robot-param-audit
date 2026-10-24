"""Mesh geometry: turn a 3D model file into mass / inertia / size facts.

Supports binary and ASCII STL plus Wavefront OBJ, using only the standard
library. The point is not rendering: it is to answer a question a human cannot
answer by eye, namely "does the mass and inertia written in the robot model
actually match the shape that was modelled?".

Volume, centroid and the inertia tensor are computed with the classic
signed-tetrahedron decomposition (the same method used by CAD kernels): the
triangles must form a closed, consistently wound surface for the result to be
meaningful, which is verified through the signed volume being positive and the
surface being watertight in the sense that every edge is shared by two
triangles. Both checks are reported rather than silently trusted.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from pathlib import Path

Vec = tuple[float, float, float]


@dataclass
class MeshInfo:
    path: str
    triangles: int
    vertices: list[Vec]
    bbox_min: Vec
    bbox_max: Vec
    volume: float
    area: float
    centroid: Vec
    closed: bool
    duplicate_vertices: float
    warnings: list[str] = field(default_factory=list)

    def inertia(self, density: float) -> tuple[float, Vec, dict[str, float]]:
        """Return (mass, centre of mass, inertia tensor about the CoM)."""
        if self.volume <= 0:
            return 0.0, self.centroid, {"ixx": 0.0, "iyy": 0.0, "izz": 0.0,
                                        "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
        mass = density * self.volume
        ixx = iyy = izz = ixy = ixz = iyz = 0.0
        cx, cy, cz = self.centroid
        tris = self._triangles()
        for a, b, c in tris:
            v = (a[0] * (b[1] * c[2] - b[2] * c[1])
                 - a[1] * (b[0] * c[2] - b[2] * c[0])
                 + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
            # second moments of the tetrahedron (origin, a, b, c)
            ixx += v * (a[1] ** 2 + b[1] ** 2 + c[1] ** 2 + a[1] * b[1]
                        + a[1] * c[1] + b[1] * c[1]
                        + a[2] ** 2 + b[2] ** 2 + c[2] ** 2 + a[2] * b[2]
                        + a[2] * c[2] + b[2] * c[2])
            iyy += v * (a[0] ** 2 + b[0] ** 2 + c[0] ** 2 + a[0] * b[0]
                        + a[0] * c[0] + b[0] * c[0]
                        + a[2] ** 2 + b[2] ** 2 + c[2] ** 2 + a[2] * b[2]
                        + a[2] * c[2] + b[2] * c[2])
            izz += v * (a[0] ** 2 + b[0] ** 2 + c[0] ** 2 + a[0] * b[0]
                        + a[0] * c[0] + b[0] * c[0]
                        + a[1] ** 2 + b[1] ** 2 + c[1] ** 2 + a[1] * b[1]
                        + a[1] * c[1] + b[1] * c[1])
            # integral of p_i p_j: V/20 * [ 2*(sum of squares) + all cross terms ]
            ixy -= v * (2 * (a[0] * a[1] + b[0] * b[1] + c[0] * c[1])
                        + a[0] * b[1] + b[0] * a[1] + a[0] * c[1] + c[0] * a[1]
                        + b[0] * c[1] + c[0] * b[1]) / 2.0
            ixz -= v * (2 * (a[0] * a[2] + b[0] * b[2] + c[0] * c[2])
                        + a[0] * b[2] + b[0] * a[2] + a[0] * c[2] + c[0] * a[2]
                        + b[0] * c[2] + c[0] * b[2]) / 2.0
            iyz -= v * (2 * (a[1] * a[2] + b[1] * b[2] + c[1] * c[2])
                        + a[1] * b[2] + b[1] * a[2] + a[1] * c[2] + c[1] * a[2]
                        + b[1] * c[2] + c[1] * b[2]) / 2.0
        # the tetrahedron second moment is V/10 * [sum p_i p_j + sum_{k<l} p_i p_j]
        ixx *= density / 10.0
        iyy *= density / 10.0
        izz *= density / 10.0
        ixy *= density / 10.0
        ixz *= density / 10.0
        iyz *= density / 10.0
        # parallel-axis shift from the origin to the centroid
        ixx -= mass * (cy * cy + cz * cz)
        iyy -= mass * (cx * cx + cz * cz)
        izz -= mass * (cx * cx + cy * cy)
        ixy += mass * cx * cy
        ixz += mass * cx * cz
        iyz += mass * cy * cz
        tensor = {"ixx": ixx, "iyy": iyy, "izz": izz,
                  "ixy": ixy, "ixz": ixz, "iyz": iyz}
        return mass, self.centroid, tensor

    def _triangles(self) -> list[tuple[Vec, Vec, Vec]]:
        tris = []
        for index in range(0, len(self.vertices) - 2, 3):
            tris.append((self.vertices[index], self.vertices[index + 1],
                         self.vertices[index + 2]))
        return tris


def _bbox(vertices: list[Vec]) -> tuple[Vec, Vec]:
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _surface_area(vertices: list[Vec]) -> float:
    area = 0.0
    for i in range(0, len(vertices) - 2, 3):
        a, b, c = vertices[i], vertices[i + 1], vertices[i + 2]
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        cx = uy * vz - uz * vy
        cy = uz * vx - ux * vz
        cz = ux * vy - uy * vx
        area += 0.5 * math.sqrt(cx * cx + cy * cy + cz * cz)
    return area


def _volume_centroid(vertices: list[Vec]) -> tuple[float, Vec]:
    volume = 0.0
    cx = cy = cz = 0.0
    for i in range(0, len(vertices) - 2, 3):
        a, b, c = vertices[i], vertices[i + 1], vertices[i + 2]
        v = (a[0] * (b[1] * c[2] - b[2] * c[1])
             - a[1] * (b[0] * c[2] - b[2] * c[0])
             + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        volume += v
        cx += v * (a[0] + b[0] + c[0]) / 4.0
        cy += v * (a[1] + b[1] + c[1]) / 4.0
        cz += v * (a[2] + b[2] + c[2]) / 4.0
    if abs(volume) < 1e-12:
        return 0.0, (0.0, 0.0, 0.0)
    return volume, (cx / volume, cy / volume, cz / volume)


def _winding_ok(vertices: list[Vec]) -> tuple[bool, float]:
    """Check that the surface is outward-facing and (roughly) closed."""
    edge_use: dict[tuple, int] = {}
    for i in range(0, len(vertices) - 2, 3):
        tri = [vertices[i], vertices[i + 1], vertices[i + 2]]
        for j in range(3):
            a = tri[j]
            b = tri[(j + 1) % 3]
            key = (round(a[0], 6), round(a[1], 6), round(a[2], 6),
                   round(b[0], 6), round(b[1], 6), round(b[2], 6))
            reverse = (key[3], key[4], key[5], key[0], key[1], key[2])
            if reverse in edge_use:
                edge_use[reverse] -= 1
            else:
                edge_use[key] = edge_use.get(key, 0) + 1
    open_edges = sum(1 for count in edge_use.values() if count > 0)
    total = max(len(edge_use), 1)
    return open_edges / total < 0.02, open_edges / total


def load_stl(path: Path) -> tuple[list[Vec], list[str]]:
    data = path.read_bytes()
    warnings: list[str] = []
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if 84 + count * 50 == len(data):
            vertices: list[Vec] = []
            for index in range(count):
                base = 84 + index * 50 + 12
                values = struct.unpack("<9f", data[base:base + 36])
                vertices.append((values[0], values[1], values[2]))
                vertices.append((values[3], values[4], values[5]))
                vertices.append((values[6], values[7], values[8]))
            return vertices, warnings
        warnings.append("binary STL size mismatch, falling back to ASCII parse")
    text = data.decode("utf-8", errors="replace")
    vertices = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not vertices:
        warnings.append("no triangles found")
    return vertices, warnings


def load_obj(path: Path) -> tuple[list[Vec], list[str]]:
    vertices: list[Vec] = []
    faces: list[tuple[int, ...]] = []
    warnings: list[str] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v" and len(parts) >= 4:
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif parts[0] == "f" and len(parts) >= 4:
            idx = []
            for token in parts[1:]:
                raw = token.split("/")[0]
                if raw:
                    value = int(raw)
                    idx.append(value - 1 if value > 0 else len(vertices) + value)
            if len(idx) >= 3:
                faces.append(tuple(idx))
    if not faces:
        warnings.append("no faces found")
    flat: list[Vec] = []
    for face in faces:
        for index in range(1, len(face) - 1):
            flat.extend([vertices[face[0]], vertices[face[index]],
                         vertices[face[index + 1]]])
    return flat, warnings


def load_mesh(path: str | Path) -> MeshInfo:
    full = Path(path)
    suffix = full.suffix.lower()
    if suffix == ".stl":
        raw, warnings = load_stl(full)
    elif suffix == ".obj":
        raw, warnings = load_obj(full)
    else:
        raise ValueError("不支持的网格格式: " + suffix + "（支持 .stl / .obj）")
    if not raw:
        raise ValueError("网格为空或无法解析: " + str(full))
    volume, centroid = _volume_centroid(raw)
    closed, open_ratio = _winding_ok(raw)
    if volume < 0:
        warnings.append("网格法向朝内（volume < 0），已取绝对值，请确认模型导出设置")
        # the centroid is (sum v*c)/(sum v) and is therefore sign invariant
        volume = -volume
    bmin, bmax = _bbox(raw)
    return MeshInfo(
        path=full.name, triangles=len(raw) // 3, vertices=raw,
        bbox_min=bmin, bbox_max=bmax, volume=volume, area=_surface_area(raw),
        centroid=centroid, closed=closed, duplicate_vertices=open_ratio,
        warnings=warnings,
    )
