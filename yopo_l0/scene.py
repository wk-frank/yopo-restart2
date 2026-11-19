"""场景卡与几何。几何只给仿真（传感器、碰撞真值），绝不进规划器。"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Box:
    name: str
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]

    def distance(self, p: np.ndarray) -> float:
        lo, hi = np.asarray(self.lo), np.asarray(self.hi)
        return float(np.linalg.norm(np.maximum(np.maximum(lo - p, 0.0), p - hi)))


@dataclass(frozen=True)
class Cylinder:
    """圆柱：底面圆心 (x, y, z_lo)，半径 r，高到 z_hi。

    ``lean_xy`` 是顶面圆心相对底面的水平偏移。缺省 (0, 0) 时是竖直圆柱，
    距离公式与原来逐位相同。
    """

    name: str
    center_xy: tuple[float, float]
    radius: float
    z_lo: float
    z_hi: float
    lean_xy: tuple[float, float] = (0.0, 0.0)

    def upright(self) -> bool:
        return self.lean_xy[0] == 0.0 and self.lean_xy[1] == 0.0

    def axis_ends(self) -> tuple[np.ndarray, np.ndarray]:
        base = np.array([self.center_xy[0], self.center_xy[1], self.z_lo], dtype=float)
        top = np.array(
            [self.center_xy[0] + self.lean_xy[0], self.center_xy[1] + self.lean_xy[1], self.z_hi],
            dtype=float,
        )
        return base, top

    def section_xy(self, z: float) -> tuple[float, float]:
        span = self.z_hi - self.z_lo
        u = 0.0 if span <= 0.0 else min(max((float(z) - self.z_lo) / span, 0.0), 1.0)
        return (self.center_xy[0] + u * self.lean_xy[0], self.center_xy[1] + u * self.lean_xy[1])

    def distance(self, p: np.ndarray) -> float:
        if self.upright():
            dr = max(float(np.hypot(p[0] - self.center_xy[0], p[1] - self.center_xy[1])) - self.radius, 0.0)
            dz = max(self.z_lo - p[2], 0.0, p[2] - self.z_hi)
            return float(np.hypot(dr, dz))
        a, b = self.axis_ends()
        ab = b - a
        length = float(np.linalg.norm(ab))
        if length < 1e-9:
            return float(max(np.linalg.norm(np.asarray(p, dtype=float) - a) - self.radius, 0.0))
        axis = ab / length
        rel = np.asarray(p, dtype=float) - a
        axial = float(np.dot(rel, axis))
        radial = float(np.linalg.norm(rel - axial * axis))
        dr = max(radial - self.radius, 0.0)
        if axial < 0.0:
            return float(np.hypot(dr, -axial))
        if axial > length:
            return float(np.hypot(dr, axial - length))
        return dr


@dataclass(frozen=True)
class Ground:
    z: float

    name: str = "ground"

    def distance(self, p: np.ndarray) -> float:
        return max(float(p[2]) - self.z, 0.0)


Obstacle = Box | Cylinder | Ground


@dataclass(frozen=True)
class SceneCard:
    path: str
    card: dict  # 场景卡原文（geometry 已内联为 dict）
    obstacles: tuple[Obstacle, ...]

    @property
    def name(self) -> str:
        return self.card["name"]

    @property
    def hash(self) -> str:
        blob = json.dumps(self.card, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(blob).hexdigest()

    @cached_property
    def _large_distance_index(self):
        upright = tuple(o for o in self.obstacles if isinstance(o, Cylinder) and o.upright())
        other = tuple(o for o in self.obstacles if not (isinstance(o, Cylinder) and o.upright()))
        return (np.asarray([o.center_xy for o in upright], dtype=float).reshape(-1, 2),
                np.asarray([o.radius for o in upright]), np.asarray([o.z_lo for o in upright]),
                np.asarray([o.z_hi for o in upright]), other)

    def min_distance(self, p: np.ndarray) -> float:
        if len(self.obstacles) < 200:
            return min(o.distance(p) for o in self.obstacles)
        xy, radius, zlo, zhi, other = self._large_distance_index
        best = min((o.distance(p) for o in other), default=float("inf"))
        if len(radius):
            dr = np.maximum(np.linalg.norm(xy - np.asarray(p)[:2], axis=1) - radius, 0.0)
            dz = np.maximum(np.maximum(zlo - p[2], p[2] - zhi), 0.0)
            best = min(best, float(np.min(np.hypot(dr, dz))))
        return best


def parse_geometry(geom: dict, ground_z: float | None) -> tuple[Obstacle, ...]:
    obs: list[Obstacle] = [Box(b["name"], tuple(b["lo"]), tuple(b["hi"])) for b in geom.get("boxes", [])]
    obs += [_cylinder_from_json(c) for c in geom.get("cylinders", [])]
    if ground_z is not None:
        obs.append(Ground(float(ground_z)))
    return tuple(obs)


def _cylinder_from_json(raw: dict) -> Cylinder:
    lean = raw.get("lean_xy") or (0.0, 0.0)
    return Cylinder(
        raw["name"],
        tuple(raw["center_xy"]),
        float(raw["radius"]),
        float(raw["z_lo"]),
        float(raw["z_hi"]),
        (float(lean[0]), float(lean[1])),
    )


def load_card(path: str | Path, geometry_override: dict | None = None) -> SceneCard:
    """读场景卡。geometry_override 只给测试用（换几何）。

    带 ``forest`` 的卡不读几何文件：树的位置由 ``yopo_l0.forest`` 按种子生成。
    """
    path = Path(path)
    card = json.loads(path.read_text())
    geom = geometry_override
    if geom is None and isinstance(card.get("wide_field"), dict):
        from .wide_scene import generate_wide_geometry

        geom = generate_wide_geometry(card["wide_field"])
    elif geom is None and isinstance(card.get("forest"), dict):
        from .forest import generate_forest_geometry

        geom = generate_forest_geometry(card["forest"])
    elif geom is None:
        geom = json.loads((path.parent / card["geometry"]).read_text())
    card = {**card, "geometry": geom}
    return SceneCard(str(path), card, parse_geometry(geom, card.get("ground_z")))
