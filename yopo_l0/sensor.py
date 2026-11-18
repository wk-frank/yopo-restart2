"""光线投射深度相机。相机系：x 右、y 下、z 前；世界系 ENU。深度为 z 深度（米），无回波为 NaN。

``YOPO_SENSOR_ACCEL`` 默认 ``"0"``（关）。只有 ``"1"`` 打开批量求交。
关掉时仍是逐障碍求交，深度与原来逐位相同。射线网格按 ``CameraConfig`` 缓存，
数组只读，两边共用；噪声仍是每帧一次 ``rng.normal``，抽取次数与几何无关。
"""

from __future__ import annotations

import functools
import math
import os
import threading
from dataclasses import dataclass

import numpy as np

from .scene import Box, Cylinder, Ground, Obstacle

# 机体 FLU → 相机 optical（右、下、前）
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


@dataclass(frozen=True)
class CameraConfig:
    width: int = 160
    height: int = 96
    hfov_deg: float = 90.0
    vfov_deg: float = 60.0
    min_range_m: float = 0.2
    max_range_m: float = 20.0
    noise_std_m: float = 0.0
    mount_xyz_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    strict_unknown: bool = False

    @classmethod
    def from_card(cls, d: dict) -> "CameraConfig":
        d = dict(d)
        if "mount_xyz_body" in d:
            d["mount_xyz_body"] = tuple(d["mount_xyz_body"])
        return cls(**d)

    @property
    def intrinsics(self) -> tuple[float, float, float, float]:
        fx = (self.width / 2) / np.tan(np.radians(self.hfov_deg) / 2)
        fy = (self.height / 2) / np.tan(np.radians(self.vfov_deg) / 2)
        return float(fx), float(fy), self.width / 2, self.height / 2


@dataclass(frozen=True)
class DepthFrame:
    stamp: float  # 仿真时钟
    depth: np.ndarray  # (H, W) float32
    K: tuple[float, float, float, float]  # fx, fy, cx, cy


def sensor_accel_enabled() -> bool:
    """批量求交开关。缺省和环境变量 ``"0"`` 都是关，只有 ``"1"`` 为开。"""
    return os.environ.get("YOPO_SENSOR_ACCEL", "0").strip() == "1"


@functools.lru_cache(maxsize=32)
def _ray_dirs_cam(cfg: CameraConfig) -> np.ndarray:
    """相机系射线，z 分量 = 1。``CameraConfig`` 可哈希，缓存只读数组。

    ``lru_cache`` 本身有锁。调用方不得写入返回值；世界系方向另做矩阵乘。
    """
    fx, fy, cx, cy = cfg.intrinsics
    u, v = np.meshgrid(np.arange(cfg.width) + 0.5, np.arange(cfg.height) + 0.5)
    dirs = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)
    dirs.setflags(write=False)
    return dirs


def _hit_box(o, d, b: Box):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (np.asarray(b.lo) - o) / d
        t2 = (np.asarray(b.hi) - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = (tmax >= tmin) & (tmin > 0)
    return np.where(hit, tmin, np.inf)


def _hit_cylinder(o, d, c: Cylinder):
    if not c.upright():
        return _hit_cylinder_lean(o, d, c)
    ox, oy = o[0] - c.center_xy[0], o[1] - c.center_xy[1]
    dx, dy = d[..., 0], d[..., 1]
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    cc = ox * ox + oy * oy - c.radius**2
    disc = b * b - 4 * a * cc
    with np.errstate(divide="ignore", invalid="ignore"):
        sq = np.sqrt(np.maximum(disc, 0.0))
        t_side = np.where((disc >= 0) & (a > 0), (-b - sq) / (2 * a), np.inf)
        t_side = np.where(t_side > 0, t_side, np.inf)
        z_side = o[2] + t_side * d[..., 2]
        t_side = np.where((z_side >= c.z_lo) & (z_side <= c.z_hi), t_side, np.inf)
        best = t_side
        for zc in (c.z_lo, c.z_hi):  # 顶面 / 底面
            t = (zc - o[2]) / d[..., 2]
            px, py = ox + t * dx, oy + t * dy
            ok = (t > 0) & (px * px + py * py <= c.radius**2)
            best = np.minimum(best, np.where(ok, t, np.inf))
    return best


def _hit_cylinder_lean(o, d, c: Cylinder):
    """有限圆柱，轴线可以从底面圆心斜到顶面圆心。竖直圆柱不走这里。"""
    a, b = c.axis_ends()
    ab = b - a
    length = float(np.linalg.norm(ab))
    axis = ab / length
    origin = np.asarray(o, dtype=float)
    oc = origin - a
    d_ax = np.sum(d * axis, axis=-1)
    d_perp = d - d_ax[..., None] * axis
    oc_ax = float(np.dot(oc, axis))
    oc_perp = oc - oc_ax * axis
    A = np.sum(d_perp * d_perp, axis=-1)
    B = 2.0 * np.sum(oc_perp * d_perp, axis=-1)
    C = float(np.dot(oc_perp, oc_perp) - c.radius ** 2)
    disc = B * B - 4.0 * A * C
    with np.errstate(divide="ignore", invalid="ignore"):
        sq = np.sqrt(np.maximum(disc, 0.0))
        denom = np.where(A > 1e-12, 2.0 * A, np.inf)
        t_near = np.where(disc >= 0.0, (-B - sq) / denom, np.inf)
        t_far = np.where(disc >= 0.0, (-B + sq) / denom, np.inf)

    def on_side(t):
        axial = oc_ax + t * d_ax
        return (t > 0.0) & (axial >= -1e-6) & (axial <= length + 1e-6)

    best = np.where(on_side(t_near), t_near, np.inf)
    best = np.minimum(best, np.where(on_side(t_far), t_far, np.inf))
    for end, outward in ((a, -axis), (b, axis)):
        denom = np.sum(d * outward, axis=-1)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.dot(end - origin, outward) / np.where(np.abs(denom) > 1e-12, denom, np.inf)
        hit = origin + t[..., None] * d
        radial = hit - end
        radial = radial - np.sum(radial * axis, axis=-1, keepdims=True) * axis
        dist = np.linalg.norm(radial, axis=-1)
        best = np.minimum(best, np.where((t > 0.0) & (dist <= c.radius + 1e-7), t, np.inf))
    return best


def _hit_ground(o, d, g: Ground):
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (g.z - o[2]) / d[..., 2]
    return np.where(t > 0, t, np.inf)


# 窗口占整幅达到这个比例就并进一次全图批量，不再按小窗切。
_LARGE_WINDOW = 0.35
_CLUSTER_SLACK = 1.6
_CLUSTER_MAX = 6
_CLUSTER_UNION = 0.25
_WIN_MARGIN_M = 1e-3
_WIN_PAD_PX = 3
_LOCAL = threading.local()


def _slab_buffers(n: int, h: int, w: int):
    """线程本地的盒子 slab 缓冲。形状变了才重新分配。"""
    key = (n, h, w)
    slot = getattr(_LOCAL, "slab", None)
    if slot is None or slot[0] != key:
        slot = (key, *(np.empty((n, h, w)) for _ in range(6)))
        _LOCAL.slab = slot
    return slot[1:]


def _hit_boxes_batch(
    origin: np.ndarray, dirs: np.ndarray, lo: np.ndarray, hi: np.ndarray,
) -> np.ndarray:
    """多个盒子一次 slab。三个轴分开归约，和逐盒 ``_hit_box`` 逐位相同。"""
    n = int(lo.shape[0])
    h, w = dirs.shape[:2]
    if n == 0:
        return np.full((h, w), np.inf)
    mn, mx, ta, tb, tmin, tmax = _slab_buffers(n, h, w)
    tmin.fill(-np.inf)
    tmax.fill(np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for axis in range(3):
            comp = dirs[..., axis]
            np.divide((lo[:, axis] - origin[axis])[:, None, None], comp, out=mn)
            np.divide((hi[:, axis] - origin[axis])[:, None, None], comp, out=mx)
            np.minimum(mn, mx, out=ta)
            np.maximum(mn, mx, out=tb)
            np.copyto(ta, -np.inf, where=np.isnan(ta))
            np.copyto(tb, np.inf, where=np.isnan(tb))
            np.maximum(tmin, ta, out=tmin)
            np.minimum(tmax, tb, out=tmax)
    hit = (tmax >= tmin) & (tmin > 0)
    np.copyto(tmin, np.inf, where=~hit)
    return np.min(tmin, axis=0)


def _hit_upright_cylinders_batch(
    origin: np.ndarray,
    dirs: np.ndarray,
    center: np.ndarray,
    r2: np.ndarray,
    z_lo: np.ndarray,
    z_hi: np.ndarray,
) -> np.ndarray:
    """多个竖直圆柱一次求交。公式与 ``_hit_cylinder`` 的竖直分支相同。"""
    n = int(center.shape[0])
    h, w = dirs.shape[:2]
    if n == 0:
        return np.full((h, w), np.inf)
    dx, dy, dz = dirs[..., 0], dirs[..., 1], dirs[..., 2]
    a = dx * dx + dy * dy
    ox = origin[0] - center[:, 0]
    oy = origin[1] - center[:, 1]
    with np.errstate(divide="ignore", invalid="ignore"):
        b = 2 * (ox[:, None, None] * dx + oy[:, None, None] * dy)
        cc = ox * ox + oy * oy - r2
        disc = b * b - 4 * a * cc[:, None, None]
        sq = np.sqrt(np.maximum(disc, 0.0))
        t = np.where((disc >= 0) & (a > 0), (-b - sq) / (2 * a), np.inf)
        t = np.where(t > 0, t, np.inf)
        z_side = origin[2] + t * dz
        t = np.where((z_side >= z_lo[:, None, None]) & (z_side <= z_hi[:, None, None]), t, np.inf)
        for zc in (z_lo, z_hi):
            tc = (zc[:, None, None] - origin[2]) / dz
            px = ox[:, None, None] + tc * dx
            py = oy[:, None, None] + tc * dy
            ok = (tc > 0) & (px * px + py * py <= r2[:, None, None])
            t = np.minimum(t, np.where(ok, tc, np.inf))
    return np.min(t, axis=0)


def _aabb_window(
    cfg: CameraConfig, origin: np.ndarray, R: np.ndarray, lo: np.ndarray, hi: np.ndarray,
):
    """包围盒在图像上的保守像素窗 ``(v0, v1, u0, u1)``。

    完全在相机后方则返回 None。有角点贴着近平面或落到后面时用整幅，
    避免投影爆掉漏像素。窗内再按原公式求交，多算的像素结果仍是 inf。
    """
    xs = (float(lo[0]), float(hi[0]))
    ys = (float(lo[1]), float(hi[1]))
    zs = (float(lo[2]), float(hi[2]))
    corners = np.array([(x, y, z) for x in xs for y in ys for z in zs], dtype=np.float64)
    cam = (corners - origin) @ np.asarray(R, dtype=np.float64)
    z = cam[:, 2]
    if bool(np.all(z <= 0.0)):
        return None
    if bool(np.any(z <= _WIN_MARGIN_M)):
        return (0, cfg.height, 0, cfg.width)
    fx, fy, cx, cy = cfg.intrinsics
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx + fx * (cam[:, 0] / z)
        v = cy + fy * (cam[:, 1] / z)
    if not (np.isfinite(u).all() and np.isfinite(v).all()):
        return (0, cfg.height, 0, cfg.width)
    u0 = max(0, math.floor(float(np.min(u))) - _WIN_PAD_PX)
    u1 = min(cfg.width, math.ceil(float(np.max(u))) + _WIN_PAD_PX)
    v0 = max(0, math.floor(float(np.min(v))) - _WIN_PAD_PX)
    v1 = min(cfg.height, math.ceil(float(np.max(v))) + _WIN_PAD_PX)
    if u0 >= u1 or v0 >= v1:
        return None
    return (v0, v1, u0, u1)


def _window_union(items):
    v0 = min(window[0] for _, window in items)
    v1 = max(window[1] for _, window in items)
    u0 = min(window[2] for _, window in items)
    u1 = max(window[3] for _, window in items)
    return v0, v1, u0, u1, (v1 - v0) * (u1 - u0)


def _clusters(small: list[tuple[int, tuple[int, int, int, int]]], image_area: int):
    """把小窗并成若干批。并集必须盖住每个成员的窗，所以不会少算像素。"""
    cur: list[tuple[int, tuple[int, int, int, int]]] = []
    area_sum = 0
    union_limit = _CLUSTER_UNION * image_area
    for item in small:
        _, window = item
        area = (window[1] - window[0]) * (window[3] - window[2])
        if not cur:
            cur = [item]
            area_sum = area
            continue
        union = _window_union(cur + [item])[4]
        fits = len(cur) < _CLUSTER_MAX and union <= union_limit
        if fits and union <= _CLUSTER_SLACK * (area_sum + area):
            cur.append(item)
            area_sum += area
            continue
        yield cur
        cur = [item]
        area_sum = area
    if cur:
        yield cur


def _paint_batch(best, origin, dirs, indexed, batch):
    """``indexed`` 是 ``(下标, 窗口或 None)``。大窗合成一次全图批量，小窗按簇批量。"""
    height, width = dirs.shape[:2]
    limit = _LARGE_WINDOW * height * width
    large: list[int] = []
    small: list[tuple[int, tuple[int, int, int, int]]] = []
    for index, window in indexed:
        if window is None:
            continue
        v0, v1, u0, u1 = window
        if (v1 - v0) * (u1 - u0) >= limit:
            large.append(index)
        else:
            small.append((index, window))
    if large:
        best = np.minimum(best, batch(origin, dirs, large))
    small.sort(key=lambda item: (item[1][2], item[1][0]))
    for cluster in _clusters(small, height * width):
        v0, v1, u0, u1, _ = _window_union(cluster)
        hit = batch(origin, dirs[v0:v1, u0:u1], [index for index, _ in cluster])
        best[v0:v1, u0:u1] = np.minimum(best[v0:v1, u0:u1], hit)
    return best


def _best_hit_accel(
    obstacles: tuple[Obstacle, ...], cfg: CameraConfig, origin, dirs, R,
) -> np.ndarray:
    """竖直圆柱和盒子走批量；斜柱和地面仍走原来的函数。"""
    origin = np.asarray(origin, dtype=np.float64).reshape(3)
    best = np.full(dirs.shape[:2], np.inf)
    boxes = [ob for ob in obstacles if isinstance(ob, Box)]
    upright = [ob for ob in obstacles if isinstance(ob, Cylinder) and ob.upright()]
    rest = [
        ob
        for ob in obstacles
        if not (isinstance(ob, Box) or (isinstance(ob, Cylinder) and ob.upright()))
    ]
    if boxes:
        lo = np.asarray([b.lo for b in boxes], dtype=np.float64)
        hi = np.asarray([b.hi for b in boxes], dtype=np.float64)

        def batch_boxes(o, d, indexes):
            sel = np.asarray(indexes, dtype=int)
            return _hit_boxes_batch(o, d, lo[sel], hi[sel])

        indexed = [(i, _aabb_window(cfg, origin, R, lo[i], hi[i])) for i in range(len(boxes))]
        best = _paint_batch(best, origin, dirs, indexed, batch_boxes)
    if upright:
        center = np.asarray([c.center_xy for c in upright], dtype=np.float64)
        r2 = np.asarray([c.radius ** 2 for c in upright], dtype=np.float64)
        z_lo = np.asarray([c.z_lo for c in upright], dtype=np.float64)
        z_hi = np.asarray([c.z_hi for c in upright], dtype=np.float64)

        def batch_cyls(o, d, indexes):
            sel = np.asarray(indexes, dtype=int)
            return _hit_upright_cylinders_batch(o, d, center[sel], r2[sel], z_lo[sel], z_hi[sel])

        indexed = []
        for i, cyl in enumerate(upright):
            x, y = float(center[i, 0]), float(center[i, 1])
            radius = float(cyl.radius)
            indexed.append((
                i,
                _aabb_window(
                    cfg,
                    origin,
                    R,
                    np.array([x - radius, y - radius, float(z_lo[i])]),
                    np.array([x + radius, y + radius, float(z_hi[i])]),
                ),
            ))
        best = _paint_batch(best, origin, dirs, indexed, batch_cyls)
    for ob in rest:
        if isinstance(ob, Cylinder):
            hit = _hit_cylinder(origin, dirs, ob)
        else:
            hit = _hit_ground(origin, dirs, ob)
        best = np.minimum(best, hit)
    return best


def render(
    obstacles: tuple[Obstacle, ...], cfg: CameraConfig, cam_pos: np.ndarray, R_world_cam: np.ndarray,
    rng: np.random.Generator, stamp: float, d435=None,
) -> DepthFrame:
    d = _ray_dirs_cam(cfg) @ R_world_cam.T  # 世界系方向，参数 t 即相机 z 深度
    if sensor_accel_enabled():
        best = _best_hit_accel(obstacles, cfg, cam_pos, d, R_world_cam)
    else:
        best = np.full(d.shape[:2], np.inf)
        for ob in obstacles:
            if isinstance(ob, Box):
                t = _hit_box(cam_pos, d, ob)
            elif isinstance(ob, Cylinder):
                t = _hit_cylinder(cam_pos, d, ob)
            else:
                t = _hit_ground(cam_pos, d, ob)
            best = np.minimum(best, t)
    # D435 开关的唯一入口。关（d435 is None）时下面三行与加开关之前逐位相同，rng 抽取次数不变。
    if d435 is not None:
        depth, out_stamp = d435.finish(best, cfg, rng, stamp)
        return DepthFrame(out_stamp, depth, cfg.intrinsics)
    # 噪声每帧整幅抽取，抽取次数与几何无关，保证同 seed 同序列。加速路径不碰 rng。
    noise = rng.normal(0.0, cfg.noise_std_m, best.shape) if cfg.noise_std_m > 0 else 0.0
    depth = best + noise
    depth[~np.isfinite(best) | (best > cfg.max_range_m) | (best < cfg.min_range_m)] = np.nan
    return DepthFrame(stamp, depth.astype(np.float32), cfg.intrinsics)


class DepthCamera:
    def __init__(self, cfg: CameraConfig, obstacles: tuple[Obstacle, ...], rng: np.random.Generator, d435=None):
        self.cfg, self._obstacles, self._rng, self._d435 = cfg, obstacles, rng, d435
        centres, radii, finite = [], [], []
        for i, ob in enumerate(obstacles):
            if isinstance(ob, Cylinder):
                a, b = ob.axis_ends()
                centres.append((a + b) * .5)
                radii.append(float(np.linalg.norm(b - a)) * .5 + ob.radius)
                finite.append(i)
            elif isinstance(ob, Box):
                lo, hi = np.asarray(ob.lo), np.asarray(ob.hi)
                centres.append((lo + hi) * .5)
                radii.append(float(np.linalg.norm(hi - lo)) * .5)
                finite.append(i)
        self._cull_centres = np.asarray(centres, dtype=float).reshape(-1,3)
        self._cull_radii = np.asarray(radii, dtype=float)
        self._cull_indices = np.asarray(finite, dtype=int)
        self._cull_infinite = tuple(ob for ob in obstacles if isinstance(ob, Ground))

    def capture(self, position: np.ndarray, yaw: float, stamp: float) -> DepthFrame:
        c, s = np.cos(yaw), np.sin(yaw)
        R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam_pos = np.asarray(position) + R_wb @ np.asarray(self.cfg.mount_xyz_body)
        obstacles = self._obstacles
        if os.environ.get("YOPO_SENSOR_SPATIAL", "0") == "1":
            reach = self.cfg.max_range_m * math.sqrt(1.0 + math.tan(math.radians(self.cfg.hfov_deg/2))**2
                                                    + math.tan(math.radians(self.cfg.vfov_deg/2))**2)
            mask = np.linalg.norm(self._cull_centres - cam_pos, axis=1) <= reach + self._cull_radii + 1e-6
            obstacles = self._cull_infinite + tuple(self._obstacles[i] for i in self._cull_indices[mask])
        return render(obstacles, self.cfg, cam_pos, R_wb @ R_BODY_CAM, self._rng, stamp, self._d435)
