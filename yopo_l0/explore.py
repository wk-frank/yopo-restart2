"""无编队水平横排起飞高效率集群穿越（ExploreFleet）。

彻底移除 CrossingFleet 的全部编队桎梏，最大化多机自主避障与前向穿越效率：
1. 无进度门控（No Progress Gate）：移除 progress_gate / _ahead_of_gate / LEADER_HOVER /
   virtual_center.speed_mps 限速，各机独立以物理安全上限全速推进。
2. 无 TDMA 轮转等待（Parallel High-Rate Replanning）：全体无人机在统一水平起飞线 x = x_0
   同时出发，每规划拍（如 10–30 Hz）并行前向重规划，经进程内 MessageBus 交换已提交三维轨迹管。
3. 无横向槽位橡皮筋（Free-Lateral Corridor Surfing）：移除 slot_body / lateral_gain /
   exit_tighten_m，各机仅追求 +X 轴穿越并在安全走廊界 [y_min, y_max] × [z_min, z_max]
   内自由顺畅绕障；越过障碍带进入终点区后就近锁定无冲突悬停终点 (x_term >= x_goal, y_term, z_goal)。
4. TJU YOPO 时间相似律提速（ratio = velocity_mps / 6.0）：输入观测按 (v / ratio, a / ratio²)
   精确还原训练态分布，解码按 (vel_max * ratio, acc_max * ratio², sgm_time / ratio) 缩放，
   保持空间避障几何曲线不变而实现 > 6 m/s 高速密林穿越（core/ 零改动）。
5. 递归可行前缀+制动安全盾与同拍确定性优先级去冲突：每条提交轨迹均附带静态障碍净空与邻机三维
   欧氏管分离校验的可停前缀+制动尾，保证障碍碰撞与机间碰撞严格为 0。
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, replace

import numpy as np

from yopo_swarm_core.candidates import (
    UPSTREAM_DECODER,
    DecoderParams,
    corridor_mu_lo,
    decode_params,
    minco_two_piece,
)
from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.plan import (
    KinematicLimits,
    brake_branch,
    hold_traj,
    quintic,
)
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment
from yopo_swarm_core.yaw import calculate_yaw

from .criteria import frozen_for_card
from .dynamics import PointMassLag
from .policy import R_yaw, read_safe_radius
from .scene import Box, Cylinder, Ground, parse_geometry

_VEL_TRAIN = 6.0
_ACC_TRAIN = 6.0
_SGM_TIME_TRAIN = 10.0 / 6.0
_DURATION_MIN_TRAIN = 0.1
_SAMPLE_DT_S = 0.02
_SHELL_NAMES = frozenset({"ceiling", "left_wall", "right_wall", "end_wall", "back_wall"})


def _positive_float(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须为正实数，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out) or out <= 0.0:
        raise ValueError(f"{name} 必须为正实数，收到 {value!r}")
    return out


def _nonneg_float(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须为非负实数，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out) or out < 0.0:
        raise ValueError(f"{name} 必须为非负实数，收到 {value!r}")
    return out


def _finite_float(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须为有限实数，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} 必须为有限实数，收到 {value!r}")
    return out


def _bounds_pair(name: str, raw, default: tuple[float, float]) -> tuple[float, float]:
    if raw is None:
        return (float(default[0]), float(default[1]))
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ValueError(f"{name} 必须是长度为 2 的区间 [lo, hi]，收到 {raw!r}")
    lo = _finite_float(f"{name}[0]", raw[0])
    hi = _finite_float(f"{name}[1]", raw[1])
    if not lo < hi:
        raise ValueError(f"{name} 必须满足 lo < hi，收到 [{lo}, {hi}]")
    return (lo, hi)


def dynamics_a_max(dynamics: dict | None) -> float:
    dyn = dict(dynamics or {})
    raw = dyn.get("a_max")
    a_max = float(PointMassLag.a_max) if raw is None else _positive_float("dynamics.a_max", raw)
    return a_max


def build_scaled_decoder(velocity_mps: float, safe_radius_m: float) -> tuple[DecoderParams, float]:
    """按 TJU YOPO 时间相似律构造解码器（不修改 core/）。返回 (decoder, ratio)。"""
    vel = _positive_float("velocity_mps", velocity_mps)
    rad = _positive_float("safe_radius_m", safe_radius_m)
    ratio = vel / _VEL_TRAIN
    dec = replace(
        UPSTREAM_DECODER,
        vel_max=_VEL_TRAIN * ratio,
        acc_max=_ACC_TRAIN * ratio * ratio,
        segment_time=_SGM_TIME_TRAIN / ratio,
        safe_radius=rad,
    )
    return dec, ratio


class LocalApproval:
    """本机策略占位对象（兼容 runner.py 与 sitl_bridge.py 的队列状态码记录）。"""

    def __init__(self) -> None:
        self.last_code = "LOCAL_HOVER"
        self.extra_codes: dict[str, int] = {}


@dataclass(frozen=True)
class _NoGeom:
    enabled: bool = False


@dataclass(frozen=True)
class _ExecRecord:
    owner: str
    plan_seq: int
    plan_hash: str


def _traj_hash(owner: str, seq: int, traj: PiecewisePolynomial) -> str:
    h = hashlib.sha256()
    h.update(owner.encode("utf-8"))
    h.update(int(seq).to_bytes(8, "little", signed=False))
    h.update(np.float64(traj.t0).tobytes())
    for seg in traj.segments:
        h.update(np.float64(seg.duration).tobytes())
        h.update(np.ascontiguousarray(seg.coeffs, dtype=np.float64).tobytes())
    return h.hexdigest()


def _eval_pva(traj: PiecewisePolynomial, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tt = min(max(float(t), float(traj.t_start)), float(traj.t_end))
    p = np.asarray(traj.evaluate(tt, 0), dtype=float)
    if float(t) > float(traj.t_end) + 1e-9 or float(t) < float(traj.t_start) - 1e-9:
        return p, np.zeros(3, dtype=float), np.zeros(3, dtype=float)
    v = np.asarray(traj.evaluate(tt, 1), dtype=float)
    a = np.asarray(traj.evaluate(tt, 2), dtype=float)
    return p, v, a


def _sample_positions(traj: PiecewisePolynomial, times: np.ndarray) -> np.ndarray:
    """向量化采样分段多项式（任意阶数 >= 1）在 times 上的三维位置 (M, 3)。"""
    t_arr = np.clip(np.asarray(times, dtype=np.float64), float(traj.t_start), float(traj.t_end))
    out = np.empty((t_arr.size, 3), dtype=np.float64)
    cursor = float(traj.t0)
    n_segs = len(traj.segments)
    for idx, seg in enumerate(traj.segments):
        seg_end = cursor + float(seg.duration)
        if idx == n_segs - 1:
            mask = t_arr >= cursor - 1e-12
        else:
            mask = (t_arr >= cursor - 1e-12) & (t_arr < seg_end)
        if np.any(mask):
            tau = np.clip(t_arr[mask] - cursor, 0.0, float(seg.duration))
            n_coeff = int(seg.coeffs.shape[1])
            powers = np.vstack([np.ones_like(tau) if k == 0 else tau**k for k in range(n_coeff)])
            out[mask] = (seg.coeffs @ powers).T
        cursor = seg_end
    return out


def _sample_pvaj(
    traj: PiecewisePolynomial, times: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """一次向量化采样分段多项式在 times 上的位置、速度、加速度与加加速度 (M, 3)。"""
    t_arr = np.clip(np.asarray(times, dtype=np.float64), float(traj.t_start), float(traj.t_end))
    m = t_arr.size
    pos = np.empty((m, 3), dtype=np.float64)
    vel = np.zeros((m, 3), dtype=np.float64)
    acc = np.zeros((m, 3), dtype=np.float64)
    jrk = np.zeros((m, 3), dtype=np.float64)
    cursor = float(traj.t0)
    n_segs = len(traj.segments)
    for idx, seg in enumerate(traj.segments):
        dur = float(seg.duration)
        seg_end = cursor + dur
        mask = (t_arr >= cursor - 1e-12) if idx == n_segs - 1 else ((t_arr >= cursor - 1e-12) & (t_arr < seg_end))
        if np.any(mask):
            tau = np.clip(t_arr[mask] - cursor, 0.0, dur)
            c = seg.coeffs
            nc = int(c.shape[1])
            t2 = tau * tau
            t3 = t2 * tau
            t4 = t3 * tau
            t5 = t4 * tau
            p_pow = [np.ones_like(tau), tau, t2, t3, t4, t5][:nc]
            pos[mask] = (c @ np.vstack(p_pow)).T
            if nc >= 2:
                v_pow = [np.ones_like(tau), 2.0 * tau, 3.0 * t2, 4.0 * t3, 5.0 * t4][: nc - 1]
                vel[mask] = (c[:, 1:] @ np.vstack(v_pow)).T
            if nc >= 3:
                a_pow = [2.0 * np.ones_like(tau), 6.0 * tau, 12.0 * t2, 20.0 * t3][: nc - 2]
                acc[mask] = (c[:, 2:] @ np.vstack(a_pow)).T
            if nc >= 4:
                j_pow = [6.0 * np.ones_like(tau), 24.0 * tau, 60.0 * t2][: nc - 3]
                jrk[mask] = (c[:, 3:] @ np.vstack(j_pow)).T
        cursor = seg_end
    return pos, vel, acc, jrk


def _fast_kin_ok(vel: np.ndarray, acc: np.ndarray, jrk: np.ndarray, limits: KinematicLimits) -> bool:
    """向量化三阶导数范数上界检查（含采样弦高 2% 保守余量）。"""
    v_lim_sq = (0.98 * float(limits.v_max)) ** 2
    a_lim_sq = (0.98 * float(limits.a_max)) ** 2
    j_lim_sq = (0.98 * float(limits.j_max)) ** 2
    if float(np.max(np.einsum("ij,ij->i", vel, vel))) > v_lim_sq:
        return False
    if float(np.max(np.einsum("ij,ij->i", acc, acc))) > a_lim_sq:
        return False
    if float(np.max(np.einsum("ij,ij->i", jrk, jrk))) > j_lim_sq:
        return False
    return True


def _fast_brake_traj(t_branch: float, head: np.ndarray, limits: KinematicLimits) -> PiecewisePolynomial | None:
    """快速构造满足动力学限幅的自然五次刹停段（避免在内层循环重复做 Sturm 根隔离）。"""
    p, v, a = np.asarray(head, dtype=float)
    speed = float(np.linalg.norm(v))
    acc_norm = float(np.linalg.norm(a))
    if speed <= 1e-6 and acc_norm <= 1e-6:
        return None
    T = max(0.20, 1.55 * speed / float(limits.a_max), 2.5 * acc_norm / float(limits.j_max))
    for _ in range(8):
        stop = p + 0.5 * v * T + (1.0 / 12.0) * a * (T * T)
        seg = quintic(head, np.array([stop, np.zeros(3), np.zeros(3)], dtype=float), T)
        tr = PiecewisePolynomial(float(t_branch), [seg])
        n = max(8, int(math.ceil(T / _SAMPLE_DT_S)) + 1)
        ts = np.linspace(float(t_branch), float(t_branch) + T, n, dtype=np.float64)
        _, vv, aa, jj = _sample_pvaj(tr, ts)
        if _fast_kin_ok(vv, aa, jj, limits):
            return tr
        T *= 1.25
    return None


def _kin_ok(traj: PiecewisePolynomial, limits: KinematicLimits) -> bool:
    return all(
        check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE
        for order, lim in limits.by_order()
    )


def _prefix(traj: PiecewisePolynomial, t_cut: float) -> PiecewisePolynomial:
    if t_cut >= traj.t_end - 1e-9:
        return traj
    t_cut = max(float(t_cut), float(traj.t_start) + 0.04)
    segs: list[PolySegment] = []
    t = float(traj.t0)
    for seg in traj.segments:
        end = t + float(seg.duration)
        if end <= t_cut + 1e-12:
            segs.append(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), float(seg.duration)))
            t = end
            continue
        segs.append(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), max(t_cut - t, 1e-3)))
        break
    if not segs:
        raise RuntimeError("轨迹前缀为空")
    return PiecewisePolynomial(float(traj.t0), segs)


def _append_stop(traj: PiecewisePolynomial, limits: KinematicLimits) -> PiecewisePolynomial | None:
    t1 = float(traj.t_end)
    head = np.array([traj.evaluate(t1, k) for k in range(3)], dtype=float)
    if float(np.linalg.norm(head[1])) <= 1e-6 and float(np.linalg.norm(head[2])) <= 1e-6:
        return traj
    stop = _fast_brake_traj(t1, head, limits)
    if stop is None:
        return None
    segs = [PolySegment(np.array(seg.coeffs, dtype=float, copy=True), float(seg.duration)) for seg in traj.segments]
    segs.extend(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), float(seg.duration)) for seg in stop.segments)
    return PiecewisePolynomial(float(traj.t0), segs)


class _FastObstacleField:
    """向量化三维障碍表面距离场（与 scene.Obstacle.distance 几何公式逐项一致）。"""

    def __init__(self, obstacles: tuple) -> None:
        up_xy, up_r, up_zlo, up_zhi = [], [], [], []
        ln_a, ln_ab, ln_len, ln_r = [], [], [], []
        bx_lo, bx_hi = [], []
        grounds = []
        for ob in obstacles:
            if isinstance(ob, Cylinder):
                if ob.upright():
                    up_xy.append(ob.center_xy)
                    up_r.append(ob.radius)
                    up_zlo.append(ob.z_lo)
                    up_zhi.append(ob.z_hi)
                else:
                    a, b = ob.axis_ends()
                    ab = b - a
                    length = float(np.linalg.norm(ab))
                    ln_a.append(a)
                    ln_ab.append(ab / max(length, 1e-9))
                    ln_len.append(length)
                    ln_r.append(ob.radius)
            elif isinstance(ob, Box):
                bx_lo.append(ob.lo)
                bx_hi.append(ob.hi)
            elif isinstance(ob, Ground):
                grounds.append(float(ob.z))
        self.has_up = bool(up_xy)
        if self.has_up:
            self.up_xy = np.asarray(up_xy, dtype=np.float64)
            self.up_r = np.asarray(up_r, dtype=np.float64)
            self.up_zlo = np.asarray(up_zlo, dtype=np.float64)
            self.up_zhi = np.asarray(up_zhi, dtype=np.float64)
        self.has_ln = bool(ln_a)
        if self.has_ln:
            self.ln_a = np.asarray(ln_a, dtype=np.float64)
            self.ln_axis = np.asarray(ln_ab, dtype=np.float64)
            self.ln_len = np.asarray(ln_len, dtype=np.float64)
            self.ln_r = np.asarray(ln_r, dtype=np.float64)
        self.has_box = bool(bx_lo)
        if self.has_box:
            self.bx_lo = np.asarray(bx_lo, dtype=np.float64)
            self.bx_hi = np.asarray(bx_hi, dtype=np.float64)
        self.ground_z = max(grounds) if grounds else None

    def min_distance(self, pts: np.ndarray) -> float:
        """返回采样点集 pts (M, 3) 到所有静态障碍表面的全局最小距离。"""
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        if p.size == 0:
            return math.inf
        best = math.inf
        if self.ground_z is not None:
            g_min = float(np.min(p[:, 2] - self.ground_z))
            best = min(best, max(g_min, 0.0))
        if self.has_up:
            dx = p[:, None, 0] - self.up_xy[None, :, 0]
            dy = p[:, None, 1] - self.up_xy[None, :, 1]
            dr = np.maximum(np.hypot(dx, dy) - self.up_r[None, :], 0.0)
            dz = np.maximum(
                np.maximum(self.up_zlo[None, :] - p[:, None, 2], p[:, None, 2] - self.up_zhi[None, :]),
                0.0,
            )
            best = min(best, float(np.min(np.hypot(dr, dz))))
        if self.has_ln:
            rel = p[:, None, :] - self.ln_a[None, :, :]
            axial = np.einsum("mnk,nk->mn", rel, self.ln_axis)
            proj = rel - axial[:, :, None] * self.ln_axis[None, :, :]
            radial = np.linalg.norm(proj, axis=-1)
            dr = np.maximum(radial - self.ln_r[None, :], 0.0)
            dz_lo = np.maximum(-axial, 0.0)
            dz_hi = np.maximum(axial - self.ln_len[None, :], 0.0)
            dz = np.maximum(dz_lo, dz_hi)
            best = min(best, float(np.min(np.hypot(dr, dz))))
        if self.has_box:
            d_lo = np.maximum(self.bx_lo[None, :, :] - p[:, None, :], 0.0)
            d_hi = np.maximum(p[:, None, :] - self.bx_hi[None, :, :], 0.0)
            d_box = np.linalg.norm(np.maximum(d_lo, d_hi), axis=-1)
            best = min(best, float(np.min(d_box)))
        return best


def _peer_tube_min_distance(times: np.ndarray, pts: np.ndarray, peer_tubes: list[np.ndarray]) -> float:
    """计算候选轨迹 (times, pts) 与所有邻机已提交轨迹管的最小同时刻三维中心距。

    与旧 inter_drone.clearance_m 的关键安全增强：
    当 t > peer_tube[-1, 0] 时，邻机已停在其可停轨迹的末端保持点 peer_tube[-1, 1:4]，
    必须按末端驻留位置继续检查碰撞，绝不能跳过而导致追尾悬停邻机。
    """
    if pts.size == 0 or not peer_tubes:
        return math.inf
    t_q = np.asarray(times, dtype=np.float64)
    p_q = np.asarray(pts, dtype=np.float64)
    best = math.inf
    for nb in peer_tubes:
        if nb.size == 0:
            continue
        ts = nb[:, 0]
        lo = float(ts[0])
        # t >= lo 的全部点都有效；超出 ts[-1] 的点由 np.interp 自动夹到末端保持点 nb[-1, 1:4]
        mask = t_q >= lo - 1e-9
        if not np.any(mask):
            continue
        tm = t_q[mask]
        pm = p_q[mask]
        nx = np.interp(tm, ts, nb[:, 1])
        ny = np.interp(tm, ts, nb[:, 2])
        nz = np.interp(tm, ts, nb[:, 3])
        diff = pm - np.column_stack((nx, ny, nz))
        d = float(np.sqrt(np.min(np.einsum("ij,ij->i", diff, diff))))
        if d < best:
            best = d
    return best


class _ExploreTrack:
    """已提交轨迹采样器，实现主动顺速度前瞻偏航对齐（look-into-turn yaw）。"""

    def __init__(
        self,
        traj: PiecewisePolynomial,
        goal: np.ndarray,
        yaw: float,
        dt: float,
        *,
        max_yaw_rate: float = 0.75,
        scan_yaw: float | None = None,
    ) -> None:
        self.traj = traj
        self.goal = np.asarray(goal, dtype=float).copy()
        self.yaw = float(yaw)
        self.dt = float(dt)
        self.max_yaw_rate = float(max_yaw_rate)
        self.scan_yaw = scan_yaw

    def pva(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return _eval_pva(self.traj, float(t))

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        t_f = float(t)
        p, v, a = self.pva(t_f)
        # 向前预瞄 0.12 s 的切向速度，让 90° 深度视场提前转入高速弯道内侧
        _, v_look, _ = self.pva(min(t_f + 0.12, float(self.traj.t_end)))
        horiz_speed = float(math.hypot(float(v_look[0]), float(v_look[1])))
        gd = self.goal - p
        if horiz_speed > 0.45:
            vel_dir = v_look
            target_dir = 0.70 * v_look + 0.30 * (gd / (float(np.linalg.norm(gd)) + 1e-6)) * horiz_speed
            self.yaw = float(calculate_yaw(vel_dir, target_dir, self.yaw, self.dt, max_yaw_rate=self.max_yaw_rate)[0])
        elif self.scan_yaw is not None:
            scan_dir = np.array([math.cos(self.scan_yaw), math.sin(self.scan_yaw), 0.0], dtype=float)
            self.yaw = float(calculate_yaw(scan_dir, scan_dir, self.yaw, self.dt, max_yaw_rate=self.max_yaw_rate)[0])
        elif float(np.linalg.norm(gd[:2])) > 0.6:
            self.yaw = float(calculate_yaw(gd, gd, self.yaw, self.dt, max_yaw_rate=self.max_yaw_rate)[0])
        return p, v, a, self.yaw


class ExploreFleet:
    """无编队水平横排起飞高效率多机穿越舰队。"""

    policy_name = "explore"
    result_key = "explore"
    uses_network = True
    transport_override = None

    def __init__(self, card: dict, physics_dt: float, *, record_inputs: bool = False) -> None:
        task = card.get("task") or {}
        if task.get("type") != "explore":
            raise ValueError("ExploreFleet 只接受 task.type == 'explore' 的场景卡")
        drones = card.get("drones") or []
        if len(drones) < 1:
            raise ValueError("explore 至少需要一架无人机")
        self.members = tuple(str(d["id"]) for d in drones)
        if len(set(self.members)) != len(self.members):
            raise ValueError("无人机 id 重复")

        self.starts: dict[str, np.ndarray] = {}
        for d in drones:
            did = str(d["id"])
            raw_s = d.get("start_enu")
            if not isinstance(raw_s, (list, tuple)) or len(raw_s) != 3:
                raise ValueError(f"{did} 的 start_enu 必须是长度为 3 的数组")
            s_vec = np.array([_finite_float(f"{did}.start_enu[{k}]", raw_s[k]) for k in range(3)], dtype=float)
            self.starts[did] = s_vec

        self.dt = _positive_float("physics_dt", physics_dt)
        self.record_inputs = bool(record_inputs)
        self.input_digests: list[str] = []
        self.deliveries = 0
        self.geom = _NoGeom(False)
        self.in_forest = isinstance(card.get("forest"), dict)
        self.frozen = frozen_for_card(card, policy="explore", density=(card.get("forest") or {}).get("density"))

        crit = card.get("criteria") or {}
        self.radius = _positive_float("criteria.body_radius_m", crit.get("body_radius_m", 0.35))
        self.safe_radius_m = read_safe_radius(card.get("policy_params"))
        self.velocity_mps = _positive_float("task.velocity_mps", task.get("velocity_mps", 9.6))
        self.decoder, self.speed_ratio = build_scaled_decoder(self.velocity_mps, self.safe_radius_m)

        rates = card.get("rates_hz") or {}
        default_plan_hz = float(rates.get("planner", 25))
        self.replan_hz = _positive_float("task.replan_hz", task.get("replan_hz", default_plan_hz))
        self.replan_period_s = 1.0 / self.replan_hz

        default_x_goal = 42.0 if self.in_forest else 19.5
        self.x_goal = _positive_float("task.x_goal_m", task.get("x_goal_m", default_x_goal))
        default_x_finish = 32.0 if self.in_forest else 15.5
        self.x_finish = _positive_float("task.x_finish_m", task.get("x_finish_m", min(default_x_finish, self.x_goal)))
        self.z_goal = _positive_float("task.z_goal_m", task.get("z_goal_m", 1.5))

        default_yb = (-13.5, 13.5) if self.in_forest else (-11.5, 11.5)
        default_zb = (0.75, 3.4) if self.in_forest else (0.75, 3.1)
        self.y_bounds = _bounds_pair("task.y_bounds_m", task.get("y_bounds_m"), default_yb)
        self.z_bounds = _bounds_pair("task.z_bounds_m", task.get("z_bounds_m"), default_zb)

        default_span = (8.0, 30.0) if self.in_forest else (3.7, 14.8)
        self.obstacle_x_span = _bounds_pair("task.obstacle_x_span_m", task.get("obstacle_x_span_m"), default_span)

        self.lookahead_m = _positive_float("task.lookahead_m", task.get("lookahead_m", 10.0))
        self.arrive_radius_m = _positive_float("task.arrive_radius_m", task.get("arrive_radius_m", 4.2 if self.in_forest else 3.4))
        self.clearance_margin_m = _nonneg_float("task.clearance_margin_m", task.get("clearance_margin_m", 0.15))
        self.peer_margin_m = _nonneg_float("task.peer_margin_m", task.get("peer_margin_m", 0.28))
        self.odom_replan_err_m = _positive_float("task.odom_replan_err_m", task.get("odom_replan_err_m", 0.22))
        self.terminal_sep_m = _positive_float(
            "task.terminal_sep_m",
            task.get("terminal_sep_m", max(2.0 * self.radius + self.peer_margin_m + 0.45, 1.6)),
        )

        a_max = dynamics_a_max(card.get("dynamics"))
        v_lim = max(22.0, 3.2 * self.velocity_mps)
        j_lim = 25000.0 * max(1.0, self.speed_ratio**3)
        self.limits = KinematicLimits(v_lim, a_max, j_lim)

        geom_dict = card.get("geometry") if isinstance(card.get("geometry"), dict) else {}
        all_obs = parse_geometry(geom_dict, card.get("ground_z")) if geom_dict else ()
        self._obs_field = _FastObstacleField(all_obs)
        # 检查是否存在靠近 x_goal 的 end_wall 以限制自然制动上界
        end_walls = [
            float(ob.lo[0])
            for ob in all_obs
            if isinstance(ob, Box) and ob.name == "end_wall" and float(ob.lo[0]) > self.x_goal
        ]
        arena_end_x = min(end_walls) - 1.8 if end_walls else self.x_goal + 4.0
        self.x_stop_max = max(self.x_goal, min(self.x_goal + 3.5, arena_end_x))

        self._source = None
        self._sense: dict[str, tuple[object, np.ndarray, np.ndarray, np.ndarray, float]] = {}
        self._committed_traj: dict[str, PiecewisePolynomial] = {}
        self._plan_seq: dict[str, int] = {did: 0 for did in self.members}
        self._plan_hash: dict[str, str] = {}
        self._last_rec: dict[str, _ExecRecord] = {}
        self._follow: dict[str, dict] = {did: {"hash": None, "sampler": None, "shadow_hash": None} for did in self.members}
        self._codes: dict[str, dict[str, int]] = {did: {} for did in self.members}
        self._last_code: dict[str, str] = {did: "LOCAL_HOVER" for did in self.members}
        self._prev_carrot_y: dict[str, float] = {did: float(self.starts[did][1]) for did in self.members}
        self._scan_yaw: dict[str, float | None] = {did: None for did in self.members}

        # 每机终端悬停目标：未锁定时随当前 y 自由平移，进终点带后锁定静止点
        self._terminal_goal: dict[str, np.ndarray | None] = {did: None for did in self.members}
        # 去中心化邻机信息缓存：仅通过 MessageBus.poll 更新！
        self._peer_heard_tubes: dict[str, dict[str, np.ndarray]] = {did: {} for did in self.members}
        self._peer_heard_goals: dict[str, dict[str, np.ndarray]] = {did: {} for did in self.members}

        self._last_replan_t: float | None = None
        self._replan_round = 0
        self.proposals = 0
        self.empty_solution = 0

        # 速度与穿越计时统计
        self._last_obs_tp: dict[str, tuple[float, np.ndarray]] = {}
        self._t_enter_belt: dict[str, float] = {}
        self._t_exit_belt: dict[str, float] = {}
        self._t_finish_plane: dict[str, float] = {}
        self._belt_speeds_3d: dict[str, list[float]] = {did: [] for did in self.members}
        self._flight_speeds_3d: dict[str, list[float]] = {did: [] for did in self.members}

        for did in self.members:
            init_traj = hold_traj(self.starts[did], 0.0, 2.0)
            self._set_commit(did, init_traj)

    @classmethod
    def from_card(cls, card: dict, physics_dt: float, *, record_inputs: bool = False) -> "ExploreFleet":
        return cls(card, physics_dt, record_inputs=record_inputs)

    @property
    def source(self):
        if self._source is None:
            from .net import InProcessSource

            self._source = InProcessSource()
        return self._source

    def _set_commit(self, did: str, traj: PiecewisePolynomial, *, scan_yaw: float | None = None) -> np.ndarray:
        seq = self._plan_seq[did] + 1
        self._plan_seq[did] = seq
        self._committed_traj[did] = traj
        ph = _traj_hash(did, seq, traj)
        self._plan_hash[did] = ph
        self._scan_yaw[did] = scan_yaw
        self._last_rec[did] = _ExecRecord(did, seq, ph)
        t0 = float(traj.t_start)
        horizon = max(float(traj.t_end) - t0 + 0.04, 0.5)
        n = max(2, int(math.ceil(horizon / _SAMPLE_DT_S)) + 1)
        times = np.linspace(t0, t0 + horizon, n, dtype=np.float64)
        pts = _sample_positions(traj, times)
        return np.column_stack((times, pts))

    def check_start(self, drone_id: str, start: np.ndarray) -> None:
        if drone_id not in self.starts:
            raise ValueError(f"未知无人机 {drone_id}")
        expect = self.starts[drone_id]
        if float(np.linalg.norm(np.asarray(start, dtype=float) - expect)) > 1e-6:
            raise ValueError(
                f"{drone_id} 的 start_enu {np.asarray(start, float).tolist()} 与场景卡起点 {expect.tolist()} 不一致"
            )

    def terminal_locked(self, drone_id: str) -> bool:
        return self._terminal_goal.get(drone_id) is not None

    def goal_of(self, drone_id: str) -> np.ndarray:
        locked = self._terminal_goal.get(drone_id)
        if locked is not None:
            return locked.copy()
        sense = self._sense.get(drone_id)
        y_now = float(self.starts[drone_id][1]) if sense is None else float(sense[1][1])
        y_clamped = min(max(y_now, self.y_bounds[0]), self.y_bounds[1])
        return np.array([self.x_goal, y_clamped, self.z_goal], dtype=float)

    def _required_obs_clearance(self) -> float:
        return self.radius + self.clearance_margin_m

    def _required_peer_separation(self) -> float:
        return 2.0 * self.radius + self.peer_margin_m

    def ingest(self, drone_id: str, frame, state) -> None:
        p = np.asarray(state.p, dtype=float).copy()
        v = np.asarray(state.v, dtype=float).copy()
        a = np.asarray(state.a, dtype=float).copy()
        yaw = float(state.yaw)
        self._sense[drone_id] = (frame, p, v, a, yaw)
        spd = float(np.linalg.norm(v))
        if self._terminal_goal.get(drone_id) is None and float(p[0]) > float(self.starts[drone_id][0]) + 0.3:
            self._flight_speeds_3d[drone_id].append(spd)
        x_lo, x_hi = self.obstacle_x_span
        if x_lo <= float(p[0]) <= x_hi:
            self._belt_speeds_3d[drone_id].append(spd)

    def observe(self, drone_id: str, t: float, position: np.ndarray) -> float:
        p = np.asarray(position, dtype=float)
        t_f = float(t)
        prev = self._last_obs_tp.get(drone_id)
        x_lo, x_hi = self.obstacle_x_span
        if prev is not None:
            t0, p0 = prev
            x0, x1 = float(p0[0]), float(p[0])
            if drone_id not in self._t_enter_belt and x0 < x_lo <= x1 and x1 > x0 + 1e-9:
                frac = (x_lo - x0) / (x1 - x0)
                self._t_enter_belt[drone_id] = t0 + frac * (t_f - t0)
            if drone_id not in self._t_exit_belt and x0 < x_hi <= x1 and x1 > x0 + 1e-9:
                frac = (x_hi - x0) / (x1 - x0)
                self._t_exit_belt[drone_id] = t0 + frac * (t_f - t0)
            if drone_id not in self._t_finish_plane and x0 < self.x_finish <= x1 and x1 > x0 + 1e-9:
                frac = (self.x_finish - x0) / (x1 - x0)
                self._t_finish_plane[drone_id] = t0 + frac * (t_f - t0)
        else:
            if float(p[0]) >= x_lo:
                self._t_enter_belt[drone_id] = t_f
        self._last_obs_tp[drone_id] = (t_f, p.copy())
        return 0.0

    def _active_bus(self, bus):
        override = self.transport_override
        if override is not None:
            return override.carrying(bus)
        return bus

    def _drain_bus(self, bus, now: float) -> None:
        for env in bus.poll(now):
            payload = env.payload
            if not isinstance(payload, dict) or payload.get("kind") != "explore_traj":
                continue
            self.deliveries += 1
            dest = str(env.dest)
            src = str(env.src)
            if dest not in self._peer_heard_tubes or src == dest:
                continue
            raw_bytes = payload.get("samples_bytes")
            if isinstance(raw_bytes, (bytes, bytearray)) and len(raw_bytes) >= 32:
                arr = np.frombuffer(raw_bytes, dtype=np.float64).reshape(-1, 4).copy()
                self._peer_heard_tubes[dest][src] = arr
            else:
                samples = payload.get("samples")
                if samples is not None:
                    arr = np.asarray(samples, dtype=np.float64)
                    if arr.ndim == 2 and arr.shape[1] == 4 and arr.shape[0] >= 1:
                        self._peer_heard_tubes[dest][src] = arr
            tgoal = payload.get("terminal_goal")
            if tgoal is not None:
                g_arr = np.asarray(tgoal, dtype=np.float64)
                if g_arr.shape == (3,):
                    self._peer_heard_goals[dest][src] = g_arr

    def _broadcast_commit(self, bus, now: float, src: str, tube: np.ndarray) -> None:
        tgoal = self._terminal_goal.get(src)
        # 用不可变 bytes 承载轨迹采样，使 MessageBus.deepcopy 开销为 O(1) 且各机完全隔离不共享可变引用
        payload = {
            "kind": "explore_traj",
            "owner": src,
            "seq": int(self._plan_seq[src]),
            "samples_bytes": np.ascontiguousarray(tube, dtype=np.float64).tobytes(),
            "terminal_goal": None if tgoal is None else tuple(float(x) for x in tgoal),
        }
        for dest in self.members:
            if dest == src:
                continue
            bus.publish(now, src=src, dest=dest, topic="explore", payload=payload)

    def exchange(self, bus, now: float) -> None:
        active_bus = self._active_bus(bus)
        now_f = float(now)
        self._drain_bus(active_bus, now_f)

        # 第 0 拍先广播初始悬停管，确保同拍低优先级机已知全体初始位置
        if self._last_replan_t is None:
            for did in self.members:
                init_tube = np.array(
                    [
                        [now_f, *self.starts[did].tolist()],
                        [now_f + 2.0, *self.starts[did].tolist()],
                    ],
                    dtype=np.float64,
                )
                self._broadcast_commit(active_bus, now_f, did, init_tube)
            self._drain_bus(active_bus, now_f)
        elif now_f - self._last_replan_t < self.replan_period_s - 1e-9:
            return

        self._last_replan_t = now_f
        n = len(self.members)
        shift = self._replan_round % n
        self._replan_round += 1
        order = [self.members[(shift + i) % n] for i in range(n)]

        for did in order:
            tube = self._plan_one_drone(did, now_f)
            if tube is not None:
                self._broadcast_commit(active_bus, now_f, did, tube)
                # 零时延总线下当拍即时送达给后续优先级飞机，杜绝同拍互撞；故障总线下按真实延迟送达
                self._drain_bus(active_bus, now_f)

    def _head_state(self, did: str, now: float) -> np.ndarray:
        traj = self._committed_traj[did]
        p_ref, v_ref, a_ref = _eval_pva(traj, now)
        head = np.array([p_ref, v_ref, a_ref], dtype=float)
        sense = self._sense.get(did)
        if sense is None:
            return head
        _, p_m, v_m, a_m, _ = sense
        if float(np.linalg.norm(p_ref - p_m)) > self.odom_replan_err_m:
            return np.array([p_m, v_m, a_m], dtype=float)
        return head

    def _count(self, did: str, code: str) -> None:
        bucket = self._codes[did]
        bucket[code] = bucket.get(code, 0) + 1
        self._last_code[did] = code

    def _digest(self, did: str, now: float, head: np.ndarray, carrot: np.ndarray, yaw: float, frame) -> str:
        h = hashlib.sha256()
        h.update(did.encode("utf-8"))
        h.update(np.float64(now).tobytes())
        h.update(np.asarray(head, dtype=np.float64).tobytes())
        h.update(np.asarray(carrot, dtype=np.float64).tobytes())
        h.update(np.float64(yaw).tobytes())
        h.update(np.float64(frame.stamp).tobytes())
        h.update(np.ascontiguousarray(np.asarray(frame.depth), dtype=np.float32).tobytes())
        return h.hexdigest()

    def _lock_terminal_goal(self, did: str, head: np.ndarray) -> np.ndarray:
        """进入障碍后空旷终点带时，就近锁定与所有邻机及障碍无冲突的三维静止终点。"""
        existing = self._terminal_goal.get(did)
        if existing is not None:
            return existing
        p = head[0]
        v = head[1]
        # 自然舒适减速前移量，保证单调向前刹停而不倒车
        vx = max(float(v[0]), 0.0)
        comfort_dx = (vx * vx) / max(1.35 * self.limits.a_max, 1e-6)
        pref_x = min(max(float(p[0]) + comfort_dx, self.x_goal), self.x_stop_max)
        pref_y = min(max(float(p[1]) + 0.25 * float(v[1]), self.y_bounds[0]), self.y_bounds[1])

        peer_goals = list(self._peer_heard_goals[did].values())
        req_obs = self._required_obs_clearance() + 0.15

        candidates: list[tuple[float, np.ndarray]] = []
        y_offsets = (0.0, -1.6, 1.6, -3.2, 3.2, -4.8, 4.8, -6.4, 6.4)
        x_offsets = (0.0, 1.2, -1.0, 2.4)
        for dx in x_offsets:
            cx = min(max(pref_x + dx, self.x_goal), self.x_stop_max)
            for dy in y_offsets:
                cy = pref_y + dy
                if not (self.y_bounds[0] <= cy <= self.y_bounds[1]):
                    continue
                pt = np.array([cx, cy, self.z_goal], dtype=float)
                if self._obs_field.min_distance(pt[None, :]) < req_obs:
                    continue
                if any(float(np.linalg.norm(pt - pg)) < self.terminal_sep_m for pg in peer_goals):
                    continue
                cost = abs(dy) + 0.6 * abs(cx - pref_x)
                candidates.append((cost, pt))
        if candidates:
            candidates.sort(key=lambda item: item[0])
            chosen = candidates[0][1]
        else:
            chosen = np.array([pref_x, pref_y, self.z_goal], dtype=float)
        self._terminal_goal[did] = chosen
        return chosen

    def _select_carrots(
        self,
        did: str,
        head_p: np.ndarray,
        yaw: float,
        depth: np.ndarray,
        peers: list[np.ndarray],
        now: float,
    ) -> tuple[list[np.ndarray], float]:
        """利用本机深度图扇区净空与邻机预测位置，生成无槽位约束的最优前向引导点列表及最开阔偏航角。"""
        d = np.asarray(depth, dtype=np.float32)
        clean = np.where(np.isfinite(d) & (d > 0.05), np.minimum(d, 20.0), 20.0)
        h, w = clean.shape
        r_mid0, r_mid1 = max(0, int(0.26 * h)), min(h, int(0.74 * h))
        r_hi0, r_hi1 = max(0, int(0.08 * h)), max(1, int(0.32 * h))

        n_sec = 15
        centers = np.linspace(12, w - 13, n_sec)
        half_w = max(6, w // 16)
        hfov_rad = math.radians(float(self.decoder.horizon_fov_deg))

        # 邻机 0.6 s 后的预测位置，用于软排斥分流到不同走廊
        peer_pts = []
        t_probe = now + 0.60
        for nb in peers:
            if nb.size == 0:
                continue
            ts = nb[:, 0]
            px = float(np.interp(t_probe, ts, nb[:, 1]))
            py = float(np.interp(t_probe, ts, nb[:, 2]))
            pz = float(np.interp(t_probe, ts, nb[:, 3]))
            if abs(px - float(head_p[0])) < 9.0:
                peer_pts.append((px, py, pz))

        L = min(max(self.x_goal - float(head_p[0]) + 1.5, 4.5), self.lookahead_m)
        y_lo, y_hi = self.y_bounds
        z_lo, z_hi = self.z_bounds
        prev_y = self._prev_carrot_y[did]

        scored: list[tuple[float, np.ndarray, float, float]] = []
        best_open_yaw = yaw
        best_open_range = -1.0

        for c_px in centers:
            c0 = max(0, int(round(c_px)) - half_w)
            c1 = min(w, int(round(c_px)) + half_w)
            # 图像左列对应机体系 +y（左偏航角为正）
            cam_yaw = (0.5 - float(c_px) / max(w - 1, 1)) * hfov_rad
            world_yaw = yaw + cam_yaw
            d_mid = float(np.percentile(clean[r_mid0:r_mid1, c0:c1], 22))
            d_hi = float(np.percentile(clean[r_hi0:r_hi1, c0:c1], 25))
            if d_mid > best_open_range:
                best_open_range = d_mid
                best_open_yaw = world_yaw

            cos_w = math.cos(world_yaw)
            if cos_w <= 0.15:
                continue
            cx = min(self.x_stop_max, float(head_p[0]) + max(3.0, L * cos_w))
            cy_raw = float(head_p[1]) + L * math.sin(world_yaw)
            cy = min(max(cy_raw, y_lo), y_hi)

            # 若中层被矮枝遮挡而上层开阔，轻抬引导高度越过矮枝
            cz = self.z_goal
            if d_mid < 5.5 and d_hi > d_mid + 2.5:
                cz = min(z_hi - 0.35, max(cz, 2.25))
            cz = min(max(cz, z_lo + 0.2), z_hi - 0.2)

            # 邻机分流项
            peer_pen = 0.0
            for _px, py, _pz in peer_pts:
                dy_peer = abs(cy - py)
                if dy_peer < 2.4:
                    peer_pen += 2.6 * (1.0 - dy_peer / 2.4)

            wall_pen = 0.8 * abs(cy_raw - cy)
            smooth_pen = 0.07 * abs(cy - prev_y)
            util = 2.2 * cos_w + 0.42 * min(d_mid, 15.0) - peer_pen - wall_pen - smooth_pen
            scored.append((util, np.array([cx, cy, cz], dtype=float), float(cy), d_mid))

        scored.sort(key=lambda item: item[0], reverse=True)
        carrots: list[np.ndarray] = []
        picked_y: list[float] = []
        for _u, pt, cy, _dm in scored:
            if all(abs(cy - py) > 1.8 for py in picked_y):
                carrots.append(pt)
                picked_y.append(cy)
                if len(carrots) == 2:
                    break

        if not carrots:
            y_clamp = min(max(float(head_p[1]), y_lo), y_hi)
            carrots.append(np.array([min(self.x_goal, float(head_p[0]) + L), y_clamp, self.z_goal], dtype=float))
        self._prev_carrot_y[did] = float(carrots[0][1])
        return carrots, best_open_yaw

    def _check_traj_safety(
        self,
        traj: PiecewisePolynomial,
        peers: list[np.ndarray],
        *,
        obs_req: float | None = None,
        peer_req: float | None = None,
    ) -> tuple[bool, float, float]:
        """单次向量化采样完成运动学、走廊边界、静态障碍净空与邻机三维管净空联合校验。"""
        span = max(float(traj.t_end) - float(traj.t_start), _SAMPLE_DT_S)
        n = max(4, int(math.ceil(span / _SAMPLE_DT_S)) + 1)
        times = np.linspace(float(traj.t_start), float(traj.t_end), n, dtype=np.float64)
        pts, vel, acc, jrk = _sample_pvaj(traj, times)
        if not _fast_kin_ok(vel, acc, jrk, self.limits):
            return False, 0.0, 0.0
        y_lo, y_hi = self.y_bounds
        z_lo, z_hi = self.z_bounds
        if (
            float(np.min(pts[:, 1])) < y_lo - 0.6
            or float(np.max(pts[:, 1])) > y_hi + 0.6
            or float(np.min(pts[:, 2])) < z_lo - 0.15
            or float(np.max(pts[:, 2])) > z_hi + 0.35
        ):
            return False, 0.0, 0.0

        need_obs = self._required_obs_clearance() if obs_req is None else float(obs_req)
        obs_d = self._obs_field.min_distance(pts)
        if obs_d + 1e-9 < need_obs:
            return False, obs_d, 0.0

        need_peer = self._required_peer_separation() if peer_req is None else float(peer_req)
        peer_d = _peer_tube_min_distance(times, pts, peers)
        if peer_d + 1e-9 < need_peer:
            return False, obs_d, peer_d
        return True, obs_d, peer_d

    def _best_yopo_motion(
        self,
        head: np.ndarray,
        t_start: float,
        carrot: np.ndarray,
        depth: np.ndarray,
        yaw: float,
        peers: list[np.ndarray],
    ) -> tuple[PiecewisePolynomial | None, str]:
        """前向一次 YOPO 并对全部 15 条候选打分，选出满足安全盾且前向速度最高的轨迹。"""
        self.proposals += 1
        R = R_yaw(yaw)
        r = self.speed_ratio
        vel_in = head[1] / r
        acc_in = head[2] / (r * r)
        params, score, radius = self.source.forward_raw(depth, vel_in, acc_in, carrot, head[0], R)

        dec = self.decoder
        inner_b, tail_b, _ = decode_params(params, dec)
        p_raw = np.asarray(params, dtype=float)
        durations = np.maximum(
            (p_raw[:, 12:14] + 1.0) * (0.5 * dec.segment_time),
            _DURATION_MIN_TRAIN / r,
        )
        inner_w = inner_b @ R.T + head[0]
        tail_w = np.einsum("ij,nkj->nki", R, tail_b)
        tail_w[:, 0] += head[0]

        mu_lo = np.asarray(corridor_mu_lo(radius, dec), dtype=np.float64)
        admitted = mu_lo >= float(dec.safe_mu)
        score_arr = np.asarray(score, dtype=np.float64)
        s_shift = score_arr - float(np.max(score_arr))

        req_obs = self._required_obs_clearance()
        req_peer = self._required_peer_separation()

        best_util = -math.inf
        best_traj: PiecewisePolynomial | None = None
        best_admitted = False

        # 先按 (准入优先, 分数与前向位移) 排序全部 15 个候选，逐条取最长可行安全前缀+制动尾
        cand_order = sorted(
            range(int(dec.count)),
            key=lambda i: (bool(admitted[i]), float(tail_w[i, 0, 0] - head[0, 0]) + 0.5 * float(s_shift[i])),
            reverse=True,
        )

        for idx in cand_order:
            is_adm = bool(admitted[idx])
            # 若已有准入安全解，不再尝试非准入候选
            if best_admitted and not is_adm:
                continue
            try:
                raw = minco_two_piece(t_start, head, tail_w[idx], inner_w[idx], durations[idx])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            total = float(raw.t_end - raw.t_start)
            if total < 0.12:
                continue

            obs_gate = req_obs if is_adm else req_obs + 0.04
            for frac in (1.0, 0.78, 0.58, 0.42, 0.28):
                h = frac * total
                if h < 0.10:
                    continue
                try:
                    pre = _prefix(raw, raw.t_start + h)
                    cand_traj = _append_stop(pre, self.limits)
                except (ValueError, RuntimeError, np.linalg.LinAlgError, FloatingPointError):
                    continue
                if cand_traj is None:
                    continue
                ok, obs_d, peer_d = self._check_traj_safety(cand_traj, peers, obs_req=obs_gate, peer_req=req_peer)
                if not ok:
                    continue
                eval_dt = min(h, 0.45)
                p_eval = np.asarray(pre.evaluate(t_start + eval_dt, 0), dtype=float)
                vx_avg = float(p_eval[0] - head[0, 0]) / max(eval_dt, 1e-3)
                obs_bonus = min(obs_d - req_obs, 0.65)
                peer_bonus = 0.65 if math.isinf(peer_d) else min(peer_d - req_peer, 0.80)
                adm_bonus = 2.0 if is_adm else 0.0
                util = (
                    1.45 * vx_avg
                    + 0.30 * float(s_shift[idx])
                    + 1.25 * obs_bonus
                    + 0.85 * peer_bonus
                    + 0.90 * frac
                    + adm_bonus
                )
                if util > best_util:
                    best_util = util
                    best_traj = cand_traj
                    best_admitted = best_admitted or is_adm
                # 对同一条曲线，一旦最长可行前缀通过即可跳出缩短循环
                break

        if best_traj is None:
            return None, ("CORRIDOR_EMPTY_HOVER" if not bool(np.any(admitted)) else "CLEARANCE_SKIP")
        return best_traj, ("OK" if best_admitted else "YOPO_RELAXED_SAFE")

    def _escape_sidestep(
        self,
        head: np.ndarray,
        t_start: float,
        peers: list[np.ndarray],
    ) -> PiecewisePolynomial | None:
        """当 15 条高速候选均受阻时，搜索短程横向/斜前/跃枝五次逃逸机动并接安全刹停。"""
        p0, v0, _a0 = head
        y_lo, y_hi = self.y_bounds
        z_lo, z_hi = self.z_bounds
        best_traj = None
        best_score = -math.inf
        req_obs = self._required_obs_clearance()
        req_peer = self._required_peer_separation()

        offsets = (
            (2.2, -1.5, 0.0),
            (2.2, 1.5, 0.0),
            (1.6, -2.2, 0.0),
            (1.6, 2.2, 0.0),
            (2.4, 0.0, 0.65),
            (1.8, -1.4, 0.55),
            (1.8, 1.4, 0.55),
            (1.0, -1.8, 0.0),
            (1.0, 1.8, 0.0),
            (0.4, -1.2, 0.0),
            (0.4, 1.2, 0.0),
        )
        duration = max(0.48, min(0.75, float(np.linalg.norm(v0)) / max(self.limits.a_max, 1.0) + 0.42))
        for dx, dy, dz in offsets:
            ty = float(p0[1]) + dy
            tz = float(p0[2]) + dz
            if not (y_lo <= ty <= y_hi and z_lo + 0.1 <= tz <= z_hi - 0.1):
                continue
            target_p = np.array([min(self.x_stop_max, float(p0[0]) + dx), ty, tz], dtype=float)
            end_v = np.array([min(3.5, max(1.5, dx / duration)), 0.5 * dy / duration, 0.0], dtype=float)
            end = np.array([target_p, end_v, np.zeros(3)], dtype=float)
            try:
                seg = quintic(head, end, duration)
                raw = PiecewisePolynomial(t_start, [seg])
                cand = _append_stop(raw, self.limits)
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            if cand is None:
                continue
            ok, obs_d, peer_d = self._check_traj_safety(cand, peers)
            if not ok:
                continue
            peer_b = 0.6 if math.isinf(peer_d) else min(peer_d - req_peer, 0.6)
            sc = 1.5 * dx + 1.2 * min(obs_d - req_obs, 0.6) + 0.8 * peer_b - 0.15 * abs(dy)
            if sc > best_score:
                best_score = sc
                best_traj = cand
        return best_traj

    def _stop_at_terminal(
        self,
        head: np.ndarray,
        t_start: float,
        goal: np.ndarray,
        peers: list[np.ndarray],
    ) -> PiecewisePolynomial | None:
        end = np.array([goal, np.zeros(3), np.zeros(3)], dtype=float)
        dist = float(np.linalg.norm(goal - head[0]))
        spd = float(np.linalg.norm(head[1]))
        base_t = max(0.45, dist / max(0.65 * self.velocity_mps, 2.0), 1.55 * spd / max(self.limits.a_max, 1.0))
        duration = base_t
        for _ in range(12):
            try:
                traj = PiecewisePolynomial(t_start, [quintic(head, end, duration)])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                duration *= 1.22
                continue
            ok, _, _ = self._check_traj_safety(traj, peers)
            if ok:
                return traj
            duration *= 1.22
        return None

    def _plan_one_drone(self, did: str, now: float) -> np.ndarray | None:
        sense = self._sense.get(did)
        if sense is None or sense[0] is None:
            self._count(did, "NO_DEPTH")
            return None
        frame, p_meas, _v_meas, _a_meas, yaw = sense
        head = self._head_state(did, now)
        peers = list(self._peer_heard_tubes[did].values())

        # 已在锁定终点停稳：保持静止，不再重复跑网络
        locked = self._terminal_goal.get(did)
        if locked is not None:
            cur_end = np.asarray(self._committed_traj[did].evaluate(self._committed_traj[did].t_end, 0), dtype=float)
            if (
                float(np.linalg.norm(cur_end - locked)) <= 0.04
                and float(np.linalg.norm(p_meas - locked)) <= 0.25
                and float(np.linalg.norm(head[1])) <= 0.05
            ):
                self._count(did, "ARRIVED_HOLD")
                return None

        # 越过障碍带进入终点区：锁定无冲突静止终点并平滑刹停
        if locked is not None or float(head[0, 0]) >= self.x_goal - self.arrive_radius_m:
            target = self._lock_terminal_goal(did, head)
            if self.record_inputs:
                self.input_digests.append(self._digest(did, now, head, target, yaw, frame))
            stop_traj = self._stop_at_terminal(head, now, target, peers)
            if stop_traj is not None:
                self._count(did, "ARRIVED_HOLD")
                return self._set_commit(did, stop_traj)

        carrots, open_yaw = self._select_carrots(did, head[0], yaw, frame.depth, peers, now)
        if self.record_inputs:
            self.input_digests.append(self._digest(did, now, head, carrots[0], yaw, frame))

        last_err = "CORRIDOR_EMPTY_HOVER"
        for ci, carrot in enumerate(carrots):
            try:
                traj, code = self._best_yopo_motion(head, now, carrot, frame.depth, yaw, peers)
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                traj, code = None, "YOPO_FAIL"
            if traj is not None:
                tag = "CORRIDOR_SHIFT" if ci > 0 else code
                self._count(did, tag)
                return self._set_commit(did, traj)
            last_err = code

        if last_err == "CORRIDOR_EMPTY_HOVER":
            self.empty_solution += 1

        # 逃逸侧移机动
        esc = self._escape_sidestep(head, now, peers)
        if esc is not None:
            self._count(did, "ESCAPE_SIDESTEP")
            return self._set_commit(did, esc, scan_yaw=open_yaw)

        # 安全制动分支回退；若制动也因极近邻机受限则沿用已验证递归安全的上一条可停轨迹并转向开阔扇区
        if float(np.linalg.norm(head[1])) > 0.05:
            brk = _fast_brake_traj(now, head, self.limits)
            if brk is not None:
                ok, _, _ = self._check_traj_safety(brk, peers)
                if ok:
                    self._count(did, "BRAKE_SAFE")
                    return self._set_commit(did, brk, scan_yaw=open_yaw)
        else:
            hold = hold_traj(head[0], now, now + 1.0)
            ok, _, _ = self._check_traj_safety(hold, peers)
            if ok:
                self._count(did, "HOLD_SCAN")
                return self._set_commit(did, hold, scan_yaw=open_yaw)

        self._scan_yaw[did] = open_yaw
        if self._follow[did]["sampler"] is not None:
            self._follow[did]["sampler"].scan_yaw = open_yaw
        self._count(did, "KEEP_SAFE_TAIL")
        return None

    def track(self, drone_id: str, now: float) -> _ExploreTrack:
        ph = self._plan_hash[drone_id]
        slot = self._follow[drone_id]
        goal = self.goal_of(drone_id)
        if slot["hash"] != ph or slot["sampler"] is None:
            yaw = 0.0 if slot["sampler"] is None else float(slot["sampler"].yaw)
            max_yr = min(1.15, max(0.65, 0.55 * self.speed_ratio))
            slot["sampler"] = _ExploreTrack(
                self._committed_traj[drone_id],
                goal,
                yaw,
                self.dt,
                max_yaw_rate=max_yr,
                scan_yaw=self._scan_yaw[drone_id],
            )
            slot["hash"] = ph
        else:
            slot["sampler"].goal = goal
        return slot["sampler"]

    def shadow_view(self, drone_id: str) -> _ExploreTrack | None:
        ph = self._plan_hash[drone_id]
        slot = self._follow[drone_id]
        if slot.get("shadow_hash") == ph:
            return None
        slot["shadow_hash"] = ph
        yaw = 0.0 if slot["sampler"] is None else float(slot["sampler"].yaw)
        return _ExploreTrack(self._committed_traj[drone_id], self.goal_of(drone_id), yaw, self.dt)

    def last_record(self, drone_id: str) -> _ExecRecord:
        return self._last_rec[drone_id]

    def code(self, drone_id: str) -> str:
        return self._last_code[drone_id]

    def summary(self, arrived: int, n_drones: int) -> dict:
        decisions: dict[str, int] = {}
        commits: dict[str, int] = {}
        for did in self.members:
            commits[did] = max(0, int(self._plan_seq[did]) - 1)
            for k, v in self._codes[did].items():
                decisions[k] = decisions.get(k, 0) + int(v)

        span_len = float(self.obstacle_x_span[1] - self.obstacle_x_span[0])
        per_belt_vx: dict[str, float | None] = {}
        per_belt_3d: dict[str, float | None] = {}
        per_flight_3d: dict[str, float | None] = {}
        belt_vx_vals: list[float] = []
        belt_3d_vals: list[float] = []
        flight_3d_vals: list[float] = []

        for did in self.members:
            t0 = self._t_enter_belt.get(did)
            t1 = self._t_exit_belt.get(did)
            if t0 is not None and t1 is not None and t1 > t0 + 1e-6:
                vx = span_len / (t1 - t0)
                per_belt_vx[did] = vx
                belt_vx_vals.append(vx)
            else:
                per_belt_vx[did] = None

            b3 = self._belt_speeds_3d[did]
            if b3:
                m_b3 = float(np.mean(b3))
                per_belt_3d[did] = m_b3
                belt_3d_vals.append(m_b3)
            else:
                per_belt_3d[did] = None

            f3 = self._flight_speeds_3d[did]
            if f3:
                m_f3 = float(np.mean(f3))
                per_flight_3d[did] = m_f3
                flight_3d_vals.append(m_f3)
            else:
                per_flight_3d[did] = None

        finish_times = [self._t_finish_plane[did] for did in self.members if did in self._t_finish_plane]
        all_finished_plane = len(finish_times) == len(self.members)

        return {
            "mode": "formation-free-horizontal-explore",
            "error_p95_m": None,
            "error_max_m": None,
            "arrival_rate": float(arrived) / float(n_drones),
            "frozen": self.frozen,
            "adjudicator": None,
            "avoidance": "tju-yopo-time-scaled-15cand",
            "goal_bias": "free-lateral-depth-corridor",
            "progress_gate": False,
            "tdma_turn_waiting": False,
            "replan_hz": self.replan_hz,
            "velocity_mps": self.velocity_mps,
            "speed_ratio": self.speed_ratio,
            "x_goal_m": self.x_goal,
            "x_finish_m": self.x_finish,
            "obstacle_x_span_m": list(self.obstacle_x_span),
            "y_bounds_m": list(self.y_bounds),
            "z_bounds_m": list(self.z_bounds),
            "body_radius_m": self.radius,
            "safe_radius_m": self.safe_radius_m,
            "clearance_margin_m": self.clearance_margin_m,
            "clearance_required_m": self._required_obs_clearance(),
            "peer_margin_m": self.peer_margin_m,
            "peer_separation_required_m": self._required_peer_separation(),
            "fly_through_finish_time_s": max(finish_times) if all_finished_plane else None,
            "avg_belt_vx_mps": float(np.mean(belt_vx_vals)) if belt_vx_vals else None,
            "avg_belt_speed_mps": float(np.mean(belt_3d_vals)) if belt_3d_vals else None,
            "avg_forest_vx_mps": float(np.mean(belt_vx_vals)) if (self.in_forest and belt_vx_vals) else None,
            "avg_forest_speed_mps": float(np.mean(belt_3d_vals)) if (self.in_forest and belt_3d_vals) else None,
            "avg_flight_speed_mps": float(np.mean(flight_3d_vals)) if flight_3d_vals else None,
            "per_drone_belt_vx_mps": per_belt_vx,
            "per_drone_belt_speed_mps": per_belt_3d,
            "per_drone_flight_speed_mps": per_flight_3d,
            "empty_solution_count": int(self.empty_solution),
            "proposals": int(self.proposals),
            "local_commits": commits,
            "decisions": dict(sorted(decisions.items())),
            "final_goals_enu": {did: self.goal_of(did).tolist() for did in self.members},
            "committed_plan_hash": dict(self._plan_hash),
        }
