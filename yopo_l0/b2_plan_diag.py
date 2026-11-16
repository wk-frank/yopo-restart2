"""票 B2：交叉卡规划净空归因的只读记录器。默认不装，装上也不改任何选择。

L0 和 SITL 用同一个记录器，对照「规划输入」：

* 规划时刻：本机里程计（深度视点）和本机正在执行的计划在同一时刻差多少（跟踪误差），
  拆成沿航向和侧向；计划头（t_start 上的计划状态）离深度视点多远。
* 深度帧和规划时刻的对齐：最新一帧深度的年龄（规划时刻减帧时刻）。
* 每次前向留下几条过了运动学的候选（``ranking.order[:6]`` 里拟合成功的条数）。
* 每份提案的几何净空：计划运动段上到场景卡障碍表面的最小距离减机体半径。
* 每拍：设定点（已提交计划）的净空、里程计的净空和两者之差。

所有时刻都用协议钟（L0 从 0 起；SITL 从全体进入 PLAN 起）。里程计记的是 ingest 进来的那份。
"""

from __future__ import annotations

import dataclasses
import math
from typing import Callable

import numpy as np

from yopo_swarm_core.plan import state_at

SCHEMA = "yopo-restart/b2-plan-diag/1"
_SAMPLE_DT = 0.02
_MAX_EVENTS = 4000


def _r(x, nd=4):
    return None if x is None or not math.isfinite(float(x)) else round(float(x), nd)


def _split(delta: np.ndarray, heading_xy: np.ndarray) -> tuple[float, float]:
    """水平差拆成沿航向和侧向（左正）。航向长度为 0 时全算侧向的模。"""
    d = np.asarray(delta, dtype=float)[:2]
    n = float(np.linalg.norm(heading_xy))
    if n < 1e-6:
        return 0.0, float(np.linalg.norm(d))
    u = np.asarray(heading_xy, dtype=float) / n
    along = float(d @ u)
    lat = float(u[0] * d[1] - u[1] * d[0])
    return along, lat


def _stats(vals: list[float]) -> dict | None:
    v = [float(x) for x in vals if x is not None and math.isfinite(float(x))]
    if not v:
        return None
    a = np.asarray(v)
    return {
        "n": len(v),
        "median": _r(np.median(a)),
        "p95": _r(np.percentile(a, 95)),
        "max": _r(a.max()),
        "min": _r(a.min()),
    }


class PlanDiag:
    """装在一个 CrossingFleet 实例上。``distance`` 是场景卡点到障碍表面的距离。"""

    def __init__(
        self,
        fleet,
        distance: Callable[[np.ndarray], float],
        body_radius: float,
        *,
        abs_offset: Callable[[], float] | None = None,
        truth: Callable[[str], np.ndarray | None] | None = None,
    ):
        self.fleet = fleet
        self.distance = distance
        self.body = float(body_radius)
        # SITL：ingest 的状态时刻是绝对仿真钟，协议钟要减去 PLAN 起点。L0 两者相同。
        self.abs_offset = abs_offset or (lambda: 0.0)
        # SITL：gz 真值位置（世界系）。L0 没有，记 None。
        self.truth = truth
        self.state: dict[str, tuple[np.ndarray, np.ndarray, float]] = {}
        self.frame_pose: dict[str, tuple[int, np.ndarray, float]] = {}
        self.plans: list[dict] = []
        self.ticks: dict[str, list[dict]] = {did: [] for did in fleet.members}
        self._options_n: int | None = None
        self._install()

    # ---- 装钩子（实例属性，不改类） ----
    def _install(self) -> None:
        fleet = self.fleet
        ingest = fleet.ingest
        track = fleet.track
        options = fleet._yopo_options

        def ingest_hook(drone_id, frame, state):
            p = np.asarray(state.p, dtype=float).copy()
            v = np.asarray(state.v, dtype=float).copy()
            t = float(state.t) - float(self.abs_offset())
            self.state[drone_id] = (p, v, t)
            if frame is not None:
                prev = self.frame_pose.get(drone_id)
                if prev is None or prev[0] != id(frame):
                    pose = getattr(frame, "_position", None)
                    pose = p if pose is None else np.asarray(pose, dtype=float).copy()
                    self.frame_pose[drone_id] = (id(frame), pose, float(frame.stamp) - float(self.abs_offset()))
            return ingest(drone_id, frame, state)

        def track_hook(drone_id, now):
            out = track(drone_id, now)
            self._tick(drone_id, float(now))
            return out

        def options_hook(*args, **kwargs):
            motions, code = options(*args, **kwargs)
            self._options_n = len(motions)
            return motions, code

        fleet.ingest = ingest_hook
        fleet.track = track_hook
        fleet._yopo_options = options_hook
        for did, node in fleet.nodes.items():
            planner = node.planner

            def plan_hook(n, now, t_start, _planner=planner, _did=did):
                self._options_n = None
                parent = n.committed.occupancy.plan
                occ = _planner(n, now, t_start)
                self._plan(_did, float(now), float(t_start), parent, occ)
                return occ

            node.planner = plan_hook

    # ---- 记录 ----
    def _clear(self, p) -> float:
        return float(self.distance(np.asarray(p, dtype=float))) - self.body

    def _motion_clearance(self, plan) -> tuple[float, float]:
        tr = plan.motion
        t0, t1 = float(tr.t_start), float(tr.t_end)
        n = max(2, int(math.ceil((t1 - t0) / _SAMPLE_DT)) + 1)
        best, at = math.inf, t0
        for t in np.linspace(t0, t1, n):
            c = self._clear(tr.evaluate(float(t)))
            if c < best:
                best, at = c, float(t)
        return best, at

    def _plan(self, did: str, now: float, t_start: float, parent, occ) -> None:
        if len(self.plans) >= _MAX_EVENTS:
            return
        st = self.state.get(did)
        fr = self.frame_pose.get(did)
        row = {"uav": did, "now": _r(now, 3), "t_start": _r(t_start, 3), "code": self.fleet.code(did),
               "options": self._options_n, "proposed": occ is not None}
        p_now, v_now, _a = state_at(parent, now)
        head = state_at(parent, t_start)[0]
        heading = np.asarray(v_now, dtype=float)[:2]
        if st is not None:
            odom_p, odom_v, odom_t = st
            along, lat = _split(odom_p - p_now, heading)
            row.update(
                odom_age_s=_r(now - odom_t, 3),
                track_err_m=_r(np.linalg.norm(odom_p - p_now)),
                track_along_m=_r(along),
                track_lat_m=_r(lat),
                speed_plan=_r(np.linalg.norm(v_now)),
                speed_odom=_r(np.linalg.norm(odom_v)),
                odom_clear_m=_r(self._clear(odom_p)),
            )
        if fr is not None:
            _fid, pose, stamp = fr
            along, lat = _split(head - pose, heading)
            row.update(frame_age_s=_r(now - stamp, 3), head_from_view_along_m=_r(along), head_from_view_lat_m=_r(lat))
        if occ is not None:
            c, at = self._motion_clearance(occ.plan)
            row.update(plan_clear_m=_r(c), plan_clear_at=_r(at, 3), plan_t_end=_r(occ.plan.t_end, 3))
        self.plans.append(row)

    def _tick(self, did: str, now: float) -> None:
        st = self.state.get(did)
        if st is None:
            return
        rows = self.ticks[did]
        if len(rows) >= _MAX_EVENTS:
            return
        rec = self.fleet._last_rec.get(did)
        if rec is None:
            return
        sp, sv, _sa = state_at(rec.occupancy.plan, now)
        odom_p = st[0]
        true_p = None if self.truth is None else self.truth(did)
        along, lat = _split(odom_p - sp, np.asarray(sv)[:2])
        rows.append({
            "t": round(now, 3),
            "sp": [float(x) for x in sp],
            "odom": [float(x) for x in odom_p],
            "truth": None if true_p is None else [float(x) for x in true_p],
            "truth_clear": None if true_p is None else self._clear(true_p),
            "sp_clear": self._clear(sp),
            "odom_clear": self._clear(odom_p),
            "err": float(np.linalg.norm(odom_p - sp)),
            "along": along,
            "lat": lat,
            "speed": float(np.linalg.norm(sv)),
            "x": float(odom_p[0]),
        })

    # ---- 汇总 ----
    def summary(self) -> dict:
        per = {}
        for did, rows in self.ticks.items():
            moving = [r for r in rows if r["speed"] > 0.5]
            worst = min(rows, key=lambda r: r["odom_clear"]) if rows else None
            worst_sp = min(rows, key=lambda r: r["sp_clear"]) if rows else None
            per[did] = {
                "ticks": len(rows),
                "track_err_moving": _stats([r["err"] for r in moving]),
                "track_lat_abs_moving": _stats([abs(r["lat"]) for r in moving]),
                "track_along_moving": _stats([r["along"] for r in moving]),
                "min_setpoint_clear_m": None if worst_sp is None else _r(worst_sp["sp_clear"]),
                "min_setpoint_clear_t": None if worst_sp is None else worst_sp["t"],
                "min_odom_clear_m": None if worst is None else _r(worst["odom_clear"]),
                "at_min_odom": None if worst is None else {
                    "t": worst["t"], "x": _r(worst["x"], 3), "setpoint_clear_m": _r(worst["sp_clear"]),
                    "err_m": _r(worst["err"]), "along_m": _r(worst["along"]), "lat_m": _r(worst["lat"]),
                    "speed": _r(worst["speed"], 3)},
            }
        proposed = [p for p in self.plans if p.get("proposed")]
        moving_plans = [p for p in self.plans if (p.get("speed_plan") or 0.0) > 0.5]
        return {
            "schema": SCHEMA,
            "per_drone": per,
            "planner_calls": len(self.plans),
            "proposals": len(proposed),
            "plan_clear_min_m": _r(min((p["plan_clear_m"] for p in proposed if p.get("plan_clear_m") is not None), default=math.inf)),
            "plan_clear": _stats([p.get("plan_clear_m") for p in proposed]),
            "at_plan_track_err": _stats([p.get("track_err_m") for p in moving_plans]),
            "at_plan_track_lat_abs": _stats([abs(p["track_lat_m"]) for p in moving_plans if p.get("track_lat_m") is not None]),
            "at_plan_head_from_view_lat_abs": _stats([abs(p["head_from_view_lat_m"]) for p in moving_plans if p.get("head_from_view_lat_m") is not None]),
            "at_plan_head_from_view_along": _stats([p.get("head_from_view_along_m") for p in moving_plans]),
            "at_plan_frame_age_s": _stats([p.get("frame_age_s") for p in self.plans]),
            "at_plan_odom_age_s": _stats([p.get("odom_age_s") for p in self.plans]),
            "head_from_odom_count": getattr(self.fleet, "head_from_odom_count", None),
            "options_per_call": _stats([p.get("options") for p in self.plans if p.get("options") is not None]),
            "plans": self.plans,
            # 每拍逐行（四位小数），给事后估跟踪滞后用。
            "ticks": {
                did: [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()} for r in rows]
                for did, rows in self.ticks.items()
            },
        }


def parse_fix(raw: str | None) -> dict:
    """``YOPO_B2_FIX``：逗号分隔的 ``key=value``。空就是全关。

    * ``lead=<秒>``：设定点取已提交计划上提前这么久的那一点（见 LiveFleetConfig.setpoint_lead_s）。
    * ``odomhead=<米>``：交叉卡实际位置偏离计划头超过这么多时，从里程计外推的头状态起规划
      （CrossingFleet 的 policy_params.b2.head_from_odom_m）。
    * ``stiff=<kp>:<kv>``：设定点加代理侧误差增益（LiveFleetConfig.setpoint_gain_p / _v）。
    * ``launch=<a_max>:<秒>``（票 B3）：起步段计划加速度上限（policy_params.b3.launch_a_max_mps2 / launch_s）。
    * ``relat=<米>``（票 B3）：横向偏差超过这么多时从里程计起规划（policy_params.b3.replan_lateral_m）。
    * ``ramp=<m/s²>``（票 B4）：交叉卡虚拟中心起步按这个加速度斜坡（policy_params.b3.center_ramp_mps2）。
    * ``reframp=<m/s²>``（票 B5）：只把编队误差参考按这个加速度斜坡（policy_params.b3.ref_ramp_mps2）。
    """
    out: dict = {}
    for item in (raw or "").split(","):
        item = item.strip()
        if not item:
            continue
        key, _, value = item.partition("=")
        if key == "lead":
            lead = float(value)
            if not 0.0 < lead <= 0.5:
                raise ValueError(f"YOPO_B2_FIX lead 要在 (0, 0.5]，收到 {value!r}")
            out["setpoint_lead_s"] = lead
        elif key == "odomhead":
            thr = float(value)
            if not (math.isfinite(thr) and thr >= 0.0):
                raise ValueError(f"YOPO_B2_FIX odomhead 要非负，收到 {value!r}")
            out["head_from_odom_m"] = thr
        elif key == "stiff":
            kp, _, kv = value.partition(":")
            gp, gv = float(kp), float(kv or 0.0)
            if not (0.0 <= gp <= 5.0 and 0.0 <= gv <= 5.0) or gp + gv <= 0.0:
                raise ValueError(f"YOPO_B2_FIX stiff 要 kp:kv 在 [0, 5] 且不全为 0，收到 {value!r}")
            out["setpoint_gain_p"] = gp
            out["setpoint_gain_v"] = gv
        elif key == "launch":
            cap, _, span = value.partition(":")
            a_cap, t_span = float(cap), float(span or 2.5)
            if not (math.isfinite(a_cap) and a_cap > 0.0 and math.isfinite(t_span) and t_span > 0.0):
                raise ValueError(f"YOPO_B2_FIX launch 要 a_max:秒 且都为正，收到 {value!r}")
            out["launch_a_max_mps2"] = a_cap
            out["launch_s"] = t_span
        elif key == "relat":
            thr = float(value)
            if not (math.isfinite(thr) and thr > 0.0):
                raise ValueError(f"YOPO_B2_FIX relat 要为正，收到 {value!r}")
            out["replan_lateral_m"] = thr
        elif key == "ramp":
            ramp = float(value)
            if not (math.isfinite(ramp) and ramp > 0.0):
                raise ValueError(f"YOPO_B2_FIX ramp 要为正，收到 {value!r}")
            out["center_ramp_mps2"] = ramp
        elif key == "pillar":
            margin_s, has_floor, floor_s = value.partition(":")
            gap = float(margin_s)
            if not (math.isfinite(gap) and gap > 0.0):
                raise ValueError("YOPO_B2_FIX pillar 要有限且为正")
            out["pillar_clearance_m"] = gap
            if has_floor:
                floor = float(floor_s)
                if not (math.isfinite(floor) and floor > 0.0):
                    raise ValueError("YOPO_B2_FIX floor 要有限且为正")
                out["pillar_fallback_min_m"] = floor
        elif key == "floor":
            floor = float(value)
            if not (math.isfinite(floor) and floor > 0.0):
                raise ValueError("YOPO_B2_FIX floor 要有限且为正")
            out["pillar_fallback_min_m"] = floor
        elif key == "reframp":
            ref = float(value)
            if not (math.isfinite(ref) and ref > 0.0):
                raise ValueError(f"YOPO_B2_FIX reframp 要为正，收到 {value!r}")
            out["ref_ramp_mps2"] = ref
        else:
            raise ValueError(f"YOPO_B2_FIX 不认识 {key!r}")
    if "pillar_fallback_min_m" in out:
        if "pillar_clearance_m" not in out:
            raise ValueError("YOPO_B2_FIX floor 需要和 pillar 一起给")
        if out["pillar_fallback_min_m"] > out["pillar_clearance_m"] + 1e-12:
            raise ValueError("YOPO_B2_FIX floor 不能大于 pillar")
    if "ref_ramp_mps2" in out and "center_ramp_mps2" in out:
        raise ValueError("YOPO_B2_FIX reframp 与 ramp 只能开一个")
    return out


def fix_deviations(fix: dict) -> list[str]:
    out = []
    if "pillar_clearance_m" in fix:
        out.append(f"偏离上游：场景柱几何辅助的规划净空过滤 {fix['pillar_clearance_m']:g} m")
    if "pillar_fallback_min_m" in fix:
        out.append(f"偏离上游：柱净空全拒退路地板 {fix['pillar_fallback_min_m']:g} m")
    if "setpoint_lead_s" in fix:
        out.append(f"偏离上游：设定点提前 {fix['setpoint_lead_s']:g} s")
    if "setpoint_gain_p" in fix:
        out.append(f"偏离上游：设定点误差增益 kp={fix['setpoint_gain_p']:g} kv={fix['setpoint_gain_v']:g}")
    if "head_from_odom_m" in fix:
        out.append(f"偏离上游：偏差超过 {fix['head_from_odom_m']:g} m 时从里程计起规划")
    if "launch_a_max_mps2" in fix:
        out.append(f"偏离上游：起步 {fix['launch_s']:g} s 内计划加速度上限 {fix['launch_a_max_mps2']:g} m/s²")
    if "replan_lateral_m" in fix:
        out.append(f"偏离上游：横向偏差超过 {fix['replan_lateral_m']:g} m 时从里程计起规划")
    if "center_ramp_mps2" in fix:
        out.append(f"偏离上游：虚拟中心起步按 {fix['center_ramp_mps2']:g} m/s² 斜坡")
    if "ref_ramp_mps2" in fix:
        out.append(f"偏离上游：编队误差参考起步按 {fix['ref_ramp_mps2']:g} m/s² 斜坡（规划时刻表不变）")
    return out


B3_KEYS = (
    "launch_a_max_mps2",
    "launch_s",
    "replan_lateral_m",
    "center_ramp_mps2",
    "ref_ramp_mps2",
    "pillar_clearance_m",
    "pillar_fallback_min_m",
)


def b3_from_fix(fix: dict) -> dict | None:
    """parse_fix 结果里票 B3 的键。没有就是 None（全关）。"""
    out = {k: fix[k] for k in B3_KEYS if k in fix}
    return out or None


def apply_b2_overrides(
    card, *, safe_radius: float | None = None, head_from_odom_m: float | None = None, b3: dict | None = None
):
    """在内存里盖过场景卡的研究开关。都缺省时原样返回同一个对象，卡哈希不变。

    返回 (card, deviations)。deviations 非空就是「偏离上游」。
    """
    deviations: list[str] = []
    if safe_radius is None and head_from_odom_m is None and not b3:
        return card, deviations
    params = dict(card.card.get("policy_params") or {})
    if safe_radius is not None:
        radius = float(safe_radius)
        if not radius > 0.0 or not math.isfinite(radius):
            raise ValueError(f"--safe-radius 必须为正，收到 {safe_radius}")
        upstream = dict(params.get("upstream") or {})
        upstream["safe_radius"] = radius
        params["upstream"] = upstream
        deviations.append(f"偏离上游：safe_radius={radius:g}")
    if head_from_odom_m is not None:
        b2 = dict(params.get("b2") or {})
        b2["head_from_odom_m"] = float(head_from_odom_m)
        params["b2"] = b2
    if b3:
        params["b3"] = {**dict(params.get("b3") or {}), **{k: float(v) for k, v in b3.items()}}
    return dataclasses.replace(card, card={**card.card, "policy_params": params}), deviations


_STIFF_MAX_DP_M = 1.5
_STIFF_MAX_DV_MPS = 3.0


def _clip_norm(x: np.ndarray, limit: float) -> np.ndarray:
    n = float(np.linalg.norm(x))
    return x if n <= limit else x * (limit / n)


def stiffen(p: np.ndarray, v: np.ndarray, state: dict, gain_p: float, gain_v: float):
    """票 B2：两个增益都为 0 时原样返回同一对数组（默认路径不变）。"""
    if gain_p <= 0.0 and gain_v <= 0.0:
        return p, v
    odom_p = np.asarray(state["position_enu"], dtype=float)
    odom_v = np.asarray(state["velocity_enu"], dtype=float)
    p2 = p + _clip_norm(gain_p * (p - odom_p), _STIFF_MAX_DP_M) if gain_p > 0.0 else p
    v2 = v + _clip_norm(gain_v * (v - odom_v), _STIFF_MAX_DV_MPS) if gain_v > 0.0 else v
    return p2, v2
