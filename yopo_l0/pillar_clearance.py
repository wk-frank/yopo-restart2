"""B6/B7/B8（默认关）：用场景柱几何过滤拟合后的候选，属于几何真值辅助实验。

B7：一批候选全被柱门拒绝时，退回离柱面最远的那个，不再把空计划交出去。
B8：可选 ``pillar_fallback_min_m`` 地板（默认关）；最远候选仍低于地板时不交退路，改走刹车 / 保持。
"""

import math

import numpy as np


class PillarClearance:
    def __init__(
        self,
        geometry: dict,
        margin_m: float,
        body_radius_m: float,
        fallback_min_m: float | None = None,
    ):
        pillars = [b for b in geometry.get("boxes", []) if str(b.get("name", "")).startswith("p_")]
        if not pillars:
            raise ValueError("pillar_clearance_m 只用于含 p_ 柱子的 crossing 几何")
        if isinstance(margin_m, bool) or not (math.isfinite(float(margin_m)) and float(margin_m) > 0.0):
            raise ValueError("pillar_clearance_m 要有限且为正")
        margin = float(margin_m)
        floor = None
        if fallback_min_m is not None:
            if isinstance(fallback_min_m, bool) or not (
                math.isfinite(float(fallback_min_m)) and float(fallback_min_m) > 0.0
            ):
                raise ValueError("pillar_fallback_min_m 要有限且为正")
            floor = float(fallback_min_m)
            if floor > margin + 1e-12:
                raise ValueError("pillar_fallback_min_m 不能大于 pillar_clearance_m")
        self.lo = np.asarray([b["lo"] for b in pillars], dtype=float)
        self.hi = np.asarray([b["hi"] for b in pillars], dtype=float)
        self.margin_m = margin
        self.body_radius_m = body_radius_m
        self.fallback_min_m = floor
        self.checks = 0
        self.rejected = 0
        self.fallbacks = 0
        self.floor_blocked = 0

    def gap(self, motion, speed_limit: float) -> float:
        # 距离函数是 1-Lipschitz；用轨迹限速 × 半个采样间隔覆盖采样点之间的运动。
        # 检查整条拟合计划（含停止尾），不把未拟合 raw 的远端混进来。
        span = float(motion.t_end - motion.t_start)
        n = max(2, int(math.ceil(span / 0.02)) + 1)
        times = np.linspace(float(motion.t_start), float(motion.t_end), n)
        points = np.asarray([motion.evaluate(float(t)) for t in times])
        delta = np.maximum(np.maximum(self.lo[None] - points[:, None], 0.0), points[:, None] - self.hi[None])
        raw = float(np.linalg.norm(delta, axis=-1).min()) - self.body_radius_m
        return raw - float(speed_limit) * span / (2 * (n - 1))

    def allows(self, motion, speed_limit: float) -> bool:
        lower = self.gap(motion, speed_limit)
        ok = math.isfinite(lower) and lower + 1e-9 >= self.margin_m
        self.checks += 1
        self.rejected += int(not ok)
        return ok

    def farthest(self, motions, speed_limit: float):
        """离柱面最远的候选。并列时取原排序更靠前的。"""
        if not motions:
            raise ValueError("没有候选可退回")
        return max(motions, key=lambda m: self.gap(m, speed_limit))

    def fallback(self, motions, speed_limit: float):
        """全拒时的退路候选。地板打开且最远候选仍低于地板时返回 None（交由上层刹车 / 保持）。"""
        best = self.farthest(motions, speed_limit)
        if self.fallback_min_m is not None:
            lower = self.gap(best, speed_limit)
            if not (math.isfinite(lower) and lower + 1e-9 >= self.fallback_min_m):
                self.floor_blocked += 1
                return None
        return best

    def summary(self) -> dict:
        out = {
            "margin_m": self.margin_m,
            "checks": self.checks,
            "rejected": self.rejected,
            "fallbacks": self.fallbacks,
            "source": "scene p_ boxes (oracle)",
            "deviation": "偏离上游：场景柱几何辅助的规划净空过滤；全拒时退回离柱最远候选",
        }
        if self.fallback_min_m is not None:
            out["fallback_min_m"] = self.fallback_min_m
            out["floor_blocked"] = self.floor_blocked
            out["deviation"] = (
                f"偏离上游：场景柱几何辅助的规划净空过滤；全拒时退回离柱最远候选（地板 {self.fallback_min_m:g} m）"
            )
        return out
