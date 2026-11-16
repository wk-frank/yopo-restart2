"""票 13b：场景卡上的进度门控。

林带默认开，开阔区必须关。分位 0 是全队最小值。

票 E-PG3：task.leader_prefix 默认关。关着时超前机仍先悬停，再退到 0.3 s 短前缀。
打开后超前机只飞 task.leader_prefix_s 那么长的一段，接不上再等待或续接。打开即偏离上游。

票 E-PG5：task.forest_six_prefix 默认关。打开后仅对带森林的六机（_forest_six()）
同时开启进度门控与短前缀（缺省 0.7 s），三机穿林与柱林保持原样。打开且生效即偏离上游。
"""

from __future__ import annotations

import math

# 短于这一截时 _fit_horizon 直接放弃。长于 2 s 就盖过六机一轮窗口，不再是短前缀。
LEADER_PREFIX_MIN_S = 0.2
LEADER_PREFIX_MAX_S = 2.0
FOREST_SIX_PREFIX_DEFAULT_S = 0.7


def read_progress_settings(task: dict, *, default_gate: bool) -> tuple[bool, float]:
    """读 progress_gate / progress_percentile。缺省门控由调用方决定，分位默认 0（min）。"""
    if "progress_gate" in task:
        gate = task["progress_gate"]
        if not isinstance(gate, bool):
            raise ValueError("progress_gate 必须是布尔")
    else:
        gate = bool(default_gate)
    raw = task.get("progress_percentile", 0)
    try:
        pct = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"progress_percentile 必须是数字，收到 {raw!r}") from exc
    if not math.isfinite(pct) or pct < 0.0 or pct > 100.0:
        raise ValueError("progress_percentile 必须在 [0, 100]")
    return gate, pct


def read_leader_margin_m(task: dict, *, default: float = 1.0) -> float:
    """读 task.leader_margin_m。缺这个字段时用 default，穿林缺省 1.0。

    与 read_progress_settings 一起在 CrossingFleet 构造时读。缺省值与改前的
    模块常量相同，不改已有场景卡的飞行结果。
    """
    if "leader_margin_m" not in task:
        margin = float(default)
    else:
        raw = task["leader_margin_m"]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"leader_margin_m 必须是数字，收到 {raw!r}")
        margin = float(raw)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError(f"leader_margin_m 必须是非负有限数，收到 {margin!r}")
    return margin


def _parse_prefix_length_s(raw: object, *, field: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError(f"{field} 必须是数字，收到 {raw!r}")
    length = float(raw)
    if (
        not math.isfinite(length)
        or length < LEADER_PREFIX_MIN_S
        or length > LEADER_PREFIX_MAX_S
    ):
        raise ValueError(
            f"{field} 必须在 [{LEADER_PREFIX_MIN_S:g}, {LEADER_PREFIX_MAX_S:g}] 秒，收到 {raw!r}"
        )
    return length


def _parse_leader_prefix_s(raw: object) -> float:
    return _parse_prefix_length_s(raw, field="leader_prefix_s")


def read_leader_prefix(task: dict, *, default_s: float = 0.3) -> tuple[bool, float]:
    """读 task.leader_prefix / task.leader_prefix_s。

    开关缺省 false。关时忽略卡上的长度，返回 default_s，飞行与改前相同。
    开时长度缺省 default_s（穿林是 0.3 s），必须落在 [0.2, 2.0]。打开即偏离上游。
    长度写了但开关关着，仍检查这个数字，非法就报错，合法则不用。
    """
    if "leader_prefix" in task:
        enabled = task["leader_prefix"]
        if enabled is not True and enabled is not False:
            raise ValueError(f"leader_prefix 必须是布尔，收到 {enabled!r}")
    else:
        enabled = False
    if "leader_prefix_s" in task:
        parsed = _parse_leader_prefix_s(task["leader_prefix_s"])
    else:
        parsed = None
    if not enabled:
        return False, float(default_s)
    return True, float(default_s if parsed is None else parsed)


def read_forest_six_prefix(
    task: dict, *, default_s: float = FOREST_SIX_PREFIX_DEFAULT_S
) -> tuple[bool, float]:
    """读 task.forest_six_prefix / task.forest_six_prefix_s。

    开关缺省 false。关时忽略卡上的长度，返回 default_s。
    开时长度缺省 default_s（0.7 s），必须落在 [0.2, 2.0]。
    是否实际生效由 CrossingFleet._forest_six() 决定：三机穿林与柱林不触发。
    """
    if "forest_six_prefix" in task:
        enabled = task["forest_six_prefix"]
        if enabled is not True and enabled is not False:
            raise ValueError(f"forest_six_prefix 必须是布尔，收到 {enabled!r}")
    else:
        enabled = False
    if "forest_six_prefix_s" in task:
        parsed = _parse_prefix_length_s(task["forest_six_prefix_s"], field="forest_six_prefix_s")
    else:
        parsed = None
    if not enabled:
        return False, float(default_s)
    return True, float(default_s if parsed is None else parsed)
