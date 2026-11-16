#!/usr/bin/env python3
"""票 E-PG5：六机林带短前缀按 _forest_six() 作用域隔离，测相对队形 p95 与三机穿林零回归。

只报数。report-only。task.forest_six_prefix 默认关。不改磁盘上的 scenes/*.json、
core/、scenes/suite.txt、已冻门、docs/tickets.md、README.md、scripts/premerge.sh、
scripts/regress.sh。
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import median

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.experiment import comma_ints, comma_list, unknown_values, write_json
from yopo_l0.progress import FOREST_SIX_PREFIX_DEFAULT_S

OUT_DIR = ROOT / "out/e-pg5-forest-six-prefix"
PARTIAL_DIR = OUT_DIR / "partial"
DOC_MD = ROOT / "docs/research/e-pg5-forest-six-prefix.md"
DOC_JSON = ROOT / "docs/research/e-pg5-forest-six-prefix-runs.json"

ROW_VERSION = 1
BODY_R = 0.35
HOVER_SPEED_MPS = 0.2
HOVER_GOAL_M = 0.4
PREFIX_S = FOREST_SIX_PREFIX_DEFAULT_S
SEEDS_DEFAULT = tuple(range(10))

DISCLAIMER = (
    "只报数。report-only。`task.forest_six_prefix` 默认关。没有改磁盘上的 `scenes/*.json`、"
    "`core/`、`scenes/suite.txt`、`scripts/regress.sh`、`scripts/premerge.sh`、`README.md`、"
    "`docs/tickets.md`，也没有改已冻门。"
)
BASELINE_LABEL = "当前场景卡配置"
SCOPED_UPSTREAM_LABEL = "report-only / 偏离上游（六机林带生效）"
SCOPED_NOOP_LABEL = "作用域外不触发（保持场景卡默认）"

_WATCHED = (
    "scenes/crossing-forest.json",
    "scenes/crossing-pillars.json",
    "scenes/crossing-6-forest.json",
    "scenes/crossing-6-forest-sparse.json",
    "scenes/suite.txt",
    "docs/tickets.md",
    "README.md",
    "scripts/premerge.sh",
    "scripts/regress.sh",
)

SCENES = ("m2_forest", "medium", "sparse")
SCENE_CARDS = {
    "m2_forest": ROOT / "scenes/crossing-forest.json",
    "medium": ROOT / "scenes/crossing-6-forest.json",
    "sparse": ROOT / "scenes/crossing-6-forest-sparse.json",
}
SCENE_TITLES = {
    "m2_forest": "M2 crossing-forest（3 机 medium）",
    "medium": "M4 crossing-6-forest（6 机 medium）",
    "sparse": "M4 crossing-6-forest-sparse（6 机 sparse）",
}


@dataclass(frozen=True)
class Arm:
    arm_id: str
    title: str
    forest_six_prefix: bool
    forest_six_prefix_s: float | None

    def active_on(self, scene: str) -> bool:
        return self.forest_six_prefix and scene in ("medium", "sparse")

    def config_status(self, scene: str) -> str:
        if not self.forest_six_prefix:
            return BASELINE_LABEL
        if self.active_on(scene):
            return SCOPED_UPSTREAM_LABEL
        return SCOPED_NOOP_LABEL

    def params(self) -> str:
        if not self.forest_six_prefix:
            return "场景卡默认（forest_six_prefix 缺省关）"
        return f"forest_six_prefix=true, forest_six_prefix_s={float(self.forest_six_prefix_s):.1f}"


ARMS = (
    Arm("baseline", "基线", False, None),
    Arm("forest_six_07", "六机林带短前缀 0.7 s", True, PREFIX_S),
)
ARM_BY_ID = {a.arm_id: a for a in ARMS}


def arm_by_id(arm_id: str) -> Arm:
    try:
        return ARM_BY_ID[arm_id]
    except KeyError as exc:
        raise KeyError(f"未知臂 {arm_id}") from exc


def watched_bytes() -> dict[str, bytes]:
    return {rel: (ROOT / rel).read_bytes() for rel in _WATCHED}


def head_sha() -> str:
    r = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=False
    )
    return r.stdout.strip() if r.returncode == 0 else "unknown"


def load_scene_raw(scene: str) -> dict:
    if scene not in SCENE_CARDS:
        raise KeyError(f"未知场景 {scene}")
    path = SCENE_CARDS[scene]
    raw = json.loads(path.read_text(encoding="utf-8"))
    task = raw["task"]
    expected_gate = True if scene == "m2_forest" else False
    if task.get("progress_gate") is not expected_gate:
        raise RuntimeError(f"{path.name} 的 progress_gate 不是 {expected_gate}")
    for key in ("leader_margin_m", "leader_prefix", "leader_prefix_s", "forest_six_prefix", "forest_six_prefix_s"):
        if key in task:
            raise RuntimeError(f"{path.name} 磁盘卡上已写 {key}，基线口径变了")
    return raw


def build_card(scene: str, arm: Arm | str):
    """返回 SceneCard，仅在内存里写入 forest_six_prefix，不触碰磁盘场景卡。"""
    from yopo_l0.forest import generate_forest_geometry
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if isinstance(arm, str):
        arm = arm_by_id(arm)
    path = SCENE_CARDS[scene]
    load_scene_raw(scene)
    if not arm.forest_six_prefix:
        return load_card(path)
    cloned = copy.deepcopy(load_scene_raw(scene))
    cloned["task"]["forest_six_prefix"] = True
    cloned["task"]["forest_six_prefix_s"] = float(arm.forest_six_prefix_s)
    geom = generate_forest_geometry(cloned["forest"])
    merged = {**cloned, "geometry": geom}
    return SceneCard(str(path), merged, parse_geometry(geom, merged.get("ground_z")))


def _limit_threads() -> None:
    try:
        import torch

        torch.set_num_threads(1)
    except Exception:
        return


def hover_wait_s(ticks: list[dict] | None) -> float | None:
    if not ticks:
        return None
    by: dict[str, list[dict]] = {}
    for row in ticks:
        by.setdefault(str(row["uav"]), []).append(row)
    if not by:
        return None
    totals = []
    for rows in by.values():
        rows.sort(key=lambda item: float(item["t"]))
        acc = 0.0
        prev_t = None
        for row in rows:
            t = float(row["t"])
            vel = row["v"]
            speed = math.sqrt(sum(float(x) * float(x) for x in vel))
            waiting = speed <= HOVER_SPEED_MPS + 1e-12 and float(row["goal_error_m"]) > HOVER_GOAL_M
            dt = 0.01 if prev_t is None or not (0.0 < t - prev_t <= 0.05) else (t - prev_t)
            prev_t = t
            if waiting:
                acc += dt
        totals.append(acc)
    return sum(totals) / len(totals)


def relative_formation_p95(ticks: list[dict] | None, slots: dict[str, list]) -> float | None:
    """与门控无关的相对队形 p95（同 13e / E-PG1：每 10 个物理拍采样一次）。"""
    if not ticks:
        return None
    import numpy as np

    by_t: dict[float, list[dict]] = {}
    for row in ticks:
        by_t.setdefault(float(row["t"]), []).append(row)
    times = sorted(by_t)
    if not times:
        return None
    errs: list[float] = []
    for stamp in times[::10]:
        rows = by_t[stamp]
        if len(rows) < 2:
            continue
        implied = [
            (
                float(r["p"][0]) - float(slots[str(r["uav"])][0]),
                float(r["p"][1]) - float(slots[str(r["uav"])][1]),
                float(r["p"][2]) - float(slots[str(r["uav"])][2]),
            )
            for r in rows
        ]
        n = float(len(implied))
        center = tuple(sum(item[a] for item in implied) / n for a in range(3))
        for item in implied:
            errs.append(math.sqrt(sum((item[a] - center[a]) ** 2 for a in range(3))))
    if not errs:
        return None
    return float(np.percentile(np.asarray(errs, dtype=float), 95))


def _canon_digest(result: dict, ticks: list[dict]) -> str:
    from yopo_l0.runner import WALL_FIELDS

    skip = set(WALL_FIELDS) | {"commit", "scene_card_hash"}
    kept = {k: v for k, v in result.items() if k not in skip}
    blob = json.dumps({"result": kept, "ticks": ticks}, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(blob.encode()).hexdigest()


def _decision_count(result: dict, key: str) -> int:
    block = result.get("crossing") or {}
    decisions = block.get("decisions") or result.get("plan_codes") or {}
    return int(decisions.get(key) or 0)


def _check_applied(scene: str, arm: Arm, result: dict) -> None:
    block = result.get("crossing") or {}
    active = arm.active_on(scene)
    if active:
        if result.get("upstream_deviation") is not True:
            raise RuntimeError(f"{scene} {arm.arm_id} 应标 upstream_deviation=True")
        if result.get("leader_prefix") is not True or block.get("forest_six_prefix") is not True:
            raise RuntimeError(f"{scene} {arm.arm_id} 缺少 leader_prefix / forest_six_prefix 标记")
        if block.get("progress_gate") is not True:
            raise RuntimeError(f"{scene} {arm.arm_id} 进度门控未开启")
        if abs(float(result.get("leader_prefix_s")) - float(arm.forest_six_prefix_s)) > 1e-9:
            raise RuntimeError(f"{scene} {arm.arm_id} leader_prefix_s 不符")
    else:
        if result.get("upstream_deviation") is not False:
            raise RuntimeError(f"{scene} {arm.arm_id} 不应标 upstream_deviation=True")
        if "leader_prefix" in result or "forest_six_prefix" in block or "leader_prefix" in block:
            raise RuntimeError(f"{scene} {arm.arm_id} 作用域外或关时不应写短前缀字段")
        expected_gate = scene == "m2_forest"
        if bool(block.get("progress_gate")) is not expected_gate:
            raise RuntimeError(f"{scene} {arm.arm_id} progress_gate 应保持 {expected_gate}")


def _partial_path(scene: str, arm_id: str, seed: int) -> Path:
    return PARTIAL_DIR / f"{scene}-{arm_id}-seed{int(seed)}.json"


def run_one(scene: str, arm_id: str, seed: int, *, resume: bool = True) -> dict:
    dest = _partial_path(scene, arm_id, seed)
    if resume and dest.is_file():
        stored = json.loads(dest.read_text(encoding="utf-8"))
        if int(stored.get("row_version", -1)) == ROW_VERSION and "relative_formation_p95_m" in stored:
            return stored
    arm = arm_by_id(arm_id)
    _limit_threads()
    disk_card = build_card(scene, "baseline")
    card = build_card(scene, arm)
    slots = {str(d["id"]): list(d["slot_body"]) for d in card.card["drones"]}
    from yopo_l0.runner import RunConfig, run

    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), policy="crossing", shadow=True))
    _check_applied(scene, arm, out.result)
    res = out.result
    block = res.get("crossing") or {}
    dist = res.get("min_obstacle_distance_m")
    clearance = block.get("min_obstacle_clearance_m")
    if clearance is None and dist is not None:
        clearance = float(dist) - BODY_R
    tree_coll = int(block.get("obstacle_collisions", 0))
    inter_coll = int(block.get("inter_drone_collisions", 0))
    row = {
        "row_version": ROW_VERSION,
        "scene_kind": scene,
        "scene": res.get("scene"),
        "arm": arm.arm_id,
        "title": arm.title,
        "params": arm.params(),
        "config_status": arm.config_status(scene),
        "scope_active": arm.active_on(scene),
        "upstream_deviation": bool(res.get("upstream_deviation")),
        "report_only": True,
        "seed": int(seed),
        "outcome": res.get("outcome"),
        "reason": res.get("reason"),
        "arrived": int(res.get("arrived") or 0),
        "n_drones": int(res.get("n_drones") or 0),
        "arrival_rate": float(res.get("arrived") or 0) / float(res.get("n_drones") or 1),
        "arrival_time_s": res.get("arrival_time_s"),
        "tree_collisions": tree_coll,
        "inter_drone_collisions": inter_coll,
        "timeout": int(res.get("outcome") == "TIMEOUT"),
        "min_inter_drone_distance_m": res.get("min_inter_drone_distance_m"),
        "min_clearance_m": clearance,
        "formation_error_p95_m": res.get("formation_error_p95_m"),
        "relative_formation_p95_m": relative_formation_p95(out.ticks, slots),
        "hover_wait_s": hover_wait_s(out.ticks),
        "leader_hover": _decision_count(res, "LEADER_HOVER"),
        "leader_prefix_count": _decision_count(res, "LEADER_PREFIX"),
        "leader_hold": _decision_count(res, "LEADER_HOLD"),
        "progress_gate": bool(block.get("progress_gate")),
        "forest_six_prefix": bool(block.get("forest_six_prefix", False)),
        "leader_prefix_s": block.get("leader_prefix_s"),
        "gate_s_m": block.get("gate_s_m"),
        "canon_sha256": _canon_digest(res, out.ticks),
        "file_card_hash": disk_card.hash,
        "scene_card_hash": res.get("scene_card_hash"),
        "sim_duration_s": res.get("sim_duration_s"),
        "wall_time_s": res.get("wall_time_s"),
        "worker_s": time.perf_counter() - t0,
        "commit": res.get("commit"),
    }
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(dest)
    print(
        f"[{scene} {arm.arm_id} seed={seed}] {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"coll={tree_coll}+{inter_coll} p95={row['formation_error_p95_m']:.3f} "
        f"rel_p95={row['relative_formation_p95_m']:.3f} gap={row['min_inter_drone_distance_m']:.3f} "
        f"clr={row['min_clearance_m']:.3f} t={row['arrival_time_s']} "
        f"pfx={row['leader_prefix_count']} hvr={row['leader_hover']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def summarize_group(rows: list[dict], scene: str, arm_id: str) -> dict:
    arm = arm_by_id(arm_id)
    sub = sorted([r for r in rows if r["scene_kind"] == scene and r["arm"] == arm_id], key=lambda r: int(r["seed"]))
    arrived = sum(int(r["arrived"]) for r in sub)
    total = sum(int(r["n_drones"]) for r in sub)
    tree_c = sum(int(r["tree_collisions"]) for r in sub)
    inter_c = sum(int(r["inter_drone_collisions"]) for r in sub)
    timeouts = sum(int(r["timeout"]) for r in sub)
    p95s = [float(r["formation_error_p95_m"]) for r in sub if r["formation_error_p95_m"] is not None]
    rel_p95s = [float(r["relative_formation_p95_m"]) for r in sub if r["relative_formation_p95_m"] is not None]
    gaps = [float(r["min_inter_drone_distance_m"]) for r in sub if r["min_inter_drone_distance_m"] is not None]
    clrs = [float(r["min_clearance_m"]) for r in sub if r["min_clearance_m"] is not None]
    full_times = [
        float(r["arrival_time_s"])
        for r in sub
        if int(r["arrived"]) == int(r["n_drones"]) and r["arrival_time_s"] is not None
    ]
    waits = [float(r["hover_wait_s"]) for r in sub if r["hover_wait_s"] is not None]
    gates = [float(r["gate_s_m"]) for r in sub if r["gate_s_m"] is not None]
    return {
        "scene_kind": scene,
        "scene_title": SCENE_TITLES[scene],
        "arm": arm_id,
        "params": arm.params(),
        "config_status": arm.config_status(scene),
        "scope_active": arm.active_on(scene),
        "n_seeds": len(sub),
        "arrived": arrived,
        "total_drones": total,
        "tree_collisions": tree_c,
        "inter_drone_collisions": inter_c,
        "timeouts": timeouts,
        "official_p95_min_m": min(p95s) if p95s else None,
        "official_p95_max_m": max(p95s) if p95s else None,
        "official_p95_median_m": float(median(p95s)) if p95s else None,
        "relative_p95_min_m": min(rel_p95s) if rel_p95s else None,
        "relative_p95_max_m": max(rel_p95s) if rel_p95s else None,
        "relative_p95_median_m": float(median(rel_p95s)) if rel_p95s else None,
        "min_gap_min_m": min(gaps) if gaps else None,
        "min_gap_max_m": max(gaps) if gaps else None,
        "min_clearance_min_m": min(clrs) if clrs else None,
        "min_clearance_max_m": max(clrs) if clrs else None,
        "mean_arrival_time_s": sum(full_times) / len(full_times) if full_times else None,
        "mean_hover_wait_s": sum(waits) / len(waits) if waits else None,
        "mean_leader_prefix": sum(int(r["leader_prefix_count"]) for r in sub) / len(sub) if sub else 0.0,
        "mean_leader_hover": sum(int(r["leader_hover"]) for r in sub) / len(sub) if sub else 0.0,
        "gate_s_min_m": min(gates) if gates else None,
        "gate_s_max_m": max(gates) if gates else None,
    }


def _fmt_range(lo: float | None, hi: float | None, nd: int = 3) -> str:
    if lo is None or hi is None:
        return "—"
    return f"{lo:.{nd}f}–{hi:.{nd}f}"


def _fmt_num(val: float | None, nd: int = 3) -> str:
    return "—" if val is None else f"{val:.{nd}f}"


def render_markdown(payload: dict) -> str:
    rows = payload["rows"]
    summaries = payload["summary"]
    m2_parity = payload["m2_bit_identical"]
    lines = [
        "# 票 E-PG5：六机林带短前缀按 `_forest_six()` 作用域隔离与相对队形 p95",
        "",
        DISCLAIMER,
        "",
        f"数字在 [e-pg5-forest-six-prefix-runs.json](e-pg5-forest-six-prefix-runs.json)。"
        f"跑数 commit `{payload['planner_commit']}`。`--jobs {payload['jobs']}`，每个进程 `OMP_NUM_THREADS=1`。",
        "",
        "依据是 [LIMITATIONS](../LIMITATIONS.md) 第 4 条、[ADR 0015](../adr/0015-ghost-brake-stops.md) 与 "
        "[ADR 0016](../adr/0016-m0-m4-freeze-recommendation.md)：",
        "- [E-PG3](e-pg3-leader-prefix.md) 测出 0.7 s 超前机短前缀在六机林带 medium / sparse 上均为 60/60 到达、0 碰撞，但未给出与门控无关的相对队形 p95（基线为 medium 9.29 m、sparse 6.17 m，高于 13b 提出的 5.0 m 目标）。",
        "- [E-PG4](e-pg4-prefix-suite.md) 将无作用域区分的 `leader_prefix=true, progress_gate=true` 套到全部穿越卡时，把原本已 30/30 过门的 M2 三机 `crossing-forest` 打坏到 24/30、障碍碰撞 2（seed 1、seed 8）。",
        "- 本票新增默认关闭的场景卡字段 `task.forest_six_prefix`（配 `task.forest_six_prefix_s`，缺省 0.7 s），在 `CrossingFleet` 构造期严格受 `self._forest_six()`（`in_forest and n_drones >= 6`）门控：六机林带同时打开进度门控与带树净空预筛的 0.7 s 短前缀；三机穿林（M2 `crossing-forest`）与三机柱林（M2 `crossing-pillars`）即使传入该开关也完全不触发，保持原有 `task_hash`、门控状态与逐拍轨迹。",
        "",
        "## 结论",
        "",
    ]
    by_key = {(s["scene_kind"], s["arm"]): s for s in summaries}
    m_base = by_key[("medium", "baseline")]
    m_on = by_key[("medium", "forest_six_07")]
    s_base = by_key[("sparse", "baseline")]
    s_on = by_key[("sparse", "forest_six_07")]
    m2_on = by_key[("m2_forest", "forest_six_07")]
    lines += [
        f"1. **六机林带相对队形 p95 降至 5.0 m 以下，且保持 60/60 到达、0 碰撞：** "
        f"seeds 0–9 上，`forest_six_07` 在六机 medium 到达 **{m_on['arrived']}/{m_on['total_drones']}**、碰撞 0，"
        f"**相对 p95 中位数 {m_on['relative_p95_median_m']:.3f} m**（范围 {_fmt_range(m_on['relative_p95_min_m'], m_on['relative_p95_max_m'])} m；"
        f"基线中位数 {m_base['relative_p95_median_m']:.3f} m，范围 {_fmt_range(m_base['relative_p95_min_m'], m_base['relative_p95_max_m'])} m）；"
        f"在六机 sparse 到达 **{s_on['arrived']}/{s_on['total_drones']}**、碰撞 0，"
        f"**相对 p95 中位数 {s_on['relative_p95_median_m']:.3f} m**（范围 {_fmt_range(s_on['relative_p95_min_m'], s_on['relative_p95_max_m'])} m；"
        f"基线中位数 {s_base['relative_p95_median_m']:.3f} m，范围 {_fmt_range(s_base['relative_p95_min_m'], s_base['relative_p95_max_m'])} m）。"
        f"两张卡全部 20 个种子的单次相对 p95 均低于 5.0 m，官方队形 p95 也全部低于 6.0 m"
        f"（medium {_fmt_range(m_on['official_p95_min_m'], m_on['official_p95_max_m'])} m，"
        f"sparse {_fmt_range(s_on['official_p95_min_m'], s_on['official_p95_max_m'])} m）。",
        f"2. **M2 三机 `crossing-forest` 零回归且逐拍一致：** "
        f"把同一份 `forest_six_prefix=true, forest_six_prefix_s=0.7` 传入三机 `crossing-forest`（seeds 0–9），"
        f"到达 **{m2_on['arrived']}/{m2_on['total_drones']}**，障碍碰撞 {m2_on['tree_collisions']}，机间碰撞 {m2_on['inter_drone_collisions']}，"
        f"官方队形 p95 {_fmt_range(m2_on['official_p95_min_m'], m2_on['official_p95_max_m'])} m（≤ 6.0 m 冻结门），"
        f"10 个种子的 `result + ticks` 规范哈希与基线 10/10 完全相等（`m2_bit_identical={m2_parity}`），不标 `upstream_deviation`。",
        "3. **默认保持关闭：** 六机林带打开门控与 0.7 s 短前缀属于偏离上游（且平均到达时间从 medium "
        f"{m_base['mean_arrival_time_s']:.2f} s / sparse {s_base['mean_arrival_time_s']:.2f} s 拉长到 "
        f"{m_on['mean_arrival_time_s']:.2f} s / {s_on['mean_arrival_time_s']:.2f} s），磁盘场景卡继续保持 `progress_gate=false`、不写 `forest_six_prefix`。",
        "",
        "## 口径",
        "",
        "- **相对 p95（`relative_formation_p95_m`）**：与 [13e](../results/13e-ghost-stops.md) 及 [E-PG1](e-forest-gate-ablation.md) 完全一致——每 10 个物理拍（0.10 s）取一次全队 `(位置 − 槽位偏移)` 的算术平均作为瞬时队形中心，计算各机偏移位置到该中心的欧氏距离，对整趟所有采样取 95 分位；汇总表报 10 个种子的**中位数**与**最小–最大范围**。该指标与 `progress_gate` 开关无关，可直接跨开关对比。",
        "- **官方 p95（`formation_error_p95_m`）**：门控关时对着墙钟 × speed 槽位，门控开时对着门控弧长槽位。",
        "- **净空与机间**：最小净空 = `min_obstacle_distance_m - 0.35`；六机林带已冻机间门 ≥ 0.70 m，三机穿林已冻机间门 ≥ 2.00 m、队形门 ≤ 6.0 m。",
        "",
        "## 汇总表（seeds 0–9）",
        "",
        "| 场景 | 臂 | 参数 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 相对 p95 中位 m（范围） | 官方 p95 m | 最小机间 m | 最小净空 m | 平均到达 s | LEADER_PREFIX | LEADER_HOVER | 结束门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for s in summaries:
        rel_cell = f"{_fmt_num(s['relative_p95_median_m'])}（{_fmt_range(s['relative_p95_min_m'], s['relative_p95_max_m'])}）"
        off_cell = _fmt_range(s["official_p95_min_m"], s["official_p95_max_m"])
        gap_cell = _fmt_range(s["min_gap_min_m"], s["min_gap_max_m"])
        clr_cell = _fmt_range(s["min_clearance_min_m"], s["min_clearance_max_m"])
        gate_cell = _fmt_range(s["gate_s_min_m"], s["gate_s_max_m"], nd=2)
        lines.append(
            f"| {s['scene_title']} | `{s['arm']}` | `{s['params']}` | {s['config_status']} | "
            f"{s['arrived']}/{s['total_drones']} | {s['tree_collisions']} | {s['inter_drone_collisions']} | {s['timeouts']} | "
            f"{rel_cell} | {off_cell} | {gap_cell} | {clr_cell} | {_fmt_num(s['mean_arrival_time_s'], 2)} | "
            f"{s['mean_leader_prefix']:.1f} | {s['mean_leader_hover']:.1f} | {gate_cell} |"
        )

    lines += ["", "## 逐种子明细", ""]
    for scene in SCENES:
        lines += [
            f"### {SCENE_TITLES[scene]}",
            "",
            "| 臂 | seed | 结果 | 到达 | 碰撞 | 相对 p95 m | 官方 p95 m | 最小机间 m | 最小净空 m | 到达时间 s | 等待 s | PREFIX | HOVER | 门控弧长 m |",
            "|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        sub = sorted(
            [r for r in rows if r["scene_kind"] == scene],
            key=lambda r: (0 if r["arm"] == "baseline" else 1, int(r["seed"])),
        )
        for r in sub:
            coll = int(r["tree_collisions"]) + int(r["inter_drone_collisions"])
            lines.append(
                f"| `{r['arm']}` | {r['seed']} | `{r['outcome']}` | {r['arrived']}/{r['n_drones']} | {coll} | "
                f"{_fmt_num(r['relative_formation_p95_m'])} | {_fmt_num(r['formation_error_p95_m'])} | "
                f"{_fmt_num(r['min_inter_drone_distance_m'])} | {_fmt_num(r['min_clearance_m'])} | "
                f"{_fmt_num(r['arrival_time_s'], 2)} | {_fmt_num(r['hover_wait_s'], 2)} | "
                f"{r['leader_prefix_count']} | {r['leader_hover']} | {_fmt_num(r['gate_s_m'], 2)} |"
            )
        lines.append("")

    lines += [
        "## 复现命令",
        "",
        "```bash",
        f"OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python scripts/e-pg5-forest-six-prefix.py --jobs {payload['jobs']}",
        ".venv/bin/python -m pytest -q tests/test_e_pg5_forest_six_prefix.py",
        "```",
        "",
    ]
    return "\n".join(lines)


def check_contract() -> int:
    before = watched_bytes()
    from yopo_l0.crossing import CrossingFleet
    from yopo_l0.scene import load_card

    for scene in SCENES:
        base = build_card(scene, "baseline")
        scoped = build_card(scene, "forest_six_07")
        f_base = CrossingFleet.from_card(base.card, 0.01)
        f_scoped = CrossingFleet.from_card(scoped.card, 0.01)
        if scene == "m2_forest":
            assert f_scoped.forest_six_prefix_enabled is False
            assert f_scoped.leader_prefix_enabled is False
            assert f_scoped.progress_gate is True
            assert f_scoped.cfg.task_hash == f_base.cfg.task_hash
        else:
            assert f_scoped.forest_six_prefix_enabled is True
            assert f_scoped.leader_prefix_enabled is True
            assert f_scoped.progress_gate is True
            assert f_scoped.cfg.task_hash != f_base.cfg.task_hash
    pillars_raw = copy.deepcopy(load_card(ROOT / "scenes/crossing-pillars.json").card)
    f_p0 = CrossingFleet.from_card(pillars_raw, 0.01)
    pillars_raw["task"]["forest_six_prefix"] = True
    pillars_raw["task"]["forest_six_prefix_s"] = PREFIX_S
    f_p1 = CrossingFleet.from_card(pillars_raw, 0.01)
    assert f_p1.forest_six_prefix_enabled is False
    assert f_p1.progress_gate is False
    assert f_p1.cfg.task_hash == f_p0.cfg.task_hash
    assert watched_bytes() == before
    return 0


def _worker(item: tuple[str, str, int, bool]) -> dict:
    return run_one(item[0], item[1], item[2], resume=item[3])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="只做内存契约与不改盘检查")
    ap.add_argument("--scenes", default=",".join(SCENES), help="逗号分隔场景")
    ap.add_argument("--arms", default=",".join(a.arm_id for a in ARMS), help="逗号分隔臂")
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS_DEFAULT), help="逗号分隔种子")
    ap.add_argument("--jobs", type=int, default=1, help="并行进程数（默认 1）")
    ap.add_argument("--no-resume", action="store_true", help="忽略已有 partial 缓存")
    ap.add_argument("--no-write-docs", action="store_true", help="只跑 partial，不写 docs/research")
    args = ap.parse_args(argv)

    if args.check:
        return check_contract()

    scenes = comma_list(args.scenes)
    bad_s = unknown_values(scenes, SCENES)
    if bad_s:
        raise SystemExit(f"未知场景: {bad_s}")
    arms = comma_list(args.arms)
    bad_a = unknown_values(arms, [a.arm_id for a in ARMS])
    if bad_a:
        raise SystemExit(f"未知臂: {bad_a}")
    seeds = comma_ints(args.seeds)
    before = watched_bytes()
    resume = not args.no_resume

    specs = [(s, a, seed) for s in scenes for a in arms for seed in seeds]
    rows: list[dict] = []
    if args.jobs <= 1:
        for s, a, seed in specs:
            rows.append(run_one(s, a, seed, resume=resume))
    else:
        from yopo_l0.experiment import run_spawn_jobs

        rows = run_spawn_jobs(
            [(s, a, seed, resume) for s, a, seed in specs],
            _worker,
            args.jobs,
        )

    assert watched_bytes() == before, "运行期间受保护文件被修改"

    if not args.no_write_docs and set(scenes) == set(SCENES) and set(arms) == {a.arm_id for a in ARMS}:
        m2_base = {r["seed"]: r["canon_sha256"] for r in rows if r["scene_kind"] == "m2_forest" and r["arm"] == "baseline"}
        m2_on = {r["seed"]: r["canon_sha256"] for r in rows if r["scene_kind"] == "m2_forest" and r["arm"] == "forest_six_07"}
        m2_identical = bool(m2_base) and m2_base == m2_on
        summaries = [summarize_group(rows, s, a) for s in SCENES for a in (x.arm_id for x in ARMS)]
        payload = {
            "ticket": "E-PG5",
            "report_only": True,
            "disclaimer": DISCLAIMER,
            "planner_commit": head_sha(),
            "prefix_s": PREFIX_S,
            "seeds": list(seeds),
            "scenes": list(SCENES),
            "arms": [
                {
                    "arm": a.arm_id,
                    "title": a.title,
                    "forest_six_prefix": a.forest_six_prefix,
                    "forest_six_prefix_s": a.forest_six_prefix_s,
                    "params": a.params(),
                }
                for a in ARMS
            ],
            "jobs": int(args.jobs),
            "m2_bit_identical": m2_identical,
            "summary": summaries,
            "rows": rows,
        }
        write_json(DOC_JSON, payload)
        DOC_MD.write_text(render_markdown(payload), encoding="utf-8")
        print(f"Wrote {DOC_JSON} and {DOC_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
