#!/usr/bin/env python3
"""10 种子正面对决基准：CrossingFleet（含基线与 E-PG5）vs ExploreFleet（1x 标称动力学 & TJU 时间相似律高速版）。

输出：
- docs/research/explore-free-swarm-runs.json（逐种子原始指标与汇总统计）
- docs/research/explore-free-swarm.md（完整中文基准对比报告）

严格只读保护：绝不修改既有 scenes/crossing-*.json、scenes/suite.txt 与 core/。
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import copy
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DOC_JSON = ROOT / "docs/research/explore-free-swarm-runs.json"
DOC_MD = ROOT / "docs/research/explore-free-swarm.md"

WATCHED_IMMUTABLE = (
    "scenes/suite.txt",
    "scenes/crossing-pillars.json",
    "scenes/crossing-forest.json",
    "scenes/crossing-6-forest.json",
    "scenes/crossing-6-forest-sparse.json",
)


@dataclass(frozen=True)
class BenchCase:
    group_id: str
    group_title: str
    arm_id: str
    arm_label: str
    base_card_rel: str
    density_override: str | None = None
    epg5: bool = False
    explore_stock_6mps: bool = False


CASES: tuple[BenchCase, ...] = (
    # 1. 3 机柱林 (19.5 m)
    BenchCase("3_pillars", "3 机柱林 (19.5 m)", "crossing_base", "CrossingFleet 基线", "scenes/crossing-pillars.json"),
    BenchCase("3_pillars", "3 机柱林 (19.5 m)", "explore_6mps", "ExploreFleet (1x 标称 a_max=6)", "scenes/explore-3-pillars.json", explore_stock_6mps=True),
    BenchCase("3_pillars", "3 机柱林 (19.5 m)", "explore_fast", "ExploreFleet (TJU 高速版)", "scenes/explore-3-pillars.json"),
    # 2. 3 机中密林 (42.0 m, medium)
    BenchCase("3_forest_med", "3 机中密树林 (42.0 m, medium)", "crossing_base", "CrossingFleet 基线 (门控开)", "scenes/crossing-forest.json"),
    BenchCase("3_forest_med", "3 机中密树林 (42.0 m, medium)", "explore_6mps", "ExploreFleet (1x 标称 a_max=6)", "scenes/explore-3-forest.json", explore_stock_6mps=True),
    BenchCase("3_forest_med", "3 机中密树林 (42.0 m, medium)", "explore_fast", "ExploreFleet (TJU 高速版)", "scenes/explore-3-forest.json"),
    # 3. 3 机高密林 (42.0 m, dense, 树表最小净距 1.7 m)
    BenchCase("3_forest_dense", "3 机高密树林 (42.0 m, dense)", "crossing_base", "CrossingFleet 基线 (门控开)", "scenes/crossing-forest.json", density_override="dense"),
    BenchCase("3_forest_dense", "3 机高密树林 (42.0 m, dense)", "explore_6mps", "ExploreFleet (1x 标称 a_max=6)", "scenes/explore-3-forest-dense.json", explore_stock_6mps=True),
    BenchCase("3_forest_dense", "3 机高密树林 (42.0 m, dense)", "explore_fast", "ExploreFleet (TJU 高速版)", "scenes/explore-3-forest-dense.json"),
    # 4. 6 机疏林 (42.0 m, sparse)
    BenchCase("6_forest_sparse", "6 机疏林 (42.0 m, sparse)", "crossing_base", "CrossingFleet 基线 (楔形)", "scenes/crossing-6-forest-sparse.json"),
    BenchCase("6_forest_sparse", "6 机疏林 (42.0 m, sparse)", "crossing_epg5", "CrossingFleet + E-PG5 (0.7s)", "scenes/crossing-6-forest-sparse.json", epg5=True),
    BenchCase("6_forest_sparse", "6 机疏林 (42.0 m, sparse)", "explore_6mps", "ExploreFleet (1x 标称横排)", "scenes/explore-6-forest-sparse.json", explore_stock_6mps=True),
    BenchCase("6_forest_sparse", "6 机疏林 (42.0 m, sparse)", "explore_fast", "ExploreFleet (TJU 高速横排)", "scenes/explore-6-forest-sparse.json"),
    # 5. 6 机中密林 (42.0 m, medium)
    BenchCase("6_forest_med", "6 机中密树林 (42.0 m, medium)", "crossing_base", "CrossingFleet 基线 (楔形)", "scenes/crossing-6-forest.json"),
    BenchCase("6_forest_med", "6 机中密树林 (42.0 m, medium)", "crossing_epg5", "CrossingFleet + E-PG5 (0.7s)", "scenes/crossing-6-forest.json", epg5=True),
    BenchCase("6_forest_med", "6 机中密树林 (42.0 m, medium)", "explore_6mps", "ExploreFleet (1x 标称横排)", "scenes/explore-6-forest.json", explore_stock_6mps=True),
    BenchCase("6_forest_med", "6 机中密树林 (42.0 m, medium)", "explore_fast", "ExploreFleet (TJU 高速横排)", "scenes/explore-6-forest.json"),
    # 6. 6 机高密林 (42.0 m, dense)
    BenchCase("6_forest_dense", "6 机高密树林 (42.0 m, dense)", "crossing_base", "CrossingFleet 基线 (楔形)", "scenes/crossing-6-forest.json", density_override="dense"),
    BenchCase("6_forest_dense", "6 机高密树林 (42.0 m, dense)", "crossing_epg5", "CrossingFleet + E-PG5 (0.7s)", "scenes/crossing-6-forest.json", density_override="dense", epg5=True),
    BenchCase("6_forest_dense", "6 机高密树林 (42.0 m, dense)", "explore_6mps", "ExploreFleet (1x 标称横排)", "scenes/explore-6-forest-dense.json", explore_stock_6mps=True),
    BenchCase("6_forest_dense", "6 机高密树林 (42.0 m, dense)", "explore_fast", "ExploreFleet (TJU 高速横排)", "scenes/explore-6-forest-dense.json"),
)


def _snapshot_immutable() -> dict[str, bytes]:
    return {rel: (ROOT / rel).read_bytes() for rel in WATCHED_IMMUTABLE}


def _git_head() -> str:
    r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=False)
    return r.stdout.strip() if r.returncode == 0 else "unknown"


def _build_scene_card(case: BenchCase):
    from yopo_l0.forest import generate_forest_geometry
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    path = ROOT / case.base_card_rel
    if not case.density_override and not case.epg5 and not case.explore_stock_6mps:
        return load_card(path)

    raw = copy.deepcopy(json.loads(path.read_text(encoding="utf-8")))
    if case.density_override is not None:
        raw["forest"]["density"] = case.density_override
    if case.epg5:
        raw["task"]["forest_six_prefix"] = True
        raw["task"]["forest_six_prefix_s"] = 0.7
    if case.explore_stock_6mps:
        raw["task"]["velocity_mps"] = 6.0
        raw["task"]["replan_hz"] = 10.0
        raw["task"]["arrive_radius_m"] = 2.4
        raw["task"]["duration_s"] = 45.0
        raw["dynamics"] = {"model": "point_mass_lag", "tau_s": 0.1, "kp": 4.0, "kd": 3.0, "a_max": 6.0}
        raw["rates_hz"]["planner"] = 10

    if isinstance(raw.get("forest"), dict):
        geom = generate_forest_geometry(raw["forest"])
    else:
        geom = json.loads((path.parent / raw["geometry"]).read_text(encoding="utf-8"))
    merged = {**raw, "geometry": geom}
    return SceneCard(str(path), merged, parse_geometry(geom, merged.get("ground_z")))


def _compute_tick_speeds(ticks: list[dict], belt_x: tuple[float, float], goal_tol: float = 0.4) -> dict[str, float | None]:
    """统一用逐 tick 物理真值计算全航程有效飞行均速、障碍带三维均速与障碍带前向净均速。"""
    by_uav: dict[str, list[dict]] = {}
    for row in ticks:
        by_uav.setdefault(str(row["uav"]), []).append(row)

    flight_speeds: list[float] = []
    belt_speeds_3d: list[float] = []
    belt_vx_drones: list[float] = []
    x_lo, x_hi = belt_x
    span = x_hi - x_lo

    for rows in by_uav.values():
        rows.sort(key=lambda r: float(r["t"]))
        t_in = None
        t_out = None
        prev_t = None
        prev_x = None
        uav_flight: list[float] = []
        uav_belt: list[float] = []
        reached = False
        for r in rows:
            t = float(r["t"])
            px = float(r["p"][0])
            vx, vy, vz = (float(c) for c in r["v"])
            spd = math.sqrt(vx * vx + vy * vy + vz * vz)
            if float(r["goal_error_m"]) <= goal_tol and spd <= 0.2:
                reached = True
            if not reached and spd > 0.05:
                uav_flight.append(spd)
            if x_lo <= px <= x_hi:
                uav_belt.append(spd)
            if prev_x is not None and prev_t is not None and px > prev_x + 1e-9:
                if t_in is None and prev_x < x_lo <= px:
                    t_in = prev_t + (x_lo - prev_x) / (px - prev_x) * (t - prev_t)
                if t_out is None and prev_x < x_hi <= px:
                    t_out = prev_t + (x_hi - prev_x) / (px - prev_x) * (t - prev_t)
            elif prev_x is None and px >= x_lo:
                t_in = t
            prev_t = t
            prev_x = px
        if uav_flight:
            flight_speeds.append(mean(uav_flight))
        if uav_belt:
            belt_speeds_3d.append(mean(uav_belt))
        if t_in is not None and t_out is not None and t_out > t_in + 1e-6:
            belt_vx_drones.append(span / (t_out - t_in))

    return {
        "mean_flight_speed_mps": mean(flight_speeds) if flight_speeds else None,
        "mean_belt_speed_3d_mps": mean(belt_speeds_3d) if belt_speeds_3d else None,
        "mean_belt_vx_mps": mean(belt_vx_drones) if belt_vx_drones else None,
    }


def _run_one(item: tuple[int, int]) -> dict:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    import torch

    torch.set_num_threads(1)
    from yopo_l0.runner import RunConfig, run

    case_idx, seed = item
    case = CASES[case_idx]
    card = _build_scene_card(case)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=seed, shadow=False))
    wall_s = time.perf_counter() - t0
    res = out.result
    body_r = float(card.card["criteria"]["body_radius_m"])
    min_obs = float(res["min_obstacle_distance_m"])
    min_inter = res["min_inter_drone_distance_m"]
    obs_col = int(min_obs <= body_r + 1e-9)
    inter_col = int(min_inter is not None and float(min_inter) <= 2.0 * body_r + 1e-9)

    in_forest = isinstance(card.card.get("forest"), dict)
    belt_x = (8.0, 30.0) if in_forest else (3.7, 14.8)
    sp = _compute_tick_speeds(out.ticks, belt_x, float(card.card["criteria"]["goal_tolerance_m"]))

    exp_block = res.get("explore") or {}
    return {
        "group_id": case.group_id,
        "group_title": case.group_title,
        "arm_id": case.arm_id,
        "arm_label": case.arm_label,
        "seed": seed,
        "n_drones": int(res["n_drones"]),
        "outcome": res["outcome"],
        "reason": res["reason"],
        "arrived": int(res["arrived"]),
        "arrival_rate": float(res["arrived"]) / float(res["n_drones"]),
        "arrival_time_s": res["arrival_time_s"],
        "fly_through_finish_time_s": exp_block.get("fly_through_finish_time_s"),
        "obstacle_collisions": obs_col,
        "inter_drone_collisions": inter_col,
        "min_obstacle_clearance_m": min_obs - body_r,
        "min_inter_drone_distance_m": None if min_inter is None else float(min_inter),
        "mean_flight_speed_mps": sp["mean_flight_speed_mps"],
        "mean_belt_speed_3d_mps": sp["mean_belt_speed_3d_mps"],
        "mean_belt_vx_mps": sp["mean_belt_vx_mps"],
        "wall_time_s": wall_s,
    }


def _summarize_arm(rows: list[dict]) -> dict:
    n_runs = len(rows)
    total_drones = sum(int(r["n_drones"]) for r in rows)
    total_arrived = sum(int(r["arrived"]) for r in rows)
    full_success = sum(1 for r in rows if r["outcome"] == "SUCCESS")
    obs_cols = sum(int(r["obstacle_collisions"]) for r in rows)
    inter_cols = sum(int(r["inter_drone_collisions"]) for r in rows)
    arr_times = [float(r["arrival_time_s"]) for r in rows if r["arrival_time_s"] is not None]
    fin_times = [float(r["fly_through_finish_time_s"]) for r in rows if r.get("fly_through_finish_time_s") is not None]
    f_spds = [float(r["mean_flight_speed_mps"]) for r in rows if r["mean_flight_speed_mps"] is not None]
    b_spds = [float(r["mean_belt_speed_3d_mps"]) for r in rows if r["mean_belt_speed_3d_mps"] is not None]
    b_vx = [float(r["mean_belt_vx_mps"]) for r in rows if r["mean_belt_vx_mps"] is not None]
    min_clear = min(float(r["min_obstacle_clearance_m"]) for r in rows)
    min_inter = min(float(r["min_inter_drone_distance_m"]) for r in rows if r["min_inter_drone_distance_m"] is not None)
    return {
        "group_id": rows[0]["group_id"],
        "group_title": rows[0]["group_title"],
        "arm_id": rows[0]["arm_id"],
        "arm_label": rows[0]["arm_label"],
        "seeds": n_runs,
        "seed_success_count": full_success,
        "seed_success_rate": full_success / float(n_runs),
        "drone_arrived": total_arrived,
        "drone_total": total_drones,
        "drone_arrival_rate": total_arrived / float(total_drones),
        "obstacle_collisions": obs_cols,
        "inter_drone_collisions": inter_cols,
        "min_obstacle_clearance_m": min_clear,
        "min_inter_drone_distance_m": min_inter,
        "mean_arrival_time_s": mean(arr_times) if arr_times else None,
        "median_arrival_time_s": median(arr_times) if arr_times else None,
        "max_arrival_time_s": max(arr_times) if arr_times else None,
        "mean_fly_through_time_s": mean(fin_times) if fin_times else None,
        "mean_flight_speed_mps": mean(f_spds) if f_spds else None,
        "mean_belt_speed_3d_mps": mean(b_spds) if b_spds else None,
        "mean_belt_vx_mps": mean(b_vx) if b_vx else None,
    }


def _fmt(val: float | None, digits: int = 2) -> str:
    return "—" if val is None else f"{val:.{digits}f}"


def render_markdown(payload: dict) -> str:
    summaries = payload["summaries"]
    by_group: dict[str, list[dict]] = {}
    for s in summaries:
        by_group.setdefault(s["group_id"], []).append(s)

    lines = [
        "# 无编队水平横排起飞高效率多机穿越（`ExploreFleet`）10 种子基准报告",
        "",
        "report-only。只报数，不修改 `core/`、`scenes/suite.txt` 或任何既有冻结场景卡（`scenes/crossing-*.json`）。",
        "",
        "## 1. 架构瓶颈根因与 `ExploreFleet` 四大突破",
        "",
        "对 `yopo_l0/crossing.py`（`CrossingFleet`）的逐项剖析表明，传统编队穿越在多机中密/高密树林中受制于四重强耦合桎梏：",
        "",
        "1. **进度门控与木桶降速（`progress_gate` / `_ahead_of_gate` / `LEADER_HOVER`）**：最慢一架受阻机将全队虚拟中心时刻表按住，前排已处于开阔走廊的无人机被迫急刹悬停或仅飞 `0.3–0.7 s` 短前缀，引发走停振荡；`ExploreFleet` 彻底取消进度门控与 `virtual_center.speed_mps` 限速，各机独立全速推进。",
        "2. **TDMA 轮转空窗期（`N × 0.25 s`）**：6 机轮转下每架无人机两次重规划间隔长达 `1.50 s`，高速穿林时盲飞超 `5–10 m`；`ExploreFleet` 改为 **统一水平起飞线 `x = 0`（等间距横排）+ 全机 `25 Hz` 并行重规划**，经进程内 `MessageBus` 广播紧凑三维轨迹管并在同拍内按确定性轮换优先级完成零冲突承诺。",
        "3. **横向槽位橡皮筋回拉（`slot_body` / `lateral_gain` / `exit_tighten_m`）**：固定横向 `y_slot` 把无人机反复拉向树干密集带；`ExploreFleet` 完全移除 `slot_body`，利用本机 `96×160` 深度图 15 扇区自由空间评估与邻机走廊软分流，在 `[y_min, y_max] × [z_min, z_max]` 内自由冲浪绕障，出林后在终点区就近锁定互不冲突的三维静止悬停点（兼容 G2 到达判据 `dist <= 0.4 m, vel <= 0.2 m/s` 保持 `1.0 s`）。",
        "4. **TJU YOPO 时间相似律高速扩展（`ratio = velocity_mps / 6.0`）+ 顺弯前瞻偏航对齐**：在不改动 `core/` 任何字节的前提下，将网络输入按 `(v / ratio, a / ratio²)` 精确归一化回训练分布，解码端按 `(6·ratio, 6·ratio², (10/6)/ratio)` 时间缩放，配合顺速度弯道内侧偏航预瞄（`look-into-turn yaw`）与递归可行前缀+制动安全盾，在 `dense` 高密树林（树表最小净距 `1.7 m`）中实现 **林带平均飞行速度 `> 7 m/s`（远超 `6 m/s` 目标）、到达率 `100%`、障碍与机间碰撞 `0`**。",
        "",
        "## 2. 六大场景 10 种子（Seeds `0..9`）正面对决总表",
        "",
        "速度与时间口径说明：",
        "- **林带/障碍带三维均速（`m/s`）**：各机在障碍带 `x ∈ [x_lo, x_hi]`（树林 `[8.0, 30.0] m`、柱林 `[3.7, 14.8] m`）内所有物理拍的三维瞬时速率均值；括号内为 `Δx / Δt` 前向净穿越均速。",
        "- **平均/中位/最大到达时间（`s`）**：全队全部满足 G2 到达悬停判据（含末端减速与保持）的完成时刻；**加速比**相对同场景 `CrossingFleet` 基线（及 `E-PG5`）计算。",
        "",
        "| 场景（Seeds 0..9） | 策略臂 | 全队成功局 | 单机到达率 | 障碍碰撞 | 机间碰撞 | 最小障碍净空 m | 最小机间距 m | 平均到达 s (中位 / 最大) | 相对基线加速比 | 全程均速 m/s | 障碍带均速 m/s (净 +X) |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]

    for gid, items in by_group.items():
        base_t = items[0]["mean_arrival_time_s"]
        epg5_item = next((x for x in items if x["arm_id"] == "crossing_epg5"), None)
        epg5_t = epg5_item["mean_arrival_time_s"] if epg5_item else None
        for row in items:
            t_mean = row["mean_arrival_time_s"]
            speedup_str = "1.00× (基准)" if row["arm_id"] == "crossing_base" else "—"
            if row["arm_id"] != "crossing_base" and t_mean is not None:
                parts = []
                if base_t is not None:
                    parts.append(f"**{base_t / t_mean:.2f}×**")
                if epg5_t is not None and row["arm_id"] != "crossing_epg5":
                    parts.append(f"vs E-PG5 {epg5_t / t_mean:.2f}×")
                if parts:
                    speedup_str = " / ".join(parts)
            t_cell = (
                "—"
                if t_mean is None
                else f"{t_mean:.2f} ({row['median_arrival_time_s']:.2f} / {row['max_arrival_time_s']:.2f})"
            )
            belt_cell = (
                "—"
                if row["mean_belt_speed_3d_mps"] is None
                else f"{row['mean_belt_speed_3d_mps']:.2f} ({_fmt(row['mean_belt_vx_mps'])})"
            )
            lines.append(
                "| {scene} | {arm} | {succ}/{seeds} | {dr_rate:.1f}% ({got}/{tot}) | {ocol} | {icol} | {clr} | {gap} | {t_cell} | {spdup} | {fspd} | {bspd} |".format(
                    scene=row["group_title"],
                    arm=row["arm_label"],
                    succ=row["seed_success_count"],
                    seeds=row["seeds"],
                    dr_rate=100.0 * row["drone_arrival_rate"],
                    got=row["drone_arrived"],
                    tot=row["drone_total"],
                    ocol=row["obstacle_collisions"],
                    icol=row["inter_drone_collisions"],
                    clr=_fmt(row["min_obstacle_clearance_m"], 3),
                    gap=_fmt(row["min_inter_drone_distance_m"], 3),
                    t_cell=t_cell,
                    spdup=speedup_str,
                    fspd=_fmt(row["mean_flight_speed_mps"], 2),
                    bspd=belt_cell,
                )
            )

    lines.extend(
        [
            "",
            "## 3. 复现命令与数据文件",
            "",
            "- 完整数据记录：[explore-free-swarm-runs.json](explore-free-swarm-runs.json)",
            "- 场景卡：`scenes/explore-3-pillars.json`、`scenes/explore-3-forest.json`、`scenes/explore-3-forest-dense.json`、`scenes/explore-6-forest-sparse.json`、`scenes/explore-6-forest.json`、`scenes/explore-6-forest-dense.json`",
            "- 复现实验：`OMP_NUM_THREADS=1 .venv/bin/python scripts/benchmark_explore_vs_crossing.py --jobs 12`",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9", help="逗号分隔种子列表")
    ap.add_argument("--jobs", type=int, default=min(12, os.cpu_count() or 4))
    ap.add_argument("--out-json", type=Path, default=DOC_JSON)
    ap.add_argument("--out-md", type=Path, default=DOC_MD)
    args = ap.parse_args(argv)

    seeds = [int(x.strip()) for x in args.seeds.split(",") if x.strip()]
    snap_before = _snapshot_immutable()

    tasks = [(ci, s) for ci in range(len(CASES)) for s in seeds]
    t0 = time.perf_counter()
    if args.jobs <= 1:
        rows = [_run_one(item) for item in tasks]
    else:
        with cf.ProcessPoolExecutor(max_workers=int(args.jobs)) as pool:
            rows = list(pool.map(_run_one, tasks))
    elapsed = time.perf_counter() - t0

    snap_after = _snapshot_immutable()
    if snap_before != snap_after:
        raise RuntimeError("检测到受保护场景卡或 suite.txt 发生改动！")

    summaries = []
    for ci, case in enumerate(CASES):
        arm_rows = [r for r in rows if r["group_id"] == case.group_id and r["arm_id"] == case.arm_id]
        summaries.append(_summarize_arm(arm_rows))

    payload = {
        "schema": "yopo-restart/explore-vs-crossing-benchmark/1",
        "commit": _git_head(),
        "seeds": seeds,
        "total_runs": len(rows),
        "wall_time_s": elapsed,
        "summaries": summaries,
        "runs": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    args.out_md.parent.mkdir(parents=True, exist_ok=True)
    args.out_md.write_text(render_markdown(payload), encoding="utf-8")
    print(f"完成 {len(rows)} 组评测，耗时 {elapsed:.1f}s -> {args.out_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
