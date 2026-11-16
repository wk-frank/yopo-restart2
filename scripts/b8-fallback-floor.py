#!/usr/bin/env python3
"""票 B8：柱净空门全拒退路地板（pillar_fallback_min_m）+ 起步限加速与斜坡组合评估。只报数。

用法：
  .venv/bin/python scripts/b8-fallback-floor.py l0 --jobs 4
  .venv/bin/python scripts/b8-fallback-floor.py sitl --arms a_reframp_p5_f3,a_reframp_p3_f2 --seeds 0,1,2,3,4,5,6,7,8,9
  .venv/bin/python scripts/b8-fallback-floor.py export
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.b2_plan_diag import apply_b2_overrides, b3_from_fix, parse_fix  # noqa: E402
from yopo_l0.criteria import evaluate_card_rows  # noqa: E402
from yopo_l0.experiment import comma_ints, comma_list, run_spawn_jobs, write_json  # noqa: E402
from yopo_l0.runner import RunConfig, run  # noqa: E402
from yopo_l0.scene import load_card  # noqa: E402

OPS_DIR = Path("/workspace/yopo-ops/b8")
L0_OUT = OPS_DIR / "l0-summary.json"
SITL_INDEX = OPS_DIR / "sitl-runs.json"
DOC_RUNS = ROOT / "docs" / "research" / "b8-fallback-floor-runs.json"
LOCK_FILE = "/tmp/yopo-sitl.lock"

ARMS: dict[str, dict[str, str]] = {
    "d": {"fix": "", "label": "缺省"},
    "a": {"fix": "launch=3:1.5", "label": "(a) launch=3:1.5"},
    "ap3_f2": {"fix": "launch=3:1.5,pillar=0.3,floor=0.2", "label": "(a)+柱门 0.3 m+地板 0.2 m"},
    "ap5_f3": {"fix": "launch=3:1.5,pillar=0.5,floor=0.3", "label": "(a)+柱门 0.5 m+地板 0.3 m"},
    "a_reframp_p3_f2": {
        "fix": "launch=3:1.5,reframp=3,pillar=0.3,floor=0.2",
        "label": "(a)+参考斜坡 3+柱门 0.3 m+地板 0.2 m",
    },
    "a_reframp_p5_f3": {
        "fix": "launch=3:1.5,reframp=3,pillar=0.5,floor=0.3",
        "label": "(a)+参考斜坡 3+柱门 0.5 m+地板 0.3 m",
    },
    "a_ramp_p3_f2": {
        "fix": "launch=3:1.5,ramp=3,pillar=0.3,floor=0.2",
        "label": "(a)+中心斜坡 3+柱门 0.3 m+地板 0.2 m",
    },
    "a_ramp_p5_f3": {
        "fix": "launch=3:1.5,ramp=3,pillar=0.5,floor=0.3",
        "label": "(a)+中心斜坡 3+柱门 0.5 m+地板 0.3 m",
    },
}


def _r(val, nd: int = 4):
    if val is None or not math.isfinite(float(val)):
        return None
    return round(float(val), nd)


def _traj_sha(ticks: list[dict]) -> str:
    h = hashlib.sha256()
    for row in ticks:
        blob = json.dumps([row["uav"], row["p"], row["v"], row["a"]], separators=(",", ":"))
        h.update(blob.encode())
    return h.hexdigest()


def _l0_worker(spec: tuple[str, str, int]) -> dict:
    arm, fix_str, seed = spec
    base = load_card(ROOT / "scenes/crossing-pillars.json")
    fix = parse_fix(fix_str)
    card, _ = apply_b2_overrides(base, b3=b3_from_fix(fix))
    out = run(card, RunConfig(policy="crossing", seed=int(seed), shadow=True))
    res = out.result
    _applied, gate_failures = evaluate_card_rows(base.card, [res], policy="crossing")
    pc = (res.get("crossing") or {}).get("pillar_clearance") or {}
    return {
        "arm": arm,
        "seed": int(seed),
        "outcome": res["outcome"],
        "reason": res["reason"],
        "arrived": int(res["arrived"]),
        "arrival_time_s": res["arrival_time_s"],
        "formation_error_p95_m": res["formation_error_p95_m"],
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "upstream_deviation": bool(res.get("upstream_deviation")),
        "gates_passed": not gate_failures,
        "gate_failures": list(gate_failures),
        "checks": int(pc.get("checks", 0)),
        "rejected": int(pc.get("rejected", 0)),
        "fallbacks": int(pc.get("fallbacks", 0)),
        "floor_blocked": int(pc.get("floor_blocked", 0)),
        "decisions": dict((res.get("crossing") or {}).get("decisions") or {}),
        "traj_sha256": _traj_sha(out.ticks),
    }


def _summarize_l0_arm(arm: str, rows: list[dict]) -> dict:
    rows = sorted(rows, key=lambda r: r["seed"])
    arr_t = [float(r["arrival_time_s"]) for r in rows if r["arrival_time_s"] is not None]
    p95s = [float(r["formation_error_p95_m"]) for r in rows if r["formation_error_p95_m"] is not None]
    obs_d = [float(r["min_obstacle_distance_m"]) for r in rows]
    inter_d = [float(r["min_inter_drone_distance_m"]) for r in rows if r["min_inter_drone_distance_m"] is not None]
    fails = [[r["seed"], r["gate_failures"]] for r in rows if not r["gates_passed"]]
    collisions = sum(1 for r in rows if r["outcome"] == "COLLISION")
    return {
        "fix": ARMS[arm]["fix"],
        "label": ARMS[arm]["label"],
        "n": len(rows),
        "success": sum(1 for r in rows if r["outcome"] == "SUCCESS"),
        "gates_pass_n": sum(1 for r in rows if r["gates_passed"]),
        "collisions": collisions,
        "t_mean": _r(np.mean(arr_t), 2) if arr_t else None,
        "t_min": _r(min(arr_t), 2) if arr_t else None,
        "t_max": _r(max(arr_t), 2) if arr_t else None,
        "form_p95_max": float(max(p95s)) if p95s else None,
        "form_p95_per_seed": [_r(r["formation_error_p95_m"], 3) for r in rows],
        "min_obs_dist": _r(min(obs_d), 3) if obs_d else None,
        "min_inter_dist": _r(min(inter_d), 3) if inter_d else None,
        "dev": any(r["upstream_deviation"] for r in rows),
        "checks": sum(r["checks"] for r in rows),
        "rej": sum(r["rejected"] for r in rows),
        "fallbacks": sum(r["fallbacks"] for r in rows),
        "floor_blocked": sum(r["floor_blocked"] for r in rows),
        "fails": fails,
        "rows": rows,
    }


def cmd_l0(args: argparse.Namespace) -> int:
    arms = comma_list(args.arms)
    seeds = comma_ints(args.seeds)
    specs = [(arm, ARMS[arm]["fix"], seed) for arm in arms for seed in seeds]
    t0 = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    rows = run_spawn_jobs(specs, _l0_worker, int(args.jobs))
    t1 = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    by_arm: dict[str, list[dict]] = {arm: [] for arm in arms}
    for row in rows:
        by_arm[row["arm"]].append(row)
    summary = {
        "started_at": t0,
        "finished_at": t1,
        "arms": {arm: _summarize_l0_arm(arm, by_arm[arm]) for arm in arms},
    }
    # 核验 (a)+参考斜坡 的飞行轨迹与 (a) 逐位一致
    if "a" in summary["arms"] and "a_reframp_p5_f3" in summary["arms"]:
        sha_a = [r["traj_sha256"] for r in summary["arms"]["a"]["rows"]]
        sha_ref = [r["traj_sha256"] for r in summary["arms"]["a_reframp_p5_f3"]["rows"]]
        summary["reframp_traj_identical_to_a"] = sha_a == sha_ref
    write_json(L0_OUT, summary)
    for arm in arms:
        blk = summary["arms"][arm]
        print(
            f"L0 {arm:18s}: success={blk['success']}/{blk['n']} gates={blk['gates_pass_n']}/{blk['n']} "
            f"coll={blk['collisions']} t_mean={blk['t_mean']} p95_max={blk['form_p95_max']:.3f} "
            f"obs_min={blk['min_obs_dist']} checks/rej/fb/blk={blk['checks']}/{blk['rej']}/{blk['fallbacks']}/{blk['floor_blocked']}",
            flush=True,
        )
    return 0


def _extract_sitl_row(seed: int, run_dir: Path, payload: dict) -> dict:
    diag = payload.get("b2_plan_diag") or {}
    per = diag.get("per_drone") or {}
    sp_vals = [
        float(v["min_setpoint_clear_m"])
        for v in per.values()
        if isinstance(v, dict) and v.get("min_setpoint_clear_m") is not None
    ]
    lat_vals = [
        float(v["track_lat_abs_moving"]["p95"])
        for v in per.values()
        if isinstance(v, dict)
        and isinstance(v.get("track_lat_abs_moving"), dict)
        and v["track_lat_abs_moving"].get("p95") is not None
    ]
    crossing_blk = payload.get("crossing") or {}
    t_arr = payload.get("arrival_after_handover_s")
    if payload.get("arrival_time_s") is not None and payload.get("formation_t0_s") is not None:
        t_arr = float(payload["arrival_time_s"]) - float(payload["formation_t0_s"])
    return {
        "seed": int(seed),
        "run_dir": str(run_dir),
        "outcome": payload.get("outcome"),
        "reason": payload.get("reason"),
        "arrived": int(payload.get("arrived") or 0),
        "collisions": int(payload.get("collisions") or 0),
        "t_arr": t_arr,
        "form_p95": payload.get("formation_error_p95_m"),
        "plan_m": _r(payload.get("min_surface_clearance_plan_m"), 3),
        "flight_m": _r(payload.get("min_surface_clearance_flight_m"), 3),
        "min_inter_plan_m": _r(payload.get("min_inter_drone_distance_plan_m"), 3),
        "lat_p95": _r(max(lat_vals), 3) if lat_vals else None,
        "b3": (payload.get("b3") or {}).get("counts"),
        "px4": payload.get("px4_parameters_written"),
        "rtf": _r(payload.get("rtf"), 2),
        "dev": payload.get("b2_deviations") or [],
        "lockstep": bool((payload.get("lockstep") or {}).get("engaged")),
        "sp_min": _r(min(sp_vals), 4) if sp_vals else None,
        "plan_clear_min": diag.get("plan_clear_min_m"),
        "decisions": crossing_blk.get("decisions") or {},
        "pillar": crossing_blk.get("pillar_clearance"),
    }


def _summarize_sitl_arm(arm: str, rows: list[dict], retries: list[dict]) -> dict:
    rows = sorted(rows, key=lambda r: r["seed"])
    t_vals = [float(r["t_arr"]) for r in rows if r["outcome"] == "SUCCESS" and r["t_arr"] is not None]
    lat_vals = [float(r["lat_p95"]) for r in rows if r["lat_p95"] is not None]
    form_vals = [float(r["form_p95"]) for r in rows if r["form_p95"] is not None]
    sp_vals = [float(r["sp_min"]) for r in rows if r["sp_min"] is not None]
    fails = [[r["seed"], r["reason"], r["plan_m"]] for r in rows if r["outcome"] != "SUCCESS"]
    fb_trips = []
    blk_trips = []
    for r in rows:
        pil = r.get("pillar") or {}
        if int(pil.get("fallbacks", 0)) > 0:
            fb_trips.append([r["seed"], r["outcome"], pil["rejected"], pil["fallbacks"], pil.get("floor_blocked", 0), r["sp_min"]])
        if int(pil.get("floor_blocked", 0)) > 0:
            blk_trips.append([r["seed"], r["outcome"], r["reason"], pil["rejected"], pil.get("floor_blocked", 0), r["decisions"]])
    return {
        "fix": ARMS[arm]["fix"],
        "label": ARMS[arm]["label"],
        "agg": {
            "n": len(rows),
            "success": sum(1 for r in rows if r["outcome"] == "SUCCESS"),
            "arrived": sum(int(r["arrived"]) for r in rows),
            "collisions": sum(int(r["collisions"]) for r in rows),
            "plan_timeouts": sum(1 for r in rows if "PLAN_TIMEOUT" in str(r.get("reason") or "")),
            "t_med": _r(np.median(t_vals), 2) if t_vals else None,
            "t_min": _r(min(t_vals), 2) if t_vals else None,
            "t_max": _r(max(t_vals), 2) if t_vals else None,
            "lat_med": _r(np.median(lat_vals), 3) if lat_vals else None,
            "lat_min": _r(min(lat_vals), 3) if lat_vals else None,
            "lat_max": _r(max(lat_vals), 3) if lat_vals else None,
            "form_p95_med": _r(np.median(form_vals), 3) if form_vals else None,
            "form_p95_max": _r(max(form_vals), 3) if form_vals else None,
            "sp_min": _r(min(sp_vals), 4) if sp_vals else None,
            "checks_total": sum(int((r.get("pillar") or {}).get("checks", 0)) for r in rows),
            "rejected_total": sum(int((r.get("pillar") or {}).get("rejected", 0)) for r in rows),
            "fallbacks_total": sum(int((r.get("pillar") or {}).get("fallbacks", 0)) for r in rows),
            "floor_blocked_total": sum(int((r.get("pillar") or {}).get("floor_blocked", 0)) for r in rows),
            "fails": fails,
            "px4_clean": all(r["px4"] == [] for r in rows),
            "lockstep": all(bool(r["lockstep"]) for r in rows),
            "fallback_trips": fb_trips,
            "floor_blocked_trips": blk_trips,
            "lockstep_timeout_retries": retries,
        },
        "rows": rows,
    }


def _run_one_sitl(arm: str, seed: int) -> tuple[Path, dict]:
    state_dir = OPS_DIR / "sitl" / arm
    state_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["YOPO_SITL_STATE"] = str(state_dir)
    env["YOPO_B2_PLAN_DIAG"] = "1"
    env["YOPO_B2_FIX"] = ARMS[arm]["fix"]
    # WSL2 Jammy 的 dockerd 在 nf_tables 下创建新网桥缺 DOCKER-ISOLATION-STAGE-2；
    # 本脚本每趟都经 flock /tmp/yopo-sitl.lock 串行独占运行，指定已有 bridge 网络既无跨容器串话也不写主机规则。
    env.setdefault("YOPO_SITL_DOCKER_NETWORK", "bridge")
    launcher = ROOT / "scripts/sitl/run_yopo_l0_sitl.sh"
    cmd = [
        "flock",
        LOCK_FILE,
        "bash",
        str(launcher),
        "--lockstep",
        "--scene",
        "scenes/crossing-pillars-sitl80.json",
        "--n",
        "3",
        "--policy",
        "crossing",
        "--seed",
        str(seed),
    ]
    proc = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True)
    combined = (proc.stdout or "") + "\n" + (proc.stderr or "")
    matched = re.findall(r"run_dir\s+(\S+)\s+rc=", combined)
    if not matched:
        raise RuntimeError(f"SITL {arm} seed {seed} 未找到 run_dir (rc={proc.returncode}):\n{combined[-1500:]}")
    run_dir = Path(matched[-1])
    res_path = run_dir / "result.json"
    if not res_path.is_file():
        raise RuntimeError(f"SITL {arm} seed {seed} 缺少 {res_path}:\n{combined[-1500:]}")
    payload = json.loads(res_path.read_text(encoding="utf-8"))
    return run_dir, payload


def cmd_sitl(args: argparse.Namespace) -> int:
    arms = comma_list(args.arms)
    seeds = comma_ints(args.seeds)
    existing: dict = {}
    if SITL_INDEX.is_file():
        existing = json.loads(SITL_INDEX.read_text(encoding="utf-8"))
    sitl_arms = dict(existing.get("arms") or {})
    for arm in arms:
        prev_block = sitl_arms.get(arm) or {}
        by_seed = {int(r["seed"]): r for r in (prev_block.get("rows") or [])}
        retries = list((prev_block.get("agg") or {}).get("lockstep_timeout_retries") or [])
        t_start = prev_block.get("started_at") or time.strftime("%Y-%m-%dT%H:%M:%S%z")
        for seed in seeds:
            if not args.force and seed in by_seed:
                print(f"SITL {arm} seed {seed}: already recorded ({by_seed[seed]['outcome']}), skipping", flush=True)
                continue
            print(f"SITL {arm} seed {seed}: starting (fix={ARMS[arm]['fix']})...", flush=True)
            run_dir, payload = _run_one_sitl(arm, seed)
            if "LOCKSTEP_TIMEOUT" in str(payload.get("reason") or ""):
                retries.append({"seed": seed, "voided_run_dir": str(run_dir), "reason": payload.get("reason")})
                print(f"SITL {arm} seed {seed}: LOCKSTEP_TIMEOUT on {run_dir.name}, retrying once...", flush=True)
                run_dir, payload = _run_one_sitl(arm, seed)
            if payload.get("px4_parameters_written") != []:
                raise RuntimeError(f"px4_parameters_written 非空：{payload.get('px4_parameters_written')}")
            row = _extract_sitl_row(seed, run_dir, payload)
            by_seed[seed] = row
            pil = row.get("pillar") or {}
            print(
                f"SITL {arm} seed {seed}: {row['outcome']} ({row['reason']}) arrived={row['arrived']}/3 "
                f"coll={row['collisions']} t_arr={row['t_arr']} sp_min={row['sp_min']} "
                f"checks/rej/fb/blk={pil.get('checks')}/{pil.get('rejected')}/{pil.get('fallbacks')}/{pil.get('floor_blocked')}",
                flush=True,
            )
            block = _summarize_sitl_arm(arm, list(by_seed.values()), retries)
            block["started_at"] = t_start
            block["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            sitl_arms[arm] = block
            write_json(SITL_INDEX, {"arms": sitl_arms})
    return 0


def cmd_export(_args: argparse.Namespace) -> int:
    l0 = json.loads(L0_OUT.read_text(encoding="utf-8")) if L0_OUT.is_file() else {}
    sitl = json.loads(SITL_INDEX.read_text(encoding="utf-8")) if SITL_INDEX.is_file() else {}
    l0_compact = {}
    for arm, blk in (l0.get("arms") or {}).items():
        l0_compact[arm] = {k: v for k, v in blk.items() if k != "rows"}
    doc = {
        "ticket": "B8",
        "title": "票 B8：柱净空门全拒退路地板与起步斜坡组合（默认关，只报数）",
        "disclaimer": (
            "只报数。所有研究开关（pillar_clearance_m、pillar_fallback_min_m、launch_a_max_mps2、"
            "ref_ramp_mps2、center_ramp_mps2）默认关。打开柱门/地板即偏离上游（读场景柱几何）。"
            "每趟 SITL px4_parameters_written 都是空列表，lockstep.engaged=true，使用每趟隔离 Docker 网络。"
        ),
        "switch": {
            "key": "policy_params.b3.pillar_fallback_min_m",
            "sitl_fix": "floor=<m>（或 pillar=<m>:<floor>，需与 pillar=<m> 同开且 0 < floor <= pillar）",
            "default": "off",
            "behavior": "一批候选全被柱门拒绝时，若离柱最远候选的保守净空 gap 仍低于 pillar_fallback_min_m，不交 PILLAR_FALLBACK，改走运动学刹车（BRAKE_ONLY）或保持上一份已提交计划（CLEARANCE_SKIP）",
        },
        "l0": {
            "started_at": l0.get("started_at"),
            "finished_at": l0.get("finished_at"),
            "reframp_traj_identical_to_a": l0.get("reframp_traj_identical_to_a"),
            "arms": l0_compact,
        },
        "sitl": (sitl.get("arms") or {}),
    }
    write_json(DOC_RUNS, doc)
    print(f"wrote {DOC_RUNS}", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_l0 = sub.add_parser("l0")
    p_l0.add_argument(
        "--arms",
        default="d,a,ap3_f2,ap5_f3,a_reframp_p3_f2,a_reframp_p5_f3,a_ramp_p3_f2,a_ramp_p5_f3",
    )
    p_l0.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    p_l0.add_argument("--jobs", type=int, default=4)
    p_l0.set_defaults(func=cmd_l0)

    p_sitl = sub.add_parser("sitl")
    p_sitl.add_argument("--arms", default="a_reframp_p5_f3,a_reframp_p3_f2")
    p_sitl.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    p_sitl.add_argument("--force", action="store_true")
    p_sitl.set_defaults(func=cmd_sitl)

    p_exp = sub.add_parser("export")
    p_exp.set_defaults(func=cmd_export)

    args = ap.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
