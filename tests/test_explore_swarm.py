"""ExploreFleet（无编队水平横排起飞高效率多机穿越）单元与闭环验证测试。"""

from __future__ import annotations

import copy
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from conftest import ROOT
from yopo_swarm_core.candidates import UPSTREAM_DECODER, decode_params, minco_two_piece
from yopo_l0.bus import MessageBus
from yopo_l0.dynamics import State
from yopo_l0.explore import (
    ExploreFleet,
    LocalApproval,
    _FastObstacleField,
    _peer_tube_min_distance,
    _sample_positions,
    build_scaled_decoder,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Box, Cylinder, Ground, load_card
from yopo_l0.sensor import DepthFrame

EXPLORE_SCENES = (
    "explore-3-pillars.json",
    "explore-3-forest.json",
    "explore-3-forest-dense.json",
    "explore-6-forest-sparse.json",
    "explore-6-forest.json",
    "explore-6-forest-dense.json",
)
_STRIP = set(WALL_FIELDS) | {"commit"}


def _strip(result: dict) -> dict:
    return {k: v for k, v in result.items() if k not in _STRIP}


def test_explore_scene_cards_strict_and_horizontal_takeoff_line():
    suite_lines = {
        line.strip()
        for line in (ROOT / "scenes/suite.txt").read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    }
    proc = subprocess.run(
        [sys.executable, str(ROOT / "scripts/check_scenes.py"), "--strict"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr

    for name in EXPLORE_SCENES:
        path = ROOT / "scenes" / name
        assert path.is_file(), f"缺少场景卡 {name}"
        assert f"scenes/{name}" not in suite_lines
        assert name not in suite_lines

        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw["task"]["type"] == "explore"
        assert raw["criteria"]["frozen"] is False
        drones = raw["drones"]
        xs = [float(d["start_enu"][0]) for d in drones]
        ys = sorted(float(d["start_enu"][1]) for d in drones)
        zs = [float(d["start_enu"][2]) for d in drones]
        # 统一水平线 x = 0 起飞且高度一致，横向等间距
        assert all(abs(x - 0.0) <= 1e-9 for x in xs)
        assert all(abs(z - 1.5) <= 1e-9 for z in zs)
        assert all("slot_body" not in d for d in drones)
        diffs = np.diff(ys)
        assert np.allclose(diffs, diffs[0], atol=1e-9)
        if len(drones) == 3:
            assert ys == pytest.approx([-8.0, 0.0, 8.0])
        else:
            assert len(drones) == 6
            assert ys == pytest.approx([-7.5, -4.5, -1.5, 1.5, 4.5, 7.5])


def test_explore_fleet_parameter_validation():
    base = copy.deepcopy(load_card(ROOT / "scenes/explore-3-pillars.json").card)
    f = ExploreFleet.from_card(base, 0.01)
    assert f.policy_name == "explore"
    assert f.result_key == "explore"
    assert f.uses_network is True
    assert f.geom.enabled is False
    assert isinstance(LocalApproval().last_code, str)

    wrong_type = copy.deepcopy(base)
    wrong_type["task"]["type"] = "crossing"
    with pytest.raises(ValueError, match="explore"):
        ExploreFleet.from_card(wrong_type, 0.01)

    dup = copy.deepcopy(base)
    dup["drones"][1]["id"] = dup["drones"][0]["id"]
    with pytest.raises(ValueError, match="重复"):
        ExploreFleet.from_card(dup, 0.01)

    for bad_vel in (0.0, -3.0, True, "9.0", float("nan"), float("inf")):
        bad = copy.deepcopy(base)
        bad["task"]["velocity_mps"] = bad_vel
        with pytest.raises(ValueError, match="velocity_mps"):
            ExploreFleet.from_card(bad, 0.01)

    bad_bounds = copy.deepcopy(base)
    bad_bounds["task"]["y_bounds_m"] = [5.0, -5.0]
    with pytest.raises(ValueError, match="y_bounds_m"):
        ExploreFleet.from_card(bad_bounds, 0.01)

    with pytest.raises(ValueError, match="start_enu"):
        f.check_start("uav0", np.array([1.0, 0.0, 1.5]))


def test_tju_yopo_time_similarity_spatial_curve_invariance():
    """验证 TJU YOPO 时间相似律 t -> t/ratio 保持三维空间避障曲线逐点严格一致。"""
    rng = np.random.default_rng(20261116)
    raw_params = rng.uniform(-0.6, 0.6, size=(15, 14))

    dec1, r1 = build_scaled_decoder(6.0, 0.05)
    dec2, r2 = build_scaled_decoder(10.2, 0.05)
    assert r1 == pytest.approx(1.0)
    assert r2 == pytest.approx(1.7)
    assert dec2.vel_max == pytest.approx(6.0 * 1.7)
    assert dec2.acc_max == pytest.approx(6.0 * 1.7 * 1.7)
    assert dec2.segment_time == pytest.approx((10.0 / 6.0) / 1.7)
    assert dec1.grid_angles()[0] == pytest.approx(UPSTREAM_DECODER.grid_angles()[0])

    inner1, tail1, _ = decode_params(raw_params, dec1)
    inner2, tail2, _ = decode_params(raw_params, dec2)
    dur1 = np.maximum((raw_params[:, 12:14] + 1.0) * (0.5 * dec1.segment_time), 0.1 / r1)
    dur2 = np.maximum((raw_params[:, 12:14] + 1.0) * (0.5 * dec2.segment_time), 0.1 / r2)

    p0 = np.array([2.0, -1.5, 1.5])
    v0 = np.array([4.2, 0.8, -0.2])
    a0 = np.array([1.1, -0.5, 0.3])
    head1 = np.array([p0, v0, a0])
    head2 = np.array([p0, v0 * r2, a0 * r2 * r2])

    for i in range(15):
        tr1 = minco_two_piece(0.0, head1, tail1[i], inner1[i], dur1[i])
        tr2 = minco_two_piece(0.0, head2, tail2[i], inner2[i], dur2[i])
        taus = np.linspace(0.0, float(tr1.t_end), 25)
        pts1 = _sample_positions(tr1, taus)
        pts2 = _sample_positions(tr2, taus / r2)
        assert np.max(np.abs(pts1 - pts2)) < 1e-10
        # 速度幅值严格按 ratio 放大
        for tau in (0.0, 0.35 * float(tr1.t_end), 0.8 * float(tr1.t_end)):
            v_unit = tr1.evaluate(float(tau), 1)
            v_fast = tr2.evaluate(float(tau / r2), 1)
            assert np.allclose(v_fast, v_unit * r2, atol=1e-9)


def test_fast_obstacle_field_matches_scene_obstacle_distances():
    obs = (
        Ground(0.0),
        Cylinder("up", (5.0, 1.0), 0.3, 0.0, 4.5, (0.0, 0.0)),
        Cylinder("lean", (8.0, -2.0), 0.25, 0.0, 5.0, (0.8, -0.6)),
        Box("branch", (7.2, -1.0, 1.2), (8.4, -0.2, 1.4)),
    )
    field = _FastObstacleField(obs)
    rng = np.random.default_rng(42)
    pts = rng.uniform(low=[0.0, -6.0, 0.2], high=[12.0, 6.0, 4.0], size=(64, 3))
    for i in range(pts.shape[0]):
        expected = min(o.distance(pts[i]) for o in obs)
        actual = field.min_distance(pts[i : i + 1])
        assert actual == pytest.approx(expected, abs=1e-9)


def test_peer_tube_catches_hovering_peer_after_tube_end_and_terminal_lock_deconflicts():
    # 邻机在 t=1.0 停在 (10, 0, 1.5)，本机在 t=1.8 经过 (10, 0.3, 1.5) 必须检出 0.3m 冲突！
    peer = np.array([[0.0, 8.0, 0.0, 1.5], [1.0, 10.0, 0.0, 1.5]], dtype=float)
    times = np.array([1.2, 1.5, 1.8], dtype=float)
    pts = np.array([[8.5, 0.3, 1.5], [9.3, 0.3, 1.5], [10.0, 0.3, 1.5]], dtype=float)
    assert _peer_tube_min_distance(times, pts, [peer]) == pytest.approx(0.3, abs=1e-9)

    # 两架飞机沿同一 y=2.0 走廊进入终点区时，经总线广播后锁定互不重叠的静止悬停点
    card = copy.deepcopy(load_card(ROOT / "scenes/explore-3-pillars.json").card)
    fleet = ExploreFleet.from_card(card, 0.01)
    bus = MessageBus()
    depth = np.full((96, 160), 18.0, dtype=np.float32)
    frame = DepthFrame(0.0, depth, (80.0, 80.0, 80.0, 48.0))

    fleet.ingest("uav0", frame, State(0.0, np.array([18.5, 2.0, 1.5]), np.array([3.0, 0.0, 0.0]), np.zeros(3), 0.0))
    fleet.ingest("uav1", frame, State(0.0, np.array([18.2, 2.05, 1.5]), np.array([3.0, 0.0, 0.0]), np.zeros(3), 0.0))
    fleet.ingest("uav2", frame, State(0.0, np.array([5.0, -8.0, 1.5]), np.array([5.0, 0.0, 0.0]), np.zeros(3), 0.0))
    fleet.exchange(bus, 0.0)

    assert fleet.terminal_locked("uav0") is True
    assert fleet.terminal_locked("uav1") is True
    assert fleet.terminal_locked("uav2") is False
    g0 = fleet.goal_of("uav0")
    g1 = fleet.goal_of("uav1")
    assert g0[0] >= fleet.x_goal - 1e-9
    assert g1[0] >= fleet.x_goal - 1e-9
    assert float(np.linalg.norm(g0 - g1)) >= fleet.terminal_sep_m - 1e-9


def test_existing_policies_remain_bit_identical():
    # 验证 crossing、formation、upstream、hover 四条既有路径确定性与行为零扰动
    for rel, pol in (
        ("scenes/g2-hover.json", "hover"),
        ("scenes/g2-pillar.json", "upstream"),
        ("scenes/crossing-pillars.json", "upstream"),
    ):
        card = load_card(ROOT / rel)
        out1 = run(card, RunConfig(seed=0, policy=pol, duration_s=1.5, shadow=False))
        out2 = run(card, RunConfig(seed=0, policy=pol, duration_s=1.5, shadow=False))
        assert _strip(out1.result) == _strip(out2.result)
        assert out1.ticks == out2.ticks


def test_explore_closed_loop_pillars_and_dense_forest():
    pillars = load_card(ROOT / "scenes/explore-3-pillars.json")
    out_p1 = run(pillars, RunConfig(seed=0, shadow=False, record_planner_inputs=True))
    out_p2 = run(pillars, RunConfig(seed=0, shadow=False, record_planner_inputs=True))
    assert _strip(out_p1.result) == _strip(out_p2.result)
    assert out_p1.planner_inputs == out_p2.planner_inputs
    assert out_p1.result["outcome"] == "SUCCESS"
    assert out_p1.result["arrived"] == 3
    assert out_p1.result["explore"]["obstacle_collisions"] == 0
    assert out_p1.result["explore"]["inter_drone_collisions"] == 0
    assert out_p1.result["explore"]["progress_gate"] is False
    assert out_p1.result["formation_error_p95_m"] is None

    dense3 = load_card(ROOT / "scenes/explore-3-forest-dense.json")
    out_d = run(dense3, RunConfig(seed=0, shadow=False))
    assert out_d.result["outcome"] == "SUCCESS"
    assert out_d.result["arrived"] == 3
    assert out_d.result["explore"]["obstacle_collisions"] == 0
    assert out_d.result["explore"]["inter_drone_collisions"] == 0
    assert float(out_d.result["explore"]["avg_forest_speed_mps"]) > 6.0


def test_benchmark_report_and_runs_json_integrity():
    runs_path = ROOT / "docs/research/explore-free-swarm-runs.json"
    md_path = ROOT / "docs/research/explore-free-swarm.md"
    assert runs_path.is_file()
    assert md_path.is_file()

    payload = json.loads(runs_path.read_text(encoding="utf-8"))
    assert payload["schema"] == "yopo-restart/explore-vs-crossing-benchmark/1"
    assert payload["seeds"] == list(range(10))
    assert payload["total_runs"] == 210
    assert len(payload["runs"]) == 210

    by_key = {(s["group_id"], s["arm_id"]): s for s in payload["summaries"]}
    for gid in (
        "3_pillars",
        "3_forest_med",
        "3_forest_dense",
        "6_forest_sparse",
        "6_forest_med",
        "6_forest_dense",
    ):
        for arm in ("explore_6mps", "explore_fast"):
            row = by_key[(gid, arm)]
            assert row["seed_success_count"] == 10
            assert row["drone_arrival_rate"] == pytest.approx(1.0)
            assert row["obstacle_collisions"] == 0
            assert row["inter_drone_collisions"] == 0
            assert row["min_obstacle_clearance_m"] > 0.0
            assert row["min_inter_drone_distance_m"] > 0.70
        fast = by_key[(gid, "explore_fast")]
        base = by_key[(gid, "crossing_base")]
        assert fast["mean_belt_speed_3d_mps"] > 6.0
        assert fast["mean_belt_vx_mps"] > 6.0
        assert fast["mean_arrival_time_s"] < base["mean_arrival_time_s"]

