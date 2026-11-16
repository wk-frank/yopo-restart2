"""票 B8：柱净空门全拒退路地板（pillar_fallback_min_m，默认关）与起步限加速+斜坡组合。"""

from dataclasses import replace

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.b2_plan_diag import apply_b2_overrides, b3_from_fix, fix_deviations, parse_fix
from yopo_l0.scene import load_card
from yopo_l0.sensor import DepthFrame
from yopo_swarm_core.plan import make_plan, quintic
from yopo_swarm_core.protocol.plan_occupancy import PlanOccupancy
from yopo_swarm_core.trajectory import PiecewisePolynomial

CROSS = ROOT / "scenes/crossing-pillars.json"


def _fleet(params=None):
    card = dict(load_card(CROSS).card)
    if params is not None:
        card["policy_params"] = params
    return crossing.CrossingFleet.from_card(card, 0.01)


class _Line:
    def __init__(self, a, b, t0=0.0, t1=1.0):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        self.t_start, self.t_end = t0, t1

    def evaluate(self, t, order=0):
        u = (t - self.t_start) / (self.t_end - self.t_start)
        return self.a + u * (self.b - self.a)


def _line(a, b):
    return _Line(a, b)


# 柱 p_c0_r1 占 x 3.7–4.3、y −0.4–0.4；body_radius=0.35，v_max=20 时采样保守量 0.20 m：
# NEAR (y=0.9): raw=0.15, gap=-0.05
# MID  (y=1.1): raw=0.35, gap=0.15
# FAR  (y=1.2): raw=0.45, gap=0.25
# CLEAR(y=2.0): raw=1.25, gap=1.05
NEAR = _line([2.0, 0.9, 1.5], [6.0, 0.9, 1.5])
MID = _line([2.0, 1.1, 1.5], [6.0, 1.1, 1.5])
FAR = _line([2.0, 1.2, 1.5], [6.0, 1.2, 1.5])
CLEAR = _line([2.0, 2.0, 1.5], [6.0, 2.0, 1.5])


def test_default_off_and_b7_compat_invariant():
    off = _fleet()
    b7 = _fleet({"b3": {"pillar_clearance_m": 0.5}})
    b8 = _fleet({"b3": {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": 0.2}})

    assert off.pillar_clearance is None
    assert b7.pillar_clearance.fallback_min_m is None
    assert "fallback_min_m" not in b7.pillar_clearance.summary()
    assert "floor_blocked" not in b7.pillar_clearance.summary()

    assert b8.pillar_clearance.fallback_min_m == 0.2
    assert b8.cfg.task_hash != off.cfg.task_hash
    assert b8.cfg.task_hash != b7.cfg.task_hash


def test_validation_errors_and_fix_parsing():
    # 没有 pillar_clearance_m 不能单开 pillar_fallback_min_m
    with pytest.raises(ValueError):
        _fleet({"b3": {"pillar_fallback_min_m": 0.2}})
    # 非正、非有限、布尔值都拒绝
    for bad in (0.0, -0.1, float("nan"), float("inf"), True, False):
        with pytest.raises(ValueError):
            _fleet({"b3": {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": bad}})
    # 地板不能高于柱门本身
    with pytest.raises(ValueError):
        _fleet({"b3": {"pillar_clearance_m": 0.3, "pillar_fallback_min_m": 0.5}})
    # 地板等于柱门合法（等价于 B6 硬门：全拒时不交任何低于门的候选）
    hard = _fleet({"b3": {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": 0.5}})
    assert hard.pillar_clearance.fallback_min_m == 0.5

    # YOPO_B2_FIX 支持 floor=<m> 与 pillar=<m>:<floor>，以及与 launch / reframp / ramp 组合
    fix1 = parse_fix("launch=3:1.5,reframp=3,pillar=0.5,floor=0.3")
    assert b3_from_fix(fix1) == {
        "launch_a_max_mps2": 3.0,
        "launch_s": 1.5,
        "ref_ramp_mps2": 3.0,
        "pillar_clearance_m": 0.5,
        "pillar_fallback_min_m": 0.3,
    }
    devs1 = fix_deviations(fix1)
    assert any("柱净空全拒退路地板 0.3 m" in d and d.startswith("偏离上游") for d in devs1)

    fix2 = parse_fix("floor=0.2,pillar=0.5,ramp=3,launch=3:1.5")
    assert b3_from_fix(fix2)["pillar_fallback_min_m"] == 0.2
    assert b3_from_fix(fix2)["center_ramp_mps2"] == 3.0

    fix3 = parse_fix("pillar=0.5:0.25")
    assert b3_from_fix(fix3) == {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": 0.25}

    for bad_fix in ("floor=0.2", "pillar=0.5,floor=0", "pillar=0.3,floor=0.5", "pillar=0.3:0.5", "ramp=3,reframp=3"):
        with pytest.raises(ValueError):
            parse_fix(bad_fix)


def test_floor_allows_above_floor_and_blocks_below_floor():
    fleet = _fleet({"b3": {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": 0.2}})
    gate = fleet.pillar_clearance
    speed = fleet.pcfg.limits.v_max
    assert gate.gap(NEAR, speed) < gate.gap(MID, speed) < 0.2 < gate.gap(FAR, speed) < 0.5

    picked = []

    def fake_as_plan(node, t_start, head, motion):
        picked.append(motion)
        return object(), None

    fleet._as_plan = fake_as_plan

    # 全拒但最远候选 FAR (gap=0.25 >= 0.20) 过地板：允许退回 FAR
    occ, reject, used_alt, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, MID, FAR])
    assert occ is not None and used_fallback is True and used_alt is False
    assert picked == [FAR]
    assert gate.fallbacks == 1 and gate.floor_blocked == 0

    # 全拒且最远候选 MID (gap=0.15 < 0.20) 低于地板：不交 PILLAR_FALLBACK，返回空由上层刹车/保持
    picked.clear()
    occ, reject, used_alt, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, MID])
    assert occ is None and reject == "CLEARANCE_SKIP" and used_fallback is False
    assert picked == []
    assert gate.fallbacks == 1 and gate.floor_blocked == 1
    s = gate.summary()
    assert s["fallback_min_m"] == 0.2 and s["floor_blocked"] == 1 and s["fallbacks"] == 1
    assert "地板 0.2 m" in s["deviation"] and s["deviation"].startswith("偏离上游")

    # 有一条过门候选 CLEAR 时，直接取 CLEAR，不计 fallback 也不计 floor_blocked
    picked.clear()
    occ, reject, used_alt, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, CLEAR, FAR])
    assert occ is not None and used_alt is True and used_fallback is False
    assert picked == [CLEAR]
    assert gate.fallbacks == 1 and gate.floor_blocked == 1


def test_planner_falls_through_to_brake_or_hold_when_floor_blocks():
    fleet = _fleet({"b3": {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": 0.2}})
    did = "uav0"
    node = fleet.nodes[did]
    depth = np.full((96, 160), 10.0, dtype=np.float32)
    fleet._sense[did] = (DepthFrame(0.0, depth, (80.0, 80.0, 80.0, 48.0)), 0.0)
    fleet._yopo_options = lambda *args, **kwargs: ([NEAR, MID], "OK")

    # 1) 静止悬停起点（v=0）：_brake_motion 为 None，地板挡住贴柱退路后保持原计划（CLEARANCE_SKIP）
    occ_hold = fleet._planner(node, 0.0, 0.25)
    assert occ_hold is None
    assert fleet.code(did) == "CLEARANCE_SKIP"
    assert "PILLAR_FALLBACK" not in fleet._codes[did]
    assert fleet.pillar_clearance.floor_blocked == 1
    assert fleet.pillar_clearance.fallbacks == 0

    # 2) 运动中（x=0 -> x=1，距柱 x=3.7 尚远）：地板挡住贴柱退路后落入运动学刹车 BRAKE_ONLY
    start_pva = np.array([[0.0, 0.0, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    mid_pva = np.array([[0.6, 0.0, 1.5], [1.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
    stop_pva = np.array([[1.2, 0.0, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    seg1 = quintic(start_pva, mid_pva, 0.8)
    seg2 = quintic(mid_pva, stop_pva, 0.8)
    moving_traj = PiecewisePolynomial(0.0, [seg1, seg2])
    moving_plan = make_plan(
        did,
        1,
        node.committed.plan_hash,
        moving_traj,
        0.0,
        start_pva,
        [0.0],
        fleet.pcfg,
    )
    assert not isinstance(moving_plan, tuple)
    moving_occ = PlanOccupancy(moving_plan, fleet.pcfg)
    node.committed = replace(
        node.committed,
        plan_seq=1,
        plan_hash=moving_plan.plan_hash(fleet.pcfg),
        occupancy=moving_occ,
    )

    occ_brake = fleet._planner(node, 0.4, 0.5)
    assert occ_brake is not None
    assert fleet.code(did) == "BRAKE_ONLY"
    assert "PILLAR_FALLBACK" not in fleet._codes[did]
    assert fleet.pillar_clearance.floor_blocked == 2
    assert fleet.pillar_clearance.fallbacks == 0
    end_v = occ_brake.plan.motion.evaluate(occ_brake.plan.motion.t_end, 1)
    assert float(np.linalg.norm(end_v)) <= 1e-6


def test_short_l0_combo_flight_marks_deviations():
    from yopo_l0.runner import RunConfig, run

    base = load_card(CROSS)
    # 缺省运行：无偏离，无 pillar_clearance 字段
    out_off = run(base, RunConfig(policy="crossing", seed=0, shadow=False, duration_s=0.5)).result
    assert out_off["upstream_deviation"] is False
    assert "pillar_clearance" not in out_off["crossing"]

    # 组合运行：(a) + 参考斜坡 + 柱门 0.5 m + 退路地板 0.3 m
    card, devs = apply_b2_overrides(
        base,
        b3={
            "launch_a_max_mps2": 3.0,
            "launch_s": 1.5,
            "ref_ramp_mps2": 3.0,
            "pillar_clearance_m": 0.5,
            "pillar_fallback_min_m": 0.3,
        },
    )
    out_on = run(card, RunConfig(policy="crossing", seed=0, shadow=False, duration_s=0.5)).result
    assert out_on["upstream_deviation"] is True
    pc = out_on["crossing"]["pillar_clearance"]
    assert pc["margin_m"] == 0.5
    assert pc["fallback_min_m"] == 0.3
    assert pc["checks"] > 0
    assert pc["floor_blocked"] == 0
    assert "地板 0.3 m" in out_on["pillar_clearance_deviation"]
    assert "规划时刻表不变" in out_on["center_ramp_deviation"]


def test_same_host_explicit_none_matches_default_bit_for_bit():
    from yopo_l0.runner import WALL_FIELDS, RunConfig, run

    base = load_card(CROSS)
    explicit_off = replace(
        base,
        card={**base.card, "policy_params": {"b3": {"pillar_fallback_min_m": None}}},
    )
    a = run(base, RunConfig(policy="crossing", seed=0, shadow=True, duration_s=0.5))
    b = run(explicit_off, RunConfig(policy="crossing", seed=0, shadow=True, duration_s=0.5))
    drop = set(WALL_FIELDS) | {"commit", "scene_card_hash"}
    assert {k: v for k, v in a.result.items() if k not in drop} == {
        k: v for k, v in b.result.items() if k not in drop
    }
    assert a.ticks == b.ticks


def test_forest_card_rejects_pillar_clearance_and_floor():
    forest_card = dict(load_card(ROOT / "scenes/crossing-forest.json").card)
    forest_card["policy_params"] = {"b3": {"pillar_clearance_m": 0.5, "pillar_fallback_min_m": 0.3}}
    with pytest.raises(ValueError, match="p_"):
        crossing.CrossingFleet.from_card(forest_card, 0.01)

