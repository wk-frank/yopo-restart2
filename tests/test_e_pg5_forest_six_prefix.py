"""票 E-PG5：六机林带短前缀按 _forest_six() 作用域隔离，默认关、同机关时与作用域外逐拍一致。"""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

from yopo_l0.crossing import CrossingFleet, _LEADER_PREFIX_S
from yopo_l0.forest import generate_forest_geometry
from yopo_l0.progress import (
    FOREST_SIX_PREFIX_DEFAULT_S,
    LEADER_PREFIX_MAX_S,
    LEADER_PREFIX_MIN_S,
    read_forest_six_prefix,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import SceneCard, load_card, parse_geometry

from conftest import ROOT

SIX = ROOT / "scenes/crossing-6-forest.json"
SPARSE = ROOT / "scenes/crossing-6-forest-sparse.json"
M2_FOREST = ROOT / "scenes/crossing-forest.json"
PILLARS = ROOT / "scenes/crossing-pillars.json"
_STRIP = set(WALL_FIELDS) | {"commit", "scene_card_hash"}


def _script():
    path = ROOT / "scripts/e-pg5-forest-six-prefix.py"
    name = "e_pg5_forest_six_prefix"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _strip(result: dict) -> dict:
    return {k: v for k, v in result.items() if k not in _STRIP}


def _card_with_task(path: Path, **task_patch) -> SceneCard:
    raw = copy.deepcopy(json.loads(path.read_text(encoding="utf-8")))
    raw["task"].update(task_patch)
    if "forest" in raw:
        geom = generate_forest_geometry(raw["forest"])
    else:
        geom = json.loads((path.parent / raw["geometry"]).read_text(encoding="utf-8"))
    merged = {**raw, "geometry": geom}
    return SceneCard(str(path), merged, parse_geometry(geom, merged.get("ground_z")))


def test_read_forest_six_prefix_defaults_and_validation():
    assert FOREST_SIX_PREFIX_DEFAULT_S == pytest.approx(0.7)
    assert read_forest_six_prefix({}) == (False, pytest.approx(0.7))
    assert read_forest_six_prefix({"forest_six_prefix": False, "forest_six_prefix_s": 1.2}) == (
        False,
        pytest.approx(0.7),
    )
    assert read_forest_six_prefix({"forest_six_prefix": True}) == (True, pytest.approx(0.7))
    assert read_forest_six_prefix({"forest_six_prefix": True, "forest_six_prefix_s": 0.5}) == (
        True,
        pytest.approx(0.5),
    )
    assert read_forest_six_prefix({"forest_six_prefix": True, "forest_six_prefix_s": LEADER_PREFIX_MIN_S})[1] == (
        pytest.approx(LEADER_PREFIX_MIN_S)
    )
    assert read_forest_six_prefix({"forest_six_prefix": True, "forest_six_prefix_s": LEADER_PREFIX_MAX_S})[1] == (
        pytest.approx(LEADER_PREFIX_MAX_S)
    )

    base = copy.deepcopy(load_card(SIX).card)
    for raw in (1, 0, "true", "false", None, 0.7):
        with pytest.raises(ValueError, match="forest_six_prefix"):
            read_forest_six_prefix({"forest_six_prefix": raw})
        bad = copy.deepcopy(base)
        bad["task"]["forest_six_prefix"] = raw
        with pytest.raises(ValueError, match="forest_six_prefix"):
            CrossingFleet.from_card(bad, 0.01)

    for raw in ("0.7", True, False, None, 0.19, 2.01, -0.5, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="forest_six_prefix_s"):
            read_forest_six_prefix({"forest_six_prefix": False, "forest_six_prefix_s": raw})
        bad = copy.deepcopy(base)
        bad["task"]["forest_six_prefix"] = False
        bad["task"]["forest_six_prefix_s"] = raw
        with pytest.raises(ValueError, match="forest_six_prefix_s"):
            CrossingFleet.from_card(bad, 0.01)


def test_scene_cards_stay_clean_and_six_drone_forest_activates_only_when_enabled():
    for path, expected_gate in ((SIX, False), (SPARSE, False), (M2_FOREST, True), (PILLARS, False)):
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw["task"]["progress_gate"] is expected_gate
        assert "forest_six_prefix" not in raw["task"]
        assert "forest_six_prefix_s" not in raw["task"]

    plain_card = load_card(SIX)
    explicit_false_card = _card_with_task(SIX, forest_six_prefix=False, forest_six_prefix_s=1.0)
    enabled_card = _card_with_task(SIX, forest_six_prefix=True)
    custom_len_card = _card_with_task(SIX, forest_six_prefix=True, forest_six_prefix_s=0.9)

    f_plain = CrossingFleet.from_card(plain_card.card, 0.01)
    f_false = CrossingFleet.from_card(explicit_false_card.card, 0.01)
    f_on = CrossingFleet.from_card(enabled_card.card, 0.01)
    f_custom = CrossingFleet.from_card(custom_len_card.card, 0.01)

    assert f_plain.forest_six_prefix_enabled is False
    assert f_false.forest_six_prefix_enabled is False
    assert f_false.leader_prefix_enabled is False
    assert f_false.progress_gate is False
    assert f_false.leader_prefix_s == pytest.approx(_LEADER_PREFIX_S)
    assert f_false.cfg.task_hash == f_plain.cfg.task_hash
    assert "forest_six_prefix" not in f_false.summary(0, 6)
    assert "leader_prefix" not in f_false.summary(0, 6)

    assert f_on.forest_six_prefix_enabled is True
    assert f_on.progress_gate is True
    assert f_on.leader_prefix_enabled is True
    assert f_on.leader_prefix_s == pytest.approx(0.7)
    assert f_on.cfg.task_hash != f_plain.cfg.task_hash
    assert f_custom.cfg.task_hash != f_on.cfg.task_hash
    s_on = f_on.summary(0, 6)
    assert s_on["forest_six_prefix"] is True
    assert s_on["leader_prefix"] is True
    assert s_on["leader_prefix_s"] == pytest.approx(0.7)
    assert s_on["progress_gate"] is True


def test_non_forest_six_cards_ignore_switch_and_preserve_trajectories():
    """三机林带、三机柱林与无林六机均不触发 forest_six_prefix，轨迹与基线同机逐位一致。"""
    # 1. 无森林的六机卡也不触发 _forest_six()
    six_pillars = copy.deepcopy(load_card(PILLARS).card)
    six_pillars["drones"] = [
        {"id": f"uav{i}", "slot_body": [0.0, float(i) * 2.0 - 5.0, 0.0]} for i in range(6)
    ]
    f_six_p0 = CrossingFleet.from_card(six_pillars, 0.01)
    six_pillars["task"]["forest_six_prefix"] = True
    f_six_p1 = CrossingFleet.from_card(six_pillars, 0.01)
    assert f_six_p1.forest_six_prefix_enabled is False
    assert f_six_p1.progress_gate is False
    assert f_six_p1.cfg.task_hash == f_six_p0.cfg.task_hash

    # 2. M2 三机穿林（seed 1，即 E-PG4 撞树的那一粒）与三机柱林短跑：同机逐拍一致
    for path, seed in ((M2_FOREST, 1), (PILLARS, 0)):
        base_out = run(load_card(path), RunConfig(seed=seed, policy="crossing", duration_s=0.6, shadow=False))
        scoped_out = run(
            _card_with_task(path, forest_six_prefix=True, forest_six_prefix_s=0.7),
            RunConfig(seed=seed, policy="crossing", duration_s=0.6, shadow=False),
        )
        assert scoped_out.result["upstream_deviation"] is False
        assert "leader_prefix" not in scoped_out.result
        assert "forest_six_prefix" not in scoped_out.result["crossing"]
        assert base_out.ticks == scoped_out.ticks
        assert _strip(base_out.result) == _strip(scoped_out.result)


def test_six_drone_forest_short_run_marks_deviation_and_explicit_false_matches_plain():
    plain_out = run(load_card(SIX), RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False))
    false_out = run(
        _card_with_task(SIX, forest_six_prefix=False, forest_six_prefix_s=0.7),
        RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False),
    )
    assert plain_out.ticks == false_out.ticks
    assert _strip(plain_out.result) == _strip(false_out.result)
    assert false_out.result["upstream_deviation"] is False

    on_out = run(
        _card_with_task(SIX, forest_six_prefix=True, forest_six_prefix_s=0.7),
        RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False),
    )
    assert on_out.result["upstream_deviation"] is True
    assert on_out.result["leader_prefix"] is True
    assert on_out.result["leader_prefix_s"] == pytest.approx(0.7)
    assert on_out.result["crossing"]["forest_six_prefix"] is True
    assert on_out.result["crossing"]["progress_gate"] is True


def test_script_contract_and_relative_p95_helper():
    mod = _script()
    assert mod.main(["--check"]) == 0
    slots = {"uav0": [0.0, -1.0, 0.0], "uav1": [0.0, 1.0, 0.0]}
    # 完美保持相对槽位（即使整体平移到 x=20），相对误差应为 0.0
    ticks = []
    for k in range(25):
        t = round(k * 0.01, 2)
        ticks.append({"t": t, "uav": "uav0", "p": [20.0 + k * 0.1, -1.0, 1.5], "v": [1, 0, 0], "goal_error_m": 5.0})
        ticks.append({"t": t, "uav": "uav1", "p": [20.0 + k * 0.1, 1.0, 1.5], "v": [1, 0, 0], "goal_error_m": 5.0})
    assert mod.relative_formation_p95(ticks, slots) == pytest.approx(0.0, abs=1e-9)
