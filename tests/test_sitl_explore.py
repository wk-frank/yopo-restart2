"""explore-sitl：无编队水平横排起飞（policy=explore）与高速多机穿越 PX4 SITL 锁步闭环单元测试。"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0.bus import BusFaults, MessageBus
from yopo_l0.scene import Box, Cylinder, load_card
from yopo_l0.sitl_bridge import (
    CORRIDOR_SHELL_NAMES,
    FLEET_POLICIES,
    LIVE_POLICIES,
    FakePx4Vehicle,
    LiveFleetConfig,
    LiveFleetSession,
    SitlBridgeConfig,
    VehicleInbox,
    YopoSitlBridge,
    apply_explore_velocity_override,
    build_scene_world_sdf,
    live_topic_plan,
    make_live_session,
    odometry_message_from_odometry,
    resolve_live_policy,
    vehicle_identity,
)
from yopo_l0.sitl_lockstep import fleet_handshake_topics

SITL = ROOT / "scripts/sitl"
DT = 0.02

E3_PILLARS = ROOT / "scenes/explore-3-pillars-sitl80.json"
E3_FOREST = ROOT / "scenes/explore-3-forest-sitl80.json"
E6_OPEN = ROOT / "scenes/explore-6-open-sitl80.json"
E6_FOREST = ROOT / "scenes/explore-6-forest-sitl80.json"
EXPLORE_SITL_CARDS = (E3_PILLARS, E3_FOREST, E6_OPEN, E6_FOREST)


def _load_script_mod(mod_name: str, filename: str):
    spec = importlib.util.spec_from_file_location(mod_name, SITL / filename)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod


fleet_mod = _load_script_mod("sitl_explore_fleet", "fleet.py")
check_scenes_spec = importlib.util.spec_from_file_location("sitl_explore_check_scenes", ROOT / "scripts/check_scenes.py")
assert check_scenes_spec is not None and check_scenes_spec.loader is not None
check_scenes_mod = importlib.util.module_from_spec(check_scenes_spec)
check_scenes_spec.loader.exec_module(check_scenes_mod)


def _inbox(
    session: LiveFleetSession,
    vehicles: list[FakePx4Vehicle],
    t: float,
    *,
    armed: bool = True,
    offboard: bool = True,
    landed: bool = False,
) -> dict[str, VehicleInbox]:
    stamp_us = int(round(t * 1e6))
    return {
        slot.ident["namespace"]: VehicleInbox(
            odometry=odometry_message_from_odometry(veh.odometry(), stamp_us),
            armed=armed,
            offboard=offboard,
            landed=landed,
            preflight_ok=True,
        )
        for slot, veh in zip(session.slots, vehicles)
    }


def test_explore_registered_in_live_and_fleet_policies_without_breaking_existing():
    assert LIVE_POLICIES == ("upstream", "formation", "crossing", "switch", "explore")
    assert FLEET_POLICIES == ("formation", "crossing", "switch", "explore")
    for card_path in EXPLORE_SITL_CARDS:
        card = load_card(card_path)
        assert resolve_live_policy(card, "explore") == "explore"
        with pytest.raises(ValueError, match="explore 场景卡需要 policy=explore"):
            resolve_live_policy(card, "upstream")
        with pytest.raises(ValueError, match="policy=crossing 只适用于"):
            resolve_live_policy(card, "crossing")

    g2 = load_card(ROOT / "scenes/g2-pillar.json")
    with pytest.raises(ValueError, match="policy=explore 只适用于"):
        resolve_live_policy(g2, "explore")


def test_explore_sitl80_cards_horizontal_line_spawns_and_strict_compliance():
    suite_text = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    for path in EXPLORE_SITL_CARDS:
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert check_scenes_mod.validate_card(raw) == []
        assert raw["task"]["type"] == "explore"
        assert raw["task"]["duration_s"] == 80.0
        assert raw["criteria"]["frozen"] is False
        assert path.stem not in suite_text

        info = fleet_mod.inspect_scene_spawns(path)
        assert info["horizontal_line"] is True
        assert info["x0_m"] == pytest.approx(0.0)
        assert info["x_span_m"] == pytest.approx(0.0)
        assert info["uniform_lateral_spacing"] is True
        assert info["min_spawn_separation_m"] >= 3.0

    info3 = fleet_mod.inspect_scene_spawns(E3_FOREST)
    assert info3["n"] == 3
    assert info3["y_sorted_m"] == pytest.approx([-8.0, 0.0, 8.0])
    assert info3["lateral_spacing_m"] == pytest.approx(8.0)

    info6 = fleet_mod.inspect_scene_spawns(E6_FOREST)
    assert info6["n"] == 6
    assert info6["y_sorted_m"] == pytest.approx([-7.5, -4.5, -1.5, 1.5, 4.5, 7.5])
    assert info6["lateral_spacing_m"] == pytest.approx(3.0)
    assert fleet_mod.horizontal_line_spawns(6, x_m=0.0, spacing_m=3.0, centered=True) == [
        (0.0, -7.5),
        (0.0, -4.5),
        (0.0, -1.5),
        (0.0, 1.5),
        (0.0, 4.5),
        (0.0, 7.5),
    ]
    assert check_scenes_mod.check_directory(ROOT / "scenes") == []


def test_yopo_poses_rewrites_3_and_6_drone_horizontal_line_spawns(tmp_path: Path):
    for n, card_path, expected_poses in (
        (3, E3_PILLARS, ["0,0", "0,8", "0,-8"]),
        (6, E6_FOREST, ["0,-7.5", "0,-4.5", "0,-1.5", "0,1.5", "0,4.5", "0,7.5"]),
    ):
        layout_file = tmp_path / f"layout_{n}.txt"
        layout_file.write_text(fleet_mod.format_shell(n), encoding="utf-8")
        proc = subprocess.run(
            [
                sys.executable,
                str(SITL / "fleet.py"),
                "yopo-poses",
                "--scene",
                str(card_path),
                "--n",
                str(n),
                "--layout",
                str(layout_file),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0, proc.stderr
        poses = [ln.split("=", 1)[1] for ln in proc.stdout.splitlines() if ln.startswith("pose=")]
        assert poses == expected_poses

    # 拒绝过近生成点（< 0.7 m），避免 Gazebo 地面刚体出生即重叠互撞。
    bad_scene = tmp_path / "overlap.json"
    bad_scene.write_text(
        json.dumps(
            {
                "name": "overlap",
                "drones": [
                    {"id": "uav0", "start_enu": [0.0, 0.0, 1.5]},
                    {"id": "uav1", "start_enu": [0.0, 0.5, 1.5]},
                ],
                "task": {"type": "explore", "duration_s": 10.0},
                "criteria": {"body_radius_m": 0.35},
            }
        ),
        encoding="utf-8",
    )
    lay2 = fleet_mod.format_shell(2)
    with pytest.raises(ValueError, match="最小水平间隔"):
        fleet_mod.apply_scene_spawn_poses(lay2, bad_scene, 2)


def test_velocity_override_scales_explore_card_in_memory_only():
    card = load_card(E6_FOREST)
    orig_v = card.card["task"]["velocity_mps"]
    orig_a = card.card["dynamics"]["a_max"]
    same_card, notes0 = apply_explore_velocity_override(card, None)
    assert same_card is card and notes0 == []

    scaled, notes = apply_explore_velocity_override(card, 9.6)
    assert scaled.card["task"]["velocity_mps"] == pytest.approx(9.6)
    assert scaled.card["dynamics"]["a_max"] > orig_a
    assert "explore.velocity_mps=9.6" in notes
    # 原内存卡与落盘 JSON 保持未改动。
    assert card.card["task"]["velocity_mps"] == pytest.approx(orig_v)
    assert json.loads(E6_FOREST.read_text(encoding="utf-8"))["task"]["velocity_mps"] == pytest.approx(orig_v)

    with pytest.raises(ValueError, match="正有限数"):
        apply_explore_velocity_override(card, 0.0)
    with pytest.raises(ValueError, match="task.type=explore"):
        apply_explore_velocity_override(load_card(ROOT / "scenes/g2-pillar.json"), 7.5)


def test_explore_world_sdf_builds_pillars_and_forest_without_px4_params():
    pillars_card = load_card(E3_PILLARS)
    sdf_p, sum_p = build_scene_world_sdf(pillars_card)
    assert sum_p["n_cylinders"] == 0
    assert sum_p["n_boxes"] == 12
    assert sum_p["px4_parameters_written"] == []
    root_p = ET.fromstring(sdf_p)
    names_p = {m.attrib["name"] for m in root_p.findall("world/model")}
    assert "yopo_obs_p_c0_r0" in names_p
    for shell in CORRIDOR_SHELL_NAMES:
        assert f"yopo_obs_{shell}" not in names_p

    for forest_path in (E3_FOREST, E6_FOREST):
        f_card = load_card(forest_path)
        expected_cyls = [o for o in f_card.obstacles if isinstance(o, Cylinder)]
        expected_br = [o for o in f_card.obstacles if isinstance(o, Box) and o.name not in CORRIDOR_SHELL_NAMES]
        assert len(expected_cyls) == 26
        assert len(expected_br) == 11
        _, sum_f = build_scene_world_sdf(f_card)
        assert sum_f["n_cylinders"] == 26
        assert sum_f["n_boxes"] == 11
        assert sum_f["px4_parameters_written"] == []


def test_run_yopo_l0_sitl_shell_supports_explore_and_velocity_mps(tmp_path: Path):
    env = dict(os.environ)
    env.pop("YOPO_EXPLORE_VELOCITY_MPS", None)
    for n, expected_default in (
        ("3", "scenes/explore-3-forest-sitl80.json"),
        ("6", "scenes/explore-6-forest-sitl80.json"),
    ):
        proc = subprocess.run(
            [
                "bash",
                str(SITL / "run_yopo_l0_sitl.sh"),
                "--dry-run",
                "--n",
                n,
                "--policy",
                "explore",
                "--lockstep",
                "--scene-world",
                "--velocity-mps",
                "8.4",
            ],
            cwd=ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0, proc.stderr
        assert f"scene={expected_default}" in proc.stdout
        assert "policy=explore" in proc.stdout
        assert "lockstep=1" in proc.stdout
        assert "scene_world=1" in proc.stdout
        assert "velocity_mps=8.4" in proc.stdout

    bad = subprocess.run(
        [
            "bash",
            str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run",
            "--n",
            "3",
            "--policy",
            "explore",
            "--velocity-mps",
            "0",
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert bad.returncode == 2


def test_explore_live_session_warmup_takeoff_free_lateral_plan_and_land_with_stub_fleet():
    """用轻量桩验证 WARMUP -> ARMING -> TAKEOFF -> PLAN(自由横向终点锁定) -> LAND -> DONE 全链路。"""

    class _StubSampler:
        def __init__(self, target: np.ndarray, v_x: float) -> None:
            self._target = target.copy()
            self._vx = v_x
            self.yaw = 0.0

        def pva(self, _t: float):
            return self._target.copy(), np.array([self._vx, 0.0, 0.0]), np.zeros(3)

        def sample(self, t: float):
            p, v, a = self.pva(t)
            return p, v, a, self.yaw

    class _StubExploreFleet:
        policy_name = "explore"
        result_key = "explore"
        uses_network = True

        def __init__(self, card_dict: dict) -> None:
            self.members = tuple(d["id"] for d in card_dict["drones"])
            self.starts = {d["id"]: np.asarray(d["start_enu"], dtype=float) for d in card_dict["drones"]}
            self.x_goal = float(card_dict["task"]["x_goal_m"])
            self._pos = {k: v.copy() for k, v in self.starts.items()}
            self._locked: dict[str, np.ndarray | None] = {k: None for k in self.members}
            self.ingest_stamps: list[float] = []

        def terminal_locked(self, did: str) -> bool:
            return self._locked[did] is not None

        def goal_of(self, did: str) -> np.ndarray:
            if self._locked[did] is not None:
                return self._locked[did].copy()
            return np.array([self.x_goal, float(self._pos[did][1]), 1.5])

        def ingest(self, did: str, frame, state) -> None:
            self.ingest_stamps.append(float(frame.stamp))
            self._pos[did] = np.asarray(state.p, dtype=float).copy()
            # 模拟横向自由偏航绕障：各机偏离初始 y 槽位，越过终点线后就地锁定自由 y 悬停点。
            if self._locked[did] is None and float(state.p[0]) >= self.x_goal - 2.0:
                self._locked[did] = np.array([self.x_goal, float(state.p[1]), 1.5])

        def exchange(self, _bus, _now: float) -> None:
            pass

        def observe(self, _did: str, _t: float, _pos) -> float:
            return 0.0

        def track(self, did: str, _now: float):
            if self._locked[did] is not None:
                return _StubSampler(self._locked[did], 0.0)
            p = self._pos[did]
            # 让每机横向自由平移 +2.2 m（远离初始 y），同时高速向 +x 推进。
            y_free = float(self.starts[did][1]) + 2.2
            target = np.array([min(self.x_goal, float(p[0]) + 3.0), y_free, 1.5])
            return _StubSampler(target, 6.5 if float(p[0]) < self.x_goal - 2.0 else 0.0)

        def code(self, _did: str) -> str:
            return "OK"

        def summary(self, arrived: int, n_drones: int) -> dict:
            return {
                "mode": "formation-free-horizontal-explore",
                "error_p95_m": None,
                "arrival_rate": float(arrived) / float(n_drones),
                "final_goals_enu": {k: self.goal_of(k).tolist() for k in self.members},
            }

    card = load_card(E3_PILLARS)
    # 换成空障碍卡以测试纯状态机与自由横向到达 + 着陆闭环
    open_card = load_card(E6_OPEN)
    card3 = type(card)(
        card.path,
        {**card.card, "geometry": open_card.card["geometry"]},
        open_card.obstacles,
    )
    stub_fleet = _StubExploreFleet(card3.card)
    bus = MessageBus(BusFaults())
    drones = card3.card["drones"]
    idents = [vehicle_identity(i, len(drones)) for i in range(len(drones))]
    bridges = [
        YopoSitlBridge(
            card3,
            SitlBridgeConfig(
                physics_dt=DT,
                hover_alt_m=1.5,
                goal_enu=tuple(stub_fleet.goal_of(d["id"])),
                drone_id=d["id"],
                drone_index=i,
                n_drones=len(drones),
                px4_namespace=idents[i]["namespace"],
                load_upstream=False,
            ),
        )
        for i, d in enumerate(drones)
    ]
    cfg = LiveFleetConfig(
        initial_phase="WARMUP",
        warmup_s=0.1,
        settle_hold_s=0.2,
        plan_timeout_s=30.0,
    )
    session = LiveFleetSession(card3, bridges, idents, cfg, fleet=stub_fleet, bus=bus, policy="explore")
    vehicles = [FakePx4Vehicle(np.array([0.0, 0.0, 0.0])) for _ in drones]
    armed = False
    offboard = False
    landed = False
    t = 0.0
    while t <= 25.0 and not session.result()["terminal"]:
        cmds = session.step(t, _inbox(session, vehicles, t, armed=armed, offboard=offboard, landed=landed))
        for veh, cmd in zip(vehicles, cmds):
            for vc in cmd.vehicle_commands:
                if vc.get("command") == 400:
                    armed = True
                elif vc.get("command") == 176:
                    offboard = True
                elif vc.get("command") == 21:
                    landed = True
                    armed = False
            if cmd.setpoint is not None:
                veh.step(cmd.setpoint, DT)
        t = round(t + DT, 10)

    res = session.result()
    assert res["ok"] is True
    assert res["outcome"] == "SUCCESS"
    assert res["arrived"] == 3
    assert res["landed"] is True
    assert res["px4_parameters_written"] == []
    assert res["formation_error_p95_m"] is None
    assert stub_fleet.ingest_stamps[0] == pytest.approx(0.0)
    sitl_meta = res["explore_sitl"]
    assert sitl_meta["horizontal_line_spawn"]["aligned_x"] is True
    assert sitl_meta["takeoff_handover"]["handover_x_span_m"] < 0.05
    assert sitl_meta["belt_avg_speed_mean_mps"] > 5.0
    for d in res["drones"]:
        # 验证自由横向到达：最终悬停 y 已偏离初始槽位 ~+2.2 m，无橡皮筋拉回初始 y！
        orig_y = next(row["start_enu"][1] for row in drones if row["id"] == d["id"])
        assert abs(d["goal_enu"][1] - (orig_y + 2.2)) < 0.3


def test_real_explore_fleet_dry_run_and_fake_px4_closed_loop(tmp_path: Path):
    from yopo_l0.explore import ExploreFleet

    assert ExploreFleet.policy_name == "explore"
    for card_path, expected_n in ((E3_PILLARS, 3), (E6_OPEN, 6)):
        card = load_card(card_path)
        plan = live_topic_plan(card, policy="explore")
        assert plan["policy"] == "explore"
        assert plan["n_drones"] == expected_n
        assert plan["px4_parameters_written"] == []
        assert plan["inter_drone_transport"] == "inprocess"
        assert len(fleet_handshake_topics(expected_n)) == expected_n

    out_plan = tmp_path / "explore6_dry.json"
    env = {**os.environ, "PYTHONPATH": f"{ROOT}:{ROOT / 'core/src'}"}
    proc = subprocess.run(
        [
            sys.executable,
            str(SITL / "yopo_l0_live.py"),
            "--dry-run",
            "--n",
            "6",
            "--scene",
            str(E6_FOREST),
            "--policy",
            "explore",
            "--lockstep",
            "--scene-world",
            "--velocity-mps",
            "8.1",
            "--out",
            str(out_plan),
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    dry = json.loads(out_plan.read_text(encoding="utf-8"))
    assert dry["policy"] == "explore"
    assert dry["n_drones"] == 6
    assert dry["scene_world"]["n_cylinders"] == 26
    assert dry["px4_parameters_written"] == []
    assert "explore.velocity_mps=8.1" in dry["explore_overrides"]

    # 在假 PX4 点质量闭环上跑通 3 机柱林与 6 机开阔横排起飞穿越，0 碰撞到达。
    for card_path, expected_n in ((E3_PILLARS, 3), (E6_OPEN, 6)):
        card = load_card(card_path)
        session = make_live_session(
            card,
            seed=0,
            policy="explore",
            live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", plan_timeout_s=35.0),
        )
        dyn = card.card["dynamics"]
        from yopo_l0.sitl_bridge import FakePx4Config

        fcfg = FakePx4Config(
            tau_s=float(dyn["tau_s"]),
            kp=float(dyn["kp"]),
            kd=float(dyn["kd"]),
            a_max=float(dyn["a_max"]),
        )
        vehicles = [
            FakePx4Vehicle(np.array([0.0, 0.0, float(slot.bridge.start_enu[2])]), cfg=fcfg)
            for slot in session.slots
        ]
        t = 0.0
        while t <= 25.0 and session.result()["outcome"] == "RUNNING":
            cmds = session.step(t, _inbox(session, vehicles, t))
            for veh, cmd in zip(vehicles, cmds):
                if cmd.setpoint is not None:
                    veh.step(cmd.setpoint, DT)
            t = round(t + DT, 10)
        res = session.result()
        assert session.fail_reason == ""
        assert res["arrived"] == expected_n
        assert res["collisions"] == 0
        assert res["px4_parameters_written"] == []
        assert res["min_inter_drone_distance_plan_m"] > 0.70


def test_explore_sitl_sources_write_no_px4_parameters():
    for rel in (
        "scripts/sitl/fleet.py",
        "scripts/sitl/world_sdf.py",
        "scripts/sitl/yopo_l0_live.py",
        "yopo_l0/sitl_bridge.py",
        "yopo_l0/sitl_lockstep.py",
    ):
        text = (ROOT / rel).read_text(encoding="utf-8")
        assert "param set" not in text, rel
        assert "PX4_PARAM_" not in text, rel

