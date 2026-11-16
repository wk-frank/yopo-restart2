"""票 S8：三机林带多圆柱 SDF 世界生成与六机 PX4 SITL 锁步抽检入口单元测试（不起 Gazebo）。"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core import px4

from conftest import ROOT
from yopo_l0.scene import Box, Cylinder, load_card
from yopo_l0.sitl_bridge import (
    CORRIDOR_SHELL_NAMES,
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    box_sdf_geometry,
    build_scene_world_sdf,
    cylinder_sdf_geometry,
    frame_note,
    live_topic_plan,
    make_live_session,
    odometry_message_from_odometry,
    sitl_line_spawn_xy,
    vehicle_identity,
    vehicle_topics,
    write_scene_world_sdf,
)
from yopo_l0.sitl_lockstep import LockstepStepper, fleet_handshake_topics, motor_topic

SITL = ROOT / "scripts/sitl"
DT = 0.02

FOREST_BASE = ROOT / "scenes/crossing-forest.json"
FOREST_SITL = ROOT / "scenes/crossing-forest-sitl80.json"
F6_BASE = ROOT / "scenes/formation-6-open.json"
F6_SITL = ROOT / "scenes/formation-6-open-sitl80.json"
C6_FOREST_BASE = ROOT / "scenes/crossing-6-forest.json"
C6_FOREST_SITL = ROOT / "scenes/crossing-6-forest-sitl80.json"
C6_OPEN_SITL = ROOT / "scenes/crossing-6-open-sitl80.json"


def _load_script_mod(mod_name: str, filename: str):
    spec = importlib.util.spec_from_file_location(mod_name, SITL / filename)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod


fleet = _load_script_mod("sitl_s8_fleet", "fleet.py")
world_sdf = _load_script_mod("sitl_s8_world_sdf", "world_sdf.py")
check_scenes = importlib.util.spec_from_file_location("sitl_s8_check_scenes", ROOT / "scripts/check_scenes.py")
assert check_scenes is not None and check_scenes.loader is not None
check_scenes_mod = importlib.util.module_from_spec(check_scenes)
check_scenes.loader.exec_module(check_scenes_mod)


def _rot_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _inbox(session, vehicles: list[FakePx4Vehicle], t: float) -> dict[str, VehicleInbox]:
    stamp_us = int(round(t * 1e6))
    return {
        slot.ident["namespace"]: VehicleInbox(
            odometry=odometry_message_from_odometry(veh.odometry(), stamp_us),
            armed=True,
            offboard=True,
            landed=False,
        )
        for slot, veh in zip(session.slots, vehicles)
    }


# ---------------------------------------------------------------------------
# 1. 六机布局、话题、端口、生成点与 1..3 回归防护
# ---------------------------------------------------------------------------


def test_six_vehicle_layout_topics_ports_and_one_domain():
    specs = fleet.fleet_specs(6)
    assert len(specs) == 6
    assert [s.index for s in specs] == [1, 2, 3, 4, 5, 6]
    assert [s.instance for s in specs] == [1, 2, 3, 4, 5, 6]
    assert [s.mav_sys_id for s in specs] == [2, 3, 4, 5, 6, 7]
    assert [s.namespace for s in specs] == ["px4_1", "px4_2", "px4_3", "px4_4", "px4_5", "px4_6"]
    assert [s.model for s in specs] == ["x500_1", "x500_2", "x500_3", "x500_4", "x500_5", "x500_6"]
    assert [s.gcs_port for s in specs] == [18571, 18572, 18573, 18574, 18575, 18576]
    assert [s.standalone for s in specs] == [False, True, True, True, True, True]
    assert [s.pose for s in specs] == ["0,0", "0,3", "0,6", "0,9", "0,12", "0,15"]
    assert [tuple(s.spawn_xy_m) for s in specs] == sitl_line_spawn_xy(6)

    for mode in ("hover", "goto"):
        lay = fleet.layout_dict(6, mode)
        assert lay["n"] == 6
        assert lay["dds_domain_id"] == 0
        assert lay["xrce_port"] == px4.XRCE_DEFAULT_PORT
        assert lay["report_only"] is True
        assert lay["criteria_frozen"] is False
        assert lay["min_geometric_separation_m"] == pytest.approx(3.0)

    with pytest.raises(ValueError, match="swap"):
        fleet.vehicle_plans(6, "swap")

    shell = fleet.format_shell(6)
    assert shell.splitlines()[0] == "n=6"
    assert shell.count("---") == 6
    assert "instance=6" in shell and "gcs_port=18576" in shell


def test_allowed_n_preserves_1_to_3_and_rejects_unsupported_counts():
    assert fleet.fleet_specs(1)[0].instance == 0
    assert [s.instance for s in fleet.fleet_specs(2)] == [1, 2]
    assert [s.instance for s in fleet.fleet_specs(3)] == [1, 2, 3]
    for bad in (-1, 0, 4, 5, 7, True, False):
        with pytest.raises(ValueError):
            fleet.fleet_specs(bad)  # type: ignore[arg-type]


def test_six_vehicle_yopo_poses_rewrite_and_mismatch_error(tmp_path: Path):
    layout_file = tmp_path / "layout6.txt"
    layout_file.write_text(fleet.format_shell(6), encoding="utf-8")
    for card_path in (F6_SITL, C6_OPEN_SITL):
        proc = subprocess.run(
            [
                sys.executable,
                str(SITL / "fleet.py"),
                "yopo-poses",
                "--scene",
                str(card_path),
                "--n",
                "6",
                "--layout",
                str(layout_file),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0, proc.stderr
        poses = [line.split("=", 1)[1] for line in proc.stdout.splitlines() if line.startswith("pose=")]
        raw = json.loads(card_path.read_text(encoding="utf-8"))
        expected = [fleet.pose_xy(d["start_enu"][0], d["start_enu"][1]) for d in raw["drones"]]
        assert poses == expected

    mismatch = subprocess.run(
        [
            sys.executable,
            str(SITL / "fleet.py"),
            "yopo-poses",
            "--scene",
            str(F6_SITL),
            "--n",
            "3",
            "--layout",
            str(layout_file),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert mismatch.returncode == 2


def test_six_vehicle_bridge_identity_topics_and_lockstep_handshake():
    card = load_card(F6_SITL)
    assert "场景卡起点与 S2 直线生成点不一致" in frame_note(card)
    plan = live_topic_plan(card, policy="formation")
    assert plan["n_drones"] == 6
    assert plan["namespaces"] == [f"px4_{i}" for i in range(1, 7)]
    assert plan["mav_sys_ids"] == [2, 3, 4, 5, 6, 7]
    assert plan["gcs_ports"] == [18571, 18572, 18573, 18574, 18575, 18576]
    assert plan["px4_parameters_written"] == []
    assert plan["inter_drone_transport"] == "inprocess"
    for i in range(6):
        ident = vehicle_identity(i, 6)
        ns = f"px4_{i + 1}"
        assert ident == {"namespace": ns, "mav_sys_id": i + 2, "gcs_port": 18571 + i, "instance": i + 1}
        topics = vehicle_topics(ns)
        assert topics["vehicle_odometry"] == f"/{ns}/fmu/out/vehicle_odometry"
        assert topics["trajectory_setpoint"] == f"/{ns}/fmu/in/trajectory_setpoint"

    assert fleet_handshake_topics(1) == [motor_topic(0)]
    assert fleet_handshake_topics(6) == [f"/x500_{i}/command/motor_speed" for i in range(1, 7)]

    # 假世界测试 6 机锁步握手：最慢的一架（px4_5）晚两轮才给电机计数，若卡死则点名超时。
    class FakeSixWorld:
        def __init__(self) -> None:
            self.t = 10.0
            self.counts = {f"px4_{i}": 5 for i in range(1, 7)}
            self.stall_ns: str | None = None
            self.polls = 0

        def sim_time(self) -> float:
            return self.t

        def motor_counts(self) -> dict[str, int]:
            self.polls += 1
            out = dict(self.counts)
            if self.stall_ns is None and self.polls < 3:
                out["px4_5"] -= 1
            return out

        def request_pause(self) -> None:
            pass

        def request_step(self) -> None:
            self.t = round(self.t + 0.004, 9)
            self.polls = 0
            for k in self.counts:
                if k != self.stall_ns:
                    self.counts[k] += 1

        def request_resume(self) -> None:
            pass

    clock = [0.0]

    def fake_sleep(dt: float) -> None:
        clock[0] += dt

    world = FakeSixWorld()
    stepper = LockstepStepper(
        world,
        [f"px4_{i}" for i in range(1, 7)],
        handshake_timeout_s=0.2,
        now=lambda: clock[0],
        sleep=fake_sleep,
    )
    assert stepper.engage() == pytest.approx(10.0)
    assert stepper.advance(0.02) == pytest.approx(10.02)
    assert stepper.stats.steps == 5
    world.stall_ns = "px4_5"
    with pytest.raises(TimeoutError, match="px4_5"):
        stepper.advance(0.004)
    assert stepper.stats.handshake_timeouts == {"px4_5": 1}


@pytest.mark.parametrize(
    "card_path,policy",
    [
        (F6_SITL, "formation"),
        (C6_OPEN_SITL, "crossing"),
    ],
)
def test_six_drone_fake_px4_live_session_reaches_goal_without_collisions(card_path: Path, policy: str):
    card = load_card(card_path)
    session = make_live_session(
        card,
        seed=0,
        policy=policy,
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", plan_timeout_s=60.0),
    )
    vehicles = [FakePx4Vehicle(np.array([0.0, 0.0, float(slot.bridge.start_enu[2])])) for slot in session.slots]
    t = 0.0
    while t <= 35.0 and session.result()["outcome"] == "RUNNING":
        cmds = session.step(t, _inbox(session, vehicles, t))
        for veh, cmd in zip(vehicles, cmds):
            if cmd.setpoint is not None:
                veh.step(cmd.setpoint, DT)
        t = round(t + DT, 10)
    res = session.result()
    assert session.fail_reason == ""
    assert res["arrived"] == 6
    assert res["collisions"] == 0
    assert res["px4_parameters_written"] == []
    assert res["min_inter_drone_distance_plan_m"] > 1.5


# ---------------------------------------------------------------------------
# 2. 林带多圆柱（竖直 + 倾斜）与矮枝 SDF 生成校验
# ---------------------------------------------------------------------------


def test_cylinder_sdf_geometry_matches_upright_and_leaning_axis_endpoints():
    upright = Cylinder("p0", (12.0, -3.5), 0.32, 0.0, 4.8, (0.0, 0.0))
    ug = cylinder_sdf_geometry(upright)
    assert ug["upright"] is True
    assert ug["pose"] == pytest.approx((12.0, -3.5, 2.4, 0.0, 0.0, 0.0))
    assert ug["radius"] == pytest.approx(0.32)
    assert ug["length"] == pytest.approx(4.8)

    leaning = Cylinder("t007", (18.4, 2.1), 0.25, 0.0, 5.1, (0.85, -0.62))
    lg = cylinder_sdf_geometry(leaning)
    assert lg["upright"] is False
    cx, cy, cz, roll, pitch, yaw = lg["pose"]
    mid = np.array([cx, cy, cz])
    rot = _rot_zyx(roll, pitch, yaw)
    half = 0.5 * lg["length"]
    p_base = mid + rot @ np.array([0.0, 0.0, -half])
    p_top = mid + rot @ np.array([0.0, 0.0, half])
    exp_base, exp_top = leaning.axis_ends()
    assert np.allclose(p_base, exp_base, atol=1e-12)
    assert np.allclose(p_top, exp_top, atol=1e-12)

    box = Box("t007-br", (17.5, 1.9, 1.22), (18.4, 2.06, 1.38))
    bg = box_sdf_geometry(box)
    assert bg["pose"] == pytest.approx((17.95, 1.98, 1.30, 0.0, 0.0, 0.0))
    assert bg["size"] == pytest.approx((0.9, 0.16, 0.16))


def test_forest_world_sdf_contains_all_trunks_and_branches_and_excludes_corridor_shell(tmp_path: Path):
    card = load_card(FOREST_SITL)
    expected_cyls = [o for o in card.obstacles if isinstance(o, Cylinder)]
    expected_branches = [o for o in card.obstacles if isinstance(o, Box) and o.name not in CORRIDOR_SHELL_NAMES]
    assert len(expected_cyls) == 26
    assert any(not c.upright() for c in expected_cyls)
    assert len(expected_branches) >= 1

    sdf_a, sum_a = build_scene_world_sdf(card)
    sdf_b, sum_b = build_scene_world_sdf(card)
    assert sdf_a == sdf_b
    assert sum_a == sum_b
    assert sum_a["n_cylinders"] == len(expected_cyls)
    assert sum_a["n_leaning_cylinders"] == sum(1 for c in expected_cyls if not c.upright())
    assert sum_a["n_boxes"] == len(expected_branches)
    assert sum_a["px4_parameters_written"] == []

    root = ET.fromstring(sdf_a)
    world_el = root.find("world")
    assert world_el is not None and world_el.attrib["name"] == "default"
    model_names = [m.attrib["name"] for m in world_el.findall("model")]
    assert len(model_names) == len(expected_cyls) + len(expected_branches)
    for shell_name in CORRIDOR_SHELL_NAMES:
        assert f"yopo_obs_{shell_name}" not in model_names
    for m in world_el.findall("model"):
        assert m.findtext("static") == "true"

    # 测试带基础目录同级文件的写出与 world_sdf.py CLI。
    base_dir = tmp_path / "base_worlds"
    base_dir.mkdir()
    base_sdf = base_dir / "default.sdf"
    sibling_sdf = base_dir / "windy.sdf"
    base_sdf.write_text(
        '<sdf version="1.9"><world name="default"><light type="directional" name="sun"/></world></sdf>\n',
        encoding="utf-8",
    )
    sibling_sdf.write_text('<sdf version="1.9"><world name="windy"/></sdf>\n', encoding="utf-8")

    out_dir = tmp_path / "out_worlds"
    summary_file = tmp_path / "world_sdf.json"
    rc = world_sdf.main(
        [
            "--scene",
            str(FOREST_SITL),
            "--base-sdf",
            str(base_sdf),
            "--out-dir",
            str(out_dir),
            "--summary",
            str(summary_file),
        ]
    )
    assert rc == 0
    written_summary = json.loads(summary_file.read_text(encoding="utf-8"))
    assert written_summary["n_cylinders"] == len(expected_cyls)
    assert (out_dir / "default.sdf").is_file()
    assert (out_dir / "windy.sdf").is_symlink()
    parsed_out = ET.fromstring((out_dir / "default.sdf").read_text(encoding="utf-8"))
    assert parsed_out.find("world/light") is not None

    no_br_dir = tmp_path / "no_branches"
    assert world_sdf.main(["--scene", str(FOREST_SITL), "--out-dir", str(no_br_dir), "--no-branches"]) == 0
    no_br_root = ET.fromstring((no_br_dir / "default.sdf").read_text(encoding="utf-8"))
    assert len(no_br_root.findall("world/model")) == len(expected_cyls)

    assert world_sdf.main(["--scene", "scenes/no-such-card.json"]) == 2
    assert world_sdf.main(["--scene", str(FOREST_SITL), "--world", "mismatch"]) == 2


# ---------------------------------------------------------------------------
# 3. CLI 参数校验、场景卡合规性与无飞控参数写入契约
# ---------------------------------------------------------------------------


def test_shell_and_live_cli_validation_for_n6_and_scene_world(tmp_path: Path):
    env = dict(os.environ)
    env.pop("YOPO_SITL_LOCKSTEP", None)
    env.pop("YOPO_SITL_SCENE_WORLD", None)

    dry6 = subprocess.run(
        ["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--dry-run", "--n", "6", "--policy", "formation"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert dry6.returncode == 0, dry6.stderr
    assert "n=6" in dry6.stdout
    assert "scene=scenes/formation-6-open-sitl80.json" in dry6.stdout
    assert "scene_world=0" in dry6.stdout
    assert "lockstep=0" in dry6.stdout

    dry_forest = subprocess.run(
        [
            "bash",
            str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run",
            "--n",
            "3",
            "--scene",
            "scenes/crossing-forest-sitl80.json",
            "--policy",
            "crossing",
            "--lockstep",
            "--scene-world",
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert dry_forest.returncode == 0, dry_forest.stderr
    assert "lockstep=1" in dry_forest.stdout
    assert "scene_world=1" in dry_forest.stdout

    for bad_args, extra_env in [
        (["--dry-run", "--n", "4"], {}),
        (["--dry-run", "--n", "5"], {}),
        (["--dry-run", "--n", "7"], {}),
        (["--dry-run", "--n", "3"], {"YOPO_SITL_SCENE_WORLD": "2"}),
    ]:
        e = {**env, **extra_env}
        bad = subprocess.run(
            ["bash", str(SITL / "run_yopo_l0_sitl.sh"), *bad_args],
            cwd=ROOT,
            env=e,
            capture_output=True,
            text=True,
            check=False,
        )
        assert bad.returncode == 2, (bad_args, extra_env, bad.stdout, bad.stderr)

    # yopo_l0_live.py --dry-run 校验 6 机锁步话题与林带世界摘要。
    out_f6 = tmp_path / "f6_plan.json"
    live_env = {**env, "PYTHONPATH": f"{ROOT}:{ROOT / 'core/src'}"}
    r6 = subprocess.run(
        [
            sys.executable,
            str(SITL / "yopo_l0_live.py"),
            "--dry-run",
            "--n",
            "6",
            "--scene",
            str(F6_SITL),
            "--policy",
            "formation",
            "--lockstep",
            "--out",
            str(out_f6),
        ],
        cwd=ROOT,
        env=live_env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert r6.returncode == 0, r6.stderr
    p6 = json.loads(out_f6.read_text(encoding="utf-8"))
    assert p6["n_drones"] == 6
    assert p6["lockstep"]["handshake_topics"] == [f"/x500_{i}/command/motor_speed" for i in range(1, 7)]
    assert p6["scene_world"] == {"enabled": False}
    assert p6["px4_parameters_written"] == []

    out_forest = tmp_path / "forest_plan.json"
    rf = subprocess.run(
        [
            sys.executable,
            str(SITL / "yopo_l0_live.py"),
            "--dry-run",
            "--n",
            "3",
            "--scene",
            str(FOREST_SITL),
            "--policy",
            "crossing",
            "--lockstep",
            "--scene-world",
            "--out",
            str(out_forest),
        ],
        cwd=ROOT,
        env=live_env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert rf.returncode == 0, rf.stderr
    pf = json.loads(out_forest.read_text(encoding="utf-8"))
    assert pf["scene_world"]["enabled"] is True
    assert pf["scene_world"]["n_cylinders"] == 26
    assert pf["scene_world"]["px4_parameters_written"] == []


@pytest.mark.parametrize(
    "base_path,sitl_path,base_dur",
    [
        (FOREST_BASE, FOREST_SITL, 50.0),
        (F6_BASE, F6_SITL, 40.0),
        (C6_FOREST_BASE, C6_FOREST_SITL, 50.0),
    ],
)
def test_s8_sitl80_cards_match_base_cards_and_pass_check_scenes(
    base_path: Path, sitl_path: Path, base_dur: float
):
    base = json.loads(base_path.read_text(encoding="utf-8"))
    sitl = json.loads(sitl_path.read_text(encoding="utf-8"))
    assert check_scenes_mod.validate_card(sitl) == []
    assert sitl["name"] == base["name"] + "-sitl80"
    assert base["task"]["duration_s"] == base_dur
    assert sitl["task"]["duration_s"] == 80.0
    note = sitl.pop("note")
    assert "duration_s" in note and "suite.txt" in note
    base["name"] = sitl["name"]
    base["task"]["duration_s"] = 80.0
    assert sitl == base

    suite_text = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert sitl_path.stem not in suite_text


def test_crossing_6_open_card_and_whole_scenes_directory_pass_strict_check():
    c6_open = json.loads(C6_OPEN_SITL.read_text(encoding="utf-8"))
    assert check_scenes_mod.validate_card(c6_open) == []
    assert c6_open["criteria"]["frozen"] is False
    assert len(c6_open["drones"]) == 6
    assert "crossing-6-open-sitl80" not in (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert check_scenes_mod.check_directory(ROOT / "scenes") == []


def test_s8_files_write_no_px4_parameters():
    for rel in (
        "scripts/sitl/world_sdf.py",
        "scripts/sitl/yopo_l0_live.py",
        "yopo_l0/sitl_bridge.py",
        "yopo_l0/sitl_lockstep.py",
    ):
        text = (ROOT / rel).read_text(encoding="utf-8")
        assert "param set" not in text, rel
        assert "PX4_PARAM_" not in text, rel



def test_run_script_passes_scene_world_and_4g_shm_to_docker_run(tmp_path: Path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log_file = tmp_path / "docker.log"
    fake_docker = bin_dir / "docker"
    fake_docker.write_text(
        f"""#!/usr/bin/env bash
printf '%s\n' "$*" >> "{log_file}"
case "$1" in
  info) exit 0 ;;
  image) exit 0 ;;
  ps) exit 0 ;;
  rm) exit 0 ;;
  network) exit 0 ;;
  run) exit 0 ;;
esac
exit 0
""",
        encoding="utf-8",
    )
    fake_docker.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "YOPO_SITL_STATE": str(tmp_path / "state"),
        "YOPO_SITL_DOCKER_NETWORK": "bridge",
    }
    proc = subprocess.run(
        [
            "bash",
            str(SITL / "run_yopo_l0_sitl.sh"),
            "--n",
            "6",
            "--scene",
            "scenes/crossing-6-forest-sitl80.json",
            "--policy",
            "crossing",
            "--lockstep",
            "--scene-world",
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    run_lines = [ln for ln in log_file.read_text(encoding="utf-8").splitlines() if "in_container.sh" in ln]
    assert len(run_lines) == 1
    assert "YOPO_SITL_N=6" in run_lines[0]
    assert "YOPO_SITL_LOCKSTEP=1" in run_lines[0]
    assert "YOPO_SITL_SCENE_WORLD=1" in run_lines[0]
    assert "--shm-size 4g" in run_lines[0]
