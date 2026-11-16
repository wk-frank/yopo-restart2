"""票 10：ROS 2 形态和进程内形态，同 seed 的到达/碰撞结论一致。

rclpy 导入不了时，带 ROS 的用例跳过（原因里写了 scripts/setup_ros2.sh）。
编码往返和锁步钩子不依赖 ROS，始终跑。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.dynamics import State
from yopo_l0.policy import Observation
from yopo_l0.ros2_codec import (
    OBS_CHUNK_BYTES,
    check_drone_id,
    decode_observation_messages,
    encode_observation,
    same_observation,
    sim_time_ns,
)
from yopo_l0.ros2_env import ensure_ros_paths
from yopo_l0.runner import WALL_FIELDS, RunConfig, _due, run
from yopo_l0.scene import load_card
from yopo_l0.sensor import DepthFrame

from conftest import ROOT

OPEN = ROOT / "scenes/open-3drone.json"
PILLAR = ROOT / "scenes/g2-pillar.json"
_XDIST_WORKER = os.environ.get("PYTEST_XDIST_WORKER")
if "ROS_DOMAIN_ID" not in os.environ:
    if _XDIST_WORKER and _XDIST_WORKER.startswith("gw") and _XDIST_WORKER[2:].isdigit():
        os.environ["ROS_DOMAIN_ID"] = str(40 + (int(_XDIST_WORKER[2:]) % 50))
    else:
        os.environ["ROS_DOMAIN_ID"] = str(40 + (os.getpid() % 50))
_COMPARE = ROOT / "out" / (
    f"ros2-compare-{_XDIST_WORKER}.md" if _XDIST_WORKER else "ros2-compare.md"
)
_DROP = set(WALL_FIELDS) | {"form"}
_CONCLUSION = (
    "outcome", "reason", "arrived", "arrival_time_s", "n_drones",
    "min_obstacle_distance_m", "min_inter_drone_distance_m",
    "shadow", "shadow_reject_rate",
)


def _ros_skip_reason() -> str | None:
    try:
        distro = ensure_ros_paths()
    except Exception as exc:
        return f"rclpy 不可导入：{exc}。运行 scripts/setup_ros2.sh"
    if distro is None:
        return (
            "rclpy 不可导入：未安装 ROS 2（/opt/ros 下没有 jazzy 或 humble）。"
            "运行 scripts/setup_ros2.sh"
        )
    try:
        import rclpy  # noqa: F401
    except Exception as exc:
        return f"rclpy 不可导入：{exc}。运行 scripts/setup_ros2.sh"
    return None


_ROS_SKIP = _ros_skip_reason()
needs_ros = pytest.mark.skipif(_ROS_SKIP is not None, reason=_ROS_SKIP or "rclpy 不可导入")


@pytest.fixture(scope="session", autouse=True)
def _fresh_compare_table():
    """本地对照表，给人工看。out/ 被 gitignore，不是测试夹具。"""
    if _COMPARE.exists():
        _COMPARE.unlink()
    yield


def _strip(result: dict) -> dict:
    return {k: v for k, v in result.items() if k not in _DROP}


def _conclusions(result: dict) -> dict:
    return {k: result[k] for k in _CONCLUSION}


def _record(scene: str, policy: str, seed: int, proc: dict, ros: dict, nodes: list[str], clock_ok: bool) -> None:
    _COMPARE.parent.mkdir(parents=True, exist_ok=True)
    if not _COMPARE.exists():
        _COMPARE.write_text(
            "| 场景 | seed | 形态 | outcome | reason | arrived | arrival_time_s | 碰撞 | min_inter_drone_m | formation_error_p95_m | shadow_reject_rate |\n"
            "|---|---:|---|---|---|---:|---|---|---:|---:|---:|\n"
        )
    lines = []
    for form, result in (("in-process", proc), ("ros2", ros)):
        collision = "是" if result["outcome"] == "COLLISION" else "否"
        arrived = result["arrival_time_s"]
        rate = result["shadow_reject_rate"]
        p95 = result.get("formation_error_p95_m")
        lines.append(
            f"| {scene}@{policy} | {seed} | {form} | {result['outcome']} | {result['reason']} "
            f"| {result['arrived']} | {arrived} | {collision} | {result['min_inter_drone_distance_m']!r} "
            f"| {p95!r} | {rate!r} |\n"
        )
    lines.append(f"\n节点 {scene}@{policy}：`{nodes}`，use_sim_time 跟上 L0 tick：{clock_ok}\n\n")
    with _COMPARE.open("a") as f:
        f.writelines(lines)


def test_sim_time_ns_is_exact_for_integer_rates():
    assert sim_time_ns(0, 100) == 0
    assert sim_time_ns(1, 100) == 10_000_000
    assert sim_time_ns(100, 100) == 1_000_000_000
    assert sim_time_ns(3, 30) == 100_000_000


def test_drone_id_namespace_rule():
    assert check_drone_id("uav0") == "uav0"
    assert check_drone_id("alpha") == "alpha"
    with pytest.raises(ValueError, match="命名空间"):
        check_drone_id("uav-0")
    with pytest.raises(ValueError, match="命名空间"):
        check_drone_id("0uav")


def test_observation_roundtrip_is_bitwise():
    rng = np.random.default_rng(0)
    wide = rng.standard_normal((3, OBS_CHUNK_BYTES // 2)).astype(np.float32)
    wide.flat[0] = np.float32(np.nan)
    wide.flat[1] = np.float32(np.inf)
    cases = [
        DepthFrame(0.25, wide, (120.5, 80.25, 40.0, 30.0)),
        DepthFrame(1 / 100, rng.standard_normal((8, 5)).astype(np.float32), (1.0, 2.0, 3.0, 4.0)),
        None,
    ]
    for frame in cases:
        obs = Observation(
            frame,
            State(1 / 3, np.array([0.1, -0.0, 1.5]), np.zeros(3), np.array([1e-16, 0.0, -2.0]), yaw=-0.125),
            np.array([6.0, 0.0, 1.5]),
            now=0.02,
        )
        messages = encode_observation(obs, drone_id="uav0", seq=4)
        if frame is not None and frame.depth.nbytes > OBS_CHUNK_BYTES:
            assert len(messages) > 2
        got = decode_observation_messages(messages)
        assert same_observation(obs, got)
        if frame is not None:
            assert got.frame.depth.tobytes() == np.ascontiguousarray(frame.depth, dtype="<f4").tobytes()


def test_lockstep_bridge_preserves_inprocess_results_and_order():
    """钩子默认关闭。打开时：每个 tick 先 begin_tick，再按卡里的飞机顺序 plan，结果与不挂桥相同。"""
    card = load_card(OPEN)
    cfg = RunConfig(seed=0, policy="hover", duration_s=0.5)
    direct = run(card, cfg)
    events: list = []

    class Bridge:
        def bind(self, drones):
            self.by = {d["id"]: d for d in drones}
            events.append(("bind", [d["id"] for d in drones]))

        def begin_tick(self, k, now):
            events.append(("tick", k, now))

        def plan(self, drone_id, obs):
            events.append(("plan", drone_id, obs.now))
            return self.by[drone_id]["policy"].plan(obs)

    bridged = run(card, cfg, bridge=Bridge())
    assert _strip(direct.result) == _strip(bridged.result)
    assert direct.ticks == bridged.ticks
    assert _conclusions(direct.result) == _conclusions(bridged.result)

    rates = card.card["rates_hz"]
    phys, plan_hz = int(rates["physics"]), int(rates["planner"])
    ids = [d["id"] for d in card.card["drones"]]
    assert events[0] == ("bind", ids)
    index = 1
    k = 0
    while index < len(events):
        kind, tick, now = events[index]
        assert kind == "tick" and tick == k and now == k / phys
        index += 1
        planned = []
        while index < len(events) and events[index][0] == "plan":
            assert events[index][2] == now
            planned.append(events[index][1])
            index += 1
        assert planned == (ids if _due(k, plan_hz, phys) else [])
        k += 1
    assert k == direct.result["ticks"]


def _run_both(scene: Path, policy: str, seed: int = 0, duration: float | None = None):
    from yopo_l0.ros2_run import execute

    card = load_card(scene)
    cfg = RunConfig(seed=seed, policy=policy, duration_s=duration)
    proc = run(card, cfg)
    info: dict = {}
    ros = execute(card, cfg, inspect=info)
    return proc, ros, info


@needs_ros
def test_ros2_namespace_follows_any_scene_card(tmp_path: Path):
    """不认识的 id、任意机数都能建节点。票 06 的编队卡走同一条，不用改 ROS 代码。"""
    from yopo_l0.ros2_run import execute

    raw = json.loads(OPEN.read_text())
    raw["name"] = "two-named"
    raw["drones"] = [
        {"id": "alpha", "start_enu": [0.0, 0.0, 1.5], "goal_enu": [1.0, 0.0, 1.5]},
        {"id": "beta", "start_enu": [0.0, 4.0, 1.5], "goal_enu": [1.0, 4.0, 1.5]},
    ]
    raw["task"] = {"type": "goto", "duration_s": 0.2}
    (tmp_path / "open-area.json").write_text((ROOT / "scenes/open-area.json").read_text())
    path = tmp_path / "two-named.json"
    path.write_text(json.dumps(raw))
    card = load_card(path)
    cfg = RunConfig(seed=0, policy="hover", duration_s=0.2)
    proc = run(card, cfg)
    info: dict = {}
    ros = execute(card, cfg, inspect=info)
    assert info["nodes"] == ["/l0_world", "/alpha/drone", "/beta/drone"]
    assert info["clock_ok"], info["clock_error"]
    assert _conclusions(proc.result) == _conclusions(ros.result)
    assert proc.ticks == ros.ticks
    assert ros.result["form"] == "ros2"
    bad = json.loads(path.read_text())
    bad["drones"][0]["id"] = "not-a-namespace"
    path.write_text(json.dumps(bad))
    with pytest.raises(ValueError, match="命名空间"):
        execute(load_card(path), cfg)


@needs_ros
def test_ros2_cli_writes_same_result_files(tmp_path: Path):
    out = tmp_path / "ros2"
    proc = subprocess.run(
        [
            sys.executable, "-m", "yopo_l0.ros2_run",
            "--scene", str(OPEN), "--seed", "0", "--policy", "hover",
            "--duration", "0.3", "--out", str(out),
        ],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )
    assert proc.returncode in (0, 1), proc.stderr + proc.stdout
    result = json.loads((out / "result.json").read_text())
    lines = [json.loads(line) for line in (out / "ticks.jsonl").read_text().splitlines()]
    assert result["form"] == "ros2"
    assert result["scene"] == "open-3drone" and result["seed"] == 0 and result["n_drones"] == 3
    assert {"k", "t", "uav", "p", "v", "a", "min_obstacle_m", "goal_error_m"} <= set(lines[0])
    inn = run(load_card(OPEN), RunConfig(seed=0, policy="hover", duration_s=0.3))
    assert _conclusions(result) == _conclusions(inn.result)
    assert lines == inn.ticks


@pytest.mark.parametrize(
    ("scene", "policy", "expect_outcome", "expect_arrived"),
    [
        pytest.param(OPEN, "hover", "SUCCESS", 3, id="open-3drone@hover"),
        pytest.param(PILLAR, "upstream", "SUCCESS", 1, marks=pytest.mark.upstream, id="g2-pillar@upstream"),
    ],
)
@needs_ros
def test_ros2_matches_inprocess_arrival_and_collision(scene: Path, policy: str, expect_outcome: str, expect_arrived: int):
    proc, ros, info = _run_both(scene, policy, seed=0)
    assert info["clock_ok"], info["clock_error"]
    assert info["nodes"][0] == "/l0_world"
    drone_ids = [d["id"] for d in json.loads(scene.read_text())["drones"]]
    assert info["nodes"][1:] == [f"/{i}/drone" for i in drone_ids]
    assert _conclusions(proc.result) == _conclusions(ros.result)
    assert proc.result["shadow"] is True
    assert proc.result["shadow_reject_rate"] is not None
    assert proc.result["shadow_reject_rate"] == ros.result["shadow_reject_rate"]
    assert proc.result["outcome"] == expect_outcome
    assert proc.result["arrived"] == expect_arrived
    assert proc.result["outcome"] != "COLLISION"
    assert _strip(proc.result) == _strip(ros.result)
    assert proc.ticks == ros.ticks
    _record(json.loads(scene.read_text())["name"], policy, 0, proc.result, ros.result, info["nodes"], info["clock_ok"])


def test_ros2_not_skipped_when_system_ros2_installed():
    has_ros = any((Path("/opt/ros") / d / "setup.bash").is_file() for d in ("jazzy", "humble"))
    if has_ros:
        assert _ROS_SKIP is None, f"系统已安装 ROS 2 但测试仍会跳过：{_ROS_SKIP}"
