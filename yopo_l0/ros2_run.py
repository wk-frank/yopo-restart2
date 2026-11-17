"""ROS 2 形态启动器：python -m yopo_l0.ros2_run --scene <卡> --seed N

写出和 `yopo_l0.run` 相同的 result.json / ticks.jsonl。结果多一个 `form: "ros2"`。
参数与进程内启动器一致。编队卡用 `--policy formation`：机间协议消息走各机话题，
不进进程内 MessageBus 队列。节点仍按场景卡里的 drones 建。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from .bus import BusFaults
from .policy import POLICIES
from .run import _normalize_shadow
from .runner import RunConfig, RunOutput, run, write
from .scene import SceneCard, load_card


def _bus(a) -> BusFaults | None:
    """与 yopo_l0.run 相同：任一 --bus-* 出现就覆盖场景卡。未出现的旋钮为 0。"""
    if a.bus_drop is None and a.bus_duplicate is None and a.bus_delay is None and a.bus_reorder is None:
        return None
    if a.bus_delay:
        lo, hi = a.bus_delay.split(",")
        delay = (float(lo), float(hi))
    else:
        delay = (0.0, 0.0)
    return BusFaults(
        drop=0.0 if a.bus_drop is None else a.bus_drop,
        duplicate=0.0 if a.bus_duplicate is None else a.bus_duplicate,
        delay=delay,
        reorder=0.0 if a.bus_reorder is None else a.bus_reorder,
    )


def _import_rclpy():
    from .ros2_env import ensure_ros_paths

    distro = ensure_ros_paths()
    if distro is None:
        raise SystemExit(
            "rclpy 不可导入：未安装 ROS 2。运行 scripts/setup_ros2.sh"
            "（Ubuntu 24.04 装 Jazzy，22.04 装 Humble：ros-*-ros-base + python3-rclpy）。"
        )
    # 发现范围要在 rclpy.init 之前定下来。容器里组播经常被丢掉，锁步只走本机。
    if distro == "humble":
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    else:
        os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")
    os.environ.setdefault("RCUTILS_LOGGING_SEVERITY", "WARN")
    try:
        import rclpy
    except OSError as exc:
        # ld 在进程启动时读 LD_LIBRARY_PATH。setup 脚本会 ldconfig；没跑过时重启自己一次。
        if os.environ.get("_YOPO_L0_ROS_REEXEC") == "1":
            raise SystemExit(f"rclpy 不可导入：{exc}。运行 scripts/setup_ros2.sh") from exc
        os.environ["_YOPO_L0_ROS_REEXEC"] = "1"
        os.execv(sys.executable, [sys.executable, *sys.argv])
    except ImportError as exc:
        raise SystemExit(f"rclpy 不可导入：{exc}。运行 scripts/setup_ros2.sh") from exc
    return rclpy


def execute(card: SceneCard, cfg: RunConfig, *, inspect: dict | None = None) -> RunOutput:
    """建世界节点和每机节点，跑与进程内相同的锁步，返回同一份 RunOutput。

    formation / explore 任务把 transport_override 指到话题传输，exchange 因此不再
    使用进程内队列。运行结束（含异常）后一定清回 None，避免漏到下一次进程内编队。
    """
    rclpy = _import_rclpy()
    from .formation import FormationFleet
    from .ros2_bus import Ros2Transport
    from .ros2_nodes import L0WorldBridge, init_rclpy

    transport = Ros2Transport() if card.card["task"]["type"] == "formation" else None
    bridge = L0WorldBridge(card, transport=transport)
    if transport is not None:
        FormationFleet.transport_override = transport
    try:
        init_rclpy(rclpy)
        out = run(card, cfg, bridge=bridge)
        out.result["form"] = "ros2"
        if inspect is not None:
            inspect["nodes"] = list(bridge.nodes_full_names)
            inspect["clock_ok"] = bridge.clock_ok
            inspect["clock_error"] = bridge.clock_error
            inspect["clock_samples"] = bridge.clock_samples
            inspect["bus_ros_deliveries"] = bridge.bus_ros_deliveries
            inspect["bus_topics"] = list(bridge.bus_topics)
        return out
    finally:
        if transport is not None:
            FormationFleet.transport_override = None
        bridge.close()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="L0 的 ROS 2 形态：每机一个节点，世界节点用 L0 tick 发布 /clock 并驱动锁步",
    )
    ap.add_argument("--scene", required=True, help="场景卡 JSON")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--policy", default="upstream", choices=sorted({*POLICIES, "formation", "explore"}),
        help="hover、upstream、certified；formation / explore 只用于同名 task.type 的场景卡",
    )
    ap.add_argument("--shadow", choices=("on", "off"), default="on",
                    help="影子盾记录：on（默认）或 off。只记录会否拒绝，不改批准动作")
    ap.add_argument("--no-shadow", action="store_true", help="同 --shadow off")
    ap.add_argument("--duration", type=float, default=None, help="仿真秒数，默认取场景卡")
    ap.add_argument("--out", default=None, help="输出目录，默认 out/<scene>/seed<N>")
    ap.add_argument("--bus-drop", type=float, default=None, help="丢包概率，对齐 G3 FaultPlan.drop")
    ap.add_argument("--bus-duplicate", type=float, default=None, help="额外一份拷贝的概率，对齐 FaultPlan.duplicate")
    ap.add_argument("--bus-delay", default=None, help="时延秒数 lo,hi，对齐 FaultPlan.delay，例如 0,0.2")
    ap.add_argument("--bus-reorder", type=float, default=None, help="插到队首的概率，对齐 G3 explore 的 reorder")
    a = ap.parse_args(_normalize_shadow(argv))
    card = load_card(a.scene)
    if a.policy in ("formation", "explore") and card.card["task"]["type"] != a.policy:
        raise SystemExit(f"--policy {a.policy} 只适用于 task.type 为 {a.policy} 的场景卡")
    out = execute(card, RunConfig(
        seed=a.seed,
        policy=a.policy,
        shadow=a.shadow == "on" and not a.no_shadow,
        duration_s=a.duration,
        bus=_bus(a),
    ))
    d = write(out, a.out or f"out/{card.name}/seed{a.seed}")
    print(json.dumps(out.result, ensure_ascii=False))
    print(f"写入 {d}/result.json, {d}/ticks.jsonl")
    return 0 if out.result["outcome"] == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
