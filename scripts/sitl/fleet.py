"""票 S2 / S3：多机 PX4 SITL 的布局与协同任务（report-only）。

``--mission hover``（默认）是票 S2：各自起飞到 1.5 m，悬停后降落。
``goto``：每机从悬停点平移，默认相对悬停点 ``(6, 0, 0)`` m。
``swap``：三机交换两端位置，巡航高度 1.5 / 2.2 / 2.9 m，同步直线段的 3D 间隔至少 0.7 m。
单机 S1 不走这里。``YOPO_SITL_MISSION=0`` 仍只表示拉起容器、不起飞。

布局不写飞控参数。instance、MAV_SYS_ID、uXRCE namespace 按 PX4 v1.17 的 posix rcS：

- ``MAV_SYS_ID = px4_instance + 1``，``UXRCE_DDS_KEY`` 同这个数（rcS 自己写）
- instance 0 没有 namespace；instance > 0 时 namespace 是 ``px4_<instance>``
- 因此 n 架从 instance 1 起，namespace 才是 ``px4_1 .. px4_n``
- 全部实例共用一个 XRCE Agent 端口、一个 DDS 域（容器里的 ``ROS_DOMAIN_ID``）

``layout`` 子命令只用标准库，方便宿主机在没有 numpy 时校验 ``--n``。
任务状态机复用 ``offboard_mission.Mission``（``land_at_hover``），ROS 胶水只在 ``run`` 里导入。
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# 与 yopo_swarm_core.px4.XRCE_DEFAULT_PORT 保持一致；布局路径不导入 px4（它依赖 numpy）。
XRCE_PORT = 8888
DDS_DOMAIN_ID = 0
ALLOWED_N = (1, 2, 3, 6)
MAX_N = 6
GCS_PORT_BASE = 18570  # px4-rc.mavlink：udp_gcs_port_local = 18570 + px4_instance
SPAWN_SPACING_M = 3.0
TAKEOFF_ALT_M = 1.5
HOVER_S = 10.0  # 仿真时间，只用来采样估计/真值误差，不是冻结判据
SCHEMA = "yopo-restart/s2-sitl-fleet/1"
SCHEMA_S3 = "yopo-restart/s3-sitl-crossing/1"
MISSION_MODES = ("hover", "goto", "swap")
# 相对悬停点。z=0 保持起飞高度。与 offboard_mission.DEFAULT_GOTO_OFFSET_ENU 相同。
GOTO_OFFSET_ENU = (6.0, 0.0, 0.0)
SWAP_ALT_M = (1.5, 2.2, 2.9)  # 相邻层 0.7 m，避免未接规划器时在同一高度对冲
MIN_GEOMETRIC_SEPARATION_M = 0.7
_PLAN_SAMPLES = 51


@dataclass(frozen=True)
class VehicleSpec:
    index: int
    instance: int
    mav_sys_id: int
    namespace: str
    model: str
    gcs_port: int
    pose: str  # PX4_GZ_MODEL_POSE；空字符串表示不覆盖模型自带的离地高度
    spawn_xy_m: tuple[float, float]
    standalone: bool


def fleet_specs(n: int) -> list[VehicleSpec]:
    if isinstance(n, bool) or not isinstance(n, int):
        raise ValueError(f"n 必须是整数，收到 {n!r}")
    if n not in ALLOWED_N:
        raise ValueError(f"n 只能是 1、2、3 或 6，收到 {n}")
    if n == 1:
        # S1 单机：instance 0、无 namespace、MAV_SYS_ID 1、Gazebo 模型 x500_0。不改生成点。
        return [VehicleSpec(1, 0, 1, "", "x500_0", GCS_PORT_BASE, "", (0.0, 0.0), False)]
    specs = []
    for i in range(1, n + 1):
        y = (i - 1) * SPAWN_SPACING_M
        pose = "0,0" if y == 0 else f"0,{y:g}"
        specs.append(
            VehicleSpec(
                index=i,
                instance=i,
                mav_sys_id=i + 1,
                namespace=f"px4_{i}",
                model=f"x500_{i}",
                gcs_port=GCS_PORT_BASE + i,
                pose=pose,
                spawn_xy_m=(0.0, float(y)),
                standalone=i != 1,
            )
        )
    domains = {DDS_DOMAIN_ID}
    if len(domains) != 1:
        raise ValueError("多机必须在同一个 DDS 域")
    return specs


def layout_dict(n: int, mission: str = "hover") -> dict:
    mode = parse_mission(mission)
    specs = fleet_specs(n)
    plans = vehicle_plans(n, mode)
    return {
        "n": n,
        "dds_domain_id": DDS_DOMAIN_ID,
        "xrce_port": XRCE_PORT,
        "takeoff_alt_m": TAKEOFF_ALT_M,
        "hover_s": HOVER_S,
        "mission": mode,
        "report_only": True,
        "criteria_frozen": False,
        "min_geometric_separation_m": min_geometric_separation_m(plans),
        "vehicles": [
            {
                "index": s.index,
                "instance": s.instance,
                "mav_sys_id": s.mav_sys_id,
                "namespace": s.namespace,
                "model": s.model,
                "gcs_port": s.gcs_port,
                "pose": s.pose,
                "spawn_xy_m": list(s.spawn_xy_m),
                "standalone": s.standalone,
            }
            for s in specs
        ],
        "waypoints": _waypoint_dicts(plans),
    }


@dataclass(frozen=True)
class VehiclePlan:
    index: int
    start_enu: tuple[float, float, float]
    hover_enu: tuple[float, float, float]
    goal_enu: tuple[float, float, float]
    takeoff_alt_m: float
    goto_target_enu: tuple[float, float, float]  # 相对悬停点


def parse_mission(mode: str) -> str:
    if mode not in MISSION_MODES:
        raise ValueError(f"mission 只能是 {'、'.join(MISSION_MODES)}，收到 {mode!r}")
    return mode


def resolve_mission(cli: str | None = None) -> str:
    """命令行优先于 ``YOPO_SITL_MISSION_MODE``，都没有则是 hover。

    ``exec_mission.sh`` 不传 ``--mission``，补跑任务时只靠容器里的环境变量。
    """
    if cli:
        return parse_mission(cli)
    env = os.environ.get("YOPO_SITL_MISSION_MODE", "").strip()
    if env:
        return parse_mission(env)
    return "hover"


def _lerp(a, b, u: float) -> tuple[float, float, float]:
    return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u)


def _dist3(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _pair_plan_separation(a: VehiclePlan, b: VehiclePlan, samples: int) -> float:
    """起飞段、悬停点、GOTO 段各自用同一进度对齐后的最小 3D 距离。"""
    best = _dist3(a.hover_enu, b.hover_enu)
    last = samples - 1
    for k in range(samples):
        u = k / last
        best = min(best, _dist3(_lerp(a.start_enu, a.hover_enu, u), _lerp(b.start_enu, b.hover_enu, u)))
        best = min(best, _dist3(_lerp(a.hover_enu, a.goal_enu, u), _lerp(b.hover_enu, b.goal_enu, u)))
    return best


def min_geometric_separation_m(plans: list[VehiclePlan], samples: int = _PLAN_SAMPLES) -> float | None:
    """同步直线路点的 3D 最小间隔。少于两架时没有机间距离。"""
    if len(plans) < 2:
        return None
    if samples < 2:
        raise ValueError("samples 至少为 2")
    best = None
    for i in range(len(plans)):
        for j in range(i + 1, len(plans)):
            dist = _pair_plan_separation(plans[i], plans[j], samples)
            best = dist if best is None else min(best, dist)
    return best


def assert_plan_separation(plans: list[VehiclePlan]) -> float | None:
    dist = min_geometric_separation_m(plans)
    if dist is not None and dist < MIN_GEOMETRIC_SEPARATION_M - 1e-6:
        raise ValueError(
            f"路点 3D 最小间隔 {dist:.3f} m 小于 {MIN_GEOMETRIC_SEPARATION_M:.1f} m"
        )
    return dist


def vehicle_plans(n: int, mission: str = "hover") -> list[VehiclePlan]:
    """每机的地面起点、悬停点和 GOTO 目标。hover 的目标就是悬停点（随后降落）。"""
    mode = parse_mission(mission)
    specs = fleet_specs(n)
    if mode == "swap" and n != 3:
        raise ValueError("swap 只用于 3 架（错高 1.5 / 2.2 / 2.9 m）")
    plans: list[VehiclePlan] = []
    for i, spec in enumerate(specs):
        x, y = spec.spawn_xy_m
        if mode == "hover":
            alt = TAKEOFF_ALT_M
            offset = (0.0, 0.0, 0.0)
        elif mode == "goto":
            alt = TAKEOFF_ALT_M
            offset = GOTO_OFFSET_ENU
        else:
            alt = SWAP_ALT_M[i]
            partner = specs[n - 1 - i]
            offset = (partner.spawn_xy_m[0] - x, partner.spawn_xy_m[1] - y, 0.0)
        start = (float(x), float(y), 0.0)
        hover = (float(x), float(y), float(alt))
        goal = (hover[0] + offset[0], hover[1] + offset[1], hover[2] + offset[2])
        plans.append(
            VehiclePlan(
                index=spec.index,
                start_enu=start,
                hover_enu=hover,
                goal_enu=goal,
                takeoff_alt_m=float(alt),
                goto_target_enu=(float(offset[0]), float(offset[1]), float(offset[2])),
            )
        )
    assert_plan_separation(plans)
    return plans


def _waypoint_dicts(plans: list[VehiclePlan]) -> list[dict]:
    return [
        {
            "index": p.index,
            "start_enu": list(p.start_enu),
            "hover_enu": list(p.hover_enu),
            "goal_enu": list(p.goal_enu),
            "takeoff_alt_m": p.takeoff_alt_m,
            "goto_target_enu": list(p.goto_target_enu),
        }
        for p in plans
    ]


def pose_xy(x: float, y: float) -> str:
    """PX4_GZ_MODEL_POSE 的 ``x,y``。九位小数够用，整数不带小数点。"""

    def one(value: float) -> str:
        text = f"{float(value):.9f}".rstrip("0").rstrip(".")
        if text in ("", "-0"):
            return "0"
        return text

    return f"{one(x)},{one(y)}"


def scene_start_xy(scene_path: str | Path) -> list[tuple[float, float]]:
    """场景卡 ``start_enu`` 的水平坐标。相对路径按仓库根解析。"""
    path = Path(scene_path)
    if not path.is_file():
        alt = Path(__file__).resolve().parents[2] / scene_path
        if alt.is_file():
            path = alt
        else:
            raise ValueError(f"找不到场景卡：{scene_path}")
    data = json.loads(path.read_text())
    drones = data.get("drones") or []
    if not drones:
        raise ValueError(f"{path} 没有飞机")
    starts = []
    for drone in drones:
        start = drone.get("start_enu")
        if not isinstance(start, (list, tuple)) or len(start) < 2:
            raise ValueError(f"{drone.get('id', '?')} 缺少 start_enu")
        starts.append((float(start[0]), float(start[1])))
    return starts


def apply_scene_spawn_poses(layout_text: str, scene_path: str | Path, n: int) -> str:
    """把 shell 布局里的 ``pose=`` 换成场景卡起点。

    S2 默认仍是沿 y 轴 0/3/6 m。三机场景卡（swap-3-open）的起点在半径 8 m 的圆上，
    和这条直线不是同一套点。YOPO 实飞以场景卡为准生成；不改 ``format_shell`` 的默认输出。
    """
    starts = scene_start_xy(scene_path)
    if len(starts) != int(n):
        raise ValueError(f"场景卡有 {len(starts)} 架，--n 是 {n}")
    poses = [pose_xy(x, y) for x, y in starts]
    index = 0
    out = []
    for line in layout_text.splitlines():
        if line.startswith("pose="):
            if index >= len(poses):
                raise ValueError("布局里的 pose 行比场景卡飞机多")
            out.append(f"pose={poses[index]}")
            index += 1
        else:
            out.append(line)
    if index != len(poses):
        raise ValueError(f"布局里有 {index} 行 pose，场景卡有 {len(poses)} 架")
    text = "\n".join(out)
    if layout_text.endswith("\n"):
        text += "\n"
    return text


def format_shell(n: int) -> str:
    data = layout_dict(n)
    lines = [
        f"n={data['n']}",
        f"dds_domain_id={data['dds_domain_id']}",
        f"xrce_port={data['xrce_port']}",
    ]
    for v in data["vehicles"]:
        lines.append("---")
        lines.append(f"instance={v['instance']}")
        lines.append(f"mav_sys_id={v['mav_sys_id']}")
        lines.append(f"namespace={v['namespace']}")
        lines.append(f"model={v['model']}")
        lines.append(f"gcs_port={v['gcs_port']}")
        lines.append(f"pose={v['pose']}")
        lines.append(f"standalone={1 if v['standalone'] else 0}")
    return "\n".join(lines) + "\n"


def _om():
    path = Path(__file__).with_name("offboard_mission.py")
    name = "yopo_sitl_offboard_mission"
    mod = sys.modules.get(name)
    if mod is None:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return mod


def fleet_mission_config():
    om = _om()
    return om.MissionConfig(
        takeoff_alt_m=TAKEOFF_ALT_M,
        hover_s=HOVER_S,
        goal_offset_enu=(0.0, 0.0, 0.0),
        land_at_hover=True,
        mission_mode="hover",
    )


def _config_for_plan(plan: VehiclePlan, mission: str):
    om = _om()
    if mission == "hover":
        return fleet_mission_config()
    return om.MissionConfig(
        takeoff_alt_m=plan.takeoff_alt_m,
        hover_s=HOVER_S,
        goal_offset_enu=(0.0, 0.0, 0.0),
        land_at_hover=False,
        mission_mode=mission,
        goto_target_enu=plan.goto_target_enu,
    )


class Fleet:
    """每机一台 Mission，共用仿真时钟。ok 只表示都落地并上锁，不看误差大小。"""

    def __init__(self, n: int, cfg=None, mission: str = "hover"):
        om = _om()
        mode = parse_mission(mission)
        self.mission_mode = mode if cfg is None else getattr(cfg, "mission_mode", mode)
        self.specs = fleet_specs(n)
        # 外部传入 cfg 时不改路点（S2 测试直接构造悬停配置）。
        self.plans = None if cfg is not None else vehicle_plans(n, mode)
        if cfg is not None:
            self.cfg = cfg
            self.missions = [om.Mission(self.cfg) for _ in self.specs]
        elif mode == "hover":
            self.cfg = fleet_mission_config()
            self.missions = [om.Mission(self.cfg) for _ in self.specs]
        else:
            cfgs = [_config_for_plan(p, mode) for p in self.plans]
            self.cfg = cfgs[0]
            self.missions = [om.Mission(c) for c in cfgs]
            for m in self.missions:
                m.block_goto = True

    @property
    def done(self) -> bool:
        return all(m.done for m in self.missions)

    def _maybe_release_goto(self) -> None:
        if self.mission_mode not in ("goto", "swap"):
            return
        if not any(m.block_goto for m in self.missions):
            return
        if any(m.phase == "FAIL" for m in self.missions):
            for m in self.missions:
                m.block_goto = False
            return
        if any(m.phase in ("WARMUP", "ARMING", "TAKEOFF") for m in self.missions):
            return
        if all(m.hover_complete for m in self.missions):
            for m in self.missions:
                m.block_goto = False

    def step(self, states: list):
        if len(states) != len(self.missions):
            raise ValueError(f"需要 {len(self.missions)} 架的状态，收到 {len(states)}")
        self._maybe_release_goto()
        return [m.step(s) for m, s in zip(self.missions, states)]


def command_message(kind: str, timestamp_us: int, target_system: int) -> dict:
    """VehicleCommand 字段。target_system 用该机的 MAV_SYS_ID，不广播。"""
    import math

    from yopo_swarm_core import px4

    om = _om()
    if kind == "offboard":
        return px4.offboard_mode_command(timestamp_us, target_system)
    if kind == "arm":
        return px4.arm_command(timestamp_us, target_system)
    if kind == "land":
        return px4.vehicle_command(om.VEHICLE_CMD_NAV_LAND, timestamp_us, target_system, param4=math.nan)
    raise ValueError(f"未知命令 {kind}")


def ros_topics(spec: VehicleSpec) -> dict[str, str]:
    from yopo_swarm_core import px4

    ns = spec.namespace
    return {
        "offboard_control_mode": px4.topic("in", "offboard_control_mode", ns),
        "trajectory_setpoint": px4.topic("in", "trajectory_setpoint", ns),
        "vehicle_command": px4.topic("in", "vehicle_command", ns),
        "vehicle_odometry": px4.topic("out", "vehicle_odometry", ns),
        "vehicle_status": px4.topic("out", "vehicle_status", ns),
        "vehicle_land_detected": px4.topic("out", "vehicle_land_detected", ns),
    }


def min_inter_drone_m_from_vehicles(vehicles: list[dict]) -> float | None:
    """全程对齐时间戳后的最小机间距离。真值列优先，没有真值时用估计位置。"""
    tracks = []
    for veh in vehicles:
        pts = {}
        for row in veh.get("trace") or []:
            if not row or len(row) < 5:
                continue
            t = row[0]
            if len(row) >= 8 and row[5] is not None and row[6] is not None and row[7] is not None:
                pts[t] = (float(row[5]), float(row[6]), float(row[7]))
            else:
                pts[t] = (float(row[2]), float(row[3]), float(row[4]))
        if pts:
            tracks.append(pts)
    if len(tracks) < 2:
        return None
    times = set(tracks[0])
    for track in tracks[1:]:
        times &= set(track)
    if not times:
        return None
    best = None
    for t in times:
        for i in range(len(tracks)):
            for j in range(i + 1, len(tracks)):
                dist = _dist3(tracks[i][t], tracks[j][t])
                best = dist if best is None else min(best, dist)
    return best


def min_inter_drone_m_from_traces(traces: dict | None) -> float | None:
    if not traces:
        return None
    return min_inter_drone_m_from_vehicles([{"trace": rows} for rows in traces.values()])


def write_trajectory_csvs(out_dir, traces: dict) -> None:
    """``/out/px4_<instance>/trajectory.csv``，供 summarize 在没有 result 轨迹时回退。"""
    root = Path(out_dir)
    for inst, rows in traces.items():
        folder = root / f"px4_{inst}"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / "trajectory.csv"
        with path.open("w", encoding="utf-8") as fh:
            fh.write("t,phase,est_x,est_y,est_z,truth_x,truth_y,truth_z\n")
            for row in rows:
                cells = ["" if value is None else str(value) for value in row]
                fh.write(",".join(cells) + "\n")


def result_payload(
    fleet: Fleet,
    *,
    positioning: str,
    wall_to_preflight_ok_s,
    preflight_checks_pass_seen: bool,
    mission_wall_s,
    mission_sim_s,
    traces: dict | None = None,
) -> dict:
    traces = traces or {}
    vehicles = []
    for spec, mission in zip(fleet.specs, fleet.missions):
        hover = None if mission.hover_sp is None else [float(x) for x in mission.hover_sp]
        goal = None if mission.goal_sp is None else [float(x) for x in mission.goal_sp]
        plan = None
        if fleet.plans:
            plan = next((p for p in fleet.plans if p.index == spec.index), None)
        vehicles.append(
            {
                "index": spec.index,
                "instance": spec.instance,
                "mav_sys_id": spec.mav_sys_id,
                "namespace": spec.namespace,
                "model": spec.model,
                "gcs_port": spec.gcs_port,
                "pose": spec.pose,
                "spawn_xy_m": list(spec.spawn_xy_m),
                "hover_enu": hover,
                "goal_enu": goal,
                "goto_target_enu": None if plan is None else list(plan.goto_target_enu),
                "ok": mission.phase == "DONE",
                "phase": mission.phase,
                "metrics": dataclasses.asdict(mission.metrics),
                "home_enu": None if mission.home is None else mission.home.tolist(),
                "trace": traces.get(spec.instance, []),
            }
        )
    ok = bool(vehicles) and all(v["ok"] for v in vehicles)
    rtf = (mission_sim_s / mission_wall_s) if mission_wall_s and mission_sim_s is not None else None
    mode = getattr(fleet, "mission_mode", "hover")
    separation = None if not getattr(fleet, "plans", None) else min_geometric_separation_m(fleet.plans)
    return {
        "schema": SCHEMA if mode == "hover" else SCHEMA_S3,
        "report_only": True,
        "criteria_frozen": False,
        "mission": mode,
        "n": len(fleet.specs),
        "dds_domain_id": DDS_DOMAIN_ID,
        "xrce_port": XRCE_PORT,
        "positioning": positioning,
        "ok": ok,
        "phase": "DONE" if ok else "FAIL",
        "config": dataclasses.asdict(fleet.cfg),
        "min_geometric_separation_m": separation,
        "min_inter_drone_m": min_inter_drone_m_from_traces(traces),
        "waypoints": [] if not getattr(fleet, "plans", None) else _waypoint_dicts(fleet.plans),
        "vehicles": vehicles,
        "wall_to_preflight_ok_s": wall_to_preflight_ok_s,
        "preflight_checks_pass_seen": preflight_checks_pass_seen,
        "mission_wall_s": mission_wall_s,
        "mission_sim_s": mission_sim_s,
        "rtf": rtf,
    }


def _fail_open(fleet: Fleet, why: str) -> None:
    for mission in fleet.missions:
        if mission.done:
            continue
        if mission.metrics.failure is None:
            mission.metrics.failure = f"{mission.phase}: {why}"
        mission.phase = "FAIL"


def run_ros(n: int, out_path: str, wall_timeout: float, world: str, preflight_timeout: float, mission: str = "hover") -> int:
    """容器内：订阅每机 uXRCE 话题和 Gazebo 真值，写出 result.json。"""
    import threading

    try:
        import numpy as np
        import rclpy
    except ImportError as exc:
        print(f"run 需要容器里的 ROS 2 / numpy（{exc}）", file=sys.stderr)
        return 2
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleOdometry,
        VehicleStatus,
    )
    from pymavlink import mavutil
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    from yopo_swarm_core import px4

    om = _om()
    mode = parse_mission(mission)
    if px4.XRCE_DEFAULT_PORT != XRCE_PORT:
        print(f"XRCE 端口与 core 不一致：布局 {XRCE_PORT}，core {px4.XRCE_DEFAULT_PORT}", file=sys.stderr)
        return 2
    specs = fleet_specs(n)
    fleet = Fleet(n, mission=mode)
    positioning = os.environ.get("YOPO_POSITIONING", "default")

    lock = threading.Lock()
    sim = {"t": None, "truth": {s.model: None for s in specs}}
    stop = threading.Event()

    def gcs_heartbeat():
        conns = [
            mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{s.gcs_port}", source_system=255, source_component=190
            )
            for s in specs
        ]
        while not stop.is_set():
            for conn in conns:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            stop.wait(1.0)

    threading.Thread(target=gcs_heartbeat, daemon=True).start()

    def on_clock(msg):
        with lock:
            sim["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        with lock:
            for p in msg.pose:
                if p.name in sim["truth"]:
                    sim["truth"][p.name] = np.array([p.position.x, p.position.y, p.position.z])

    gz = GzNode()
    if not gz.subscribe(Clock, f"/world/{world}/clock", on_clock):
        print("gz clock subscribe failed", file=sys.stderr)
        return 2
    if not gz.subscribe(Pose_V, f"/world/{world}/pose/info", on_pose):
        print("gz pose subscribe failed", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("yopo_s2_offboard")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def fill(msg, fields: dict):
        for k, v in fields.items():
            setattr(msg, k, v)
        return msg

    pubs = {}
    st = {}
    for spec in specs:
        topics = ros_topics(spec)
        st[spec.instance] = {"odom": None, "status": None, "land": None}
        pubs[spec.instance] = {
            "ocm": node.create_publisher(OffboardControlMode, topics["offboard_control_mode"], qos),
            "sp": node.create_publisher(TrajectorySetpoint, topics["trajectory_setpoint"], qos),
            "cmd": node.create_publisher(VehicleCommand, topics["vehicle_command"], qos),
        }
        slot = st[spec.instance]
        node.create_subscription(VehicleOdometry, topics["vehicle_odometry"], lambda m, s=slot: s.__setitem__("odom", m), qos)
        node.create_subscription(VehicleStatus, topics["vehicle_status"], lambda m, s=slot: s.__setitem__("status", m), qos)
        node.create_subscription(
            VehicleLandDetected, topics["vehicle_land_detected"], lambda m, s=slot: s.__setitem__("land", m), qos
        )

    def now_us() -> int:
        return node.get_clock().now().nanoseconds // 1000

    wall0 = time.monotonic()
    wall_ready = sim_ready = None
    traces = {s.instance: [] for s in specs}
    preflight_seen = False
    try:
        while not fleet.done:
            rclpy.spin_once(node, timeout_sec=0.02)
            wall = time.monotonic() - wall0
            if wall > wall_timeout:
                _fail_open(fleet, f"wall timeout {wall_timeout}s")
                break
            with lock:
                t_sim = sim["t"]
                truths = {k: (None if v is None else v.copy()) for k, v in sim["truth"].items()}
            missing = []
            if t_sim is None:
                missing.append("gz_clock")
            slots = []
            for spec in specs:
                slot = st[spec.instance]
                if slot["odom"] is None or slot["status"] is None or slot["land"] is None:
                    missing.append(spec.namespace or spec.model)
                slots.append(slot)
            if missing:
                if wall > preflight_timeout:
                    _fail_open(fleet, f"PREFLIGHT: no {','.join(missing)} after {preflight_timeout}s wall")
                    break
                continue
            armed_ok = all(slot["status"].pre_flight_checks_pass for slot in slots)
            preflight_seen = preflight_seen or armed_ok
            if wall_ready is None:
                if not armed_ok:
                    if wall > preflight_timeout:
                        _fail_open(fleet, f"PREFLIGHT: pre_flight_checks_pass=false after {preflight_timeout}s wall")
                        break
                    continue
                wall_ready, sim_ready = wall, t_sim
            states = []
            bad = False
            for spec, slot in zip(specs, slots):
                o, vs, ld = slot["odom"], slot["status"], slot["land"]
                try:
                    enu = px4.odometry_to_enu(o.position, o.q, o.velocity, o.timestamp, o.pose_frame, o.velocity_frame)
                except ValueError:
                    bad = True
                    break
                states.append(
                    om.VehicleState(
                        t=t_sim,
                        position=enu.position,
                        velocity=enu.velocity,
                        armed=vs.arming_state == om.ARMING_STATE_ARMED,
                        offboard=vs.nav_state == om.NAV_STATE_OFFBOARD,
                        landed=bool(ld.landed),
                        truth=truths.get(spec.model),
                    )
                )
            if bad:
                continue
            actions = fleet.step(states)
            ts = now_us()
            for spec, act, s in zip(specs, actions, states):
                pub = pubs[spec.instance]
                if act.heartbeat:
                    pub["ocm"].publish(fill(OffboardControlMode(), px4.offboard_control_mode(ts)))
                if act.setpoint_pva is not None:
                    pub["sp"].publish(
                        fill(TrajectorySetpoint(), px4.trajectory_setpoint(act.setpoint_pva, act.yaw_enu, ts))
                    )
                for c in act.commands:
                    pub["cmd"].publish(fill(VehicleCommand(), command_message(c, ts, spec.mav_sys_id)))
                tr = traces[spec.instance]
                if not tr or t_sim - tr[-1][0] >= 0.1:
                    truth = truths.get(spec.model)
                    tz = [None] * 3 if truth is None else [round(float(x), 4) for x in truth]
                    tr.append([round(t_sim, 3), fleet.missions[spec.index - 1].phase, *[round(float(x), 4) for x in s.position], *tz])
            time.sleep(0.02)
    finally:
        stop.set()
        wall_end = time.monotonic() - wall0
        with lock:
            t_end = sim["t"]
        node.destroy_node()
        rclpy.try_shutdown()

    mission_wall = None if wall_ready is None else wall_end - wall_ready
    mission_sim = None if sim_ready is None or t_end is None else t_end - sim_ready
    out = result_payload(
        fleet,
        positioning=positioning,
        wall_to_preflight_ok_s=wall_ready,
        preflight_checks_pass_seen=preflight_seen,
        mission_wall_s=mission_wall,
        mission_sim_s=mission_sim,
        traces=traces,
    )
    write_trajectory_csvs(Path(out_path).parent, traces)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
    brief = {k: v for k, v in out.items() if k != "vehicles"}
    brief["vehicles"] = [{k: v for k, v in veh.items() if k != "trace"} for veh in out["vehicles"]]
    print(json.dumps(brief, indent=1))
    return 0 if out["ok"] else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="多机 PX4 SITL 布局 / 协同任务（report-only）")
    sub = ap.add_subparsers(dest="cmd")
    lay = sub.add_parser("layout", help="打印机群布局，不起飞")
    lay.add_argument("--n", type=int, required=True)
    lay.add_argument("--format", choices=("shell", "json"), default="json")
    lay.add_argument("--mission", choices=MISSION_MODES, default=None)
    run = sub.add_parser("run", help="容器内跑 hover / goto / swap")
    run.add_argument("--n", type=int, required=True)
    run.add_argument("--out", required=True)
    run.add_argument("--wall-timeout", type=float, default=900.0)
    run.add_argument("--world", default="default")
    run.add_argument("--preflight-timeout", type=float, default=240.0)
    run.add_argument("--mission", choices=MISSION_MODES, default=None)
    poses = sub.add_parser("yopo-poses", help="按场景卡起点改写 shell 布局的 pose，不改 S2 默认生成点")
    poses.add_argument("--scene", required=True)
    poses.add_argument("--n", type=int, required=True)
    poses.add_argument("--layout", required=True)
    # 兼容：`fleet.py --n 3 --format shell`（up.sh / in_container.sh 用这个）
    ap.add_argument("--n", type=int)
    ap.add_argument("--format", choices=("shell", "json"), default="json")
    ap.add_argument("--mission", choices=MISSION_MODES, default=None)
    args = ap.parse_args(argv)
    try:
        if args.cmd == "yopo-poses":
            text = Path(args.layout).read_text()
            sys.stdout.write(apply_scene_spawn_poses(text, args.scene, args.n))
            return 0
        mode = resolve_mission(args.mission)
        if args.cmd == "run":
            return run_ros(args.n, args.out, args.wall_timeout, args.world, args.preflight_timeout, mode)
        n = args.n
        fmt = args.format
        if n is None:
            ap.error("需要 --n")
        if fmt == "shell":
            # 先校验路点。shell 行集保持 S2，in_container.sh 只认那一组前缀。
            vehicle_plans(n, mode)
            sys.stdout.write(format_shell(n))
        else:
            json.dump(layout_dict(n, mode), sys.stdout, indent=1)
            sys.stdout.write("\n")
        return 0
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
