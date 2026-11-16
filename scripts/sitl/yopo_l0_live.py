"""票 S4b：容器内把 YOPO L0 规划桥接到 PX4 ROS 2 话题（report-only）。

订阅每机 ``VehicleOdometry``（``/fmu/out/vehicle_odometry`` 或 ``/px4_N/...``）、
可选的深度图 ``/yopo/depth_image`` 和障碍摘要 ``/yopo/obstacle_summary``。
ENU↔NED 只走 ``yopo_swarm_core.px4``。设定点是 NED 的 ``TrajectorySetpoint``。

``rclpy`` 只在真正开环时导入。``--dry-run`` 只打印话题表，不起飞，不写飞控参数。
默认离线假 PX4 仍走 ``run_yopo_l0.py``。本文件不改 ``offboard_mission.py``。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

from yopo_l0.b2_plan_diag import PlanDiag, apply_b2_overrides, b3_from_fix, fix_deviations, parse_fix
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    ClockSkewMeter,
    FLEET_POLICIES,
    LiveFleetConfig,
    VehicleInbox,
    live_topic_plan,
    make_live_session,
    normalize_positioning,
    outcome_exit_code,
    resolve_live_policy,
    build_scene_world_sdf,
    vehicle_topics,
)
from yopo_l0.sitl_lockstep import (
    DEFAULT_PHYSICS_STEP_S,
    REQUEST_TIMEOUT_MS,
    LockstepStepper,
    SimStampSkewMeter,
    model_name,
    motor_topic,
)

_REPO = Path(__file__).resolve().parents[2]
ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14
# PX4 v1.17 ``COM_OF_LOSS_T`` 默认 1 s。规划一步会堵住主循环，设定点要在这段时间里继续发。
SETPOINT_PERIOD_S = 0.05


class MissionClock:
    """任务时间从第一次拿到仿真钟开始。

    Gazebo ``/clock`` 是开机以来的绝对时间。PX4 启动先走掉二十多秒，
    再拿它和场景卡 ``duration_s`` 比较，会话里实际飞不够这个时长。
    """

    def __init__(self, max_s: float):
        limit = float(max_s)
        if not limit > 0.0:
            raise ValueError(f"max_s 必须为正，收到 {max_s!r}")
        self.max_s = limit
        self.origin: float | None = None

    def elapsed(self, t_sim: float) -> float:
        t = float(t_sim)
        if self.origin is None:
            self.origin = t
        return t - self.origin

    def expired(self, t_sim: float) -> bool:
        return self.elapsed(t_sim) > self.max_s


def hold_setpoint(setpoint: dict | None) -> dict | None:
    """心跳重发只守位置和航向。速度、加速度前馈清零。

    规划一步常常超过 1 s。若把上一拍的 velocity 按 20 Hz 再发，PX4 会把这口
    前馈一直积下去，三机都顺着这口速度漂。位置设定点仍是已经转到本地 NED 的那一份。
    """
    if setpoint is None:
        return None
    held = dict(setpoint)
    held["velocity"] = [0.0, 0.0, 0.0]
    held["acceleration"] = [0.0, 0.0, 0.0]
    return held


def streamed_setpoint(command, tracked: dict | None = None) -> tuple[dict | None, dict | None]:
    """心跳发什么。

    PLAN 阶段 ``tracked`` 是轨迹在当前仿真钟上的设定点，速度和加速度保留。
    这口速度属于这一时刻，不是把 1 s 前的前馈再积一遍。
    没有轨迹时，以及其他阶段，仍只守位置和航向。
    """
    if not getattr(command, "heartbeat", False):
        return None, None
    if tracked is None:
        return offboard_stream(command)
    mode = command.offboard
    stamp = tracked.get("timestamp")
    if mode is not None and stamp is not None:
        mode = dict(mode)
        mode["timestamp"] = stamp
    return mode, tracked


def offboard_stream(command) -> tuple[dict | None, dict | None]:
    """持续心跳只带 OffboardControlMode 和位置保持设定点。

    解锁、降落这类 VehicleCommand 不在这里重复发。设定点已是本地 NED，这里不再做坐标变换。
    """
    if not getattr(command, "heartbeat", False):
        return None, None
    return command.offboard, hold_setpoint(command.setpoint)


class PreflightLatch:
    """启动预检只要求第一次全体通过，通过后锁存。

    状态：
    - ``WAIT``：还没有一拍是每架都有样本且 ``preflight_ok``。这一拍不 step。
    - ``LATCHED``：解锁 / 起飞前已经通过一次。之后继续 step。
    - ``FAIL``：锁存前墙钟超过 ``--preflight-timeout``。这才是启动预检失败。

    PX4 用 ``canArm(当前模式)`` 刷新 ``pre_flight_checks_pass``，着陆上锁后会把它拉低。
    锁存之后这种失败留在 ``LATCHED``：不停 step，也不记成启动 ``PREFLIGHT``。
    """

    WAIT = "WAIT"
    LATCHED = "LATCHED"
    FAIL = "FAIL"

    def __init__(self, timeout_s: float):
        timeout = float(timeout_s)
        if not timeout > 0.0:
            raise ValueError(f"preflight timeout 必须为正，收到 {timeout_s!r}")
        self.timeout_s = timeout
        self.state = self.WAIT
        self.failure = ""

    def allow_step(self, boxes: dict, wall_s: float) -> bool:
        """``boxes`` 的值是带 ``preflight_ok`` 的样本；``None`` 表示这一拍还没齐。"""
        if self.state == self.FAIL:
            return False
        if self.state == self.LATCHED:
            return True
        ready = bool(boxes) and all(box is not None and bool(box.preflight_ok) for box in boxes.values())
        if ready:
            self.state = self.LATCHED
            return True
        if float(wall_s) > self.timeout_s:
            missing = [ns or "/" for ns, box in boxes.items() if box is None]
            self.failure = "PREFLIGHT " + (",".join(missing) if missing else "checks")
            self.state = self.FAIL
            return False
        return False



def _scene_path(raw: str) -> Path:
    path = Path(raw)
    if path.is_file():
        return path
    alt = _REPO / raw
    if alt.is_file():
        return alt
    raise SystemExit(f"找不到场景卡：{raw}")


def _write(path: str, payload: dict) -> None:
    text = json.dumps(payload, indent=1, ensure_ascii=False) + "\n"
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    sys.stdout.write(text)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", default="scenes/g2-pillar.json")
    ap.add_argument("--n", type=int, default=None, help="必须和场景卡里的飞机数一致")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--wall-timeout", type=float, default=900.0)
    ap.add_argument("--max-s", type=float, default=None, help="仿真时间上限，默认用场景卡 duration_s")
    ap.add_argument("--positioning", choices=("default", "rtk"), default=None)
    ap.add_argument(
        "--policy",
        choices=("upstream", "formation", "crossing", "switch"),
        default="upstream",
        help="upstream 走 YOPO。formation / crossing / switch 只用于同名 task.type 的卡，机间消息留在进程内总线",
    )
    ap.add_argument("--world", default="default")
    ap.add_argument("--preflight-timeout", type=float, default=240.0)
    ap.add_argument("--dry-run", action="store_true", help="只写话题和定位模式，不连 ROS")
    # 票 B1：仿真钟锁步。默认关；也可用 YOPO_SITL_LOCKSTEP=1 打开。
    ap.add_argument(
        "--lockstep",
        action="store_true",
        default=_env_flag("YOPO_SITL_LOCKSTEP"),
        help="按物理步推进 gz，每步等齐各机 PX4 电机指令；代理算多久世界就停多久（偏离上游）",
    )
    ap.add_argument(
        "--physics-step",
        type=float,
        default=DEFAULT_PHYSICS_STEP_S,
        help="gz world 的 max_step_size，锁步按它对齐（PX4 默认 world 是 0.004 s）",
    )
    ap.add_argument(
        "--agent-delay-s",
        type=float,
        default=float(os.environ.get("YOPO_AGENT_DELAY_S", "0") or 0.0),
        help="人为延迟：每个代理控制拍（0.02 s 仿真）额外睡这么久墙钟，只用于证明结果与算力无关",
    )
    # 票 SH2：影子盾环境证据。默认关；也可用 YOPO_SHADOW_ENV_EVIDENCE=1 打开。打开即偏离上游。
    ap.add_argument(
        "--shadow-env-evidence",
        action="store_true",
        default=_env_flag("YOPO_SHADOW_ENV_EVIDENCE"),
        help="把场景卡几何交给影子盾（默认关，打开即偏离上游）",
    )
    ap.add_argument(
        "--fleet-shadow",
        action="store_true",
        default=_env_flag("YOPO_FLEET_SHADOW"),
        help="crossing / switch / formation 也记影子盾（只读，默认关）",
    )
    ap.add_argument(
        "--shadow-contract-align",
        action="store_true",
        default=_env_flag("YOPO_SHADOW_CONTRACT_ALIGN"),
        help="研究开关，默认关。打开后按上游轨迹语义修正影子盾误报契约码（偏离上游）",
    )
    ap.add_argument(
        "--shadow-tracking-align",
        action="store_true",
        default=_env_flag("YOPO_SHADOW_TRACKING_ALIGN"),
        help="研究开关，默认关。打开后 0.15 m 的 TRACKING_BOUND 仍记录，但不计入 would_reject（偏离上游）",
    )
    # 票 B2：交叉卡规划净空归因。都默认关。
    ap.add_argument(
        "--plan-diag",
        action="store_true",
        default=_env_flag("YOPO_B2_PLAN_DIAG"),
        help="只读记录规划输入（跟踪误差、深度帧年龄、计划净空），写进 result.json 的 b2_plan_diag，不改选择",
    )
    ap.add_argument(
        "--safe-radius",
        type=float,
        default=_env_float("YOPO_B2_SAFE_RADIUS"),
        help="在内存里盖过场景卡 policy_params.upstream.safe_radius（缺省 0.05 m，打开即偏离上游）",
    )
    ap.add_argument(
        "--b2-fix",
        default=os.environ.get("YOPO_B2_FIX", ""),
        help="修复候选，逗号分隔 key=value；目前 lead=<秒>：设定点提前取（偏离上游）。缺省空 = 全关",
    )
    # 票 S8：场景多圆柱/林带世界 SDF 注入标记（默认关，也可经 YOPO_SITL_SCENE_WORLD=1 打开）。
    ap.add_argument(
        "--scene-world",
        action="store_true",
        default=_env_flag("YOPO_SITL_SCENE_WORLD"),
        help="在 Gazebo 世界中注入场景卡圆柱与矮枝静态几何（默认关）",
    )
    ap.add_argument(
        "--world-summary",
        default=os.environ.get("YOPO_SITL_WORLD_SUMMARY", "/out/world_sdf.json"),
        help="容器启动时 world_sdf.py 写出的摘要路径",
    )
    return ap.parse_args(argv)


def _env_float(name: str) -> float | None:
    raw = os.environ.get(name, "").strip()
    return float(raw) if raw else None


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def _positioning(arg: str | None) -> str:
    if arg:
        return normalize_positioning(arg)
    return normalize_positioning(os.environ.get("YOPO_POSITIONING", "default"))


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.wall_timeout > 0.0:
        print("--wall-timeout 必须为正", file=sys.stderr)
        return 2
    if not args.physics_step > 0.0 or args.agent_delay_s < 0.0:
        print("--physics-step 必须为正，--agent-delay-s 不能为负", file=sys.stderr)
        return 2
    positioning = _positioning(args.positioning)
    card = load_card(_scene_path(args.scene))
    try:
        args.b2_fix_cfg = parse_fix(args.b2_fix)
        card, b2_deviations = apply_b2_overrides(
            card,
            safe_radius=args.safe_radius,
            head_from_odom_m=args.b2_fix_cfg.get("head_from_odom_m"),
            b3=b3_from_fix(args.b2_fix_cfg),
        )
        b2_deviations += fix_deviations(args.b2_fix_cfg)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    args.b2_deviations = b2_deviations
    n = len(card.card["drones"])
    if args.n is not None and int(args.n) != n:
        print(f"--n {args.n} 和场景 {card.name} 的飞机数 {n} 不一致", file=sys.stderr)
        return 2
    try:
        policy = resolve_live_policy(card, args.policy)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.dry_run:
        plan = live_topic_plan(card, positioning=positioning, seed=args.seed, policy=policy)
        plan["lockstep"] = lockstep_plan(card, args)
        plan["scene_world"] = scene_world_plan(card, args)
        if args.shadow_env_evidence or args.fleet_shadow:
            # 缺省 dry-run 仍是 policy==upstream 才 shadow=True。打开本票开关才改这份计划。
            plan["shadow"] = True
            plan["fleet_shadow"] = bool(args.fleet_shadow or args.shadow_env_evidence)
        if args.shadow_env_evidence:
            plan["shadow_env_evidence"] = True
            plan["upstream_deviation"] = True
        if args.shadow_contract_align:
            plan["shadow_contract_align"] = True
            plan["upstream_deviation"] = True
        if args.shadow_tracking_align:
            plan["shadow_tracking_align"] = True
            plan["upstream_deviation"] = True
        plan["b2"] = {"plan_diag": bool(args.plan_diag), "deviations": list(b2_deviations), "fix": dict(args.b2_fix_cfg)}
        _write(args.out, plan)
        return 0
    try:
        return _run_ros(args, card, positioning, policy)
    except ModuleNotFoundError as exc:
        missing = exc.name or "依赖"
        print(f"实飞入口停住：当前解释器没有 {missing}。不写飞控参数。", file=sys.stderr)
        return 2


def lockstep_plan(card, args) -> dict:
    """dry-run 里列出锁步会等哪些 gz 话题。关时只写 enabled=false。"""
    from yopo_l0.sitl_bridge import vehicle_identity

    n = len(card.card["drones"])
    if not args.lockstep:
        return {"enabled": False, "agent_delay_s": float(args.agent_delay_s)}
    return {
        "enabled": True,
        "deviation": "偏离上游：仿真钟锁步",
        "world_control": f"/world/{args.world}/control",
        "clock": f"/world/{args.world}/clock",
        "physics_step_s": float(args.physics_step),
        "handshake_topics": [motor_topic(vehicle_identity(i, n)["instance"]) for i in range(n)],
        "agent_delay_s": float(args.agent_delay_s),
        "px4_parameters_written": [],
    }


def scene_world_plan(card, args) -> dict:
    """dry-run 与结果记录里的场景几何世界摘要。默认关时只写 enabled=false。"""
    if not getattr(args, "scene_world", False):
        return {"enabled": False}
    summary_path = Path(getattr(args, "world_summary", "") or "")
    if summary_path.is_file():
        try:
            loaded = json.loads(summary_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict) and loaded.get("scene") == card.name:
                return loaded
        except (OSError, ValueError):
            pass
    _sdf_xml, summary = build_scene_world_sdf(card, world_name=str(args.world))
    return summary


class _LockstepAbort(Exception):
    """锁步没接上：不退回自由跑，直接写失败结果。"""


class GzWorldPort:  # pragma: no cover - 需要 gz-transport
    """``LockstepStepper`` 的 gz-transport 实现。clock 和电机计数由订阅回调写进共享 dict。"""

    def __init__(self, node, world: str, sim: dict, lock, counts: dict):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.world_control_pb2 import WorldControl

        self._node = node
        self._service = f"/world/{world}/control"
        self._sim = sim
        self._lock = lock
        self._counts = counts
        self._WorldControl = WorldControl
        self._Boolean = Boolean

    def sim_time(self):
        with self._lock:
            return self._sim["t"]

    def motor_counts(self) -> dict:
        with self._lock:
            return dict(self._counts)

    def _request(self, req) -> None:
        # 回复丢了不要紧：请求已经生效，是否到位看 clock。
        try:
            self._node.request(self._service, req, self._WorldControl, self._Boolean, REQUEST_TIMEOUT_MS)
        except Exception:
            pass

    def request_pause(self) -> None:
        self._request(self._WorldControl(pause=True))

    def request_step(self) -> None:
        # 世界停着走一步再停；不会因为目标落后而放开世界。
        self._request(self._WorldControl(pause=True, multi_step=1))

    def request_resume(self) -> None:
        for _ in range(3):
            self._request(self._WorldControl(pause=False))


def _truth_reader(session, truth: dict, lock):  # pragma: no cover - 需要 gz-transport
    names = {slot.bridge.drone_id: model_name(int(slot.ident["instance"])) for slot in session.slots}

    def read(drone_id: str):
        with lock:
            p = truth.get(names.get(drone_id, ""))
        if p is None:
            return None
        import numpy as np

        return np.asarray(p, dtype=float)

    return read


def _fill(msg, fields: dict):
    for key, value in fields.items():
        setattr(msg, key, value)
    return msg


def _run_ros(args, card, positioning: str, policy: str) -> int:  # pragma: no cover - 需要 rclpy / px4_msgs
    import rclpy
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleOdometry,
        VehicleStatus,
    )
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    task = card.card.get("task") or {}
    max_s = float(args.max_s) if args.max_s is not None else float(task.get("duration_s", 45.0))
    # 编队飞行大约 20 s。45 s 的缺省 PLAN 超时留给上游。编队放宽到 60 s，仍不含起飞。
    # 票 S7：crossing / switch 也走协议 fleet，同样放宽到 60 s。upstream 仍是 45 s。
    plan_timeout_s = max(60.0, max_s - 20.0) if policy in FLEET_POLICIES else 45.0
    session = make_live_session(
        card,
        seed=args.seed,
        policy=policy,
        live=LiveFleetConfig(
            positioning=positioning,
            depth_source="auto",
            initial_phase="WARMUP",
            plan_timeout_s=plan_timeout_s,
            shadow_env_evidence=bool(args.shadow_env_evidence),
            fleet_shadow=bool(args.fleet_shadow),
            shadow_contract_align=bool(args.shadow_contract_align),
            shadow_tracking_align=bool(args.shadow_tracking_align),
            setpoint_lead_s=float(getattr(args, "b2_fix_cfg", {}).get("setpoint_lead_s", 0.0)),
            setpoint_gain_p=float(getattr(args, "b2_fix_cfg", {}).get("setpoint_gain_p", 0.0)),
            setpoint_gain_v=float(getattr(args, "b2_fix_cfg", {}).get("setpoint_gain_v", 0.0)),
        ),
    )
    truth: dict = {}  # 票 B2：gz 真值位置，记录器建好前先有这个 dict，订阅回调往里写
    truth_lock = threading.Lock()
    plan_diag = None
    if args.plan_diag and session.fleet is not None and hasattr(session.fleet, "ingest"):
        plan_diag = PlanDiag(
            session.fleet,
            card.min_distance,
            float(card.card["criteria"]["body_radius_m"]),
            abs_offset=lambda: float(session._plan_origin or 0.0),
            truth=_truth_reader(session, truth, truth_lock),
        )
    ports = [int(slot.ident["gcs_port"]) for slot in session.slots]
    stop = threading.Event()

    def gcs_heartbeat() -> None:
        from pymavlink import mavutil

        conns = [
            mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=255, source_component=190)
            for port in ports
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

    lock = threading.Lock()
    sim = {"t": None}
    motor_counts: dict[str, int] = {}

    def on_clock(msg) -> None:
        with lock:
            sim["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_odom(box: dict, msg) -> None:
        # 票 B1：记下收到这条里程计时的 gz 仿真钟，机间偏差按仿真钟量，不靠 PX4 时间戳换算。
        with lock:
            box["odom_sim"] = sim["t"]
        box["odom_n"] += 1
        box["odom"] = msg

    try:
        from gz.msgs10.clock_pb2 import Clock
        from gz.transport13 import Node as GzNode

        gz = GzNode()
        gz.subscribe(Clock, f"/world/{args.world}/clock", on_clock)
    except Exception:
        gz = None
    if args.lockstep:
        if gz is None:
            print("锁步要 gz-transport（gz.transport13 / gz.msgs10），当前解释器没有。", file=sys.stderr)
            return 2
        from gz.msgs10.actuators_pb2 import Actuators

        for i, slot in enumerate(session.slots):
            ns = slot.ident["namespace"]
            motor_counts[ns] = 0

            def on_motor(_msg, ns=ns) -> None:
                with lock:
                    motor_counts[ns] += 1

            gz.subscribe(Actuators, motor_topic(int(slot.ident["instance"])), on_motor)

    if args.plan_diag and gz is not None:
        # 票 B2：只读。gz 世界系的真值位置，和 PX4 的 EKF 里程计对照，分清是控制没跟上还是估计偏了。
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V

            def on_poses(msg) -> None:
                with truth_lock:
                    for pose in msg.pose:
                        if pose.name.startswith("x500_"):
                            truth[pose.name] = (pose.position.x, pose.position.y, pose.position.z)

            gz.subscribe(Pose_V, f"/world/{args.world}/dynamic_pose/info", on_poses)
        except Exception:
            truth.clear()

    rclpy.init()
    node = rclpy.create_node("yopo_l0_sitl_live")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pubs = {}
    slots = {}
    for slot in session.slots:
        ns = slot.ident["namespace"]
        topics = vehicle_topics(ns)
        state = {"odom": None, "status": None, "land": None, "depth": None, "summary": None, "odom_sim": None, "odom_n": 0}
        slots[ns] = state
        pubs[ns] = {
            "ocm": node.create_publisher(OffboardControlMode, topics["offboard_control_mode"], qos),
            "sp": node.create_publisher(TrajectorySetpoint, topics["trajectory_setpoint"], qos),
            "cmd": node.create_publisher(VehicleCommand, topics["vehicle_command"], qos),
        }
        node.create_subscription(VehicleOdometry, topics["vehicle_odometry"], lambda msg, box=state: on_odom(box, msg), qos)
        node.create_subscription(VehicleStatus, topics["vehicle_status"], lambda msg, box=state: box.__setitem__("status", msg), qos)
        node.create_subscription(
            VehicleLandDetected, topics["vehicle_land_detected"], lambda msg, box=state: box.__setitem__("land", msg), qos
        )
        node.create_subscription(Image, topics["depth_image"], lambda msg, box=state: box.__setitem__("depth", msg), sensor_qos)
        node.create_subscription(
            String, topics["obstacle_summary"], lambda msg, box=state: box.__setitem__("summary", msg.data), sensor_qos
        )

    def inbox_for(ns: str) -> VehicleInbox | None:
        state = slots[ns]
        odom, status, land = state["odom"], state["status"], state["land"]
        if odom is None or status is None or land is None:
            return None
        summary = state["summary"]
        return VehicleInbox(
            odometry=odom,
            depth=state["depth"],
            obstacle_summary=summary if summary else None,
            armed=status.arming_state == ARMING_STATE_ARMED,
            offboard=status.nav_state == NAV_STATE_OFFBOARD,
            landed=bool(land.landed),
            preflight_ok=bool(status.pre_flight_checks_pass),
        )

    wall0 = time.monotonic()
    last_elapsed = None
    physics_dt = float(session.slots[0].bridge.cfg.physics_dt)
    failure = ""
    preflight = PreflightLatch(args.preflight_timeout)
    clock = MissionClock(max_s)
    skew = ClockSkewMeter()  # 票 S7：只报数
    sim_skew = SimStampSkewMeter()  # 票 B1：按仿真钟量的里程计到达时刻
    stepper = None
    odom_wait_timeouts = 0
    streamed = {"commands": []}
    stream_lock = threading.Lock()

    def publish_stream() -> None:
        # 规划（场景卡相机 + 网络）会占住主循环超过 COM_OF_LOSS_T。
        # PLAN 阶段按仿真钟沿当前轨迹往前采样，速度是这一时刻的前馈。
        # 其他阶段只重复位置和航向。解锁、降落不在这里重发。
        while not stop.is_set():
            with lock:
                t_sim = sim["t"]
            origin = clock.origin
            elapsed = None if t_sim is None or origin is None else float(t_sim) - float(origin)
            with stream_lock:
                commands = list(streamed["commands"])
            for command in commands:
                tracked = None
                if elapsed is not None and getattr(command, "phase", "") == "PLAN":
                    tracked = session.stream_local_setpoint(command.namespace, elapsed)
                mode, setpoint = streamed_setpoint(command, tracked)
                pub = pubs[command.namespace]
                if mode is not None:
                    pub["ocm"].publish(_fill(OffboardControlMode(), mode))
                if setpoint is not None:
                    pub["sp"].publish(_fill(TrajectorySetpoint(), setpoint))
            stop.wait(SETPOINT_PERIOD_S)

    executor = None
    if args.lockstep:
        from rclpy.executors import SingleThreadedExecutor

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
        stepper = LockstepStepper(
            GzWorldPort(gz, args.world, sim, lock, motor_counts),
            list(slots),
            physics_step_s=float(args.physics_step),
        )
    else:
        threading.Thread(target=publish_stream, daemon=True).start()

    def odom_recv() -> dict:
        with lock:
            return {ns: state["odom_sim"] for ns, state in slots.items()}

    def lock_advance() -> str:
        # 握手或 clock 超时就失败关闭：写结果、放开世界，不丢下慢的 PX4 继续跑。
        try:
            stepper.advance(physics_dt)
        except TimeoutError as exc:
            return f"LOCKSTEP_TIMEOUT {exc}"
        return ""

    try:
        if stepper is not None:
            try:
                stepper.engage()
            except TimeoutError as exc:
                failure = f"LOCKSTEP_TIMEOUT {exc}"
                stepper = None
                raise _LockstepAbort() from exc
        while stepper is not None:
            # 票 B1：锁步主循环。世界停着时代理算多久都行；每拍算完再推进一拍（physics_dt）。
            wall = time.monotonic() - wall0
            if wall > args.wall_timeout:
                failure = f"WALL_TIMEOUT {args.wall_timeout}s"
                break
            t_sim = stepper.t
            boxes = {ns: inbox_for(ns) for ns in slots}
            if not preflight.allow_step(boxes, wall):
                if preflight.failure:
                    failure = preflight.failure
                    break
                failure = lock_advance()
                if failure:
                    break
                continue
            elapsed = clock.elapsed(float(t_sim))
            if elapsed > max_s and session.result()["outcome"] == "RUNNING":
                failure = f"MAX_S {max_s}"
                break
            skew.note(
                t_sim,
                {ns: (None if box is None else float(box.odometry.timestamp) * 1e-6) for ns, box in boxes.items()},
            )
            sim_skew.note(t_sim, odom_recv())
            commands = session.step(float(elapsed), boxes)
            last_elapsed = float(elapsed)
            if args.agent_delay_s > 0.0:
                time.sleep(float(args.agent_delay_s))
            for command in commands:
                pub = pubs[command.namespace]
                for fields in command.vehicle_commands:
                    pub["cmd"].publish(_fill(VehicleCommand(), fields))
                tracked = None
                if getattr(command, "phase", "") == "PLAN":
                    tracked = session.stream_local_setpoint(command.namespace, float(elapsed))
                mode, setpoint = streamed_setpoint(command, tracked)
                if mode is not None:
                    pub["ocm"].publish(_fill(OffboardControlMode(), mode))
                if setpoint is not None:
                    pub["sp"].publish(_fill(TrajectorySetpoint(), setpoint))
            snapshot = session.result()
            if snapshot["ok"] or snapshot.get("terminal"):
                break
            before = {ns: state["odom_n"] for ns, state in slots.items()}
            failure = lock_advance()
            if failure:
                break
            # 这一拍里 PX4 已经发过里程计；等它经 uXRCE 到达本进程再算下一拍（墙钟上限 50 ms）。
            odom_wait_limit_s = 0.10 if len(slots) >= 6 else 0.05
            w_odom = time.monotonic()
            while any(slots[ns]["odom_n"] <= before[ns] for ns in slots):
                if time.monotonic() - w_odom > odom_wait_limit_s:
                    odom_wait_timeouts += 1
                    break
                time.sleep(0.0005)
        while stepper is None and not failure:
            rclpy.spin_once(node, timeout_sec=0.02)
            wall = time.monotonic() - wall0
            if wall > args.wall_timeout:
                failure = f"WALL_TIMEOUT {args.wall_timeout}s"
                break
            with lock:
                t_sim = sim["t"]
            boxes = {ns: inbox_for(ns) for ns in slots}
            if not preflight.allow_step(boxes, wall):
                if preflight.failure:
                    failure = preflight.failure
                    break
                time.sleep(0.02)
                continue
            if t_sim is None:
                present = next((box for box in boxes.values() if box is not None), None)
                if present is None:
                    if last_elapsed is None:
                        time.sleep(0.02)
                        continue
                    elapsed = last_elapsed
                else:
                    elapsed = clock.elapsed(float(present.odometry.timestamp) * 1e-6)
            else:
                elapsed = clock.elapsed(float(t_sim))
            if elapsed > max_s and session.result()["outcome"] == "RUNNING":
                failure = f"MAX_S {max_s}"
                break
            fresh = last_elapsed is None or float(elapsed) + 1e-9 >= float(last_elapsed) + physics_dt
            if fresh:
                skew.note(
                    t_sim,
                    {
                        ns: (None if box is None else float(box.odometry.timestamp) * 1e-6)
                        for ns, box in boxes.items()
                    },
                )
                sim_skew.note(t_sim, odom_recv())
                commands = session.step(float(elapsed), boxes)
                last_elapsed = float(elapsed)
                if args.agent_delay_s > 0.0:
                    time.sleep(float(args.agent_delay_s))
            else:
                # 仿真时钟还没走到下一拍时，只重复上一拍的心跳和设定点，避免把解锁/降落再发一遍。
                commands = session.step(float(last_elapsed), boxes)
            with stream_lock:
                streamed["commands"] = list(commands)
            for command in commands:
                if fresh:
                    pub = pubs[command.namespace]
                    for fields in command.vehicle_commands:
                        pub["cmd"].publish(_fill(VehicleCommand(), fields))
            snapshot = session.result()
            if snapshot["ok"] or snapshot.get("terminal"):
                break
            time.sleep(0.02)
    except _LockstepAbort:
        pass
    finally:
        stop.set()
        if stepper is not None:
            stepper.release()
        if executor is not None:
            # 先停后台 spin 线程，避免 rclpy 关闭时线程里抛 ExternalShutdownException 导致进程 abort（rc=134）。
            executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()

    payload = session.result(adapter="px4-ros")
    # 锁存后的 preflight_ok=false 不会写入 failure，着陆上锁不能把已飞完的结果改成启动预检失败。
    if failure and not payload["ok"]:
        payload["outcome"] = "FAIL"
        payload["reason"] = failure
        payload["ok"] = False
    payload["positioning"] = positioning
    payload["mission_t0_s"] = clock.origin
    payload["clock_skew"] = skew.summary()
    payload["odom_sim_skew"] = sim_skew.summary()
    payload["agent_delay_s"] = float(args.agent_delay_s)
    if plan_diag is not None:
        payload["b2_plan_diag"] = plan_diag.summary()
    if getattr(args, "b2_deviations", None):
        payload["b2_deviations"] = list(args.b2_deviations)
    if getattr(session.fleet, "b3", None):
        payload["b3"] = {"config": dict(session.fleet.b3), "counts": dict(session.fleet.b3_counts)}
    if stepper is not None:
        payload["lockstep"] = stepper.stats.summary()
        payload["lockstep"]["odom_wait_timeouts"] = int(odom_wait_timeouts)
        payload["deviation"] = "偏离上游：仿真钟锁步"
    else:
        payload["lockstep"] = {"enabled": False}
    payload["scene_world"] = scene_world_plan(card, args)
    _write(args.out, payload)
    return outcome_exit_code(payload)


if __name__ == "__main__":
    sys.exit(main())
