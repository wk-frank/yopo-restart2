"""票 S4 / S4b：YOPO L0 与 PX4 Offboard 之间的 ENU/NED 闭环桥。

规划核心保持 ENU。PX4 ``VehicleOdometry`` / ``TrajectorySetpoint`` 是 NED。
两边的向量和航向只经过 ``yopo_swarm_core.px4``（``px4.py`` 模块头注明的 ADR-0011 边界）。
本文件不写第二套轴映射，也不导入 ``rclpy``。

离线闭环用 ``FakePx4Vehicle``：在 NED 里按设定点做点质量 + 一阶加速度滞后，
增益与 ``PointMassLag`` 相同。票 S4b 把同一份桥接接到 ROS 消息字段：
``VehicleOdometry``、深度图或障碍摘要进 ``Observation``，``TrajectorySetpoint`` 出去。
容器里的订阅和发布在 ``scripts/sitl/yopo_l0_live.py``，见 ``docs/research/s4b-sitl-live.md``。

实飞多机时，每架 PX4 的 ``VehicleOdometry`` 在自己的本地系，生成点附近就是原点。
``LiveFleetSession`` 加上该机场景卡起点的水平偏置，得到场景卡世界系；下发
``TrajectorySetpoint`` 时减回去。速度、加速度和航向是平移不变量，不改。
不订阅 ``vehicle_global_position``：本地里程计已经以生成点为原点，再引入一套
WGS84 参考并不更可靠。三机场景卡起点和 S2 直线生成点不一致时，容器按场景卡
起点生成（``fleet.py yopo-poses``），S2 的默认布局不动。

票 S5a：``policy=formation`` 时，PLAN 阶段不再调用 ``UpstreamPolicy``。
同一份 ``FormationFleet`` 在进程内 ``MessageBus`` 上交换提案 / ACK / COMMIT，
设定点仍是这条已提交计划采样后的 NED。不发布协议话题。编队钟从全体进入
PLAN 的那一拍记 0，起飞不计入 ``t_go``。心跳仍按仿真钟往前采样，到达截住
沿用 0.4 m / 0.2 m/s / 保持 1 s。
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from yopo_swarm_core import px4

from .bus import BusFaults, MessageBus
from .dynamics import State
from .formation import FormationFleet
from .inter_drone import samples_from_pva
from .policy import Observation, PolicyContext, UpstreamPolicy
from .scene import Box, Cylinder, SceneCard
from .sensor import R_BODY_CAM, CameraConfig, DepthCamera, DepthFrame
from .shadow import DroneShadow, aggregate, aggregate_drones, approved_pva, gate_status
from . import shadow_sitl

SCHEMA = "yopo-restart/s4-sitl-bridge/1"
SCHEMA_LIVE = "yopo-restart/s4b-sitl-live/1"
# 每个阶段留几拍。只留全局前 16 拍的话，全是 WARMUP，看不到规划方向。
TRACE_PER_PHASE = 8
# 与 scripts/sitl/fleet.py 的 SPAWN_SPACING_M 相同：S2 多机沿东向每隔 3 m 一架。
SITL_LINE_SPACING_M = 3.0
# ``--partial`` 只跑这么长的前缀，用来做离线校验，不等到达。
PARTIAL_PREFIX_S = 0.5
POSITIONING_MODES = ("default", "rtk")
LIVE_POLICIES = ("upstream", "formation", "crossing", "switch", "explore")
# 票 S7 / explore-sitl：带进程内 MessageBus 协同 fleet 的实飞策略。策略名必须和场景卡 task.type 相同。
FLEET_POLICIES = ("formation", "crossing", "switch", "explore")
VEHICLE_CMD_NAV_LAND = 21  # MAV_CMD_NAV_LAND，与 offboard_mission 同一条命令号
GCS_PORT_BASE = 18570
# 与 yopo_l0.runner 的总线盐相同。零故障时这条随机流不被消耗。
_BUS_SEED_SALT = 0xB05
# 仿真钟一次跳很远时，按 physics_dt 补协议拍，避免整窗提案被跳过。80 s / 0.02 s。
_FORMATION_SUBSTEPS = 6000
_REPO = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class SitlBridgeConfig:
    """一架飞机的桥接参数。``physics_dt`` 是设定点周期，``planner_dt`` 是重规划周期。"""

    physics_dt: float = 0.02
    planner_dt: float = 0.1
    hover_alt_m: float = 1.5
    goal_enu: tuple[float, float, float] | None = None
    policy_params: dict = field(default_factory=dict)
    drone_id: str = "uav0"
    seed: int = 0
    drone_index: int = 0
    n_drones: int = 1
    a_max: float = 6.0
    tau_s: float = 0.1
    kp: float = 4.0
    kd: float = 3.0
    px4_namespace: str = ""
    # formation 不跑上游网络。缺省仍构造 UpstreamPolicy，已有闭环不变。
    load_upstream: bool = True

    def __post_init__(self) -> None:
        if not self.physics_dt > 0.0 or not self.planner_dt > 0.0:
            raise ValueError(f"physics_dt 和 planner_dt 必须为正，收到 {self.physics_dt}, {self.planner_dt}")
        if self.n_drones < 1 or not 0 <= self.drone_index < self.n_drones:
            raise ValueError(f"drone_index={self.drone_index} 不在 0..{self.n_drones - 1}")
        if self.goal_enu is not None and len(self.goal_enu) != 3:
            raise ValueError(f"goal_enu 需要 3 个数，收到 {self.goal_enu!r}")


@dataclass(frozen=True)
class NeighborEnu:
    """邻机扩展口：世界系 ENU 位置/速度，以及可选的已提交轨迹采样（t, x, y, z）。"""

    drone_id: str
    position: np.ndarray
    velocity: np.ndarray
    samples: list | np.ndarray | None = None


@dataclass(frozen=True)
class BridgeOutput:
    """一次 ``command`` 的 ENU 期望和已经转成 NED 的 PX4 字段。"""

    now_s: float
    p_enu: np.ndarray
    v_enu: np.ndarray
    a_enu: np.ndarray
    yaw_enu: float
    setpoint: dict
    offboard: dict
    planned: bool
    plan_code: str
    roundtrip_error_m: float
    roundtrip_velocity_mps: float
    roundtrip_acceleration_mps2: float


@dataclass(frozen=True)
class OffboardCommand:
    """填进 px4_msgs 的字段。容器侧的 publisher 不在这个仓库模块里。"""

    trajectory_setpoint: dict
    offboard_control_mode: dict
    namespace: str = ""


@dataclass(frozen=True)
class FakePx4Config:
    """与 ``PointMassLag`` 相同的增益，积分在 NED 里做。"""

    tau_s: float = 0.1
    kp: float = 4.0
    kd: float = 3.0
    a_max: float = 6.0


def _as_vec3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} 必须是有限的 3 维向量，收到 shape={arr.shape}")
    return arr


def roundtrip_abs(enu: np.ndarray) -> float:
    """``ned_to_enu(enu_to_ned(v))`` 与 ``v`` 的最大绝对分量差。"""
    src = np.asarray(enu, dtype=float)
    back = px4.ned_to_enu(px4.enu_to_ned(src))
    return float(np.max(np.abs(back - src)))


def enu_state_from_ned(
    position_ned,
    velocity_ned,
    now_s: float,
    *,
    q_ned_frd=None,
    yaw_ned: float | None = None,
    acceleration_ned=None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """VehicleOdometry 口径的 NED 位置/速度 → ENU 位置、速度、加速度和航向。

    给了四元数就走 ``px4.odometry_to_enu``（姿态里的航向）。否则用 ``yaw_ned``。
    两者都缺则拒绝，不猜朝向。
    """
    position_ned = _as_vec3(position_ned, "position_ned")
    velocity_ned = _as_vec3(velocity_ned, "velocity_ned")
    if q_ned_frd is not None:
        enu = px4.odometry_to_enu(
            position_ned,
            q_ned_frd,
            velocity_ned,
            int(round(float(now_s) * 1e6)),
        )
        p = np.asarray(enu.position, dtype=float)
        v = np.asarray(enu.velocity, dtype=float)
        yaw = math.atan2(float(enu.R_wb[1, 0]), float(enu.R_wb[0, 0]))
    elif yaw_ned is not None:
        if not math.isfinite(float(yaw_ned)):
            raise ValueError("yaw_ned 不是有限值")
        p = np.asarray(px4.ned_to_enu(position_ned), dtype=float)
        v = np.asarray(px4.ned_to_enu(velocity_ned), dtype=float)
        yaw = float(px4.yaw_ned_to_enu(float(yaw_ned)))
    else:
        raise ValueError("需要 q_ned_frd 或 yaw_ned")
    if acceleration_ned is None:
        a = np.zeros(3)
    else:
        a = np.asarray(px4.ned_to_enu(_as_vec3(acceleration_ned, "acceleration_ned")), dtype=float)
    return p, v, a, yaw


def trajectory_setpoint_ned(p_enu, v_enu, a_enu, yaw_enu: float, now_s: float) -> dict:
    """ENU 的 (p, v, a, yaw) → ``px4.trajectory_setpoint`` 字段（NED）。"""
    pva = np.stack([_as_vec3(p_enu, "p_enu"), _as_vec3(v_enu, "v_enu"), _as_vec3(a_enu, "a_enu")])
    return px4.trajectory_setpoint(pva, float(yaw_enu), int(round(float(now_s) * 1e6)))


def offboard_command(out: BridgeOutput, namespace: str = "") -> OffboardCommand:
    return OffboardCommand(out.setpoint, out.offboard, namespace)


def _yaw_ned_quaternion(yaw_ned: float) -> np.ndarray:
    """水平机体、NED 航向 ``yaw_ned`` 的 FRD→NED 四元数。旋转是在 NED 里构造的。"""
    c, s = math.cos(yaw_ned), math.sin(yaw_ned)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return px4.rot_to_quat(rot)


def _camera_rng(seed: int, index: int, n: int) -> np.random.Generator:
    """与 ``runner.run`` 一样：``SeedSequence(seed).spawn(n)[index]``。"""
    child = np.random.SeedSequence(int(seed)).spawn(int(n))[int(index)]
    return np.random.default_rng(child)


def _git_commit() -> str:
    try:
        sha = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=_REPO, capture_output=True, text=True, check=True
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=_REPO,
            capture_output=True,
            text=True,
        ).stdout.strip()
        return sha + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


class YopoSitlBridge:
    """一架飞机：NED 里程计进来，NED ``TrajectorySetpoint`` 出去。

    ``command`` 每个 ``physics_dt`` 调一次。到了 ``planner_dt`` 才渲染深度并
    ``UpstreamPolicy.plan``；两次规划之间只对当前 ``_TrajPlan`` 采样。
    同一个 ``now_s`` 再调一次不重复采样（``_TrajPlan.sample`` 会推进偏航）。
    """

    def __init__(self, card: SceneCard, cfg: SitlBridgeConfig | None = None):
        self.cfg = cfg or SitlBridgeConfig()
        self.card = card
        drones = [d for d in card.card["drones"] if d["id"] == self.cfg.drone_id]
        if len(drones) != 1:
            raise ValueError(f"场景 {card.name} 里没有唯一的 {self.cfg.drone_id}")
        drone = drones[0]
        if self.cfg.goal_enu is not None:
            goal = np.asarray(self.cfg.goal_enu, dtype=float)
        else:
            raw = drone.get("goal_enu", drone.get("hover_enu", drone["start_enu"]))
            goal = np.asarray(raw, dtype=float)
        if goal.shape != (3,):
            raise ValueError(f"{self.cfg.drone_id} 的目标不是 3 维")
        self.goal = goal.copy()
        self.hover_alt_m = float(self.cfg.hover_alt_m)
        self.drone_id = self.cfg.drone_id
        self.start_enu = np.asarray(drone["start_enu"], dtype=float).copy()
        cam = CameraConfig.from_card(card.card["camera"])
        rng = _camera_rng(self.cfg.seed, self.cfg.drone_index, self.cfg.n_drones)
        self.camera = DepthCamera(cam, card.obstacles, rng)
        ctx = PolicyContext(
            self.drone_id,
            float(self.cfg.physics_dt),
            dict(self.cfg.policy_params),
            obstacles=card.obstacles,
            a_max=float(self.cfg.a_max),
        )
        self.policy = UpstreamPolicy(ctx) if self.cfg.load_upstream else None
        self.neighbors: tuple[NeighborEnu, ...] = ()
        self.neighbor_sample_deliveries = 0
        self._plan = None
        self._broadcast_samples: list | None = None
        self._last_plan_s: float | None = None
        self._last_sample_s: float | None = None
        self._last_output: BridgeOutput | None = None
        self._measured_enu: np.ndarray | None = None
        self.goal_hold = False
        self.last_observation: Observation | None = None
        self.plan_counts: dict[str, int] = {}

    @property
    def plan(self):
        """当前 ``UpstreamPolicy`` 交回的 ``_TrajPlan``（还没规划时是 None）。"""
        return self._plan

    def set_neighbors(self, neighbors: tuple[NeighborEnu, ...] | list[NeighborEnu]) -> None:
        """记下邻机 ENU 状态。采样在下一次 ``plan`` 之前交给 ``note_neighbor_samples``。"""
        self.neighbors = tuple(neighbors)

    def broadcast_samples(self) -> list | None:
        """最近一次规划广播的 ENU 采样。没有 ``pva`` 的计划是 None。"""
        return self._broadcast_samples

    def _deliver_neighbors(self) -> None:
        note = getattr(self.policy, "note_neighbor_samples", None)
        if note is None:
            return
        for nb in self.neighbors:
            if nb.samples is None:
                continue
            note(nb.drone_id, nb.samples)
            self.neighbor_sample_deliveries += 1

    def _due(self, now_s: float) -> bool:
        if self._plan is None or self._last_plan_s is None:
            return True
        return now_s - self._last_plan_s + 1e-8 >= self.cfg.planner_dt

    def command(
        self,
        position_ned,
        velocity_ned,
        now_s: float,
        *,
        q_ned_frd=None,
        yaw_ned: float | None = None,
        acceleration_ned=None,
        depth_frame: DepthFrame | None = None,
    ) -> BridgeOutput:
        now_s = float(now_s)
        if self.policy is None:
            raise RuntimeError(f"{self.drone_id} 没有上游策略，编队设定点不走 command()")
        if self._last_output is not None and self._last_sample_s is not None:
            if abs(now_s - self._last_sample_s) <= 1e-12:
                return self._last_output
        if depth_frame is not None and not isinstance(depth_frame, DepthFrame):
            raise TypeError(f"depth_frame 必须是 DepthFrame，收到 {type(depth_frame).__name__}")
        p, v, a_state, yaw = enu_state_from_ned(
            position_ned,
            velocity_ned,
            now_s,
            q_ned_frd=q_ned_frd,
            yaw_ned=yaw_ned,
            acceleration_ned=acceleration_ned,
        )
        # 实飞主循环一规划就要 1 s 以上，sample() 里的偏航只按 physics_dt=0.02 走一步。
        # 机头因此一直朝东，网络的前向轨迹被转到 +x。间隔够大时先按同一限速把期望航向补到目标方向，
        # 这一拍的机体旋转用补过的航向，位置设定点才朝目标而不是朝机头。
        # 假动力学每拍 0.02 s，进不了这个分支。
        gap = 0.0 if self._last_sample_s is None else now_s - float(self._last_sample_s)
        stalled = gap >= STALL_S
        if stalled:
            # plan_from_reference 会用旧轨迹在 now 的点当下一拍的头。
            # now 已经跳了 1 s 以上，那个点飞机还没飞到。先把参考锚回测量状态。
            self._anchor_reference(now_s, p, v)
        yaw = self._catch_up_yaw(now_s, p, yaw)
        planned = False
        if self._due(now_s):
            self._deliver_neighbors()
            frame = self.camera.capture(p, yaw, now_s) if depth_frame is None else depth_frame
            state = State(now_s, p.copy(), v.copy(), a_state.copy(), float(yaw))
            obs = Observation(frame, state, self.goal.copy(), now_s)
            self.last_observation = obs
            self._plan = self.policy.plan(obs)
            code = str(self.policy.last_code)
            self.plan_counts[code] = self.plan_counts.get(code, 0) + 1
            self._broadcast_samples = samples_from_pva(self._plan, now_s)
            self._last_plan_s = now_s
            planned = True
        # 这一拍用规划时刻 now。间隔里的点由心跳按仿真钟往前采样，不在这里把 1.2 s 一次走完。
        p_ref, v_ref, a_ref, yaw_ref = self._plan.sample(now_s)
        self._measured_enu = np.asarray(p, dtype=float).copy()
        self.goal_hold = False
        if stalled:
            p_ref, v_ref, a_ref, self.goal_hold = clamp_stalled_setpoint(
                p, p_ref, v_ref, a_ref, self.goal, self._arrive_radius(), self.start_enu
            )
        p_ref = np.asarray(p_ref, dtype=float).copy()
        v_ref = np.asarray(v_ref, dtype=float).copy()
        a_ref = np.asarray(a_ref, dtype=float).copy()
        yaw_ref = float(yaw_ref)
        setpoint = trajectory_setpoint_ned(p_ref, v_ref, a_ref, yaw_ref, now_s)
        offboard = px4.offboard_control_mode(setpoint["timestamp"])
        out = BridgeOutput(
            now_s=now_s,
            p_enu=p_ref,
            v_enu=v_ref,
            a_enu=a_ref,
            yaw_enu=yaw_ref,
            setpoint=setpoint,
            offboard=offboard,
            planned=planned,
            plan_code=str(self.policy.last_code),
            roundtrip_error_m=roundtrip_abs(p_ref),
            roundtrip_velocity_mps=roundtrip_abs(v_ref),
            roundtrip_acceleration_mps2=roundtrip_abs(a_ref),
        )
        self._last_sample_s = now_s
        self._last_output = out
        return out

    def _catch_up_yaw(self, now_s: float, position_enu, yaw_odom: float) -> float:
        """时间跳变时把期望航向按限速转到目标。正常 0.02 s 节拍原样返回里程计航向。"""
        last = self._last_sample_s
        plan = self._plan
        if last is None or plan is None or not hasattr(plan, "yaw"):
            return float(yaw_odom)
        dt = float(now_s) - float(last)
        if dt < 0.5:
            return float(yaw_odom)
        from yopo_swarm_core.yaw import calculate_yaw

        goal_dir = np.asarray(self.goal, dtype=float) - np.asarray(position_enu, dtype=float)
        if float(np.linalg.norm(goal_dir)) <= 1e-6:
            return float(yaw_odom)
        plan.yaw = float(calculate_yaw(goal_dir, goal_dir, float(plan.yaw), dt)[0])
        return float(plan.yaw)

    def _arrive_radius(self) -> float:
        params = getattr(self.policy, "p", None)
        if isinstance(params, dict) and "arrive_radius_m" in params:
            return float(params["arrive_radius_m"])
        return 1.5

    def _anchor_reference(self, now_s: float, position, velocity) -> None:
        """把参考头放到测量位置和测量速度上，避免下一拍从旧轨迹的未来点接上。"""
        plan = getattr(self.policy, "plan_obj", None)
        if plan is None or not hasattr(plan, "traj"):
            return
        from yopo_swarm_core.plan import quintic
        from yopo_swarm_core.trajectory import PiecewisePolynomial

        p = np.asarray(position, dtype=float).reshape(3)
        v = np.asarray(velocity, dtype=float).reshape(3)
        head = np.array([p, v, np.zeros(3)])
        end = np.array([p + v, v, np.zeros(3)])
        plan.traj = PiecewisePolynomial(float(now_s), [quintic(head, end, 1.0)])

    def tracking_sample(self, now_s: float):
        """心跳用：轨迹在 ``now_s`` 的点。没有计划时是 None。不推进偏航。"""
        measured = self.start_enu if self._measured_enu is None else self._measured_enu
        return sample_plan_at(self._plan, now_s, measured, self.goal, self.start_enu, self._arrive_radius())


class FakePx4Vehicle:
    """按 NED 设定点推进的点质量。加速度一阶滞后，速度、位置用新加速度积分。

    公式与 ``PointMassLag.step`` 相同，坐标已经是设定点所在的 NED，不再做一次轴映射。
    """

    def __init__(self, position_enu, velocity_enu=None, yaw_enu: float = 0.0, cfg: FakePx4Config | None = None):
        self.cfg = cfg or FakePx4Config()
        if not self.cfg.tau_s > 0.0 or not self.cfg.a_max > 0.0:
            raise ValueError("tau_s 和 a_max 必须为正")
        self.p_ned = np.asarray(px4.enu_to_ned(_as_vec3(position_enu, "position_enu")), dtype=float).copy()
        v0 = np.zeros(3) if velocity_enu is None else _as_vec3(velocity_enu, "velocity_enu")
        self.v_ned = np.asarray(px4.enu_to_ned(v0), dtype=float).copy()
        self.a_ned = np.zeros(3)
        if not math.isfinite(float(yaw_enu)):
            raise ValueError("yaw_enu 不是有限值")
        self.yaw_ned = float(px4.yaw_enu_to_ned(float(yaw_enu)))

    def quaternion(self) -> np.ndarray:
        return _yaw_ned_quaternion(self.yaw_ned)

    def odometry(self) -> dict:
        q = self.quaternion()
        return {
            "position_ned": self.p_ned.copy(),
            "velocity_ned": self.v_ned.copy(),
            "acceleration_ned": self.a_ned.copy(),
            "yaw_ned": float(self.yaw_ned),
            "q_ned_frd": q.copy(),
        }

    def step(self, setpoint: dict, dt: float) -> None:
        if not dt > 0.0:
            raise ValueError(f"dt 必须为正，收到 {dt}")
        p_sp = _as_vec3(setpoint["position"], "setpoint.position")
        v_sp = _as_vec3(setpoint["velocity"], "setpoint.velocity")
        a_sp = _as_vec3(setpoint["acceleration"], "setpoint.acceleration")
        a_cmd = a_sp + self.cfg.kp * (p_sp - self.p_ned) + self.cfg.kd * (v_sp - self.v_ned)
        n = float(np.linalg.norm(a_cmd))
        if n > self.cfg.a_max:
            a_cmd = a_cmd * (self.cfg.a_max / n)
        alpha = 1.0 - math.exp(-dt / self.cfg.tau_s)
        self.a_ned = self.a_ned + alpha * (a_cmd - self.a_ned)
        self.v_ned = self.v_ned + self.a_ned * dt
        self.p_ned = self.p_ned + self.v_ned * dt
        yaw = setpoint.get("yaw")
        if yaw is not None and math.isfinite(float(yaw)):
            self.yaw_ned = float(yaw)


def _vec_list(value) -> list[float]:
    return [float(x) for x in np.asarray(value, dtype=float).reshape(-1)]


def _hz(dt: float) -> float:
    hz = 1.0 / float(dt)
    rounded = round(hz)
    if abs(hz - rounded) <= 1e-6:
        return float(rounded)
    return hz


def run_closed_loop(
    card: SceneCard,
    *,
    seed: int = 0,
    max_s: float = 15.0,
    partial: bool = False,
    partial_s: float = PARTIAL_PREFIX_S,
    physics_dt: float = 0.02,
    planner_dt: float = 0.1,
    ros_messages: bool = False,
) -> dict:
    """场景卡上的全体飞机：假 PX4 NED 闭环。多机在每次规划前交换上一拍的采样。

    ``partial`` 把时域收成 ``min(max_s, partial_s)``。碰撞或全体按场景判据到达并保持后提前停。
    本结果始终 ``report_only``，不采用场景卡上的冻结门。
    """
    if not max_s > 0.0:
        raise ValueError(f"max_s 必须为正，收到 {max_s}")
    if partial and not partial_s > 0.0:
        raise ValueError(f"partial_s 必须为正，收到 {partial_s}")
    horizon = min(float(max_s), float(partial_s)) if partial else float(max_s)
    task = card.card.get("task") or {}
    if not partial and "duration_s" in task:
        horizon = min(horizon, float(task["duration_s"]))
    dyn = card.card.get("dynamics") or {}
    crit = card.card.get("criteria") or {}
    body = float(crit.get("body_radius_m", 0.35))
    track_goal = "goal_tolerance_m" in crit
    if track_goal:
        gtol = float(crit["goal_tolerance_m"])
        gspeed = float(crit["goal_speed_mps"])
        ghold = float(crit["goal_hold_s"])
    else:
        gtol = float(crit.get("hover_tolerance_m", 0.3))
        gspeed = math.inf
        ghold = 0.0
    drones = list(card.card["drones"])
    n = len(drones)
    params = dict(card.card.get("policy_params") or {})
    a_max = float(dyn.get("a_max", 6.0))
    tau_s = float(dyn.get("tau_s", 0.1))
    kp = float(dyn.get("kp", 4.0))
    kd = float(dyn.get("kd", 3.0))
    bridges: list[YopoSitlBridge] = []
    vehicles: list[FakePx4Vehicle] = []
    for i, drone in enumerate(drones):
        start = drone["start_enu"]
        cfg = SitlBridgeConfig(
            physics_dt=physics_dt,
            planner_dt=planner_dt,
            hover_alt_m=float(start[2]),
            goal_enu=tuple(drone.get("goal_enu", drone.get("hover_enu", start))),
            policy_params=params,
            drone_id=drone["id"],
            seed=int(seed),
            drone_index=i,
            n_drones=n,
            a_max=a_max,
            tau_s=tau_s,
            kp=kp,
            kd=kd,
            px4_namespace="" if n == 1 else f"px4_{i + 1}",
        )
        bridges.append(YopoSitlBridge(card, cfg))
        vehicles.append(
            FakePx4Vehicle(
                start, yaw_enu=0.0, cfg=FakePx4Config(tau_s=tau_s, kp=kp, kd=kd, a_max=a_max)
            )
        )

    wall0 = time.perf_counter()
    n_steps = int(round(horizon / physics_dt))
    rt_p = 0.0
    rt_v = 0.0
    rt_a = 0.0
    min_clear = math.inf
    min_obs = math.inf
    min_inter: float | None = None
    outcome = None
    reason = ""
    goal_since = [None] * n
    arrival = [None] * n
    final_p = [bridges[i].start_enu.copy() for i in range(n)]
    final_err = [float(np.linalg.norm(bridges[i].start_enu - bridges[i].goal)) for i in range(n)]
    ran = False
    k_last = -1
    start_positions = []
    for bridge in bridges:
        dist0 = float(card.min_distance(bridge.start_enu))
        min_obs = min(min_obs, dist0)
        min_clear = min(min_clear, dist0 - body)
        start_positions.append(np.asarray(bridge.start_enu, dtype=float))

    def _note_inter(positions: list[np.ndarray]) -> None:
        nonlocal min_inter, outcome, reason
        if n < 2:
            return
        for i in range(n):
            for j in range(i + 1, n):
                dist = float(np.linalg.norm(positions[i] - positions[j]))
                min_inter = dist if min_inter is None else min(min_inter, dist)
                if outcome is None and dist <= 2.0 * body:
                    outcome, reason = "COLLISION", f"{drones[i]['id']}-{drones[j]['id']}:INTER_DRONE"

    _note_inter(start_positions)
    for k in range(n_steps):
        if outcome is not None:
            break
        ran = True
        k_last = k
        now = round(k * physics_dt, 10)
        for i, bridge in enumerate(bridges):
            others = []
            for j, other in enumerate(bridges):
                if j == i:
                    continue
                odom_j = vehicles[j].odometry()
                if ros_messages:
                    decoded_j = decode_vehicle_odometry(
                        odometry_message_from_odometry(odom_j, int(round(now * 1e6))), now_s=now
                    )
                    pj, vj = decoded_j["position_enu"], decoded_j["velocity_enu"]
                else:
                    pj, vj, _, _ = enu_state_from_ned(
                        odom_j["position_ned"],
                        odom_j["velocity_ned"],
                        now,
                        q_ned_frd=odom_j["q_ned_frd"],
                    )
                others.append(NeighborEnu(other.drone_id, pj, vj, other.broadcast_samples()))
            bridge.set_neighbors(others)
        for i, bridge in enumerate(bridges):
            odom = vehicles[i].odometry()
            depth_frame = None
            position_ned = odom["position_ned"]
            velocity_ned = odom["velocity_ned"]
            q_ned = odom["q_ned_frd"]
            yaw_ned = odom["yaw_ned"]
            acceleration_ned = odom["acceleration_ned"]
            if ros_messages:
                decoded_i = decode_vehicle_odometry(
                    odometry_message_from_odometry(odom, int(round(now * 1e6))), now_s=now
                )
                position_ned = decoded_i["position_ned"]
                velocity_ned = decoded_i["velocity_ned"]
                q_ned = decoded_i["q_ned_frd"]
                acceleration_ned = decoded_i["acceleration_ned"]
                yaw_ned = None
                # 只有到了重规划拍才渲染。否则会多抽一次相机噪声，和直接 command 的随机序列错开。
                if bridge._due(now):
                    preview = bridge.camera.capture(decoded_i["position_enu"], decoded_i["yaw_enu"], now)
                    depth_frame = decode_depth_image(encode_depth_image(preview))
            out = bridge.command(
                position_ned,
                velocity_ned,
                now,
                q_ned_frd=q_ned,
                yaw_ned=yaw_ned,
                acceleration_ned=acceleration_ned,
                depth_frame=depth_frame,
            )
            vehicles[i].step(out.setpoint, physics_dt)
            odom_enu = px4.ned_to_enu(odom["position_ned"])
            rt_p = max(rt_p, out.roundtrip_error_m, roundtrip_abs(odom_enu))
            rt_v = max(rt_v, out.roundtrip_velocity_mps)
            rt_a = max(rt_a, out.roundtrip_acceleration_mps2)
        t_state = round((k + 1) * physics_dt, 10)
        positions = []
        for i, vehicle in enumerate(vehicles):
            odom = vehicle.odometry()
            p_enu, v_enu, _, _ = enu_state_from_ned(
                odom["position_ned"], odom["velocity_ned"], t_state, q_ned_frd=odom["q_ned_frd"]
            )
            positions.append(p_enu)
            dist = float(card.min_distance(p_enu))
            clearance = dist - body
            min_obs = min(min_obs, dist)
            min_clear = min(min_clear, clearance)
            err = float(np.linalg.norm(p_enu - bridges[i].goal))
            final_p[i] = p_enu
            final_err[i] = err
            if outcome is None and dist <= body:
                outcome, reason = "COLLISION", f"{bridges[i].drone_id}:OBSTACLE"
            elif outcome is None and arrival[i] is None:
                speed = float(np.linalg.norm(v_enu))
                if track_goal:
                    if err <= gtol and speed <= gspeed:
                        goal_since[i] = t_state if goal_since[i] is None else goal_since[i]
                        if t_state - goal_since[i] >= ghold - 1e-9:
                            arrival[i] = goal_since[i]
                    else:
                        goal_since[i] = None
                elif err <= gtol:
                    arrival[i] = t_state
        _note_inter(positions)
        if outcome is not None:
            break
        if all(t is not None for t in arrival):
            outcome, reason = "SUCCESS", "GOAL_REACHED_AND_HELD"
            break

    if outcome is None and track_goal:
        outcome, reason = ("PARTIAL", "PREFIX") if partial else ("TIMEOUT", "GOAL_NOT_REACHED")
    if outcome is None:
        worst = max(final_err) if final_err else 0.0
        outcome, reason = ("SUCCESS", "HOVER_HELD") if worst <= gtol else ("HOVER_DRIFT", "HOVER_TOLERANCE")

    arrived_n = sum(t is not None for t in arrival)
    drone_rows = []
    for i, bridge in enumerate(bridges):
        drone_rows.append(
            {
                "id": bridge.drone_id,
                "px4_namespace": bridge.cfg.px4_namespace,
                "start_enu": _vec_list(bridge.start_enu),
                "goal_enu": _vec_list(bridge.goal),
                "hover_alt_m": bridge.hover_alt_m,
                "arrived": arrival[i] is not None,
                "arrival_time_s": arrival[i],
                "final_position_enu": _vec_list(final_p[i]),
                "final_goal_error_m": float(final_err[i]),
                "plans": int(sum(bridge.plan_counts.values())),
                "plan_codes": dict(bridge.plan_counts),
                "neighbor_sample_deliveries": int(bridge.neighbor_sample_deliveries),
            }
        )
    clear_out = None if min_clear is math.inf else float(min_clear)
    obs_out = None if min_obs is math.inf else float(min_obs)
    return {
        "schema": SCHEMA,
        "report_only": True,
        "criteria_frozen": False,
        "scene_card_criteria_frozen": bool(crit.get("frozen", False)),
        "commit": _git_commit(),
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": int(seed),
        "partial": bool(partial),
        "max_s": float(max_s),
        "horizon_s": float(horizon),
        "sim_duration_s": (k_last + 1) * physics_dt if ran else 0.0,
        "n_drones": n,
        "policy": "upstream",
        "adapter": "fake-px4",
        "coordinate_conversion": "yopo_swarm_core.px4",
        "physics_dt_s": float(physics_dt),
        "planner_dt_s": float(planner_dt),
        "setpoint_hz": _hz(physics_dt),
        "planner_hz": _hz(planner_dt),
        "outcome": outcome,
        "reason": reason,
        "arrived": arrived_n,
        "arrival_rate": float(arrived_n) / float(n),
        "arrival_time_s": max(arrival) if track_goal and arrived_n == n else None,
        "collisions": 0 if outcome != "COLLISION" else 1,
        "collision_reason": reason if outcome == "COLLISION" else None,
        "min_surface_clearance_m": clear_out,
        "min_obstacle_distance_m": obs_out,
        "min_inter_drone_distance_m": min_inter,
        "body_radius_m": body,
        "enu_ned_roundtrip_max_error_m": float(rt_p),
        "enu_ned_roundtrip_max_velocity_mps": float(rt_v),
        "enu_ned_roundtrip_max_acceleration_mps2": float(rt_a),
        "neighbor_sample_deliveries": int(sum(b.neighbor_sample_deliveries for b in bridges)),
        "drones": drone_rows,
        "wall_time_s": time.perf_counter() - wall0,
    }


def normalize_positioning(value: str | None) -> str:
    """定位模式只记进结果。``rtk`` 仍由容器启动时已有的白名单环境注入，桥不写参数。"""
    text = "default" if value is None or str(value).strip() == "" else str(value).strip().lower()
    if text not in POSITIONING_MODES:
        raise ValueError(f"positioning 只要 default 或 rtk，收到 {value!r}")
    return text


def apply_explore_velocity_override(
    card: SceneCard, velocity_mps: float | None
) -> tuple[SceneCard, list[str]]:
    """在内存里覆写 explore 场景卡 ``task.velocity_mps``（不改落盘 JSON，也不写 PX4 参数）。"""
    if velocity_mps is None:
        return card, []
    vel = float(velocity_mps)
    if not math.isfinite(vel) or vel <= 0.0:
        raise ValueError(f"velocity_mps 必须是正有限数，收到 {velocity_mps!r}")
    task = dict(card.card.get("task") or {})
    if str(task.get("type") or "") != "explore":
        raise ValueError(f"--velocity-mps 只适用于 task.type=explore 的场景卡，收到 {task.get('type')!r}")
    task["velocity_mps"] = vel
    dyn = dict(card.card.get("dynamics") or {})
    cur_a = float(dyn.get("a_max", 6.0))
    ratio = vel / 6.0
    needed_a = round(1.18 * 6.0 * ratio * ratio, 3)
    notes = [f"explore.velocity_mps={vel:g}"]
    if needed_a > cur_a:
        dyn["a_max"] = needed_a
        notes.append(f"dynamics.a_max={needed_a:g}")
    updated = {**card.card, "task": task, "dynamics": dyn}
    return SceneCard(card.path, updated, card.obstacles), notes


def resolve_live_policy(card: SceneCard, policy: str | None) -> str:
    """实飞策略。formation / crossing / switch / explore 只接同名 task.type 的卡，机间消息留在进程内总线。"""
    name = "upstream" if policy is None or str(policy).strip() == "" else str(policy).strip().lower()
    if name not in LIVE_POLICIES:
        raise ValueError(f"policy 只要 {' 或 '.join(LIVE_POLICIES)}，收到 {policy!r}")
    task = str((card.card.get("task") or {}).get("type") or "")
    if name in FLEET_POLICIES and task != name:
        raise ValueError(f"policy={name} 只适用于 task.type 为 {name} 的场景卡")
    if task in FLEET_POLICIES and name != task:
        raise ValueError(f"{task} 场景卡需要 policy={task}。机间消息走进程内 MessageBus，不走上游 YOPO")
    return name


def _fleet_for(name: str, card: SceneCard, physics_dt: float):
    """按策略名建同一套 L0 协同 fleet。crossing / switch / explore 只在用到时导入。"""
    if name == "formation":
        return FormationFleet.from_card(card.card, float(physics_dt))
    if name == "crossing":
        from .crossing import CrossingFleet

        return CrossingFleet.from_card(card.card, float(physics_dt))
    if name == "switch":
        from .switch import SwitchFleet

        return SwitchFleet.from_card(card.card, float(physics_dt))
    if name == "explore":
        from .explore import ExploreFleet

        return ExploreFleet.from_card(card.card, float(physics_dt))
    return None


class ClockSkewMeter:
    """票 S7：每机里程计时间戳相对仿真钟的偏移。只报数，不进判据。

    每拍记 ``odom_ts - t_sim``。单机的极差是消息龄抖动加漂移；同一拍各机偏移的极差是机间时钟偏差。
    """

    def __init__(self) -> None:
        self.offsets: dict[str, list[float]] = {}
        self.spreads: list[float] = []
        self.zero_stamps = 0

    def note(self, t_sim: float | None, odom_ts_s: dict[str, float | None]) -> None:
        if t_sim is None:
            return
        row = []
        for ns, ts in odom_ts_s.items():
            if ts is None or not float(ts) > 0.0:
                # PX4 退出或重启时里程计时间戳会是 0，不算偏差样本。
                self.zero_stamps += ts is not None
                continue
            off = float(ts) - float(t_sim)
            self.offsets.setdefault(ns, []).append(off)
            row.append(off)
        if len(row) >= 2:
            self.spreads.append(max(row) - min(row))

    def summary(self) -> dict | None:
        if not self.offsets:
            return None
        per = {}
        for ns, vals in sorted(self.offsets.items()):
            arr = np.asarray(vals, dtype=float)
            per[ns or "/"] = {
                "samples": int(arr.size),
                "offset_median_s": float(np.median(arr)),
                "offset_range_s": float(arr.max() - arr.min()),
                "offset_p95_abs_dev_s": float(np.percentile(np.abs(arr - np.median(arr)), 95)),
            }
        spreads = np.asarray(self.spreads, dtype=float)
        return {
            "per_vehicle": per,
            "inter_vehicle_spread_median_s": None if spreads.size == 0 else float(np.median(spreads)),
            "inter_vehicle_spread_p95_s": None if spreads.size == 0 else float(np.percentile(spreads, 95)),
            "inter_vehicle_spread_max_s": None if spreads.size == 0 else float(spreads.max()),
            "zero_stamps_skipped": int(self.zero_stamps),
        }


def vehicle_identity(drone_index: int, n_drones: int) -> dict:
    """与 ``fleet.fleet_specs`` 对齐的 namespace、MAV_SYS_ID 和 GCS 端口。

    1 架是 instance 0：空 namespace、system 1、端口 18570。
    多架从 instance 1 起：``px4_1`` 的 system 是 2，端口 18571。
    """
    if isinstance(n_drones, bool) or not isinstance(n_drones, int) or n_drones < 1:
        raise ValueError(f"n_drones 必须是正整数，收到 {n_drones!r}")
    if not 0 <= int(drone_index) < n_drones:
        raise ValueError(f"drone_index={drone_index} 不在 0..{n_drones - 1}")
    if n_drones == 1:
        return {"namespace": "", "mav_sys_id": 1, "gcs_port": GCS_PORT_BASE, "instance": 0}
    instance = int(drone_index) + 1
    return {
        "namespace": f"px4_{instance}",
        "mav_sys_id": instance + 1,
        "gcs_port": GCS_PORT_BASE + instance,
        "instance": instance,
    }


def _perception_topic(namespace: str, kind: str) -> str:
    if kind not in ("depth_image", "obstacle_summary"):
        raise ValueError(f"未知感知话题 {kind}")
    ns = namespace.strip("/")
    prefix = f"/{ns}" if ns else ""
    return f"{prefix}/yopo/{kind}"


def vehicle_topics(namespace: str) -> dict[str, str]:
    """一架飞机的 PX4 话题和本桥的感知话题。空 namespace 就是 ``/fmu/...``。"""
    return {
        "vehicle_odometry": px4.topic("out", "vehicle_odometry", namespace),
        "vehicle_status": px4.topic("out", "vehicle_status", namespace),
        "vehicle_land_detected": px4.topic("out", "vehicle_land_detected", namespace),
        "trajectory_setpoint": px4.topic("in", "trajectory_setpoint", namespace),
        "offboard_control_mode": px4.topic("in", "offboard_control_mode", namespace),
        "vehicle_command": px4.topic("in", "vehicle_command", namespace),
        "depth_image": _perception_topic(namespace, "depth_image"),
        "obstacle_summary": _perception_topic(namespace, "obstacle_summary"),
    }


def vehicle_command_message(kind: str, timestamp_us: int, target_system: int) -> dict:
    """解锁、offboard、降落。目标 system 用该机的 MAV_SYS_ID，不广播，也不改参数。"""
    if kind == "offboard":
        return px4.offboard_mode_command(timestamp_us, target_system)
    if kind == "arm":
        return px4.arm_command(timestamp_us, target_system, arm=True)
    if kind == "land":
        return px4.vehicle_command(
            VEHICLE_CMD_NAV_LAND, timestamp_us, target_system, param4=math.nan
        )
    raise ValueError(f"未知 VehicleCommand {kind}")


def land_status_done(command: dict, *, landed: bool, armed: bool) -> str:
    """降落命令发出后，只有「已落地且上锁」才算收尾完成。"""
    if int(command.get("command", -1)) != VEHICLE_CMD_NAV_LAND:
        return "LAND"
    if landed and not armed:
        return "DONE"
    return "LAND"


def _msg_get(msg, name: str, default=None):
    if msg is None:
        return default
    if isinstance(msg, dict):
        return msg[name] if name in msg else default
    return getattr(msg, name, default)


def odometry_message_from_odometry(odom: dict, timestamp_us: int) -> dict:
    """``FakePx4Vehicle.odometry`` → ``px4_msgs/VehicleOdometry`` 字段（仍是 NED）。"""
    msg = {
        "position": _vec_list(odom["position_ned"]),
        "velocity": _vec_list(odom["velocity_ned"]),
        "q": _vec_list(odom["q_ned_frd"]),
        "timestamp": int(timestamp_us),
        "pose_frame": int(px4.POSE_FRAME_NED),
        "velocity_frame": int(px4.VELOCITY_FRAME_NED),
    }
    if odom.get("acceleration_ned") is not None:
        msg["acceleration"] = _vec_list(odom["acceleration_ned"])
    if odom.get("yaw_ned") is not None and math.isfinite(float(odom["yaw_ned"])):
        msg["yaw"] = float(odom["yaw_ned"])
    return msg


def decode_vehicle_odometry(msg, now_s: float | None = None) -> dict:
    """``VehicleOdometry`` 字段 → ENU 位置/速度/加速度/航向。帧不对或有 NaN 时拒绝。"""
    position = _as_vec3(_msg_get(msg, "position"), "odometry.position")
    velocity = _as_vec3(_msg_get(msg, "velocity"), "odometry.velocity")
    quat = _msg_get(msg, "q")
    if quat is None:
        raise ValueError("VehicleOdometry 缺少 q")
    quat = np.asarray(quat, dtype=float).reshape(-1)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        raise ValueError("VehicleOdometry.q 必须是有限的 4 维四元数")
    pose_frame = int(_msg_get(msg, "pose_frame", px4.POSE_FRAME_NED))
    vel_frame = int(_msg_get(msg, "velocity_frame", px4.VELOCITY_FRAME_NED))
    timestamp = _msg_get(msg, "timestamp")
    if timestamp is None:
        if now_s is None:
            raise ValueError("VehicleOdometry 缺少 timestamp")
        timestamp_us = int(round(float(now_s) * 1e6))
    else:
        timestamp_us = int(timestamp)
    enu = px4.odometry_to_enu(position, quat, velocity, timestamp_us, pose_frame, vel_frame)
    raw_acc = _msg_get(msg, "acceleration")
    if raw_acc is None:
        acceleration_ned = None
        acceleration_enu = np.zeros(3)
    else:
        acceleration_ned = _as_vec3(raw_acc, "odometry.acceleration")
        acceleration_enu = np.asarray(px4.ned_to_enu(acceleration_ned), dtype=float)
    yaw = math.atan2(float(enu.R_wb[1, 0]), float(enu.R_wb[0, 0]))
    return {
        "position_enu": np.asarray(enu.position, dtype=float).copy(),
        "velocity_enu": np.asarray(enu.velocity, dtype=float).copy(),
        "acceleration_enu": acceleration_enu.copy(),
        "yaw_enu": float(yaw),
        "rotation_enu_flu": np.asarray(enu.R_wb, dtype=float).copy(),
        "timestamp_us": timestamp_us,
        "position_ned": position.copy(),
        "velocity_ned": velocity.copy(),
        "acceleration_ned": None if acceleration_ned is None else acceleration_ned.copy(),
        "q_ned_frd": quat.copy(),
    }


def world_offset_enu(start_enu) -> np.ndarray:
    """场景卡起点的水平平移。本地 z 已是离地高度，不再叠起点高度。"""
    start = _as_vec3(start_enu, "start_enu")
    return np.array([float(start[0]), float(start[1]), 0.0], dtype=float)


def sitl_line_spawn_xy(n: int) -> list[tuple[float, float]]:
    """S2 直线生成点。单机是原点；多机是 (0, 0)、(0, 3)、…。"""
    count = int(n)
    if isinstance(n, bool) or count < 1:
        raise ValueError(f"n 必须为正整数，收到 {n!r}")
    if count == 1:
        return [(0.0, 0.0)]
    return [(0.0, float(i) * SITL_LINE_SPACING_M) for i in range(count)]


def _fmt_xy(points) -> str:
    return "[" + ", ".join(f"({x:g}, {y:g})" for x, y in points) + "]"


def frame_note(card: SceneCard) -> str:
    """场景卡起点和 S2 直线生成点不一致时说明，并以场景卡为准。一致时是空串。"""
    drones = list(card.card["drones"])
    starts = [(float(d["start_enu"][0]), float(d["start_enu"][1])) for d in drones]
    line = sitl_line_spawn_xy(len(drones))
    if all(abs(a - c) <= 1e-6 and abs(b - d) <= 1e-6 for (a, b), (c, d) in zip(starts, line)):
        return ""
    return (
        "场景卡起点与 S2 直线生成点不一致，实飞以场景卡为准生成。"
        f" 场景卡 xy={_fmt_xy(starts)}；S2 直线 xy={_fmt_xy(line)}。"
        "各机 VehicleOdometry 在自己的本地系，桥接加上场景卡起点的水平偏置得到统一世界系，设定点下发时减回去。"
    )


def apply_world_offset(decoded: dict, offset_enu) -> dict:
    """本地 ENU/NED 位置加上生成点偏置。速度、加速度、航向不动。"""
    offset = np.asarray(offset_enu, dtype=float).reshape(3)
    out = dict(decoded)
    out["position_enu"] = np.asarray(decoded["position_enu"], dtype=float) + offset
    out["position_ned"] = np.asarray(decoded["position_ned"], dtype=float) + np.asarray(
        px4.enu_to_ned(offset), dtype=float
    )
    return out


# 实飞主循环一次规划常常超过 0.5 s。假动力学是 0.02 s，不进这条。
STALL_S = 0.5


def sample_plan_at(plan, now_s: float, measured, goal, start, radius: float):
    """按规划时间戳取 ``plan.pva(now_s)``。

    进入到达半径，或者这个点已经越过起点到目标的那一段，就改成目标点悬停。
    不调用 ``plan.sample``：那个函数每调一次都按 physics_dt 推进一步偏航，
    心跳 20 Hz 来读会把航向转快。返回 (position, velocity, acceleration, yaw, hold)。
    """
    if plan is None or not hasattr(plan, "pva"):
        return None
    p, v, a = plan.pva(float(now_s))
    yaw = float(getattr(plan, "yaw", 0.0))
    p, v, a, hold = clamp_stalled_setpoint(measured, p, v, a, goal, radius, start)
    return p, v, a, yaw, hold


def clamp_stalled_setpoint(position, setpoint, velocity, acceleration, goal, radius: float, start):
    """时间跳变时，不要把设定点发到目标后面。

    进入 ``stop_at_goal`` 的 1.5 m，或者飞机/设定点已经越过起点到目标的那一段，
    就改成目标点悬停。返回 (position, velocity, acceleration, hold)。
    """
    position = np.asarray(position, dtype=float).reshape(3)
    setpoint = np.asarray(setpoint, dtype=float).reshape(3)
    velocity = np.asarray(velocity, dtype=float).reshape(3)
    acceleration = np.asarray(acceleration, dtype=float).reshape(3)
    goal = np.asarray(goal, dtype=float).reshape(3)
    start = np.asarray(start, dtype=float).reshape(3)
    radius = float(radius)
    if float(np.linalg.norm(position - goal)) <= radius or _past_goal(setpoint, goal, start, radius) or _past_goal(position, goal, start, radius):
        return goal.copy(), np.zeros(3), np.zeros(3), True
    to_goal = goal - position
    reach = float(np.dot(to_goal, to_goal))
    if reach > 1e-8:
        to_sp = setpoint - position
        progress = float(np.dot(to_sp, to_goal) / reach)
        lateral = float(np.linalg.norm(to_sp - progress * to_goal))
        if progress >= 1.0 and lateral <= radius:
            return goal.copy(), np.zeros(3), np.zeros(3), True
    return setpoint.copy(), velocity.copy(), acceleration.copy(), False


def _past_goal(point, goal, start, radius: float) -> bool:
    """水平面上，点在起点→目标的延长线上，并且已经过了目标。横着偏开的不算。高度不参与。"""
    axis = (np.asarray(goal, dtype=float) - np.asarray(start, dtype=float))[:2]
    span = float(np.dot(axis, axis))
    if span <= 1e-8:
        return False
    rel = (np.asarray(point, dtype=float) - np.asarray(goal, dtype=float))[:2]
    along = float(np.dot(rel, axis) / math.sqrt(span))
    if along <= 0.05:
        return False
    lateral = float(np.linalg.norm(rel - (float(np.dot(rel, axis)) / span) * axis))
    return lateral <= max(float(radius), 1.5)


def setpoint_to_local(setpoint: dict | None, offset_enu) -> dict | None:
    """世界系 NED 设定点减回生成点偏置。NaN 轴保持 NaN。不改速度、加速度和航向。"""
    if setpoint is None:
        return None
    local = dict(setpoint)
    position = local.get("position")
    if position is None:
        return local
    pos = np.asarray(position, dtype=float).reshape(-1)
    if pos.shape != (3,):
        return local
    delta = np.asarray(px4.enu_to_ned(np.asarray(offset_enu, dtype=float).reshape(3)), dtype=float)
    shifted = []
    for i in range(3):
        value = float(pos[i])
        if math.isfinite(value):
            shifted.append(value - float(delta[i]))
        else:
            shifted.append(value)
    local["position"] = shifted
    return local


def realtime_factor(sim_elapsed_s, wall_s) -> float | None:
    """仿真时间 / 墙钟。墙钟不是正数时没有 RTF。"""
    if sim_elapsed_s is None or wall_s is None:
        return None
    sim = float(sim_elapsed_s)
    wall = float(wall_s)
    if not math.isfinite(sim) or not math.isfinite(wall) or not wall > 0.0:
        return None
    return sim / wall


def outcome_exit_code(result: dict) -> int:
    """进程退出码跟 outcome 走。只有 SUCCESS 且 ok 时是 0。"""
    if result.get("outcome") == "SUCCESS" and result.get("ok") is True:
        return 0
    return 1


def encode_depth_image(frame: DepthFrame) -> dict:
    """``DepthFrame`` → ``sensor_msgs/Image`` 的 32FC1 字段。字节是小端 float32。"""
    depth = np.ascontiguousarray(frame.depth, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"深度必须是二维，收到 {depth.shape}")
    return {
        "height": int(depth.shape[0]),
        "width": int(depth.shape[1]),
        "encoding": "32FC1",
        "is_bigendian": 0,
        "step": int(depth.shape[1] * 4),
        "data": depth.tobytes(),
        "stamp_s": float(frame.stamp),
        "K": [float(x) for x in frame.K],
    }


def decode_depth_image(msg, *, fallback_K=None, stamp_s: float | None = None) -> DepthFrame:
    """``sensor_msgs/Image`` 字段 → ``DepthFrame``。32FC1 是米，16UC1 是毫米，0 视为无回波。"""
    if isinstance(msg, DepthFrame):
        return msg
    height = int(_msg_get(msg, "height"))
    width = int(_msg_get(msg, "width"))
    encoding = str(_msg_get(msg, "encoding", "32FC1"))
    if height < 1 or width < 1:
        raise ValueError(f"深度图尺寸非法：{height}x{width}")
    data = _msg_get(msg, "data")
    if isinstance(data, np.ndarray) and data.ndim == 2:
        depth = np.array(data, dtype=np.float32, copy=True)
        if depth.shape != (height, width):
            raise ValueError(f"深度数组形状 {depth.shape} 和 {height}x{width} 不一致")
    else:
        raw = bytes(data)
        big = int(_msg_get(msg, "is_bigendian", 0) or 0) == 1
        endian = ">" if big else "<"
        if encoding == "32FC1":
            expect = height * width * 4
            if len(raw) != expect:
                raise ValueError(f"32FC1 字节数 {len(raw)} != {expect}")
            depth = np.frombuffer(raw, dtype=f"{endian}f4").reshape(height, width).astype(np.float32, copy=True)
        elif encoding == "16UC1":
            expect = height * width * 2
            if len(raw) != expect:
                raise ValueError(f"16UC1 字节数 {len(raw)} != {expect}")
            mm = np.frombuffer(raw, dtype=f"{endian}u2").reshape(height, width)
            depth = mm.astype(np.float32) * np.float32(0.001)
            depth[mm == 0] = np.nan
        else:
            raise ValueError(f"不支持的深度编码 {encoding}")
    raw_k = _msg_get(msg, "K", fallback_K)
    if raw_k is None:
        raise ValueError("深度图缺少 K")
    intrinsics = tuple(float(x) for x in raw_k)
    if len(intrinsics) != 4 or not all(math.isfinite(x) for x in intrinsics):
        raise ValueError(f"K 需要 4 个有限数，收到 {raw_k!r}")
    stamp = float(stamp_s) if stamp_s is not None else float(_msg_get(msg, "stamp_s", 0.0))
    return DepthFrame(stamp, depth, intrinsics)  # type: ignore[arg-type]


def depth_from_obstacle_summary(summary, position_enu, yaw_enu: float, camera: CameraConfig, stamp: float) -> DepthFrame:
    """障碍摘要 → 一帧 z 深度。点用 ENU；只有距离时按机体航向排成水平扇面。

    投影和 ``DepthCamera`` 同一套光学轴（``R_BODY_CAM``）。摘要里没有的像素是 NaN。
    """
    if isinstance(summary, str):
        summary = json.loads(summary)
    if not isinstance(summary, dict):
        raise ValueError("障碍摘要必须是对象或 JSON 对象字符串")
    height, width = int(camera.height), int(camera.width)
    depth = np.full((height, width), np.nan, dtype=np.float32)
    points = summary.get("points_enu")
    if points is None and summary.get("ranges_m") is not None:
        ranges = np.asarray(summary["ranges_m"], dtype=float).reshape(-1)
        if "bearings_rad" in summary:
            bearings = np.asarray(summary["bearings_rad"], dtype=float).reshape(-1)
        else:
            half = math.radians(float(camera.hfov_deg)) / 2.0
            bearings = np.linspace(float(yaw_enu) - half, float(yaw_enu) + half, int(ranges.size))
        if bearings.shape != ranges.shape:
            raise ValueError("ranges_m 和 bearings_rad 长度不一致")
        if "elevations_rad" in summary:
            elevations = np.asarray(summary["elevations_rad"], dtype=float).reshape(-1)
            if elevations.shape != ranges.shape:
                raise ValueError("elevations_rad 和 ranges_m 长度不一致")
        else:
            elevations = np.zeros(ranges.size)
        origin = np.asarray(position_enu, dtype=float).reshape(3)
        points = np.stack(
            [
                origin[0] + ranges * np.cos(elevations) * np.cos(bearings),
                origin[1] + ranges * np.cos(elevations) * np.sin(bearings),
                origin[2] + ranges * np.sin(elevations),
            ],
            axis=1,
        )
    if points is not None:
        cloud = np.asarray(points, dtype=float).reshape(-1, 3)
        yaw = float(yaw_enu)
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam_pos = np.asarray(position_enu, dtype=float).reshape(3) + rotation @ np.asarray(camera.mount_xyz_body, dtype=float)
        world_cam = rotation @ R_BODY_CAM
        fx, fy, cx, cy = camera.intrinsics
        for point in cloud:
            if not np.all(np.isfinite(point)):
                continue
            rel = world_cam.T @ (point - cam_pos)
            z = float(rel[2])
            if not math.isfinite(z) or z < float(camera.min_range_m) or z > float(camera.max_range_m):
                continue
            uc = cx + fx * (float(rel[0]) / z)
            vc = cy + fy * (float(rel[1]) / z)
            ui, vi = int(math.floor(uc)), int(math.floor(vc))
            if ui < 0 or vi < 0 or ui >= width or vi >= height:
                continue
            current = float(depth[vi, ui])
            if not math.isfinite(current) or z < current:
                depth[vi, ui] = np.float32(z)
    return DepthFrame(float(stamp), depth, camera.intrinsics)


def resolve_depth_frame(
    bridge: YopoSitlBridge,
    decoded: dict,
    now_s: float,
    *,
    depth=None,
    obstacle_summary=None,
    depth_source: str = "auto",
) -> tuple[str, DepthFrame | None]:
    """选出这一拍的深度。返回的帧是 None 时，``command`` 仍用场景卡相机渲染。"""
    if depth_source not in ("auto", "depth", "summary", "scene-card"):
        raise ValueError(f"depth_source 只要 auto、depth、summary 或 scene-card，收到 {depth_source!r}")
    if depth_source in ("auto", "depth") and depth is not None:
        return "ros-depth", decode_depth_image(depth, fallback_K=bridge.camera.cfg.intrinsics, stamp_s=now_s)
    if depth_source in ("auto", "summary") and obstacle_summary is not None:
        frame = depth_from_obstacle_summary(
            obstacle_summary, decoded["position_enu"], decoded["yaw_enu"], bridge.camera.cfg, now_s
        )
        return "ros-obstacle", frame
    if depth_source in ("auto", "scene-card"):
        return "scene-card", None
    return "missing", None


def command_from_ros(
    bridge: YopoSitlBridge,
    odometry,
    now_s: float,
    *,
    depth=None,
    obstacle_summary=None,
    depth_source: str = "auto",
) -> tuple[BridgeOutput, str]:
    """一条里程计消息（加可选深度或障碍摘要）→ NED 设定点。"""
    decoded = decode_vehicle_odometry(odometry, now_s=now_s)
    source, frame = resolve_depth_frame(
        bridge, decoded, now_s, depth=depth, obstacle_summary=obstacle_summary, depth_source=depth_source
    )
    if source == "missing":
        raise ValueError(f"{bridge.cfg.px4_namespace or '/'} 这一拍没有 {depth_source} 感知")
    out = bridge.command(
        decoded["position_ned"],
        decoded["velocity_ned"],
        now_s,
        q_ned_frd=decoded["q_ned_frd"],
        acceleration_ned=decoded["acceleration_ned"],
        depth_frame=frame,
    )
    return out, source


def _goal_limits(card: SceneCard) -> tuple[float, float, float, float]:
    crit = card.card.get("criteria") or {}
    body = float(crit.get("body_radius_m", 0.35))
    if "goal_tolerance_m" in crit:
        return float(crit["goal_tolerance_m"]), float(crit["goal_speed_mps"]), float(crit["goal_hold_s"]), body
    return float(crit.get("hover_tolerance_m", 0.3)), math.inf, 0.0, body


@dataclass
class VehicleInbox:
    """一架飞机这一拍的 ROS 字段。布尔量由调用方从 VehicleStatus / LandDetected 译好。"""

    odometry: object | None = None
    depth: object | None = None
    obstacle_summary: object | None = None
    armed: bool | None = None
    offboard: bool | None = None
    landed: bool | None = None
    preflight_ok: bool = True


@dataclass(frozen=True)
class LiveCommand:
    namespace: str
    mav_sys_id: int
    gcs_port: int
    topics: dict
    phase: str
    heartbeat: bool
    setpoint: dict | None
    offboard: dict | None
    vehicle_commands: tuple
    planned: bool
    depth_source: str
    roundtrip_error_m: float
    position_enu: tuple | None
    rates_setpoint: dict | None = None


@dataclass
class LiveFleetConfig:
    positioning: str = "default"
    depth_source: str = "auto"
    initial_phase: str = "WARMUP"
    warmup_s: float = 1.0
    arm_timeout_s: float = 60.0
    arm_retry_s: float = 1.0
    takeoff_timeout_s: float = 30.0
    plan_timeout_s: float = 45.0
    land_timeout_s: float = 60.0
    settle_pos_m: float = 0.2
    settle_vel_mps: float = 0.3
    settle_hold_s: float = 1.0
    shadow: bool = True  # 默认开。只记录会否拒绝，不改批准动作
    # 票 SH2：默认关。打开后把场景卡几何交给影子盾，结果标偏离上游。
    shadow_env_evidence: bool = False
    # 票 SH2：默认关。打开后 crossing / switch / formation 也记影子盾（只读）。
    # 打开 shadow_env_evidence 时编队类策略也会开影子，否则证据接不上。
    fleet_shadow: bool = False
    # 票 SH3 / SH4：默认关。打开即偏离上游。
    shadow_contract_align: bool = False
    shadow_tracking_align: bool = False
    # 票 B2：默认 0（关）。>0 时编队类策略的设定点取已提交计划上 t + lead 的那一点，
    # 补 PX4 位置环的跟踪滞后。打开即偏离上游。计划、协议和深度都不变。
    setpoint_lead_s: float = 0.0
    # 票 B2：默认 0（关）。>0 时编队类策略的设定点加上代理侧误差增益：
    # p_sp = p_ref + kp·(p_ref − p_odom)，v_sp = v_ref + kv·(v_ref − v_odom)，修正量限幅。
    # 等于从代理侧把 PX4 位置环、速度环增益抬高（不写 PX4 参数）。打开即偏离上游。
    setpoint_gain_p: float = 0.0
    setpoint_gain_v: float = 0.0
    control_mode: str = "pva"
    record_dt_s: float = 0.0

    def __post_init__(self) -> None:
        self.positioning = normalize_positioning(self.positioning)
        if self.control_mode not in ("pva", "ctbr"):
            raise ValueError("control_mode must be pva or ctbr")
        if not math.isfinite(self.record_dt_s) or self.record_dt_s < 0.0:
            raise ValueError("record_dt_s must be finite and nonnegative")
        if not (0.0 <= float(self.setpoint_lead_s) <= 0.5):
            raise ValueError(f"setpoint_lead_s 要在 [0, 0.5]，收到 {self.setpoint_lead_s}")
        for name in ("setpoint_gain_p", "setpoint_gain_v"):
            if not (0.0 <= float(getattr(self, name)) <= 5.0):
                raise ValueError(f"{name} 要在 [0, 5]，收到 {getattr(self, name)}")
        if self.depth_source not in ("auto", "depth", "summary", "scene-card"):
            raise ValueError(f"depth_source 非法：{self.depth_source!r}")
        if self.initial_phase not in ("WARMUP", "PLAN"):
            raise ValueError(f"initial_phase 只要 WARMUP 或 PLAN，收到 {self.initial_phase!r}")
        for name in ("warmup_s", "arm_timeout_s", "takeoff_timeout_s", "plan_timeout_s", "land_timeout_s"):
            if not float(getattr(self, name)) > 0.0:
                raise ValueError(f"{name} 必须为正")


class _LiveSlot:
    def __init__(self, bridge: YopoSitlBridge, ident: dict, phase: str):
        self.bridge = bridge
        self.ident = ident
        self.phase = phase
        self.t_phase: float | None = None
        self.home: np.ndarray | None = None
        self.hover: np.ndarray | None = None
        self.goal_since: float | None = None
        self.arrival: float | None = None
        self.settle_since: float | None = None
        self.last_cmd_s = -math.inf
        self.depth_source = ""
        self.position: np.ndarray | None = None
        self.velocity: np.ndarray | None = None
        self.yaw = 0.0
        self.land_sent = False
        self.landed_done = False
        self.land_give_up = False
        self.offset_enu = world_offset_enu(bridge.start_enu)
        self.trace: list[dict] = []
        self.formation_sampler = None
        self.takeoff_handover_s: float | None = None
        self.shadow = None
        self.ctbr = None
        self.ctbr_ticks = 0
        self.ctbr_finished = False
        self.pva_ticks = 0
        self.ctbr_last_yaw = None
        self.ctbr_last_t = None
        self.flight_samples: list[dict] = []
        self.max_horizontal_displacement_m = 0.0
        self.hover_calibration_attempted = False
        self.fleet_frame = None  # 票 S7：crossing 最近一帧场景卡深度



class _LazyDepthFrame:
    """票 S7：按相机频率记下位姿和时刻，协议读 ``depth`` 时才渲染。

    crossing 只在出提案时读最新一帧。实飞时三机 30 Hz 全渲染约占 1.6 倍实时的 CPU，
    和 Gazebo / PX4 抢核。渲染出来的帧与当时立即渲染的位姿、时刻相同；噪声序列按实际渲染次序取。
    """

    __slots__ = ("_camera", "_position", "_yaw", "stamp", "K", "_frame")

    def __init__(self, camera, position: np.ndarray, yaw: float, stamp: float):
        self._camera = camera
        self._position = np.asarray(position, dtype=float).copy()
        self._yaw = float(yaw)
        self.stamp = float(stamp)
        self.K = camera.cfg.intrinsics
        self._frame = None

    @property
    def rendered(self) -> bool:
        return self._frame is not None

    @property
    def depth(self) -> np.ndarray:
        if self._frame is None:
            self._frame = self._camera.capture(self._position, self._yaw, self.stamp)
        return self._frame.depth


def _formation_plan_code(fleet, drone_id: str) -> str | None:
    """还没 track 过的机在 result() 里没有 last_record，不能抛 KeyError。"""
    if fleet is None:
        return None
    last = getattr(fleet, "_last_rec", None)
    if isinstance(last, dict):
        if drone_id not in last:
            return None
        return fleet.code(drone_id)
    code_fn = getattr(fleet, "code", None)
    if not callable(code_fn):
        return None
    try:
        return code_fn(drone_id)
    except KeyError:
        return None


def _cross_x_time(t_prev: float, x_prev: float, t_curr: float, x_curr: float, x_target: float) -> float | None:
    """线性插值计算 ``x`` 首次越过 ``x_target`` 的协议时刻。"""
    if x_prev >= x_target:
        return float(t_prev)
    if x_curr < x_target:
        return None
    dx = float(x_curr) - float(x_prev)
    if dx <= 1e-12:
        return float(t_curr)
    alpha = max(0.0, min(1.0, (float(x_target) - float(x_prev)) / dx))
    return float(t_prev) + alpha * (float(t_curr) - float(t_prev))


class _ExploreSitlTelemetry:
    """只读记录 ``policy=explore`` 在 SITL 实飞中的横线起飞对齐、穿林速度与自由终端悬停指标。"""

    def __init__(self, card: SceneCard) -> None:
        raw = card.card
        task = dict(raw.get("task") or {})
        drones = list(raw.get("drones") or [])
        has_forest = isinstance(raw.get("forest"), dict)
        starts_xy = [(float(d["start_enu"][0]), float(d["start_enu"][1])) for d in drones]
        xs = [pt[0] for pt in starts_xy]
        ys = sorted(pt[1] for pt in starts_xy)
        gaps = [float(ys[i + 1] - ys[i]) for i in range(len(ys) - 1)]
        x_span = 0.0 if not xs else float(max(xs) - min(xs))
        self.spawn_summary = {
            "aligned_x": bool(x_span <= 1e-6),
            "x0_m": None if not xs else float(xs[0]),
            "spawn_x_span_m": x_span,
            "y_sorted_m": ys,
            "lateral_gaps_m": gaps,
            "uniform_lateral_spacing": bool(len(gaps) <= 1 or (max(gaps) - min(gaps) <= 1e-6)),
        }
        default_goal_x = 42.0 if has_forest else 19.5
        default_finish_x = 32.0 if has_forest else 15.5
        default_span = [8.0, 30.0] if has_forest else [3.7, 14.8]
        span_raw = task.get("obstacle_x_span_m", default_span)
        self.x_goal_m = float(task.get("x_goal_m", default_goal_x))
        self.x_finish_m = float(task.get("x_finish_m", default_finish_x))
        self.belt_lo_m = float(span_raw[0])
        self.belt_hi_m = float(span_raw[1])
        self.velocity_target_mps = float(task.get("velocity_mps", 9.0))
        self.launch_t_s: float | None = None
        self.handover_positions_enu: dict[str, list[float]] = {}
        self._prev_tx: dict[str, tuple[float, float]] = {}
        self._belt_enter: dict[str, float] = {}
        self._belt_exit: dict[str, float] = {}
        self._finish_t: dict[str, float] = {}
        self._belt_speeds: dict[str, list[float]] = {str(d["id"]): [] for d in drones}
        self._peak_speed: dict[str, float] = {str(d["id"]): 0.0 for d in drones}

    def note_launch(self, now_s: float, slots: list[_LiveSlot], decoded: dict[str, dict | None]) -> None:
        if self.launch_t_s is not None:
            return
        self.launch_t_s = float(now_s)
        for slot in slots:
            did = slot.bridge.drone_id
            st = decoded.get(slot.ident["namespace"])
            pos = slot.position if st is None else np.asarray(st["position_enu"], dtype=float)
            if pos is not None:
                vec = _vec_list(pos)
                self.handover_positions_enu[did] = vec
                x0 = float(vec[0])
                self._prev_tx[did] = (0.0, x0)
                if x0 >= self.belt_lo_m:
                    self._belt_enter[did] = 0.0
                if x0 >= self.belt_hi_m:
                    self._belt_exit[did] = 0.0
                if x0 >= self.x_finish_m:
                    self._finish_t[did] = 0.0

    def note_step(self, drone_id: str, t_plan_s: float, position_enu, velocity_enu) -> None:
        p = np.asarray(position_enu, dtype=float).reshape(3)
        v = np.asarray(velocity_enu, dtype=float).reshape(3)
        t = float(t_plan_s)
        x = float(p[0])
        spd = float(np.linalg.norm(v))
        self._peak_speed[drone_id] = max(self._peak_speed.get(drone_id, 0.0), spd)
        if self.belt_lo_m <= x <= self.belt_hi_m:
            self._belt_speeds.setdefault(drone_id, []).append(spd)
        prev = self._prev_tx.get(drone_id)
        if prev is None:
            if x >= self.belt_lo_m and drone_id not in self._belt_enter:
                self._belt_enter[drone_id] = t
            if x >= self.belt_hi_m and drone_id not in self._belt_exit:
                self._belt_exit[drone_id] = t
            if x >= self.x_finish_m and drone_id not in self._finish_t:
                self._finish_t[drone_id] = t
        else:
            t0, x0 = prev
            if drone_id not in self._belt_enter:
                tc = _cross_x_time(t0, x0, t, x, self.belt_lo_m)
                if tc is not None:
                    self._belt_enter[drone_id] = tc
            if drone_id not in self._belt_exit:
                tc = _cross_x_time(t0, x0, t, x, self.belt_hi_m)
                if tc is not None:
                    self._belt_exit[drone_id] = tc
            if drone_id not in self._finish_t:
                tc = _cross_x_time(t0, x0, t, x, self.x_finish_m)
                if tc is not None:
                    self._finish_t[drone_id] = tc
        self._prev_tx[drone_id] = (t, x)

    def summary(self, slots: list[_LiveSlot], fleet) -> dict:
        handover_xs = [pos[0] for pos in self.handover_positions_enu.values()]
        handover_ys = sorted(pos[1] for pos in self.handover_positions_enu.values())
        handover_span = None if not handover_xs else float(max(handover_xs) - min(handover_xs))
        belt_span = max(0.0, self.belt_hi_m - self.belt_lo_m)
        per_drone: dict[str, dict] = {}
        belt_avg_list: list[float] = []
        sample_all: list[float] = []
        locked_fn = getattr(fleet, "terminal_locked", None)
        for slot in slots:
            did = slot.bridge.drone_id
            t_in = self._belt_enter.get(did)
            t_out = self._belt_exit.get(did)
            t_fin = self._finish_t.get(did)
            trav = None
            avg_spd = None
            if t_in is not None and t_out is not None and t_out > t_in + 1e-9 and belt_span > 0.0:
                trav = float(t_out - t_in)
                avg_spd = float(belt_span / trav)
                belt_avg_list.append(avg_spd)
            samples = self._belt_speeds.get(did) or []
            sample_all.extend(samples)
            locked = bool(locked_fn(did)) if callable(locked_fn) else None
            per_drone[did] = {
                "t_belt_enter_s": t_in,
                "t_belt_exit_s": t_out,
                "belt_traversal_s": trav,
                "belt_avg_speed_mps": avg_spd,
                "belt_sample_speed_mean_mps": None if not samples else float(np.mean(samples)),
                "t_finish_after_launch_s": t_fin,
                "peak_speed_mps": float(self._peak_speed.get(did, 0.0)),
                "terminal_locked": locked,
                "terminal_goal_enu": _vec_list(slot.bridge.goal),
            }
        all_finish = (
            max(self._finish_t[slot.bridge.drone_id] for slot in slots)
            if slots and all(slot.bridge.drone_id in self._finish_t for slot in slots)
            else None
        )
        all_belt_exit = (
            max(self._belt_exit[slot.bridge.drone_id] for slot in slots)
            if slots and all(slot.bridge.drone_id in self._belt_exit for slot in slots)
            else None
        )
        return {
            "velocity_target_mps": self.velocity_target_mps,
            "speed_ratio": float(self.velocity_target_mps / 6.0),
            "x_goal_m": self.x_goal_m,
            "x_finish_m": self.x_finish_m,
            "obstacle_x_span_m": [self.belt_lo_m, self.belt_hi_m],
            "horizontal_line_spawn": dict(self.spawn_summary),
            "takeoff_handover": {
                "launch_t_s": self.launch_t_s,
                "positions_enu": dict(self.handover_positions_enu),
                "handover_x_span_m": handover_span,
                "handover_y_sorted_m": handover_ys,
            },
            "all_crossed_belt_after_launch_s": all_belt_exit,
            "all_crossed_finish_after_launch_s": all_finish,
            "belt_avg_speed_mean_mps": None if not belt_avg_list else float(np.mean(belt_avg_list)),
            "belt_avg_speed_min_mps": None if not belt_avg_list else float(min(belt_avg_list)),
            "belt_sample_speed_mean_mps": None if not sample_all else float(np.mean(sample_all)),
            "peak_speed_mps": max(self._peak_speed.values(), default=0.0),
            "per_drone": per_drone,
        }


class LiveFleetSession:
    """容器侧状态机：预热、解锁、起飞、按桥规划、全体到达后 NAV_LAND。

    不写 PX4 参数。每架只读自己 namespace 的消息。``upstream`` 的设定点来自
    ``YopoSitlBridge``。``formation`` 的设定点来自 ``FormationFleet`` 已提交的计划，
    机间消息只进构造时传入的进程内 ``MessageBus``。
    进来的里程计先加上该机场景卡起点的水平偏置；``_pack`` 把设定点减回去。
    """

    def __init__(
        self,
        card: SceneCard,
        bridges: list[YopoSitlBridge],
        idents: list[dict],
        cfg: LiveFleetConfig,
        *,
        fleet: FormationFleet | None = None,
        bus: MessageBus | None = None,
        policy: str = "upstream",
    ):
        if len(bridges) != len(idents) or not bridges:
            raise ValueError("bridges 和 idents 必须非空且一一对应")
        if policy not in LIVE_POLICIES:
            raise ValueError(f"policy 只要 {' 或 '.join(LIVE_POLICIES)}，收到 {policy!r}")
        if (fleet is None) != (policy not in FLEET_POLICIES):
            raise ValueError("formation / crossing / switch / explore 会话要带同名 fleet，其他策略不带")
        if policy in FLEET_POLICIES and bus is None:
            raise ValueError(f"{policy} 会话要带 MessageBus")
        if cfg.control_mode == "ctbr" and policy != "explore":
            raise ValueError("CTBR currently requires policy=explore")
        self.card = card
        self.cfg = cfg
        self.policy = policy
        self.fleet = fleet
        self.bus = bus
        self._explore_telemetry = _ExploreSitlTelemetry(card) if policy == "explore" else None
        self._plan_origin: float | None = None
        self._formation_t: float | None = None
        self.slots = [_LiveSlot(bridge, ident, cfg.initial_phase) for bridge, ident in zip(bridges, idents)]
        if cfg.control_mode == "ctbr":
            from .ctbr import FlatnessCTBRController
            for slot in self.slots:
                slot.ctbr = FlatnessCTBRController()
        self.goal_tol, self.goal_speed, self.goal_hold, self.body = _goal_limits(card)
        self.fail_reason = ""
        self.px4_parameters_written: list = []
        self.land_commands_sent = 0
        self.max_roundtrip_m = 0.0
        self._last_now: float | None = None
        self._last_cmds: list[LiveCommand] = []
        self._min_inter: float | None = None
        self._min_clear = math.inf
        # 票 S7：旧字段 min_surface_clearance_m 仍计全部离地样本（含降落段）。
        # 下面两个只计飞行段：flight = TAKEOFF / PLAN，plan = 只计 PLAN（与 L0 可比）。
        self._min_clear_flight = math.inf
        self._min_clear_plan = math.inf
        self._min_inter_plan: float | None = None
        self._decoded: dict[str, dict | None] = {}
        self._last_capture: dict[str, float] = {}
        self._sim0: float | None = None
        self._sim1: float | None = None
        self._wall0: float | None = None
        self._shadow_ticks: list[dict] = []
        self.shadow = shadow_sitl.shadow_enabled(cfg, policy)
        self.frame_note = frame_note(card)
        if self.shadow:
            params = dict(card.card.get("policy_params") or {})
            for slot in self.slots:
                shadow_sitl.bind_slot_shadow(slot, card, cfg, params)

    def step(self, now_s: float, inbox_by_ns: dict[str, VehicleInbox]) -> list[LiveCommand]:
        now_s = float(now_s)
        if self._last_now is not None and abs(now_s - self._last_now) <= 1e-12:
            return list(self._last_cmds)
        if self._wall0 is None:
            self._wall0 = time.perf_counter()
            self._sim0 = now_s
        self._sim1 = now_s
        decoded: dict[str, dict | None] = {}
        for slot in self.slots:
            ns = slot.ident["namespace"]
            inbox = inbox_by_ns.get(ns)
            if inbox is None or inbox.odometry is None:
                decoded[ns] = None
                continue
            try:
                decoded[ns] = apply_world_offset(decode_vehicle_odometry(inbox.odometry, now_s=now_s), slot.offset_enu)
            except ValueError:
                decoded[ns] = None
        self._decoded = decoded
        if self.cfg.control_mode == "ctbr":
            ground = self.card.card.get("ground_z")
            for slot in self.slots:
                state = decoded.get(slot.ident["namespace"])
                if (slot.phase == "PLAN" and ground is not None and state is not None
                        and state["position_enu"][2] <= float(ground) + self.body):
                    self._fail(f"{slot.bridge.drone_id}:GROUND_COLLISION", now_s)
        self._exchange(decoded)
        self._note_separation(decoded, now_s)
        self._formation_exchange(now_s)
        commands = [self._step_slot(slot, now_s, inbox_by_ns.get(slot.ident["namespace"]), decoded[slot.ident["namespace"]]) for slot in self.slots]
        if self.fail_reason == "" and all(slot.arrival is not None for slot in self.slots):
            for slot in self.slots:
                if slot.phase == "PLAN":
                    slot.phase = "LAND"
                    slot.t_phase = now_s
        self._last_now = now_s
        self._last_cmds = commands
        return list(commands)

    def stream_local_setpoint(self, namespace: str, now_s: float) -> dict | None:
        """PLAN 阶段心跳：按仿真钟采样当前轨迹，减回本地 NED。其他阶段返回 None。"""
        if self.fleet is not None:
            return self._formation_stream(namespace, float(now_s))
        for slot in self.slots:
            if slot.ident["namespace"] != namespace or slot.phase != "PLAN":
                continue
            sample = slot.bridge.tracking_sample(now_s)
            if sample is None:
                return None
            p, v, a, yaw, _hold = sample
            world = trajectory_setpoint_ned(p, v, a, yaw, now_s)
            return setpoint_to_local(world, slot.offset_enu)
        return None

    def _formation_exchange(self, now_s: float) -> None:
        """全体进入 PLAN 之后才走协议。时钟从这一拍记 0，起飞不计入 t_go。

        平时一拍一次。仿真钟跳过多个 physics_dt 时按步长补拍，窗口才不会被整段跳过。
        补拍只推进协议，不改里程计。设定点仍只在本拍的编队钟上采样一次。
        """
        if self.fleet is None or self.bus is None:
            return
        if any(slot.phase != "PLAN" for slot in self.slots):
            return
        now_s = float(now_s)
        if self._plan_origin is None and self._explore_telemetry is not None:
            self._explore_telemetry.note_launch(now_s, self.slots, self._decoded)
        self._fleet_ingest(now_s)
        if self._plan_origin is None:
            self._plan_origin = now_s
            self._formation_t = 0.0
            self.fleet.exchange(self.bus, 0.0)
            return
        target = now_s - float(self._plan_origin)
        if target < float(self._formation_t) - 1e-9:
            return
        dt = float(self.slots[0].bridge.cfg.physics_dt)
        t = float(self._formation_t)
        guard = 0
        while t + dt < target - 1e-9 and guard < _FORMATION_SUBSTEPS:
            t = round(t + dt, 10)
            self.fleet.exchange(self.bus, t)
            guard += 1
        if target > t + 1e-12:
            self.fleet.exchange(self.bus, target)
            t = target
        self._formation_t = t

    def _fleet_ingest(self, now_s: float) -> None:
        """crossing / explore 的提案要本机深度：按场景卡相机频率在里程计位姿上渲染，和 L0 一样每拍喂最新一帧。

        formation / switch 没有 ingest，不进这里。深度来自场景卡几何，不依赖 Gazebo 里有没有模型。
        """
        ingest = getattr(self.fleet, "ingest", None)
        if ingest is None:
            return
        cam_hz = float((self.card.card.get("rates_hz") or {}).get("camera", 30.0))
        period = 1.0 / cam_hz if cam_hz > 0.0 else 0.0
        ingest_t = (
            (0.0 if self._plan_origin is None else max(0.0, float(now_s) - float(self._plan_origin)))
            if self.policy == "explore"
            else float(now_s)
        )
        for slot in self.slots:
            state = self._decoded.get(slot.ident["namespace"])
            if state is None:
                continue
            did = slot.bridge.drone_id
            last = self._last_capture.get(did)
            frame = slot.fleet_frame
            if frame is None or last is None or now_s - last >= period - 1e-9:
                frame = _LazyDepthFrame(
                    slot.bridge.camera, np.asarray(state["position_enu"], dtype=float), float(state["yaw_enu"]), ingest_t
                )
                slot.fleet_frame = frame
                self._last_capture[did] = now_s
            acc = (
                np.asarray(state["acceleration_enu"], dtype=float).copy()
                if self.policy == "explore" and state.get("acceleration_enu") is not None
                else np.zeros(3)
            )
            view = State(
                p=np.asarray(state["position_enu"], dtype=float).copy(),
                v=np.asarray(state["velocity_enu"], dtype=float).copy(),
                a=acc,
                yaw=float(state["yaw_enu"]),
                t=ingest_t,
            )
            ingest(did, frame, view)
            slot.depth_source = "scene-card"

    def _explore_terminal_pva(
        self, slot: _LiveSlot, p: np.ndarray, v: np.ndarray, a: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """explore 进入终点无障碍区并锁定无冲突驻留点后，直接下发静止位置设定点避免里程计重锚振荡。"""
        if self.policy != "explore" or self.fleet is None:
            return p, v, a
        did = slot.bridge.drone_id
        locked_fn = getattr(self.fleet, "terminal_locked", None)
        if not (callable(locked_fn) and locked_fn(did)):
            return p, v, a
        goal = np.asarray(self.fleet.goal_of(did), dtype=float).copy()
        slot.bridge.goal = goal
        if self.fleet.code(did) == "ARRIVED_HOLD" and slot.position is not None:
            if float(np.linalg.norm(slot.position - goal)) <= 5.0:
                return goal.copy(), np.zeros(3), np.zeros(3)
        return p, v, a

    def _formation_stream(self, namespace: str, now_s: float) -> dict | None:
        """心跳只读已经提交的采样器。不调用 sample，也不再 exchange。"""
        if self._plan_origin is None:
            return None
        t = float(now_s) - float(self._plan_origin)
        if t < 0.0:
            return None
        for slot in self.slots:
            if slot.ident["namespace"] != namespace or slot.phase != "PLAN":
                continue
            sampler = slot.formation_sampler
            if sampler is None:
                return None
            lead = float(self.cfg.setpoint_lead_s)
            p, v, a = sampler.pva(t + lead if lead > 0.0 else t)
            p, v, a = self._explore_terminal_pva(
                slot,
                np.asarray(p, dtype=float),
                np.asarray(v, dtype=float),
                np.asarray(a, dtype=float),
            )
            world = trajectory_setpoint_ned(p, v, a, float(sampler.yaw), now_s)
            return setpoint_to_local(world, slot.offset_enu)
        return None

    def _exchange(self, decoded: dict[str, dict | None]) -> None:
        if self.fleet is not None:
            return
        for slot in self.slots:
            if slot.phase != "PLAN":
                continue
            others = []
            for other in self.slots:
                if other is slot:
                    continue
                state = decoded.get(other.ident["namespace"])
                if state is None:
                    continue
                others.append(
                    NeighborEnu(
                        other.bridge.drone_id,
                        state["position_enu"],
                        state["velocity_enu"],
                        other.bridge.broadcast_samples(),
                    )
                )
            slot.bridge.set_neighbors(others)

    def _note_separation(self, decoded: dict[str, dict | None], now_s: float) -> None:
        """离地超过机体半径才计最小净空和最小机间。碰撞仍只在 TAKEOFF / PLAN 里判。

        先记下全部离地样本，再落第一个失败原因。已经有失败时不再改原因。
        """
        airborne = []
        obstacle = None
        for slot in self.slots:
            state = decoded.get(slot.ident["namespace"])
            if state is None:
                continue
            position = np.asarray(state["position_enu"], dtype=float)
            if float(position[2]) <= self.body:
                continue
            dist = float(self.card.min_distance(position))
            self._min_clear = min(self._min_clear, dist - self.body)
            judged = slot.phase in ("TAKEOFF", "PLAN")
            if judged:
                self._min_clear_flight = min(self._min_clear_flight, dist - self.body)
            if slot.phase == "PLAN":
                self._min_clear_plan = min(self._min_clear_plan, dist - self.body)
            if judged and obstacle is None and self.fail_reason == "" and dist <= self.body:
                obstacle = f"{slot.bridge.drone_id}:OBSTACLE"
            airborne.append((slot, position, judged))
        if obstacle is not None:
            self._fail(obstacle, now_s)
        inter = None
        for i in range(len(airborne)):
            for j in range(i + 1, len(airborne)):
                dist = float(np.linalg.norm(airborne[i][1] - airborne[j][1]))
                self._min_inter = dist if self._min_inter is None else min(self._min_inter, dist)
                if airborne[i][0].phase == "PLAN" and airborne[j][0].phase == "PLAN":
                    self._min_inter_plan = dist if self._min_inter_plan is None else min(self._min_inter_plan, dist)
                if inter is None and self.fail_reason == "" and airborne[i][2] and airborne[j][2] and dist <= 2.0 * self.body:
                    left, right = airborne[i][0].bridge.drone_id, airborne[j][0].bridge.drone_id
                    inter = f"{left}-{right}:INTER_DRONE"
        if inter is not None:
            self._fail(inter, now_s)

    def _fail(self, reason: str, now_s: float) -> None:
        if self.fail_reason == "":
            self.fail_reason = reason
        for slot in self.slots:
            if slot.phase == "DONE" or slot.landed_done or slot.phase == "FAIL":
                continue
            slot.phase = "FAIL"
            slot.t_phase = now_s
            slot.land_sent = False

    def _pack(
        self,
        slot: _LiveSlot,
        now_s: float,
        *,
        heartbeat: bool,
        setpoint: dict | None,
        commands: list[dict],
        planned: bool,
        depth_source: str,
        roundtrip: float,
        position,
    ) -> LiveCommand:
        rates = None
        if (slot.ctbr is not None and self.fleet is not None and slot.position is not None
                and slot.velocity is not None and self.fleet.terminal_locked(slot.bridge.drone_id)
                and np.linalg.norm(slot.position - slot.bridge.goal) <= self.goal_tol
                and np.linalg.norm(slot.velocity) <= self.goal_speed):
            slot.ctbr_finished = True
        terminal_hold = slot.ctbr_finished
        state = self._decoded.get(slot.ident["namespace"])
        if (slot.ctbr is not None and slot.phase == "PLAN" and setpoint is not None
                and slot.formation_sampler is not None and state is not None and not terminal_hold):
            from .ctbr import GRAVITY
            p, v, a = [np.asarray(px4.ned_to_enu(setpoint[key]), dtype=float)
                       for key in ("position", "velocity", "acceleration")]
            sampler = slot.formation_sampler
            t = float(now_s) - float(self._plan_origin)
            traj = sampler.traj
            jerk = (np.asarray(traj.evaluate(t, 3), dtype=float)
                    if traj.t_start <= t <= traj.t_end else np.zeros(3))
            yaw = float(sampler.yaw)
            dt = 0.0 if slot.ctbr_last_t is None else now_s - slot.ctbr_last_t
            yaw_rate = (math.atan2(math.sin(yaw - slot.ctbr_last_yaw), math.cos(yaw - slot.ctbr_last_yaw)) / dt
                        if dt > 1e-6 and slot.ctbr_last_yaw is not None else 0.0)
            control = slot.ctbr.command(p, v, a, jerk, yaw, yaw_rate,
                                       state["position_enu"], state["velocity_enu"], state["rotation_enu_flu"])
            rates = control.fields(now_s)
            slot.ctbr_last_yaw, slot.ctbr_last_t = yaw, now_s
            slot.ctbr_ticks += 1
        else:
            slot.pva_ticks += int(setpoint is not None)
            slot.ctbr_last_t = None
        if position is not None:
            displacement = float(np.linalg.norm(np.asarray(position)[:2] - slot.bridge.start_enu[:2]))
            slot.max_horizontal_displacement_m = max(slot.max_horizontal_displacement_m, displacement)
        if self.cfg.record_dt_s > 0.0 and position is not None:
            if not slot.flight_samples or now_s - slot.flight_samples[-1]["t_s"] >= self.cfg.record_dt_s - 1e-9:
                slot.flight_samples.append({"t_s": float(now_s), "phase": slot.phase,
                    "p": _vec_list(position), "v": None if slot.velocity is None else _vec_list(slot.velocity),
                    "yaw": float(slot.yaw), "control": "ctbr" if rates is not None else "pva",
                    "ref": None if setpoint is None else _vec_list(px4.ned_to_enu(setpoint["position"])),
                    "rates": rates,
                    "body_up": None if state is None else _vec_list(state["rotation_enu_flu"][:, 2]),
                    "a_ref": None if setpoint is None else _vec_list(px4.ned_to_enu(setpoint["acceleration"]))})
        self._record_trace(slot, now_s, position, setpoint)
        setpoint = setpoint_to_local(setpoint, slot.offset_enu)
        if slot.trace and slot.trace[-1].get("t_s") == float(now_s) and setpoint is not None:
            slot.trace[-1]["setpoint_local_ned"] = _vec_list(setpoint["position"]) if setpoint.get("position") is not None else None
        offboard = None if setpoint is None else px4.offboard_control_mode(setpoint["timestamp"])
        if rates is not None:
            offboard = dict(offboard, position=False, velocity=False, acceleration=False,
                            attitude=False, body_rate=True)
        if position is not None:
            self.max_roundtrip_m = max(self.max_roundtrip_m, float(roundtrip))
        for command in commands:
            if int(command.get("command", -1)) == VEHICLE_CMD_NAV_LAND:
                self.land_commands_sent += 1
        pos = None if position is None else tuple(float(x) for x in np.asarray(position, dtype=float).reshape(3))
        return LiveCommand(
            namespace=slot.ident["namespace"],
            mav_sys_id=int(slot.ident["mav_sys_id"]),
            gcs_port=int(slot.ident["gcs_port"]),
            topics=vehicle_topics(slot.ident["namespace"]),
            phase=slot.phase,
            heartbeat=heartbeat,
            setpoint=setpoint,
            offboard=offboard,
            vehicle_commands=tuple(commands),
            planned=planned,
            depth_source=depth_source,
            roundtrip_error_m=float(roundtrip),
            position_enu=pos,
            rates_setpoint=rates,
        )

    def _hold(self, position, yaw: float, now_s: float) -> tuple[dict, float]:
        setpoint = trajectory_setpoint_ned(position, np.zeros(3), np.zeros(3), yaw, now_s)
        return setpoint, roundtrip_abs(position)

    def _lock_home(self, slot: _LiveSlot, state: dict) -> None:
        if slot.home is None:
            slot.home = np.asarray(state["position_enu"], dtype=float).copy()
            slot.hover = np.array([slot.home[0], slot.home[1], float(slot.bridge.hover_alt_m)], dtype=float)

    def _step_slot(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        if state is not None:
            slot.position = np.asarray(state["position_enu"], dtype=float).copy()
            slot.velocity = np.asarray(state["velocity_enu"], dtype=float).copy()
            slot.yaw = float(state["yaw_enu"])
            self._lock_home(slot, state)
        if slot.phase == "DONE":
            return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=[], planned=False, depth_source="", roundtrip=0.0, position=slot.position)
        if state is None or slot.home is None:
            return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=[], planned=False, depth_source="", roundtrip=0.0, position=None)
        if slot.t_phase is None:
            slot.t_phase = now_s
            if slot.phase == "PLAN" and slot.takeoff_handover_s is None:
                slot.takeoff_handover_s = now_s
        if slot.phase == "FAIL":
            return self._land_pack(slot, now_s, inbox)
        if slot.phase == "WARMUP":
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            if now_s - slot.t_phase >= self.cfg.warmup_s:
                slot.phase = "ARMING"
                slot.t_phase = now_s
            return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False, depth_source="", roundtrip=roundtrip, position=slot.position)
        if slot.phase == "ARMING":
            return self._arm(slot, now_s, inbox, state)
        if slot.phase == "TAKEOFF":
            return self._takeoff(slot, now_s, inbox, state)
        if slot.phase == "PLAN":
            return self._plan(slot, now_s, inbox, state)
        if slot.phase == "LAND":
            return self._land_pack(slot, now_s, inbox)
        return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=[], planned=False, depth_source="", roundtrip=0.0, position=slot.position)

    def _arm(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        armed = bool(inbox and inbox.armed)
        offboard = bool(inbox and inbox.offboard)
        if armed and offboard:
            slot.phase = "TAKEOFF"
            slot.t_phase = now_s
            slot.settle_since = None
            return self._takeoff(slot, now_s, inbox, state)
        if now_s - float(slot.t_phase) > self.cfg.arm_timeout_s:
            self._fail(f"{slot.bridge.drone_id}:ARM_TIMEOUT", now_s)
            return self._land_pack(slot, now_s, inbox)
        commands = []
        if now_s - slot.last_cmd_s >= self.cfg.arm_retry_s:
            slot.last_cmd_s = now_s
            ts = int(round(now_s * 1e6))
            if not offboard:
                commands.append(vehicle_command_message("offboard", ts, slot.ident["mav_sys_id"]))
            if not armed:
                commands.append(vehicle_command_message("arm", ts, slot.ident["mav_sys_id"]))
        setpoint, roundtrip = self._hold(slot.home, slot.yaw, now_s)
        return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=commands, planned=False, depth_source="", roundtrip=roundtrip, position=slot.position)

    def _takeoff(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        target = slot.hover
        err = float(np.linalg.norm(slot.position - target))
        speed = float(np.linalg.norm(slot.velocity))
        if err < self.cfg.settle_pos_m and speed < self.cfg.settle_vel_mps:
            slot.settle_since = now_s if slot.settle_since is None else slot.settle_since
            if now_s - slot.settle_since >= self.cfg.settle_hold_s - 1e-9:
                slot.phase = "PLAN"
                slot.t_phase = now_s
                slot.takeoff_handover_s = now_s
                slot.goal_since = None
        else:
            slot.settle_since = None
        if slot.phase == "TAKEOFF" and now_s - float(slot.t_phase) > self.cfg.takeoff_timeout_s:
            self._fail(f"{slot.bridge.drone_id}:TAKEOFF_TIMEOUT", now_s)
            return self._land_pack(slot, now_s, inbox)
        if slot.phase == "PLAN":
            return self._plan(slot, now_s, inbox, state)
        setpoint = trajectory_setpoint_ned(target, np.zeros(3), np.zeros(3), slot.yaw, now_s)
        return self._pack(
            slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False,
            depth_source="", roundtrip=roundtrip_abs(target), position=slot.position,
        )

    def _plan(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        if state is None:
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False, depth_source="missing", roundtrip=roundtrip, position=slot.position)
        if now_s - float(slot.t_phase) > self.cfg.plan_timeout_s and slot.arrival is None:
            self._fail(f"{slot.bridge.drone_id}:PLAN_TIMEOUT", now_s)
            return self._land_pack(slot, now_s, inbox)
        if self.fleet is not None:
            return self._plan_formation(slot, now_s, state)
        source, frame = resolve_depth_frame(
            slot.bridge,
            state,
            now_s,
            depth=None if inbox is None else inbox.depth,
            obstacle_summary=None if inbox is None else inbox.obstacle_summary,
            depth_source=self.cfg.depth_source,
        )
        slot.depth_source = source
        if source == "missing":
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False, depth_source=source, roundtrip=roundtrip, position=slot.position)
        out = slot.bridge.command(
            state["position_ned"],
            state["velocity_ned"],
            now_s,
            q_ned_frd=state["q_ned_frd"],
            acceleration_ned=state["acceleration_ned"],
            depth_frame=frame,
        )
        self._observe_shadow(slot, now_s, state, planned=out.planned)
        self._note_arrival(slot, now_s, state["position_enu"], state["velocity_enu"])
        return self._pack(
            slot,
            now_s,
            heartbeat=True,
            setpoint=out.setpoint,
            commands=[],
            planned=out.planned,
            depth_source=source,
            roundtrip=out.roundtrip_error_m,
            position=state["position_enu"],
        )

    def _observe_shadow(self, slot: _LiveSlot, now_s: float, state: dict, *, planned: bool) -> None:
        """只读复验本拍选出的计划和跟踪点。不改设定点。逻辑在 shadow_sitl。"""
        shadow_sitl.observe(self, slot, now_s, state, planned=planned)

    def _note_arrival(self, slot: _LiveSlot, now_s: float, position, velocity) -> None:
        """场景卡到达口径：距目标 ≤ 容差且速度 ≤ 上限，连续保持 goal_hold。"""
        err = float(np.linalg.norm(np.asarray(position, dtype=float) - slot.bridge.goal))
        speed = float(np.linalg.norm(np.asarray(velocity, dtype=float)))
        if err <= self.goal_tol and speed <= self.goal_speed:
            slot.goal_since = now_s if slot.goal_since is None else slot.goal_since
            if now_s - slot.goal_since >= self.goal_hold - 1e-9:
                slot.arrival = slot.goal_since
        else:
            slot.goal_since = None

    def _plan_formation(self, slot: _LiveSlot, now_s: float, state: dict) -> LiveCommand:
        """已提交编队计划在编队钟上的一点。不渲染深度，不调用上游网络。"""
        t = self._formation_t
        if t is None:
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            return self._pack(
                slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False,
                depth_source="formation", roundtrip=roundtrip, position=slot.position,
            )
        did = slot.bridge.drone_id
        self.fleet.observe(did, float(t), state["position_enu"])
        if self._explore_telemetry is not None:
            self._explore_telemetry.note_step(did, float(t), state["position_enu"], state["velocity_enu"])
        sampler = self.fleet.track(did, float(t))
        lead = float(self.cfg.setpoint_lead_s)
        p, v, a, yaw = sampler.sample(float(t) + lead if lead > 0.0 else float(t))
        slot.formation_sampler = sampler
        code_now = self.fleet.code(did)
        is_planned = (
            code_now not in ("", "LOCAL_HOVER", None)
            if self.policy == "explore"
            else code_now == "LOCAL_APPROVED"
        )
        self._observe_shadow(slot, now_s, state, planned=is_planned)
        source = "scene-card" if getattr(self.fleet, "ingest", None) is not None else "formation"
        p = np.asarray(p, dtype=float).copy()
        v = np.asarray(v, dtype=float).copy()
        a = np.asarray(a, dtype=float).copy()
        if self.policy == "explore":
            slot.bridge.goal = np.asarray(self.fleet.goal_of(did), dtype=float).copy()
            p, v, a = self._explore_terminal_pva(slot, p, v, a)
        if self.cfg.setpoint_gain_p > 0.0 or self.cfg.setpoint_gain_v > 0.0:
            from yopo_l0.b2_plan_diag import stiffen  # 票 B2，默认不进来

            p, v = stiffen(p, v, state, float(self.cfg.setpoint_gain_p), float(self.cfg.setpoint_gain_v))
        setpoint = trajectory_setpoint_ned(p, v, a, float(yaw), now_s)
        if self.policy == "explore":
            if slot.arrival is None:
                self._note_arrival(slot, now_s, state["position_enu"], state["velocity_enu"])
        else:
            self._note_arrival(slot, now_s, state["position_enu"], state["velocity_enu"])
        return self._pack(
            slot,
            now_s,
            heartbeat=True,
            setpoint=setpoint,
            commands=[],
            planned=is_planned,
            depth_source=source,
            roundtrip=roundtrip_abs(p),
            position=state["position_enu"],
        )

    def _land_pack(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None) -> LiveCommand:
        landed = bool(inbox and inbox.landed)
        armed = True if inbox is None or inbox.armed is None else bool(inbox.armed)
        command = vehicle_command_message("land", int(round(now_s * 1e6)), slot.ident["mav_sys_id"])
        if land_status_done(command, landed=landed, armed=armed) == "DONE":
            slot.landed_done = True
            commands = [command] if not slot.land_sent else []
            slot.land_sent = True
            # 失败之后的落地只表示飞机已经上锁，不把 FAIL 改成 DONE，也不改失败原因。
            if slot.phase != "FAIL" and self.fail_reason == "":
                slot.phase = "DONE"
            return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=commands, planned=False, depth_source=slot.depth_source, roundtrip=0.0, position=slot.position)
        commands = []
        if not slot.land_sent or now_s - slot.last_cmd_s >= self.cfg.arm_retry_s:
            slot.last_cmd_s = now_s
            slot.land_sent = True
            commands = [command]
        timed_out = slot.t_phase is not None and now_s - float(slot.t_phase) > self.cfg.land_timeout_s
        if timed_out:
            if slot.phase != "FAIL" and self.fail_reason == "":
                self._fail(f"{slot.bridge.drone_id}:LAND_TIMEOUT", now_s)
            slot.land_give_up = True
        return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=commands, planned=False, depth_source=slot.depth_source, roundtrip=0.0, position=slot.position)

    def _record_trace(self, slot: _LiveSlot, now_s: float, world_enu, setpoint_world: dict | None) -> None:
        """每个阶段留几拍。世界系设定点是减偏置之前的 NED 转回 ENU。"""
        phase_n = sum(1 for row in slot.trace if row.get("phase") == slot.phase)
        if phase_n >= TRACE_PER_PHASE:
            return
        now_s = float(now_s)
        if slot.trace and abs(float(slot.trace[-1]["t_s"]) - now_s) <= 1e-9:
            return
        sample: dict = {
            "t_s": now_s,
            "phase": slot.phase,
            "yaw_enu": float(slot.yaw),
            "goal_hold": bool(getattr(slot.bridge, "goal_hold", False)),
        }
        if world_enu is None:
            sample["odometry_world_enu"] = None
            sample["odometry_local_enu"] = None
        else:
            world = np.asarray(world_enu, dtype=float).reshape(3)
            sample["odometry_world_enu"] = _vec_list(world)
            sample["odometry_local_enu"] = _vec_list(world - slot.offset_enu)
        if setpoint_world is None or setpoint_world.get("position") is None:
            sample["setpoint_world_enu"] = None
            sample["setpoint_velocity_enu"] = None
            sample["setpoint_velocity_ned"] = None
            sample["setpoint_acceleration_ned"] = None
            sample["setpoint_local_ned"] = None
        else:
            sample["setpoint_world_enu"] = _vec_list(px4.ned_to_enu(setpoint_world["position"]))
            sample["setpoint_velocity_enu"] = _vec_list(px4.ned_to_enu(setpoint_world["velocity"]))
            sample["setpoint_velocity_ned"] = _vec_list(setpoint_world["velocity"])
            sample["setpoint_acceleration_ned"] = _vec_list(setpoint_world.get("acceleration", [0.0, 0.0, 0.0]))
            yaw_sp = setpoint_world.get("yaw")
            sample["setpoint_yaw_ned"] = None if yaw_sp is None or not math.isfinite(float(yaw_sp)) else float(yaw_sp)
            sample["setpoint_local_ned"] = None
        slot.trace.append(sample)

    def _failure_settled(self) -> bool:
        if any(slot.land_give_up for slot in self.slots):
            return True
        return bool(self.slots) and all(slot.landed_done or slot.phase == "DONE" for slot in self.slots)

    def result(self, *, adapter: str = "px4-ros") -> dict:
        n = len(self.slots)
        arrived = sum(slot.arrival is not None for slot in self.slots)
        all_arrived = bool(self.slots) and arrived == n
        all_done = bool(self.slots) and all(slot.phase == "DONE" for slot in self.slots)
        if self.fail_reason:
            outcome, reason, ok = "FAIL", self.fail_reason, False
        elif all_done and all_arrived:
            outcome, reason, ok = "SUCCESS", "GOAL_REACHED_AND_HELD", True
        elif all_done:
            outcome, reason, ok = "FAIL", "GOAL_NOT_REACHED", False
        else:
            outcome, reason, ok = "RUNNING", self.slots[0].phase if self.slots else "", False
        terminal = bool(ok) or (outcome == "FAIL" and self._failure_settled())
        drones = []
        for slot in self.slots:
            drones.append(
                {
                    "id": slot.bridge.drone_id,
                    "px4_namespace": slot.ident["namespace"],
                    "mav_sys_id": int(slot.ident["mav_sys_id"]),
                    "gcs_port": int(slot.ident["gcs_port"]),
                    "phase": slot.phase,
                    "goal_enu": _vec_list(slot.bridge.goal),
                    "world_offset_enu": _vec_list(slot.offset_enu),
                    "arrived": slot.arrival is not None,
                    "arrival_time_s": slot.arrival,
                    "takeoff_handover_s": slot.takeoff_handover_s,
                    "arrival_after_handover_s": (
                        None
                        if slot.arrival is None or slot.takeoff_handover_s is None
                        else float(slot.arrival) - float(slot.takeoff_handover_s)
                    ),
                    "position_enu": None if slot.position is None else _vec_list(slot.position),
                    "depth_source": slot.depth_source,
                    "plans": int(sum(slot.bridge.plan_counts.values())),
                    "plan_code": _formation_plan_code(self.fleet, slot.bridge.drone_id),
                    "trace": list(slot.trace),
                }
            )
        sim_elapsed = None
        wall = None
        if self._sim0 is not None and self._sim1 is not None and self._wall0 is not None:
            sim_elapsed = float(self._sim1) - float(self._sim0)
            wall = time.perf_counter() - float(self._wall0)
        clear_out = None if self._min_clear is math.inf else float(self._min_clear)
        payload = {
            "schema": SCHEMA_LIVE,
            "report_only": True,
            "criteria_frozen": False,
            "adapter": adapter,
            "fake_px4": False,
            "scene": self.card.name,
            "n_drones": n,
            "positioning": self.cfg.positioning,
            "px4_parameters_written": list(self.px4_parameters_written),
            "coordinate_conversion": "yopo_swarm_core.px4",
            "frame_note": self.frame_note,
            "namespaces": [slot.ident["namespace"] for slot in self.slots],
            "mav_sys_ids": [int(slot.ident["mav_sys_id"]) for slot in self.slots],
            "gcs_ports": [int(slot.ident["gcs_port"]) for slot in self.slots],
            "topics": {slot.ident["namespace"]: vehicle_topics(slot.ident["namespace"]) for slot in self.slots},
            "outcome": outcome,
            "reason": reason,
            "ok": ok,
            "terminal": terminal,
            "arrived": arrived,
            "arrival_rate": float(arrived) / float(n),
            "arrival_time_s": max((slot.arrival for slot in self.slots), default=None) if all_arrived else None,
            "takeoff_handover_s": (
                max(slot.takeoff_handover_s for slot in self.slots)
                if self.slots and all(slot.takeoff_handover_s is not None for slot in self.slots)
                else None
            ),
            "arrival_after_handover_s": (
                max(float(slot.arrival) - float(slot.takeoff_handover_s) for slot in self.slots)
                if all_arrived and all(slot.takeoff_handover_s is not None for slot in self.slots)
                else None
            ),
            "landed": bool(self.slots) and all(slot.landed_done or slot.phase == "DONE" for slot in self.slots),
            "land_commands_sent": int(self.land_commands_sent),
            "collisions": 1 if self.fail_reason and (":OBSTACLE" in self.fail_reason or ":INTER_DRONE" in self.fail_reason) else 0,
            "collision_reason": (
                self.fail_reason
                if self.fail_reason and (":OBSTACLE" in self.fail_reason or ":INTER_DRONE" in self.fail_reason)
                else None
            ),
            "body_radius_m": float(self.body),
            "min_inter_drone_distance_m": self._min_inter,
            "min_surface_clearance_m": clear_out,
            "min_surface_clearance_flight_m": None if self._min_clear_flight is math.inf else float(self._min_clear_flight),
            "min_surface_clearance_plan_m": None if self._min_clear_plan is math.inf else float(self._min_clear_plan),
            "min_inter_drone_distance_plan_m": self._min_inter_plan,
            "clearance_note": (
                "min_surface_clearance_m 计全部离地样本（含降落段，语义不变）；"
                "_flight_m 只计 TAKEOFF / PLAN；_plan_m 只计 PLAN，和 L0 可比"
            ),
            "sim_time_s": sim_elapsed,
            "wall_time_s": wall,
            "rtf": realtime_factor(sim_elapsed, wall),
            "enu_ned_roundtrip_max_error_m": float(self.max_roundtrip_m),
            "depth_sources": sorted({slot.depth_source for slot in self.slots if slot.depth_source}),
            "policy": self.policy,
            "drones": drones,
        }
        if self.fleet is not None and self.bus is not None:
            block = self.fleet.summary(arrived, n)
            key = getattr(self.fleet, "result_key", "formation")
            payload[key] = block
            payload["formation_error_p95_m"] = block.get("error_p95_m")
            payload["formation_t0_s"] = self._plan_origin
            payload["formation_time_s"] = self._formation_t
            payload["inter_drone_transport"] = "inprocess"
            payload["bus"] = {
                "drop": float(self.bus.faults.drop),
                "duplicate": float(self.bus.faults.duplicate),
                "delay_s": [float(self.bus.faults.delay[0]), float(self.bus.faults.delay[1])],
                "reorder": float(self.bus.faults.reorder),
                **self.bus.counts(),
            }
            if self._explore_telemetry is not None:
                payload["explore_sitl"] = self._explore_telemetry.summary(self.slots, self.fleet)
        if self.cfg.control_mode != "pva" or self.cfg.record_dt_s > 0.0:
            payload["control_mode"] = self.cfg.control_mode
            payload["flight_record"] = {
                slot.bridge.drone_id: {"samples": slot.flight_samples,
                    "ctbr_ticks": slot.ctbr_ticks, "pva_ticks": slot.pva_ticks,
                    "max_horizontal_displacement_m": slot.max_horizontal_displacement_m,
                    "hover_thrust": None if slot.ctbr is None else slot.ctbr.hover.value,
                    "hover_calibration_samples": 0 if slot.ctbr is None else slot.ctbr.hover.samples}
                for slot in self.slots}
        payload["shadow"] = bool(self.shadow)
        payload["shadow_reject_rate"] = None
        payload["shadow_reject_histogram"] = None
        payload["shadow_gates"] = None
        payload["shadow_drones"] = None
        if self.shadow:
            rate, hist = aggregate(self._shadow_ticks)
            payload["shadow_reject_rate"] = rate
            payload["shadow_reject_histogram"] = hist
            payload["shadow_gates"] = gate_status()
            payload["shadow_drones"] = aggregate_drones(self._shadow_ticks, [slot.bridge.drone_id for slot in self.slots])
        shadow_sitl.annotate_result(self, payload)
        return payload


def make_live_session(
    card: SceneCard,
    *,
    seed: int = 0,
    physics_dt: float = 0.02,
    planner_dt: float = 0.1,
    live: LiveFleetConfig | None = None,
    policy: str = "upstream",
) -> LiveFleetSession:
    """按场景卡建一队桥。1 机 namespace 为空；多机是 ``px4_1`` 起。

    ``policy=formation`` 时目标取编队终点槽位，不加载上游网络。
    机间总线与 ``runner.run`` 用同一盐，零故障时本拍投递。
    """
    name = resolve_live_policy(card, policy)
    cfg = live or LiveFleetConfig()
    drones = list(card.card["drones"])
    n = len(drones)
    dyn = card.card.get("dynamics") or {}
    params = dict(card.card.get("policy_params") or {})
    fleet = None
    bus = None
    if name in FLEET_POLICIES:
        fleet = _fleet_for(name, card, float(physics_dt))
        bus = MessageBus(
            BusFaults.from_mapping(card.card.get("bus")),
            rng=np.random.default_rng(np.random.SeedSequence([int(seed), _BUS_SEED_SALT])),
        )
    bridges = []
    idents = []
    for i, drone in enumerate(drones):
        ident = vehicle_identity(i, n)
        idents.append(ident)
        start = drone["start_enu"]
        if fleet is not None:
            fleet.check_start(drone["id"], np.asarray(start, dtype=float))
            goal = tuple(float(x) for x in fleet.goal_of(drone["id"]))
        else:
            goal = tuple(drone.get("goal_enu", drone.get("hover_enu", start)))
        bridges.append(
            YopoSitlBridge(
                card,
                SitlBridgeConfig(
                    physics_dt=physics_dt,
                    planner_dt=planner_dt,
                    hover_alt_m=float(start[2]),
                    goal_enu=goal,
                    policy_params=params,
                    drone_id=drone["id"],
                    seed=int(seed),
                    drone_index=i,
                    n_drones=n,
                    a_max=float(dyn.get("a_max", 6.0)),
                    tau_s=float(dyn.get("tau_s", 0.1)),
                    kp=float(dyn.get("kp", 4.0)),
                    kd=float(dyn.get("kd", 3.0)),
                    px4_namespace=ident["namespace"],
                    load_upstream=fleet is None,
                ),
            )
        )
    return LiveFleetSession(card, bridges, idents, cfg, fleet=fleet, bus=bus, policy=name)


def live_topic_plan(card: SceneCard, *, positioning: str = "default", seed: int = 0, policy: str = "upstream") -> dict:
    """不起上游网络、不连 ROS。只列出每机话题、system id 和目标，给容器入口的 dry-run。

    formation 会建一次 ``FormationFleet``，用来核对槽位并写出终点。不 exchange。
    """
    positioning = normalize_positioning(positioning)
    name = resolve_live_policy(card, policy)
    drones = list(card.card["drones"])
    n = len(drones)
    idents = [vehicle_identity(i, n) for i in range(n)]
    fleet = _fleet_for(name, card, 0.02) if name in FLEET_POLICIES else None
    goals = []
    for drone in drones:
        if fleet is not None:
            goals.append(_vec_list(fleet.goal_of(drone["id"])))
        else:
            goals.append(_vec_list(drone.get("goal_enu", drone.get("hover_enu", drone["start_enu"]))))
    return {
        "schema": SCHEMA_LIVE,
        "report_only": True,
        "criteria_frozen": False,
        "dry_run": True,
        "adapter": "px4-ros",
        "fake_px4": False,
        "scene": card.name,
        "seed": int(seed),
        "n_drones": n,
        "policy": name,
        "shadow": name == "upstream",
        "inter_drone_transport": "inprocess" if name in FLEET_POLICIES else None,
        "positioning": positioning,
        "px4_parameters_written": [],
        "coordinate_conversion": "yopo_swarm_core.px4",
        "namespaces": [ident["namespace"] for ident in idents],
        "mav_sys_ids": [ident["mav_sys_id"] for ident in idents],
        "gcs_ports": [ident["gcs_port"] for ident in idents],
        "drone_ids": [drone["id"] for drone in drones],
        "topics": {ident["namespace"]: vehicle_topics(ident["namespace"]) for ident in idents},
        "goals_enu": goals,
    }


def run_mock_ros_closed_loop(
    card: SceneCard,
    *,
    positioning: str = "default",
    land_on_arrival: bool = True,
    **kwargs,
) -> dict:
    """假 PX4 上走 ROS 消息字段的闭环。默认在到达并保持之后补一条 NAV_LAND 收尾。

    物理仍是 ``FakePx4Vehicle``。``positioning`` 只写入结果，不产生任何参数写入。
    """
    positioning = normalize_positioning(positioning)
    result = dict(run_closed_loop(card, ros_messages=True, **kwargs))
    n = int(result["n_drones"])
    idents = [vehicle_identity(i, n) for i in range(n)]
    for row, ident in zip(result["drones"], idents):
        if row["px4_namespace"] != ident["namespace"]:
            raise RuntimeError(
                f"{row['id']} 的 namespace {row['px4_namespace']!r} 和 {ident['namespace']!r} 不一致"
            )
    commands: list[dict] = []
    landed = False
    finish = str(result["reason"])
    ok = False
    if land_on_arrival and result["outcome"] == "SUCCESS":
        stamp = int(round(float(result["sim_duration_s"]) * 1e6))
        commands = [vehicle_command_message("land", stamp, ident["mav_sys_id"]) for ident in idents]
        if commands and all(land_status_done(cmd, landed=True, armed=False) == "DONE" for cmd in commands):
            landed = True
            finish = "NAV_LAND"
            ok = True
    result.update(
        schema=SCHEMA_LIVE,
        adapter="mock-ros",
        fake_px4=True,
        positioning=positioning,
        px4_parameters_written=[],
        perception="ros-depth",
        namespaces=[ident["namespace"] for ident in idents],
        mav_sys_ids=[ident["mav_sys_id"] for ident in idents],
        gcs_ports=[ident["gcs_port"] for ident in idents],
        topics={ident["namespace"]: vehicle_topics(ident["namespace"]) for ident in idents},
        land_commands=commands,
        landed=landed,
        finish=finish,
        ok=ok,
        report_only=True,
        criteria_frozen=False,
    )
    return result


# 票 S8：把场景卡里的多圆柱（含倾斜树干）和矮枝静态几何转成 Gazebo Harmonic SDF 模型。
CORRIDOR_SHELL_NAMES = frozenset({"ceiling", "left_wall", "right_wall", "end_wall", "back_wall"})

DEFAULT_MINIMAL_WORLD_SDF = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
  </world>
</sdf>
"""


def _xml_attr(value: str) -> str:
    return (
        str(value)
        .replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def _fmt_sdf_num(value: float) -> str:
    val = 0.0 if abs(float(value)) < 1e-12 else float(value)
    text = f"{val:.9f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def cylinder_sdf_geometry(cyl: Cylinder) -> dict:
    """``Cylinder`` → Gazebo SDF 圆柱的中心位姿 ``(x, y, z, roll, pitch, yaw)``、半径与轴长。

    SDF 圆柱局部对称轴为 ``+z``，中心在局部原点；外旋 XYZ（即 ``R_z(yaw) R_y(pitch) R_x(roll)``）
    在 ``roll=0`` 时将 ``(0, 0, 1)`` 映到 ``(cos(yaw)*sin(pitch), sin(yaw)*sin(pitch), cos(pitch))``，
    因此上下端面中心与 ``cyl.axis_ends()`` 精确重合。
    """
    if not float(cyl.radius) > 0.0:
        raise ValueError(f"圆柱 {cyl.name!r} 半径必须为正，收到 {cyl.radius}")
    base, top = cyl.axis_ends()
    delta = top - base
    length = float(np.linalg.norm(delta))
    if not length > 0.0 or not float(cyl.z_hi) > float(cyl.z_lo):
        raise ValueError(f"圆柱 {cyl.name!r} 高度必须为正，收到 z_lo={cyl.z_lo}, z_hi={cyl.z_hi}")
    mid = 0.5 * (base + top)
    roll = 0.0
    if cyl.upright():
        pitch = 0.0
        yaw = 0.0
    else:
        horiz = math.hypot(float(delta[0]), float(delta[1]))
        pitch = math.atan2(horiz, float(delta[2]))
        yaw = math.atan2(float(delta[1]), float(delta[0]))
    return {
        "name": str(cyl.name),
        "pose": (float(mid[0]), float(mid[1]), float(mid[2]), roll, pitch, yaw),
        "radius": float(cyl.radius),
        "length": length,
        "upright": bool(cyl.upright()),
    }


def box_sdf_geometry(box: Box) -> dict:
    """``Box`` → Gazebo SDF 盒子的中心位姿与三维边长。"""
    lo = np.asarray(box.lo, dtype=float).reshape(3)
    hi = np.asarray(box.hi, dtype=float).reshape(3)
    size = hi - lo
    if np.any(size <= 0.0):
        raise ValueError(f"盒子 {box.name!r} 尺寸必须为正，收到 lo={box.lo}, hi={box.hi}")
    mid = 0.5 * (lo + hi)
    return {
        "name": str(box.name),
        "pose": (float(mid[0]), float(mid[1]), float(mid[2]), 0.0, 0.0, 0.0),
        "size": (float(size[0]), float(size[1]), float(size[2])),
    }


def scene_world_obstacles(
    card: SceneCard,
    *,
    include_branches: bool = True,
    include_corridor_shell: bool = False,
) -> tuple[list[dict], list[dict]]:
    """提取场景卡中的静态圆柱与矮枝盒子（默认跳过走廊外框墙与顶棚）。"""
    cylinders: list[dict] = []
    boxes: list[dict] = []
    for obs in card.obstacles:
        if isinstance(obs, Cylinder):
            cylinders.append(cylinder_sdf_geometry(obs))
        elif isinstance(obs, Box):
            if obs.name in CORRIDOR_SHELL_NAMES and not include_corridor_shell:
                continue
            if include_branches:
                boxes.append(box_sdf_geometry(obs))
    return cylinders, boxes


def render_obstacle_models_sdf(
    cylinders: list[dict],
    boxes: list[dict] | None = None,
    *,
    model_prefix: str = "yopo_obs_",
) -> str:
    """渲染静态障碍 ``<model>`` 片段（只含静态碰撞体与视觉几何）。"""
    seen: set[str] = set()
    blocks: list[str] = []
    for item in cylinders:
        model_name = f"{model_prefix}{item['name']}"
        if model_name in seen:
            raise ValueError(f"重复的 SDF 模型名：{model_name}")
        seen.add(model_name)
        pose_str = " ".join(_fmt_sdf_num(v) for v in item["pose"])
        rad_str = _fmt_sdf_num(item["radius"])
        len_str = _fmt_sdf_num(item["length"])
        esc_name = _xml_attr(model_name)
        geom_xml = f"<cylinder><radius>{rad_str}</radius><length>{len_str}</length></cylinder>"
        blocks.append(
            f'    <model name="{esc_name}">\n'
            f"      <static>true</static>\n"
            f"      <pose>{pose_str}</pose>\n"
            f'      <link name="link">\n'
            f'        <collision name="collision"><geometry>{geom_xml}</geometry></collision>\n'
            f'        <visual name="visual"><geometry>{geom_xml}</geometry></visual>\n'
            f"      </link>\n"
            f"    </model>"
        )
    for item in boxes or []:
        model_name = f"{model_prefix}{item['name']}"
        if model_name in seen:
            raise ValueError(f"重复的 SDF 模型名：{model_name}")
        seen.add(model_name)
        pose_str = " ".join(_fmt_sdf_num(v) for v in item["pose"])
        size_str = " ".join(_fmt_sdf_num(v) for v in item["size"])
        esc_name = _xml_attr(model_name)
        geom_xml = f"<box><size>{size_str}</size></box>"
        blocks.append(
            f'    <model name="{esc_name}">\n'
            f"      <static>true</static>\n"
            f"      <pose>{pose_str}</pose>\n"
            f'      <link name="link">\n'
            f'        <collision name="collision"><geometry>{geom_xml}</geometry></collision>\n'
            f'        <visual name="visual"><geometry>{geom_xml}</geometry></visual>\n'
            f"      </link>\n"
            f"    </model>"
        )
    return "\n".join(blocks)


def build_scene_world_sdf(
    card: SceneCard,
    *,
    base_sdf: str | None = None,
    world_name: str = "default",
    include_branches: bool = True,
    include_corridor_shell: bool = False,
) -> tuple[str, dict]:
    """基于基础世界 SDF 注入场景卡圆柱与矮枝，返回 ``(sdf_xml, summary_dict)``。"""
    import hashlib
    import xml.etree.ElementTree as ET

    template = DEFAULT_MINIMAL_WORLD_SDF if base_sdf is None else str(base_sdf)
    root = ET.fromstring(template)
    world_el = root.find("world")
    if world_el is None:
        raise ValueError("基础 SDF 缺少 <world> 节点")
    if isinstance(card.card.get("wide_field"), dict):
        size = 2.0 * float(card.card["wide_field"]["half_extent_m"]) + 200.0
        for plane_size in world_el.findall(".//geometry/plane/size"):
            plane_size.text = f"{size:g} {size:g}"
        template = ET.tostring(root, encoding="unicode")
    actual_world = world_el.attrib.get("name", "")
    if world_name and actual_world != world_name:
        raise ValueError(f"基础 SDF world name={actual_world!r} 与期望 {world_name!r} 不一致")
    cylinders, boxes = scene_world_obstacles(
        card,
        include_branches=include_branches,
        include_corridor_shell=include_corridor_shell,
    )
    models_xml = render_obstacle_models_sdf(cylinders, boxes)
    idx = template.rfind("</world>")
    if idx < 0:
        raise ValueError("基础 SDF 缺少闭合 </world>")
    if models_xml:
        sdf_text = template[:idx] + models_xml + "\n  " + template[idx:]
    else:
        sdf_text = template
    # 校验拼接后的完整 SDF 仍是合法 XML。
    ET.fromstring(sdf_text)
    summary = {
        "enabled": True,
        "scene": card.name,
        "world_name": actual_world or world_name,
        "n_cylinders": len(cylinders),
        "n_leaning_cylinders": sum(1 for c in cylinders if not c["upright"]),
        "n_boxes": len(boxes),
        "sdf_sha256": hashlib.sha256(sdf_text.encode("utf-8")).hexdigest(),
        "px4_parameters_written": [],
    }
    return sdf_text, summary


def write_scene_world_sdf(
    card: SceneCard,
    out_dir: str | Path,
    *,
    base_sdf_path: str | Path | None = None,
    world_name: str = "default",
    include_branches: bool = True,
) -> dict:
    """把合成后的 ``<world_name>.sdf`` 写入 ``out_dir``，并链接基础目录里的同级世界文件。"""
    dest = Path(out_dir)
    dest.mkdir(parents=True, exist_ok=True)
    base_text = None
    if base_sdf_path is not None:
        src_file = Path(base_sdf_path)
        if not src_file.is_file():
            raise ValueError(f"找不到基础世界 SDF：{base_sdf_path}")
        base_text = src_file.read_text(encoding="utf-8")
        for sibling in sorted(src_file.parent.iterdir()):
            if sibling.name == f"{world_name}.sdf":
                continue
            target = dest / sibling.name
            if not target.exists() and not target.is_symlink():
                target.symlink_to(sibling.resolve())
    sdf_text, summary = build_scene_world_sdf(
        card,
        base_sdf=base_text,
        world_name=world_name,
        include_branches=include_branches,
    )
    out_sdf = dest / f"{world_name}.sdf"
    out_sdf.write_text(sdf_text, encoding="utf-8")
    summary["sdf_path"] = str(out_sdf)
    return summary
