"""票 B1：PX4 SITL + Gazebo 的仿真钟锁步（方向 B，默认关，只报数）。

PX4 v1.17 的 GZBridge 在每个 ``/world/<w>/clock`` 回调里把自己的单调钟设成 gz 仿真钟，
所以 PX4 已经「跟着」gz 走。缺的是另一半：gz 自己按墙钟自由跑，不等 PX4，也不等代理。
代理一慢（网络 + 深度渲染），仿真时间照走，PX4 在主机抢核时收不到 IMU，报传感器超时。

锁步做法（全在代理侧，不改 PX4 参数，不改镜像）：

1. 进入主循环先暂停世界（``WorldControl.pause``）。
2. 每个物理步（默认 world 的 ``max_step_size`` 0.004 s）发一次
   ``WorldControl(pause=True, multi_step=1)``：世界走一步后仍停着。
   不用 ``run_to_sim_time``：它要带 ``pause=False``，目标一旦落在当前时刻之前，gz 就直接
   放开世界自由跑（B1 第一版实测：代理每拍睡 20 ms 时世界跑到步进器前面 40–56 ms）。
   请求回复丢了不要紧（实测约 3% 不回，但已生效），按 clock 判断；200 ms clock 没动才重发。
3. 握手：等 ``/world/<w>/clock`` 走到 ``t_k``，并且每架 PX4 在 gz 上发出的
   ``/<model>/command/motor_speed`` 在发出这一步请求之后又来了新的一条。PX4 每吃一帧 IMU 出一条
   电机指令（实测 250 Hz、每步恰好 1 条），所以这一条到了就说明这架 PX4 已经算完了这一步。
   等不到就抛 ``TimeoutError``，入口写失败结果，不丢下慢实例继续跑。
4. 代理在两个物理步之间算多久都行：世界停着，PX4 的钟也停着，``COM_OF_LOSS_T`` 和传感器
   超时都按仿真钟算，不会触发。

所有 PX4 实例接同一个 gz 服务端、同一条 world 钟，所以多机天然共享一个仿真钟；
握手按机等齐，最慢的那架决定这一步什么时候结束。

本模块不导入 gz 或 rclpy。``WorldPort`` 是一个最小接口，容器里由 ``yopo_l0_live.py``
用 gz-transport 实现；单元测试用假的世界。
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

SCHEMA_LOCKSTEP = "yopo-restart/b1-sitl-lockstep/1"
# PX4 gz 默认 world（Tools/simulation/gz/worlds/default.sdf）的 max_step_size。
DEFAULT_PHYSICS_STEP_S = 0.004
# 一次请求等回复的上限。gz-transport 的 python 请求偶尔不回，但请求已经生效；
# 回不回都靠观察 clock 判断，所以这里设得很短。
REQUEST_TIMEOUT_MS = 5
# clock 这么久没走到目标就重发同一个 run_to_sim_time（幂等）。
RESEND_AFTER_S = 0.2
# 一步里等 PX4 电机指令的墙钟上限。超时中止，绝不丢掉慢实例继续跑。
HANDSHAKE_TIMEOUT_S = 10.0


class WorldPort(Protocol):
    """锁步需要的 gz 侧最小接口。"""

    def sim_time(self) -> float | None: ...

    def motor_counts(self) -> dict[str, int]: ...

    def request_pause(self) -> None: ...

    def request_step(self) -> None: ...

    def request_resume(self) -> None: ...


def model_name(instance: int) -> str:
    """与 ``scripts/sitl/fleet.py`` 相同：instance i 的 gz 模型名是 ``x500_i``。"""
    i = int(instance)
    if i < 0:
        raise ValueError(f"instance 不能为负，收到 {instance!r}")
    return f"x500_{i}"


def motor_topic(instance: int) -> str:
    """PX4 GZMixingInterfaceESC 在 gz 上发电机转速的话题。"""
    return f"/{model_name(instance)}/command/motor_speed"


def fleet_handshake_topics(n_drones: int) -> list[str]:
    """按本仓库实例编号返回 ``n_drones`` 架飞机的锁步电机握手话题（1 机是 x500_0，多机是 x500_1..x500_n）。"""
    if isinstance(n_drones, bool) or not isinstance(n_drones, int) or n_drones < 1:
        raise ValueError(f"n_drones 必须是正整数，收到 {n_drones!r}")
    if n_drones == 1:
        return [motor_topic(0)]
    return [motor_topic(i) for i in range(1, n_drones + 1)]


def quantize(t_sim: float, step_s: float) -> float:
    """对齐到物理步网格，避免浮点累积让目标落在两步之间。"""
    k = round(float(t_sim) / float(step_s))
    return round(k * float(step_s), 9)


def split_time(t_sim: float) -> tuple[int, int]:
    """gz.msgs.Time 的 (sec, nsec)。"""
    t = float(t_sim)
    if not math.isfinite(t) or t < 0.0:
        raise ValueError(f"仿真时刻非法：{t_sim!r}")
    ns_total = int(round(t * 1e9))
    return ns_total // 1_000_000_000, ns_total % 1_000_000_000


@dataclass
class LockstepStats:
    engaged: bool = False
    physics_step_s: float = DEFAULT_PHYSICS_STEP_S
    t_engage_s: float | None = None
    steps: int = 0
    resends: int = 0
    handshake_timeouts: dict[str, int] = field(default_factory=dict)
    clock_overshoots: int = 0
    clock_ahead: int = 0  # 步进前发现世界已经跑在前面的次数（应为 0）
    clock_ahead_max_s: float = 0.0
    step_wall_s: list[float] = field(default_factory=list)
    agent_wall_s: float = 0.0  # 世界停着、代理在算的墙钟合计
    stepping_wall_s: float = 0.0  # 推进世界 + 等握手的墙钟合计

    def summary(self) -> dict:
        arr = np.asarray(self.step_wall_s, dtype=float)
        sim = self.steps * float(self.physics_step_s)
        total = self.agent_wall_s + self.stepping_wall_s
        return {
            "schema": SCHEMA_LOCKSTEP,
            "enabled": True,
            "engaged": bool(self.engaged),
            "mechanism": "gz WorldControl pause+multi_step=1 per physics step + PX4 motor_speed handshake",
            "physics_step_s": float(self.physics_step_s),
            "t_engage_s": self.t_engage_s,
            "steps": int(self.steps),
            "sim_stepped_s": round(sim, 6),
            "resends": int(self.resends),
            "handshake_timeouts": dict(sorted(self.handshake_timeouts.items())),
            "handshake_timeouts_total": int(sum(self.handshake_timeouts.values())),
            "clock_overshoots": int(self.clock_overshoots),
            "clock_ahead": int(self.clock_ahead),
            "clock_ahead_max_s": round(float(self.clock_ahead_max_s), 6),
            "step_wall_ms_median": None if arr.size == 0 else float(np.median(arr) * 1e3),
            "step_wall_ms_p95": None if arr.size == 0 else float(np.percentile(arr, 95) * 1e3),
            "step_wall_ms_max": None if arr.size == 0 else float(arr.max() * 1e3),
            "agent_wall_s": round(float(self.agent_wall_s), 3),
            "stepping_wall_s": round(float(self.stepping_wall_s), 3),
            "rtf_locked": None if total <= 0.0 else round(sim / total, 4),
        }


class LockstepStepper:
    """按物理步推进 gz，每步等齐所有 PX4 的电机指令。

    ``now`` / ``sleep`` 可注入，单元测试不用真墙钟。
    """

    def __init__(
        self,
        world: WorldPort,
        namespaces: list[str],
        *,
        physics_step_s: float = DEFAULT_PHYSICS_STEP_S,
        handshake_timeout_s: float = HANDSHAKE_TIMEOUT_S,
        resend_after_s: float = RESEND_AFTER_S,
        now: Callable[[], float] = time.perf_counter,
        sleep: Callable[[float], None] = time.sleep,
        poll_s: float = 0.0002,
    ):
        if not physics_step_s > 0.0:
            raise ValueError(f"physics_step_s 必须为正，收到 {physics_step_s!r}")
        if not namespaces:
            raise ValueError("锁步至少要一架")
        self.world = world
        self.namespaces = list(namespaces)
        self.step_s = float(physics_step_s)
        self.handshake_timeout_s = float(handshake_timeout_s)
        self.resend_after_s = float(resend_after_s)
        self._now = now
        self._sleep = sleep
        self.poll_s = float(poll_s)
        self.stats = LockstepStats(physics_step_s=self.step_s)
        self.t: float | None = None  # 当前已握手完成的仿真时刻
        self._mark: float | None = None  # 上一次推进结束的墙钟，代理计时从这里起

    # -- 生命周期 -------------------------------------------------------
    def engage(self, settle_s: float = 0.05, timeout_s: float = 10.0) -> float:
        """暂停世界，等 clock 停稳、在途电机指令收完。返回暂停时的仿真时刻。"""
        t0 = self._now()
        last_req = -math.inf
        last_t = None
        last_counts = None
        still_since = None
        while True:
            now = self._now()
            if now - last_req >= self.resend_after_s:
                self.world.request_pause()
                last_req = now
            t = self.world.sim_time()
            counts = self.world.motor_counts()
            # clock 停住、各机在途的电机指令也收完，基线才不会被旧消息提前满足。
            if t is not None and last_t is not None and abs(t - last_t) < 1e-9 and counts == last_counts:
                still_since = still_since if still_since is not None else now
                if now - still_since >= settle_s:
                    break
            else:
                still_since = None
            last_t = t
            last_counts = counts
            if now - t0 > timeout_s:
                raise TimeoutError("锁步：暂停世界超时（clock 一直在走或没有 clock）")
            self._sleep(0.005)
        t_now = float(last_t)
        self.t = t_now
        self.stats.engaged = True
        self.stats.t_engage_s = t_now
        self._mark = self._now()
        return t_now

    def release(self) -> None:
        """结束时放开世界，让 PX4 正常收尾。"""
        try:
            self.world.request_resume()
        except Exception:  # pragma: no cover - 收尾尽力而为
            pass

    # -- 推进 -----------------------------------------------------------
    def advance(self, dt_s: float) -> float:
        """从当前时刻往前走 ``dt_s``（按物理步取整，至少一步）。返回新时刻。"""
        if self.t is None:
            raise RuntimeError("先 engage()")
        if self._mark is not None:
            self.stats.agent_wall_s += max(0.0, self._now() - self._mark)
        n = max(1, int(round(float(dt_s) / self.step_s)))
        w0 = self._now()
        for _ in range(n):
            self._one_step()
        self.stats.stepping_wall_s += self._now() - w0
        self._mark = self._now()
        return float(self.t)

    def _one_step(self) -> None:
        w0 = self._now()
        t_obs = self.world.sim_time()
        if t_obs is not None and t_obs > float(self.t) + 0.5 * self.step_s:
            # 世界自己跑到前面去了：记下来，从实际时刻接着走，不假装还在锁步。
            self.stats.clock_ahead += 1
            self.stats.clock_ahead_max_s = max(self.stats.clock_ahead_max_s, t_obs - float(self.t))
            self.t = float(t_obs)  # gz clock 本来就在步长网格上，不再量化，免得目标落到够不着的格点
        target = round(float(self.t) + self.step_s, 9)
        before = self.world.motor_counts()
        self.world.request_step()
        last_req = w0
        # 1) clock 走到目标
        while True:
            t = self.world.sim_time()
            if t is not None and t >= target - 1e-7:
                if t > target + 0.5 * self.step_s:
                    self.stats.clock_overshoots += 1
                break
            now = self._now()
            if now - w0 > 10.0:
                raise TimeoutError("锁步：Gazebo clock 推进超时")
            if now - last_req >= self.resend_after_s and (t is None or t <= float(self.t) + 1e-7):
                # clock 一点没动才重发；已经在动就再等等，避免多走一步。
                self.world.request_step()
                self.stats.resends += 1
                last_req = now
            self._sleep(self.poll_s)
        # 本步之前的消息绝不顶账；每个实例都必须在发步进请求之后出新指令。
        waiting = list(self.namespaces)
        while waiting:
            counts = self.world.motor_counts()
            waiting = [ns for ns in waiting if int(counts.get(ns, 0)) <= before[ns]]
            if not waiting:
                break
            if self._now() - w0 > self.handshake_timeout_s:
                for ns in waiting:
                    self.stats.handshake_timeouts[ns] = self.stats.handshake_timeouts.get(ns, 0) + 1
                raise TimeoutError("锁步：PX4 电机握手超时：" + ",".join(waiting))
            self._sleep(self.poll_s)
        self.t = max(target, float(self.world.sim_time() or target))
        self.stats.steps += 1
        self.stats.step_wall_s.append(self._now() - w0)


class SimStampSkewMeter:
    """票 B1：用仿真钟记每机最新里程计「是哪一刻收到的」，不依赖 PX4 的时间戳换算。

    PX4 的 uXRCE 把出站时间戳加上「代理墙钟 − PX4 钟」的偏移（``UXRCE_DDS_SYNCT`` 默认 1）。
    PX4 钟跟着 gz 仿真钟，所以 S7 的 ``ClockSkewMeter`` 量到的里程计时间戳是墙钟 epoch 量级，
    仿真越慢偏得越多。本表只用收到消息时的 gz 仿真钟：
    - 单机 ``age``：当前仿真时刻 − 最新一条里程计到达时的仿真时刻。
    - 机间 ``spread``：同一拍各机最新里程计到达时刻的极差。
    """

    def __init__(self) -> None:
        self.ages: dict[str, list[float]] = {}
        self.spreads: list[float] = []

    def note(self, t_sim: float | None, recv_sim: dict[str, float | None]) -> None:
        if t_sim is None:
            return
        row = []
        for ns, ts in recv_sim.items():
            if ts is None:
                continue
            self.ages.setdefault(ns, []).append(float(t_sim) - float(ts))
            row.append(float(ts))
        if len(row) >= 2:
            self.spreads.append(max(row) - min(row))

    def summary(self) -> dict | None:
        if not self.ages:
            return None
        per = {}
        for ns, vals in sorted(self.ages.items()):
            arr = np.asarray(vals, dtype=float)
            per[ns or "/"] = {
                "samples": int(arr.size),
                "age_median_s": float(np.median(arr)),
                "age_p95_s": float(np.percentile(arr, 95)),
                "age_max_s": float(arr.max()),
            }
        sp = np.asarray(self.spreads, dtype=float)
        return {
            "per_vehicle": per,
            "inter_vehicle_spread_median_s": None if sp.size == 0 else float(np.median(sp)),
            "inter_vehicle_spread_p95_s": None if sp.size == 0 else float(np.percentile(sp, 95)),
            "inter_vehicle_spread_max_s": None if sp.size == 0 else float(sp.max()),
        }


_TIMEOUT_RE = re.compile(r"\]\s*(?P<name>[^\]]*?)\s+fail:\s+TIMEOUT")


def count_sensor_timeouts(text: str) -> dict[str, int]:
    """从 PX4 日志数传感器超时行，如 ``ERROR [sensors] Accel #0 fail:  TIMEOUT!``。键是传感器名。"""
    out: dict[str, int] = {}
    for line in str(text).splitlines():
        m = _TIMEOUT_RE.search(line)
        if m is None:
            continue
        name = m.group("name").strip() or "unknown"
        out[name] = out.get(name, 0) + 1
    return out
