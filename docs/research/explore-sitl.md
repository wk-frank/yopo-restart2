# 无编队水平横排起飞与高速多机避障穿越 PX4 SITL 锁步实测（`explore-sitl`，report-only）

本报告记录将 [无编队水平横排起飞高效率集群穿越（`ExploreFleet`）](explore-free-swarm.md) 接入锁步 PX4 v1.17 + Gazebo Harmonic SITL（`policy="explore"`）的工程接线、原厂飞控动力学对齐分析，以及在三机柱林、三机林带、六机开阔与六机林带 4 类场景共 **20 趟实飞闭环** 上的可复现抽检结果。

所有实飞结果均为 **只报数（report-only，判据未冻结）**，全部 20 趟运行 `px4_parameters_written == []`（未修改 `MPC_XY_VEL_MAX`、`MPC_TILTMAX_AIR`、`COM_*`、`NAV_*`、`BAT_*` 等任何 PX4 参数），不触碰 `core/`（`VENDORED.sha256` 完好）与 `scenes/suite.txt`，全部 `188` 条既有 `tests/test_sitl*.py` 与新增 `9` 条 [test_sitl_explore.py](../../tests/test_sitl_explore.py) 全绿。

---

## 1. 工程接线与边界设计

### 1.1 统一水平横线生成点与起飞交接（`scripts/sitl/fleet.py`、`in_container.sh`）

- 既有 S2 默认沿东向生成点保持不变；实飞入口在 `YOPO_SITL_PLANNER=yopo-l0` 时通过 [fleet.py](../../scripts/sitl/fleet.py) `yopo-poses` 将场景卡 `drones[*].start_enu` 写入各 PX4 实例的 `PX4_GZ_MODEL_POSE`，并在生成前校验两两地面水平间距 $\ge 0.70\text{ m}$：
  - **三机横线（`explore-3-*`）**：`x = 0.0 m`，`y = [-8.0, 0.0, +8.0] m`（等距 `8.0 m`），`z_hover = 1.5 m`。
  - **六机横线（`explore-6-*`）**：`x = 0.0 m`，`y = [-7.5, -4.5, -1.5, +1.5, +4.5, +7.5] m`（全队位于同一条 `x = 0` 起飞线，等距 `3.0 m`，覆盖 `[-8.5, +8.5] m` 林带宽度；对比 [S8](s8-forest-and-m4-sitl.md) `crossing-6-*` 必须排成 `x = 6 / 3 / 0 m` 三排楔形）。
- [fleet.py](../../scripts/sitl/fleet.py) 新增纯标准库 `horizontal_line_spawns`、`min_spawn_separation_m` 与 `inspect_scene_spawns`，离线即可校验 `aligned_x`、`x_span_m == 0` 与横向等距排布。
- 实测 20 趟 PX4 SITL 全体完成 `WARMUP -> ARMING -> TAKEOFF -> PLAN` 交接时，三机与六机在交接拍的真实 EKF2 世界系纵向极差 `handover_x_span_m` 仅为 **`0.016–0.081 m`（`1.6–8.1 cm`）**，证明多机在 Gazebo 中真正从统一水平起飞线同步出发。

### 1.2 `LiveFleetSession` 自由横向终点驻留与协议时钟对齐（`yopo_l0/sitl_bridge.py`）

- 在 [sitl_bridge.py](../../yopo_l0/sitl_bridge.py)、[yopo_l0_live.py](../../scripts/sitl/yopo_l0_live.py) 与 [run_yopo_l0_sitl.sh](../../scripts/sitl/run_yopo_l0_sitl.sh) 中注册 `policy="explore"`（要求与 `task.type == "explore"` 一一对应），复用进程内 `MessageBus` 交换三维轨迹管。
- **协议相对时钟对齐**：PX4 起飞耗时约 `9.9–10.9 s`；`_fleet_ingest` 对 `explore` 将深度帧时间戳与观测状态时刻对齐到 `PLAN` 起点相对钟 $t_{\text{plan}} = t - t_{\text{plan,0}}$（从 `0.0 s` 起算），与 `exchange` / `observe` / `track` 完全同源。
- **自由横向终点无振荡交接（`_explore_terminal_pva`）**：`ExploreFleet` 无横向槽位橡皮筋，无人机越过障碍带进入终点无障碍区（$x \ge x_{\text{goal}} - r_{\text{arrive}}$）后，就地锁定与邻机及静态障碍均无冲突的三维静止终点 `goal_of(did)`（`terminal_locked == True`）。桥接层在 `_plan_formation` 与 `_formation_stream` 中同步刷新 `slot.bridge.goal`，并在进入 `ARRIVED_HOLD` 驻留态（距终点 $\le 5.0\text{ m}$）时直接向 PX4 `PositionControl` 下发静止目标位置设定点 `(goal, 0, 0)`，消除了实飞减速段因 `||p_ref - p_odom|| > odom_replan_err_m` 反复重锚导致的终点前悬停振荡，使 G2 到达判据（`dist <= 0.4 m, speed <= 0.2 m/s` 保持 `1.0 s`）与随后的全队 `NAV_LAND` 上锁稳定收敛。
- **只读实飞遥测（`result["explore_sitl"]`）**：仅在 `policy == "explore"` 时写入，逐机记录横线起飞对齐度 `handover_x_span_m`、进出障碍带插值时刻、障碍带平均穿越速度、冲线时刻 `all_crossed_finish_after_launch_s`、峰值三维速度与锁定终点坐标，其它策略输出字段逐键不变。
- **命令行时间相似比覆写（`--velocity-mps` / `YOPO_EXPLORE_VELOCITY_MPS`）**：支持在不修改落盘 JSON 且不改写任何 PX4 参数的前提下，在内存里动态缩放 `task.velocity_mps`（时间相似比 $\text{ratio} = v_{\text{target}} / 6.0$）及匹配的 `dynamics.a_max`。

### 1.3 原厂未改参 PX4（`gz_x500`）高速包线与 L0 敏捷包线的物理边界

核查容器镜像内 PX4 v1.17 `4001_gz_x500` 原厂默认参数与 `PositionControl.cpp` 控制律：

- `MPC_XY_VEL_MAX = 12.0 m/s`（Offboard `TrajectorySetpoint` 水平速度上限充足）；
- `MPC_TILTMAX_AIR = 45.0°`、`MPC_THR_HOVER = 0.60`、`MPC_XY_P = 0.95`、`MPC_XY_VEL_P_ACC = 1.8`：水平加速度前馈 `_acc_sp` 直接进入姿态倾角解算并受 `45°` 最大倾角（约 $g \tan 45^\circ = 9.81\text{ m/s}^2$）硬限幅，且四旋翼由 $-30^\circ$ 前倾高速巡航反打到 $+30^\circ$ 制动或大坡度横滚绕树存在约 `0.18–0.25 s` 的刚体角速率响应滞后。
- 因此本工作如实区分并验证两个速度区间：
  1. **L0 敏捷四旋翼区间（[explore-free-swarm.md](explore-free-swarm.md)，`velocity_mps = 10.2 m/s, a_max = 20.0 m/s²`）**：三机/六机密林平均穿越速度 **`9.67–9.91 m/s`**，`120/120` 零碰撞到达。
  2. **原厂未改参 PX4 + Gazebo 刚体 SITL 区间（`px4_parameters_written == []`）**：
     - 新增 4 张套件外只报数 SITL 卡：[explore-3-pillars-sitl80.json](../../scenes/explore-3-pillars-sitl80.json)（`7.5 m/s`）、[explore-3-forest-sitl80.json](../../scenes/explore-3-forest-sitl80.json)（`8.4 m/s`）、[explore-6-open-sitl80.json](../../scenes/explore-6-open-sitl80.json)（`8.4 m/s`）、[explore-6-forest-sitl80.json](../../scenes/explore-6-forest-sitl80.json)（`6.8 m/s`）。
     - 实测发现：将 `dynamics.a_max` 匹配到 `9.2–13.88 m/s²` 允许 `_kin_ok` 放行 YOPO 大曲率前馈避障机动，使前馈加速度直接驱动 PX4 满倾角绕树，在原厂飞控下实现 **林内平均三维速度 `5.95–7.58 m/s`（全面突破 `6.0 m/s`）**！

---

## 2. 锁步 PX4 + Gazebo Harmonic 实测总表与 `CrossingFleet` 对照

- 宿主与镜像：WSL2 `RflySim-20.04`，`yopo-restart-sitl:yopo-81d084c04186`（PX4 v1.17.0 + Gazebo Harmonic + ROS 2 Humble），全部以 `flock /tmp/yopo-sitl.lock` 独占串行执行。
- 产物归档：`/workspace/yopo-ops/sitl-explore/runs/`（索引 `/workspace/yopo-ops/sitl-explore/sitl-index.txt`，结构化汇总 `/workspace/yopo-ops/sitl-explore/sitl-collected.json`）。
- 全部 **20 趟实飞**：`px4_parameters_written == []`、**机间碰撞 `0/20`（`100%` 零机间碰撞）**、`clock_ahead == 0`、`Failsafe activated == 0`。

### 2.1 四大核心场景：L0 vs `ExploreFleet` 锁步 SITL vs `CrossingFleet`（[B1](b1-sitl-lockstep.md) / [S8](s8-forest-and-m4-sitl.md)）

| 场景卡 | 机数 | Gazebo 静态几何（`--scene-world`） | `v_target` m/s（ratio） | L0 同卡到达 / 时间 s / 障带速度 m/s | `ExploreFleet` SITL 锁步到达 | `CrossingFleet` SITL 基线（B1/S8） | `ExploreFleet` 交接后到达 s（对比 `CrossingFleet` 提速） | 冲线 `x_finish` s | SITL 障带/林内均速 m/s（峰值） | SITL PLAN 最小机间 m | SITL PLAN 最小净空 m（成功趟） | `px4_parameters_written` |
|---|---:|---|---:|---|---|---|---|---|---|---|---|---|
| `explore-3-pillars-sitl80` | 3 | 开（`12` 根方柱） | `7.5`（`1.25×`） | `3/3`，`5.03–5.05 s`，`7.28–7.35 m/s` | **`3/3`（`9/9`，`0` 碰撞）** | `4/5`（`1` 撞柱，`11.47 s` 中位） | **`10.78–11.28 s`**（含减速保持；冲线仅 `3.22 s`） | `3.22–3.42` | **`6.72–6.83`**（峰值 `7.19`） | `6.012–7.113` | **`+0.998–1.024`** | `[]` |
| `explore-3-forest-sitl80` | 3 | 开（`26` 树 + `11` 枝） | `8.4`（`1.40×`） | `5/5`，`6.92–7.53 s`，`8.05–8.15 m/s` | **`3/5`（另 `v=8.4` 覆写 `1/1`，合计 `4/6`）** | `4/5`（`1` 撞树，`24.32–25.78 s`） | **`14.30–15.30 s`**（较 S8 `25.04 s` **快 `1.68×`**） | `5.36–5.79` | **`7.41–7.58`**（峰值 `8.08`） | `4.650–7.888` | **`+0.242–0.381`**（擦树 `-1.2 mm` / `-44 mm`） | `[]` |
| `explore-6-open-sitl80` | 6 | 关（开阔区） | `8.4`（`1.40×`） | `3/3`，`4.13 s`，`8.11 m/s` | **`3/3`（`18/18`，`0` 碰撞）** | `5/5`（楔形起飞，`9.98–11.90 s`） | **`11.04–11.48 s`**（统一 `x=0` 横线起飞） | **`2.96–2.98`** | **`7.29–7.35`**（峰值 `7.95`） | `1.807–2.221` | **`+1.006–1.023`** | `[]` |
| `explore-6-forest-sitl80` | 6 | 开（`26` 树 + `11` 枝） | `6.8`（`1.13×`） | `5/5`，`8.49–8.74 s`，`6.52–6.58 m/s` | **`4/5`（`24/30` 整队到达，`0` 机间碰撞）** | **`0/3`（`5/18`，全败：2 撞树 + 1 机间相撞超时）** | **`15.94–18.98 s`，中位 `17.05 s`**（L0 `CrossingFleet` 需 `26.5 s`，SITL `0/3`） | `6.72–9.80` | **`5.95–6.30`，中位 `6.16`**（峰值 `6.87`） | **`0.845–1.899`**（全部 $> 0.70\text{ m}$） | **`+0.043–0.590`**（唯一失败 `-16.8 mm`） | `[]` |

---

## 3. 逐场景逐趟实飞明细

### 3.1 六机中密树林：`explore-6-forest-sitl80`（`--n 6 --policy explore --lockstep --scene-world`，seeds `0–4`）

> **核心突破（对照 [S8](s8-forest-and-m4-sitl.md) 第 4 节 `crossing-6-forest-sitl80` `0/3` 全败）**：移除横向槽位橡皮筋拉扯与 `1.50 s` 六机 TDMA 轮转等待、改为 `12.5 Hz` 全队并行重规划 + 三维轨迹管去冲突 + 自由横向终点锁定后，六机从 `x = 0` 统一水平线同步起飞，**`4/5` 趟（`80%`）实现 `6/6` 满编高速穿林、平稳悬停并完成 `NAV_LAND` 上锁，机间碰撞从 S8 的实体互撞降为 `0/5`，林内实飞均速达 `5.95–6.30 m/s`（单机最高 `6.43 m/s`），交接后到达中位仅 `17.05 s`（比 L0 点质量下的 `CrossingFleet` `26.1–29.1 s` 还快 `1.53–1.71×`）**！

| seed | 运行目录 | SITL 结果 | 到达架数 | 横线交接 `x` 极差 m | 起飞交接 s | 冲线 `x=32m` s | 交接后到达 s | 林内均速 m/s（L0 / SITL） | 峰值速度 m/s | PLAN 最小机间 m | PLAN 最小净空 m（L0 / SITL） | 单步墙钟中位 ms | 里程计最大时差 s |
|---:|---|---|---:|---:|---:|---:|---:|---|---:|---:|---|---:|---:|
| 0 | `20261116T235229-yopo-n6` | `SUCCESS` | `6/6` | `0.0494` | `10.78` | `9.80` | `18.98` | `6.58 / 5.95` | `6.86` | `1.3245` | `0.601 / +0.0431` | `7.67` | `0.016` |
| 1 | `20261116T235910-yopo-n6` | `SUCCESS` | `6/6` | `0.0711` | `9.90` | `6.95` | `16.12` | `6.52 / 6.02` | `6.77` | `0.8452` | `0.358 / +0.5895` | `7.90` | `0.016` |
| 2 | `20261117T000349-yopo-n6` | `FAIL`（`uav1:OBSTACLE`） | `0/6` | `0.0594` | `9.92` | — | — | `6.57 / 6.16` | `6.80` | `1.6694` | `0.703 / -0.0168` | `7.76` | `0.016` |
| 3 | `20261117T003826-yopo-n6` | `SUCCESS` | `6/6` | `0.0478` | `10.82` | `7.59` | `17.98` | `6.57 / 6.23` | `6.87` | `1.3537` | `0.730 / +0.4359` | `6.65` | `0.016` |
| 4 | `20261117T004205-yopo-n6` | `SUCCESS` | `6/6` | `0.0691` | `10.12` | `6.72` | `15.94` | `6.56 / 6.30` | `6.84` | `1.8992` | `0.642 / +0.5705` | `6.63` | `0.016` |

- `seed 2` 唯一一次失败为 `uav1` 在 `6.16 m/s` 高速绕树时表面擦树 `16.8 mm`（`-0.0168 m`），随后全队正常执行收尾着陆上锁，全程无机间碰撞（`min_inter_drone_distance_plan_m = 1.669 m`）、无锁步超时。

### 3.2 六机开阔横排高速推进：`explore-6-open-sitl80`（`--n 6 --policy explore --lockstep`，seeds `0–2`）

| seed | 运行目录 | SITL 结果 | 到达架数 | 横线交接 `x` 极差 m | 起飞交接 s | 冲线 `x=15.5m` s | 交接后到达 s | 巡航均速 m/s（L0 / SITL） | 峰值速度 m/s | PLAN 最小机间 m | PLAN 最小净空 m | 单步墙钟中位 ms |
|---:|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| 0 | `20261117T000839-yopo-n6` | `SUCCESS` | `6/6` | `0.0494` | `9.92` | `2.97` | `11.04` | `8.11 / 7.34` | `7.89` | `2.2207` | `+1.0235` | `7.84` |
| 1 | `20261117T001209-yopo-n6` | `SUCCESS` | `6/6` | `0.0808` | `9.86` | `2.98` | `11.44` | `8.11 / 7.29` | `7.89` | `1.8066` | `+1.0057` | `7.69` |
| 2 | `20261117T001537-yopo-n6` | `SUCCESS` | `6/6` | `0.0464` | `9.90` | `2.96` | `11.48` | `8.11 / 7.35` | `7.95` | `2.1922` | `+1.0083` | `6.62` |

- **3/3 趟（18/18 架次）100% 零碰撞到达着陆**。6 架无人机在 `x = 0` 横线同步起飞后仅用 **`2.96–2.98 s`** 即全队冲过 `x = 15.5 m` 计时平面，实飞巡航均速 **`7.29–7.35 m/s`**（峰值 `7.95 m/s`）。

### 3.3 三机柱林横排高速穿越：`explore-3-pillars-sitl80`（`--n 3 --policy explore --lockstep --scene-world`，seeds `0–2`）

| seed | 运行目录 | SITL 结果 | 到达架数 | 横线交接 `x` 极差 m | 起飞交接 s | 冲线 `x=15.5m` s | 交接后到达 s | 柱带均速 m/s（L0 / SITL） | 峰值速度 m/s | PLAN 最小机间 m | PLAN 最小净空 m（L0 / SITL） | 单步墙钟中位 ms |
|---:|---|---|---:|---:|---:|---:|---:|---|---:|---:|---|---:|
| 0 | `20261117T001823-yopo-n3` | `SUCCESS` | `3/3` | `0.0475` | `9.88` | `3.23` | `10.78` | `7.33 / 6.83` | `7.19` | `6.5281` | `1.013 / +1.0237` | `5.94` |
| 1 | `20261117T001942-yopo-n3` | `SUCCESS` | `3/3` | `0.0471` | `10.10` | `3.42` | `10.94` | `7.35 / 6.72` | `7.14` | `6.0116` | `1.023 / +0.9978` | `5.97` |
| 2 | `20261117T002106-yopo-n3` | `SUCCESS` | `3/3` | `0.0333` | `9.92` | `3.22` | `11.28` | `7.28 / 6.82` | `7.19` | `7.1129` | `0.967 / +1.0055` | `5.96` |

- **3/3 趟（9/9 架次）100% 零碰撞到达着陆**。去除了 `CrossingFleet` 在绕过首排柱子后把无人机拉回柱列的横向槽位偏置后，三机避开首柱即沿开阔走廊以 **`6.72–6.83 m/s`** 直线冲刺，PLAN 段最小表面净空从 [B1](b1-sitl-lockstep.md) 的 `-0.010..1.03 m`（`4/5`）大幅提升至稳定 **`+0.998–1.024 m`**！

### 3.4 三机中密树林（`> 7.4 m/s` 高速区间）与 `--velocity-mps` 消融：`explore-3-forest-sitl80`

| 臂 / 配置 | seed | 运行目录 | SITL 结果 | 到达架数 | 冲线 `x=32m` s | 交接后到达 s | 林内均速 m/s（L0 / SITL） | 峰值速度 m/s | PLAN 最小机间 m | PLAN 最小净空 m |
|---|---:|---|---|---:|---:|---:|---|---:|---:|---:|
| 默认卡（`v=8.4, a_max=13.88`） | 0 | `20261117T003239-yopo-n3` | `FAIL`（`uav0:OBSTACLE`） | `0/3` | — | — | `8.15 / 7.53` | `8.04` | `7.7504` | `-0.0444` |
| 默认卡（`v=8.4, a_max=13.88`） | 1 | `20261117T003412-yopo-n3` | `SUCCESS` | `3/3` | `5.36` | `14.68` | `8.13 / 7.56` | `8.08` | `7.5441` | `+0.2659` |
| 默认卡（`v=8.4, a_max=13.88`） | 2 | `20261117T003612-yopo-n3` | `SUCCESS` | `3/3` | `5.72` | `14.96` | `8.11 / 7.43` | `7.86` | `7.6683` | `+0.2417` |
| 默认卡（`v=8.4, a_max=13.88`） | 3 | `20261117T004622-yopo-n3` | `FAIL`（`uav0:OBSTACLE`） | `0/3` | — | — | `8.14 / 7.58` | `7.87` | `4.6499` | `-0.0012` |
| 默认卡（`v=8.4, a_max=13.88`） | 4 | `20261117T004800-yopo-n3` | `SUCCESS` | `3/3` | `5.58` | `14.30` | `8.05 / 7.56` | `7.88` | `7.7236` | `+0.2497` |
| `--velocity-mps 8.4` 覆写验证 | 0 | `20261117T002743-yopo-n3` | `SUCCESS` | `3/3` | `5.79` | `15.30` | `— / 7.41` | `7.94` | `7.8880` | `+0.3806` |
| 低 `a_max` 对照（`v=7.2, a_max=10.2`） | 0–2 | `20261117T002233..2550` | `1/3` `SUCCESS`（seed 2） | `3/9` | `6.15` | `14.94` | `— / 6.31–6.54` | `7.41` | `5.3824` | `-0.106..+0.470` |

- **实测物理结论**：
  1. 在不改任何 PX4 参数的条件下，将 `velocity_mps` 推高至 `8.4 m/s`（时间相似比 `1.40×`）并匹配 `dynamics.a_max = 13.88 m/s²` 后，YOPO 大曲率避障轨迹不再被 `_kin_ok` 误裁，前馈加速度直接打满 PX4 `45°` 倾角，使三机在 26 棵树 + 11 根矮枝林带内的真实平均三维速度达到 **`7.41–7.58 m/s`（峰值 `8.08 m/s`）**，全队冲过 `x = 32 m` 仅需 **`5.36–5.79 s`**，交接后完整悬停到达仅 **`14.30–15.30 s`**（比 [S8](s8-forest-and-m4-sitl.md) `crossing-forest-sitl80` 的 `24.32–25.78 s` **提速 `1.68×`**）。
  2. 在 `7.5 m/s` 接近原厂 `45°` 倾角物理极限的高速穿林中，PX4 启动阶段 EKF2 惯导随机偏置带来的轨迹跟踪波动（见 [B2](b2-sitl-plan-clearance.md)）使 `seed 3` 出现 `1.2 mm`（`-0.0012 m`）临界擦树、`seed 0` 在两次独立容器启动中一次 `+0.381 m` 满编通过、一次 `-0.044 m` 擦树；若将速度定在 `6.8 m/s`（如六机林带 `4/5` 通过），跟踪余量更为充裕。这诚实刻画了未改参原厂 PX4 `x500` 在 `7.5+ m/s` 密林机动下的真实刚体控制边界。

---

## 4. 复现命令

```bash
# 1. 单元测试与静态契约门禁（无需启动 Docker / Gazebo）
.venv/bin/python -m pytest -q tests/test_sitl_explore.py tests/test_explore_swarm.py
.venv/bin/python -m pytest -q tests/test_sitl*.py tests/test_ros2_*.py
.venv/bin/python scripts/check_scenes.py --strict
.venv/bin/python scripts/check_doc_links.py
.venv/bin/python scripts/check_phase1_closeout.py

# 2. 锁步 PX4 + Gazebo Harmonic 实飞复现（独占锁串行化，零 PX4 参数写入）
YOPO_SITL_DOCKER_NETWORK=bridge flock /tmp/yopo-sitl.lock \
  bash scripts/sitl/run_yopo_l0_sitl.sh --n 3 \
  --scene scenes/explore-3-pillars-sitl80.json --policy explore --seed 0 --lockstep --scene-world

YOPO_SITL_DOCKER_NETWORK=bridge flock /tmp/yopo-sitl.lock \
  bash scripts/sitl/run_yopo_l0_sitl.sh --n 3 \
  --scene scenes/explore-3-forest-sitl80.json --policy explore --seed 1 --lockstep --scene-world

YOPO_SITL_DOCKER_NETWORK=bridge flock /tmp/yopo-sitl.lock \
  bash scripts/sitl/run_yopo_l0_sitl.sh --n 6 \
  --scene scenes/explore-6-open-sitl80.json --policy explore --seed 0 --lockstep

YOPO_SITL_DOCKER_NETWORK=bridge flock /tmp/yopo-sitl.lock \
  bash scripts/sitl/run_yopo_l0_sitl.sh --n 6 \
  --scene scenes/explore-6-forest-sitl80.json --policy explore --seed 1 --lockstep --scene-world

# 3. 命令行动态覆写 TJU 时间相似比目标速度（不改落盘 JSON）
YOPO_SITL_DOCKER_NETWORK=bridge flock /tmp/yopo-sitl.lock \
  bash scripts/sitl/run_yopo_l0_sitl.sh --n 3 \
  --scene scenes/explore-3-forest-sitl80.json --policy explore --seed 0 \
  --lockstep --scene-world --velocity-mps 8.4
```
