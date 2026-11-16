# 票 S8：三机林带多圆柱世界与六机 M4 锁步 PX4 SITL 抽检（report-only，判据未冻结）

本票补齐 [LIMITATIONS](../LIMITATIONS.md) 与 [ADR 0016](../adr/0016-m0-m4-freeze-recommendation.md) 里剩下的两块 PX4 SITL 锁步抽检空白：

1. **M2 三机林带（`crossing-forest`）**：支持把场景卡里的多圆柱树干（含倾斜树干 `lean_xy`）与矮枝盒子注入 Gazebo Harmonic 世界 SDF（`--scene-world` / `YOPO_SITL_SCENE_WORLD=1`，默认关），跑通三机锁步穿林抽检。
2. **M4 六机编队与穿行（`--n 6`）**：把 `scripts/sitl/`（`fleet.py`、`up.sh`、`smoke.sh`、`in_container.sh`、`run_yopo_l0_sitl.sh`、`yopo_l0_live.py`）和 `yopo_l0/sitl_bridge.py`、`yopo_l0/sitl_lockstep.py` 扩展到 `--n 6`（保留 `1..3` 行为不变，拒绝未定义的 `4`、`5` 等机数），跑通六机开阔编队（`formation-6-open-sitl80`）、六机开阔穿行（`crossing-6-open-sitl80`）及六机林带（`crossing-6-forest-sitl80`）锁步抽检。

所有结果都是 **只报数（report-only，判据未冻结）**，不改任何飞控安全参数（全部 18 趟实飞 `px4_parameters_written == []`），不改 `core/`，也不把 SITL 对照卡写进 `scenes/suite.txt`。

## 构造与边界

### 1. 六机机群布局与锁步握手（`--n 6`）

沿用 [S2](../../scripts/sitl/README.md) 与 PX4 v1.17 posix `rcS` 的多实例约定：

- `ALLOWED_N = (1, 2, 3, 6)`：单机仍是 `instance=0`、空 namespace、`MAV_SYS_ID=1`、`x500_0`、GCS UDP `18570`。
- 六机从 `instance=1..6` 起：`namespace = px4_1..px4_6`，`MAV_SYS_ID = 2..7`，Gazebo 模型 `x500_1..x500_6`，GCS UDP `18571..18576`，共用同一个 `MicroXRCEAgent`（UDP `8888`）和 `ROS_DOMAIN_ID=0`。`instance=1` 拉起 Gazebo 服务端，`instance=2..6` 以 `PX4_GZ_STANDALONE=1` 接入同一世界。
- 实飞入口按场景卡 `drones[*].start_enu` 覆写 `PX4_GZ_MODEL_POSE`（`fleet.py yopo-poses --n 6`），`LiveFleetSession` 用各机起点水平偏置把本地 NED `VehicleOdometry` 抬回统一 ENU 世界系。
- `LockstepStepper` 对全部 6 个电机转速话题 `/x500_1/command/motor_speed` .. `/x500_6/command/motor_speed` 做逐物理步（`0.004 s`）屏障握手；容器对 `--n 6` 分配 `--shm-size 4g`，里程计等待上限从 `50 ms` 放宽到 `100 ms`，实测 13 趟六机运行 `odom_wait_timeouts` 全为 `0`。

### 2. 林带多圆柱与矮枝 SDF 世界生成（`--scene-world`）

`yopo_l0/sitl_bridge.py` 新增纯 Python 的 `cylinder_sdf_geometry`、`box_sdf_geometry`、`build_scene_world_sdf`、`write_scene_world_sdf`，由 `scripts/sitl/world_sdf.py` 暴露命令行：

- 对 `SceneCard.obstacles` 中的每根 `Cylinder`（含 `yopo_l0.forest` 按种子生成的竖直树干与带 `lean_xy` 的倾斜树干），计算端面中点 $M = \frac{1}{2}(A+B)$、轴长 $L = \|B-A\|$ 与外旋欧拉角 $(0, \theta, \psi)$，使 SDF `<cylinder>` 的上下端面中心与 `Cylinder.axis_ends()` 精确重合（误差 $< 10^{-12}\text{ m}$）。
- 同时导出非走廊外框的静态矮枝 `Box`（`t*-br`），跳过 `ceiling` 与四面外框墙，保留 Gazebo 顶光与地面模型。
- 容器启动时若 `YOPO_SITL_SCENE_WORLD=1`（`--scene-world`，默认 `0`），以 `${PX4_GZ_WORLDS}/default.sdf` 为基底写入 `/out/worlds/default.sdf` 与 `/out/world_sdf.json`，保持 `<world name="default">` 不变，使 `/world/default/clock`、`/world/default/control` 与 PX4 `gz_bridge` 无需改话题即可加载完整林带物理几何。
- `crossing-forest`（`medium`，`seed=7`）共生成 **26 根静态圆柱树干（其中 3 根倾斜）+ 11 个静态矮枝盒子**，合成世界 SDF 哈希前缀为 `82b3f3cbc249`。

### 3. 新增只报数 SITL 对照场景卡

均不进 `scenes/suite.txt`，通过 `scripts/check_scenes.py --strict` 与 `scripts/check_phase1_closeout.py`：

- [crossing-forest-sitl80.json](../../scenes/crossing-forest-sitl80.json)：与 `scenes/crossing-forest.json` 只差 `name`、`task.duration_s`（`50 → 80`）和 `note`。
- [formation-6-open-sitl80.json](../../scenes/formation-6-open-sitl80.json)：与 `scenes/formation-6-open.json` 只差 `name`、`task.duration_s`（`40 → 80`）和 `note`。
- [crossing-6-open-sitl80.json](../../scenes/crossing-6-open-sitl80.json)：六机楔形槽位 + `open-area.json`（`length_m=19.5`，`duration_s=80`，`criteria.frozen=false`），用于隔离检验六机 `CrossingFleet` + 6 路 YOPO 网络 + 深度渲染在锁步 SITL 下的闭环跟踪。
- [crossing-6-forest-sitl80.json](../../scenes/crossing-6-forest-sitl80.json)：与 `scenes/crossing-6-forest.json` 只差 `name`、`task.duration_s`（`50 → 80`）和 `note`。

## 实测环境

- 宿主环境：WSL2 `RflySim-20.04`（Ubuntu 22.04.5 LTS，Intel Core Ultra 7 265KF，Docker Engine `28.5.1`）。
- 镜像：`yopo-restart-sitl:s1b-23bc262f44d8`（PX4 v1.17.0 + Gazebo Harmonic + ROS 2 Humble）与分层镜像 `yopo-restart-sitl:yopo-81d084c04186`（参见 [S6](s6-sitl-upstream-m0.md)、[S7](s7-sitl-crossing-switch.md)、[B1](b1-sitl-lockstep.md)）。
- 串行化：每趟容器运行均以 `flock /tmp/yopo-sitl.lock` 独占执行，单容器独占网络，无跨容器 DDS / `gz-transport` 串话。产物目录在 `/workspace/yopo-ops/sitl-s8/runs/`（索引 `/workspace/yopo-ops/sitl-s8/s8-index.txt`，汇总 `/workspace/yopo-ops/sitl-s8/s8-collected.json`）；L0 对照产物在 `/workspace/yopo-ops/sitl/s8-l0/`。

## 汇总对照表

| 场景卡 | 机数 | 策略 | Gazebo 林带世界 | seeds | L0 到达 | SITL 锁步到达 | L0 碰撞 | SITL 碰撞 | L0 到达时间 s | SITL 交接后到达 s | L0 队形 p95 m | SITL 队形 p95 m | L0 最小机间 m | SITL PLAN 最小机间 m | L0 最小净空 m | SITL PLAN 最小净空 m | SITL RTF 中位 | `px4_parameters_written` |
|---|---:|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---|---|---:|---|
| `crossing-forest-sitl80` | 3 | `crossing` | 开（26 树 + 11 枝） | 0–4 | 5/5（15/15） | **4/5（12/15）** | 0 | 1 | 20.77–23.63 | 24.32–25.78（4 趟） | 4.112–4.466 | 4.146–4.559（4 趟） | 6.622–7.388 | 6.347–6.815（4 趟；含失败 3.833） | 0.260–0.445 | 0.217–0.718（4 趟；失败 −0.020） | 0.437 | `[]` |
| `formation-6-open-sitl80` | 6 | `formation` | 关（开阔区） | 0–4 | 5/5（30/30） | **5/5（30/30）** | 0 | **0** | 20.13 | 20.98–23.32（中位 21.66） | 0.066 | 0.257–0.572（中位 0.271） | 2.980 | 2.582–2.906 | 1.150（离地） | 0.853–1.031（离地） | 0.463 | `[]` |
| `crossing-6-open-sitl80` | 6 | `crossing` | 关（开阔区） | 0–4 | 5/5（30/30） | **5/5（30/30）** | 0 | **0** | 7.89 | 9.98–11.90（中位 10.72） | 5.559 | 5.586–5.703（中位 5.633） | 2.835–2.838 | 2.326–2.823 | 1.117（离地） | 0.979–1.022（离地） | 0.421 | `[]` |
| `crossing-6-forest-sitl80` | 6 | `crossing` | 开（26 树 + 11 枝） | 0–2 | 3/3（18/18） | **0/3（5/18）** | 0 | 3 | 26.14–26.90 | 无整队到达 | 23.942–24.592 | 12.134–23.826 | 1.016–1.392 | 0.662–1.694 | 0.202–0.315 | −0.012–0.002 | 0.301 | `[]` |

*注：L0 净空 = `min_obstacle_distance_m - 0.35`；SITL 净空列取 `min_surface_clearance_plan_m`（只计 PLAN 段，不含起飞离地与降落触地，口径与 [S7](s7-sitl-crossing-switch.md) 相同）。*

## 逐趟结果明细

### 1. M2 三机林带：`crossing-forest-sitl80`（`--n 3 --policy crossing --lockstep --scene-world`，seeds 0–4）

| seed | 运行目录 | L0 到达 s | SITL 结果 | 到达架数 | 起飞交接 s | 交接后到达 s | 队形 p95 m（L0 / SITL） | PLAN 最小机间 m | PLAN 最小净空 m（L0 / SITL） | RTF | 单步墙钟中位 ms | 里程计仿真时差最大 s | `scene_world` |
|---:|---|---:|---|---:|---:|---:|---|---:|---|---:|---:|---:|---|
| 0 | `20261116T082654-yopo-n3` | 23.32 | `SUCCESS` | 3/3 | 9.58 | 24.32 | 4.466 / 4.209 | 6.8154 | 0.3162 / 0.5201 | 0.4369 | 6.12 | 0.012 | `26` 柱（`3` 斜）+ `11` 枝 |
| 1 | `20261116T083010-yopo-n3` | 23.59 | `SUCCESS` | 3/3 | 9.82 | 25.04 | 4.129 / 4.146 | 6.3473 | 0.2646 / 0.3356 | 0.4282 | 6.10 | 0.012 | `26` 柱（`3` 斜）+ `11` 枝 |
| 2 | `20261116T083348-yopo-n3` | 20.77 | `SUCCESS` | 3/3 | 9.80 | 25.78 | 4.112 / 4.559 | 6.5013 | 0.2597 / 0.7182 | 0.4496 | 6.03 | 0.012 | `26` 柱（`3` 斜）+ `11` 枝 |
| 3 | `20261116T083847-yopo-n3` | 23.63 | `FAIL`（`uav1:OBSTACLE`） | 0/3 | 10.12 | — | 4.236 / 4.351 | 3.8325 | 0.4449 / −0.0196 | 0.4587 | 6.06 | 0.012 | `26` 柱（`3` 斜）+ `11` 枝 |
| 4 | `20261116T084156-yopo-n3` | 23.37 | `SUCCESS` | 3/3 | 9.98 | 25.60 | 4.311 / 4.377 | 6.7210 | 0.3037 / 0.2168 | 0.4230 | 6.10 | 0.012 | `26` 柱（`3` 斜）+ `11` 枝 |

- **结论**：5 趟全部 `clock_ahead == 0`、`handshake_timeouts == 0`、`sensor_timeouts == 0`，三机里程计仿真时差最大 `0.012 s`。**4/5 趟满格到达并平稳降落上锁**，成功趟交接后到达 `24.32–25.78 s`（比 L0 慢 `1.0–5.0 s`），队形误差 p95 `4.146–4.559 m` 与 L0（`4.112–4.466 m`）同量级。
- `seed 3` 的失败是 `uav1` 在林带中段擦树 `1.96 cm`（`min_surface_clearance_plan_m = -0.0196 m`），与 [B1](b1-sitl-lockstep.md)（柱林锁步 `4/5`）和 [B2](b2-sitl-plan-clearance.md) 归因的「上游规划从已提交轨迹取起点 + PX4 位置环横向跟踪滞后」同一机理（三机林带在 L0 自身最紧净空也只有 `0.260 m`）。
- 注入 26 根圆柱树干与 11 个矮枝后，单物理步墙钟中位 `6.03–6.12 ms`，与 [B1](b1-sitl-lockstep.md) 空世界三机锁步的 `6.1–6.9 ms` 持平，说明静态 SDF 几何不会拖慢 Gazebo 步进。

### 2. M4 六机开阔编队：`formation-6-open-sitl80`（`--n 6 --policy formation --lockstep`，seeds 0–4）

| seed | 运行目录 | SITL 结果 | 到达架数 | 起飞交接 s | 交接后到达 s | 队形 p95 m（L0 / SITL） | PLAN 最小机间 m（L0 / SITL） | PLAN 最小净空 m | RTF | 单步墙钟中位 ms | 六机里程计仿真时差最大 s |
|---:|---|---|---:|---:|---:|---|---|---:|---:|---:|---:|
| 0 | `20261116T081709-yopo-n6` | `SUCCESS` | 6/6 | 9.16 | 21.52 | 0.0660 / 0.2706 | 2.9798 / 2.9059 | 1.0309 | 0.4045 | 7.10 | 0.020 |
| 1 | `20261116T084540-yopo-n6` | `SUCCESS` | 6/6 | 9.98 | 21.96 | 0.0660 / 0.4683 | 2.9798 / 2.8886 | 0.8535 | 0.4631 | 6.68 | 0.016 |
| 2 | `20261116T084907-yopo-n6` | `SUCCESS` | 6/6 | 10.98 | 23.32 | 0.0660 / 0.5723 | 2.9798 / 2.5815 | 0.9444 | 0.4784 | 6.68 | 0.012 |
| 3 | `20261116T085233-yopo-n6` | `SUCCESS` | 6/6 | 10.88 | 21.66 | 0.0660 / 0.2580 | 2.9798 / 2.8876 | 0.9400 | 0.4479 | 6.78 | 0.012 |
| 4 | `20261116T085621-yopo-n6` | `SUCCESS` | 6/6 | 9.98 | 20.98 | 0.0660 / 0.2567 | 2.9798 / 2.8019 | 1.0206 | 0.4814 | 6.62 | 0.016 |

- **结论**：**5/5 趟（30/30 架次）全部到达并完成 `NAV_LAND` 着陆上锁，碰撞 0**。交接后到达中位 `21.66 s`（L0 `20.13 s`），PLAN 段最小机间 `2.582–2.906 m`（全部高于 L0 冻结参考值 `2.50 m`），队形误差 p95 中位 `0.271 m`（与 [S5b](s5b-sitl-formation-verify.md) 三机编队保持 SITL 的 `0.310–0.361 m` 一致，反映 PX4 位置环与 GNSS 估计噪声）。六机单物理步墙钟中位仅 `6.62–7.10 ms`，六机里程计仿真时差最大 `0.012–0.020 s`。

### 3. M4 六机开阔穿行：`crossing-6-open-sitl80`（`--n 6 --policy crossing --lockstep`，seeds 0–4）

| seed | 运行目录 | SITL 结果 | 到达架数 | 起飞交接 s | 交接后到达 s | 队形 p95 m（L0 / SITL） | PLAN 最小机间 m（L0 / SITL） | PLAN 最小净空 m | RTF | 单步墙钟中位 ms | 六机里程计仿真时差最大 s |
|---:|---|---|---:|---:|---:|---|---|---:|---:|---:|---:|
| 0 | `20261116T085947-yopo-n6` | `SUCCESS` | 6/6 | 9.98 | 10.72 | 5.5592 / 5.5859 | 2.8351 / 2.3258 | 1.0196 | 0.4163 | 6.65 | 0.016 |
| 1 | `20261116T090446-yopo-n6` | `SUCCESS` | 6/6 | 10.80 | 11.90 | 5.5592 / 5.5970 | 2.8362 / 2.7742 | 0.9791 | 0.4205 | 6.66 | 0.016 |
| 2 | `20261116T090804-yopo-n6` | `SUCCESS` | 6/6 | 9.94 | 11.34 | 5.5592 / 5.6325 | 2.8346 / 2.3266 | 1.0138 | 0.4239 | 6.58 | 0.016 |
| 3 | `20261116T091111-yopo-n6` | `SUCCESS` | 6/6 | 10.06 | 9.98 | 5.5592 / 5.7026 | 2.8359 / 2.8230 | 0.9959 | 0.4211 | 6.56 | 0.012 |
| 4 | `20261116T091437-yopo-n6` | `SUCCESS` | 6/6 | 10.04 | 10.12 | 5.5592 / 5.6590 | 2.8375 / 2.8033 | 1.0221 | 0.4245 | 6.60 | 0.016 |

- **结论**：6 架飞机同时在环跑 YOPO 网络前向 + 深度相机渲染 + `CrossingFleet` 协议交换（`commit_horizon_s` 自动抬到 `1.75 s`），**5/5 趟（30/30 架次）全部到达并着陆上锁，碰撞 0，`sensor_timeouts == 0`**。虽然每拍代理计算负载翻倍，锁步机制把仿真时钟严格停在每 `0.02 s` 控制拍上，六机里程计仿真时差最大 `0.012–0.016 s`，交接后到达 `9.98–11.90 s`（L0 `7.89 s`）。

### 4. M4 六机林带探索抽检：`crossing-6-forest-sitl80`（`--n 6 --policy crossing --lockstep --scene-world`，seeds 0–2）

| seed | 运行目录 | SITL 结果与原因 | 到达架数 | 碰撞原因 | PLAN 最小机间 m | PLAN 最小净空 m（L0 / SITL） | RTF | 单步墙钟中位 ms |
|---:|---|---|---:|---|---:|---|---:|---:|
| 0 | `20261116T091623-yopo-n6` | `FAIL`（`uav0:OBSTACLE`） | 0/6 | `uav0:OBSTACLE` | 1.6944 | 0.3148 / −0.0051 | 0.3010 | 6.66 |
| 1 | `20261116T091834-yopo-n6` | `FAIL`（`uav4:OBSTACLE`） | 0/6 | `uav4:OBSTACLE` | 1.4169 | 0.2303 / −0.0119 | 0.3502 | 6.64 |
| 2 | `20261116T092034-yopo-n6` | `FAIL`（`LOCKSTEP_TIMEOUT`） | 5/6 | `uav2-uav4:INTER_DRONE` | 0.6622 | 0.2023 / 0.0020 | 0.2692 | 6.58 |

- **真实物理差距记录**：
  - 六机林带在 L0 点质量下最紧净空本来就只有 `0.126 m`（`seeds 0–2` 为 `0.202–0.315 m`，提交前净空门 margin 默认 `0.15 m`，见 [LIMITATIONS](../LIMITATIONS.md)）。到了 PX4 SITL，位置环横向跟踪误差 p95 约 `0.40–0.66 m`（见 [B2](b2-sitl-plan-clearance.md)），吃穿了这层余量：`seed 0` 与 `seed 1` 分别擦树 `5.1 mm` 与 `11.9 mm` 后正常触发收尾降落。
  - `seed 2` 有 `5/6` 架到达终点，穿林段最小表面净空保持正值（`+2.0 mm`），但 `uav2` 与 `uav4` 机间距离缩到 `0.6622 m < 0.70 m`（两倍机体半径），在 Gazebo 里发生实体接触后物理服务端停止响应，`LockstepStepper` 按设计失败关闭并记下 `LOCKSTEP_TIMEOUT`。这验证了锁步与碰撞记录不会掩盖六机密集林带的跟踪余量缺口。

### 5. 飞控参数与电池告警核对

- 全部 18 趟运行目录的 `result.json` 中 `px4_parameters_written` 均为 `[]`；容器导出的 `px4-1-params-changed.txt` .. `px4-6-params-changed.txt` 仅含 `sitl-battery.env` 既有的 SITL 模拟器组字段，未写入任何安全或控制参数。
- 18 趟合计仅 `f6-open seed=0`（3 架次）出现过 [BAT1](bat1-lockstep-battery.md) 已归因的解锁后 `Battery unhealthy` 控制台行，全部 18 趟 `Failsafe activated` 行数均为 `0`。

## 复现命令

```bash
# 1. 单元测试与场景卡静态契约检查（无需 Gazebo / Docker）
.venv/bin/python -m pytest -q tests/test_sitl_s8_forest_and_m4.py
.venv/bin/python scripts/check_scenes.py --strict
.venv/bin/python scripts/check_doc_links.py

# 2. 离线生成林带多圆柱 + 矮枝 Gazebo 世界 SDF
PYTHONPATH=".:core/src" .venv/bin/python scripts/sitl/world_sdf.py \
  --scene scenes/crossing-forest-sitl80.json \
  --out-dir /tmp/s8-worlds \
  --summary /tmp/s8-worlds/world_sdf.json

# 3. 锁步 PX4 SITL 抽检（共享机以 flock 串行化）
flock /tmp/yopo-sitl.lock bash scripts/sitl/run_yopo_l0_sitl.sh \
  --n 3 --scene scenes/crossing-forest-sitl80.json --policy crossing --seed 0 --lockstep --scene-world

flock /tmp/yopo-sitl.lock bash scripts/sitl/run_yopo_l0_sitl.sh \
  --n 6 --scene scenes/formation-6-open-sitl80.json --policy formation --seed 0 --lockstep

flock /tmp/yopo-sitl.lock bash scripts/sitl/run_yopo_l0_sitl.sh \
  --n 6 --scene scenes/crossing-6-open-sitl80.json --policy crossing --seed 0 --lockstep

flock /tmp/yopo-sitl.lock bash scripts/sitl/run_yopo_l0_sitl.sh \
  --n 6 --scene scenes/crossing-6-forest-sitl80.json --policy crossing --seed 0 --lockstep --scene-world
```
