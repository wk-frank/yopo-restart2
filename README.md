# yopo-restart

yopo 编队项目重启。规格见 [docs/SPEC.md](docs/SPEC.md)，票见 [docs/tickets.md](docs/tickets.md)。

一条命令复现单机、三机、六机 demo（L0 锁步仿真，不是 PX4）：

```bash
make venv           # 第一次需要
scripts/demo.sh     # 产物在 out/demo/
```

干净检出后按上面做即可复现。各跑 seed 0：单机 `g2-pillar`、三机 `formation-hold`、六机 `formation-6-open`。

## 交付包（M5）

- [一键 demo](scripts/demo.sh)：干净检出后先 `make venv`，再跑这条命令，一次复现单机、三机、六机（产物在 `out/demo/`）。
- [结果总表](docs/results/SUMMARY.md)：M0–M4 各场景的到达、碰撞和冻结状态，每行带层级和 commit。
- [视频索引](docs/results/videos.md)：每个里程碑一段不超过 60 s 的俯视录像，都是 seed 0、层级 L0。
- [已知限制](docs/LIMITATIONS.md)：说明 L0 是假动力学、所有偏离上游的构造，以及继续只报数的清单。
- [上游策略 vs 影子盾](docs/results/12e-upstream-vs-shield.md)：同一 commit 上上游策略与影子盾、certified 硬门的对照；影子盾只记录、不改动作。

## CTBR 六机大场景（report-only）

[无先验地图高密度树林验收](docs/research/depth-only-dense-forest900.md)：2.2×2.2 km 场景包含 **75,616 根树干**，为上一版 12,100 根的 6.249 倍，树径及高度范围保持不变，仅保留中心起飞区。六机从中心同向出发，避障时允许编队伸缩；本趟 seed 0 中每机 CTBR 段 Gazebo 真值水平位移 **937.02–937.40 m**，6/6 到达并降落，碰撞及真值缺失样本均为 0，15 项机器验收通过。同趟 Gazebo/Ogre2 直接记录 436.0 秒 RGB 飞行画面。

规划器和桥接层已剥离已知障碍几何，避障仅使用六路 Gazebo 原生深度和机间通信轨迹。本机姿态、里程计和任务目标仍用于控制，全图几何只供 Gazebo 建模及独立真值验收。密集树干使用带物理碰撞的分块网格，保留逐树位置与尺寸 manifest 及资产哈希。

```bash
docker build --pull=false -t yopo-restart-sitl:ctbr-opencv-4.10 -f scripts/sitl/Dockerfile.ctbr scripts/sitl
docker build -f scripts/sitl/Dockerfile.gazebo-depth -t yopo-restart-sitl:gazebo-depth .
YOPO_SITL_DOCKER_NETWORK=bridge YOPO_SITL_WALL_TIMEOUT=14400 \
YOPO_SITL_MEMORY_LIMIT=11g YOPO_LIVE_DEPTH_SOURCE=gazebo \
YOPO_SITL_GAZEBO_GPU=wsl-d3d12 YOPO_DEPTH_BACKEND=opencv \
flock /tmp/yopo-sitl.lock scripts/sitl/run_yopo_l0_sitl.sh \
  --image yopo-restart-sitl:gazebo-depth \
  --scene scenes/explore-6-depth-only-dense-forest900.json --n 6 --policy explore \
  --lockstep --scene-world --control-mode ctbr --record-dt 0.2 --seed 0
```

`gazebo-flight.mp4`、深度抽样、时间戳、场景及源码哈希写入打印出的运行目录；`scripts/render_gazebo_flight.py --run <运行目录> --out <输出目录>` 可生成位移标注版和加速预览。缺帧或帧龄超过 0.20 s 时失败关闭。D3D12 选项用于本机 WSL GPU，默认软件 EGL。

这是单场景、单种子的 SITL 验证。局部自由空间检查没有形式化安全保证，未验证完整 SLAM、全图遍历、真实无线链路或墙钟实时性。起飞与终点保持使用 PVA，巡航绕障使用 CTBR，最后 NAV_LAND；不写 PX4 参数，不是实体飞机实飞。历史 [3025 障碍原生深度结果](docs/research/gazebo-native-center900.md) 使用已知地图几何安全盾，不能充作本轮验收。[早期场景卡深度记录](docs/research/ctbr-center900.md) 和 [CTBR-1 高速性能对照](docs/research/ctbr-flatness-ticket.md) 保留，后者尚待完成。

## L0（box CPU，无界面，锁步确定性）

```bash
make venv                     # 一次（Python 3.12 + torch 2.4.1 CPU，uv）
.venv/bin/python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0   # 单次：out/g2-hover/seed0/{result.json,ticks.jsonl}
.venv/bin/python -m yopo_l0.run --scene scenes/g2-pillar.json --seed 0 --policy upstream    # 单机过 G2 柱子
.venv/bin/python -m yopo_l0.run --scene scenes/open-3drone.json --seed 0 --policy hover      # 三机开阔区，各自飞向 goal_enu
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/g2-pillar.json@certified --jobs 8  # M0 表
scripts/regress.sh            # 合 main 前回归：pytest + 批跑 + Markdown 表
```

L0 默认 `upstream` + 影子盾开（`--policy hover|certified`、`--shadow off` / `--no-shadow` 可覆盖）。影子盾只记录会否拒绝，不改批准动作。M0–M4 已冻指标见 [冻结表](docs/results/11-freeze-recommendation.md) 和 ADR 0016；失败时批跑退出非 0。其余仍只报数（碰撞数如实报）。`upstream` 的终端规则是项目的 `stop_at_goal`，半径 1.5 m 已采纳（[ADR 0009](docs/adr/0009-stop-at-goal-vs-hold-here.md)）。上游 5 m `hold_here` 到不了 G2 oracle 的 0.4 m。

决策记录见 [docs/adr/](docs/adr/README.md)。M0 样例表（commit、场景卡哈希、复现命令）见 [docs/results/m0-g2-pillar.md](docs/results/m0-g2-pillar.md)。M0–M4 冻结表见 [docs/results/11-freeze-recommendation.md](docs/results/11-freeze-recommendation.md)（ADR 0016，frank 2026-11-10 批准；M2 队形门 6.0 m）。

YOPO 推理走 vendored `core/`（yopo-swarm core@00b85bf，见 core/VENDORED.md）：钉定上游 d5e1e60 + 权重 sha256 校验，CPU 单线程，与金样一致。

接缝：场景卡（`scenes/*.json`）→ `yopo_l0.runner.run(card, RunConfig)` → 结果记录 + 逐 tick 日志。
- 世界：轴对齐盒子 + 竖直圆柱 + 地面；G2 几何原样取自 yopo-swarm `sim/px4/g2-scene.json`。
- 传感器：光线投射深度，内参由分辨率 + 视场算，量程/噪声/帧率可配，时间戳 = 仿真时钟。
- 动力学：`step(state, approved_plan, dt)`，点质量 + 一阶加速度滞后。
- 锁步：每 tick 全体传感 → 全体规划 → 进程内总线投递 → 动力学 → 碰撞真值（障碍用机体半径 0.35 m，机间用两倍半径）；墙钟不进仿真时间。
- 规划器只拿 `Observation`（深度帧、里程计、目标、时间），不拿几何、不拿他机状态。每机一个策略实例。策略在 `yopo_l0.policy.POLICIES` 注册：`hover` / `upstream` / `certified`。
- 机间消息只走 `yopo_l0.bus.MessageBus`（payload 深拷贝）。旋钮 `drop` / `duplicate` / `delay` 对齐 G3 `FaultPlan`，`reorder` 对齐 G3 explore 的乱序（插到当前队首）。默认全 0。场景卡可写 `bus`，或用 `--bus-drop` 等覆盖。本机计划对象留在私有槽，总线只送令牌；他机只收到不含对象身份的公告。

结果记录里的 `tier` 总是 `L0`：这是假动力学，不是 PX4 结果。

## 结果记录

`result.json`（墙钟 `wall_time_s` 不参与确定性比较）：

| 字段 | 开（默认，`--shadow on`） | 关（`--shadow off` 或 `--no-shadow`） |
|---|---|---|
| `shadow` | `true` | `false` |
| `shadow_reject_rate` | 全体 tick 里会拒的比例。一步都不拒是 `0.0` | `null` |
| `shadow_reject_histogram` | 全体拒绝码 → tick 次数。一 tick 多个码各计一次。不拒是 `{}` | `null` |
| `shadow_drones` | 每机一行：`id`、`shadow_reject_rate`、`shadow_reject_histogram`。顺序与场景卡一致 | `null` |
| `shadow_gates` | 四个门的 `wired` / `impl` / `note` | `null` |

关的时候拒绝率是 `null` 而不是 `0`：`0.0` 只表示盾开着且没有会拒的 tick。拒绝不改 `outcome`。碰撞仍是硬失败，不进拒绝码。批跑表的「影子盾」列读 `shadow`，「影子拒绝率」列读 `shadow_reject_rate`（关着是 —）。

`ticks.jsonl` 每行一个 tick：

| 字段 | 含义 |
|---|---|
| `cmd_p` `cmd_v` `cmd_a` | 本 tick 批准计划的位置采样（不经 `sample`，所以不推进航向）。开/关逐位相同 |
| `would_reject` | 开：`true`/`false`。关：`null` |
| `reject_codes` | 开：拒绝码列表，空列表表示查了且不拒。关：`null` |

每机一份影子记录。选择门（证据、计划契约、SafetyGate）在该机规划时刷新，并抄到它执行该选择的物理 tick 上。oracle 跟踪/落后用本 tick 实际交给动力学的批准计划（总线尚未投递时是原地保持）。码和缺口见 `yopo_l0/shadow.py`。

## ROS 2 形态（票 10）

世界一个节点，每机一个节点。世界按 L0 的 tick 往 `/clock` 发仿真时间（各节点 `use_sim_time`），锁步仍是传感 → 规划 → 总线 → 动力学 → 碰撞真值，和 `yopo_l0.runner` 同一条循环。规划在该机节点里调用已有的 `yopo_l0.policy`。命名空间是场景卡里的无人机 id：`/uav0/drone`、`/uav1/drone`……影子盾也在这条循环里记，所以 `shadow_reject_rate` 和逐 tick 的 `would_reject` 与进程内形态相同。

编队（M1，`scenes/formation-hold.json`）的机间协议消息不进进程内 `MessageBus` 队列。一次运行里 `FormationFleet.transport_override` 指向 ROS 传输：没被故障丢掉的信封按锁步走完三跳，`poll` 才看得到话题上回来的那一份。

1. `/{src}/bus_tx`：世界把信封交给源无人机节点
2. `/{dest}/bus_rx`：源节点转发给目的无人机
3. `/{dest}/bus_done`：目的节点交回世界

单线程执行器，一条消息的分片按序走完才发下一条，DDS 谁先到不进仿真结果。丢包、重复、时延、乱序仍用和进程内相同的 `BusFaults` 抽样（时延只改 `deliver_at`，不睡墙钟）。被丢掉的消息不上话题。非编队场景的他机公告仍走进程内总线。

```bash
scripts/setup_ros2.sh    # Ubuntu 24.04 → Jazzy，22.04 → Humble；ros-*-ros-base + rclpy
.venv/bin/python -m yopo_l0.ros2_run --scene scenes/open-3drone.json --seed 0 --policy hover
.venv/bin/python -m yopo_l0.ros2_run --scene scenes/g2-pillar.json --seed 0 --policy upstream
.venv/bin/python -m yopo_l0.ros2_run --scene scenes/formation-hold.json --seed 0 --policy formation
```

输出和 `yopo_l0.run` 一样：`out/<场景>/seed<N>/{result.json,ticks.jsonl}`，结果里多一个 `"form": "ros2"`。`--policy`、`--seed`、`--shadow on|off` / `--no-shadow`、`--duration`、`--out`、`--bus-*` 与进程内启动器相同。`--policy formation` 只接受 `task.type` 为 `formation` 的卡。节点仍按卡里的 `drones` 建，id 即命名空间。

## 六机开阔区 + 故障档位（票 11a，M4）

楔形六机在开阔区保持队形。进程内六档的到达、碰撞、队形误差 p95 和机间距离已冻结；ROS 2 形态和墙钟仍只报数。柱林六机（11b）不在这张票里。丢 20% 时收不齐 ACK 的补票是票 06c：窗口内重发同一份提案，不改追赶轨迹和跟踪参数。票 13d 在此之上把新轨留在 Delay Check，δ_DC = 0.05 s，已提交轨到提交前都不换。全丢仍然停在起点。穿林不打开这段等待。

```bash
.venv/bin/python -m yopo_l0.batch scenes/formation-6-open.json@formation \
  --form ros2 --tiers scenes/formation-6-tiers.json --out out/m4
```

每个档位一张表：到达率、碰撞、最小机间距离、队形误差 p95、影子拒绝率、单次墙钟。`--wall-cap` 默认 120 秒；某次超过上限时，分批方案写进该次结果和 `out/m4/batching_plan.json`，退出码不因此失败。进程内零故障（档位 none）的 seed 0 低于 60 秒，已在 `scenes/suite.txt`。档位清单：none、drop 5%、drop 20%、duplicate 10%、reorder+delay、combined。

## 六机穿林（票 11b）

同一只楔形穿过 07b 的 medium 林，sparse 用同一张规划参数再跑一遍。避障是上游 YOPO。林带外面，队形只偏置目标，和票 07 同一套。林带里面（票 13c）六机的槽位是软目标：侧向、纵向只在界内松弛，瞄准一条还能走的前方 goto；出林再收紧回刚性槽位。COMMIT 前有一道硬净空门，轨迹采样的最小距离要 ≥ 机体半径 + margin（默认 0.15 m），不够就 `CLEARANCE_SKIP`，不提交。承诺视界按 13a 盖住一轮窗口，计划 a_max 与动力学相同。票 13b 起虚拟中心按全队沿中心投影的低分位走（默认 min），不再按墙钟 × speed 开环推进；超前机只提案悬停或短前缀。开阔区编队卡写 `progress_gate=false`，时刻表不变。票 13e 起六机林带卡暂关这道门控（`progress_gate=false`，ADR 0015），同时六机林带的提案前自检只看邻机的 committed + previous 和预留，不再被已替换计划的刹车停点挡住；medium / sparse 均 60/60，见 `docs/results/13e-ghost-stops.md`。seeds 0–9。到达、碰撞和最小机间距离（≥ 0.70 m）已冻结；两种 p95 和数值净空仍只报数。虚拟中心起点在 x=6 m，好让最后一排停在后墙前面。L0 仍是 upstream，safe_radius 仍是 0.05 m，kp / kd / tau / a_max 不改。

```bash
.venv/bin/python -m yopo_l0.batch \
  scenes/crossing-6-forest.json@crossing \
  scenes/crossing-6-forest-sparse.json@crossing \
  --jobs 4 --out out/m4b
.venv/bin/python -m yopo_l0.batch scenes/formation-6-open.json@formation \
  --form inprocess --tiers scenes/formation-6-tiers.json --jobs 4 --out out/m4b-open
```

第二行是 06b 合入之后重跑开阔区六档。ROS 2 对账用 medium 的 seed 0：`--form ros2 --seeds 0`。

`rclpy` 导入不了时，`tests/test_ros2_form.py`、`tests/test_ros2_formation.py` 和 `tests/test_ros2_six.py` 里的 ROS 用例跳过，原因里写明要跑 `scripts/setup_ros2.sh`。进程内回归不依赖 ROS。自己开解释器时再 `source /opt/ros/<jazzy|humble>/setup.bash`；启动器会自己把已安装的发行版加进模块路径。

批准计划对象留在本进程。动力学要 `sample` 策略交出来的那一个（上游偏航和 certified 运行时都挂在上面），ROS 消息里是观测和计划令牌。编队协议载荷用 `float.hex` 往返，计划哈希和占用规范形式不变。

## 场景套件、视频、nightly（票 04）

`scenes/suite.txt` 是 regress 与 nightly 共用的列表（`卡@策略`）：

- `g2-hover` / `g2-hover-lift` @ `hover`
- `g2-pillar` @ `upstream`（票 02 的场景卡和策略；本票不改策略。upstream 的到达、碰撞和到达时间已冻结；certified 对照仍只报数）
- `pillar-forest` / `narrow-corridor` @ `hover`（新几何，悬停占位）
- `open-3drone` @ `hover`（票 05 的三机卡；几何是空的 `open-area.json`）
- `formation-hold` @ `formation`（票 06）
- `crossing-pillars` @ `crossing`（票 07）
- `formation-switch` @ `switch`（票 08：窄门触发横排↔纵列。L0 变阵的到达、过渡碰撞和队形误差已冻结；L1 抽检仍只报数）
- `crossing-forest` @ `crossing` seed 0（票 07b medium，只跑 seed 0）
- `formation-6-open` @ `formation`（票 11a，进程内零故障。故障档位和 ROS 2 形态不进套件）
- `crossing-6-forest` / `crossing-6-forest-sparse` @ `crossing` seed 0（票 13f。单次墙钟约 216 s / 156 s，超过 60 s，只跑 seed 0；10 seed 全量仍用上面的批跑命令）

G2 几何仍只在 `scenes/g2-scene.json`。柱林穿越是票 07，编队是票 06。

```bash
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/open-3drone.json@hover --jobs 4 --out out/suite
.venv/bin/python -m yopo_l0.render --log out/suite/g2-pillar@upstream/seed0/ticks.jsonl --scene scenes/g2-pillar.json --out out/suite/g2-pillar@upstream/seed0/preview.mp4
scripts/nightly.sh
```

`scripts/nightly.sh` 把 JSON、Markdown、逐 tick 日志和每张卡首个 seed 的俯视+侧视视频写到 **`out/nightly/YYYYMMDD/`**。播放时长 ≤60 s。同一天再跑会先清空该目录。失败时 `ticks.jsonl` 留在目录里，用上面的 `render` 命令重放。合 main 前的门仍是 `scripts/regress.sh`（另含 certified 冒烟）。不接 GitHub Actions；box 上把 `scripts/nightly.sh` 放进 cron 即可。系统需要 `ffmpeg`。

## L1 抽检（票 09 / 09b，Orin）

真 PX4 SITL + Gazebo 在 Orin 上跑。驱动是 yopo-swarm 的 `scripts/run_g2.sh nominal`（Docker Compose + `sim/px4/start.sh` + `scripts/g2_run.py`），不是 `ros2 launch`。Orin 上的 `uavs.launch.py` 只声明 `vehicles`、`ipc_root`、`log_root`、`body_radius`、`tracking_margin`、`code_sha`，没有 scene / seed / policy / shadow / start / goal。宿主没有 ROS 2 Humble，也没有 `ros2_ws/install`。

box 只做三件事：把场景卡导出成记录目录加一张工单、事后把 Orin 的 `result.json` 收成和 L0 同一张记录（`tier=L1`）、以及同卡的 L0/L1 对照。不在这里启动 PX4，也不改 PX4 参数和 `g2_run.py` 里的安全清单数值。

- 世界记录：`scene.json` 的字段与 `sim/px4/g2-scene.json` 相同（`world`、`goal_enu`、轴对齐 `boxes`）。圆柱和别的几何直接拒绝。nominal 用栈里已有的 G2 世界，不把场景路径传给 launch。
- 安全清单：`g2_run.py`（机体半径 0.35、`tracking_margin` 0.15、`switch_delay` 2.0、`start_pad_radius` 1.2）。这些数不进要执行的命令。
- 种子：只经环境变量 `YOPO_SIM_SEED`。
- 策略：今日这条路径是已部署的 certified 栈，不接受 `--policy upstream`。对照表里 L1 写 `policy=deployed-certified`。把 upstream 移植进 L1 要改 yopo-swarm，不在本票。

工单 schema 是 `yopo-restart/l1-job-card/1`（`--prepare-only` 写到 `--out/job-card.json`）。调度器要看的字段：`repo`、`swarm_sha` / `swarm_sha_command`、`command`、`env`、`artifacts`、`timeout_s`（1800）、`pass_fail`。`uavs_launch_py.used` 是 false。

Python：export / import / report 只要 numpy。脚本不在 Orin 宿主上创建 venv，也不安装 torch（aarch64 没有 `torch==2.4.1+cpu` 的 wheel）。已有 `.venv` 或宿主 `python3` 能 `import numpy` 就用它；否则用已经存在的镜像 `yopo-swarm:orin-infer-cu13`。仿真容器是 `yopo-sim-g2`，由 `run_g2.sh` 自己拉起。

```bash
# box 或 Orin，只写工单，不飞：
scripts/l1_spotcheck.sh --prepare-only --scene scenes/g2-pillar.json --seed 0 \
  --policy upstream --shadow on --out out/l1/g2-pillar
# Orin 全量。PX4_PARAM_DUMP_CMD 必须只读（param show 可以）。
YOPO_SWARM_ROOT=/path/to/yopo-swarm PX4_PARAM_DUMP_CMD='param show' \
  scripts/l1_spotcheck.sh --scene scenes/g2-pillar.json --seed 0 \
  --policy upstream --shadow on --out out/l1/g2-pillar
# 对照。已有 L0 结果就给 --l0；x86 上也可以让报告按同一张卡重跑 seed 0：
.venv/bin/python -m yopo_l1.report --l0 out/g2-pillar/seed0 --l1 out/l1/g2-pillar/log \
  --out out/l1/g2-pillar/l0-vs-l1.md
.venv/bin/python -m yopo_l1.report --scene scenes/g2-pillar.json --seed 0 \
  --l1 out/l1/g2-pillar/log --out out/l1/g2-pillar/l0-vs-l1.md
```

脚本内部执行的就是 `YOPO_SIM_SEED=<seed> bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal`。跑前跑后各做一次只读参数 dump，并给 `g2_run.py` 和文件名含 safety / checklist 的文本做 sha256；有差异就失败。`param set` / `save` / `load` / `reset` / `import` 会在执行前拒绝。批跑表末尾两列是起飞交接和 RTF，L0 留空。RTF = 仿真秒 / 墙钟秒。

## 合 main（premerge）

不接 GitHub Actions。每个要进 main 的 PR 在本机跑：

```bash
scripts/premerge.sh 123            # 查 MERGEABLE，临时 worktree 里跑 regress.sh，查 Co-authored-by，在 PR 留一条评论
scripts/premerge.sh 123 --merge    # 上面全部通过后：gh pr ready，再 gh pr merge --merge --match-head-commit <检验过的 sha>
scripts/premerge.sh 123 --squash   # 同样只在全部通过后合并；说明由各提交拼成，并去掉 Co-authored-by 行
```

不可合并、regress 非 0、解析不到 pytest 计数或批跑表、任一提交带 `Co-authored-by` trailer、评论或合并失败，退出码都不是 0。评论里有 head sha、L0 与 core 的 passed/skipped、regress 退出码、总退出码，以及批跑 Markdown 表。

不给 `--merge` / `--squash` 时脚本不合并。它不 `git push`，也不 force-push。worktree 用完即删。

`make venv` 在 `.venv/bin/python` 已在、且 `requirements.txt` / `pyproject.toml` / `core/pyproject.toml` 的哈希未变时直接返回，避免 `uv venv` 报 virtual environment already exists。第三方装在 `~/.cache/yopo-restart/deps/<哈希>`（本目录只挂 editable）；wheel 走 uv 缓存，所以新 worktree 不重装 torch。
