# 已知限制（G7）

G7 的口径是「一键启动与可追溯结果」。这份文档是 [M5 交付包](SPEC.md) 里的已知限制：写清 L0 是哪一层、和上游有哪些已经登记的偏离、哪些数字只报数、L1 抽检卡在哪、合入前回归覆盖到哪一步，以及朝真实仿真还缺什么。

每次运行都要带层级（L0 / L1）、策略（`upstream` / `certified`）和影子盾开/关。不要把 L0 的点质量结果当成 PX4 或 Gazebo 的结果（[SPEC](SPEC.md) 用户故事 2，以及「L0 过于乐观」那条风险）。

下面出现的数字都指向结果文档或 ADR，不是编的。

## L0 是什么，不是什么

[SPEC](SPEC.md) 把仿真分成两层：

- **L0**：box 上 ROS-free、锁步、确定性的轻量仿真层。
- **L1**：现有 PX4 SITL + Gazebo 保真层。

L0 已经落地的构造是：

- **动力学**：点质量加一阶加速度滞后（`PointMassLag`）。场景卡和跟踪器的 `a_max` 都是 6。急刹不得再写死计划 `a_max=80`，见 [ADR 0011](adr/0011-brake-gap-horizon-and-amax.md)。这不是 PX4 位置环。SPEC 里后加的一阶 SO3 已由 [E-SO3b](research/e-so3b-suite.md)（[#130](https://github.com/unununnnn/yopo-restart/pull/130)）在冻结套件上测过，默认仍关；打开后六机稀疏到达 59/60（点质量 60/60），碰撞 0。
- **传感器**：按相机内外参做光线投射，输出深度图；量程、视场、分辨率、噪声、帧率可配，时间戳由仿真时钟给。规划器只拿 `Observation`（深度帧、里程计、目标、时间），拿不到场景几何真值。这不是渲染级相机：纹理和光照在 SPEC 的 Out of Scope 里。
- **多机总线**：每机一个独立规划实例和独立传感器。机间消息只走进程内消息总线，没有共享内存捷径；总线可以注入延迟、丢失、重复、乱序。
- **锁步时钟**：每 tick 依次是传感器 → 规划 → 进程内总线投递 → 动力学。规划消耗的墙钟不计入仿真时间。规划预算、帧龄、有效期一律按仿真时间判定。同一份代码、同场景卡、同 seed，两次运行的结果记录逐字段一致（整回路墙钟、规划墙钟、以及仓库 `commit` 脏标记除外，见 [T2](research/t2-six-drone-det.md)）。

L0 **不等价于** L1，也不是实飞：

- 它不经过 PX4 飞控，不经过 Gazebo 的传感器和物理，也没有起飞交接耗时和 RTF。结果记录里的 `tier` 是 `L0`。
- [票 10](tickets.md) 的 ROS 2 形态仍然是 L0 世界和 L0 锁步时钟，只是把每机规划放到 ROS 2 节点里，用 `/clock` 发仿真时间。不要把它读成 L1。
- 里程计是真值。没有 PX4 估计误差，也没有实机噪声以外、场景卡里没写的那一类传感缺陷。
- 实飞、改安全清单数值、改 PX4 参数，都在 SPEC 的 Out of Scope 里。

## 偏离上游

和 YOPO-MINCO 上游不一致的构造都记在 ADR 里。口头说「upstream 过了」时，要连这些偏离一起读。默认 `safe_radius` 0.05 m 与上游 d5e1e60 相同，**不是**偏离；设成别的值才标「偏离上游」，见 [ADR 0006 (h)](adr/0006-yopo-minco-baseline.md) 和 [07d](results/07d-safe-radius.md)。本仓库场景用的都是 0.05 m。

下面各条用对应 ADR 的决定原文（或原文要点），不另写一套说法。

| 项 | 决定（ADR 原文） | ADR |
|---|---|---|
| 终端规则 | 「L0 的 `upstream` 默认终端是项目规则 `stop_at_goal`，半径 1.5 m，停车段 2 s。不用上游原样的 `hold_here`。」上游半径是 5 m。在 G2 oracle（距目标 ≤ 0.4 m、速度 ≤ 0.2 m/s、连续保持 1 s）下，`hold_here` 到不了终点。 | [0009](adr/0009-stop-at-goal-vs-hold-here.md) |
| 承诺视界与急刹加速度 | 「构造 `CrossingFleet` 时强制 `commit_horizon_s ≥ N_agents × window_len_s + 0.25`。」「急刹和计划契约的 `a_max` 等于该卡的 L0 动力学 `a_max`。卡上没写时用 `PointMassLag.a_max`（当前 6）。禁止再写死计划 `a_max=80`。」六机视界因此抬到 1.75 s；三机卡上写的 1.0 s 已在下界上，构造时不抬高。 | [0011](adr/0011-brake-gap-horizon-and-amax.md) |
| Delay Check | 「编队把 δ_DC 设为 0.05 s」。收齐 ACK 并且到达 `broadcast_t + δ_DC` 之后才 COMMIT。「穿林和变阵显式保持 `delay_check_s = 0`。YOPO 的候选、解码和 `safe_radius` 都不改。」不换 YOPO，也不引入 Gurobi。上游协议没有这段等待。 | [0012](adr/0012-rmader-dual-track.md) |
| 进度门控 | 「穿林（`CrossingFleet`，场景卡带 `forest`）默认 `progress_gate=true`。」「门控开时，虚拟中心不再用墙钟 × `speed_mps`。」「开阔区 `FormationFleet` 的 `progress_gate` 固定为 false。」票 13e 起，「六机 medium / sparse 穿林卡写 `progress_gate=false`」，门控代码和开关保留。三机穿林卡仍写 true，柱林仍默认关。 | [0013](adr/0013-forest-progress-gate.md)、[0015](adr/0015-ghost-brake-stops.md) |
| 弹性槽位与净空门 | 「只对带 `forest` 且不少于六机的 `CrossingFleet` 生效。」「林带内……槽位是软目标。」「COMMIT 前对已拟合轨迹按 0.02 s 采样。中心到树干或矮枝表面的最小距离必须 ≥ `body_radius + clearance_margin_m`。margin 默认 0.15 m……不够就记 `CLEARANCE_SKIP`，不把这条多项式交给 `_as_plan`，也就不会提案、不会 COMMIT。」「这是过滤，不是优化。」不引入 CPLEX。 | [0014](adr/0014-elastic-slot-clearance-gate.md) |
| 幽灵刹车停点 | 「六机林带（`_forest_six()`）的提案前自检改为 `_conflicts_with_live()`：每个邻机只看 `peer.plans[-2:]`，也就是它的 committed + previous，加上它的预留（`delay_check_s > 0` 时再加 heard）。这与邻机投票时用的集合一致。三机穿林和柱林仍走 `conflicts_with_neighbors()`，行为不变。」六机林带卡同时暂关进度门控，见上。 | [0015](adr/0015-ghost-brake-stops.md) |
| 无解时的刹车 | 「L0 `UpstreamPolicy` 在走廊为空时，本帧停在当前里程计（`hold_traj`，码 `CORRIDOR_EMPTY_HOVER`），下一帧重规划。上游 d5e1e60 的 `control_pub` 在 `self.brake` 时不采样这条新轨迹，重复上一条 `PositionCommand`……crossing 无解且契约允许时提交一段运动学刹车（`BRAKE_ONLY`），否则留在上一条已提交计划。」 | [0006 (i)](adr/0006-yopo-minco-baseline.md) |

另外两件容易误读、但本身不是「改了上游选候选规则」的决定：

- L0 默认 `upstream` + 影子盾开。影子盾只记录会否拒绝，不改批准动作。要看现行全部门，必须显式 `@certified`。见 [ADR 0007](adr/0007-l0-default-upstream-shadow.md)。
- L0 上 `certified` 的 `ENV:UNKNOWN` / `NO_ENV_SAFE_CANDIDATE` / 到达 0 写进表，不作为合入失败，也不把 UNKNOWN 改成 FREE。见 [ADR 0010](adr/0010-certified-env-unknown-reported.md)。

## 只报数、不挡合入

[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md)（accepted，frank 2026-11-10 批准；M2 队形门 6.0 m）打开了已冻指标的批跑退出码。[ADR 0008](adr/0008-report-only-until-frozen.md) 仍然管还没冻的项：这些数字每次都写进表，失败不把 `yopo_l0.batch` / `scripts/regress.sh` 置成非 0。全表和出处见 [票 11 冻结建议表](results/11-freeze-recommendation.md)。建议表里的数字来自 main `8ee3ca1d11056b653c659acba311c5e19482b134`（票 13e）。

继续只报数的是：

| 项 | 已有数字 | 出处 |
|---|---|---|
| 所有墙钟 | 随机器负载变，同 seed 也可以差几秒到几倍；不参与确定性比较。六机林带单次已经超过 60 s。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |
| certified 对照 | 同一张 G2 柱子卡、10 seed、全时长：到达 0/10，碰撞 0，结果都是 `TIMEOUT`。规划码是 `ENV:UNKNOWN` / `NO_ENV_SAFE_CANDIDATE`，飞机留在起点。这是对照臂，不是里程碑通过线。同一 commit 上 `upstream` + `stop_at_goal` 是 10/10、碰撞 0。 | [ADR 0010](adr/0010-certified-env-unknown-reported.md)、[M0 样例](results/m0-g2-pillar.md) |
| 三机林 sparse / dense | 07d（13a / 13b 之前，默认 `safe_radius` 0.05 m）：sparse 10/10、碰撞 0；dense 1/10、撞树 9。两档都没有在冻结用的 main `8ee3ca1` 上重跑。密档本来就不是通过线，不要冻成门。`crossing-forest` 只对 medium 开门。 | [07d](results/07d-safe-radius.md)、[冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |
| 三机林 medium 的数值净空 | 当前 main 重跑最紧 0.260 m，不够支撑柱林那道 0.40 m 的门。碰撞 = 0 和到达 10/10 已冻结。 | [冻结表](results/11-freeze-recommendation.md)（`out/t11-m2-forest`） |
| 六机林带的官方 p95 和相对 p95 | 官方 p95：medium 23.04–26.30 m，sparse 18.73–22.92 m。相对 p95（每拍以全队平均位置为中心，再取各 seed p95 的中位数）：medium 9.29 m，sparse 6.17 m。[ADR 0015](adr/0015-ghost-brake-stops.md) 写成 9.3 m / 6.2 m，是同一组数的四舍五入。林带卡 `progress_gate=false`，官方 p95 对着墙钟 × speed 的槽位，和 M1 的相对槽位不是一个口径。默认关的 `task.forest_six_prefix`（[E-PG5](research/e-pg5-forest-six-prefix.md)）按 `_forest_six()` 隔离后，把相对 p95 中位数压到 medium 2.742 m / sparse 2.701 m（全部 20 粒 < 5.0 m，60/60 到达，碰撞 0），且不触碰三机穿林；磁盘卡仍保持默认关。 | [冻结表](results/11-freeze-recommendation.md)、[13e](results/13e-ghost-stops.md)、[E-PG5](research/e-pg5-forest-six-prefix.md)、[ADR 0015](adr/0015-ghost-brake-stops.md) |
| 六机林带的数值净空 | 最紧 0.126 m（medium）/ 0.158 m（sparse）。COMMIT 前计划净空门是 0.50 m，跟踪差会吃掉一截。已冻的是碰撞 = 0，以及最小机间距离 ≥ 0.70 m。 | [冻结表](results/11-freeze-recommendation.md)、[13e](results/13e-ghost-stops.md)、[ADR 0015](adr/0015-ghost-brake-stops.md) |
| M4 的 ROS 2 全档矩阵 | 已由 [M4-ROS2](research/m4-ros2-matrix.md)（[#129](https://github.com/unununnnn/yopo-restart/pull/129)）跑完：六档 × seeds 0–4，30/30 到达，碰撞 0，队形 p95 0.066 m，最小机间 2.980 m，到达时间 20.13 s，与进程内一致。进程内六档仍按冻结表。 | [M4-ROS2](research/m4-ros2-matrix.md)、[冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |
| M3 的 L1 抽检 | 新方案已接到本仓库 PX4 SITL，数字只报数、判据未冻结。S6（[#128](https://github.com/unununnnn/yopo-restart/pull/128)）单机 g2-pillar 10/10；S5b（[#134](https://github.com/unununnnn/yopo-restart/pull/134)）编队保持 15/15；S7（[#139](https://github.com/unununnnn/yopo-restart/pull/139)）变阵 5/5、交叉自由跑 0/5；B1（[#142](https://github.com/unununnnn/yopo-restart/pull/142)）锁步交叉 4/5、变阵 5/5；[S8](research/s8-forest-and-m4-sitl.md) 补齐三机林带 `crossing-forest-sitl80` 4/5、六机 `formation-6-open-sitl80` 5/5（碰撞 0）、`crossing-6-open-sitl80` 5/5（碰撞 0）及 `crossing-6-forest-sitl80` 0/3 锁步抽检。票 09 的旧仓路径仍关闭，见下一节。M3 的 L0 变阵门仍按冻结表。 | [s6](research/s6-sitl-upstream-m0.md)、[s5b](research/s5b-sitl-formation-verify.md)、[s7](research/s7-sitl-crossing-switch.md)、[b1](research/b1-sitl-lockstep.md)、[冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md)、[tickets](tickets.md) |
| 影子拒绝率 | L0 上开盾时多为 1.000。那是「若开硬门会拒」的记录，不改动作，不能当成失败门。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0007](adr/0007-l0-default-upstream-shadow.md) |
| 六机柱林 | 未测。单次墙钟超过 60 s，不进套件，没有当前 main 的表。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |

已冻的到达、碰撞和距离门见冻结表和场景卡 `criteria.gates`。那些门不在这里放宽，也不在这里重写。

## 票 09 已关闭，以及它留下的缺口

[票 09](tickets.md) 是 SPEC 里 M3 后半的 L1 抽检通道：同一张 M0 场景卡在现有 PX4 SITL 栈上跑一次，产出 L0 / L1 对照表。frank 2026-11-09 关闭了这张票。

关闭原因写在票列表里：09b 跑的是旧仓 yopo-swarm 已部署的 `deployed-certified` 栈，不验证本仓库的新方案（L0 默认 `upstream` + 影子盾）。L1 要等新方案接到 PX4 SITL 之后另开票。09a / 09b 的通道代码保留。

已知缺口（同一段票注）：

- 这条路径仍是已部署的 certified 栈，不接受 `--policy upstream`。对照表写 `policy=deployed-certified`（导出时是 `policy_requested=upstream`、`policy_executed=deployed-certified`）。
- 把 upstream 移植进 L1 要等以后的 yopo-swarm 票。本仓库不改那个仓库。
- 不要在这条已关闭的工单上重跑 Orin，也不要改 PX4 参数或安全清单数值。

票 09 的旧仓路径仍然关闭。新方案的 L1 抽检已由 S6、S5b、S7、B1 另开票完成，数字只报数，见上一节的「只报数」表和下面的「朝真实仿真的下一步」。不要在已关闭的 09 上重跑 Orin，也不要改 PX4 参数。

## 合入前回归里的六机林带

票 13f 把 `crossing-6-forest.json@crossing@0` 和 `crossing-6-forest-sparse.json@crossing@0` 写进了 `scenes/suite.txt`，合入前回归不再漏跑这两张卡。见 [tickets.md](tickets.md) 的 13f 条。

限制是：

- **只跑 seed 0。** 单次墙钟约 216 s（medium）和 156 s（sparse），超过「单次超过 60 s 不进套件」的旧线，所以按 07b 惯例只收 seed 0。10 seed 全量仍用 README 里的批跑命令，不在每次合入前跑。
- 有更细的墙钟出处：[PR #31](https://github.com/unununnnn/yopo-restart/pull/31) 正文写 medium seed 0 墙钟 216.2 s、sparse seed 0 墙钟 156.4 s，到达都是 6/6，障碍和机间碰撞都是 0；同一次本机回归的批跑套件墙钟是 613 s（已含这两张卡的 seed 0）。再加上 L0 / core 的 pytest，整次 `scripts/premerge.sh` / `scripts/regress.sh` 要十几到二十分钟。
- 10 seed 的到达、碰撞门已经在 13e 上冻过：medium 60/60、sparse 60/60，碰撞 0，见 [13e](results/13e-ghost-stops.md) 和 [ADR 0016](adr/0016-m0-m4-freeze-recommendation.md)。套件里的 seed 0 是回归冒烟，不是把 10 seed 换成 1 seed。

## 朝真实仿真的下一步

这些是限制带来的后续工作，不是这张文档要改的代码。下面「已完成或已有结论」里的数字摘自对应 research 文档，只报数。

### 已完成或已有结论

1. **本仓库的 `upstream` + 影子盾已接到 PX4 SITL，方向 B 已启动。** 不要重开已关闭的票 09，也不要在 Orin 上重跑旧仓 `deployed-certified`。L1 的安全清单数值和 PX4 参数仍然不动。本仓库仍然不改 yopo-swarm。
   - [S6](research/s6-sitl-upstream-m0.md)（[#128](https://github.com/unununnnn/yopo-restart/pull/128)）：同一张 M0 卡 `g2-pillar`，L0 / SITL 都是 10/10 到达、碰撞 0；去掉起飞交接后，SITL 到达仍比 L0 慢约 1.9–2.9 s；影子拒绝率两边都是 1.000。
   - [S5b](research/s5b-sitl-formation-verify.md)（[#134](https://github.com/unununnnn/yopo-restart/pull/134)）：编队保持 seeds 0–4，L0 / SITL 都是 15/15 到达、碰撞 0；队形 p95 从 0.027 m 升到 0.310–0.361 m。
   - [S7](research/s7-sitl-crossing-switch.md)（[#139](https://github.com/unununnnn/yopo-restart/pull/139)）：变阵 5/5，交叉卡 `crossing-pillars-sitl80` 自由跑 0/5（两臂合计 0/10），碰撞 0，RTF 最低 0.4382，机间时间戳极差最大 251.063 s。
   - [B1](research/b1-sitl-lockstep.md)（[#142](https://github.com/unununnnn/yopo-restart/pull/142)）：锁步交叉 4/5（12/15 到达，碰撞 1），三机里程计仿真时差 0.012–0.016 s；变阵仍是 5/5，队形 p95 中位 0.178 m。B1 写过剩下的失败是规划轨迹擦柱、PLAN 段最小净空 −0.010 m；[B2](research/b2-sitl-plan-clearance.md) 更正：这个字段量的是 PLAN 段里程计位置离柱面的净空，不是规划轨迹。缺省失败趟的设定点全程净空 ≥ 0.47 m，撞柱那一拍 0.81–1.10 m。
   - [B2](research/b2-sitl-plan-clearance.md)（[#144](https://github.com/unununnnn/yopo-restart/pull/144)，`f957d0c`）：锁步交叉补到 seeds 0–9 是 7/10（21/30 到达，碰撞 3）。失败发生在 PLAN 起算 1.68–2.18 s、刚加速到 5.24–5.47 m/s 时：规划器给出约 1 m 的反向横移，PX4 机体几乎没有横向动作，直线撞柱。根因是 PX4 起步跟踪滞后，加上规划器从已提交计划取起点、不看真实状态。规划时横向跟踪误差 p95 在 SITL 是 0.40–0.66 m，L0 约 0.06 m。本票的修复候选都没有好过缺省，开关默认关。
   - [B3](research/b3-sitl-launch-tracking.md)（[#148](https://github.com/unununnnn/yopo-restart/pull/148)，`14c22f0`）：起步限加速 3 m/s²、持续 1.5 s 的候选 (a) 是 8/10（24/30 到达，碰撞 2），缺省 7/10；到达中位数 10.49 s（缺省 14.02 s），横向跟踪 p95 中位数 0.72 → 0.65 m。锁步下同 seed 会翻，8/10 对 7/10 只差一趟，在噪声内；更能说明 (a) 起作用的是失败样子变了（推迟到 PLAN 后 2.6–3.1 s，设定点离柱面 0.15–0.19 m，有一趟 −0.17 m）。按真实状态重规划的候选 (b) 不推荐：0.5 m 档是 4/10，比缺省差。(a) 在 L0 `crossing-pillars` 的 seed 1、6、9 上编队误差 p95 是 6.13 / 6.29 / 6.13 m，越过冻结线 6.0 m，开关默认关。
   - [B4](research/b4-center-launch-ramp.md)（[#150](https://github.com/unununnnn/yopo-restart/pull/150)，`48cd746`）：(a)+中心斜坡让 L0 `crossing-pillars` 10/10 过冻结线，编队误差 p95 最大 4.31 m（缺省 4.61 m，(a) 单独开时 6.29 m）。锁步 SITL 两批是 8/10、7/10（合计 15/20），横向跟踪 p95 中位数 0.81、0.98 m。剩下的失败仍是计划贴柱（设定点净空 0.10–0.25 m）。中心斜坡仍默认关。
   - [B5](research/b5-a-rerun-ramp-ref.md)（[#151](https://github.com/unununnnn/yopo-restart/pull/151)，`5b5e942`）：(a) 又跑一批仍是 8/10，两批合计 16/20（50/60 到达，碰撞 3），横向跟踪 p95 中位数 0.65、0.63 m。参考斜坡（`ref_ramp_mps2`）让 L0 `crossing-pillars` 10/10 过冻结线，编队误差 p95 最大 4.41 m，但只改了编队误差的量尺，不改飞行（到达时间、最小障碍距离和最小机间距离与 (a) 逐位相同），默认关。B5 补记：B4 里「中心斜坡让横向跟踪变差」站不住，因为同一份 (a) 规划不同批次的横向跟踪 p95 中位数本身在 0.63–0.88 m 波动（0.65、0.63、0.88 m）；B4 两批的 0.81、0.98 m 落在这个范围内。中心斜坡仍默认关。
   - [B6](research/b6-plan-pillar-clearance.md)（[#156](https://github.com/unununnnn/yopo-restart/pull/156)，`47da396`）：柱净空门 `pillar_clearance_m`（读场景柱几何，打开即偏离上游）默认关。L0 `crossing-pillars` 上一次都没拒绝，飞行与不开逐位相同。锁步 SITL 上 (a)+0.5 m 是 10/10（30/30 到达，碰撞 0），(a)+0.3 m 是 9/10（碰撞 0，剩下一趟是门全拒导致的 PLAN_TIMEOUT）；和 (a) 两批的 16/20 比仍在噪声内（p≈0.27）。单开柱门 6/10、碰撞 4，撞柱趟设定点净空都 ≥ 0.77 m，是跟踪滞后，柱门管不了。
   - [B7](research/b7-clearance-fallback.md)（[#160](https://github.com/unununnnn/yopo-restart/pull/160)，`285e2ee`）：柱门全拒时退回离柱最远候选，不是新开关，默认仍关。L0 上门一次都没拒绝，退路未触发，飞行与 B6 逐位相同。锁步 SITL 上 (a)+0.3 m 的 seed 7 不再 PLAN_TIMEOUT（本趟退路未触发，拒绝 0）；两臂 20 趟超时都是 0。(a)+0.3 m 仍是 9/10，但失败从超时变成撞柱（seed 5，退路触发，设定点净空 0.22 m）；(a)+0.5 m 从 B6 的 10/10 变成 9/10，seed 6 同样是退路后撞柱（设定点 0.25 m）。
   - [B8](research/b8-fallback-floor.md)（`2e2b670`）：增加默认关的退路净空地板 `pillar_fallback_min_m`，当全拒最远候选仍低于地板时改走运动学刹停或保持，不再交出贴柱退路；配起步斜坡后 L0 `crossing-pillars` 10/10 过门（p95 最大 4.410 / 4.306 m ≤ 6.0 m），锁步 SITL 20 趟碰撞降为 0（`(a)+reframp=3+0.3/0.2` 为 9/10、29/30 到达；`(a)+reframp=3+0.5/0.3` 为 8/10、28/30 到达，`px4_parameters_written == []`）。
   - [S8](research/s8-forest-and-m4-sitl.md)（`41a5894`）：锁步 SITL 扩到 `--n 6` 与林带多圆柱/矮枝 SDF 世界（`--scene-world`，不改飞控参数）。M2 `crossing-forest-sitl80` 4/5（14/15 到达），M4 `formation-6-open-sitl80` 5/5（30/30 到达，碰撞 0），`crossing-6-open-sitl80` 5/5（30/30 到达，碰撞 0），`crossing-6-forest-sitl80` 0/3（5/18 到达，暴露出六机密树下 PX4 横向跟踪滞后）。
2. **M4 的 ROS 2 全档全 seed 矩阵已跑完，Ubuntu 22.04 Humble 实跑桥接已打通。** [M4-ROS2](research/m4-ros2-matrix.md)（[#129](https://github.com/unununnnn/yopo-restart/pull/129)）：六档 × seeds 0–4，30/30 到达，碰撞 0，队形 p95 0.066 m，最小机间 2.980 m，到达时间 20.13 s，与进程内一致。[ros2-rflysim-humble](research/ros2-rflysim-humble.md)（`6ce8033`）在 Ubuntu 22.04（`RflySim-20.04`）上打通 Python 3.12 `.venv` 到系统 CPython 3.10 `/opt/ros/humble` `rclpy` 的锁步管道桥接，`tests/test_ros2_*.py` 29/29 通过、0 跳过。
3. **一阶 SO3 已在冻结套件上测过，默认仍是点质量、`a_max=6`。** [E-SO3b](research/e-so3b-suite.md)（[#130](https://github.com/unununnnn/yopo-restart/pull/130)）：13 行里破了 1 行，M4 六机稀疏到达 59/60（点质量 60/60），碰撞 0。不改默认动力学；若以后打开，到达和净空要重测，不能沿用冻结表里的秒数和米数。
4. **六机林带短前缀已测，不写进场景卡，开关保持默认关。** [ADR 0015](adr/0015-ghost-brake-stops.md) 写过的「只修自检、门控仍开时 50 s 内飞不完」，已由短前缀量过：
   - [E-PG3](research/e-pg3-leader-prefix.md)（[#131](https://github.com/unununnnn/yopo-restart/pull/131)）：0.7 s 短前缀两张六机林带都是 60/60 到达，障碍和机间碰撞 0，平均到达 medium 42.71 s、sparse 36.84 s。
   - [E-PG4](research/e-pg4-prefix-suite.md)（[#137](https://github.com/unununnnn/yopo-restart/pull/137)）：同一档在 M2 `crossing-forest`（medium）破门，到达 24/30（点质量 30/30），碰撞 2（点质量 0）。
   - [E-PG5](research/e-pg5-forest-six-prefix.md)（`afe44dd`）：新增默认关的 `task.forest_six_prefix`（缺省 0.7 s），严格受 `_forest_six()` 门控。六机林带 medium / sparse（seeds 0–9）均保持 60/60 到达、0 碰撞，相对队形 p95 中位数降至 2.742 m（2.286–3.084 m）/ 2.701 m（2.260–3.471 m，全部 < 5.0 m）；同时 M2 三机 `crossing-forest` 保持 30/30 到达、0 碰撞且逐拍哈希与基线 10/10 一致。
5. **影子盾环境证据已接入，并接到了 SITL；契约码多数是误报。** 研究开关都默认关。
   - [SH1](research/sh1-shadow-env-evidence.md)（[#133](https://github.com/unununnnn/yopo-restart/pull/133)）：打开 `shadow_env_evidence` 后，三张卡的环境拒绝率都变成 0；`would_reject` 只有 formation-hold 从 1.000 降到 0.028，g2-pillar 和 crossing-pillars 仍是 1.000，剩下的是计划契约码。当时开关还没接到 `sitl_bridge`。
   - [SH2](research/sh2-sitl-shadow-evidence.md) 随 [#147](https://github.com/unununnnn/yopo-restart/pull/147)（`c37ce78`）合入，[#146](https://github.com/unununnnn/yopo-restart/pull/146) 已关：开关接到 `sitl_bridge`。L0 上结论与 SH1 一致。
   - [SH2b](research/sh2-sitl-shadow-evidence.md)（[#149](https://github.com/unununnnn/yopo-restart/pull/149)，`c929491`）：被串话污染的 4 趟在隔离网络上重跑，4/4 成功到达、碰撞 0，结论不变。锁步交叉 seeds 0–4 合计：开证据后环境拒绝率 0.931→0.013，`would_reject` 仍是 0.997，主要靠契约码和 `TRACKING_BOUND`（0.15 m）打满。
   - [SH3](research/sh3-contract-codes.md)（[#147](https://github.com/unununnnn/yopo-restart/pull/147)）：契约码多数是误报（把已经在飞的上游多项式按认证 `Plan` 语义再包一层）。修正开关 `shadow_contract_align` 默认关；打开后 L0 上 g2-pillar 的 `would_reject` 从 1.000 降到 0.0114，crossing-pillars 从 1.000 降到 0.0176。对齐后不再把 `TRACKING_BOUND` 算进 `would_reject`，阈值 0.15 m 本身不改。
   - [SH4](research/sh4-tracking-bound.md)（[#155](https://github.com/unununnnn/yopo-restart/pull/155)，`9cb9eab`）：把 `TRACKING_BOUND` 当成计划危险是误报。开关 `shadow_tracking_align` 默认关；打开后 0.15 m 超界仍记在直方图，但不单独打满 `would_reject`。阈值不改。当时没有新跑 SITL。
   - [SH5](research/sh5-sitl-shadow-align.md)（[#159](https://github.com/unununnnn/yopo-restart/pull/159)，`ac3c1af`）：在隔离网络的锁步 SITL 上重测四个影子臂，开关默认仍关，0.15 m 不改。`crossing-pillars-sitl80` seeds 0–4：缺省 `would_reject` 1.000、环境拒绝率 0.9046；开证据后环境拒绝率 0.0144，`would_reject` 仍是 0.9931；再开契约对齐，`would_reject` 降到 0.0519，`TRACKING_BOUND` 从码表拿掉；三个开关一起开，`would_reject` 0.0203，0.15 m 超界 7861 次仍在直方图，留下的是 `TRANSITION:MOTION_KINEMATICS` 和少量 `ENV:OCCUPIED`。到达在 2/5 到 3/5 之间起伏，不能写成开关效果。`g2-pillar-sitl80` seeds 0–4 四臂都是 5/5 到达、碰撞 0；开证据后环境拒绝率 1.000→0，契约对齐后 `would_reject` 0.0125，再开跟踪对齐是 0.0655（全部是 `LAG_VIOLATION`），跟踪超界未计入会拒。

6. **打开后段之后的评估次数缺省已改为 20。** 研究开关仍默认关。
   - [E-B2q](research/e-b2q-evals-diag.md)（[#136](https://github.com/unununnnn/yopo-restart/pull/136)）：密林 10 次是 12/30、截断 130/362，20 次是 21/30、截断 0/418；014 对向 10 次是 8/30、截断 470/1293，20 次是 20/30、截断 0。
   - [E-B2r](research/e-b2r-evals-default.md)（[#140](https://github.com/unununnnn/yopo-restart/pull/140)）：把缺省从 10 改成 20，重跑与 E-B2q 的 20 次臂相同（密林 21/30，014 对向 20/30，截断都是 0）。
7. **学生网络线已测到噪声边界，先暂停。** 开关和库默认都没有改。
   - [D6e](research/d6e-start-knot.md)（[#135](https://github.com/unununnnn/yopo-restart/pull/135)）：把起步结点改写成沿目标位移，开阔三机两臂都是 0/30；开臂 g2 两个高度都是 5/5 撞柱。
   - [D7](research/d7-dagger.md)（[#138](https://github.com/unununnnn/yopo-restart/pull/138)）：开阔三机从 r0 的 0/30 到 r2 的 20/30，再回落到 r3 的 10/30（1.0 m）和 0/30（1.5 m），不单调；g2 从 r0 的 8/10 降到 r1–r3 的 0/10。
   - [D8](research/d8-dagger-variance.md)（[#141](https://github.com/unununnnn/yopo-restart/pull/141)）：同一份数据只换训练种子，同一格就能从 0/30 跳到 20/30；r0 的 g2 8/10 是幸运种子（换种子 1、2 都是 0/10）。限速 2.0 m/s 的配对对照里，g2 从 5/30 升到 15/30，开阔从 40/180 升到 70/180。
   - [D9](research/d9-cap-weight.md)（[#143](https://github.com/unununnnn/yopo-restart/pull/143)）：5 个训练种子下，D8 的两个正向信号都没有确认。限速 2.5 m/s 是唯一在开阔（150/300 → 170/300）、g2（15/50 → 30/50）、撞障（35 → 20 场）三项上都没有负配对差的设置，但不改库默认。
8. **高负载下 6 机同 seed「不一致」已定位到一次确定性比较误报。** [T2](research/t2-six-drone-det.md) 复现：两次 `run()` 之间只要有进程改了本工作区的已跟踪文件，结果里的 `commit` 就会从 `sha` 变成 `sha-dirty`，批跑报「不一致」，但 ticks 仍然相同。比较已不再看 `commit`；真的不一致时会把剩余差分写到 `det_diff.json`。硬断言保留。本票没有复现轨迹分叉，也没有去钉 BLAS 线程。[T1](research/t1-load-tolerant-timing.md)（[#153](https://github.com/unununnnn/yopo-restart/pull/153)）去掉了 `test_iteration_cap_and_time_budget` 里 `time_ms < 50` 的墙钟限，改成断言迭代帽和评估次数预算截断（缺省 `budget` 是 `"evals"`）；墙钟预算由假时钟测试覆盖。没有把 50 ms 放宽成更大的墙钟阈值，也没有改截断逻辑。六机同 seed 在高负载下偶发不一致，本票没有拿掉硬断言，也没有复现到差分。
9. **锁步下电池模拟报 unhealthy，已诊断、未修。** [BAT1](research/bat1-lockstep-battery.md)（[#157](https://github.com/unununnnn/yopo-restart/pull/157)）：根因是部分 PX4 实例解锁后停止发布 `battery_status`，指挥官按仿真钟 5 s 超时；电压始终 16.2 V。B1 第二版 10 趟三机交叉是 5/10 趟、8/30 机 unhealthy、failsafe 0；更大样本锁步第二版 72/207 趟、91/615 机；自由跑/早期 S7 9/37 趟、10/91 机。unhealthy 相对解锁中位 5.076 s（范围 5.016–5.100 s）。要修需改 PX4 或镜像，本仓不做。

10. **无 `avx512f` 的 x86_64 宿主（如 Intel Core Ultra 7 265KF）上的静态金样哈希存在 1-ULP float32 SIMD 差异，属预期跨微架构行为。** 本机同种子回归（`scenes/suite.txt` 全部 17 项批跑与同机开关等价断言）均 100% 确定性通过、碰撞 0；早期 5 条直接比对 AVX-512 机器录制静态 JSON/SHA256 夹具的 L0 用例，以及 `core/tests/test_train_shim.py::test_loss_terms_match_upstream_golden`（实测相对误差 `1.71e-5`，落在 `CROSS_PLATFORM_LOSS_RTOL = 1e-4` 诊断界内，但 `train_golden.REFERENCE_ENV` 仅按 `x86_64` 判 `1e-6` 严格门，`core/` 受 `VENDORED.sha256` 冻结不改），在无 `avx512f` 指令集的 CPU 上会因 oneDNN AVX2 与 AVX-512 归约顺序差异报出这 6 项固定偏差。不放宽静态金样，保留 AVX-512 CI 逐位校验。

### 下一步

1. **起步限加速 (a) 配合斜坡与退路净空地板（[B8](research/b8-fallback-floor.md)）已消除退路贴柱碰撞，所有研究开关仍保持默认关。** `pillar_fallback_min_m` 把 [B7](research/b7-clearance-fallback.md) 退路触发的撞柱清零（锁步 SITL 20 趟碰撞 0，`(a)+reframp=3+0.3/0.2` 达 9/10），配合 `ref_ramp_mps2` / `center_ramp_mps2` 在 L0 `crossing-pillars` 上也 10/10 守住 6.0 m 队形冻结线；但柱净空门依赖场景几何真值，仍标偏离上游、保持默认关。
2. **六机密林 PX4 横向跟踪滞后仍是 L1 的真实物理边界。** [S8](research/s8-forest-and-m4-sitl.md) 在锁步 PX4 SITL 上跑通了三机林带（4/5）与六机开阔编队/穿行（均 5/5、碰撞 0），但六机 medium 林带（L0 最紧净空仅 0.126 m）在飞控位置环滞后下 0/3 未过。L0 上 [E-PG5](research/e-pg5-forest-six-prefix.md) 已将六机林带相对队形 p95 压到 2.74 / 2.70 m（< 5.0 m）且不伤三机穿林，后续可在不改飞控安全参数的前提下评估其在六机林带 SITL 上的跟踪余量。
3. **锁步下电池模拟报 unhealthy（不改 `BAT_` 参数）。** [BAT1](research/bat1-lockstep-battery.md)（[#157](https://github.com/unununnnn/yopo-restart/pull/157)）已诊断：B1 第二版 10 趟三机交叉仍是 5/10 出现 `Battery unhealthy`、0 趟 failsafe。根因是部分 PX4 实例解锁后停止发布 `battery_status`，指挥官按仿真钟 5 s 超时；电压始终 16.2 V，自由跑也会出现。要修仍需改 PX4 或镜像。
4. **学生网络线暂停。** D6e 到 D9 的收益都在训练噪声范围内，人手先放到方向 B。限速 2.5 m/s 仍只是 D9 的候选对照臂，不改库默认。
5. **实飞仍不在范围内。** 不要把 L0 或 SITL 的到达率读成实飞。
6. **`explore`（无编队水平横排）只报数，不冻结，不进 `scenes/suite.txt`。** [EXPLORE](research/explore-free-swarm.md)：L0 十 seed 六个场景 120/120 到达、碰撞 0，六机密林 9.67 m/s、6.29 s；这用了 `a_max` 约 20 m/s² 的敏捷动力学和 ratio=1.7 的时间相似律，同卡 1x 标称动力学（`a_max=6`）为 5.70 m/s、11.70 s，两档都在报告里。偏离上游，结果标 `upstream_deviation`。[EXPLORE-SITL](research/explore-sitl.md)：锁步 PX4 SITL 在不改 PX4 参数下六机林带 4/5（对照 S8 `crossing-6-forest-sitl80` 0/3）、三机柱林 3/3、六机开阔 3/3、三机林带 4/6；速度受未改参数的 PX4 位置环限制（约 6–7.5 m/s），不能读成 L0 的 9.7 m/s，也不是实飞。
7. **微分平坦 CTBR 已有显式实验实现，尚未完成高速性能验收。** [中心向外长程报告](research/ctbr-center900.md) 记录六机大场景链路，`--control-mode ctbr --lockstep --policy explore` 才启用，默认仍为 PVA。起飞使用 PVA，PLAN 巡航用几何角速率/集体推力，终点目标锁定后由 PVA 一次性接管减速和保持，最后 NAV_LAND；不写 PX4 参数，不修改核心或冻结卡。当前目标速度 5 m/s，不能宣称已达到 [CTBR-1](research/ctbr-flatness-ticket.md) 原定的 8.5–10 m/s 提速目标，也不是实体飞机实飞。
