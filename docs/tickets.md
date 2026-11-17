# 票列表（来源：docs/SPEC.md「Tickets」）

状态：`todo` / `in-progress` / `done`。依赖未完成的票不开工。

## 已定（2026-11-09）

frank 拍板卡片未回；父代理代定（仅 L0，L1 不动）：

1. **L0 默认 `upstream` 策略 + 影子盾开**（`--policy` / `--no-shadow` 可覆盖）。逐 tick 拒绝码在票 03：每机一条，全体另汇总。
2. **M0–M4 判据先报数、后冻结**：frank 2026-11-10 已批准冻结建议表（ADR 0016 accepted；M2 队形门 6.0 m）。已冻指标失败时批跑退出非 0；其余仍只报数。碰撞数始终作为硬事实如实报。

| 票 | 标题 | 被谁阻塞 | 状态 |
|---|---|---|---|
| 01 | L0 单机悬停纵切（世界 + 深度 + 动力学 + 锁步 + 结果记录） | 无 | done |
| 02 | 上游等价策略 + 单机过 G2 柱子（M0 核心） | 01 | done |
| 03 | 影子盾（真正记录实现） | 02 | done（每机一条影子记录；总拒绝率+直方图；判据先报数） |
| 04 | 场景卡套件 + 结果表 + 视频 + nightly | 02 | done |
| 05 | L0 多机骨架 + 消息总线 + 故障注入 | 02 | done |
| 06 | 三机编队保持（M1） | 05 | done |
| 06b | 编队同步出发丢 ACK 后的追赶 | 06 | done |
| 06c | 六机开阔区 drop 20% 窗口内重发提案 | 06b | done |
| 13d | 双轨 Delay Check（RMADER 语义，不换 YOPO） | 06c | done |
| 07 | 三机交叉避障（M2） | 06, 04 | done |
| 07b | 不规则林（泊松盘、疏密档） | 07 | done |
| 07c | 密档撞树定位（不改规划器） | 07b | done |
| 07d | 可选 safe_radius，默认仍是上游 0.05 m | 07c | done |
| 08 | 一种变阵（M3 前半） | 07 | done |
| 09 | L1 抽检通道（M3 后半）。Orin 驱动是 `scripts/run_g2.sh nominal`，不是 `ros2 launch`。L1 策略记 `deployed-certified` | 03, 04 | closed（frank 2026-11-09 关闭：09b 跑的是旧仓 deployed-certified，不验证新方案；L1 等新方案接 PX4 SITL 另开票。通道代码 09a/09b 保留） |
| 10 | ROS 2 形态 | 06 | done（M1 三机同 seed：到达/碰撞/队形误差 p95/机间距离/影子拒绝率与进程内一致；机间消息走 ROS 话题） |
| 11 | 六机 + 故障注入（M4） | 07, 08, 10 | done（11a 开阔区楔形已进 `suite.txt`。11b：同一楔形穿 07b 的 medium / sparse 林。11c：六机林中撞树定位后只改编队层。13a–13e 已合。13e：六机林带 medium / sparse 均 60/60。收口冻结建议表见 `docs/results/11-freeze-recommendation.md` 和 ADR 0016（accepted，frank 2026-11-10 批准全部建议，M2 队形门 6.0 m）。已冻指标见场景卡 `criteria.gates`。柱林六机若单次超过 60 s 不进 `suite.txt`） |
| 13f | 六机林带卡加入合入前回归 | 13e | done（`crossing-6-forest.json@crossing@0` 与 `crossing-6-forest-sparse.json@crossing@0` 已进 `suite.txt`。单次墙钟约 216 s / 156 s，超过 60 s，按 07b 惯例只跑 seed 0。13e 上 10 seed 已是 60/60、碰撞 0。已冻到达满格、碰撞 = 0、最小机间距离 ≥ 0.70 m；两种 p95 和数值净空仍只报数。） |
| 12 | 交付包（M5） | 11 | done（12a–12f 已合，见下行） |
| 12a | README 顶部一键命令 | 11 | done（#33，合入 `78f63bcdb9bc`） |
| 12b | 结果总表 | 11 | done（#36，合入 `252bae86781e`） |
| 12c | 各里程碑交付视频 | 11 | done（#35，合入 `cf9c0b0b9832`） |
| 12d | 已知限制文档 | 11 | done（#34，合入 `7a545ac3a645`） |
| 12e | 上游策略 vs 影子盾对照表 | 11 | done（#37，合入 `0bb043cae739`） |
| 12f | 收口：README 交付包入口与票表 | 12a–12e | done（本票） |
| E-C1 | 外部跑 ego-planner-swarm，填论文同列的记录 | 无 | done（#42，合入 `ad20920c8c24`） |
| E-C2 | 3 机开阔对向卡，只报数 | 无 | done（#40，合入 `4c17a8d0b60f`） |
| E-C3 | 结果记录增加单次规划墙钟 | 无 | done（#41，合入 `0d7e138f7a80`） |
| E-C4 | 8 机对向，以及一张标明假设的密度场 | E-C2 | done（#50，合入 `c958a3e87fed`。八机开阔对向 seed 0 只报数，另有密度几何文件；该票本身未在密度场上跑 L0，后续见下行） |
| E-B1 | 后段接缝说明，不写求解器 | E-C1 | done（#47，与 E-B2 合入 `616909b78e6c`） |
| E-B2 | 单机 B 样条原型，默认关 | E-B1 | done（#47，与 E-B1 合入 `616909b78e6c`） |
| E-B3 | 机间椭球惩罚接到后段 | E-B2 | done（#65，合入 `f5cedba4540c`。默认关，只报数。收口见 [e-b3-closeout.md](research/e-b3-closeout.md)） |
| E-B3c | 机间椭球改到段内采样，默认关，只报数 | E-B3 | done（#76，合入 `41e820f2ed63`） |
| E-B3 收口 | 结论、开关保留与票表同步 | E-B3c | done（本票） |
| E-A1 | 机间过滤开关，默认关 | E-C2 | done（#45，合入 `e07d93659428`） |
| D1 | 几何回退 | 无 | done（#46，合入 `14dc25a8dab8`。默认关） |
| D2 | EGO-Swarm 教师数据 | 无 | done（#49，合入 `be734163b85b`。密度 0.42：单机 9/16，三机与六机 0/16） |
| D3 | 单机多头连续学生脚手架，默认关 | 无 | done（#51，合入 `7ef6a33d989f`。默认关，不进飞行环，未提交权重；可用教师集很薄） |
| P1 | 推理加速 | 无 | done（#48，合入 `80f375baae8d`。默认仍是逐位相同的 inference_mode；YOPO_INFER_ACCEL 保持关） |
| 组合 | E-A1、D1、E-B2 三个开关一起只报数 | E-A1, D1, E-B2 | done（#52，合入 `54407cb153ee`。开关仍默认关，不冻结） |
| 组合-六机 | 六机林带上 E-A1、D1、E-B2 三个开关一起只报数 | E-A1, D1, E-B2 | done（#56，合入 `026cad288b6d`。开关仍默认关，不冻结） |
| E-B2p | 后段提速，多数调用能在 0.02 s 预算内收敛 | E-B2 | done（#58，合入 `f05eac028893`） |
| E-B4 | 机间碰撞诊断，只报数 | 无 | done（#66，合入 `f5b668ff8cb6`） |
| E-B4b | 无解时从参考头刹停（empty_brake，默认关） | E-B4 | done（#77，合入 `d6d00c386f3f`） |
| E-C4 八机补跑 | 八机开阔对向 seeds 1–9，只报数 | E-C4 | done（#54，合入 `07bd3b1eda93`） |
| E-C4 密度 L0 | 密度场 L0 seed 0，只报数 | E-C4 | done（#55，合入 `72450291481c`） |
| E-C4 密度 L0 补跑 | 密度场 L0 seeds 1–9 与失败归因，只报数 | E-C4 | done（#59，合入 `cc6f5f7800be`） |
| E-C4b | 补跑密度场 combo 臂 seeds 0–9，只报数 | E-C4 | done（#78，合入 `c0c4623e40d8`） |
| 走廊半径 | 走廊 safe_radius 对照，只报数 | 无 | done（#75，合入 `ae1ab82a302a`。票号写在 [ego-swarm-comparison.md](research/ego-swarm-comparison.md) 第 5 节。`tests/test_e_c5_corridor_radius.py` 断言本文件不含该票号，所以这里不重复） |
| D2b | 加密度 0 / 0.14 / 0.28 的 EGO-Swarm 教师轨迹（中途收口） | D2 | done（#60，合入 `ea62b302b048`） |
| D2c | 登记 D2b 补齐后的全量教师数据清单 | D2b | done（#61，合入 `a30eb2b6b290`） |
| D4 | 学生干路加邻机输入，默认关 | D3 | done（#57，合入 `34d3ed52f421`） |
| D5 | 训练学生网络并接到 L0 闭环，默认关 | D4 | done（#62，合入 `737d88e28a45`） |
| D5b | 学生闭环接口修正与改训练，默认关，report-only | D5 | done（#69，合入 `7520c62197f4`） |
| D6 | 教师飞行高度扩到 {1.0, 1.5} 的数据规划与 z 平移可行性，report-only | D5b | done（#74，合入 `314ee0566f22`） |
| S1 | box 上 headless 单机 PX4 SITL + Gazebo 冒烟，report-only | 无 | done（#63，合入 `8f5608d01071`） |
| S1b | PX4 SITL 采用 RTK 厘米级定位源，默认关，report-only | S1 | done（#64，合入 `4dcc55c63521`） |
| S2 | 三机 PX4 SITL + Gazebo headless 骨架，report-only | S1 | done（#73，合入 `7b5c58b9d4ea`） |
| C1 | regress.sh 提速（pytest 并行、批跑 jobs 按 nproc、分段耗时表） | 无 | done（#71，合入 `056d9ae582be`） |
| C2 | scripts/status.sh 只读状态快照 | 无 | done（#67，合入 `438cc313ebc9`） |
| C3 | scripts/pr_summary.py 从 regress 输出生成 PR 说明 | 无 | done（#68，合入 `42936cf8b66f`） |
| C5 | premerge 已验树缓存（record-verified-tree.sh） | 无 | done（#72，合入 `59bae63b6160`） |
| V1 | E-B4 修复前后回放动图脚本，只报数 | E-B4 | done（#70，合入 `468c869f355c`） |
| S-RFLYSIM | Ubuntu 22.04 Humble ROS 2 锁步桥接与 status 动态远端解析 | 10, C2 | done（合入 `6ce80332a666`，见 [ros2-rflysim-humble.md](research/ros2-rflysim-humble.md)） |
| B8 | 柱净空门全拒退路地板与起步斜坡组合，默认关，只报数 | B7 | done（合入 `2e2b67041685`，见 [b8-fallback-floor.md](research/b8-fallback-floor.md)） |
| E-PG5 | 六机林带短前缀按 `_forest_six()` 作用域隔离与相对队形 p95，默认关，只报数 | 13e | done（合入 `afe44dd642b8`，见 [e-pg5-forest-six-prefix.md](research/e-pg5-forest-six-prefix.md)） |
| S8 | 三机林带多圆柱世界与六机 M4 锁步 PX4 SITL 抽检，report-only | S2 | done（合入 `41a5894f0825`，见 [s8-forest-and-m4-sitl.md](research/s8-forest-and-m4-sitl.md)） |
| CLOSEOUT | 收口刷新 M0–M4 结果总表、交付视频索引、已知限制与研究目录索引 | S-RFLYSIM, B8, E-PG5, S8 | done（本票） |
| EXPLORE | 无编队水平横排起飞 `ExploreFleet`（`policy=explore`）：无进度门控、无轮转、无槽位回拉，TJU 时间相似律 ratio=1.7，6 机密林 9.67 m/s，只报数 | CLOSEOUT | done（合入 `bdfbdfebcd0b`，见 [explore-free-swarm.md](research/explore-free-swarm.md)） |
| EXPLORE-SITL | `explore` 接入锁步 PX4 + Gazebo SITL（不写 PX4 参数），六机林带 4/5、三机柱林 3/3，只报数 | EXPLORE, S8 | done（合入 `992f7e4dafaf`，见 [explore-sitl.md](research/explore-sitl.md)） |

12：交付包收口。12a README 一键命令（#33，`78f63bcdb9bc`）；12b 结果总表（#36，`252bae86781e`）；12c 视频索引（#35，`cf9c0b0b9832`）；12d 已知限制（#34，`7a545ac3a645`）；12e 上游策略 vs 影子盾（#37，`0bb043cae739`）；12f 本票：README 顶部增加交付包入口，并核验所有表格带层级和 commit。干净检出复现由 12a 的 `tests/test_demo.py` 覆盖。

E 票：来自 `docs/research/ego-swarm-comparison.md`（#39，合入 `47a3041474a6`）。E-C1（#42，`ad20920c8c24`）、E-C2（#40，`4c17a8d0b60f`）、E-C3（#41，`0d7e138f7a80`）、E-A1（#45，`e07d93659428`）、E-B1 与 E-B2（#47，`616909b78e6c`）、E-B2p（#58，`f05eac028893`）、E-B3（#65，`f5cedba4540c`）、E-B3c（#76，`41e820f2ed63`）、E-B4（#66，`f5b668ff8cb6`）、E-B4b（#77，`d6d00c386f3f`）、E-C4（#50，`c958a3e87fed`）、E-C4 的后续只报数（#54 `07bd3b1eda93`、#55 `72450291481c`、#59 `cc6f5f7800be`）、E-C4b（#78，`c0c4623e40d8`）、走廊 safe_radius 对照（#75，`ae1ab82a302a`）已合。E-B3 实现线收口见 [e-b3-closeout.md](research/e-b3-closeout.md)（本票）：不再开后续调参或叠加欧氏铰链的实现票。已冻门不动。E-A1、D1、E-B2 三个开关一起只报数（#52，`54407cb153ee`；六机林带 #56，`026cad288b6d`）：开关仍默认关，不冻结。

D / P 票：D1（#46，`14dc25a8dab8`，默认关）、D2（#49，`be734163b85b`。密度 0.42：单机 9/16，三机与六机 0/16）、D2b（#60，`ea62b302b048`）、D2c（#61，`a30eb2b6b290`）、D3（#51，`7ef6a33d989f`。默认关的学生脚手架，不进飞行环，未提交权重；可用教师集很薄）、D4（#57，`34d3ed52f421`）、D5（#62，`737d88e28a45`）、D5b（#69，`7520c62197f4`）、D6（#74，`314ee0566f22`）、P1（#48，`80f375baae8d`。默认仍是逐位相同的 inference_mode，YOPO_INFER_ACCEL 保持关）已合。

S / C / V：S1（#63，`8f5608d01071`）、S1b（#64，`4dcc55c63521`）、S2（#73，`7b5c58b9d4ea`）、C1（#71，`056d9ae582be`）、C2（#67，`438cc313ebc9`）、C3（#68，`42936cf8b66f`）、C5（#72，`59bae63b6160`）、V1（#70，`468c869f355c`）已合。这些是只报数或工具脚本，不改已冻门。

06b：同步出发那一轮收不齐 ACK 时，本机在 t_start 超时 RELEASE，并按 resend_period 重发 RELEASE/COMMIT。下一轮自己的窗口重新提案，沿本槽弧长追到当前时刻表（速度/加速度不超过计划限幅），只飞本机已提交的计划。随机丢包+乱序只报数。

06c：06b 之后 drop 20% 的到达率仍低，是因为提案和 ACK 在窗口内不重传（COMMIT/RELEASE 才按窗口长重发）。六机一轮要 5 张票，单次成功概率约 0.8^10。错过前几窗之后追赶拟合也不再产出轨迹，有的飞机 `local_commits=0` 停在起点；晚提交的轨迹终点落在 40 s 之后，到不了。修复在 `FormationFleet.exchange`：同一 reservation 在截止前按 0.02 s 重发，静默 0.10 s。06c 本身不改 vendored 协议，也不改单机规划和 L0 跟踪参数。判据未冻结（report-only）。数字在 `docs/results/06c-drop20.md`。票 13d 合入时票齐后不再重发，免得 Delay Check 把同一份提案再送出去。

13d：在 ACK/COMMIT 上增加 PROPOSED → TENTATIVE（已广播）→ DELAY_CHECK → COMMITTED。编队 δ_DC = 0.05 s。Delay Check 期间收到冲突或迟到的邻机轨迹就作废新轨，已提交的安全轨留下，本窗口还能再提。穿林和变阵的 `delay_check_s` 保持 0，不换 YOPO，不引入 Gurobi。偏离写在 ADR 0012。数字在 `docs/results/13d-dual-track.md`。

09：Orin 工单由 `scripts/l1_spotcheck.sh --prepare-only --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar` 写到 `out/l1/g2-pillar/job-card.json`（schema `yopo-restart/l1-job-card/1`）。全量命令是 `YOPO_SWARM_ROOT=<yopo-swarm> PX4_PARAM_DUMP_CMD='<只读 param show>' scripts/l1_spotcheck.sh --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar`。脚本内部执行 `YOPO_SIM_SEED=<seed> bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal`，超时 1800 s。产物是 `bundle/scene.json`、`bundle/run.json`、`log/orin-result.json`、`log/events/`、`log/result.json`、`guard/` 里的参数表和安全文件哈希。过：`run_g2.sh` 退出码 0，参数表和安全哈希前后一致，且收下了 `result.json`。不过：退出码非 0、参数或安全文件有差异、没有 `result.json`，或命令里出现 param set/save/load/reset/import。已知缺口：这条路径仍是已部署的 certified 栈，不接受 `--policy upstream`；对照表写 `policy=deployed-certified`。把 upstream 移植进 L1 要等以后的 yopo-swarm 票，本仓库不改那个仓库。宿主上不要 `make venv`（aarch64 没有 torch cpu wheel）；export/import/report 只用 numpy，缺解释器时用已有镜像 `yopo-swarm:orin-infer-cu13`。

可并行前沿：01 → 02；02 后 03 / 04 / 05 可并行；06 后 07 / 10 可并行；09 在 03、04 完成后可与多机线并行。

每张票的验收清单见 docs/SPEC.md 对应小节。所有票都通过同一接缝验收：场景卡 → 运行器 → 结果。

07：三机穿柱林。每机用本机深度跑上游 YOPO，编队槽位只作为目标偏置，避障优先；运动更新仍走轮转窗口和预留，只飞本机 COMMIT 的计划。判据未冻结（report-only），碰撞数照实入表。

07b：同一套规划，换成泊松盘树干林。`forest.density` 选 sparse / medium / dense。密档若碰撞或到不了，照实写进结果表，不改规划器去掩盖。

07c：只定位密档 uav0 撞树。不改上游选择，也不改票 07 的规划参数。类别和撞前 1 s 的表在 `docs/results/07c-dense-diag.md`。

07d：修法 1 做成 `safe_radius` 开关。场景卡 `policy_params.upstream.safe_radius` 或 CLI `--safe-radius`。默认 0.05 m（上游 d5e1e60）。0.35 m 才是修法。结果表标出半径，非 0.05 m 写「偏离上游」。不改 L0 跟踪增益。疏/中/密对照在 `docs/results/07d-safe-radius.md`。

11c：#18 的六机 medium 有 6 个 seed 撞树，sparse 的 seed 2 也撞。首撞都是已提交的计划。主因是轮转 1.5 s 长于承诺地平线 1.0 s，末端急刹按 a_max=80 生成，动力学 a_max=6 停不住；同时槽位时刻表把目标线段拉进树干。林带里改为前方 goto，侧向让到净空够的位置，前缀盖住下一轮窗口，进树的多项式不提交。不改 safe_radius 0.05 m，也不改 kp / kd / tau / a_max。前后表在 `docs/results/11c-six-forest.md`。判据仍是 report-only。

13b：穿林虚拟中心按全队沿中心投影进度的 min 或低分位推进（`progress_percentile` 默认 0，即 min），不再用墙钟 × speed。落后机门控；超前机只提案悬停或短前缀。开阔区 `FormationFleet` 明确 `progress_gate=false`，时刻表仍是开环。场景卡可写这两个字段。决定是 ADR 0013（0011 留给 13a，0012 留给 13d，0014 留给 13c）。六机 medium / sparse seeds 0–9 的前后表在 `docs/results/13b-progress-gate.md`。判据仍是 report-only。不换 YOPO，不引入外部求解器，不改 yopo-swarm。

13e：13b 叠在 13c / 13a 上之后，六机穿林 medium 0/60、sparse 9/60。根因是提案前自检拿邻机全部历史计划来比，被替换计划的刹车停点一直占到 inf，邻机投票却只看 committed + previous 和预留；超前机悬停留下的停点堵死落后机，门控取 min 又让超前机继续悬停。六机林带自检改为每个邻机只看最后两份计划加预留，与投票口径一致；三机和柱林仍走 `conflicts_with_neighbors`。六机林带卡写 `progress_gate=false`，门控代码和开关保留。决定是 ADR 0015。各修法对照表在 `docs/results/13e-ghost-stops.md`。判据仍是 report-only。不改 yopo-swarm，不改 vendored core。
11 收口：M0–M4 冻结建议表写在 `docs/results/11-freeze-recommendation.md`，决定记在 ADR 0016。数字来自 main `8ee3ca1`（票 13e）和建议票在同一 sha 上的重跑，不是编的。frank 2026-11-10 批准全部建议，M2 队形门用 6.0 m（不用 5.0 m），到达率保持 100%。已冻的是到达满格、碰撞 = 0，以及 M1/M3/M4 开阔区的相对槽位 p95 和机间距离、M2 柱林净空、M0 到达时间、六机林带的机间距离（≥ 0.70 m）。继续只报数的是墙钟、certified 对照、三机林 sparse/dense、三机林 medium 的数值净空、六机林带的两种 p95 和数值净空、M4 的 ROS 2 全矩阵、M3 的 L1 抽检。对应场景卡已设 `criteria.frozen=true` 和 `criteria.gates`；冻结指标失败时批跑退出非 0。


13f：把 `crossing-6-forest.json@crossing@0` 和 `crossing-6-forest-sparse.json@crossing@0` 写进 `scenes/suite.txt`，合入前回归不再漏跑这两张卡。13e 上 10 seed 已是 60/60、碰撞 0。单次墙钟约 216 s / 156 s，超过 60 s，按 07b 惯例只跑 seed 0；10 seed 全量仍用 README 里的批跑命令。票 11d 已冻到达满格、碰撞 = 0、最小机间距离 ≥ 0.70 m；两种 p95 和数值净空仍只报数。

13c：六机林带里槽位改成软目标。侧向在 `lateral_slack_m`（默认 2 m）内、纵向在 `longitudinal_slack_m`（默认等于 `min_ahead_m`）内松弛，只挑整段净空够的射线；出林前 `exit_tighten_m`（默认 4 m）把松弛收到 0，出林后回到原来的刚性槽位前瞻。避障是硬的：COMMIT 前对轨迹按 0.02 s 采样，中心到树的最小距离必须 ≥ `body_radius + clearance_margin_m`（默认 margin 0.15 m，场景卡 `task.clearance_margin_m` 可改）。不够记 `CLEARANCE_SKIP`，不提案、不提交。这是过滤，不进优化器，也不引入 CPLEX。只对带森林的六机生效。不改 `safe_radius` 默认 0.05 m，不改 kp / kd / tau / a_max，也不改 yopo-swarm。决定记在 ADR 0014。前后表在 `docs/results/13c-elastic-clearance.md`。判据仍是 report-only。

08：一种变阵，横排 ↔ 纵列。开阔区中间有一道窄门（两根柱子）。各机用场景几何量前方走廊净空，低于阈值就在自己的窗口里提案切到纵列槽位，出了窄门再提案切回横排。新计划从父计划在 t_start 的状态接上，经 ACK/COMMIT 才飞，不瞬移。失败拒绝码 `SWITCH:NO_ACK`、`SWITCH:CONFLICT`。判据未冻结（report-only）。过渡期碰撞是硬失败，照实入表。
