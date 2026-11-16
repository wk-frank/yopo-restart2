# M0–M4 结果总表

数字全部来自下面列出的批跑结果，对应 commit `abde0881f993`。没有从冻结建议表抄数，也没有编数。缺的行只在「本批未覆盖」里说明，不填数。结果记录里的 commit 是跑那一次时的 HEAD；若后来只合进文档或脚本、没有改 `yopo_l0` 或场景卡，这些数字对当前 main 仍然有效。
层级默认 L0，策略默认 `upstream` + 影子盾开。冻结口径见 [ADR 0016](../adr/0016-m0-m4-freeze-recommendation.md) 和场景卡 `criteria.frozen` / `criteria.gates`。已冻指标失败时批跑退出非 0；未写入 gates 的指标只报数，不挡退出码。碰撞数始终作为硬事实入表。已知限制见 [LIMITATIONS.md](../LIMITATIONS.md)，交付视频索引见 [videos.md](videos.md)。

输入：

- `out/m5/core/results.json`
- `out/m5/m2-forest/results.json`
- `out/m5/m4-open/results.json`
- `out/m5/m4-forest/results.json`

## 总表

| 里程碑 | 场景卡 | 策略 | 层级 | 形态 / 档位 | 影子盾 | commit | 场景卡哈希 | seeds | 到达 | 碰撞 | 队形误差 p95 m | 最小机间距离 m | 最小净空 m | 规划墙钟 p95 ms（只报数） | 状态 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| M0 | g2-pillar | upstream | L0 | 进程内 | 开 | `abde0881f993` | `9d385f4cfce5` | 0–9 | 10/10 | 0 | — | — | 0.477 | 28.100–40.342 | 判据已冻结（最小净空只报数） |
| M1 | formation-hold | formation | L0 | 进程内 | 开 | `abde0881f993` | `1b1894f5864a` | 0–2 | 9/9 | 0 | 0.027 | 1.987 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M2 | crossing-pillars | crossing | L0 | 进程内 | 开 | `abde0881f993` | `1d39afbd7807` | 0–9 | 30/30 | 0 | 4.130–4.610 | 3.350 | 0.528 | 21.047–44.206 | 判据已冻结（到达时间只报数） |
| M2 | crossing-forest（medium） | crossing | L0 | 进程内 | 开 | `abde0881f993` | `e66b39061158` | 0–9 | 30/30 | 0 | 4.094–4.466 | 4.602 | 0.260 | 21.736–28.494 | 判据已冻结（到达时间、最小净空只报数） |
| M3 | formation-switch | switch | L0 | 进程内 | 开 | `abde0881f993` | `4bff8a51c881` | 0–9 | 30/30 | 0 | 前 0.025 / 后 0.025（全程 0.036） | 1.536（过渡 1.536） | 0.693 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | formation-6-open | formation | L0 | 进程内 / none | 开 | `abde0881f993` | `a803d8021c77` | 0–4 | 30/30 | 0 | 0.066 | 2.980 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | formation-6-open | formation | L0 | 进程内 / drop-5 | 开 | `abde0881f993` | `a803d8021c77` | 0–4 | 30/30 | 0 | 0.066 | 2.980 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | formation-6-open | formation | L0 | 进程内 / drop-20 | 开 | `abde0881f993` | `a803d8021c77` | 0–4 | 30/30 | 0 | 0.066 | 2.980 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | formation-6-open | formation | L0 | 进程内 / duplicate-10 | 开 | `abde0881f993` | `a803d8021c77` | 0–4 | 30/30 | 0 | 0.066 | 2.980 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | formation-6-open | formation | L0 | 进程内 / reorder-delay | 开 | `abde0881f993` | `a803d8021c77` | 0–4 | 30/30 | 0 | 0.066 | 2.980 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | formation-6-open | formation | L0 | 进程内 / combined | 开 | `abde0881f993` | `a803d8021c77` | 0–4 | 30/30 | 0 | 0.066 | 2.980 | 离地 1.150 | — | 判据已冻结（到达时间、最小净空只报数） |
| M4 | crossing-6-forest（medium） | crossing | L0 | 进程内 | 开 | `abde0881f993` | `657efeb6fae1` | 0–9 | 60/60 | 0 | 23.300–26.108 | 1.016 | 0.126 | 24.670–31.573 | 判据已冻结（到达时间、队形误差 p95、最小净空只报数） |
| M4 | crossing-6-forest-sparse（sparse） | crossing | L0 | 进程内 | 开 | `abde0881f993` | `7e3328908444` | 0–9 | 60/60 | 0 | 18.725–22.906 | 0.959 | 0.158 | 20.964–29.817 | 判据已冻结（到达时间、队形误差 p95、最小净空只报数） |

队形误差 p95 取结果里的 `formation_error_p95_m`（官方口径）；多个 seed 写出最小到最大。最小机间距离和最小净空取各 seed 里最紧的值。净空 = 机体中心到障碍表面的距离 − 0.35 m。开阔区的净空是离地，不是避障净空，只报数。规划墙钟是每次网络前向（或后段优化）的墙钟 p95，单位毫秒，只报数，不进 gates；没有这一字段的旧结果写 —。整回路墙钟 `wall_time_s` 随负载变化，含光线投射和动力学，不进这张表，也不进 gates。

## 本批未覆盖（不填数）

| 里程碑 | 场景 | 说明 |
|---|---|---|
| M0 | g2-pillar（certified，10 seed 全程） | 继续只报数。本批没有收齐全 10 个 seed 的全程结果，不填数。合入前回归里的 certified 冒烟是 4 s 前缀，不能代替全程。 |
| M2 | crossing-forest（sparse / dense） | 继续只报数；本批结果没有这两档，不填数。 |
| M3 | formation-switch（L1 抽检） | 票 09 已关闭，没有新方案对照。继续只报数。 |
| M4 | formation-6-open（ROS 2 全矩阵） | 继续只报数；本批结果没有 ROS 2 形态，全矩阵另开票。 |
| M4 | 六机柱林 | 未测，不进套件。继续只报数。 |

## 复现

```bash
git checkout abde0881f993a1236cda0419a7d02db0bd3b889a
make venv
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/formation-hold.json@formation scenes/crossing-pillars.json@crossing scenes/formation-switch.json@switch --jobs 4 --out out/m5/core
.venv/bin/python -m yopo_l0.batch scenes/crossing-forest.json@crossing --jobs 4 --out out/m5/m2-forest
.venv/bin/python -m yopo_l0.batch scenes/formation-6-open.json@formation --form inprocess --tiers scenes/formation-6-tiers.json --jobs 4 --out out/m5/m4-open
.venv/bin/python -m yopo_l0.batch scenes/crossing-6-forest.json@crossing scenes/crossing-6-forest-sparse.json@crossing --jobs 4 --out out/m5/m4-forest
.venv/bin/python -m yopo_l0.summary out/m5/core/results.json out/m5/m2-forest/results.json out/m5/m4-open/results.json out/m5/m4-forest --md docs/results/SUMMARY.md --json docs/results/SUMMARY.json
```

生成时间：2026-11-16 10:56（America/La_Paz，UTC-4）。
