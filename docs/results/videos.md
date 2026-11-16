# 交付视频（票 12c）

视频不进仓库：`out/` 已在 `.gitignore`。成片写在 `out/delivery/`，本表指向这些路径。用 `scripts/render_delivery.sh` 在当前检出上重渲。
另有一份复制在 `/workspace/yopo-delivery-videos/`。

| 项 | 值 |
|---|---|
| 代码 commit | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| 层级 | L0 |
| seed | 0 |
| 播放时长上限 | 60 s |
| 渲染器 | `python -m yopo_l0.render`（票 04）。字幕叠里程碑、层级、commit、seed。 |

| 文件 | 时长 s | 大小 | 场景 | commit |
|---|---|---|---|---|
| `M0-g2-pillar-upstream-seed0.mp4` | 5.30 | 33.0 KiB | scenes/g2-pillar.json@upstream | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| `M1-formation-hold-seed0.mp4` | 19.60 | 68.3 KiB | scenes/formation-hold.json@formation | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| `M2-crossing-pillars-seed0.mp4` | 8.00 | 53.8 KiB | scenes/crossing-pillars.json@crossing | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| `M2-crossing-forest-medium-seed0.mp4` | 24.30 | 92.6 KiB | scenes/crossing-forest.json@crossing | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| `M3-formation-switch-seed0.mp4` | 28.00 | 110.8 KiB | scenes/formation-switch.json@switch | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| `M4-formation-6-open-seed0.mp4` | 21.10 | 93.2 KiB | scenes/formation-6-open.json@formation | `abde0881f993a1236cda0419a7d02db0bd3b889a` |
| `M4-crossing-6-forest-seed0.mp4` | 27.10 | 182.3 KiB | scenes/crossing-6-forest.json@crossing | `abde0881f993a1236cda0419a7d02db0bd3b889a` |

每条都是 seed 0、层级 L0。M2、M4 各有两张场景卡，各渲一条。
