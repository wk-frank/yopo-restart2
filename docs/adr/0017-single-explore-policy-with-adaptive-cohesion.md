---
Status: accepted
Date: 2026-11-18
Scope: 三机 Orin Nano 8GB 与 D435 实机编队避障策略
---

# 全程采用单一 Explore 策略加开阔度自适应凝聚（Formation 动态切换留作未来项）

编号是 0017。三机实机进入含障碍任务（阶段 F）时，全程运行 `ExploreFleet`（`policy=explore`），通过前方扇区深度开阔度动态插值 `cohesion.lateral_gain`（障碍密集时弱回归放开横向绕障，开阔区强回归拉回初始横排通道），不在当前阶段实现 `FormationFleet`（`policy=formation`）与 `ExploreFleet` 之间的空中运行时策略切换；跨策略动态交接仅作为未来扩展项登记。阶段 E 仍单独使用 `FormationFleet` 验证纯开阔区刚性槽位基线。

## Context

`FormationFleet`（[formation.py](../../yopo_l0/formation.py)）基于预积分虚拟中心弧长时刻表 `_CenterPath` 与四阶段预约投票协议 `ProtocolNode`，其启动与追赶拟合要求飞机静止在初始槽位原点，且不消费深度图。六机 900 m 原生深度与高密林交付（[gazebo-native-center900.md](../research/gazebo-native-center900.md)、[depth-only-dense-forest900.md](../research/depth-only-dense-forest900.md)）全程只运行 `ExploreFleet`，未调用 `FormationFleet`。

## Consequences

- 阶段 F 的实机与配对仿真代码路径同构（`scenes/explore-3-d435-center900.json`），无需在飞控桥中维护两套分布式状态机的空中交接。
- 开阔段与绕障段的横向收拢强度由 `ExploreFleet.effective_lateral_gain` 连续调节，避免策略切换瞬间产生加速度或偏航跳变。
