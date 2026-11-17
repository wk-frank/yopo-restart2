# CTBR-CENTER900：中心起飞、全域障碍与六机长程探索

状态：实现与验证中；report-only，判据未冻结。基线为 `8b470d3022aca094f46b75d4fa7f3ae2bb99dd3a`。

## 已确认的任务口径

- 六架飞机从地图中心成组、同方向向外探索，避障时允许队形伸缩。
- 900 m 指每架飞机相对各自起点的水平直线位移，不是累计航程，也不是六机合计。
- 地图为 2200 m × 2200 m，障碍覆盖前后左右全部 55 × 55 个 40 m 网格；中心只留半径 18 m 的安全起飞区。每个网格仍有障碍，不清空航路或终点走廊。
- 必须使用实际 YOPO 网络与 PX4/Gazebo SITL 的 CTBR 控制，并保留全程里程计、Gazebo 真值、控制模式与指令记录。SITL 是仿真，不代表实体飞机实飞。
- 本轮按用户选择由主代理本地推进；不以可用的新模型替代偏好中缺失的子代理模型。

## 实现边界

独立场景卡为 [explore-6-center900-ctbr.json](../../scenes/explore-6-center900-ctbr.json)，障碍由 `yopo_l0/wide_scene.py` 按种子生成。目标 X=940 m，终点保持后全机降落。弹性协同通过已收到的邻机轨迹限制前后差，并给横向偏离初始槽位增加软代价；这不保证刚性队形。

CTBR 位于 `yopo_l0/ctbr.py`，仅显式 `--control-mode ctbr --lockstep --policy explore` 启用。起飞使用原有 PVA；悬停时用只读 uORB listener 取得实际推力；PLAN 与减速使用 50 Hz 机体角速率及集体推力；在目标附近停稳后切回 PVA，降落使用 NAV_LAND。

平坦度前馈用完整 `R_d` 的解析导数，包含倾斜与航向耦合。原 CTBR-1 设计中的偏航速率简式不足以通过非零 jerk、非零偏航速率的数值微分核对，因此实现采用完整导数。

首轮短程检查揭示：`ARRIVED_HOLD` 同时表示已提交减速轨迹，不能用它直接触发 PVA 切换。切换现以实测位置和速度为准并锁存。新增 CTBR 的 PLAN 地面碰撞判定，避免原“只统计离地样本”的逻辑漏报贴地事件。

大场景传感器加速通过显式 `YOPO_SENSOR_SPATIAL=1 YOPO_SENSOR_ACCEL=1` 启用。空间筛选使用保守视距球，保留相机噪声随机流；距离场用全数组宽相位筛选远处障碍，不删除世界中的模型。

`core/`、冻结套件与既有场景卡保持不变；控制模式默认 `pva`；所有运行记录必须满足 `px4_parameters_written == []`。

## 运行命令

```bash
YOPO_SITL_DOCKER_NETWORK=bridge \
YOPO_SITL_WALL_TIMEOUT=3600 \
YOPO_SENSOR_SPATIAL=1 YOPO_SENSOR_ACCEL=1 \
flock /tmp/yopo-sitl.lock scripts/sitl/run_yopo_l0_sitl.sh \
  --scene scenes/explore-6-center900-ctbr.json --n 6 --policy explore \
  --lockstep --scene-world --control-mode ctbr --record-dt 0.2 --seed 0
```

结果目录由启动脚本打印。`result.json` 为最终结果，`progress.json` 为运行中进度；`flight_record` 保存每机 5 Hz 的位置、速度、参考、控制模式及 CTBR 指令。回放必须由记录生成，不以预设动画替代实际轨迹。

## 验证记录

测试与最终飞行统计将在运行结束后补充。未完成 900 m 验证前，不以场景目标距离宣称验收达标。
