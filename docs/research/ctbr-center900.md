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

CTBR 位于 `yopo_l0/ctbr.py`，仅显式 `--control-mode ctbr --lockstep --policy explore` 启用。起飞使用原有 PVA；悬停时用只读 uORB listener 取得实际推力；PLAN 巡航与绕障使用仿真钟 50 Hz 机体角速率及集体推力；终点目标锁定后一次性切回 PVA，由 PX4 完成减速与保持，接管状态锁存，降落使用 NAV_LAND。

平坦度前馈用完整 `R_d` 的解析导数，包含倾斜与航向耦合。原 CTBR-1 设计中的偏航速率简式不足以通过非零 jerk、非零偏航速率的数值微分核对，因此实现采用完整导数。

首轮短程检查揭示：`ARRIVED_HOLD` 同时表示已提交减速轨迹，不能用它直接触发 PVA 切换。首版以实测位置和速度决定切换，在第一次 900 m 运行中仍出现终点持续 CTBR 纠偏与掉高；现改为终点目标锁定后 PVA 一次性接管并锁存。新增 CTBR 的 PLAN 地面碰撞判定，避免原“只统计离地样本”的逻辑漏报贴地事件。

大场景传感器加速通过显式 `YOPO_SENSOR_SPATIAL=1 YOPO_SENSOR_ACCEL=1` 启用。空间筛选使用保守视距球，保留相机噪声随机流；距离场用全数组宽相位筛选远处障碍，不删除世界中的模型。

`core/`、冻结套件与既有场景卡保持不变；控制模式默认 `pva`；所有运行记录必须满足 `px4_parameters_written == []`。

## 运行命令

```bash
# 可选加速运行时：复用既有 PX4/ROS 镜像，仅新增固定版本 OpenCV。
docker build --pull=false -t yopo-restart-sitl:ctbr-opencv-4.10 -f scripts/sitl/Dockerfile.ctbr scripts/sitl
YOPO_SITL_DOCKER_NETWORK=bridge \
YOPO_SITL_WALL_TIMEOUT=7200 \
YOPO_DEPTH_BACKEND=opencv YOPO_SENSOR_SPATIAL=1 YOPO_SENSOR_ACCEL=1 \
flock /tmp/yopo-sitl.lock scripts/sitl/run_yopo_l0_sitl.sh \
  --image yopo-restart-sitl:ctbr-opencv-4.10 \
  --scene scenes/explore-6-center900-ctbr.json --n 6 --policy explore \
  --lockstep --scene-world --control-mode ctbr --record-dt 0.2 --seed 0
```

结果目录由启动脚本打印。`result.json` 为最终结果，`progress.json` 为运行中进度；`flight_record` 保存每机 5 Hz 的位置、速度、参考、控制模式及 CTBR 指令。回放必须由记录生成，不以预设动画替代实际轨迹。

## 验证记录

- 当前受影响检查：`53 passed`，包括 CTBR 数学、模式切换、地图覆盖、传感器等价性、每机位移验收与既有桥接检查。
- 全库 L0 首次检查：1098 passed、6 failed、3 skipped。其中 5 个数值夹具失败在未修改的 `8b470d3` 独立副本中全部复现；另一个要求 Git 工作区干净，提交实现后重跑通过。核心包：452 passed、1 failed、3 skipped；失败为开发指南已记录的 `test_loss_terms_match_upstream_golden`，核心字节未修改。
- 冻结套件批跑通过。场景严格校验、文档链接、研究索引与一期收口核验通过；59 个核心钉定文件哈希一致。
- 60 m 新地图前缀检查：实际 PX4/Gazebo、六机 CTBR，6/6 到达并降落，仿真 49.10 s；Gazebo 各机水平最大位移 60.43–61.98 m，碰撞样本 0、缺失样本 0，最小机体表面净空 0.454 m、最小机间中心距 0.996 m，参数写入为空。该前缀只证明控制链路，不是 900 m 验收。
- 900 m 正式长程正在运行，目标速度为 5 m/s，实际判定以最终记录为准。该速度选择服务于本轮长距离安全验证，CTBR-1 原定的 8.5–10 m/s 同种子性能对照仍未完成。
- HTML 已做 JavaScript 语法与起点/中途/终点绘图逻辑检查；PNG 已目视核对。后台浏览器宿主不可用，未声称完成真实浏览器交互验收。


## 终点修复与深度预处理复验

第一次长程（`20261117T044315-yopo-n6`）六机均超过 900 m，但终点 CTBR 持续纠偏出现掉高，触发 `uav3:GROUND_COLLISION`，仅 3/6 被记为完成到达，完整任务失败。该记录保留，不作为成功结果。终点接管现以 `terminal_locked` 为边界，PVA 接管后不再因规划码变化切回 CTBR。

修复后 60 m 全场景检查（`20261117T060832-yopo-n6`）：6/6 到达并降落，仿真 35.88 s、墙钟 168.99 s，Gazebo 碰撞/缺失样本均为 0。900–940 m 终点区域检查（`20261117T061336-yopo-n6`）：6/6 到达并降落，仿真 31.14 s、墙钟 142.91 s，Gazebo 碰撞/缺失样本均为 0，最小机体表面净空 1.479 m、最小机间中心距 3.215 m。

性能剖析确认纯 Python NS 补洞为主要规划热点。在外围新增默认关闭的 `YOPO_DEPTH_BACKEND=opencv`，固定 `opencv-python-headless==4.10.0.84`，不修改 `core/`，YOPO 推理仍为原有单线程 fp32。8 张沿途深度帧与核心补洞逐像素一致（差异 0）；该次微基准核心耗时 0.638 s、OpenCV 耗时 0.099 s。该结果只描述深度准备，不是整机性能倍率承诺；实验结果明确记录后端，默认继续走核心实现。

复验还要求每机在 CTBR 控制样本中的 PX4 与 Gazebo 水平位移均超过 900 m，避免只短暂使用 CTBR 后以 PVA 飞完全程。
