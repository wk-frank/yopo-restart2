# 票 S-RFLYSIM：RflySim-20.04（Ubuntu 22.04 Humble）ROS 2 锁步桥接、status 动态仓库与 L1 边界核查

只报数。report-only。本记录覆盖 `RflySim-20.04`（Ubuntu 22.04.5 Jammy，`/opt/ros/humble`）下的三项交付：

1. [status.sh](../../scripts/status.sh) 动态解析 `origin` 远端仓库 slug（如 `wk-frank/yopo-restart2`），无 `origin` 或本地裸路径时回退 `unununnnn/yopo-restart`。
2. [ros2_env.py](../../yopo_l0/ros2_env.py)、[ros2_run.py](../../yopo_l0/ros2_run.py)、[setup_ros2.sh](../../scripts/setup_ros2.sh) 与 `tests/test_ros2_*.py` 在 `.venv`（Python 3.12，`numpy==2.5.4`，`torch==2.4.1+cpu`）与系统 `/opt/ros/humble`（CPython 3.10 C 扩展）之间建立真实 ROS 2 Humble 子进程桥接，零跳过跑通全部 ROS 2 锁步对账测试，且不改动 `core/`、场景卡或默认策略开关。
3. 只读核查 `yopo_l1/` 与 `/mnt/e/rflysimtools/RflySimAPIs/RflySimSDK`，确认仓库内不存在 RflySim L1 工单或适配器，且按 [SPEC.md](../SPEC.md) 属于范围外。

## 1. Python 3.12 `.venv` ↔ CPython 3.10 Humble `rclpy` 桥接机制

### 根因

- 仓库 `.venv` 固定 Python 3.12 + `numpy==2.5.4` + `torch==2.4.1+cpu`（见 `Makefile` 与 `core/VENDORED.md`；系统 Python 3.10 的 `numpy 1.26.4` 与 `torch 2.8.0+cpu` 在第 0 个 tick 即与 3.12 产生 `1e-20` 级浮点尾差，导致逐位对账 `proc.ticks == ros.ticks` 失败）。
- `RflySim-20.04`（Ubuntu 22.04 Jammy）的 `/opt/ros/humble` 全部 `rclpy` 与 `rosidl` C 扩展编译目标为 `cpython-310-x86_64-linux-gnu.so`，无法直接在 Python 3.12 进程内 `dlopen`。
- 此外，Humble 原生 `rclpy` 相对 Jazzy 存在两处薄接口差异：无 `rclpy.clock_type` 子模块（枚举位于 `rclpy.clock.ClockType`），且 `Subscription` 无 `get_publisher_count()`（需由所属 `Node.count_publishers(topic_name)` 查询）。

### 最小非侵入设计

在不触碰 [ros2_nodes.py](../../yopo_l0/ros2_nodes.py) 与 `core/` 的前提下，由 [ros2_env.py](../../yopo_l0/ros2_env.py) 的 `ensure_ros_paths()` 自动分派：

- **同 ABI 原生路径**（Ubuntu 24.04 Jazzy + Python 3.12，或 Ubuntu 22.04 Humble + 系统 Python 3.10）：仅将与当前解释器主次版本一致的 `/opt/ros/<distro>` Python 路径并入 `sys.path`，并在 Humble 下补齐 `rclpy.clock_type` 与 `Subscription.get_publisher_count()` 兼容层。
- **跨 ABI Humble 桥接路径**（Ubuntu 22.04 Humble + `.venv` Python 3.12）：
  - 不把 `python3.10` 路径混入 Python 3.12 的 `sys.path` / `PYTHONPATH`，保证 `.venv` 内 `numpy 2.5.4` 与 `torch 2.4.1+cpu` 纯净。
  - 在 `rclpy.init()` 时通过独立匿名管道（`os.pipe()`，与 `stdout`/`stderr` 隔离，避免 FastDDS 日志干扰二进制帧）拉起 `/usr/bin/python3.10` 辅助进程，真实运行 `/opt/ros/humble` 的 `Node`、`SingleThreadedExecutor`、`/clock`（`use_sim_time=True` / `ClockType.ROS_TIME`）、`/{id}/observation`、`/{id}/plan` 及三跳机间总线 `/{src}/bus_tx → /{dest}/bus_rx → /{dest}/bus_done`。
  - 每次 `spin_once` 同步返回各节点真实 `ROS_TIME` 纳秒读数与话题回调载荷，规划器 `policy.plan(obs)` 与动力学积分仍在 `.venv` Python 3.12 进程内执行，从而同时满足真实 ROS 2 Humble 话题收发与 `proc.ticks == ros.ticks` 逐位一致。
- **并发 `pytest -n 4` 域隔离与 RMW 选型**：
  - [test_ros2_form.py](../../tests/test_ros2_form.py) 在调用方未显式指定 `ROS_DOMAIN_ID` 时，按 `PYTEST_XDIST_WORKER`（或单进程 PID）分配独立 `ROS_DOMAIN_ID`（`40..89`），避免多 worker 共享 `/clock` 与 `/uav*/observation` 串扰。
  - Humble 默认的 FastDDS 2.6（`rmw_fastrtps_cpp`）在多进程高 CPU 争用下，同进程 `KEEP_LAST(64)` 深度分片突发发布偶发丢弃 `header` 首分片；当本机存在 `librmw_cyclonedds_cpp.so` 且调用方未显式指定 `RMW_IMPLEMENTATION` 时，[ros2_env.py](../../yopo_l0/ros2_env.py) 默认选用 `rmw_cyclonedds_cpp`（[setup_ros2.sh](../../scripts/setup_ros2.sh) 在 Humble 上同步列入 `ros-humble-rmw-cyclonedds-cpp`），连续 3 轮 `-n 4` 并发全量压测 100% 稳定零丢片。

## 2. `scripts/status.sh` 动态仓库解析

- [status.sh](../../scripts/status.sh) 新增 `resolve_gh_repo`：优先读取 `YOPO_GH_REPO`；未设置时以 `GIT_OPTIONAL_LOCKS=0 git -C <dir> remote get-url origin` 只读解析 HTTPS / SSH 远端 URL（支持 `.git` 后缀、`git@host:owner/repo`、`ssh://`、`https://`），在当前 `wk-frank/yopo-restart2` 检出与工作树下自动解析为 `wk-frank/yopo-restart2`；非 git 目录或无法提取 `owner/repo` 时干净回退至 `unununnnn/yopo-restart`。
- [test_status_sh.py](../../tests/test_status_sh.py) 覆盖 HTTPS、SSH、默认未设 `YOPO_REPO_DIR` 检出目录、本地路径回退与目录快照完全只读断言（9/9 通过）。

## 3. 验证结果（`RflySim-20.04`）

| 测试项 | 命令 | 结果 |
|---|---|---|
| ROS 2 环境脚本 | `./scripts/setup_ros2.sh` | `ROS 2 humble 已安装` + `.venv 可以 import rclpy。`（退出码 `0`） |
| 状态脚本单测 | `.venv/bin/python -m pytest -q tests/test_status_sh.py` | `9 passed, 0 skipped` |
| ROS 2 全量套件（含 Humble 桥接单测，`-n 4`） | `.venv/bin/python -m pytest -v -n 4 tests/test_ros2_*.py` | `29 passed, 0 skipped, 0 failed` |
| ROS 2 全量对账明细 | `open-3drone@hover`、`g2-pillar@upstream`、`formation-hold`（seeds 0/1/2 + drop/reorder + CLI + bus_topics）、`formation-6-open`（seed 0 + batch）、`crossing-6-forest`（seed 0 前缀） | `clock_ok == True`，`proc.ticks == ros.ticks`，0 碰撞 |

## 4. `yopo_l1/` 与 `RflySimSDK` 边界核查（只读）

- **`yopo_l1/` 现状**：检查 [jobcard.py](../../yopo_l1/jobcard.py)、[launch_contract.py](../../yopo_l1/launch_contract.py)、[export.py](../../yopo_l1/export.py)、[importer.py](../../yopo_l1/importer.py)、[collect.py](../../yopo_l1/collect.py)、[report.py](../../yopo_l1/report.py)，`yopo_l1/` 内**不存在任何 RflySim 工单（jobcard）或适配器**。现有 `yopo_l1` 仅面向早期 Orin `yopo-swarm` 部署栈的 `run_g2`（及可选 `yopo_swarm_px4/uavs.launch.py`）抽检回收，`dynamics` 固定为 `"px4_sitl"`（对应票 `09`，已于 2026-11-09 关闭并由方向 B `scripts/sitl/*` + [sitl_bridge.py](../../yopo_l0/sitl_bridge.py) 接替）。仓库内唯一提及 `rflysim` 的位置是 [minco-port-map.md](../ref/minco-port-map.md) 的历史来源对照表。
- **`/mnt/e/rflysimtools/RflySimAPIs/RflySimSDK` 现状**：挂载目录下包含 Windows/WSL 跨网桥 UDP 控制与视觉接口（`ctrl/PX4MavCtrlV4ROS.py`、`vision/VisionCaptureApi.py`、`ue/UE4CtrlAPI.py`、`comm/rostrans/` 含 `x86_64-u22.04-ros2-humble` 预编译库），依赖 Windows 宿主前台运行 `RflySim3D.exe` 与 `CopterSim.exe` 并经 WSL2 默认网关 UDP `20010` 通信。
- **范围判定**：按 [SPEC.md](../SPEC.md) 第 6 行、第 84 行与「Out of Scope」（第 113–122 行），本项目 **L1** 明确定义为 **PX4 SITL + Gazebo 保真层**（现行锁步实现位于 `scripts/sitl/` 与 `yopo_l0/sitl_lockstep.py`），不开发渲染级外部仿真适配，因此新增 RflySim L1 jobcard/adapter 属于 **SPEC 范围外（out-of-scope）**。
