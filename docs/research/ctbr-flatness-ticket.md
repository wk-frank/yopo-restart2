# 票 CTBR-1：微分平坦 CTBR（集体推力 + 机体角速率 `VehicleRatesSetpoint`）接缝与验收规范

状态：`todo`（仅立项与接缝规范，本票不合入控制器实现代码；后续实验属 `report-only` / 只报数，判据未冻结）。

## 1. 问题背景与定量动机（L0 敏捷上限 vs PX4 位置/速度外环相位滞后）

1. **L0 无编队水平横排上限已打通：** 在 [explore-free-swarm.md](explore-free-swarm.md) 中，`ExploreFleet`（`yopo_l0/explore.py`）去除进度门控、TDMA 轮转等待与横向槽位回拉，并在不触碰 `core/` 的前提下按 TJU YOPO 时间相似律（`ratio = 1.7`，`v_target = 10.2 m/s`，敏捷点质量动力学 `a_max ≈ 20 m/s²`）解码 5 阶 MINCO 候选轨迹。在 6 张水平横排场景卡 × 10 seeds（共 120 趟）上取得 **120/120（100%）到达、0 障碍碰撞、0 机间碰撞**，林带三维均速达 **9.67–9.91 m/s**（六机密林 `explore-6-forest-dense` 平均到达时间 **6.29 s**）。
2. **锁步 PX4 SITL 位置/速度外环（PVA `TrajectorySetpoint`）的速度与跟踪瓶颈：** 在 [explore-sitl.md](explore-sitl.md) 中，保持飞控原厂参数不写（`px4_parameters_written == []`），通过 `/px4_i/fmu/in/trajectory_setpoint` 下发 `(p, v, a, yaw)` 跑满 20 趟无头锁步 PX4 v1.17 + Gazebo Harmonic 实飞验证：
   - `explore-6-forest-sitl80`：到达 **4/5 趟（24/30 机）**（对照 [S8](s8-forest-and-m4-sitl.md) 编队穿林 `0/3` 趟），林带均速 **6.02–6.30 m/s**；
   - `explore-3-forest-sitl80`：林带均速 **7.41–7.58 m/s**（峰值 **8.08 m/s**，较 S8 提速 **1.68×**），到达 **4/6 趟**；
   - `explore-6-open-sitl80` 与 `explore-3-pillars-sitl80`：均为 **3/3（100%）零碰撞到达**（均速 **7.34 m/s** 与 **6.80 m/s**）。
3. **根因定位：** 在严格遵守 `px4_parameters_written == []`（不改写 `MPC_XY_VEL_MAX`、`MPC_ACC_HOR`、`MPC_XY_P`、`MPC_XY_VEL_P_ACC`）的前提下，PX4 级联 `Position -> Velocity -> Attitude` 外环会对高频横向机动加速度施加限幅与相位滞后（见 [B2](b2-sitl-plan-clearance.md) 记录的 0.40–0.98 m 横向跟踪误差与 [explore-sitl.md](explore-sitl.md) 中 7.5 m/s 急转擦树案例）。绕过 PX4 位置/速度外环、在 `PLAN` 阶段直接由 5 阶 MINCO 微分平坦前馈加几何 `SE(3)` 反馈输出 **集体推力 + 机体角速率（CTBR，`VehicleRatesSetpoint`）**，是在不改任何 PX4 参数的前提下将 SITL 穿林速度推向 **8.5–10.0 m/s** 并压低横向跟踪滞后的正交路径。

## 2. 模块接缝与状态机边界

- **新增独立模块 `yopo_l0/ctbr.py`：** 纯数学与无状态/轻状态控制器实现（`FlatnessCTBRController`、`HoverThrustEstimator`、`enu_flu_to_ned_frd_rates`），不修改 `core/`（`core/VENDORED.sha256` 59 个文件字节不变），也不影响 L0 默认点质量仿真器。
- **桥接开关 `--control-mode {pva,ctbr}`（默认 `pva`）：**
  - 接入 `yopo_l0/sitl_bridge.py` 与 `scripts/sitl/yopo_l0_live.py`（`Px4RosAdapter` 在保留 `/px4_i/fmu/in/trajectory_setpoint` 的同时新增 `/px4_i/fmu/in/vehicle_rates_setpoint` 发布器）。
  - 缺省 `--control-mode pva` 时，发布话题、`OffboardControlMode` 标志位与设定点字节与当前主干逐位一致。
  - 保持 `px4_parameters_written == []`、`scenes/suite.txt` 与全部既有场景卡不变。
- **分阶段模式切换（仅 `PLAN` 段启用 CTBR）：**
  - `WARMUP`、`TAKEOFF`、到达终点后的悬停收敛段与 `LAND`（`NAV_LAND`）全程保留现有 `TrajectorySetpoint`（`OffboardControlMode(position=True, velocity=True, acceleration=True, attitude=False, body_rate=False)`）。
  - 仅当 `--control-mode ctbr` 且无人机处于 `phase == "PLAN"` 时，切换为 `OffboardControlMode(position=False, velocity=False, acceleration=False, attitude=False, body_rate=True)` 并以锁步控制频率（50–100 Hz）发布 `/px4_i/fmu/in/vehicle_rates_setpoint`。一旦进入终点保持、超时刹停或降落阶段，立即无冲击切回 `TrajectorySetpoint` / `NAV_LAND`。

## 3. 微分平坦与几何 `SE(3)` 控制律规范（ENU/FLU → PX4 NED/FRD）

`ExploreFleet` 提交的每段 5 阶多项式轨迹（段内 $C^4$ 连续）在采样时刻 $t$ 直接给出世界系 ENU 下的平坦参考量 $(p_d, v_d, a_d, j_d) \in \mathbb{R}^3$ 以及偏航参考 $(\psi_d, \dot{\psi}_d)$：

1. **带饱和几何平动反馈加速度（世界系 ENU，$g = 9.80665\,\text{m/s}^2$，$e_3 = [0, 0, 1]^\top$）：**
   $$a_{\text{des}} = a_d + K_p\,\operatorname{clip}(p_d - p,\,-e_{p,\max},\,e_{p,\max}) + K_v\,\operatorname{clip}(v_d - v,\,-e_{v,\max},\,e_{v,\max}) + g\,e_3$$
   （初始增益 $K_p, K_v, K_R$ 可按 Gazebo `gz_x500_depth` 惯量整定；若需历史平坦/几何控制增益对照，可只读参考外部归档 `/mnt/g/Grok/yopo`。）
2. **期望机体姿态阵 $R_d \in \mathrm{SO}(3)$（世界 ENU 下机体 FLU 轴系）：**
   $$b_{3d} = \frac{a_{\text{des}}}{\|a_{\text{des}}\|}, \quad b_{1c} = [\cos\psi_d,\,\sin\psi_d,\,0]^\top, \quad b_{2d} = \frac{b_{3d} \times b_{1c}}{\|b_{3d} \times b_{1c}\|}, \quad b_{1d} = b_{2d} \times b_{3d}, \quad R_d = [b_{1d}\;\; b_{2d}\;\; b_{3d}]$$
   当 $\|b_{3d} \times b_{1c}\| < 10^{-3}$ 时退化到上一拍连续水平参考轴，避免奇异翻转。
3. **由加加速度 $j_d$ 与 $\dot{\psi}_d$ 解析求取前馈机体角速率 $\omega_{\text{ff}}$（FLU）：**
   $$h_\omega = \frac{1}{\|a_{\text{des}}\|}\Bigl(j_d - (b_{3d}^\top j_d)\,b_{3d}\Bigr), \quad \omega_{x,\text{ff}} = -b_{2d}^\top h_\omega, \quad \omega_{y,\text{ff}} = b_{1d}^\top h_\omega, \quad \omega_{z,\text{ff}} = \dot{\psi}_d\,(e_3^\top b_{3d})$$
4. **$\mathrm{SO}(3)$ 姿态误差反馈与 FLU → PX4 FRD 符号映射：**
   设当前里程计姿态矩阵为 $R \in \mathrm{SO}(3)$（FLU 相对 ENU），机体系总指令角速率与 FRD 转换：
   $$e_R = \frac{1}{2}(R_d^\top R - R^\top R_d)^\vee, \quad \omega_{\text{flu}} = R^\top R_d\,\omega_{\text{ff}} - K_R\,e_R$$
   $$(p, q, r)_{\text{frd}} = \operatorname{clip}\!\bigl((\omega_{x,\text{flu}},\; -\omega_{y,\text{flu}},\; -\omega_{z,\text{flu}}),\; -\omega_{\max},\; \omega_{\max}\bigr)$$
5. **在线悬停油门估计（`HoverThrustEstimator`）与归一化集体推力：**
   - 在 `TAKEOFF` 末端切入 `PLAN` 前的稳态悬停窗口（$|v_z| < 0.10\,\text{m/s}$、$|a_z| < 0.30\,\text{m/s}^2$）锁定基准悬停油门 $\hat{\tau}_h \in [0.35, 0.80]$，并在 `PLAN` 段以低增益新息限幅更新，无需读取或改写 PX4 `MPC_THR_HOVER` 参数。
   - 沿当前机体上轴 $b_3 = R e_3$ 投影推力（大姿态角时乘余弦保护上限），映射为 PX4 `VehicleRatesSetpoint.thrust_body`（FRD 机体 $z$ 轴向下，向上升力为负值）：
     $$\tau_{\text{norm}} = \operatorname{clip}\!\left(\frac{\max(a_{\text{des}}^\top b_3,\, 0)}{g}\,\hat{\tau}_h,\; \tau_{\min},\; \tau_{\max}\right), \quad \text{thrust\_body} = [0,\; 0,\; -\tau_{\text{norm}}]^\top$$

## 4. 验收标准（供后续实现票核验）

1. **默认路径零回归：** `--control-mode pva`（缺省）下，L0 回归套件 `scenes/suite.txt` 与既有 `sitl_bridge` 单元测试逐位不变；`core/VENDORED.sha256` 全绿；`px4_parameters_written == []`。
2. **平坦度与坐标变换单元测试（`tests/test_ctbr_flatness.py`）：**
   - 在含非零 jerk 与非零偏航角速度的 5 阶多项式轨迹上，验证解析前馈角速率 $\omega_{\text{ff}}(t)$ 与姿态阵数值微分 $(R_d^\top \dot{R}_d)^\vee$ 的最大误差 `< 1e-5 rad/s`；
   - 验证 ENU/FLU 与 PX4 NED/FRD 的滚转/俯仰/偏航角速率极性、`thrust_body[2] < 0` 符号、接近奇异姿态保护、油门/角速率限幅与悬停油门估计器收敛性；
   - 验证 `TAKEOFF (PVA) -> PLAN (CTBR) -> LAND (PVA/NAV_LAND)` 状态机切换无跳变、无残留 `body_rate=True` 标志。
3. **锁步 PX4 + Gazebo Harmonic SITL 同场对照（`report-only` / 只报数，判据未冻结）：**
   - 在 `explore-3-forest-sitl80` 与 `explore-6-forest-sitl80`（seeds `0..4`，无头锁步，`px4_parameters_written == []`）上同种子对比 `--control-mode pva` 与 `--control-mode ctbr`：
     - `PLAN` 段横向跟踪误差 p95 较 `pva` 基线显著下降；
     - 在保持 **机间碰撞 = 0、障碍碰撞 = 0** 的前提下，将 Gazebo 实测林带均速由 `6.16–7.58 m/s` 推向 **8.5–10.0 m/s**，并提升六机与三机高目标速度档的满编到达率。
