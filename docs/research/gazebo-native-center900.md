# Gazebo 原生深度：六机 YOPO＋CTBR 900 m 验收

状态：本次 seed 0 锁步 SITL 验收通过，report-only。六机从 2.2 km × 2.2 km 场景中心同向出发，避障时允许队形伸缩；6/6 到达并降落。每机在 CTBR 阶段的 Gazebo 水平位移为 936.72–937.55 m。

运行：`20261117T071921-yopo-n6`；运行源码提交：`f58469b7a4ededfc98c324f424c05b1b8390450e`。开始与结束的源码及 HEAD 一致，所有运行文件哈希已逐一对照该提交中的 Git 对象。13 项机器验收检查全部通过，见 [机器可读结果](gazebo-native-center900-runs.json)。

## 每机结果

900 m 指每机相对各自起点的水平直线位移，不是累计航程。PX4 里程计和 Gazebo 真值独立核对，且要求 CTBR 控制样本中两者同时达到 900 m。

| 飞机 | Gazebo 位移 / m | PX4 位移 / m | CTBR 段真值位移 / m | 消费的不同深度帧 | CTBR 指令数 |
|---|---:|---:|---:|---:|---:|
| uav0 | 942.81 | 940.01 | 937.20 | 3293 | 10450 |
| uav1 | 945.76 | 942.97 | 937.55 | 3293 | 10680 |
| uav2 | 945.33 | 942.48 | 937.38 | 3293 | 10470 |
| uav3 | 942.92 | 940.11 | 937.10 | 3293 | 10530 |
| uav4 | 942.95 | 940.11 | 937.43 | 3293 | 10550 |
| uav5 | 943.11 | 940.40 | 936.72 | 3293 | 10455 |

Gazebo PLAN 真值碰撞样本为 0，六机缺失样本均为 0；最小机体表面净空 0.298 m，最小机间中心距 2.358 m。PX4 参数写入为 `[]`。

仿真时长 237.10 s，任务墙钟 1650.73 s，锁步控制不代表墙钟实时运行。

碰撞及净空按 Gazebo 机心真值、场景几何与半径 0.35 m 的球形机体包络计算，PLAN 阶段按仿真 50 Hz 记录；本轮未采集原生 contact 事件。

## 原生深度链路

每架 x500 的 `base_link` 绑定 Gazebo `depth_camera`，由 Sensors/Ogre2 渲染，经 gz-transport 的各机独立 `/world/default/model/x500_N/link/base_link/sensor/yopo_depth/depth_image` 话题进入 YOPO。像素格式 `R_FLOAT32`，单位米，保留 Gazebo 捕获时间戳。编队入口已改为消费外部帧；显式原生模式缺帧或帧龄超过 0.20 s 时失败关闭，不回退至场景卡射线投射。

相机为 160 × 96、水平视场 90°、竖直视场 61.9275°、近远裁剪 0.2–20 m、15 Hz；机体系安装位置为 `(0.35, 0, 0.18)` m，以避开自身机体遮挡。本次原生渲染不额外添加深度噪声。六路 CameraInfo 实测 `fx=fy=80, cx=80, cy=48`，与输入标定一致。输入来自带机体完整姿态的原生传感器。

最大消费帧龄 0.076 s，解码错误为 0，无合成深度回退。`depth-consumed.jsonl` 保存消费帧时间和 SHA-256；另每 5 s 保存各机原始米制深度 NPZ，共 264 帧，全部已与消费哈希核对。

世界文件保留 3025 个圆柱的视觉和碰撞几何，覆盖 55 × 55 个网格；场景前后左右均有障碍，仅中心留安全起飞空间。未清空航路或终点。新功能默认关闭，原场景卡、冻结 suite、core 钉定文件与 PX4 参数均未改写。

## Gazebo 实际画面

独立 Gazebo RGB 观察相机跟随六机真值中心，直接记录本次动态世界中的 x500 与障碍物，经 ffmpeg 编码。Ogre2 日志确认渲染设备为 `D3D12 (NVIDIA GeForce RTX 5060)`。这份录像来自正在飞行的 Gazebo 世界，不是按轨迹重新绘制的动画。

原片为 1280 × 720、10 fps、2371 帧、237.1 s，最大相邻捕获时间差 0.100 s；原始 MP4 的 SHA-256 与记录一致。带数据版保留完整视野并叠加同趟保存的位移、控制模式与阶段；原始全景版完整保留。轨迹数据按 5 Hz 保存，视频数据标注据此对齐。30 秒预览压缩播放时间，不代表墙钟实时飞行。

## 复现

```bash
docker build -f scripts/sitl/Dockerfile.gazebo-depth -t yopo-restart-sitl:gazebo-depth .
YOPO_SITL_DOCKER_NETWORK=bridge YOPO_SITL_LOCKSTEP=1 \
YOPO_LIVE_DEPTH_SOURCE=gazebo YOPO_SITL_GAZEBO_GPU=wsl-d3d12 \
YOPO_DEPTH_BACKEND=opencv YOPO_SITL_WALL_TIMEOUT=7200 \
scripts/sitl/run_yopo_l0_sitl.sh --image yopo-restart-sitl:gazebo-depth \
  --scene scenes/explore-6-center900-gz-depth.json --n 6 --seed 0 \
  --policy explore --lockstep --scene-world --control-mode ctbr --record-dt 0.2
```

GPU 选项是本机 WSL D3D12 环境的显式选择；未启用时可使用软件 EGL 渲染，耗时会增加。运行前固定源码与提交，飞行期间不要修改运行文件或提交 HEAD；运行器会核对前后源码并拒绝把变更中的运行记为通过。

## 验证和边界

106 项受影响检查通过（原 105 项与新增的非 explore 策略启动前拒绝检查）。四项必需检查通过：场景严格校验、文档链接、研究索引、一期收口；59 个 core 钉定哈希保持一致。新增测试覆盖米制深度/填充行解码、外部帧实际进入 YOPO 且禁止合成回退、缺帧拒绝、SDF 惯性/碰撞不变与原生验收失败关闭。完整回归中的既有数值夹具问题沿用前轮独立基线复现结论，见 [前轮报告](ctbr-center900.md)。

这是一个场景、一个任务种子的 SITL 结果，尚非实体飞机实飞。YOPO 使用原生深度，但原有已知地图几何安全盾和机间轨迹协同仍在；没有宣称未知地图 SLAM、frontier 分配或遍历全图。当前任务沿预设 +X 方向推进。起飞与终点保持由 PVA 承担，巡航和绕障为 CTBR，降落由 PX4 NAV_LAND 完成。50 Hz 指仿真钟控制频率，不是墙钟实时性能；原 CTBR-1 高速多种子对照仍待做。

## 产物校验

原片 ffprobe 核对为 H.264/yuv420p、1280 × 720、2371 帧、237.1 s；标注版为 1600 × 900、2371 帧、237.1 s；30 秒预览为 600 帧/20 fps。原片逐帧时间戳最大间隔 0.100 s，全部 264 个深度 NPZ 抽样与消费哈希一致。起点、中途和到达画面已检查，HTML 脚本语法已通过 Node 检查；尚未声称完成真实浏览器交互验收。
