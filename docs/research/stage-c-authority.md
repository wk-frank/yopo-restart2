# 阶段 C 飞控接收端权限复验

在项目目录用两个独立进程回放板端和仿真飞控接收端。输出目录须尚未创建。

```sh
python -m yopo_l0.stage_c_authority --scenario handover --out out/stage-c-handover
python -m yopo_l0.stage_c_authority --scenario deadlines --out out/stage-c-deadlines
python -m pytest -q tests/test_stage_c_authority.py
```

`handover` 回放 PVA 主控与 CTBR 影子并跑、CTBR 实控、终点交回 PVA 和人工接管。影子结果只进入板端日志。终点交还发生后，请求切回 CTBR 会得到 `TERMINAL_LATCHED`。

`deadlines` 在规划线程暂停时继续运行控制循环。请求超期后使用有效窗内已验证的台架制动参考。制动参考到期、定位中断或控制超周期时，接收端记录权限失效。恢复定位和收到新的有效计划后才重新获得 CTBR 权限。回放包含超期回复，以及定位中断前发起、返回时仍未过截止期但已经失效的回复。

- `board.jsonl` 记录模式请求、规划请求与采用或拒绝、CTBR 影子、控制周期和降级处置。
- `flight_controller.jsonl` 记录接收端接受的 PVA 或 CTBR 指令、时间、会话、权限版本、参考速度及所用里程计时间与位置。
- `flight_controller_authority.jsonl` 记录接收端的模式切换与失效。旧指令最迟在自身有效期结束时失效，也可被更高权限版本提前撤销。人工接管和无自动控制权只发送权限消息。
- `flight_controller_rejected.jsonl` 记录接收端拒绝的报文与原因。

共用入口 `yopo_l0.board_runtime` 可用 `--backend 模块 --backend-config 路径` 加载同进程的适配器。模块的 `build_backend(config, log, stopping, backend_config)` 返回 `BoardBackend(control_step, invalidate)`。控制回调提供最新读数和限时轨迹采样。规划任务在独立工作线程或进程完成，提交必须携带 `BoardControl.begin_plan` 发出的请求票据。控制侧在模式切换、定位失效或截止期到达时使旧票据失效。控制端只在姿态与位置新鲜且制动参考处于其核验时间窗内时使用预先获准的制动尾。

这两个用例在回环网络运行，使用软件回放定位和制动参考。实体制动包络与飞控权限仍需逐板台架记录验证。
