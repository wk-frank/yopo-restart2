# 阶段 C 三进程保持用例

在 Linux 或 WSL 的项目目录运行，输出路径须为尚未创建的目录。

```sh
python -m yopo_l0.stage_c_hold --out out/stage-c-hold
python -m pytest -q tests/test_stage_c_hold.py
```

启动器各用一个进程运行 `python -m yopo_l0.board_runtime --config <config.json> --log <board.jsonl>`。三个实例分别绑定本机静态位置输入、身份、UDP 端口与仿真飞控接收端。控制周期为 20 ms，指令有效期为 100 ms。主机接收端从套接字读取命令并用同一主机单调时钟独立记录接收时间。保持用例使用配置中的静态位置，运行数据中没有地图或裁判位置。

默认用 `SIGSTOP` 暂停 `uav2` 至少 300 ms。其余两机继续发出保持指令，收到的 `uav2` 最近一包的龄期持续增长。随后终止并重新启动 `uav2`，由其余两机的心跳接收记录核对新会话。只运行正常启动可加 `--scenario hold`。

- `faults.jsonl` 保存暂停与重启时刻，可与每机记录按单调时钟比对。
- 每机目录的 `config.json` 对应本机身份、输入和端点。 `board.jsonl` 记录进程、会话、控制周期、心跳收发与消息龄期。
- 每机目录的 `flight_controller.jsonl` 保存实际收到且仍有效的命令。 `flight_controller_rejected.jsonl` 保存拒绝记录及原始报文。

本用例的接收端和控制输出均用于仿真台架软件检查。
