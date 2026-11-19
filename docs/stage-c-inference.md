# 阶段 C 板端 TensorRT FP32

每块板运行一个推理服务，通过本机 Unix 套接字给本板规划循环返回候选。规划按本机单调时钟限制每次请求。服务和规划进程使用同一单调时钟及独立的板端套接字。控制循环继续按自身周期运行。

## 构建引擎

离线导出环境需要固定上游模型、PyTorch 和 ONNX。下面命令校验上游提交及权重，并记录模型、配置和导出文件的 SHA-256。

```bash
python -m yopo_l0.board_inference_service export-onnx --out yopo.onnx
```

在目标 Orin 的 TensorRT 10 环境构建引擎并保存构建日志。使用静态 FP32 图和 FP32 默认精度，关闭 TF32。记录目标系统版本、TensorRT 版本、构建参数及引擎 SHA-256。将预期引擎摘要写入板端配置后再启动。

```bash
trtexec --onnx=yopo.onnx --saveEngine=yopo-fp32.engine --noTF32 --skipInference > build-engine.log 2>&1
sha256sum yopo-fp32.engine
```

[TensorRT 10 Python API](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/inference-library/python-api-docs.html)

## 启动与对账

常驻服务依赖 NumPy、SciPy、TensorRT 10 和与板端 CUDA 匹配的 CUDA Python 绑定。服务加载本机序列化引擎，校验引擎摘要及五个 I/O 张量的形状、类型和位置。套接字路径、日志各板独立。服务与规划使用同一账户，或共享对套接字有权限的用户组。

```bash
python -m yopo_l0.board_inference_service serve \
  --socket /run/yopo/u1-infer.sock --engine yopo-fp32.engine \
  --engine-sha256 "<已核验的引擎 SHA-256>" --log u1-infer-service.jsonl
```

从同一块板的另一进程通过实际套接字对账。下列命令使用项目固定 FP32 金样输入，校验三组输出的 `rtol=1e-4`、`atol=1e-5` 及最高分候选索引，并附上请求和时序记录。将同一引擎摘要传给规划端和对账命令。

```bash
python -m yopo_l0.board_inference_service golden \
  --socket /run/yopo/u1-infer.sock --engine-sha256 "<已核验的引擎 SHA-256>" \
  --golden-npz core/tests/golden/minco_d5e1e60_cpu.npz --out u1-fp32-golden.json
```

规划记录请求发送、回复接收、采纳或拒绝的本机单调时标。服务记录计算完成时刻，以及过期请求的回复连接已关闭的情况。按 `session_id`、`frame_id` 和 `request_id` 对齐记录。规划每次只保持一个未完成请求，当前帧变化或截止时间到达后作废旧请求。

主机回放会标明 `replay-fp32`。历史 Orin 样例与主机回放可作为对照。当前引擎的精度与性能以目标板重新生成的日志和金样报告为准。
