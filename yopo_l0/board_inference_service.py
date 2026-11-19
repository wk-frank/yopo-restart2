"""Build, serve and verify the board-local TensorRT FP32 candidate source."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from hashlib import sha256
import json
from pathlib import Path
import signal
from time import monotonic_ns, sleep
from uuid import uuid4

import numpy as np

from yopo_swarm_core.baseline import (
    GOLDEN_ATOL,
    GOLDEN_PATH,
    GOLDEN_RTOL,
    WEIGHTS_SHA256,
    fixed_inputs,
)
from yopo_swarm_core.candidates import UPSTREAM_DECODER
from .board_inference_ipc import serve
from .board_inference import BoardInference, InferenceIdentity
from .board_runtime import JsonlLog


_INPUT_SHAPES = {"depth": (1, 1, 96, 160), "obs": (1, 9, 3, 5)}
_OUTPUT_SHAPES = {"endstate": (1, 14, 3, 5), "score": (1, 3, 5), "radius": (1, 20, 3, 5)}
_TENSOR_SHAPES = {**_INPUT_SHAPES, **_OUTPUT_SHAPES}
_GOLDEN_NAMES = ("traj_params", "score", "radius")


def _grid_rotations() -> np.ndarray:
    from scipy.spatial.transform import Rotation

    yaw, pitch = UPSTREAM_DECODER.grid_angles()
    angles = np.column_stack((yaw, -pitch, np.zeros_like(yaw))).astype(np.float32)
    return Rotation.from_euler("ZYX", angles).as_matrix().astype(np.float32)


def _prepare_obs(obs: np.ndarray, image_rotations: np.ndarray) -> np.ndarray:
    obs = obs.copy()
    obs[:, :3] /= UPSTREAM_DECODER.vel_max
    obs[:, 3:6] /= UPSTREAM_DECODER.acc_max
    goal = obs[:, 6:9]
    goal /= np.maximum(np.linalg.norm(goal, axis=1, keepdims=True),
                       UPSTREAM_DECODER.tail_radio_max)
    rotated = obs.reshape(1, 1, 3, 3) @ image_rotations
    return np.ascontiguousarray(rotated.reshape(1, 15, 9).transpose(0, 2, 1).reshape(1, 9, 3, 5))


def _cuda(result: tuple):
    code, *values = result
    if int(code) != 0:
        raise RuntimeError(f"CUDA runtime failed: {code}")
    return values[0] if values else None


class TensorRtFp32:
    """Single serialized engine and execution context. Call in the service thread."""

    def __init__(self, path: Path, engine_sha256: str, lifetime: ExitStack) -> None:
        engine_bytes = path.read_bytes()
        if sha256(engine_bytes).hexdigest() != engine_sha256:
            raise ValueError("TensorRT engine SHA-256 mismatch")

        import tensorrt as trt
        try:
            from cuda.bindings import runtime as cudart
        except ImportError:
            from cuda import cudart

        self._cudart = cudart
        self.version = trt.__version__
        self._runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        self._engine = self._runtime.deserialize_cuda_engine(engine_bytes)
        if self._engine is None:
            raise RuntimeError("could not deserialize local TensorRT engine")
        names = {self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)}
        if names != _TENSOR_SHAPES.keys():
            raise ValueError("TensorRT engine IO names differ from the pinned YOPO layout")
        for name, shape in _TENSOR_SHAPES.items():
            mode = trt.TensorIOMode.INPUT if name in _INPUT_SHAPES else trt.TensorIOMode.OUTPUT
            if (tuple(self._engine.get_tensor_shape(name)) != shape
                    or self._engine.get_tensor_dtype(name) != trt.float32
                    or self._engine.get_tensor_mode(name) != mode
                    or self._engine.get_tensor_format(name) != trt.TensorFormat.LINEAR
                    or self._engine.get_tensor_location(name) != trt.TensorLocation.DEVICE):
                raise ValueError(f"unsupported TensorRT binding: {name}")

        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("could not create the TensorRT execution context")
        self._rotations = _grid_rotations()
        self._stream = _cuda(cudart.cudaStreamCreate())
        lifetime.callback(_cuda_cleanup, cudart.cudaStreamDestroy, self._stream)
        self._addresses = {}
        self._output = {name: np.empty(shape, dtype=np.float32)
                        for name, shape in _OUTPUT_SHAPES.items()}
        for name, shape in _TENSOR_SHAPES.items():
            address = _cuda(cudart.cudaMalloc(int(np.prod(shape)) * np.dtype(np.float32).itemsize))
            lifetime.callback(_cuda_cleanup, cudart.cudaFree, address)
            if not self._context.set_tensor_address(name, int(address)):
                raise RuntimeError(f"could not register TensorRT buffer: {name}")
            self._addresses[name] = address
        lifetime.callback(_cuda_cleanup, cudart.cudaStreamSynchronize, self._stream)

    def __call__(self, depth: np.ndarray, obs: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if (depth.shape != _INPUT_SHAPES["depth"] or obs.shape != (1, 9)
                or depth.dtype != np.float32 or obs.dtype != np.float32
                or not np.isfinite(depth).all() or not np.isfinite(obs).all()):
            raise ValueError("invalid FP32 inference inputs")
        inputs = {"depth": np.ascontiguousarray(depth), "obs": _prepare_obs(obs, self._rotations)}
        cuda = self._cudart
        try:
            for name, array in inputs.items():
                _cuda(cuda.cudaMemcpyAsync(self._addresses[name], array.ctypes.data, array.nbytes,
                                           cuda.cudaMemcpyKind.cudaMemcpyHostToDevice, self._stream))
            if not self._context.execute_async_v3(int(self._stream)):
                raise RuntimeError("TensorRT execution failed")
            for name, array in self._output.items():
                _cuda(cuda.cudaMemcpyAsync(array.ctypes.data, self._addresses[name], array.nbytes,
                                           cuda.cudaMemcpyKind.cudaMemcpyDeviceToHost, self._stream))
        finally:
            _cuda(cuda.cudaStreamSynchronize(self._stream))

        return (
            self._output["endstate"].transpose(0, 2, 3, 1).reshape(1, 15, 14).copy(),
            self._output["score"].reshape(1, 15).copy(),
            self._output["radius"].transpose(0, 2, 3, 1).reshape(1, 15, 20).copy(),
        )


def _cuda_cleanup(operation, handle) -> None:
    _cuda(operation(handle))


def _export_onnx(out: Path) -> None:
    # Torch is an offline export dependency. The service only imports TensorRT and CUDA.
    import torch
    from yopo_swarm_core.baseline import fetch_upstream, load_verified_model

    model = load_verified_model(fetch_upstream())
    torch.onnx.export(
        model._net,
        tuple(torch.zeros(shape, dtype=torch.float32) for shape in _INPUT_SHAPES.values()),
        str(out),
        input_names=list(_INPUT_SHAPES),
        output_names=list(_OUTPUT_SHAPES),
        opset_version=17,
        dynamo=False,
    )
    print(json.dumps({"onnx_sha256": sha256(out.read_bytes()).hexdigest(),
                      "model_sha256": model.weights_sha256, "config_sha256": model.config_hash}))


def _golden(args, identity: InferenceIdentity) -> int:
    depth, obs = fixed_inputs({"vertical_num": 3, "horizon_num": 5, "downsample_factor": 32})
    events = []
    deadline = monotonic_ns() + 2_000_000_000
    result = None
    with BoardInference(args.socket, session_id=uuid4().hex, identity=identity,
                        log=lambda event, **fields: events.append({"event": event, **fields})) as client:
        client.submit("fixed-golden", depth, obs, deadline_ns=deadline)
        while result is None and monotonic_ns() < deadline:
            result = client.poll("fixed-golden")
            if events and events[-1]["event"] == "inference_rejected":
                break
            if result is None:
                sleep(0.001)

    report = {"backend": identity.backend, **identity.provenance, "events": events,
              "pass_tol": False, "argmax_equal": False}
    if result is not None:
        with np.load(args.golden_npz) as reference:
            comparisons = {key: (getattr(result, key), reference[key]) for key in _GOLDEN_NAMES}
        report["pass_tol"] = all(
            observed.shape == expected.shape
            and bool(np.allclose(observed, expected, rtol=GOLDEN_RTOL, atol=GOLDEN_ATOL))
            for observed, expected in comparisons.values()
        )
        report["argmax_equal"] = bool(result.score.argmax() == comparisons["score"][1].argmax())
        report["max_abs"] = {
            key: float(np.max(np.abs(observed - expected))) if observed.shape == expected.shape else None
            for key, (observed, expected) in comparisons.items()
        }
    report["passed"] = report["pass_tol"] and report["argmax_equal"]
    raw = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.out is not None:
        args.out.write_text(raw, encoding="utf-8")
    else:
        print(raw, end="")
    return 0 if report["passed"] else 1


def _interrupt_on_terminate(_signal, _frame) -> None:
    raise KeyboardInterrupt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    server = commands.add_parser("serve", help="run a board's TensorRT FP32 service")
    server.add_argument("--engine", type=Path, required=True)
    server.add_argument("--log", type=Path, required=True)
    server.set_defaults(backend="tensorrt-fp32")

    compare = commands.add_parser("golden", help="check the fixed golden input through the actual IPC service")
    compare.add_argument("--backend", choices=("tensorrt-fp32", "replay-fp32"), default="tensorrt-fp32")
    compare.add_argument("--golden-npz", type=Path, default=GOLDEN_PATH)
    compare.add_argument("--out", type=Path)
    for command in (server, compare):
        command.add_argument("--socket", type=Path, required=True)
        command.add_argument("--model-sha256", default=WEIGHTS_SHA256)
        command.add_argument("--engine-sha256", required=True)

    export = commands.add_parser("export-onnx", help="export the pinned model offline")
    export.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    if args.action == "export-onnx":
        _export_onnx(args.out)
        return 0
    identity = InferenceIdentity(args.backend, args.model_sha256, args.engine_sha256)
    if args.action == "golden":
        return _golden(args, identity)
    if identity.model_sha256 != WEIGHTS_SHA256:
        parser.error("the board service requires the pinned YOPO model digest")
    with ExitStack() as stack:
        engine = TensorRtFp32(args.engine, identity.engine_sha256, stack)
        log = JsonlLog(stack.enter_context(args.log.open("a", encoding="utf-8")))
        log("inference_service_started", backend=identity.backend, **identity.provenance,
            socket=str(args.socket), tensorrt_version=engine.version)
        try:
            serve(args.socket, engine, identity.backend, identity=identity.provenance, log=log)
        except KeyboardInterrupt:
            return 0
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _interrupt_on_terminate)
    raise SystemExit(main())
