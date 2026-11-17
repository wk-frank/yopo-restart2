"""YOPO 网络 CPU 推理（进程内）。走 vendored `yopo_swarm_core.baseline` 的金样路径：
钉定上游 SHA + 权重 sha256 校验。默认 1 线程 fp32 + inference_mode，
与 `no_grad` 逐位相同。多线程和 ONNX Runtime 走 `YOPO_INFER_ACCEL`，见 infer_accel。
"""

from __future__ import annotations

import functools

import numpy as np
from yopo_swarm_core import baseline as B
from .depth_accel import network_depth
from yopo_swarm_core.infer_ipc import body_obs


@functools.lru_cache(maxsize=1)
def load_model() -> B.BaselineModel:
    import torch

    # 与 infer_ipc 服务端一致。多线程只在 YOPO_INFER_ACCEL=threads 时由 infer_accel 改。
    torch.set_num_threads(1)
    return B.load_verified_model(B.fetch_upstream())


def provenance() -> dict:
    return {"upstream_sha": B.UPSTREAM_SHA, "weights_sha256": B.WEIGHTS_SHA256, "backend": "torch-cpu-fp32"}


class InProcessSource:
    """`yopo_swarm_core.planner.CandidateSource` 的进程内实现（替代 L1 的 IpcSource）。"""

    name = "in-process-cpu"

    def __init__(self):
        from .infer_accel import configure

        self.model = load_model()
        configure(self.model)
        self._raw: np.ndarray | None = None
        self._stamp: float | None = None

    def bind_depth(self, raw_m: np.ndarray, stamp: float):
        self._raw, self._stamp = raw_m, stamp

    def describe(self) -> dict:
        return {"name": self.name, **provenance()}

    def forward_raw(self, raw_m, velocity_w, acceleration_w, goal_w, position_w, R_wb):
        from .infer_accel import run_forward
        from .plan_wall import measure

        o = body_obs(velocity_w, acceleration_w, goal_w, position_w, R_wb)
        depth = network_depth(raw_m)
        with measure():
            out = run_forward(self.model, depth, o)
        return out.traj_params[0], out.score[0], out.radius[0]

    def __call__(self, obs, dec):
        if self._raw is None or self._stamp != obs.frame.stamp:
            raise ValueError("network raw depth must be bound to this observation")
        s = obs.state
        return self.forward_raw(self._raw, s.velocity, s.acceleration, obs.goal, s.position, s.R_wb)
