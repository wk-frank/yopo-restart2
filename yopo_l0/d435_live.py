"""D435 exposure-timestamp pose interpolation, stopping-distance speed gate, and IPC source."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
import numpy as np

from yopo_swarm_core.infer_ipc import IpcClient, body_obs
from .depth_accel import network_depth


def _orthonormalize(R: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(R, dtype=float).reshape(3, 3))
    out = u @ vt
    if np.linalg.det(out) < 0:
        u[:, -1] *= -1.0
        out = u @ vt
    return out


@dataclass(frozen=True)
class InterpolatedState:
    stamp_s: float
    position_enu: np.ndarray
    velocity_enu: np.ndarray
    acceleration_enu: np.ndarray
    rotation_enu_flu: np.ndarray


class PoseRingBuffer:
    """Monotonic-clock pose history for interpolating vehicle state at camera exposure time.

    Invalidates history on clock regression, large gap, or EKF/VIO position jump.
    """

    def __init__(
        self,
        *,
        max_samples: int = 128,
        max_pos_jump_m: float = 0.60,
        max_time_gap_s: float = 0.25,
    ) -> None:
        self.max_samples = int(max_samples)
        self.max_pos_jump_m = float(max_pos_jump_m)
        self.max_time_gap_s = float(max_time_gap_s)
        self._rows: list[InterpolatedState] = []
        self.invalid_reason: str = ""

    @property
    def valid(self) -> bool:
        return self.invalid_reason == "" and len(self._rows) > 0

    def reset(self) -> None:
        self._rows.clear()
        self.invalid_reason = ""

    def push(
        self,
        stamp_s: float,
        position_enu,
        velocity_enu,
        rotation_enu_flu,
        acceleration_enu=None,
    ) -> bool:
        t = float(stamp_s)
        p = np.asarray(position_enu, dtype=float).reshape(3)
        v = np.asarray(velocity_enu, dtype=float).reshape(3)
        a = np.zeros(3) if acceleration_enu is None else np.asarray(acceleration_enu, dtype=float).reshape(3)
        R = np.asarray(rotation_enu_flu, dtype=float).reshape(3, 3)
        if not (math.isfinite(t) and np.isfinite(p).all() and np.isfinite(v).all() and np.isfinite(R).all()):
            self.invalid_reason = "NONFINITE_STATE"
            return False
        if self._rows:
            prev = self._rows[-1]
            dt = t - prev.stamp_s
            if dt < -1e-6:
                self.invalid_reason = "CLOCK_REGRESSION"
                return False
            if dt > self.max_time_gap_s:
                self.invalid_reason = "ODOM_GAP"
                return False
            expected_step = max(float(np.linalg.norm(prev.velocity_enu)) * max(dt, 0.0) * 2.0, 0.0)
            if float(np.linalg.norm(p - prev.position_enu)) > self.max_pos_jump_m + expected_step:
                self.invalid_reason = "EKF_POSITION_JUMP"
                return False
            if dt <= 1e-9:
                self._rows[-1] = InterpolatedState(t, p.copy(), v.copy(), a.copy(), _orthonormalize(R))
                return self.valid
        self._rows.append(InterpolatedState(t, p.copy(), v.copy(), a.copy(), _orthonormalize(R)))
        if len(self._rows) > self.max_samples:
            self._rows = self._rows[-self.max_samples :]
        return self.valid

    def at_exposure(self, exposure_stamp_s: float, *, max_extrapolation_s: float = 0.025) -> InterpolatedState | None:
        if not self.valid:
            return None
        t = float(exposure_stamp_s)
        first, last = self._rows[0], self._rows[-1]
        if t < first.stamp_s - max_extrapolation_s or t > last.stamp_s + max_extrapolation_s:
            return None
        if t <= first.stamp_s:
            dt = t - first.stamp_s
            return InterpolatedState(
                t,
                first.position_enu + first.velocity_enu * dt,
                first.velocity_enu.copy(),
                first.acceleration_enu.copy(),
                first.rotation_enu_flu.copy(),
            )
        if t >= last.stamp_s:
            dt = t - last.stamp_s
            return InterpolatedState(
                t,
                last.position_enu + last.velocity_enu * dt,
                last.velocity_enu.copy(),
                last.acceleration_enu.copy(),
                last.rotation_enu_flu.copy(),
            )
        for i in range(len(self._rows) - 1):
            a, b = self._rows[i], self._rows[i + 1]
            if a.stamp_s <= t <= b.stamp_s:
                span = b.stamp_s - a.stamp_s
                alpha = 0.0 if span <= 1e-9 else (t - a.stamp_s) / span
                p = (1.0 - alpha) * a.position_enu + alpha * b.position_enu
                v = (1.0 - alpha) * a.velocity_enu + alpha * b.velocity_enu
                acc = (1.0 - alpha) * a.acceleration_enu + alpha * b.acceleration_enu
                R = _orthonormalize((1.0 - alpha) * a.rotation_enu_flu + alpha * b.rotation_enu_flu)
                return InterpolatedState(t, p, v, acc, R)
        return None


@dataclass(frozen=True)
class StoppingDistanceGate:
    """Enforce reliable_clearance_m >= v * delay_s + brake_distance(v) + margin_m."""

    response_delay_s: float = 0.15
    effective_decel_mps2: float = 2.0
    margin_m: float = 0.70

    def required_clearance_m(self, speed_mps: float, measured_brake_m: float | None = None) -> float:
        v = max(float(speed_mps), 0.0)
        brake = (
            float(measured_brake_m)
            if measured_brake_m is not None
            else (v * v) / (2.0 * max(self.effective_decel_mps2, 1e-6))
        )
        return v * self.response_delay_s + brake + self.margin_m

    def allow_speed(self, speed_mps: float, reliable_clearance_m: float, measured_brake_m: float | None = None) -> bool:
        return float(reliable_clearance_m) >= self.required_clearance_m(speed_mps, measured_brake_m) - 1e-9

    def max_safe_speed_mps(self, reliable_clearance_m: float) -> float:
        avail = float(reliable_clearance_m) - self.margin_m
        if avail <= 0.0:
            return 0.0
        a = max(self.effective_decel_mps2, 1e-6)
        tau = max(self.response_delay_s, 0.0)
        # Solve v^2 / (2a) + tau * v - avail = 0
        disc = max(0.0, tau * tau + 2.0 * avail / a)
        return max(0.0, a * (-tau + math.sqrt(disc)))


class IpcCandidateSource:
    """Drop-in ExploreFleet candidate source backed by yopo_swarm_core.infer_ipc."""

    name = "orin-ipc-fp32"

    def __init__(self, socket_path: str, *, timeout: float = 2.0) -> None:
        self.socket_path = str(socket_path)
        deadline = time.monotonic() + float(timeout)
        while True:
            try:
                self.client = IpcClient(self.socket_path, timeout=timeout)
                break
            except (ConnectionRefusedError, FileNotFoundError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.005)
        self.infer_ms: list[float] = []
        self.rtt_ms: list[float] = []

    def forward_raw(self, raw_m, velocity_w, acceleration_w, goal_w, position_w, R_wb):
        o = body_obs(velocity_w, acceleration_w, goal_w, position_w, R_wb)
        depth = network_depth(raw_m)
        t0 = time.perf_counter()
        traj_params, score, radius = self.client.forward(depth, o)
        self.rtt_ms.append((time.perf_counter() - t0) * 1e3)
        if "infer_ms" in self.client.last_header:
            self.infer_ms.append(float(self.client.last_header["infer_ms"]))
        return traj_params[0], score[0], radius[0]

    def summary(self) -> dict:
        def stats(vals: list[float]) -> dict:
            if not vals:
                return {"count": 0, "p50_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
            arr = np.asarray(vals, dtype=float)
            return {
                "count": int(arr.size),
                "p50_ms": float(np.percentile(arr, 50)),
                "p95_ms": float(np.percentile(arr, 95)),
                "p99_ms": float(np.percentile(arr, 99)),
                "max_ms": float(arr.max()),
            }

        return {
            "socket": self.socket_path,
            "backend": self.client.last_header.get("backend"),
            "infer": stats(self.infer_ms),
            "rtt": stats(self.rtt_ms),
        }

    def close(self) -> None:
        self.client.close()
