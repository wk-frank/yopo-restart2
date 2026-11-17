"""Opt-in ENU/FLU flatness controller; PX4 receives FRD rates and negative Z thrust.

No PX4 parameters are read or written. The attitude derivative includes the full
heading/tilt coupling, rather than the approximate yaw-rate expression in CTBR-1.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

GRAVITY = 9.80665


def _vec(x):
    out = np.asarray(x, dtype=float)
    if out.shape != (3,) or not np.isfinite(out).all():
        raise ValueError("expected a finite three-vector")
    return out


def enu_flu_to_ned_frd_rates(rates):
    """Body-rate coordinates change FLU -> FRD; no world-axis swap applies."""
    return _vec(rates) * np.array([1.0, -1.0, -1.0])


def flatness_attitude(acceleration, jerk, yaw=0.0, yaw_rate=0.0):
    """Return R_ENU_FLU and exact vee(R.T Rdot) for a nonsingular flat output."""
    force = _vec(acceleration) + np.array([0.0, 0.0, GRAVITY])
    jerk = _vec(jerk)
    if not math.isfinite(yaw) or not math.isfinite(yaw_rate):
        raise ValueError("yaw and yaw_rate must be finite")
    norm = float(np.linalg.norm(force))
    if norm < 1e-6:
        force = np.array([0.0, 0.0, GRAVITY])
        norm = GRAVITY
        jerk = np.zeros(3)
    z = force / norm
    zdot = (jerk - z * np.dot(z, jerk)) / norm
    heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    hdot = yaw_rate * np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    cross = np.cross(z, heading)
    if np.linalg.norm(cross) < 1e-3:
        # Least aligned world axis is a deterministic, finite singular fallback.
        heading = np.eye(3)[int(np.argmin(np.abs(z)))]
        hdot = np.zeros(3)
        cross = np.cross(z, heading)
    size = float(np.linalg.norm(cross))
    y = cross / size
    cdot = np.cross(zdot, heading) + np.cross(z, hdot)
    ydot = (cdot - y * np.dot(y, cdot)) / size
    x = np.cross(y, z)
    xdot = np.cross(ydot, z) + np.cross(y, zdot)
    rotation = np.column_stack((x, y, z))
    derivative = np.column_stack((xdot, ydot, zdot))
    skew = rotation.T @ derivative
    rates = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    return rotation, rates


@dataclass
class HoverThrustEstimator:
    """Calibrate from PX4's published PVA thrust before switching to CTBR.

    During CTBR, measured vertical velocity differences supply a slow bounded
    correction. A mere configuration default is not reported as calibration.
    """
    value: float = 0.5
    samples: int = 0

    def observe_hover(self, thrust, vertical_speed, vertical_acceleration):
        if (math.isfinite(thrust) and 0.15 <= thrust <= 0.85
                and abs(vertical_speed) < 0.15 and abs(vertical_acceleration) < 0.4):
            gain = 1.0 / min(self.samples + 1, 30)
            self.value = float(np.clip(self.value + gain * (thrust - self.value), 0.15, 0.85))
            self.samples += 1
        return self.value

    def adapt(self, thrust, acceleration_z, body_up_z, dt):
        if 0.0 < dt < 0.1 and body_up_z > 0.85 and abs(acceleration_z) < 3.0:
            expected = GRAVITY + float(acceleration_z)
            estimate = thrust * GRAVITY * body_up_z / expected
            innovation = float(np.clip(estimate - self.value, -0.03, 0.03))
            self.value = float(np.clip(self.value + 0.15 * dt * innovation, 0.15, 0.85))


@dataclass(frozen=True)
class CTBRCommand:
    rates_frd: np.ndarray
    thrust: float
    desired_rotation: np.ndarray

    def fields(self, now_s):
        return {"timestamp": int(round(now_s * 1e6)), "roll": float(self.rates_frd[0]),
                "pitch": float(self.rates_frd[1]), "yaw": float(self.rates_frd[2]),
                "thrust_body": [0.0, 0.0, -float(self.thrust)], "reset_integral": False}


class FlatnessCTBRController:
    def __init__(self, *, kp=3.5, kv=3.0, kr=5.0, max_rate=2.0):
        self.kp, self.kv, self.kr = float(kp), float(kv), float(kr)
        self.max_rate = float(max_rate)
        self.hover = HoverThrustEstimator()
        self.last_thrust = None
        self.last_rates = np.zeros(3)

    def command(self, p_ref, v_ref, a_ref, j_ref, yaw, yaw_rate, position, velocity, rotation):
        R = np.asarray(rotation, dtype=float)
        if R.shape != (3, 3) or not np.isfinite(R).all():
            raise ValueError("expected finite ENU/FLU rotation")
        acc = (_vec(a_ref) + self.kp * np.clip(_vec(p_ref) - _vec(position), -2.0, 2.0)
               + self.kv * np.clip(_vec(v_ref) - _vec(velocity), -3.0, 3.0))
        acc[2] = np.clip(acc[2], -4.0, 5.0)
        # A 30-degree tilt ceiling and collective/rate limits bound feedback.
        horizontal = float(np.linalg.norm(acc[:2]))
        limit = (GRAVITY + acc[2]) * math.tan(math.radians(30.0))
        if horizontal > limit:
            acc[:2] *= limit / horizontal
        Rd, ff = flatness_attitude(acc, j_ref, float(yaw), float(yaw_rate))
        skew = 0.5 * (Rd.T @ R - R.T @ Rd)
        error = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        # Jerk can jump at receding-horizon joins and after acceleration clipping.
        # Bound feedforward independently so it cannot drive past the tilt target.
        rates = R.T @ Rd @ np.clip(ff, -.6, .6) - self.kr * error
        rates = np.clip(enu_flu_to_ned_frd_rates(rates), -self.max_rate, self.max_rate)
        rates = self.last_rates + np.clip(rates - self.last_rates, -.12, .12)
        self.last_rates = rates.copy()
        force = acc + np.array([0.0, 0.0, GRAVITY])
        projected = max(float(force @ R[:, 2]), 0.0)
        # During a forward-to-braking tilt reversal, projection alone removes
        # vertical lift. Preserve vertical support while the rate loop turns.
        vertical_support = max(float(force[2]), 0.0) / max(float(R[2, 2]), .6)
        thrust = float(np.clip(max(projected, vertical_support) / GRAVITY * self.hover.value, 0.05, 0.98))
        self.last_thrust = thrust
        return CTBRCommand(rates, thrust, Rd)
