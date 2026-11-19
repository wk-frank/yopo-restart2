"""Board-local stopping constraints shared by planning and tracking."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from numbers import Real

from .d435_live import StoppingDistanceGate


def _nonnegative(value) -> float | None:
    if isinstance(value, Real) and not isinstance(value, bool):
        try:
            value = float(value)
        except OverflowError:
            return None
        if math.isfinite(value) and value >= 0.0:
            return value
    return None


@dataclass(frozen=True)
class StoppingDecision:
    target_speed_mps: float | None
    current_speed_mps: float | None
    reliable_clearance_m: float | None
    required_clearance_m: float | None
    current_required_clearance_m: float | None
    remaining_stop_margin_m: float | None
    safe_speed_mps: float
    speed_limit_mps: float
    command_speed_mps: float
    plan_allowed: bool
    brake_room_verified: bool
    action: str
    reason: str
    calibration: StoppingConstraints

    def limit_velocity(
        self, velocity_enu_mps: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Bound a reference in its existing direction; a brake has zero speed demand."""
        x, y, z = map(float, velocity_enu_mps)
        speed = math.hypot(x, y, z)
        if not math.isfinite(speed) or self.command_speed_mps == 0.0:
            return (0.0, 0.0, 0.0)
        if speed <= self.command_speed_mps:
            return (x, y, z)
        ratio = self.command_speed_mps / speed
        bounded = (x * ratio, y * ratio, z * ratio)
        if math.hypot(*bounded) > self.command_speed_mps:
            bounded = tuple(math.nextafter(value, 0.0) for value in bounded)
        return bounded

    def evidence(self) -> dict:
        """Data needed to re-evaluate a planning or execution decision offline."""
        return {
            "target_speed_mps": self.target_speed_mps,
            "current_speed_mps": self.current_speed_mps,
            "reliable_clearance_m": self.reliable_clearance_m,
            "required_clearance_m": self.required_clearance_m,
            "current_required_clearance_m": self.current_required_clearance_m,
            "remaining_stop_margin_m": self.remaining_stop_margin_m,
            "safe_speed_mps": self.safe_speed_mps,
            "speed_limit_mps": self.speed_limit_mps,
            "command_speed_mps": self.command_speed_mps,
            "plan_allowed": self.plan_allowed,
            "brake_room_verified": self.brake_room_verified,
            "action": self.action,
            "reason": self.reason,
            "response_delay_s": self.calibration.response_delay_s,
            "effective_decel_mps2": self.calibration.effective_decel_mps2,
            "margin_m": self.calibration.margin_m,
            "measured_brake_points": self.calibration.measured_brake_points,
        }


@dataclass(frozen=True)
class StoppingConstraints:
    response_delay_s: float = 0.15
    effective_decel_mps2: float = 2.0
    margin_m: float = 0.70
    measured_brake_points: tuple[tuple[float, float], ...] = ()
    _gate: StoppingDistanceGate = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        delay = _nonnegative(self.response_delay_s)
        decel = _nonnegative(self.effective_decel_mps2)
        margin = _nonnegative(self.margin_m)
        if delay is None or decel is None or decel < 1e-6 or margin is None:
            raise ValueError("invalid stopping calibration or effective deceleration below the gate minimum")
        points = []
        previous_speed, previous_brake = 0.0, 0.0
        for speed, brake in self.measured_brake_points:
            speed, brake = _nonnegative(speed), _nonnegative(brake)
            if (speed is None or speed <= previous_speed
                    or brake is None or brake < previous_brake):
                raise ValueError("brake bins require increasing speeds and nondecreasing distance bounds")
            points.append((speed, brake))
            previous_speed, previous_brake = speed, brake
        object.__setattr__(self, "response_delay_s", delay)
        object.__setattr__(self, "effective_decel_mps2", decel)
        object.__setattr__(self, "margin_m", margin)
        object.__setattr__(self, "measured_brake_points", tuple(points))
        object.__setattr__(self, "_gate", StoppingDistanceGate(delay, decel, margin))

    def _required(self, speed: float | None) -> float | None:
        if speed is None:
            return None
        model = _nonnegative(self._gate.required_clearance_m(speed))
        if not self.measured_brake_points or speed == 0.0 or model is None:
            return model
        for upper_speed, measured_brake in self.measured_brake_points:
            if speed <= upper_speed:
                measured = _nonnegative(self._gate.required_clearance_m(speed, measured_brake))
                return None if measured is None else max(model, measured)
        return None

    def _safe_speed(self, clearance: float) -> float:
        model_cap = _nonnegative(self._gate.max_safe_speed_mps(clearance)) or 0.0
        if not self.measured_brake_points:
            return model_cap

        # Each bin certifies all speeds above the previous bound through its upper edge.
        # Use its measured upper distance across the bin and intersect with the model cap.
        safe_speed, lower_edge = 0.0, 0.0
        for upper_edge, measured_brake in self.measured_brake_points:
            available = clearance - self.margin_m - measured_brake
            if available < 0.0 or lower_edge >= model_cap:
                break
            within_bin = min(upper_edge, model_cap)
            if self.response_delay_s > 0.0:
                within_bin = min(within_bin, available / self.response_delay_s)
            if within_bin > lower_edge:
                safe_speed = within_bin
            lower_edge = upper_edge
        return safe_speed

    def evaluate(self, target_speed_mps, current_speed_mps, reliable_clearance_m) -> StoppingDecision:
        target = _nonnegative(target_speed_mps)
        current = _nonnegative(current_speed_mps)
        clearance = _nonnegative(reliable_clearance_m)
        required = self._required(target)
        current_required = self._required(current)
        remaining = None
        safe_speed = 0.0
        limit = 0.0
        if clearance is not None:
            safe_speed = self._safe_speed(clearance)
            scaled = safe_speed * 100.0
            centi_speed = math.floor(scaled + 1e-12) if math.isfinite(scaled) else 0
            limit = centi_speed / 100.0
            limit_required = self._required(limit)
            if centi_speed and (limit_required is None or limit_required > clearance):
                # Recheck the adjacent lower step when floating-point roundoff
                # places the computed limit directly on a clearance boundary.
                lower = (centi_speed - 1) / 100.0
                lower_required = self._required(lower)
                limit = lower if lower_required is not None and lower_required <= clearance else 0.0
            if current_required is not None:
                remaining = clearance - current_required

        brake_room = remaining is not None and remaining >= 0.0
        command = 0.0
        if target is None or current is None or clearance is None:
            action, reason = "brake", "STOPPING_INPUT_INVALID"
        elif current_required is None:
            action, reason = "brake", "CURRENT_BRAKE_UNCALIBRATED"
        elif current > 0.0 and not brake_room:
            action, reason = "brake", "CURRENT_STOPPING_CLEARANCE_INSUFFICIENT"
        elif current > limit:
            action, reason = "brake", "CURRENT_SPEED_EXCEEDS_LIMIT"
        elif required is None or target > limit:
            action = "limit"
            reason = "TARGET_BRAKE_UNCALIBRATED" if required is None else "TARGET_SPEED_EXCEEDS_LIMIT"
            command = limit
        else:
            action, reason = "allow", "WITHIN_STOPPING_LIMIT"
            command = target
        return StoppingDecision(
            target, current, clearance, required, current_required, remaining, safe_speed,
            limit, command, action == "allow", brake_room, action, reason, self,
        )
