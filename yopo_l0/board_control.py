"""Single-authority control and the asynchronous planning handoff for the bench."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from threading import Lock
import time
from typing import TYPE_CHECKING

from .ctbr import FlatnessCTBRController

if TYPE_CHECKING:
    from .board_runtime import ControlOutput, ControlTick, JsonlLog


Vector3 = tuple[float, float, float]
ZERO = (0.0, 0.0, 0.0)
IDENTITY_ROTATION = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _finite_vector(value) -> bool:
    try:
        return len(value) == 3 and all(map(math.isfinite, value))
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class Odometry:
    stamp_ns: int
    position_enu_m: Vector3
    velocity_enu_mps: Vector3 = ZERO
    rotation_enu_flu: tuple[Vector3, Vector3, Vector3] = IDENTITY_ROTATION


@dataclass(frozen=True)
class ControlReference:
    position_enu_m: Vector3
    velocity_enu_mps: Vector3 = ZERO
    acceleration_enu_mps2: Vector3 = ZERO
    jerk_enu_mps3: Vector3 = ZERO
    yaw_rad: float = 0.0
    yaw_rate_radps: float = 0.0

    def __post_init__(self):
        if (not all(map(_finite_vector, (
            self.position_enu_m, self.velocity_enu_mps, self.acceleration_enu_mps2, self.jerk_enu_mps3,
        ))) or not math.isfinite(self.yaw_rad) or not math.isfinite(self.yaw_rate_radps)):
            raise ValueError("control reference must be finite")


@dataclass(frozen=True)
class BrakeTail:
    sample: Callable[[int], ControlReference]
    valid_from_ns: int
    valid_until_ns: int

    def __post_init__(self):
        if (not callable(self.sample) or type(self.valid_from_ns) is not int
                or type(self.valid_until_ns) is not int
                or not 0 <= self.valid_from_ns < self.valid_until_ns):
            raise ValueError("invalid verified brake validity window")


@dataclass(frozen=True)
class ControlPlan:
    sample: Callable[[int], ControlReference]
    valid_until_ns: int
    verified_brake: BrakeTail | None = None

    def __post_init__(self):
        if not callable(self.sample) or type(self.valid_until_ns) is not int or self.valid_until_ns <= 0:
            raise ValueError("invalid control plan")


@dataclass(frozen=True, eq=False)
class PlanTicket:
    """An in-process capability for one pending worker result."""

    generation: int
    sequence: int
    deadline_ns: int


class BoardControl:
    """Run control independently of planner or inference work.

    Mode, odometry and invalidation decisions belong to the control loop.
    Planner threads may only request a ticket and offer its result. Neither
    planning work nor reference sampling runs under the shared mailbox lock.
    """

    def __init__(self, *, log: JsonlLog, max_odometry_age_ns: int = 120_000_000):
        if max_odometry_age_ns <= 0:
            raise ValueError("odometry validity must be positive")
        self.log = log
        self.max_odometry_age_ns = max_odometry_age_ns
        self._lock = Lock()
        self._ticket: PlanTicket | None = None
        self._sequence = 0
        self._generation = 0
        self._plan: ControlPlan | None = None
        self._brake: BrakeTail | None = None
        self._requested = "PVA"
        self._shadow = False
        self._terminal = False
        self._reason = "INITIAL_HOLD"
        self._disposition: tuple[str, str] | None = None
        self._hold_reference: Vector3 | None = None
        self._last_ctbr_ns: int | None = None
        self._ctbr = FlatnessCTBRController()

    def request_mode(self, mode: str, *, shadow: bool = False, terminal: bool = False) -> bool:
        if mode not in ("PVA", "CTBR", "MANUAL") or ((shadow or terminal) and mode != "PVA"):
            raise ValueError("invalid control mode request")
        if self._terminal and mode == "CTBR":
            self.log("mode_request_rejected", mode=mode, reason="TERMINAL_LATCHED")
            return False
        if mode == self._requested and shadow == self._shadow and not (terminal and not self._terminal):
            return True
        reason = "TERMINAL_HANDOVER" if terminal else ("MANUAL_TAKEOVER" if mode == "MANUAL" else "MODE_REQUEST")
        with self._lock:
            self._requested, self._shadow = mode, shadow
            self._terminal |= terminal
            self._invalidate_locked(reason, preserve_brake=False)
        self._last_ctbr_ns = None
        self._ctbr.last_rates[:] = 0
        self.log("mode_requested", mode=mode, shadow=shadow, terminal_latched=self._terminal, reason=reason)
        return True

    def _invalidate_locked(self, reason: str, *, preserve_brake: bool) -> None:
        if self._plan is not None and preserve_brake:
            self._brake = self._plan.verified_brake
        elif not preserve_brake:
            self._brake = None
        self._generation += 1
        self._ticket = None
        self._plan = None
        self._reason = reason

    def invalidate(self, reason: str) -> None:
        with self._lock:
            self._invalidate_locked(reason, preserve_brake=True)
            generation = self._generation
        self.log("plan_invalidated", reason=reason, generation=generation)

    def begin_plan(self, deadline_ns: int) -> PlanTicket:
        if type(deadline_ns) is not int or deadline_ns <= 0:
            raise ValueError("planning deadline must be a positive monotonic timestamp")
        with self._lock:
            ticket = PlanTicket(self._generation, self._sequence, deadline_ns)
            self._sequence += 1
            self._ticket = ticket
        self.log("plan_requested", plan_sequence=ticket.sequence, generation=ticket.generation,
                 deadline_ns=deadline_ns)
        return ticket

    def offer_plan(self, ticket: PlanTicket, plan: ControlPlan, *, received_ns: int | None = None) -> bool:
        with self._lock:
            now_ns = time.monotonic_ns()
            received_ns = now_ns if received_ns is None else received_ns
            if now_ns >= ticket.deadline_ns or now_ns >= plan.valid_until_ns:
                reason = "PLAN_RESULT_EXPIRED"
            elif self._ticket is not ticket or self._requested == "MANUAL":
                reason = "PLAN_RESULT_SUPERSEDED"
            elif self._plan is not None and now_ns >= self._plan.valid_until_ns:
                self._invalidate_locked("PLAN_EXPIRED", preserve_brake=True)
                reason = "PLAN_EXPIRED"
            elif (self._brake is not None and
                  self._brake.valid_from_ns <= now_ns < self._brake.valid_until_ns):
                reason = "VERIFIED_BRAKE_ACTIVE"
            else:
                self._ticket = None
                self._plan = plan
                self._brake = None
                reason = None
        self.log("plan_adopted" if reason is None else "plan_result_rejected",
                 plan_sequence=ticket.sequence, generation=ticket.generation, reason=reason,
                 received_ns=received_ns, adopted_ns=now_ns if reason is None else None,
                 deadline_ns=ticket.deadline_ns, valid_until_ns=plan.valid_until_ns)
        return reason is None

    def _report_disposition(self, disposition: str, reason: str, tick: ControlTick) -> None:
        if self._disposition != (disposition, reason):
            self.log("control_disposition", disposition=disposition, reason=reason, sequence=tick.sequence)
            self._disposition = (disposition, reason)

    def _is_current(self, sample: Odometry | None, now_ns: int) -> bool:
        return (sample is not None and type(sample.stamp_ns) is int
                and 0 <= now_ns - sample.stamp_ns < self.max_odometry_age_ns
                and _finite_vector(sample.position_enu_m) and _finite_vector(sample.velocity_enu_mps)
                and sample.rotation_enu_flu is not None and len(sample.rotation_enu_flu) == 3
                and all(map(_finite_vector, sample.rotation_enu_flu)))

    def step(self, tick: ControlTick, odometry: Odometry | None) -> ControlOutput:
        from .board_runtime import ControlOutput

        now_ns = tick.now_ns
        with self._lock:
            requested, shadow = self._requested, self._shadow
            if requested == "MANUAL":
                authority, disposition, reason = "MANUAL", "relinquished", self._reason
                reference, valid_until = None, None
            elif not self._is_current(odometry, now_ns):
                if self._reason != "LOCALIZATION_UNAVAILABLE" or self._plan is not None or self._ticket is not None:
                    self._invalidate_locked("LOCALIZATION_UNAVAILABLE", preserve_brake=False)
                self._hold_reference = None
                authority, disposition, reason = "NONE", "relinquished", "LOCALIZATION_UNAVAILABLE"
                reference, valid_until = None, None
            else:
                if self._ticket is not None and now_ns >= self._ticket.deadline_ns:
                    self._invalidate_locked("PLAN_REQUEST_EXPIRED", preserve_brake=True)
                elif self._plan is not None and now_ns >= self._plan.valid_until_ns:
                    self._invalidate_locked("PLAN_EXPIRED", preserve_brake=True)
                if self._brake is not None and not (
                    self._brake.valid_from_ns <= now_ns < self._brake.valid_until_ns
                ):
                    self._brake = None
                plan, brake, reason = self._plan, self._brake, self._reason
                if plan is not None:
                    authority, disposition = requested, "tracking"
                    reason = "TERMINAL_HANDOVER" if self._terminal else "PLAN_ACTIVE"
                    reference, valid_until = plan.sample, plan.valid_until_ns
                elif brake is not None:
                    authority, disposition = requested, "verified_brake"
                    reference, valid_until = brake.sample, brake.valid_until_ns
                elif math.hypot(*odometry.velocity_enu_mps) <= 0.05 and requested == "PVA":
                    authority, disposition = "PVA", "hold"
                    if self._hold_reference is None:
                        self._hold_reference = tuple(odometry.position_enu_m)
                    reference = ControlReference(self._hold_reference)
                    valid_until = None
                else:
                    authority, disposition = "NONE", "relinquished"
                    reference, valid_until = None, None
        self._report_disposition(disposition, reason, tick)
        if authority in ("NONE", "MANUAL"):
            self._last_ctbr_ns = None
            self._ctbr.last_rates[:] = 0
            return ControlOutput(authority, {"kind": "authority_only", "reason": reason},
                                 disposition=disposition, reason=reason)

        if disposition != "hold":
            self._hold_reference = None
        if callable(reference):
            reference = reference(now_ns)
        assert odometry is not None
        valid_until_ns = odometry.stamp_ns + self.max_odometry_age_ns
        if valid_until is not None:
            valid_until_ns = min(valid_until_ns, valid_until)
        position = tuple(float(value) for value in odometry.position_enu_m)
        speed = math.hypot(*reference.velocity_enu_mps)
        if authority == "CTBR" or shadow:
            dt = ((now_ns - self._last_ctbr_ns) / 1e9 if self._last_ctbr_ns is not None
                  else 1.0 / tick.config.control_hz)
            command = self._ctbr.command(
                reference.position_enu_m, reference.velocity_enu_mps,
                reference.acceleration_enu_mps2, reference.jerk_enu_mps3,
                reference.yaw_rad, reference.yaw_rate_radps, odometry.position_enu_m,
                odometry.velocity_enu_mps, odometry.rotation_enu_flu, dt=dt,
            )
            fields = command.fields(now_ns / 1e9)
            self._last_ctbr_ns = now_ns
            if authority == "CTBR":
                return ControlOutput(authority, {"kind": "ctbr", **fields}, valid_until_ns,
                                     disposition, position, odometry.stamp_ns, speed, reason)
            self.log("ctbr_shadow", sequence=tick.sequence, ctbr=fields, position_enu_m=position,
                     position_stamp_ns=odometry.stamp_ns)
        else:
            self._last_ctbr_ns = None
        command = {
            "kind": "hold" if disposition == "hold" else "pva",
            "position_enu_m": [float(value) for value in reference.position_enu_m],
            "velocity_enu_mps": [float(value) for value in reference.velocity_enu_mps],
            "acceleration_enu_mps2": [float(value) for value in reference.acceleration_enu_mps2],
            "yaw_rad": float(reference.yaw_rad),
        }
        return ControlOutput(authority, command, valid_until_ns, disposition, position,
                             odometry.stamp_ns, speed, reason)
