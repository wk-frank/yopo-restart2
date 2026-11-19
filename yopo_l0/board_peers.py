"""Board-local UDP coordination using mapped common-clock seconds."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, field
import math
import socket
import struct
import time
import zlib

import numpy as np

from yopo_swarm_core.collision import ConstantMargin, LinearMargin, Verdict, check_derivative_bound, check_separation
from yopo_swarm_core.plan import hold_traj
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment
from .swarm_udp import PolyPacket, StaggeredSlotScheduler, decode_poly_packet, encode_poly_packet

Endpoint = tuple[str, int]
_MAGIC = b"YSB1"
_OBSERVATION = struct.Struct("<4sd3dB")
_CRC = struct.Struct("<I")
_WIRE_LIMIT = 349
_STOP_TOL = 1e-5


@dataclass(frozen=True)
class PeerBudget:
    """Bounds per aircraft, except for pairwise clock error and network latency."""

    body_radius_m: float = 0.35
    localization_error_m: float = 0.10
    tracking_error_m: float = 0.10
    curve_sampling_error_m: float = 0.02
    peer_speed_bound_mps: float = 3.5
    clock_error_s: float = 0.005
    stale_after_s: float = 0.20
    delay_union_s: float = 0.05
    max_plan_window_s: float = 6.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(x) and x >= 0.0 for x in asdict(self).values()):
            raise ValueError("peer budgets must be finite and nonnegative")
        if min(self.body_radius_m, self.peer_speed_bound_mps, self.max_plan_window_s) <= 0.0:
            raise ValueError("body size, physical speed bound and plan window must be positive")
        if self.stale_after_s <= self.clock_error_s:
            raise ValueError("peer freshness must exceed the clock error bound")

    @property
    def base_separation_m(self) -> float:
        return 2.0 * (self.body_radius_m + self.localization_error_m
                      + self.tracking_error_m + self.curve_sampling_error_m)


@dataclass(frozen=True)
class PeerUpdate:
    packet: PolyPacket
    measured_at_s: float
    position_enu_m: tuple[float, float, float]
    holding: bool


def encode_peer_datagram(update: PeerUpdate) -> bytes:
    """Serialize an atomic plan and its independent measured-position sample."""
    body = _OBSERVATION.pack(_MAGIC, update.measured_at_s, *update.position_enu_m, update.holding)
    body += encode_poly_packet(update.packet)
    if len(body) + _CRC.size > _WIRE_LIMIT:
        raise ValueError("peer datagram exceeds the 349-byte budget")
    return body + _CRC.pack(zlib.crc32(body))


def decode_peer_datagram(raw: bytes) -> PeerUpdate:
    if not _OBSERVATION.size + _CRC.size < len(raw) <= _WIRE_LIMIT:
        raise ValueError("invalid peer datagram length")
    body = raw[:-_CRC.size]
    if zlib.crc32(body) != _CRC.unpack_from(raw, len(body))[0]:
        raise ValueError("peer datagram CRC mismatch")
    magic, measured_at, x, y, z, holding = _OBSERVATION.unpack_from(body)
    if (magic != _MAGIC or holding not in (0, 1)
            or not all(map(math.isfinite, (measured_at, x, y, z)))):
        raise ValueError("invalid peer datagram header")
    packet = decode_poly_packet(body[_OBSERVATION.size:])
    return PeerUpdate(packet, measured_at, (x, y, z), bool(holding))


@dataclass(frozen=True)
class PeerDecision:
    allowed: bool
    reason: str
    peer_id: str | None = None
    witness_s: float | None = None


@dataclass
class _Peer:
    current: PeerUpdate | None = None
    previous: PeerUpdate | None = None
    previous_until_s: float = -math.inf
    initial_hold_seen: bool = False
    pending: PeerUpdate | None = None
    retired_sessions: set[int] = field(default_factory=set)
    coordinate_fault: PeerUpdate | None = None


def _same_track(a: PiecewisePolynomial, b: PiecewisePolynomial) -> bool:
    return a.t_start == b.t_start and len(a.segments) == len(b.segments) and all(
        x.duration == y.duration and np.array_equal(x.coeffs, y.coeffs)
        for x, y in zip(a.segments, b.segments)
    )


def _hold_at(position, duration: float) -> PolySegment:
    return PolySegment(np.asarray(position, dtype=float).reshape(3, 1), duration)


def _pad_track(traj: PiecewisePolynomial, lo: float, hi: float) -> PiecewisePolynomial:
    starts_at = min(lo, traj.t_start)
    segments = list(traj.segments)
    if lo < traj.t_start:
        segments.insert(0, _hold_at(traj.evaluate(traj.t_start), traj.t_start - lo))
    if traj.t_end < hi:
        segments.append(_hold_at(traj.evaluate(traj.t_end), hi - traj.t_end))
    return PiecewisePolynomial(starts_at, segments)


def _continuous_position(traj: PiecewisePolynomial, tolerance_m: float) -> bool:
    return all(
        np.linalg.norm(before.evaluate_local(before.duration) - after.coeffs[:, 0]) <= tolerance_m
        for before, after in zip(traj.segments, traj.segments[1:])
    )


def _terminates_at_rest(traj: PiecewisePolynomial) -> bool:
    return all(float(np.linalg.norm(traj.evaluate(traj.t_end, order))) <= _STOP_TOL for order in (1, 2))


class PeerGuard:
    """Own a UDP endpoint, peer histories and the shared-space admission decision.

    The caller serializes operations. It supplies the complete local trajectory from
    the present through the end of braking and retains the last approved braking tail.
    """

    def __init__(
        self,
        board_id: str,
        session_id: int,
        bind: Endpoint,
        peers: Mapping[str, Endpoint],
        *,
        budget: PeerBudget = PeerBudget(),
        coord_version: int = 1,
        log: Callable[..., None] | None = None,
    ) -> None:
        self.board_id = board_id
        self.session_id = session_id
        self.coord_version = coord_version
        self.budget = budget
        self.log = log
        self._seq = 0
        self._local_hold_seen = False
        self._started = False
        self._link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._link.bind(bind)
            self._link.setblocking(False)
            self.endpoint: Endpoint = self._link.getsockname()
            self.set_endpoints(peers)
        except BaseException:
            self._link.close()
            raise

    def __enter__(self) -> PeerGuard:
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def close(self) -> None:
        self._link.close()

    def set_endpoints(self, peers: Mapping[str, Endpoint]) -> None:
        """Set peer endpoints before the session starts; useful with ephemeral host ports."""
        if self._started:
            raise RuntimeError("peer endpoints are fixed for the session")
        if self.board_id in peers or len(set(peers.values())) != len(peers) or self.endpoint in peers.values():
            raise ValueError("expected distinct local and peer endpoints")
        self._peers = dict(peers)
        self._owners = {addr: owner for owner, addr in peers.items()}
        self._tracks = {owner: _Peer() for owner in peers}
        self._schedule = StaggeredSlotScheduler(sorted((self.board_id, *peers)), 15.0)

    def next_broadcast_s(self, after_s: float) -> float:
        return self._schedule.next_fire_time(self.board_id, after_s)

    def _event(self, event: str, **fields) -> None:
        if self.log is not None:
            self.log(event, **fields)

    def broadcast(
        self,
        plan: PiecewisePolynomial,
        *,
        now_s: float,
        measured_at_s: float,
        position_enu_m: tuple[float, float, float],
        holding: bool = False,
        omit: tuple[str, ...] = (),
    ) -> int:
        """Fan out the plan through braking and publish the current independent measurement."""
        self._started = True
        if plan.t_start != plan.t0 or plan.t_end != plan.support_end():
            raise ValueError("wire plan must use the full polynomial support")
        seq = self._seq
        packet = PolyPacket(
            self.coord_version, self.session_id, self.board_id, seq, now_s,
            plan.t_start, plan.t_end, None, plan,
        )
        update = PeerUpdate(packet, measured_at_s, tuple(position_enu_m), holding)
        if reason := self._invalid_update(update, now_s):
            raise ValueError(reason)
        raw = encode_peer_datagram(update)
        serialized = decode_peer_datagram(raw)
        if reason := self._invalid_update(serialized, now_s):
            raise ValueError(reason)
        if not self._within_wire_error(plan, serialized.packet.traj):
            raise ValueError("WIRE_PLAN_ERROR_EXCEEDS_BUDGET")
        self._seq += 1
        self._local_hold_seen |= holding
        for peer, address in self._peers.items():
            fields = dict(peer_id=peer, peer_session_id=self.session_id, sequence=seq,
                          sent_s=now_s, endpoint=address, wire_hex=raw.hex())
            if peer in omit:
                self._event("peer_tx_dropped", **fields, reason="FAULT_INJECTION")
                continue
            try:
                self._link.sendto(raw, address)
            except OSError as error:
                self._event("peer_tx_error", **fields, reason=str(error))
            else:
                self._event("peer_tx", **fields)
        return seq

    def _within_wire_error(self, original: PiecewisePolynomial, decoded: PiecewisePolynomial) -> bool:
        timing_error = 0.
        for before, after in zip(original.segments, decoded.segments):
            timing_error += abs(before.duration - after.duration)
            powers = max(before.duration, after.duration) ** np.arange(before.coeffs.shape[1])
            coefficient_error = np.linalg.norm(np.abs(before.coeffs - after.coeffs) @ powers)
            displacement = coefficient_error + self.budget.peer_speed_bound_mps * timing_error
            if displacement > self.budget.curve_sampling_error_m + 1e-9:
                return False
        return True

    def _invalid_update(self, update: PeerUpdate, now_s: float) -> str | None:
        return self._invalid_observation(update, now_s) or self._invalid_plan(update, now_s)

    def _invalid_observation(self, update: PeerUpdate, now_s: float) -> str | None:
        packet = update.packet
        skew = self.budget.clock_error_s
        pose = update.position_enu_m
        if len(pose) != 3 or not all(map(math.isfinite, (now_s, update.measured_at_s, *pose))):
            return "INVALID_MEASURED_POSITION"
        if (not math.isfinite(packet.t_gen) or packet.t_gen > now_s + skew
                or update.measured_at_s > min(packet.t_gen, now_s) + skew):
            return "PEER_CLOCK_ERROR"
        if now_s - min(update.measured_at_s, packet.t_gen) + skew >= self.budget.stale_after_s:
            return "PEER_STALE"
        return None

    def _invalid_plan(self, update: PeerUpdate, now_s: float) -> str | None:
        packet = update.packet
        skew = self.budget.clock_error_s
        traj = packet.traj
        if (not traj.is_well_formed() or traj.t_end - traj.t_start > self.budget.max_plan_window_s
                or traj.t_start > packet.t_gen + skew or not _terminates_at_rest(traj)
                or not _continuous_position(traj, self.budget.curve_sampling_error_m + _STOP_TOL)):
            return "INVALID_PEER_PLAN"
        if now_s + skew >= packet.t_expire or now_s + skew >= traj.t_end:
            return "PEER_PLAN_EXPIRED"
        if update.holding:
            hold_allowance = self.budget.localization_error_m + self.budget.tracking_error_m
            if any(np.any(seg.coeffs[:, 1:] != 0)
                   or np.linalg.norm(seg.coeffs[:, 0] - update.position_enu_m) > hold_allowance
                   for seg in traj.segments):
                return "INVALID_PEER_HOLD"
        return None

    def poll(self, now_s: float) -> None:
        self._started = True
        for _ in range(32):
            try:
                raw, addr = self._link.recvfrom(_WIRE_LIMIT + 1)
            except BlockingIOError:
                break
            received_ns = time.monotonic_ns()
            owner = self._owners.get(addr)
            fields = dict(peer_id=owner, received_s=now_s, received_monotonic_ns=received_ns,
                          endpoint=addr, wire_hex=raw.hex())
            if owner is None:
                self._event("peer_rx_rejected", **fields, reason="UNKNOWN_PEER_ADDRESS")
                continue
            try:
                update = decode_peer_datagram(raw)
            except (ValueError, struct.error):
                self._event("peer_rx_rejected", **fields, reason="MALFORMED_OR_CRC")
                continue
            packet = update.packet
            fields.update(peer_session_id=packet.session_id, sequence=packet.seq,
                          coord_version=packet.coord_version, sent_s=packet.t_gen,
                          measured_at_s=update.measured_at_s, position_enu_m=update.position_enu_m,
                          holding=update.holding)
            reason = ("PEER_IDENTITY_MISMATCH" if packet.owner != owner
                      else self._invalid_observation(update, now_s))
            if reason is None:
                plan_reason = self._invalid_plan(update, now_s)
                reason = self._admit(owner, update, now_s, plan_reason=plan_reason)
                fields["plan_reason"] = plan_reason
            self._event("peer_rx_rejected" if reason else "peer_rx", **fields, reason=reason)

    @staticmethod
    def _sequence_fault(update: PeerUpdate, previous: PeerUpdate) -> str | None:
        if update.packet.seq == previous.packet.seq:
            return "PEER_DUPLICATE"
        if update.packet.seq < previous.packet.seq:
            return "PEER_OUT_OF_ORDER"
        if (update.measured_at_s < previous.measured_at_s
                or update.packet.t_gen < previous.packet.t_gen):
            return "PEER_OBSERVATION_TIME_REGRESSION"
        return None

    def _admit(
        self, owner: str, update: PeerUpdate, now_s: float, *, plan_reason: str | None,
    ) -> str | None:
        state = self._tracks[owner]
        packet = update.packet
        same_coordinates = packet.coord_version == self.coord_version
        if packet.session_id in state.retired_sessions:
            return "PEER_RETIRED_SESSION"

        current = state.current
        if current is not None and packet.session_id != current.packet.session_id:
            pending = state.pending
            if pending is not None and packet.session_id == pending.packet.session_id:
                if reason := self._sequence_fault(update, pending):
                    return reason
            else:
                latest = pending or state.coordinate_fault or current
                if packet.t_gen <= latest.packet.t_gen or update.measured_at_s < latest.measured_at_s:
                    return "PEER_OLD_SESSION"
            state.pending = update
            return "PEER_SESSION_RESTART" if same_coordinates else "PEER_COORD_VERSION_MISMATCH"

        if state.pending is not None:
            return "PEER_QUARANTINED"
        last = state.coordinate_fault or current
        if last is not None and last.packet.session_id == packet.session_id:
            if reason := self._sequence_fault(update, last):
                return reason
        if not same_coordinates:
            state.coordinate_fault = update
            return "PEER_COORD_VERSION_MISMATCH"

        if plan_reason is not None:
            return plan_reason
        state.coordinate_fault = None
        if current is not None and not _same_track(packet.traj, current.packet.traj):
            state.previous = current
            state.previous_until_s = now_s + self.budget.delay_union_s + self.budget.clock_error_s
        state.current = update
        state.initial_hold_seen |= update.holding
        return None

    def pending_session(self, owner: str) -> int | None:
        """Return the quarantined session observed at the configured peer endpoint."""
        pending = self._tracks[owner].pending
        return None if pending is None else pending.packet.session_id

    def verify_rejoin(self, owner: str, session_id: int, *, now_s: float) -> bool:
        """Admit a verified new session only with a fresh measured, compatible holding state."""
        state = self._tracks[owner]
        update = state.pending
        if update is None or update.packet.session_id != session_id:
            reason = "PEER_REJOIN_SESSION_MISMATCH"
        elif update.packet.coord_version != self.coord_version:
            reason = "PEER_COORD_VERSION_MISMATCH"
        else:
            reason = self._invalid_update(update, now_s)
            if reason is None and state.current is not None:
                if update.measured_at_s <= state.current.measured_at_s:
                    reason = "PEER_REJOIN_OLD_OBSERVATION"
            if reason is None and not update.holding:
                reason = "PEER_INITIAL_HOLD_MISSING"

        self._event("peer_rejoin_rejected" if reason else "peer_rejoined",
                    peer_id=owner, peer_session_id=session_id, verified_s=now_s, reason=reason,
                    measured_at_s=update.measured_at_s if update is not None else None,
                    sequence=update.packet.seq if update is not None else None)
        if reason is not None:
            return False
        assert update is not None and state.current is not None
        state.retired_sessions.add(state.current.packet.session_id)
        state.current = update
        state.pending = None
        state.previous = None
        state.coordinate_fault = None
        state.initial_hold_seen = True
        return True

    def _availability(self, owner: str, now_s: float) -> str | None:
        state = self._tracks[owner]
        if state.pending is not None:
            if state.pending.packet.coord_version != self.coord_version:
                return "PEER_COORD_VERSION_MISMATCH"
            return "PEER_SESSION_RESTART"
        if state.coordinate_fault is not None:
            return "PEER_COORD_VERSION_MISMATCH"
        if state.current is None:
            return "PEER_MISSING"
        if reason := self._invalid_update(state.current, now_s):
            return reason
        if not state.initial_hold_seen:
            return "PEER_INITIAL_HOLD_MISSING"
        return None

    def availability(self, now_s: float) -> PeerDecision:
        """Check member readiness before keeping an already-verified execution active."""
        if len(self._peers) != 2 or not self._local_hold_seen:
            return PeerDecision(False, "LOCAL_INITIAL_HOLD_MISSING")
        if not math.isfinite(now_s):
            return PeerDecision(False, "LOCAL_CLOCK_ERROR")
        for owner in self._peers:
            if reason := self._availability(owner, now_s):
                return PeerDecision(False, reason, owner)
        return PeerDecision(True, "ALL_PEERS_READY")

    def check(self, candidate: PiecewisePolynomial, *, now_s: float, brake_start_s: float) -> PeerDecision:
        """Verify switch, execution and braking intervals against both peer envelopes."""
        readiness = self.availability(now_s)
        evidence = []
        if not readiness.allowed:
            decision = readiness
        elif (not candidate.is_well_formed() or not candidate.t_start <= now_s < candidate.t_end
              or not candidate.t_start <= brake_start_s < candidate.t_end
              or candidate.t_end - now_s > self.budget.max_plan_window_s or not _terminates_at_rest(candidate)
              or not _continuous_position(candidate, _STOP_TOL)):
            decision = PeerDecision(False, "INVALID_BRAKING_HORIZON")
        elif check_derivative_bound(
            candidate, 1, self.budget.peer_speed_bound_mps,
            window=(now_s, candidate.t_end), max_nodes=128,
        ).verdict is not Verdict.SAFE:
            decision = PeerDecision(False, "LOCAL_PHYSICAL_SPEED_BOUND")
        else:
            decision = self._check_separation(candidate, now_s, evidence)
        trajectory = None
        if self.log is not None and candidate.is_well_formed():
            trajectory = dict(t0_s=candidate.t0, valid_from_s=candidate.t_start, segments=[
                dict(duration_s=segment.duration, coefficients_enu=segment.coeffs.tolist())
                for segment in candidate.segments
            ])
        self._event("peer_decision", checked_s=now_s if math.isfinite(now_s) else None,
                    brake_start_s=brake_start_s if math.isfinite(brake_start_s) else None,
                    brake_end_s=candidate.t_end if math.isfinite(candidate.t_end) else None,
                    budget=asdict(self.budget), trajectory=trajectory, peers=evidence, **asdict(decision))
        return decision

    def _check_separation(self, candidate: PiecewisePolynomial, now_s: float, evidence: list) -> PeerDecision:
        hi = candidate.t_end
        window = (now_s, hi)
        speed = self.budget.peer_speed_bound_mps
        skew = self.budget.clock_error_s
        base = self.budget.base_separation_m
        for peer_id, state in self._tracks.items():
            update = state.current
            assert update is not None
            tracks = [update]
            if state.previous is not None and now_s <= state.previous_until_s:
                tracks.append(state.previous)
            age_bound = max(0., now_s - update.measured_at_s + skew)
            margin = LinearMargin(now_s, base + speed * age_bound, speed)
            anchor = hold_traj(update.position_enu_m, now_s, hi)
            reachable = check_separation(candidate, anchor, margin, window=window, max_nodes=128)
            row = dict(peer_id=peer_id, measured_at_s=update.measured_at_s,
                       position_enu_m=update.position_enu_m, peer_session_id=update.packet.session_id,
                       track_sequences=[track.packet.seq for track in tracks],
                       age_bound_s=age_bound, reach_radius_at_brake_end_m=margin.at(hi) - base,
                       reachability=reachable.verdict.value, tracks=[])
            evidence.append(row)
            if reachable.verdict is not Verdict.SAFE:
                return PeerDecision(False, "PEER_REACHABLE_SET", peer_id, reachable.witness_time)
            for track in tracks:
                result = check_separation(
                    candidate, _pad_track(track.packet.traj, now_s, hi),
                    ConstantMargin(base + speed * skew), window=window, max_nodes=128,
                )
                row["tracks"].append(dict(sequence=track.packet.seq, verdict=result.verdict.value,
                                          lower_bound_m=result.lower_bound))
                if result.verdict is not Verdict.SAFE:
                    return PeerDecision(False, "PEER_TRACK_CONFLICT", peer_id, result.witness_time)
        return PeerDecision(True, "PEER_ENVELOPES_SEPARATED")
