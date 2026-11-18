"""Staggered one-way polynomial broadcast and Dual-Track Occupancy Union for 3-UAV Orin Nano."""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import zlib
import numpy as np

from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment
from .explore import _peer_tube_min_distance, _sample_positions

MAGIC = b"YSP1"
_HEADER_FMT = "<4sHIQIdddB3fB"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_SAMPLE_DT_S = 0.02


@dataclass(frozen=True)
class PolyPacket:
    coord_version: int
    session_id: int
    owner: str
    seq: int
    t_gen: float
    t_start: float
    t_expire: float
    terminal_goal: tuple[float, float, float] | None
    traj: PiecewisePolynomial


def encode_poly_packet(pkt: PolyPacket) -> bytes:
    """Pack a PiecewisePolynomial trajectory packet into < 350 bytes with CRC32."""
    owner_bytes = pkt.owner.encode("utf-8")
    if not (1 <= len(owner_bytes) <= 16):
        raise ValueError("owner id must be 1..16 UTF-8 bytes")
    segs = pkt.traj.segments
    if not (1 <= len(segs) <= 6):
        raise ValueError(f"unsupported segment count {len(segs)}")
    has_goal = 1 if pkt.terminal_goal is not None else 0
    gx, gy, gz = pkt.terminal_goal if pkt.terminal_goal is not None else (0.0, 0.0, 0.0)
    header = struct.pack(
        _HEADER_FMT,
        MAGIC,
        int(pkt.coord_version),
        int(pkt.seq),
        int(pkt.session_id),
        len(owner_bytes),
        float(pkt.t_gen),
        float(pkt.t_start),
        float(pkt.t_expire),
        has_goal,
        float(gx),
        float(gy),
        float(gz),
        len(segs),
    )
    parts = [header, owner_bytes]
    for seg in segs:
        c = np.asarray(seg.coeffs, dtype=np.float32)
        if c.ndim != 2 or c.shape[0] != 3 or not (1 <= c.shape[1] <= 6):
            raise ValueError(f"invalid segment coeffs shape {c.shape}")
        padded = np.zeros((3, 6), dtype="<f4")
        padded[:, : c.shape[1]] = c
        parts.append(struct.pack("<fB", float(seg.duration), int(c.shape[1])))
        parts.append(padded[:, : c.shape[1]].tobytes())
    body = b"".join(parts)
    crc = zlib.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("<I", crc)


def decode_poly_packet(raw: bytes) -> PolyPacket:
    """Verify CRC32 and unpack a PolyPacket."""
    if len(raw) < _HEADER_SIZE + 5:
        raise ValueError("packet too short")
    body, crc_bytes = raw[:-4], raw[-4:]
    (expected_crc,) = struct.unpack("<I", crc_bytes)
    if (zlib.crc32(body) & 0xFFFFFFFF) != expected_crc:
        raise ValueError("CRC32 mismatch")
    (
        magic,
        coord_version,
        seq,
        session_id,
        owner_len,
        t_gen,
        t_start,
        t_expire,
        has_goal,
        gx,
        gy,
        gz,
        n_segs,
    ) = struct.unpack(_HEADER_FMT, body[:_HEADER_SIZE])
    if magic != MAGIC:
        raise ValueError("bad packet magic")
    cursor = _HEADER_SIZE
    if len(body) < cursor + owner_len or owner_len < 1 or n_segs < 1:
        raise ValueError("malformed owner/segment header")
    owner = body[cursor : cursor + owner_len].decode("utf-8")
    cursor += owner_len
    if not (math.isfinite(t_gen) and math.isfinite(t_start) and math.isfinite(t_expire) and t_expire > t_start):
        raise ValueError("invalid packet timestamps")
    segs: list[PolySegment] = []
    for _ in range(n_segs):
        if len(body) < cursor + 5:
            raise ValueError("truncated segment header")
        dur, nc = struct.unpack("<fB", body[cursor : cursor + 5])
        cursor += 5
        if dur <= 0.0 or not math.isfinite(dur) or not (1 <= nc <= 6):
            raise ValueError("invalid segment metadata")
        n_bytes = 3 * nc * 4
        if len(body) < cursor + n_bytes:
            raise ValueError("truncated segment coefficients")
        coeffs = np.frombuffer(body[cursor : cursor + n_bytes], dtype="<f4").reshape(3, nc).astype(np.float64)
        cursor += n_bytes
        segs.append(PolySegment(coeffs, float(dur)))
    if cursor != len(body):
        raise ValueError("trailing bytes in packet")
    goal = (float(gx), float(gy), float(gz)) if has_goal else None
    traj = PiecewisePolynomial(float(t_start), segs)
    return PolyPacket(
        coord_version=int(coord_version),
        session_id=int(session_id),
        owner=owner,
        seq=int(seq),
        t_gen=float(t_gen),
        t_start=float(t_start),
        t_expire=float(t_expire),
        terminal_goal=goal,
        traj=traj,
    )


class StaggeredSlotScheduler:
    """Clock-synced phase-offset scheduler (e.g. 0 / 33.3 / 66.7 ms for 3 UAVs at 10 Hz)."""

    def __init__(self, members: tuple[str, ...] | list[str], replan_hz: float = 10.0) -> None:
        self.members = tuple(str(m) for m in members)
        if not self.members or len(set(self.members)) != len(self.members):
            raise ValueError("members must be non-empty and unique")
        if replan_hz <= 0.0 or not math.isfinite(replan_hz):
            raise ValueError("replan_hz must be positive")
        self.period_s = 1.0 / float(replan_hz)
        self.slot_s = self.period_s / len(self.members)
        self.offsets_s = {m: i * self.slot_s for i, m in enumerate(self.members)}

    def next_fire_time(self, drone_id: str, after_s: float) -> float:
        offset = self.offsets_s[drone_id]
        k = math.floor((float(after_s) - offset) / self.period_s) + 1
        return k * self.period_s + offset

    def is_due(self, drone_id: str, now_s: float, last_plan_s: float | None, *, jitter_tol_s: float = 0.008) -> bool:
        now_f = float(now_s)
        if last_plan_s is not None and now_f - float(last_plan_s) < self.period_s - jitter_tol_s:
            return False
        offset = self.offsets_s[drone_id]
        phase = (now_f - offset) % self.period_s
        dist = min(phase, self.period_s - phase)
        return dist <= jitter_tol_s


def _tube_from_traj(traj: PiecewisePolynomial) -> np.ndarray:
    t0 = float(traj.t_start)
    horizon = max(float(traj.t_end) - t0 + 0.04, 0.5)
    n = max(2, int(math.ceil(horizon / _SAMPLE_DT_S)) + 1)
    times = np.linspace(t0, t0 + horizon, n, dtype=np.float64)
    pts = _sample_positions(traj, times)
    return np.column_stack((times, pts))


@dataclass(frozen=True)
class PeerObservation:
    """Measured peer position in the shared frame at stamp_s (not a planned position)."""

    session_id: int
    coord_version: int
    stamp_s: float
    position: tuple[float, float, float]


class DualTrackPeerTable:
    """Maintain Dual-Track Occupancy Union and asymmetric link-loss inflation for peers."""

    def __init__(
        self,
        self_id: str,
        members: tuple[str, ...] | list[str],
        *,
        coord_version: int = 1,
        delay_union_s: float = 0.05,
        stale_after_s: float = 0.15,
        uncertainty_speed_mps: float = 1.5,
        max_uncertainty_m: float = 1.2,
        max_clock_skew_s: float = 0.05,
    ) -> None:
        self.self_id = str(self_id)
        self.members = tuple(str(m) for m in members if str(m) != self.self_id)
        if (not math.isfinite(stale_after_s) or stale_after_s <= 0.
                or not math.isfinite(max_clock_skew_s) or max_clock_skew_s < 0.):
            raise ValueError("invalid peer freshness or clock skew bound")
        self.coord_version = int(coord_version)
        self.delay_union_s = float(delay_union_s)
        self.stale_after_s = float(stale_after_s)
        self.uncertainty_speed_mps = float(uncertainty_speed_mps)
        self.max_uncertainty_m = float(max_uncertainty_m)
        self.max_clock_skew_s = float(max_clock_skew_s)
        self.sessions: dict[str, int] = {}
        self.last_seq: dict[str, int] = {}
        self.last_rx_s: dict[str, float] = {}
        self.expires: dict[str, float] = {}
        self.current_tube: dict[str, np.ndarray] = {}
        self.previous_tube: dict[str, tuple[float, np.ndarray]] = {}
        self.terminal_goals: dict[str, np.ndarray] = {}
        self.quarantined_restarts: set[str] = set()
        self.rejected_counts: dict[str, int] = {}

    def _reject(self, reason: str) -> bool:
        self.rejected_counts[reason] = self.rejected_counts.get(reason, 0) + 1
        return False

    def ingest_raw(self, raw: bytes, now_s: float) -> bool:
        try:
            pkt = decode_poly_packet(raw)
        except ValueError:
            return self._reject("MALFORMED_OR_CRC")
        return self.ingest(pkt, now_s)

    def ingest(self, pkt: PolyPacket, now_s: float) -> bool:
        now_f = float(now_s)
        if pkt.owner == self.self_id or pkt.owner not in self.members:
            return self._reject("UNKNOWN_OWNER")
        if pkt.coord_version != self.coord_version:
            return self._reject("COORD_VERSION_MISMATCH")
        if (not all(map(math.isfinite, (now_f, pkt.t_gen, pkt.t_start, pkt.t_expire)))
                or pkt.t_gen > now_f + self.max_clock_skew_s or pkt.t_expire <= max(now_f, pkt.t_start)):
            return self._reject("BAD_TIMESTAMP_OR_EXPIRED")
        known_session = self.sessions.get(pkt.owner)
        if known_session is not None and pkt.session_id != known_session:
            # Peer restarted with a new session ID -> quarantine new shared-space motion until re-verified.
            self.quarantined_restarts.add(pkt.owner)
            return self._reject("PEER_SESSION_RESTART")
        if pkt.owner in self.quarantined_restarts:
            return self._reject("PEER_QUARANTINED")
        prev_seq = self.last_seq.get(pkt.owner, -1)
        if pkt.seq <= prev_seq:
            return self._reject("STALE_OR_OUT_OF_ORDER_SEQ")
        if (any(not np.isfinite(seg.coeffs).all() or not math.isfinite(seg.duration) or seg.duration <= 0
                for seg in pkt.traj.segments)
                or (pkt.terminal_goal is not None and not np.isfinite(pkt.terminal_goal).all())):
            return self._reject("MALFORMED_TRAJECTORY")
        tube = _tube_from_traj(pkt.traj)
        if not np.isfinite(tube).all():
            return self._reject("MALFORMED_TRAJECTORY")
        if pkt.owner in self.current_tube:
            self.previous_tube[pkt.owner] = (now_f, self.current_tube[pkt.owner])
        self.sessions[pkt.owner] = pkt.session_id
        self.last_seq[pkt.owner] = pkt.seq
        self.last_rx_s[pkt.owner] = now_f
        self.expires[pkt.owner] = min(pkt.t_expire, pkt.t_gen + self.stale_after_s + self.max_clock_skew_s)
        self.current_tube[pkt.owner] = tube
        if pkt.terminal_goal is not None:
            self.terminal_goals[pkt.owner] = np.asarray(pkt.terminal_goal, dtype=np.float64)
        return True

    def acknowledge_restart(self, owner: str, new_session_id: int) -> None:
        self.quarantined_restarts.discard(owner)
        self.sessions[owner] = int(new_session_id)
        self.last_seq[owner] = -1
        self.current_tube.pop(owner, None)
        self.previous_tube.pop(owner, None)
        self.terminal_goals.pop(owner, None)
        self.last_rx_s.pop(owner, None)
        self.expires.pop(owner, None)

    def active_tubes_for_peer(self, owner: str, now_s: float) -> list[np.ndarray]:
        tubes: list[np.ndarray] = []
        cur = self.current_tube.get(owner)
        if cur is not None:
            tubes.append(cur)
        prev = self.previous_tube.get(owner)
        if prev is not None and float(now_s) - prev[0] <= self.delay_union_s:
            tubes.append(prev[1])
        return tubes

    def inflation_for_peer(self, owner: str, now_s: float) -> float:
        last = self.last_rx_s.get(owner)
        if last is None:
            return self.max_uncertainty_m
        age = max(0.0, float(now_s) - last)
        if owner in self.quarantined_restarts:
            return self.max_uncertainty_m
        if age <= self.stale_after_s:
            return 0.0
        return min(self.max_uncertainty_m, self.uncertainty_speed_mps * (age - self.stale_after_s))

    def check_candidate_clearance(
        self,
        times: np.ndarray,
        pts: np.ndarray,
        now_s: float,
        base_peer_sep_m: float,
        *,
        observations: dict[str, PeerObservation] | None = None,
        peer_speed_bound_mps: float | None = None,
    ) -> tuple[bool, float]:
        """Approve only a measured, bounded horizon, including an unreceived change of plan.

        base_peer_sep_m includes body, localization, tracking and chord error allowances.
        The speed bound applies to actual peer motion, including its fallback, for this horizon.
        """
        times = np.asarray(times, dtype=float)
        pts = np.asarray(pts, dtype=float)
        if (observations is None or peer_speed_bound_mps is None
                or not math.isfinite(peer_speed_bound_mps) or peer_speed_bound_mps <= 0
                or not math.isfinite(base_peer_sep_m) or base_peer_sep_m <= 0
                or not math.isfinite(now_s) or times.ndim != 1 or len(times) < 2
                or pts.shape != (len(times), 3) or not np.isfinite(pts).all()
                or not np.isfinite(times).all() or abs(times[0] - now_s) > 1e-9
                or np.any(np.diff(times) <= 0)):
            return False, -math.inf
        delta = np.diff(pts, axis=0)
        length2 = np.einsum("ij,ij->i", delta, delta)
        min_margin = math.inf
        for peer in self.members:
            tubes = self.active_tubes_for_peer(peer, now_s)
            obs = observations.get(peer)
            if (not tubes or peer in self.quarantined_restarts or now_s >= self.expires[peer]
                    or obs is None or obs.coord_version != self.coord_version
                    or obs.session_id != self.sessions[peer] or not math.isfinite(obs.stamp_s)
                    or obs.stamp_s > now_s + self.max_clock_skew_s
                    or now_s - obs.stamp_s > self.stale_after_s):
                return False, -math.inf
            anchor = np.asarray(obs.position, dtype=float)
            if anchor.shape != (3,) or not np.isfinite(anchor).all():
                return False, -math.inf
            # Bound every chord by the peer's reachable ball at the end of the interval.
            dot = np.einsum("ij,ij->i", anchor - pts[:-1], delta)
            frac = np.divide(dot, length2, out=np.zeros_like(dot), where=length2 > 0)
            closest = pts[:-1] + np.clip(frac, 0., 1.)[:, None] * delta
            radius = peer_speed_bound_mps * np.maximum(0., times[1:] - obs.stamp_s + self.max_clock_skew_s)
            margin = float(np.min(np.linalg.norm(closest - anchor, axis=1) - radius - base_peer_sep_m))
            if margin <= 0:
                return False, margin
            min_margin = min(min_margin, margin)
            d = _peer_tube_min_distance(times, pts, tubes)
            req = float(base_peer_sep_m) + self.inflation_for_peer(peer, now_s)
            min_margin = min(min_margin, d - req)
            if d <= req:
                return False, d - req
        return True, min_margin
