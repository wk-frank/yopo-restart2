import numpy as np
import pytest
from yopo_swarm_core.plan import hold_traj, quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial
from yopo_l0.swarm_udp import (
    DualTrackPeerTable,
    PolyPacket,
    StaggeredSlotScheduler,
    decode_poly_packet,
    encode_poly_packet,
)


def _sample_traj(t0: float, p0, p1, duration: float = 1.0) -> PiecewisePolynomial:
    head = np.array([p0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float)
    tail = np.array([p1, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float)
    return PiecewisePolynomial(t0, [quintic(head, tail, duration)])


def test_poly_packet_wire_size_under_350_bytes_and_crc_tamper_rejected():
    traj = _sample_traj(1.0, [0.0, 0.0, 2.0], [3.0, 0.5, 2.0])
    pkt = PolyPacket(
        coord_version=1,
        session_id=1001,
        owner="uav0",
        seq=7,
        t_gen=1.0,
        t_start=1.0,
        t_expire=5.0,
        terminal_goal=(940.0, -4.0, 2.5),
        traj=traj,
    )
    raw = encode_poly_packet(pkt)
    assert len(raw) < 350
    decoded = decode_poly_packet(raw)
    assert decoded.owner == "uav0" and decoded.seq == 7 and decoded.session_id == 1001
    assert np.allclose(decoded.traj.evaluate(1.5, 0), traj.evaluate(1.5, 0), atol=1e-4)

    tampered = bytearray(raw)
    tampered[12] ^= 0xFF
    with pytest.raises(ValueError, match="CRC32"):
        decode_poly_packet(bytes(tampered))


def test_staggered_slot_scheduler_assigns_non_overlapping_phases():
    sched = StaggeredSlotScheduler(("uav0", "uav1", "uav2"), replan_hz=10.0)
    assert sched.offsets_s["uav0"] == pytest.approx(0.0)
    assert sched.offsets_s["uav1"] == pytest.approx(1.0 / 30.0)
    assert sched.offsets_s["uav2"] == pytest.approx(2.0 / 30.0)
    assert sched.is_due("uav0", 1.000, None)
    assert not sched.is_due("uav1", 1.000, None)
    assert sched.is_due("uav1", 1.0 + 1.0 / 30.0, None)


def test_dual_track_union_and_asymmetric_link_loss_and_reboot_quarantine():
    table = DualTrackPeerTable("uav1", ("uav0", "uav1", "uav2"), coord_version=1, delay_union_s=0.05, stale_after_s=0.10)

    # First plan from uav0 holds at y = 0.0
    p1 = PolyPacket(1, 500, "uav0", 1, 1.00, 1.00, 4.00, None, hold_traj([5.0, 0.0, 2.0], 1.0, 3.0))
    assert table.ingest_raw(encode_poly_packet(p1), now_s=1.00)

    # Second plan at t=1.02 shifts uav0 to y = +3.0; within delay_union_s=0.05 BOTH y=0 and y=+3 are occupied!
    p2 = PolyPacket(1, 500, "uav0", 2, 1.02, 1.02, 4.00, None, hold_traj([5.0, 3.0, 2.0], 1.02, 3.0))
    assert table.ingest_raw(encode_poly_packet(p2), now_s=1.02)
    assert len(table.active_tubes_for_peer("uav0", 1.04)) == 2

    times = np.array([1.2, 1.4])
    pts_near_old = np.array([[5.0, 0.5, 2.0], [5.0, 0.5, 2.0]])
    ok_during_union, _ = table.check_candidate_clearance(times, pts_near_old, now_s=1.04, base_peer_sep_m=1.2)
    assert not ok_during_union  # Blocked by previous trajectory in Dual-Track Union window!

    # Out-of-order / stale seq=1 or wrong coord_version must be rejected.
    assert not table.ingest_raw(encode_poly_packet(p1), now_s=1.05)
    bad_coord = PolyPacket(2, 500, "uav0", 3, 1.05, 1.05, 4.00, None, hold_traj([5.0, 3.0, 2.0], 1.05, 3.0))
    assert not table.ingest_raw(encode_poly_packet(bad_coord), now_s=1.05)

    # Asymmetric link loss: at t=1.60 (0.58s since last packet), uav0's safety zone inflates.
    assert table.inflation_for_peer("uav0", 1.60) > 0.5
    pts_marginal = np.array([[5.0, 1.5, 2.0], [5.0, 1.5, 2.0]])  # 1.5m from y=3.0 (>1.2m base, but < inflated)
    ok_stale, _ = table.check_candidate_clearance(times, pts_marginal, now_s=1.60, base_peer_sep_m=1.2)
    assert not ok_stale

    # Reboot with a new session_id=501 must quarantine uav0 until explicitly re-acknowledged.
    reboot_pkt = PolyPacket(1, 501, "uav0", 1, 1.65, 1.65, 5.00, None, hold_traj([5.0, 3.0, 2.0], 1.65, 3.0))
    assert not table.ingest_raw(encode_poly_packet(reboot_pkt), now_s=1.65)
    assert "uav0" in table.quarantined_restarts
    table.acknowledge_restart("uav0", 501)
    assert table.ingest_raw(encode_poly_packet(reboot_pkt), now_s=1.66)
