from yopo_swarm_core.plan import hold_traj
from yopo_l0.board_peers import PeerGuard


def test_shared_space_waits_for_a_measured_hold_from_both_peers():
    with (
        PeerGuard("uav0", 10, ("127.0.0.1", 0), {}) as first,
        PeerGuard("uav1", 11, ("127.0.0.1", 0), {"uav0": first.endpoint}) as second,
        PeerGuard("uav2", 12, ("127.0.0.1", 0), {"uav0": first.endpoint}) as third,
    ):
        first.set_endpoints({"uav1": second.endpoint, "uav2": third.endpoint})
        t = 100.0
        stay = hold_traj([0.0, 0.0, 2.0], t, t + 1.0)
        first.broadcast(stay, now_s=t, measured_at_s=t, position_enu_m=(0.0, 0.0, 2.0), holding=True)

        def send_hold(guard, y):
            position = (0.0, y, 2.0)
            guard.broadcast(
                hold_traj(position, t, t + 1.0), now_s=t, measured_at_s=t,
                position_enu_m=position, holding=True,
            )
            first.poll(t)

        send_hold(second, 10.0)
        denied = first.check(stay, now_s=t, brake_start_s=t + 0.3)
        assert (denied.allowed, denied.reason, denied.peer_id) == (False, "PEER_MISSING", "uav2")

        send_hold(third, -10.0)
        decision = first.check(stay, now_s=t, brake_start_s=t + 0.3)
        assert decision.allowed


def test_unheard_conflicting_replan_is_blocked_before_the_brake_tail_finishes():
    from yopo_swarm_core.plan import quintic
    from yopo_swarm_core.trajectory import PiecewisePolynomial

    records = []
    other_records = []
    t = 100.0
    with (
        PeerGuard("uav0", 10, ("127.0.0.1", 0), {},
                  log=lambda event, **fields: records.append((event, fields))) as first,
        PeerGuard("uav1", 11, ("127.0.0.1", 0), {}) as second,
        PeerGuard("uav2", 12, ("127.0.0.1", 0), {},
                  log=lambda event, **fields: other_records.append((event, fields))) as third,
    ):
        first.set_endpoints({"uav1": second.endpoint, "uav2": third.endpoint})
        second.set_endpoints({"uav0": first.endpoint, "uav2": third.endpoint})
        third.set_endpoints({"uav0": first.endpoint, "uav1": second.endpoint})
        for guard, y in ((first, 0.), (second, 3.5), (third, -12.)):
            position = (0., y, 2.)
            guard.broadcast(
                hold_traj(position, t, t + 3.0), now_s=t, measured_at_s=t,
                position_enu_m=position, holding=True,
            )
        now = t + 0.06
        new_track = PiecewisePolynomial(now, [
            quintic([[0., 3.5, 2.], [0., 0., 0.], [0., 0., 0.]],
                    [[0., 0.8, 2.], [0., 0., 0.], [0., 0., 0.]], 1.5),
        ])
        seq = second.broadcast(new_track, now_s=now, measured_at_s=now,
                               position_enu_m=(0., 3.5, 2.), omit=("uav0",))
        first.poll(now)
        third.poll(now)
        assert any(event == "peer_rx" and row["peer_id"] == "uav1" and row["sequence"] == seq
                   for event, row in other_records)

        local_position = (0., 0., 2.)
        short = hold_traj(local_position, now, now + 0.30)
        assert first.check(short, now_s=now, brake_start_s=now + 0.1).allowed

        full = hold_traj(local_position, now, now + 1.50)
        decision = first.check(full, now_s=now, brake_start_s=short.t_end)
        assert (decision.allowed, decision.reason, decision.peer_id) == (False, "PEER_REACHABLE_SET", "uav1")
        event, log = records[-1]
        assert event == "peer_decision"
        assert log["brake_end_s"] == full.t_end
        assert log["peers"][0]["track_sequences"] == [0]
        assert log["peers"][0]["measured_at_s"] == t
        assert log["peers"][0]["reach_radius_at_brake_end_m"] > 5.0


def test_peer_reboot_blocks_immediately_and_recovers_from_fresh_verified_hold():
    t = 100.0
    with (
        PeerGuard("uav0", 10, ("127.0.0.1", 0), {}) as first,
        PeerGuard("uav2", 12, ("127.0.0.1", 0), {"uav0": first.endpoint}) as third,
    ):
        old_peer = PeerGuard("uav1", 11, ("127.0.0.1", 0), {"uav0": first.endpoint})
        source = old_peer.endpoint
        first.set_endpoints({"uav1": source, "uav2": third.endpoint})
        try:
            for guard, y in ((first, 0.), (old_peer, 10.), (third, -10.)):
                position = (0., y, 2.)
                guard.broadcast(
                    hold_traj(position, t, t + 2.0), now_s=t, measured_at_s=t,
                    position_enu_m=position, holding=True,
                )
            first.poll(t)
            assert first.availability(t).allowed
        finally:
            old_peer.close()

        now = t + 0.04
        position = (0., 10., 2.)
        with PeerGuard("uav1", 22, source, {"uav0": first.endpoint}) as restarted:
            track = hold_traj(position, now, now + 2.0)
            restarted.broadcast(track, now_s=now, measured_at_s=now, position_enu_m=position)
            first.poll(now)
            assert first.availability(now).reason == "PEER_SESSION_RESTART"
            assert not first.verify_rejoin("uav1", 22, now_s=now)

            after = now + 0.02
            restarted.broadcast(track, now_s=after, measured_at_s=after,
                                position_enu_m=position, holding=True)
            first.poll(after)
            assert not first.availability(after).allowed
            assert not first.verify_rejoin("uav1", 11, now_s=after)
            assert first.verify_rejoin("uav1", 22, now_s=after)
            assert first.availability(after).allowed


def test_duplicate_and_out_of_order_never_refresh_a_peer_and_coordinate_fault_blocks():
    import socket
    from yopo_l0.board_peers import PeerUpdate, encode_peer_datagram
    from yopo_l0.swarm_udp import PolyPacket

    t = 100.0
    events = []
    with (
        PeerGuard("uav0", 10, ("127.0.0.1", 0), {},
                  log=lambda event, **fields: events.append((event, fields))) as first,
        PeerGuard("uav2", 12, ("127.0.0.1", 0), {"uav0": first.endpoint}) as third,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
    ):
        sender.bind(("127.0.0.1", 0))
        first.set_endpoints({"uav1": sender.getsockname(), "uav2": third.endpoint})
        for guard, y in ((first, 0.), (third, -10.)):
            p = (0., y, 2.)
            guard.broadcast(hold_traj(p, t, t + 2.0), now_s=t, measured_at_s=t,
                            position_enu_m=p, holding=True)

        def update(seq, stamp, version=1):
            p = (0., 10., 2.)
            tr = hold_traj(p, t, t + 2.0)
            packet = PolyPacket(version, 11, "uav1", seq, stamp, t, tr.t_end, None, tr)
            sender.sendto(encode_peer_datagram(PeerUpdate(packet, stamp, p, True)), first.endpoint)
            first.poll(stamp)
            return events[-1][1]["reason"]

        assert update(2, t) is None
        assert update(2, t + .04) == "PEER_DUPLICATE"
        assert update(1, t + .06) == "PEER_OUT_OF_ORDER"
        assert first.availability(t + .07).allowed
        assert update(3, t + .08, version=2) == "PEER_COORD_VERSION_MISMATCH"
        assert first.availability(t + .08).reason == "PEER_COORD_VERSION_MISMATCH"
        assert update(2, t + .09) == "PEER_OUT_OF_ORDER"
        assert first.availability(t + .09).reason == "PEER_COORD_VERSION_MISMATCH"
        assert update(4, t + .10) is None
        assert first.availability(t + .10).allowed
        stale = first.availability(t + .35)
        assert (stale.allowed, stale.reason) == (False, "PEER_STALE")


def test_piecewise_jump_cannot_stand_in_for_a_verified_switch_or_braking_tail():
    import numpy as np
    from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

    t = 100.0
    with (
        PeerGuard("uav0", 10, ("127.0.0.1", 0), {}) as first,
        PeerGuard("uav1", 11, ("127.0.0.1", 0), {"uav0": first.endpoint}) as second,
        PeerGuard("uav2", 12, ("127.0.0.1", 0), {"uav0": first.endpoint}) as third,
    ):
        first.set_endpoints({"uav1": second.endpoint, "uav2": third.endpoint})
        for guard, y in ((first, 0.), (second, 10.), (third, -10.)):
            p = (0., y, 2.)
            guard.broadcast(hold_traj(p, t, t + 2.0), now_s=t, measured_at_s=t,
                            position_enu_m=p, holding=True)
        first.poll(t)
        jump = PiecewisePolynomial(t, [
            PolySegment(np.array([[0.0], [0.0], [2.0]]), 0.2),
            PolySegment(np.array([[1.0], [0.0], [2.0]]), 0.3),
        ])
        decision = first.check(jump, now_s=t, brake_start_s=t + 0.2)
        assert (decision.allowed, decision.reason) == (False, "INVALID_BRAKING_HORIZON")


def test_track_switch_retains_previous_occupancy_until_the_propagation_window_ends():
    t = 100.0
    with (
        PeerGuard("uav0", 10, ("127.0.0.1", 0), {}) as first,
        PeerGuard("uav1", 11, ("127.0.0.1", 0), {"uav0": first.endpoint}) as second,
        PeerGuard("uav2", 12, ("127.0.0.1", 0), {"uav0": first.endpoint}) as third,
    ):
        first.set_endpoints({"uav1": second.endpoint, "uav2": third.endpoint})
        for guard, y in ((first, 0.), (second, 10.), (third, -10.)):
            p = (0., y, 2.)
            guard.broadcast(hold_traj(p, t, t + 2.0), now_s=t, measured_at_s=t,
                            position_enu_m=p, holding=True)
        first.poll(t)
        candidate = hold_traj((0., 0., 2.), t, t + .35)
        assert first.check(candidate, now_s=t, brake_start_s=t + .25).allowed

        first_switch = t + .02
        # A received plan and the independent observation disagree in this fault fixture.
        second.broadcast(hold_traj((0., 0., 2.), t, t + 2.0), now_s=first_switch,
                         measured_at_s=first_switch, position_enu_m=(0., 10., 2.))
        first.poll(first_switch)
        assert first.check(candidate, now_s=first_switch, brake_start_s=t + .25).reason == "PEER_TRACK_CONFLICT"

        next_switch = t + .04
        measured = (0., 10., 2.)
        second.broadcast(hold_traj(measured, t, t + 2.0), now_s=next_switch,
                         measured_at_s=next_switch, position_enu_m=measured, holding=True)
        first.poll(next_switch)
        assert first.check(candidate, now_s=t + .07, brake_start_s=t + .25).reason == "PEER_TRACK_CONFLICT"
        assert first.check(candidate, now_s=t + .10, brake_start_s=t + .25).allowed
