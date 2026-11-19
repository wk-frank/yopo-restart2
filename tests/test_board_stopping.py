import io
import json
import math
import socket
from threading import Event, Thread

import pytest

from yopo_l0.board_control import BoardControl, BrakeTail, ControlPlan, ControlReference, Odometry
from yopo_l0.board_receiver import SimFlightController
from yopo_l0.board_runtime import BoardBackend, BoardConfig, JsonlLog, run_board
from yopo_l0.board_stopping import StoppingConstraints


def test_stop_distance_worked_example_gates_new_plan_and_execution():
    constraints = StoppingConstraints()
    allowed = constraints.evaluate(2.74, 0.0, 3.0)
    denied = constraints.evaluate(2.75, 0.0, 3.0)
    too_fast = constraints.evaluate(3.0, 0.0, 3.0)

    assert allowed.safe_speed_mps == pytest.approx(2.747950, abs=5e-7)
    assert allowed.speed_limit_mps == 2.74
    assert allowed.plan_allowed and allowed.command_speed_mps == 2.74
    assert not denied.plan_allowed and denied.command_speed_mps == 2.74
    assert denied.required_clearance_m == pytest.approx(3.003125)
    assert too_fast.required_clearance_m == pytest.approx(3.40)
    assert too_fast.evidence()["reliable_clearance_m"] == 3.0

    moving = constraints.evaluate(2.74, 3.0, 3.0)
    assert moving.action == "brake" and moving.command_speed_mps == 0.0
    assert moving.remaining_stop_margin_m == pytest.approx(-0.40)
    assert not moving.plan_allowed and not moving.brake_room_verified


def test_measured_envelope_caps_between_bins_and_refuses_uncalibrated_motion():
    limits = StoppingConstraints(measured_brake_points=((1.0, 0.4), (2.0, 1.3), (3.0, 3.2)))

    assert limits.evaluate(2.0, 1.0, 3.0).plan_allowed
    between = limits.evaluate(2.01, 1.0, 3.0)
    assert between.required_clearance_m == pytest.approx(4.2015)
    assert between.speed_limit_mps == 2.0 and not between.plan_allowed

    overspeed = limits.evaluate(1.0, 3.01, 9.0)
    assert overspeed.action == "brake" and overspeed.command_speed_mps == 0.0
    assert overspeed.current_required_clearance_m is None
    assert overspeed.remaining_stop_margin_m is None and not overspeed.brake_room_verified
    assert overspeed.speed_limit_mps == 3.0


def test_revised_range_and_brake_curve_change_actual_receiver_commands():
    nominal = StoppingConstraints(measured_brake_points=(
        (1.0, 0.35), (2.0, 1.0), (2.74, 1.85), (3.0, 2.25),
    ))
    updated = StoppingConstraints(measured_brake_points=(
        (1.0, 0.35), (2.0, 1.2005), (2.74, 2.1), (3.0, 2.5),
    ))
    board_rows, received_rows, rejects = io.StringIO(), io.StringIO(), io.StringIO()
    stopped = Event()
    at = (0.0, 0.0, 1.0)

    def backend(_, log, _stopping, _backend_config):
        control = BoardControl(log=log)
        desired = (2.74, 0.0, 0.0)
        brake = ControlReference(at, acceleration_enu_mps2=(-2.0, 0.0, 0.0))

        def step(tick):
            if tick.sequence < 4:
                calibration, clearance, actual = nominal, 3.0, 1.0
            elif tick.sequence < 8:
                calibration, clearance, actual = nominal, 2.2, 1.0
            else:
                calibration, clearance, actual = updated, 2.2, 1.995
            decision = calibration.evaluate(math.hypot(*desired), actual, clearance)
            log("stopping_checked", sequence=tick.sequence, **decision.evidence())
            if decision.action == "brake":
                control.invalidate(decision.reason)
            else:
                adjusted = decision.limit_velocity(desired)
                assert calibration.evaluate(math.hypot(*adjusted), actual, clearance).plan_allowed
                reference = ControlReference(at, adjusted)
                ticket = control.begin_plan(tick.now_ns + 50_000_000)
                tail = BrakeTail(lambda _: brake, tick.now_ns, tick.now_ns + 400_000_000)
                control.offer_plan(ticket, ControlPlan(lambda _: reference, tick.now_ns + 100_000_000, tail))
            return control.step(tick, Odometry(tick.now_ns, at, (actual, 0.0, 0.0)))

        return BoardBackend(step, control.invalidate)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as wire:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            board_endpoint = reservation.getsockname()
        wire.bind(("127.0.0.1", 0))
        wire.settimeout(1)
        config = BoardConfig("u1", "odom-u1", "fc-u1", board_endpoint, wire.getsockname(), {}, at)
        receiver = SimFlightController(config, wire, JsonlLog(received_rows), JsonlLog(rejects))
        process = Thread(target=run_board, args=(config, JsonlLog(board_rows), stopped),
                         kwargs={"backend_factory": backend}, daemon=True)
        process.start()
        try:
            for _ in range(40):
                command = receiver.receive()
                if command is not None and command["disposition"] == "verified_brake":
                    break
        finally:
            stopped.set()
            process.join(1)
        assert not process.is_alive()

    decisions = {row["sequence"]: row for row in map(json.loads, board_rows.getvalue().splitlines())
                 if row["event"] == "stopping_checked"}
    joined = [(msg, decisions[msg["sequence"]]) for msg in map(json.loads, received_rows.getvalue().splitlines())]
    for action, output_speed in (("allow", 2.74), ("limit", 2.0), ("brake", 0.0)):
        matching = [(msg, decision) for msg, decision in joined if decision["action"] == action]
        assert matching
        for msg, decision in matching:
            assert msg["authority"] == "PVA"
            assert msg["received_ns"] >= decision["monotonic_ns"]
            assert msg["reference_speed_mps"] == output_speed
            assert msg["command"]["velocity_enu_mps"] == [output_speed, 0.0, 0.0]

    brake_rx, brake_check = next((msg, decision) for msg, decision in joined if decision["action"] == "brake")
    assert brake_check["measured_brake_points"][1] == [2.0, 1.2005]
    assert brake_check["remaining_stop_margin_m"] == pytest.approx(0.00025)
    assert brake_check["brake_room_verified"]
    assert brake_rx["disposition"] == "verified_brake"
    assert brake_rx["command"]["acceleration_enu_mps2"] == [-2.0, 0.0, 0.0]
