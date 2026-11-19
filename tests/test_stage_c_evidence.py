"""Receiver and independently sampled truth are required for 900 m credit."""

import json

import pytest

from yopo_l0.stage_c_evidence import ctbr_distance, recompute


_STEP = 200_000_000


def _evidence(samples):
    commands, truth = [], []
    for index, (time_index, mode, epoch, odometry, judge) in enumerate(samples):
        receive_ns = 1_000_000_000 + time_index * _STEP
        stamp_ns = receive_ns
        commands.append({
            "event": "command_received", "board_id": "uav1", "input_id": "sim-odometry/uav1",
            "session_id": "first", "authority_epoch": epoch, "sequence": index,
            "authority": mode, "command": {
                "kind": "ctbr", "timestamp": receive_ns // 1000, "roll": 0., "pitch": 0.,
                "yaw": 0., "thrust_body": [0., 0., -.5], "reset_integral": False,
            } if mode == "CTBR" else {"kind": "pva"},
            "received_ns": receive_ns, "valid_until_ns": receive_ns + 250_000_000,
            "position_stamp_ns": stamp_ns, "position_enu_m": [odometry, 0.0, 2.0],
        })
        if judge is not None:
            truth.append({"board_id": "uav1", "source": "sim-plant-truth/uav1",
                          "stamp_ns": stamp_ns, "position_enu_m": [judge, 0.0, 2.0]})
    return commands, truth


def _section(pos_max, origin=0.0, mode="CTBR", epoch=1, start=0):
    count = int(pos_max / .5)
    return [(start + index, mode, epoch, origin + index * .5, origin + index * .5)
            for index in range(count + 1)]


@pytest.mark.parametrize(("case", "expected"), [
    ("both", True),
    ("pva_then_one_ctbr", False),
    ("two_segments", False),
    ("odometry_only", False),
    ("different_times", False),
    ("sampling_gap", False),
    ("expired_command_gap", False),
    ("pre_handover_pose", False),
    ("missing_truth", False),
])
def test_horizontal_net_distance_is_joint_and_never_stitches(case, expected):
    if case in {"both", "sampling_gap", "expired_command_gap", "pre_handover_pose", "missing_truth"}:
        samples = _section(900)
        if case == "sampling_gap":
            samples = [row if row[0] < 900 else (row[0] + 1, *row[1:]) for row in samples]
        if case == "missing_truth":
            samples[900] = (*samples[900][:-1], None)
    elif case == "pva_then_one_ctbr":
        samples = _section(900, mode="PVA") + [(1801, "CTBR", 2, 900.0, 900.0)]
    elif case == "two_segments":
        samples = (_section(600) + [(1201, "PVA", 2, 600.0, 600.0)]
                   + _section(600, origin=600.0, epoch=3, start=1202))
    else:
        samples = [(i, "CTBR", 1, min(i, 1800) * .5, min(i, 1200) * .5) for i in range(1801)]
        if case == "different_times":
            samples += [(i, "CTBR", 1, 900 - (i - 1800) * .5, 600 + (i - 1800) * .5)
                        for i in range(1801, 2401)]
    commands, truth = _evidence(samples)
    if case == "expired_command_gap":
        commands[899]["valid_until_ns"] = commands[899]["received_ns"] + 100_000_000
    if case == "pre_handover_pose":
        commands[0]["position_stamp_ns"] -= 10
        truth[0]["stamp_ns"] -= 10
    scored = ctbr_distance(commands, [], truth, board_id="uav1")
    assert scored["passed_900_m"] is expected
    if expected:
        assert scored["joint_net_m"] == pytest.approx(900.0)
    else:
        assert scored["joint_net_m"] < 900.0



def test_missing_boards_and_malformed_archive_are_not_a_pass(tmp_path):
    (tmp_path / "manifest.json").write_text(json.dumps({"boards": [], "scenario": "normal"}))
    (tmp_path / "judge.jsonl").write_text('{"board_id": "uav1"}\nnot-json\n')
    result = recompute(tmp_path)
    assert not result["all_boards_900_m"]
    assert not result["software_scenario_observed"]
    assert "manifest.json:expected-three-board-identities" in result["missing_evidence"]
    assert "judge.jsonl" in result["missing_evidence"]
    assert "uav2/flight_controller.jsonl" in result["missing_evidence"]


@pytest.mark.parametrize(("change", "expected"), [
    ("brake", True), ("restart", False), ("manual", False), ("deadline", False),
])
def test_continuous_actual_ctbr_counts_across_brake_permission_only(change, expected):
    commands, truth = _evidence(_section(900))
    before, after = commands[899:901]
    fence = after["received_ns"]
    authority = [{
        "event": "command_invalidated", "invalidated_ns": fence,
        **{field: before[field] for field in ("board_id", "session_id", "authority_epoch", "sequence")},
    }]
    for row in commands[900:]:
        row["authority_epoch"] = 2
        row["disposition"] = "verified_brake"
        if change == "restart":
            row["session_id"] = "second"
    if change == "manual":
        authority = [{
            "event": "authority_changed", "board_id": "uav1", "authority": "MANUAL",
            "received_ns": fence - 10,
        }]
    if change == "deadline":
        authority[0]["invalidated_ns"] -= 10
    assert ctbr_distance(commands, authority, truth, board_id="uav1")["passed_900_m"] is expected
