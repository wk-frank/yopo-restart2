"""Exercise stage C control authority at the UDP flight-control receiver."""

import json
import subprocess
import sys


def _run(tmp_path, scenario):
    out = tmp_path / scenario
    subprocess.run(
        [sys.executable, "-m", "yopo_l0.stage_c_authority", "--out", str(out),
         "--scenario", scenario],
        check=True, timeout=25,
    )
    return out / "uav1"


def _rows(directory, name):
    return [json.loads(line) for line in (directory / name).read_text().splitlines()]


def test_receiver_has_one_authority_through_shadow_ctbr_terminal_and_manual(tmp_path):
    board = _run(tmp_path, "handover")
    receiver = _rows(board, "flight_controller.jsonl")
    changes = _rows(board, "flight_controller_authority.jsonl")
    runtime = _rows(board, "board.jsonl")

    transitions = [row for row in changes if row["event"] == "authority_changed"]
    assert [row["authority"] for row in transitions] == ["PVA", "CTBR", "PVA", "MANUAL"]
    manual = transitions[-1]
    assert manual["authority"] == "MANUAL"
    assert all(row["received_ns"] < manual["received_ns"] for row in receiver)
    for row in transitions[1:]:
        assert row["old_command_invalidated_ns"] <= row["received_ns"]
    shadow = [row for row in runtime if row["event"] == "ctbr_shadow"]
    assert len(shadow) >= 2
    assert all(isinstance(row["ctbr"]["thrust_body"], list) for row in shadow)
    shadow_sequences = {row["sequence"] for row in shadow}
    assert all(row["command"]["kind"] == "pva"
               for row in receiver if row["sequence"] in shadow_sequences)
    assert all("thrust_body" not in row["command"] for row in receiver if row["authority"] == "PVA")
    actual = [row for row in receiver if row["authority"] == "CTBR"]
    assert len(actual) >= 2
    assert all(len(row["command"]["thrust_body"]) == 3 for row in actual)
    assert all(row["reference_speed_mps"] > 0.0 for row in actual)
    assert all(len(row["position_enu_m"]) == 3 for row in actual)
    assert any(row["event"] == "mode_request_rejected" and row["reason"] == "TERMINAL_LATCHED"
               for row in runtime)



def test_expired_or_invalidated_plans_cannot_retake_authority(tmp_path):
    board = _run(tmp_path, "deadlines")
    runtime = _rows(board, "board.jsonl")
    receiver = _rows(board, "flight_controller.jsonl")
    changes = _rows(board, "flight_controller_authority.jsonl")
    blocked, = [row for row in runtime if row["event"] == "planner_blocked"]
    late, superseded = [row for row in runtime if row["event"] == "plan_result_rejected"]
    assert late["plan_sequence"] == blocked["plan_sequence"]
    assert late["reason"] == "PLAN_RESULT_EXPIRED"
    assert superseded["reason"] == "PLAN_RESULT_SUPERSEDED"
    assert superseded["received_ns"] < superseded["deadline_ns"]
    ticks = [row for row in runtime if row["event"] == "control_tick"
             and blocked["monotonic_ns"] < row["started_ns"] < late["received_ns"]]
    assert len(ticks) >= 8

    brake, *other = [row for row in receiver if row["disposition"] == "verified_brake"]
    assert blocked["deadline_ns"] <= brake["received_ns"] < late["received_ns"]
    assert brake["authority"] == "CTBR"
    assert brake["reference_speed_mps"] == 0.0
    assert len(other) >= 2
    assert any(row["event"] == "authority_changed" and row["authority"] == "NONE"
               and row["received_ns"] < late["received_ns"] for row in changes)

    localization, = [row for row in changes if row.get("reason") == "LOCALIZATION_UNAVAILABLE"]
    recovered, = [row for row in changes if row["event"] == "authority_changed"
                  and row["authority"] == "CTBR" and row["received_ns"] > localization["received_ns"]]
    assert localization["received_ns"] <= superseded["received_ns"] < recovered["received_ns"]
    assert not any(localization["received_ns"] <= row["received_ns"] < recovered["received_ns"]
                   for row in receiver)

    overrun, = [row for row in changes if row.get("reason") == "CONTROL_OVERRUN"]
    command_timeout, = [row for row in changes if row.get("reason") == "COMMAND_DEADLINE"
                        and row["received_ns"] > recovered["received_ns"]]
    assert command_timeout["invalidated_ns"] == command_timeout["prior_valid_until_ns"]
    assert command_timeout["invalidated_ns"] < overrun["received_ns"]
    assert not any(row["received_ns"] > overrun["received_ns"] for row in receiver)
