"""Exercise the three real board subprocesses and the archived-result entry."""

import json
import subprocess
import sys

import pytest


def _run(out, case):
    subprocess.run([sys.executable, "-m", "yopo_l0.stage_c_sim", "run",
                    "--out", str(out), "--scenario", case], check=True, timeout=35)
    return json.loads((out / "result.json").read_text())


def test_three_process_receiver_commands_drive_independent_judge(tmp_path):
    run = tmp_path / "normal"
    result = _run(run, "normal")
    assert result["missing_evidence"] == []
    assert result["software_scenario_observed"]
    assert not result["all_boards_900_m"]
    for board in result["boards"].values():
        assert board["ctbr_received"] >= 5
        assert board["joint_net_m"] > .01
        assert board["paired_samples"] >= 5
    for board_id in ("uav1", "uav2", "uav3"):
        log = [json.loads(line) for line in (run / board_id / "board.jsonl").read_text().splitlines()]
        selected = [row["target_speed_mps"] for row in log if row["event"] == "forward_plan"]
        assert selected and all(speed == pytest.approx(.45) for speed in selected)
    judge = [json.loads(line) for line in (run / "judge.jsonl").read_text().splitlines()]
    assert {row["board_id"] for row in judge if row["consumed_authority"] == "CTBR"} == {
        "uav1", "uav2", "uav3",
    }
    reevaluated = tmp_path / "recomputed.json"
    subprocess.run([sys.executable, "-m", "yopo_l0.stage_c_evidence", str(run),
                    "--out", str(reevaluated)], check=True, timeout=10)
    assert json.loads(reevaluated.read_text()) == result


@pytest.mark.parametrize(("case", "board", "reason"), [
    ("all-blind", "uav1", "ALL_BLIND"),
    ("inference-timeout", "uav1", "deadline_exceeded"),
    ("one-way-drop", "uav1", "PEER_REACHABLE_SET"),
    ("restart", "uav1", "PEER_SESSION_RESTART"),
])
def test_three_process_fault_is_observed_at_receiver_or_peer_gate(tmp_path, case, board, reason):
    result = _run(tmp_path / case, case)
    assert result["missing_evidence"] == []
    assert any(row["event"] == "fault_injected" for row in result["faults"])
    assert reason in result["boards"][board]["fault_reasons"]
    assert result["software_scenario_observed"], result["fault_observations"]
