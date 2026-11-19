"""Exercise the wall-clock board entry and independent receiver logs."""

import json
import signal
import subprocess
import sys

import pytest


@pytest.fixture(scope="module")
def hold_run(tmp_path_factory):
    if not hasattr(signal, "SIGSTOP"):
        pytest.skip("requires POSIX process pause")
    run_dir = tmp_path_factory.mktemp("stage-c") / "run"
    subprocess.run(
        [sys.executable, "-m", "yopo_l0.stage_c_hold", "--out", str(run_dir)],
        check=True,
        timeout=20,
    )

    return run_dir


def _records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_three_board_hold_receiver_identity(hold_run):
    run_dir = hold_run
    receiver_sessions = set()
    receiver_endpoints = set()
    first_session_pids = set()
    for board in ("uav1", "uav2", "uav3"):
        config = json.loads((run_dir / board / "config.json").read_text())
        runtime = _records(run_dir / board / "board.jsonl")
        first_session_pids.add(runtime[0]["pid"])
        assert runtime[0]["event"] == "session_started"
        assert runtime[1]["event"] == "sim_input_bound"
        assert runtime[1]["source_kind"] == "static_position_fixture"
        assert runtime[1]["position_enu_m"] == config["hold_position_enu_m"]
        records = _records(run_dir / board / "flight_controller.jsonl")
        assert len(records) >= 3
        receiver_sessions.add(records[0]["session_id"])
        receiver_endpoints.add(tuple(config["flight_controller_endpoint"]))
        for record in records:
            assert record["event"] == "command_received"
            assert record["board_id"] == board
            assert record["input_id"] == f"sim-odometry/{board}"
            assert record["flight_controller_id"] == f"sim-fc/{board}"
            assert record["sent_ns"] <= record["received_ns"] < record["valid_until_ns"]
            assert record["command"]["kind"] == "hold"
            assert record["command"]["position_enu_m"] == config["hold_position_enu_m"]
    assert len(receiver_sessions) == len(receiver_endpoints) == len(first_session_pids) == 3



def test_pausing_one_process_ages_its_messages_without_pausing_peers(hold_run):
    events = {row["event"]: row for row in _records(hold_run / "faults.jsonl")}
    paused_ns = events["board_paused"]["monotonic_ns"]
    restarting_ns = events["board_restarting"]["monotonic_ns"]
    paused_board = events["board_paused"]["board_id"]
    assert paused_board == "uav2"

    paused_commands = _records(hold_run / paused_board / "flight_controller.jsonl")
    assert not any(paused_ns <= row["sent_ns"] < restarting_ns for row in paused_commands)
    sessions_before = {row["session_id"] for row in paused_commands if row["sent_ns"] < paused_ns}
    sessions_after = {row["session_id"] for row in paused_commands if row["sent_ns"] >= restarting_ns}
    assert len(sessions_before) == len(sessions_after) == 1
    assert sessions_before.isdisjoint(sessions_after)

    for board in ("uav1", "uav3"):
        commands = _records(hold_run / board / "flight_controller.jsonl")
        during_pause = [row for row in commands if paused_ns <= row["sent_ns"] < restarting_ns]
        assert len(during_pause) >= 5
        assert len({row["session_id"] for row in commands}) == 1
        runtime = _records(hold_run / board / "board.jsonl")
        ages = [
            row["peer_ages_ns"][paused_board]
            for row in runtime
            if row["event"] == "control_tick" and paused_ns + 50_000_000 <= row["started_ns"] < restarting_ns
        ]
        assert ages == sorted(ages)
        assert ages[-1] - ages[0] >= 100_000_000
        assert any(row["event"] == "heartbeat_received" and row["peer_id"] == paused_board
                   and row["peer_session_id"] in sessions_after for row in runtime)
