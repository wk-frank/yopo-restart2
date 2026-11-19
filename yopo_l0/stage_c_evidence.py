"""Recompute stage C results using only archived receiver and independent judge rows."""

from __future__ import annotations

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path


MAX_EVIDENCE_GAP_NS = 250_000_000
TARGET_M = 900.0
BOARDS = ("uav1", "uav2", "uav3")
HISTORICAL = {
    "source": "docs/research/orin-nano-3uav-d435.md",
    "aircraft": 6,
    "stage": "historical-lockstep-px4-gazebo",
    "odometry_net_range_m": [934.605, 934.932],
    "truth_net_range_m": [937.042, 937.386],
}


def _position(value):
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        if all(type(coordinate) in (float, int) and math.isfinite(coordinate) for coordinate in value):
            return value
    except OverflowError:
        return None
    return None


def _stamp(value):
    return type(value) is int and value >= 0


def _identity(command):
    return command.get("session_id"), command.get("authority_epoch"), command.get("sequence")


def _actual_ctbr(command) -> bool:
    if not isinstance(command, dict) or command.get("kind") != "ctbr":
        return False
    rates = [command.get(axis) for axis in ("roll", "pitch", "yaw")]
    return (_position(rates) is not None and _position(command.get("thrust_body")) is not None
            and _stamp(command.get("timestamp")) and type(command.get("reset_integral")) is bool)


def ctbr_distance(commands: list[dict], authority: list[dict], truth: list[dict], *, board_id: str) -> dict:
    """Same-clock paired position within a continuous receiver-active CTBR interval."""
    judges: dict[int, dict | None] = {}
    for row in truth:
        stamp = row.get("stamp_ns")
        if row.get("board_id") == board_id and _stamp(stamp):
            judges[stamp] = None if stamp in judges else row

    ended: dict[tuple, int] = {}
    for row in authority:
        if (row.get("board_id") == board_id and row.get("event") == "command_invalidated"
                and _stamp(row.get("invalidated_ns"))):
            key = _identity(row)
            if isinstance(key[0], str) and _stamp(key[1]) and _stamp(key[2]):
                ended[key] = min(ended.get(key, row["invalidated_ns"]), row["invalidated_ns"])

    interruptions = sorted(
        row["received_ns"] for row in authority
        if row.get("board_id") == board_id and row.get("event") == "authority_changed"
        and row.get("authority") in ("PVA", "NONE", "MANUAL") and _stamp(row.get("received_ns"))
    )
    anchor = None
    previous_ctbr = None
    interval_start = None
    last_receipt = -1
    last_pose_stamp = None
    best_joint = 0.0
    peak_pair = (0.0, 0.0)
    paired_samples = segments = absent_pairs = 0
    for row in commands:
        recv, deadline = row.get("received_ns"), row.get("valid_until_ns")
        key = _identity(row)
        if (row.get("event") != "command_received" or row.get("board_id") != board_id
                or not _stamp(recv) or not _stamp(deadline) or not last_receipt < recv < deadline
                or not isinstance(key[0], str) or not _stamp(key[1]) or not _stamp(key[2])):
            previous_ctbr = anchor = last_pose_stamp = None
            continue
        last_receipt = recv
        command = row.get("command")
        if row.get("authority") != "CTBR" or not _actual_ctbr(command):
            previous_ctbr = anchor = last_pose_stamp = None
            continue

        deadline = min(deadline, ended.get(key, deadline))
        if deadline <= recv:
            previous_ctbr = anchor = last_pose_stamp = None
            continue
        interruptions_seen = bisect_right(interruptions, recv)
        consecutive = (previous_ctbr is not None and key[0] == previous_ctbr[0]
                       and recv <= previous_ctbr[2] and recv - previous_ctbr[1] <= MAX_EVIDENCE_GAP_NS
                       and interruptions_seen == previous_ctbr[3])
        if not consecutive:
            interval_start = recv
            anchor = last_pose_stamp = None
        previous_ctbr = (key[0], recv, deadline, interruptions_seen)

        stamp = row.get("position_stamp_ns")
        if _stamp(stamp) and stamp < interval_start:
            continue  # odometry sampled before this CTBR interval
        if _stamp(stamp) and stamp == last_pose_stamp:
            continue  # controller reused the same valid odometry sample
        judge = judges.get(stamp) if _stamp(stamp) else None
        estimated = _position(row.get("position_enu_m"))
        independent = _position(judge.get("position_enu_m")) if judge else None
        if (estimated is None or independent is None or stamp > recv
                or not judge.get("source") or judge.get("source") == row.get("input_id")):
            absent_pairs += 1
            anchor = last_pose_stamp = None
            continue

        if (last_pose_stamp is None or not 0 < stamp - last_pose_stamp <= MAX_EVIDENCE_GAP_NS):
            anchor = (estimated, independent)
            segments += 1
        last_pose_stamp = stamp
        paired_samples += 1
        pair = (math.dist(estimated[:2], anchor[0][:2]), math.dist(independent[:2], anchor[1][:2]))
        joint = min(pair)
        if joint > best_joint:
            best_joint, peak_pair = joint, pair

    return {"passed_900_m": best_joint >= TARGET_M, "target_m": TARGET_M, "joint_net_m": best_joint,
            "paired_excursion_m": {"odometry": peak_pair[0], "independent_truth": peak_pair[1]},
            "paired_samples": paired_samples, "continuous_segments": segments,
            "missing_pairs": absent_pairs}


def _reject_constant(token):
    raise ValueError(f"invalid JSON numeric value {token}")


def _json_object(text: str) -> dict:
    row = json.loads(text, parse_constant=_reject_constant)
    if not isinstance(row, dict):
        raise ValueError("expected an object")
    return row


def _rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as stream:
        rows = [_json_object(line) for line in stream if line.strip()]
    if any(not isinstance(row.get("event"), str) or not _stamp(row.get("monotonic_ns")) for row in rows):
        raise ValueError("log row lacks an event or monotonic timestamp")
    return rows


def _timing(values: list[int]) -> dict:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "p95_ms": None, "p99_ms": None, "max_ms": None}
    n = len(ordered)
    return {
        "count": n, "p95_ms": ordered[math.ceil(n * .95) - 1] / 1e6,
        "p99_ms": ordered[math.ceil(n * .99) - 1] / 1e6, "max_ms": ordered[-1] / 1e6,
    }


def _scenario_check(case, streams, faults):
    if case == "normal":
        return {"three_boards_track": all(any(row.get("authority") == "CTBR"
                                             for row in streams[b][1]) for b in BOARDS)}
    start = next((row.get("timestamp_ns") for row in faults
                  if row.get("event") == "fault_injected" and row.get("fault") == case), None)
    if not _stamp(start):
        return {"fault_timestamp": False}

    def event(rows, name, **fields):
        return next((row for row in rows if row.get("event") == name
                     and _stamp(row.get("monotonic_ns")) and row["monotonic_ns"] >= start
                     and all(row.get(key) == value for key, value in fields.items())), None)

    def disposed(board, reason=None):
        return any(row.get("authority") == "CTBR" and row.get("disposition") == "verified_brake"
                   and _stamp(row.get("received_ns")) and row["received_ns"] > start
                   and (reason is None or row.get("reason") == reason) for row in streams[board][1])

    rows, _, authority = streams["uav1"]
    if case == "all-blind":
        invalidated = event(rows, "depth_invalidated", reason="ALL_BLIND")
        return {"all_blind_observed": bool(invalidated),
                "old_plan_revoked": bool(event(rows, "plan_invalidated", reason="ALL_BLIND")),
                "receiver_verified_brake": disposed("uav1", "ALL_BLIND"),
                "new_depth": bool(invalidated and any(
                    row.get("event") == "depth_observed" and row.get("monotonic_ns", 0) > invalidated["monotonic_ns"]
                    for row in rows))}
    if case == "inference-timeout":
        return {
            "inference_deadline": bool(event(rows, "inference_rejected", reason="deadline_exceeded")),
            "receiver_verified_brake": disposed("uav1", "PLAN_REQUEST_EXPIRED"),
            "control_ticked": sum(row.get("event") == "control_tick"
                                  and _stamp(row.get("started_ns"))
                                  and start <= row["started_ns"] < start + 320_000_000
                                  for row in rows) >= 8,
        }
    if case == "one-way-drop":
        drop = event(streams["uav2"][0], "peer_tx_dropped", peer_id="uav1")
        rejected = next((row for row in rows if drop and row.get("event") == "conflict_probe"
                         and row.get("reason") == "PEER_REACHABLE_SET" and row.get("peer_id") == "uav2"
                         and drop["monotonic_ns"] <= row.get("monotonic_ns", 0)
                         < start + 110_000_000), None)
        seq = drop.get("sequence") if drop else None
        return {"actual_one_way_omission": bool(drop and event(streams["uav3"][0], "peer_rx", peer_id="uav2",
                                                               sequence=seq)),
                "victim_did_not_receive": bool(drop and not event(rows, "peer_rx", peer_id="uav2", sequence=seq)),
                "candidate_denied": bool(rejected)}
    if case == "restart":
        rebooted = [row for row in streams["uav2"][0] if row.get("event") == "session_started"]
        return {"new_session": len({row.get("session_id") for row in rebooted}) >= 2,
                "receiver_expired_old_session": bool(event(streams["uav2"][2], "command_invalidated",
                                                             reason="COMMAND_DEADLINE")),
                "peer_quarantine": all(event(streams[board][0], "peer_rx_rejected", reason="PEER_SESSION_RESTART",
                                               peer_id="uav2") for board in ("uav1", "uav3")),
                "verified_rejoin": all(event(streams[board][0], "peer_rejoined", peer_id="uav2")
                                       for board in ("uav1", "uav3"))}
    return {"known_scenario": False}


def recompute(run: Path) -> dict:
    outputs = {}
    missing = []

    def document(relative):
        try:
            return _json_object((run / relative).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            missing.append(str(relative))
            return {}

    def read(relative, *, allow_empty=False):
        try:
            rows = _rows(run / relative)
            if rows or allow_empty:
                return rows
        except (OSError, ValueError):
            pass
        missing.append(str(relative))
        return []

    manifest = document("manifest.json")
    identities = manifest.get("boards")
    if not isinstance(identities, list) or sorted(map(str, identities)) != list(BOARDS):
        missing.append("manifest.json:expected-three-board-identities")
    scenario = manifest.get("scenario")
    if not isinstance(manifest.get("clock_mapping"), dict) or not manifest.get("build_commit"):
        missing.append("manifest.json:clock-or-build")
    truth = read(Path("judge.jsonl"))
    faults = read(Path("faults.jsonl"), allow_empty=scenario == "normal")
    streams = {}
    used_endpoints = set()
    used_inference_sockets = set()
    for board in BOARDS:
        prefix = Path(board)
        config = document(prefix / "config.json")
        backend = document(prefix / "backend.json")
        commands = read(prefix / "flight_controller.jsonl")
        authority = read(prefix / "flight_controller_authority.jsonl")
        board_log = read(prefix / "board.jsonl")
        service = read(prefix / "inference_service.jsonl")
        inputs = read(prefix / "input.jsonl")
        identities_ok = (config.get("board_id") == board
                         and config.get("input_id") == "sim-odometry/" + board
                         and config.get("flight_controller_id") == "sim-fc/" + board
                         and backend.get("camera") == "sim-D435/" + board)
        if not identities_ok:
            missing.append(f"{board}:bound-input-and-receiver-identities")
        for source, key in ((config, "bind"), (config, "flight_controller_endpoint"),
                            (backend, "sensor_bind"), (backend, "peer_bind")):
            endpoint = source.get(key)
            if (not isinstance(endpoint, list) or len(endpoint) != 2
                    or endpoint[0] != "127.0.0.1" or type(endpoint[1]) is not int
                    or not 0 < endpoint[1] <= 65_535 or tuple(endpoint) in used_endpoints):
                missing.append(f"{board}:invalid-or-reused-{key}")
            else:
                used_endpoints.add(tuple(endpoint))
        inference_socket = backend.get("infer_socket")
        if (not isinstance(inference_socket, str) or not inference_socket
                or inference_socket in used_inference_sockets):
            missing.append(f"{board}:invalid-or-reused-infer_socket")
        else:
            used_inference_sockets.add(inference_socket)
        archived_frames = [row.get("archive") for row in board_log
                           if row.get("event") in {"depth_observed", "depth_invalidated"}]
        if (not archived_frames or any(not isinstance(name, str) or Path(name).name != name
                                       or not (run / prefix / "depth" / name).is_file()
                                       for name in archived_frames)):
            missing.append(f"{board}:archived-depth-arrays")
        if any(row.get("board_id") != board or row.get("input_id") != config.get("input_id")
               or row.get("flight_controller_id") != config.get("flight_controller_id") for row in commands):
            missing.append(f"{board}:accepted-command-identities")
            commands = []
        if not any(row.get("board_id") == board and row.get("source") == "sim-plant-truth/" + board for row in truth):
            missing.append(f"{board}:independent-judge")
        distance = ctbr_distance(commands, authority, truth, board_id=board)
        streams[board] = (board_log, commands, authority)
        reasons = sorted({row["reason"] for row in board_log if isinstance(row.get("reason"), str)})
        timeline_events = {"control_disposition", "plan_invalidated", "control_output_revoked",
                           "plan_result_rejected", "candidate_declined", "peer_rejoined", "inference_rejected"}
        timeline = [row for row in board_log if row.get("event") in timeline_events]
        timeline += [row for row in authority if row.get("event") in {"authority_changed", "command_invalidated"}]
        timeline.sort(key=lambda row: row.get("received_ns") or row.get("monotonic_ns") or 0)
        tick_logs = [row for row in board_log if row.get("event") == "control_tick"]
        received_ns = [row["received_ns"] for row in commands if _stamp(row.get("received_ns"))]
        exposures = [row["received_ns"] - row["exposure_ns"] for row in commands
                     if _stamp(row.get("exposure_ns")) and _stamp(row.get("received_ns"))
                     and row["received_ns"] >= row["exposure_ns"]]
        outputs[board] = {
            **distance, "received_commands": len(commands),
            "ctbr_received": sum(row.get("authority") == "CTBR" for row in commands),
            "verified_brake_received": sum(row.get("disposition") == "verified_brake" for row in commands),
            "board_sessions": [row["session_id"] for row in board_log
                               if row.get("event") == "session_started" and "session_id" in row],
            "timing": {
                "control_compute": _timing([row["completed_ns"] - row["started_ns"] for row in tick_logs
                                            if _stamp(row.get("completed_ns")) and _stamp(row.get("started_ns"))]),
                "planning_compute": _timing([row["duration_ns"] for row in board_log
                                             if row.get("event") == "planning_compute" and _stamp(row.get("duration_ns"))]),
                "exposure_to_receiver": _timing(exposures),
                "longest_receiver_gap_ms": max((b-a for a, b in zip(received_ns, received_ns[1:])), default=0) / 1e6,
                "control_missed_periods": sum(row["missed_periods"] for row in tick_logs
                                              if _stamp(row.get("missed_periods"))),
            },
            "fault_reasons": reasons, "input_records": len(inputs), "inference_records": len(service),
            "timeline": timeline,
        }

    observations = _scenario_check(scenario, streams, faults)
    passed = not missing and all(value["passed_900_m"] for value in outputs.values())
    return {
        "schema": 1, "scenario": scenario, "boards": outputs,
        "faults": faults, "fault_observations": observations,
        "software_scenario_observed": not missing and all(observations.values()),
        "missing_evidence": missing, "all_boards_900_m": passed,
        "historical_six_aircraft": HISTORICAL, "physical_stage_c": "not_evaluated",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="archived three-process run directory")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = json.dumps(recompute(args.run), ensure_ascii=False, allow_nan=False, indent=2) + "\n"
    if args.out is None:
        print(result, end="")
    else:
        args.out.write_text(result, encoding="utf-8")


if __name__ == "__main__":
    main()
