"""Stage C inference evidence through the board API and a real local socket."""

from contextlib import contextmanager
from threading import Event, Thread
import io
import json
import socket
import time

import numpy as np

from yopo_swarm_core import infer_ipc
from yopo_swarm_core.baseline import GOLDEN_ATOL, GOLDEN_PATH, GOLDEN_RTOL, fixed_inputs
from yopo_l0.board_inference import BoardInference, InferenceIdentity
from yopo_l0 import board_inference_ipc
from yopo_l0.board_runtime import BoardConfig, JsonlLog, run_board


IDENTITY = InferenceIdentity("replay-fp32", "a" * 64, "b" * 64)


def _record(rows):
    def log(event, **fields):
        rows.append({"event": event, **fields})
    return log


@contextmanager
def _server(path, forward, identity=IDENTITY, requests=1, log=None):
    thread = Thread(target=board_inference_ipc.serve, args=(path, forward, identity.backend, requests),
                    kwargs={"identity": identity.provenance, "log": log}, daemon=True)
    thread.start()
    try:
        _wait(path.is_socket)
        yield path
    finally:
        thread.join(2)
    assert not thread.is_alive()


def _wait(callback, timeout=2):
    limit = time.monotonic() + timeout
    while time.monotonic() < limit:
        value = callback()
        if value is not None and value is not False:
            return value
        time.sleep(0.002)
    raise AssertionError("timed out waiting for IPC evidence")


def _inputs():
    return fixed_inputs({"vertical_num": 3, "horizon_num": 5, "downsample_factor": 32})


def test_fixed_replay_records_exact_request_and_adoption(tmp_path):
    rows = []
    depth, obs = _inputs()
    with np.load(GOLDEN_PATH) as data:
        golden = tuple(data[key] for key in ("traj_params", "score", "radius"))

    def fixed_forward(sent_depth, sent_obs):
        np.testing.assert_array_equal(sent_depth, depth)
        np.testing.assert_array_equal(sent_obs, obs)
        return golden

    with _server(tmp_path / "infer.sock", fixed_forward) as path:
        with BoardInference(path, session_id="bench-session", identity=IDENTITY, log=_record(rows)) as client:
            deadline = time.monotonic_ns() + 1_000_000_000
            assert client.submit(17, depth, obs, deadline_ns=deadline)
            result = _wait(lambda: client.poll(17))

    assert result.frame_id == 17 and result.deadline_ns == deadline
    for actual, expected in zip((result.traj_params, result.score, result.radius), golden):
        np.testing.assert_allclose(actual, expected, rtol=GOLDEN_RTOL, atol=GOLDEN_ATOL)
    assert result.score.argmax() == golden[1].argmax()
    by_event = {row["event"]: row for row in rows}
    sent = by_event["inference_sent"]
    received = by_event["inference_received"]
    adopted = by_event["inference_adopted"]
    assert sent["sent_ns"] <= received["completed_ns"] <= received["received_ns"] <= adopted["adopted_ns"] < deadline
    assert all(row["request_id"] == result.request_id for row in (sent, received, adopted))
    assert adopted["model_sha256"] == IDENTITY.model_sha256
    assert adopted["engine_sha256"] == IDENTITY.engine_sha256


def test_receiver_keeps_ticking_while_late_reply_is_discarded(tmp_path):
    rows, service_rows = [], []
    started = Event()
    release = Event()
    depth, obs = _inputs()
    with np.load(GOLDEN_PATH) as data:
        golden = tuple(data[key] for key in ("traj_params", "score", "radius"))

    def paused_forward(_depth, _obs):
        if not started.is_set():
            started.set()
            assert release.wait(2)
        return golden

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        config = BoardConfig("u1", "state-u1", "fc-u1", ("127.0.0.1", 0),
                             receiver.getsockname(), {}, (0.0, 0.0, 1.0))
        stopping = Event()
        loop = Thread(target=run_board, args=(config, JsonlLog(io.StringIO()), stopping),
                      daemon=True)
        loop.start()
        try:
            with _server(tmp_path / "infer.sock", paused_forward, requests=2,
                         log=_record(service_rows)) as path:
                with BoardInference(path, session_id="bench-session", identity=IDENTITY,
                                    log=_record(rows)) as client:
                    deadline = time.monotonic_ns() + 200_000_000
                    assert client.submit(21, depth, obs, deadline_ns=deadline)
                    assert started.wait(1)
                    during_block = []
                    while time.monotonic_ns() < deadline + 35_000_000:
                        packet = json.loads(receiver.recv(8192))
                        assert packet["authority"] == "PVA"
                        assert packet["command"]["kind"] == "hold"
                        during_block.append(packet)
                        assert client.poll(21) is None
                    assert len(during_block) >= 3
                    assert during_block[-1]["sent_ns"] > deadline
                    assert any(row.get("reason") == "deadline_exceeded" for row in rows)
                    release.set()

                    late = _wait(lambda: next(
                        (r for r in service_rows if r["event"] == "inference_reply_disconnected"), None
                    ))
                    assert late["frame_id"] == 21 and late["completed_ns"] > deadline
                    assert client.poll(22) is None
                    assert client.submit(22, depth, obs,
                                         deadline_ns=time.monotonic_ns() + 1_000_000_000)
                    assert _wait(lambda: client.poll(22)).frame_id == 22
        finally:
            release.set()
            stopping.set()
            loop.join(1)
    assert all(row["frame_id"] == 22 for row in rows if row["event"] == "inference_adopted")


def test_golden_command_checks_fp32_tolerance_and_rank(tmp_path):
    from yopo_l0 import board_inference_service

    with np.load(GOLDEN_PATH) as data:
        params, scores, radii = (data[key] for key in ("traj_params", "score", "radius"))
    for changed, expected_code in ((scores, 0), (scores + np.float32(0.1), 1)):
        socket_path = tmp_path / "inference.sock"
        report_path = tmp_path / "report.json"
        with _server(socket_path, lambda *_: (params, changed, radii)):
            code = board_inference_service.main([
                "golden", "--socket", str(socket_path), "--out", str(report_path),
                "--backend", IDENTITY.backend, "--model-sha256", IDENTITY.model_sha256,
                "--engine-sha256", IDENTITY.engine_sha256,
            ])
        assert code == expected_code
        report = json.loads(report_path.read_text())
        assert report["model_sha256"] == IDENTITY.model_sha256
        assert report["engine_sha256"] == IDENTITY.engine_sha256
        assert report["argmax_equal"] is True
        assert report["pass_tol"] is (expected_code == 0)


def test_previous_session_reply_is_rejected_with_wire_identity(tmp_path):
    path = tmp_path / "identity.sock"
    depth, obs = _inputs()
    rows = []
    with np.load(GOLDEN_PATH) as data:
        arrays = {name: data[name] for name in ("traj_params", "score", "radius")}

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        listener.listen(1)

        def reply_with_old_session():
            client, _ = listener.accept()
            with client:
                request, _ = infer_ipc.recv_msg(client)
                echo = {key: request[key] for key in board_inference_ipc.REQUEST_FIELDS}
                echo["session_id"] = "old-session"
                infer_ipc.send_msg(client, {**echo, **IDENTITY.provenance, "op": "result",
                                            "seq": request["seq"], "backend": IDENTITY.backend,
                                            "completed_ns": time.monotonic_ns()}, arrays)

        server = Thread(target=reply_with_old_session, daemon=True)
        server.start()
        with BoardInference(path, session_id="current-session", identity=IDENTITY,
                            log=_record(rows)) as client:
            assert client.submit(17, depth, obs, deadline_ns=time.monotonic_ns() + 1_000_000_000)

            def poll_until_rejected():
                assert client.poll(17) is None
                return next((r for r in rows if r["event"] == "inference_rejected"), None)

            reason = _wait(poll_until_rejected)
            assert reason["reason"] == "identity_mismatch"
        server.join(1)

    reply = next(r for r in rows if r["event"] == "inference_received")
    assert reply["reply_identity"]["session_id"] == "old-session"
    assert reply["session_id"] == "current-session"
