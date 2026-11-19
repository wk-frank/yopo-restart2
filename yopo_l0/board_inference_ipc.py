"""Stage C request metadata over the shared YSI1 Unix socket framing."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import socket
from time import monotonic_ns

from yopo_swarm_core.infer_ipc import ForwardFn, IpcError, recv_msg, send_msg


REQUEST_FIELDS = ("session_id", "request_id", "frame_id", "deadline_ns")
_WIRE_ERRORS = (IpcError, OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError)


class _DeadlineSocket:
    """Re-arm the same monotonic deadline before every blocking transfer."""

    def __init__(self, sock: socket.socket, deadline_ns: int) -> None:
        self._sock = sock
        self._deadline_ns = deadline_ns

    def _arm(self) -> None:
        remaining_ns = self._deadline_ns - monotonic_ns()
        if remaining_ns <= 0:
            raise TimeoutError("inference request deadline exceeded")
        self._sock.settimeout(remaining_ns / 1e9)

    def connect(self, path: Path | str) -> None:
        self._arm()
        self._sock.connect(str(path))

    def sendall(self, data: bytes) -> None:
        self._arm()
        self._sock.sendall(data)

    def recv(self, size: int) -> bytes:
        self._arm()
        return self._sock.recv(size)


def request_candidates(path, request: dict, depth, obs, on_sent: Callable[[int], None]):
    """Exchange one request over this board's socket before its absolute deadline."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        link = _DeadlineSocket(sock, request["deadline_ns"])
        link.connect(path)
        sent_ns = monotonic_ns()
        send_msg(link, {**request, "op": "forward", "seq": 1}, {"depth": depth, "obs": obs})
        on_sent(sent_ns)
        header, arrays = recv_msg(link)
        received_ns = monotonic_ns()
    if type(header.get("seq")) is not int or header["seq"] != 1:
        raise IpcError("invalid inference IPC sequence")
    if header.get("op") != "result":
        raise IpcError(header.get("reason", "invalid inference response"))
    return header, arrays, sent_ns, received_ns


def serve(
    path: Path | str,
    forward: ForwardFn,
    backend: str,
    max_requests: int | None = None,
    *,
    identity: dict[str, str],
    log: Callable | None = None,
) -> None:
    """Run a dedicated engine on a fresh per-board socket with correlated records."""
    path = Path(path)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        try:
            path.chmod(0o660)
            listener.listen(1)
            served = 0
            while max_requests is None or served < max_requests:
                conn, _ = listener.accept()
                with conn:
                    request_link = _DeadlineSocket(conn, monotonic_ns() + 1_000_000_000)
                    try:
                        header, arrays = recv_msg(request_link)
                        receipt = {key: header[key] for key in REQUEST_FIELDS}
                        seq = header["seq"]
                    except _WIRE_ERRORS:
                        continue
                    receipt.update(**identity, backend=backend, seq=seq)
                    started_ns = monotonic_ns()
                    deadline_ns = receipt["deadline_ns"]
                    output = None
                    try:
                        if (header.get("op") != "forward" or type(seq) is not int
                                or not all(isinstance(receipt[k], str) and receipt[k]
                                           for k in ("session_id", "request_id"))
                                or type(receipt["frame_id"]) not in (int, str)
                                or type(deadline_ns) is not int):
                            raise ValueError("invalid stage C request identity")
                        if deadline_ns <= started_ns:
                            raise TimeoutError("inference request deadline exceeded")
                        params, score, radius = forward(arrays["depth"], arrays["obs"])
                        completed_ns = monotonic_ns()
                        response = {**receipt, "op": "result", "completed_ns": completed_ns,
                                    "infer_ms": (completed_ns - started_ns) / 1e6}
                        output = {"traj_params": params, "score": score, "radius": radius}
                        if log is not None:
                            log("inference_completed", **receipt, started_ns=started_ns,
                                completed_ns=completed_ns)
                    except Exception as error:
                        response = {**receipt, "op": "error",
                                    "reason": f"{type(error).__name__}: {error}"}
                    served += 1
                    try:
                        conn.settimeout(1.0)
                        send_msg(conn, response, output)
                    except OSError:
                        if log is not None:
                            log("inference_reply_disconnected", **receipt,
                                completed_ns=response.get("completed_ns"))
        finally:
            path.unlink(missing_ok=True)
