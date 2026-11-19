"""Bounded local inference requests for the stage C planning loop."""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path
from queue import Empty, SimpleQueue
import re
from time import monotonic_ns
from uuid import uuid4

import numpy as np

from yopo_swarm_core.infer_ipc import IpcError

from .board_inference_ipc import REQUEST_FIELDS, request_candidates


@dataclass(frozen=True)
class InferenceIdentity:
    backend: str
    model_sha256: str
    engine_sha256: str

    def __post_init__(self) -> None:
        if self.backend not in {"tensorrt-fp32", "replay-fp32"}:
            raise ValueError("expected the TensorRT FP32 engine or an explicit replay backend")
        if any(not re.fullmatch(r"[0-9a-f]{64}", value)
               for value in (self.model_sha256, self.engine_sha256)):
            raise ValueError("model and engine must have SHA-256 identities")

    @property
    def provenance(self) -> dict[str, str]:
        return {"model_sha256": self.model_sha256, "engine_sha256": self.engine_sha256}


@dataclass(frozen=True)
class InferenceResult:
    frame_id: int | str
    request_id: str
    deadline_ns: int
    traj_params: np.ndarray
    score: np.ndarray
    radius: np.ndarray


_CANDIDATE_SHAPES = ((1, 15, 14), (1, 15), (1, 15, 20))


class BoardInference:
    """One active request, owned by the planner. Transport runs in an IO worker."""

    def __init__(
        self,
        socket_path: str | Path,
        *,
        session_id: str,
        identity: InferenceIdentity,
        log: Callable,
    ) -> None:
        if not isinstance(session_id, str) or not session_id:
            raise ValueError("the inference source needs the board session ID")
        self._path = socket_path
        self._session_id = session_id
        self._identity = asdict(identity)
        self._log = log
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="board-inference")
        self._events: SimpleQueue = SimpleQueue()
        self._active: Future | None = None
        self._request: dict | None = None
        self._rejected = False

    def __enter__(self) -> BoardInference:
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def close(self) -> None:
        self.invalidate("inference_closed")
        self._worker.shutdown(wait=True)
        self._flush()

    def submit(self, frame_id: int | str, depth, obs, *, deadline_ns: int) -> bool:
        """Copy this exposure and start a request, or refuse if transport is busy."""
        if type(frame_id) not in (int, str) or frame_id == "":
            raise ValueError("inference requires a frame ID")
        if type(deadline_ns) is not int:
            raise ValueError("inference deadline must be a monotonic nanosecond timestamp")
        request = dict(zip(REQUEST_FIELDS, (self._session_id, uuid4().hex, frame_id, deadline_ns)))
        if self._active is not None:
            self._log("inference_rejected", **request, **self._identity, reason="inference_busy")
            return False
        if monotonic_ns() >= deadline_ns:
            self._log("inference_rejected", **request, **self._identity, reason="deadline_exceeded")
            return False
        depth = np.array(depth, dtype=np.float32, order="C", copy=True)
        obs = np.array(obs, dtype=np.float32, order="C", copy=True)
        if depth.shape != (1, 1, 96, 160) or obs.shape != (1, 9):
            raise ValueError("inference depth and observation do not match the FP32 input layout")
        if not (np.isfinite(depth).all() and np.isfinite(obs).all()):
            raise ValueError("inference inputs must be finite")
        self._request = request
        self._rejected = False
        self._active = self._worker.submit(self._forward, request, depth, obs)
        return True

    def _forward(self, request: dict, depth, obs) -> tuple[tuple | None, str | None]:
        deadline = request["deadline_ns"]
        try:
            header, data, sent_ns, received_ns = request_candidates(
                self._path, request, depth, obs,
                lambda stamp: self._events.put(("inference_sent", {"sent_ns": stamp})),
            )
            arrays = tuple(data[key] for key in ("traj_params", "score", "radius"))
        except TimeoutError:
            return None, "deadline_exceeded"
        except (OSError, IpcError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return None, "transport_error"

        # Record arrival even when the reply is unsuitable for use.
        completed_ns = header.get("completed_ns")
        expected = {**request, **self._identity}
        observed = {key: header[key] for key in expected if type(header.get(key)) in (str, int)}
        self._events.put(("inference_received", {
            "received_ns": received_ns,
            "completed_ns": completed_ns if type(completed_ns) is int else None,
            "reply_identity": observed,
        }))
        if any(type(header.get(k)) is not type(v) or header[k] != v for k, v in expected.items()):
            return None, "identity_mismatch"
        if type(completed_ns) is not int or not sent_ns <= completed_ns <= received_ns:
            return None, "invalid_completion_time"
        if any(a.shape != shape or a.dtype != np.float32 or not np.isfinite(a).all()
               for a, shape in zip(arrays, _CANDIDATE_SHAPES)):
            return None, "invalid_candidates"
        if received_ns >= deadline:
            return None, "deadline_exceeded"
        return arrays, None

    def _flush(self) -> None:
        while True:
            try:
                event, fields = self._events.get_nowait()
            except Empty:
                break
            self._log(event, **self._request, **self._identity, **fields)

    def invalidate(self, reason: str) -> None:
        if self._request is not None and not self._rejected:
            self._log("inference_rejected", **self._request, **self._identity, reason=reason)
            self._rejected = True

    def poll(self, frame_id: int | str) -> InferenceResult | None:
        """Adopt a fresh reply for the current exposure, or return immediately."""
        self._flush()
        if self._active is None:
            return None
        req = self._request
        if monotonic_ns() >= req["deadline_ns"]:
            self.invalidate("deadline_exceeded")
        elif type(frame_id) is not type(req["frame_id"]) or frame_id != req["frame_id"]:
            self.invalidate("stale_frame")
        if not self._active.done():
            return None

        arrays, error = self._active.result()
        self._flush()
        self._active = None
        if error is not None:
            self.invalidate(error)
        adopted_ns = monotonic_ns()
        if adopted_ns >= req["deadline_ns"]:
            self.invalidate("deadline_exceeded")
        self._request = None
        if self._rejected:
            return None

        result = InferenceResult(req["frame_id"], req["request_id"], req["deadline_ns"], *arrays)
        self._log("inference_adopted", **req, **self._identity, adopted_ns=adopted_ns,
                  traj_params=result.traj_params.tolist(), score=result.score.tolist(),
                  radius=result.radius.tolist())
        return result
