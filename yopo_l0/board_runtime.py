"""Process-local wall-clock control entry for the stage C bench."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from importlib import import_module
import json
import math
import os
from pathlib import Path
import secrets
import signal
import socket
from threading import Event, Lock
import time
from typing import TextIO


Address = tuple[str, int]
NS_PER_S = 1_000_000_000


def _address(value: list) -> Address:
    host, port = value
    socket.inet_pton(socket.AF_INET, host)
    if type(port) is not int or not 0 < port <= 65535:
        raise ValueError(f"invalid port: {port!r}")
    return host, port


@dataclass(frozen=True)
class BoardConfig:
    board_id: str
    input_id: str
    flight_controller_id: str
    bind: Address
    flight_controller_endpoint: Address
    peers: dict[str, Address]
    hold_position_enu_m: tuple[float, float, float]
    control_hz: float = 50.0
    command_ttl_s: float = 0.10

    @classmethod
    def load(cls, path: Path) -> BoardConfig:
        data = json.loads(path.read_text(encoding="utf-8"))
        data["bind"] = _address(data["bind"])
        data["flight_controller_endpoint"] = _address(data["flight_controller_endpoint"])
        data["peers"] = {peer: _address(addr) for peer, addr in data["peers"].items()}
        data["hold_position_enu_m"] = tuple(data["hold_position_enu_m"])
        config = cls(**data)
        endpoints = (config.bind, config.flight_controller_endpoint, *config.peers.values())
        if len(set(endpoints)) != len(endpoints) or config.board_id in config.peers:
            raise ValueError("board, peers and flight-controller endpoints must be distinct")
        if not all(isinstance(value, str) and value for value in (
            config.board_id, config.input_id, config.flight_controller_id, *config.peers,
        )):
            raise ValueError("identities must be nonempty strings")
        if len(config.hold_position_enu_m) != 3 or not all(map(math.isfinite, config.hold_position_enu_m)):
            raise ValueError("hold position must have three finite ENU coordinates")
        if not 50 <= config.control_hz <= 100 or not 0 < config.command_ttl_s <= 1:
            raise ValueError("invalid control frequency or command validity period")
        return config


class JsonlLog:
    """Append one event per line to a process-owned stream."""

    def __init__(self, stream: TextIO, **identity):
        self.stream = stream
        self.identity = identity
        self._lock = Lock()

    def __call__(self, event: str, **fields) -> None:
        row = {"event": event, "monotonic_ns": time.monotonic_ns(), **self.identity, **fields}
        line = json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n"
        with self._lock:
            self.stream.write(line)
            self.stream.flush()


@dataclass(frozen=True)
class ControlTick:
    config: BoardConfig
    session_id: str
    sequence: int
    scheduled_ns: int
    now_ns: int
    peer_ages_ns: dict[str, int | None]


@dataclass(frozen=True)
class ControlOutput:
    authority: str
    command: dict
    valid_until_ns: int | None = None
    disposition: str = "tracking"
    position_enu_m: tuple[float, float, float] | None = None
    position_stamp_ns: int | None = None
    reference_speed_mps: float | None = None
    reason: str | None = None
    frame_id: int | None = None
    exposure_ns: int | None = None
    brake_start_ns: int | None = None
    peer_hold_ready: bool | None = None


@dataclass(frozen=True)
class BoardBackend:
    """Process-local hooks returned by a backend module's build_backend factory."""

    control_step: Callable[[ControlTick], ControlOutput | None]
    invalidate: Callable[[str], None] | None = None
    close: Callable[[], None] | None = None


BackendFactory = Callable[[BoardConfig, JsonlLog, Event, Path | None], BoardBackend]


def _hold(tick: ControlTick) -> ControlOutput:
    position = tick.config.hold_position_enu_m
    return ControlOutput("PVA", {"kind": "hold", "position_enu_m": position}, disposition="hold",
                         position_enu_m=position, position_stamp_ns=tick.now_ns, reference_speed_mps=0.0)


def _send(link: socket.socket, addr: Address, payload: dict, log: JsonlLog) -> bool:
    try:
        link.sendto(json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8"), addr)
    except OSError as error:
        log("send_error", endpoint=addr, reason=str(error), sent_type=payload["type"])
        return False
    return True


def _receive_heartbeats(
    link: socket.socket,
    peers_by_addr: dict[Address, str],
    seen: dict[str, tuple[int, str, int]],
    log: JsonlLog,
) -> None:
    for _ in range(32):
        try:
            raw, addr = link.recvfrom(4096)
        except BlockingIOError:
            return
        arrived_ns = time.monotonic_ns()
        try:
            message = json.loads(raw)
            if not isinstance(message, dict):
                continue
            peer_id = peers_by_addr.get(addr)
            if peer_id is None or message.get("type") != "heartbeat" or message.get("board_id") != peer_id:
                continue
            sent_ns = message["sent_ns"]
            sequence = message["sequence"]
            session_id = message["session_id"]
            if (type(sent_ns) is not int or not 0 <= sent_ns <= arrived_ns
                    or type(sequence) is not int or sequence < 0
                    or not isinstance(session_id, str) or not session_id):
                continue
        except (ValueError, KeyError):
            continue
        previous = seen.get(peer_id)
        if previous is not None and sent_ns <= previous[0]:
            continue
        seen[peer_id] = (sent_ns, session_id, sequence)
        log("heartbeat_received", peer_id=peer_id, peer_session_id=session_id,
            sequence=sequence, sent_ns=sent_ns, received_ns=arrived_ns)


def run_board(
    config: BoardConfig,
    log: JsonlLog,
    stopping: Event,
    control_step: Callable[[ControlTick], ControlOutput | None] = _hold,
    *,
    backend_factory: BackendFactory | None = None,
    backend_config: Path | None = None,
) -> None:
    session_id = secrets.token_hex(8)
    session_started_ns = time.monotonic_ns()
    period_ns = round(NS_PER_S / config.control_hz)
    ttl_ns = round(config.command_ttl_s * NS_PER_S)
    identity = {
        "board_id": config.board_id,
        "input_id": config.input_id,
        "flight_controller_id": config.flight_controller_id,
        "session_id": session_id,
        "session_started_ns": session_started_ns,
    }
    log = JsonlLog(log.stream, **{**log.identity, **identity})
    peers_by_addr = {addr: peer for peer, addr in config.peers.items()}
    seen: dict[str, tuple[int, str, int]] = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as link:
        link.bind(config.bind)
        link.setblocking(False)
        log("session_started", board_bind=config.bind, peer_endpoints=config.peers,
            flight_controller_endpoint=config.flight_controller_endpoint,
            clock="host_monotonic_ns", pid=os.getpid())
        if backend_factory is None:
            log("sim_input_bound", source_kind="static_position_fixture",
                position_enu_m=config.hold_position_enu_m)
            backend = BoardBackend(control_step)
        else:
            backend = backend_factory(config, log, stopping, backend_config)
        scheduled_ns = time.monotonic_ns()
        sequence = 0
        authority_epoch = 0
        last_permission = None
        try:
            while not stopping.wait(max(0.0, (scheduled_ns - time.monotonic_ns()) / NS_PER_S)):
                _receive_heartbeats(link, peers_by_addr, seen, log)
                now_ns = time.monotonic_ns()
                missed = max(0, (now_ns - scheduled_ns) // period_ns)
                scheduled_ns += missed * period_ns
                peer_ages = {
                    peer: now_ns - seen[peer][0] if peer in seen else None
                    for peer in config.peers
                }
                tick = ControlTick(config, session_id, sequence, scheduled_ns, now_ns, peer_ages)
                output = backend.control_step(tick)
                completed_ns = time.monotonic_ns()
                reason = None
                if missed or completed_ns >= scheduled_ns + period_ns:
                    reason = "CONTROL_OVERRUN"
                elif (output is not None and output.valid_until_ns is not None
                      and completed_ns >= output.valid_until_ns):
                    reason = "CONTROL_OUTPUT_EXPIRED"
                if reason is not None:
                    if backend.invalidate is not None:
                        backend.invalidate(reason)
                    output = ControlOutput("NONE", {"kind": "authority_only", "reason": reason},
                                           disposition="relinquished", reason=reason)
                    log("control_output_revoked", sequence=sequence, reason=reason)
                log("control_tick", sequence=sequence, scheduled_ns=scheduled_ns,
                    started_ns=now_ns, completed_ns=completed_ns, missed_periods=missed,
                    peer_ages_ns=peer_ages)
                if output is not None:
                    sent_ns = time.monotonic_ns()
                    valid_until_ns = tick.scheduled_ns + ttl_ns
                    if output.valid_until_ns is not None:
                        valid_until_ns = min(output.valid_until_ns, valid_until_ns)
                    if output.authority in ("NONE", "MANUAL"):
                        # A revocation must still reach the receiver when a control tick overruns.
                        valid_until_ns = sent_ns + ttl_ns
                    if sent_ns < valid_until_ns:
                        permission = (output.authority, output.disposition)
                        if permission != last_permission:
                            authority_epoch += 1
                            last_permission = permission
                        envelope = {
                            **identity, "type": "command", "sequence": sequence,
                            "sent_ns": sent_ns, "valid_until_ns": valid_until_ns,
                            "authority": output.authority, "authority_epoch": authority_epoch,
                            "disposition": output.disposition, "command": output.command,
                            "position_enu_m": output.position_enu_m,
                            "position_stamp_ns": output.position_stamp_ns,
                            "reference_speed_mps": output.reference_speed_mps, "reason": output.reason,
                            "frame_id": output.frame_id, "exposure_ns": output.exposure_ns,
                            "brake_start_ns": output.brake_start_ns,
                            "peer_hold_ready": output.peer_hold_ready,
                        }
                        if _send(link, config.flight_controller_endpoint, envelope, log):
                            log("command_sent", **envelope)
                    else:
                        log("command_expired", sequence=sequence, valid_until_ns=valid_until_ns)
                heartbeat = {"type": "heartbeat", "board_id": config.board_id, "session_id": session_id,
                             "sequence": sequence, "sent_ns": time.monotonic_ns()}
                for peer, addr in config.peers.items():
                    if _send(link, addr, heartbeat, log):
                        log("heartbeat_sent", peer_id=peer, sequence=sequence, sent_ns=heartbeat["sent_ns"])
                scheduled_ns += period_ns
                sequence += 1
        finally:
            stopping.set()
            if backend.close is not None:
                backend.close()
        log("session_stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--backend", help="module exposing build_backend for the bench")
    parser.add_argument("--backend-config", type=Path)
    args = parser.parse_args()
    if args.backend_config is not None and args.backend is None:
        parser.error("--backend-config requires --backend")
    factory = import_module(args.backend).build_backend if args.backend else None
    config = BoardConfig.load(args.config)
    if any(addr[0] != "127.0.0.1" for addr in (
        config.bind, config.flight_controller_endpoint, *config.peers.values(),
    )):
        parser.error("the simulated flight-controller entry requires loopback endpoints")
    stopping = Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: stopping.set())
    with args.log.open("a", encoding="utf-8") as stream:
        run_board(config, JsonlLog(stream), stopping, backend_factory=factory,
                  backend_config=args.backend_config)


if __name__ == "__main__":
    main()
