"""Process-local wall-clock control entry for the stage C bench."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import secrets
import signal
import socket
from threading import Event
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

    def __call__(self, event: str, **fields) -> None:
        row = {"event": event, "monotonic_ns": time.monotonic_ns(), **self.identity, **fields}
        self.stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
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


def _hold(tick: ControlTick) -> ControlOutput:
    return ControlOutput("PVA", {"kind": "hold", "position_enu_m": tick.config.hold_position_enu_m})


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
) -> None:
    session_id = secrets.token_hex(8)
    period_ns = round(NS_PER_S / config.control_hz)
    ttl_ns = round(config.command_ttl_s * NS_PER_S)
    identity = {
        "board_id": config.board_id,
        "input_id": config.input_id,
        "flight_controller_id": config.flight_controller_id,
        "session_id": session_id,
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
        log("sim_input_bound", source_kind="static_position_fixture",
            position_enu_m=config.hold_position_enu_m)
        scheduled_ns = time.monotonic_ns()
        sequence = 0
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
            output = control_step(tick)
            completed_ns = time.monotonic_ns()
            log("control_tick", sequence=sequence, scheduled_ns=scheduled_ns,
                started_ns=now_ns, completed_ns=completed_ns, missed_periods=missed,
                peer_ages_ns=peer_ages)
            if output is not None:
                sent_ns = time.monotonic_ns()
                valid_until_ns = tick.scheduled_ns + ttl_ns
                if output.valid_until_ns is not None:
                    valid_until_ns = min(output.valid_until_ns, valid_until_ns)
                if sent_ns < valid_until_ns:
                    envelope = {
                        **identity, "type": "command", "sequence": sequence,
                        "sent_ns": sent_ns, "valid_until_ns": valid_until_ns,
                        "authority": output.authority, "command": output.command,
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
        log("session_stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    config = BoardConfig.load(args.config)
    if any(addr[0] != "127.0.0.1" for addr in (
        config.bind, config.flight_controller_endpoint, *config.peers.values(),
    )):
        parser.error("the simulated hold backend requires loopback endpoints")
    stopping = Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: stopping.set())
    with args.log.open("a", encoding="utf-8") as stream:
        run_board(config, JsonlLog(stream), stopping)


if __name__ == "__main__":
    main()
