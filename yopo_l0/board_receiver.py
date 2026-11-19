"""The isolated, wall-clock flight-controller receiver used by the software bench."""

from __future__ import annotations

import json
import math
import socket
import time

from .board_runtime import BoardConfig, JsonlLog


def _reject_json_constant(token: str):
    raise ValueError(f"nonfinite JSON value: {token}")


def _finite(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _vector(value) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(map(_finite, value))


def _valid_command(authority: str, command: dict) -> bool:
    kind = command.get("kind")
    if authority in ("NONE", "MANUAL"):
        return (kind == "authority_only" and set(command) == {"kind", "reason"}
                and isinstance(command["reason"], str))
    if authority == "PVA":
        if kind not in ("hold", "pva") or not _vector(command.get("position_enu_m")):
            return False
        required = {"velocity_enu_mps", "acceleration_enu_mps2", "yaw_rad"}
        keys = set(command) - {"kind", "position_enu_m"}
        if kind == "hold" and not keys:
            return True
        return (keys == required and _vector(command["velocity_enu_mps"])
                and _vector(command["acceleration_enu_mps2"]) and _finite(command["yaw_rad"])
                and (kind == "pva" or (not any(command["velocity_enu_mps"])
                                      and not any(command["acceleration_enu_mps2"]))))
    if authority == "CTBR":
        required = {"kind", "timestamp", "roll", "pitch", "yaw", "thrust_body", "reset_integral"}
        return (kind == "ctbr" and set(command) == required and type(command["timestamp"]) is int
                and all(_finite(command[key]) for key in ("roll", "pitch", "yaw"))
                and _vector(command["thrust_body"]) and type(command["reset_integral"]) is bool)
    return False


class SimFlightController:
    """Enforce expiry, session fencing and a single active command at the receiver."""

    def __init__(self, config: BoardConfig, receiver: socket.socket, log: JsonlLog,
                 reject_log: JsonlLog, authority_log: JsonlLog | None = None) -> None:
        self.config = config
        self.receiver = receiver
        self.log = log
        self.reject_log = reject_log
        self.authority_log = authority_log
        self.session_id: str | None = None
        self._session_started_ns = -1
        self._sequence = -1
        self._epoch = 0
        self._authority = "NONE"
        self._disposition = "relinquished"
        self._active: dict | None = None
        self._last_invalidation_ns: int | None = None

    @property
    def current_command(self) -> dict | None:
        self.expire()
        return self._active

    @property
    def effective_authority(self) -> str:
        self.expire()
        if self._active is not None or self._authority == "MANUAL":
            return self._authority
        return "NONE"

    def _end_command(self, stamp_ns: int, reason: str) -> None:
        if self._active is None:
            return
        old = self._active
        self._active = None
        self._last_invalidation_ns = min(stamp_ns, old["valid_until_ns"])
        if self.authority_log is not None:
            self.authority_log(
                "command_invalidated", board_id=self.config.board_id, reason=reason,
                session_id=old["session_id"], sequence=old["sequence"], authority=old["authority"],
                authority_epoch=old["authority_epoch"], received_ns=stamp_ns,
                invalidated_ns=self._last_invalidation_ns, prior_valid_until_ns=old["valid_until_ns"],
            )

    def expire(self, now_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        if self._active is not None and now_ns >= self._active["valid_until_ns"]:
            self._end_command(now_ns, "COMMAND_DEADLINE")

    def _validate(self, message: dict, sender, received_ns: int) -> None:
        config = self.config
        if (not isinstance(message, dict) or sender != config.bind
                or message.get("board_id") != config.board_id
                or message.get("input_id") != config.input_id
                or message.get("flight_controller_id") != config.flight_controller_id
                or message.get("type") != "command"):
            raise ValueError("endpoint or identity mismatch")
        sent, deadline, start = (message[key] for key in ("sent_ns", "valid_until_ns", "session_started_ns"))
        if (any(type(value) is not int for value in (sent, deadline, start))
                or not 0 <= start <= sent <= received_ns < deadline):
            raise ValueError("invalid or expired command")
        session, seq, epoch = (message[key] for key in ("session_id", "sequence", "authority_epoch"))
        authority, disposition, command = (message[key] for key in ("authority", "disposition", "command"))
        if (not isinstance(session, str) or not session
                or type(seq) is not int or type(epoch) is not int or seq < 0 or epoch <= 0
                or not isinstance(disposition, str) or not disposition):
            raise ValueError("invalid session, sequence or authority epoch")
        if not isinstance(command, dict) or not _valid_command(authority, command):
            raise ValueError("authority and command payload disagree")
        position = message.get("position_enu_m")
        stamp = message.get("position_stamp_ns")
        speed = message.get("reference_speed_mps")
        if ((position is not None or stamp is not None)
                and (not _vector(position) or type(stamp) is not int or not 0 <= stamp <= sent)):
            raise ValueError("invalid command odometry evidence")
        if speed is not None and (not _finite(speed) or speed < 0.0):
            raise ValueError("invalid reference speed evidence")
        if session == self.session_id:
            if start != self._session_started_ns or seq <= self._sequence or epoch < self._epoch:
                raise ValueError("stale sequence, session or authority epoch")
            if epoch == self._epoch and (authority, disposition) != (self._authority, self._disposition):
                raise ValueError("permission changed without a new authority epoch")
        elif start <= self._session_started_ns:
            raise ValueError("stale session")

    def receive(self) -> dict | None:
        raw, sender = self.receiver.recvfrom(4096)
        received_ns = time.monotonic_ns()
        self.expire(received_ns)
        try:
            message = json.loads(raw, parse_constant=_reject_json_constant)
            self._validate(message, sender, received_ns)
        except (KeyError, ValueError, OverflowError) as error:
            self.reject_log(
                "command_rejected", received_ns=received_ns, sender=sender,
                reason=str(error), wire_hex=raw.hex(),
            )
            return None
        record = {**message, "received_ns": received_ns, "sender": sender,
                  "receiver": self.config.flight_controller_endpoint}
        session, epoch, authority = (message[key] for key in ("session_id", "authority_epoch", "authority"))
        new_session = session != self.session_id
        if new_session or epoch != self._epoch:
            previous = self._authority
            self._end_command(received_ns, "SESSION_CHANGED" if new_session else "PERMISSION_CHANGED")
            if self.authority_log is not None:
                self.authority_log(
                    "authority_changed" if previous != authority or new_session else "permission_replaced",
                    session_id=session, board_id=self.config.board_id, sequence=message["sequence"],
                    previous_authority=previous, authority=authority, authority_epoch=epoch,
                    disposition=message["disposition"], received_ns=received_ns,
                    old_command_invalidated_ns=self._last_invalidation_ns,
                    reason=message.get("reason") or message["command"].get("reason"),
                )
            self._last_invalidation_ns = None
        self._authority = authority
        self._disposition = message["disposition"]
        self.session_id, self._epoch = session, epoch
        self._session_started_ns = message["session_started_ns"]
        self._sequence = message["sequence"]
        if authority in ("NONE", "MANUAL"):
            return None
        self._active = record
        self.log("command_received", **record)
        return record
