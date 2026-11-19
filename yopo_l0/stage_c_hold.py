"""Run three wall-clock board instances against isolated simulated receivers."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import selectors
import signal
import socket
import subprocess
import sys
import time

from .board_receiver import SimFlightController
from .board_runtime import BoardConfig, JsonlLog


def _loopback_socket(stack: ExitStack) -> socket.socket:
    link = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
    link.bind(("127.0.0.1", 0))
    return link


class HoldBench:
    def __init__(self, out: Path, stack: ExitStack):
        self.out = out
        self.stack = stack
        self.selector = stack.enter_context(selectors.DefaultSelector())
        self.processes: dict[str, subprocess.Popen] = {}
        self.received: dict[str, list[dict]] = {}
        self.boards = ("uav1", "uav2", "uav3")
        self.configs: dict[str, BoardConfig] = {}
        self.flight_controllers: dict[str, SimFlightController] = {}
        with ExitStack() as reservations:
            binds = {name: _loopback_socket(reservations).getsockname() for name in self.boards}
            for i, name in enumerate(self.boards):
                self._configure_receiver(name, i, binds)

    def _configure_receiver(self, name, lane, binds) -> None:
        directory = self.out / name
        directory.mkdir()
        receiver = _loopback_socket(self.stack)
        config_file = directory / "config.json"
        config_file.write_text(json.dumps({
            "board_id": name,
            "input_id": f"sim-odometry/{name}",
            "flight_controller_id": f"sim-fc/{name}",
            "bind": binds[name],
            "flight_controller_endpoint": receiver.getsockname(),
            "peers": {board: addr for board, addr in binds.items() if board != name},
            "hold_position_enu_m": (0.0, lane * 10.0, 0.0),
            "control_hz": 50.0,
            "command_ttl_s": 0.10,
        }, indent=2) + "\n", encoding="utf-8")
        config = BoardConfig.load(config_file)
        self.configs[name] = config
        self.received[name] = []
        accepted_log = self.stack.enter_context(
            (directory / "flight_controller.jsonl").open("x", encoding="utf-8"),
        )
        rejected_log = self.stack.enter_context(
            (directory / "flight_controller_rejected.jsonl").open("x", encoding="utf-8"),
        )
        authority_log = self.stack.enter_context(
            (directory / "flight_controller_authority.jsonl").open("x", encoding="utf-8"),
        )
        flight_controller = SimFlightController(
            config, receiver, JsonlLog(accepted_log), JsonlLog(rejected_log), JsonlLog(authority_log),
        )
        self.flight_controllers[name] = flight_controller
        self.selector.register(receiver, selectors.EVENT_READ, flight_controller)

    def start(self, board: str, *, extra_args: tuple[str, ...] = ()) -> None:
        directory = self.out / board
        with (directory / "runtime_stderr.log").open("ab") as stderr:
            self.processes[board] = subprocess.Popen(
                [sys.executable, "-m", "yopo_l0.board_runtime",
                 "--config", str(directory / "config.json"),
                 "--log", str(directory / "board.jsonl"), *extra_args],
                stdout=subprocess.DEVNULL, stderr=stderr,
            )

    def stop(self, board: str) -> None:
        process = self.processes.pop(board)
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def close(self) -> None:
        for board in tuple(self.processes):
            self.stop(board)

    def collect(self, timeout_s: float) -> None:
        for key, _ in self.selector.select(timeout_s):
            receiver = key.data
            record = receiver.receive()
            if record is not None:
                self.received[receiver.config.board_id].append(record)
        for receiver in self.flight_controllers.values():
            receiver.expire()

    def until(self, predicate, *, deadline_s=5.0) -> None:
        deadline = time.monotonic() + deadline_s
        while not predicate():
            for board, process in self.processes.items():
                if process.poll() is not None:
                    raise RuntimeError(f"{board} exited with {process.returncode}; see runtime_stderr.log")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("simulated receiver evidence did not arrive")
            self.collect(min(0.05, remaining))


def _pause_and_restart(bench: HoldBench, log: JsonlLog) -> None:
    board = "uav2"
    process = bench.processes[board]
    old_session = bench.received[board][-1]["session_id"]
    process.send_signal(signal.SIGSTOP)
    status = os.waitid(os.P_PID, process.pid, os.WSTOPPED | os.WEXITED | os.WNOWAIT)
    if status.si_code != os.CLD_STOPPED:
        raise RuntimeError(f"{board} exited before pausing")
    paused_ns = time.monotonic_ns()
    log("board_paused", board_id=board, session_id=old_session, monotonic_ns=paused_ns)
    peers = [peer for peer in bench.boards if peer != board]

    def peers_advanced() -> bool:
        return time.monotonic_ns() - paused_ns >= 300_000_000 and all(
            sum(record["sent_ns"] >= paused_ns for record in bench.received[peer]) >= 5
            for peer in peers
        )

    bench.until(peers_advanced)
    if any(record["sent_ns"] >= paused_ns for record in bench.received[board]):
        raise RuntimeError(f"{board} generated a command after it paused")

    process.kill()
    process.wait()
    log("board_restarting", board_id=board, old_session_id=old_session)
    bench.start(board)
    bench.until(lambda: sum(
        row["session_id"] != old_session for row in bench.received[board]
    ) >= 5)


def run_hold(out: Path, *, pause_restart: bool = True) -> None:
    out.mkdir(parents=True)
    with ExitStack() as stack:
        bench = HoldBench(out, stack)
        stack.callback(bench.close)
        for board in bench.boards:
            bench.start(board)
        bench.until(lambda: all(len(records) >= 5 for records in bench.received.values()))
        with (out / "faults.jsonl").open("x", encoding="utf-8") as stream:
            if pause_restart:
                _pause_and_restart(bench, JsonlLog(stream))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="new run output directory")
    parser.add_argument("--scenario", choices=("hold", "pause-restart"), default="pause-restart")
    args = parser.parse_args()
    if args.scenario == "pause-restart" and not hasattr(signal, "SIGSTOP"):
        parser.error("the pause-restart scenario requires POSIX process pause")
    run_hold(args.out, pause_restart=args.scenario == "pause-restart")


if __name__ == "__main__":
    main()
