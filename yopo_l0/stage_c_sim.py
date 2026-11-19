"""Run and archive the software-only three-process stage C loop."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import monotonic_ns

import numpy as np

from .board_inference_ipc import serve
from .board_runtime import JsonlLog
from .ctbr import GRAVITY
from .so3 import _exp_so3
from .stage_c_evidence import recompute
from .stage_c_hold import HoldBench, _loopback_socket


NS = 1_000_000_000
POSE_PERIOD_NS = 20_000_000
DEPTH_PERIOD_NS = NS // 15
BOARD_POSITIONS = ((0., 0., 2.), (0., 3.5, 2.), (0., -7.0, 2.))
CASES = ("normal", "all-blind", "inference-timeout", "one-way-drop", "restart")


def _json(path, doc):
    path.write_text(json.dumps(doc, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _write_fixture(path):
    params = np.zeros((1, 15, 14), dtype=np.float32)
    params[:, :, 5] = -0.96
    params[:, :, 6] = 0.20 / 6.0
    params[:, 7, 6] = 0.45 / 6.0
    scores = np.zeros((1, 15), dtype=np.float32)
    scores[:, 7] = 1.0
    radii = np.full((1, 15, 20), 0.5, dtype=np.float32)
    np.savez(path, traj_params=params, score=scores, radius=radii)
    return sha256(path.read_bytes()).hexdigest()


def _run_service(data_path: Path, endpoint: Path, log_path: Path):
    digest = sha256(data_path.read_bytes()).hexdigest()
    with np.load(data_path) as bundle:
        outputs = tuple(bundle[name] for name in ("traj_params", "score", "radius"))
    identity = {"model_sha256": digest, "engine_sha256": digest}
    signal.signal(signal.SIGTERM, _interrupt)
    with log_path.open("a", encoding="utf-8") as stream:
        log = JsonlLog(stream)
        log("inference_service_started", backend="replay-fp32", **identity)
        try:
            serve(endpoint, lambda _depth, _obs: outputs, "replay-fp32", identity=identity, log=log)
        except KeyboardInterrupt:
            pass


def _interrupt(_number, _frame):
    raise KeyboardInterrupt


def _stop_service(child):
    if child.poll() is not None:
        return
    child.send_signal(signal.SIGCONT)
    child.terminate()
    try:
        child.wait(2)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


class Plant:
    """Software host plant. CTBR drives attitude and thrust from received rates."""

    def __init__(self, position):
        self.position = np.array(position, dtype=float)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.rotation = np.eye(3)
        self.last_command = None
        self.used_command = None
        self.peak_speed = 0.0

    def advance(self, start_ns, end_ns, receiver):
        latest = receiver.current_command
        previous = self.last_command
        cut = end_ns
        if latest != previous:
            stamp = latest["received_ns"] if latest else receiver.last_motion_invalidated_ns
            cut = min(end_ns, max(start_ns, stamp or end_ns))
        self._interval(start_ns, cut, previous)
        self._interval(cut, end_ns, latest)
        self.last_command = latest
        self.peak_speed = max(self.peak_speed, float(np.linalg.norm(self.velocity)))

    def _interval(self, start, end, record):
        if end <= start:
            return
        active_until = start if record is None else min(end, max(start, record["valid_until_ns"]))
        if start < active_until:
            self._integrate((active_until - start) / NS, record)
        if active_until < end:
            self._integrate((end - active_until) / NS, None)

    def _integrate(self, period, record):
        count = max(1, int(np.ceil(period / 0.01)))
        dt = period / count
        for _ in range(count):
            if record is not None and record["authority"] == "CTBR":
                cmd = record["command"]
                rates = np.array([cmd["roll"], -cmd["pitch"], -cmd["yaw"]])
                self.rotation = self.rotation @ _exp_so3(rates * dt)
                thrust = 2.0 * GRAVITY * -cmd["thrust_body"][2]
                self.acceleration = self.rotation[:, 2] * thrust - (0.0, 0.0, GRAVITY)
            elif record is not None:
                cmd = record["command"]
                p, v, a = [np.asarray(cmd.get(key, (0.0, 0.0, 0.0))) for key in
                           ("position_enu_m", "velocity_enu_mps", "acceleration_enu_mps2")]
                goal_acc = a + 4.0 * (p - self.position) + 3.0 * (v - self.velocity)
                self.acceleration += (1.0 - np.exp(-dt / .10)) * (goal_acc - self.acceleration)
            else:
                # Host-local damped stop when no fresh auto command is active.
                # It is neither CTBR motion nor a board-certified brake.
                self.acceleration = -6.0 * self.velocity
            self.velocity += self.acceleration * dt
            self.position += self.velocity * dt
        self.used_command = record


class Simulation:
    def __init__(self, out, case, stack, socket_root, normal_duration_s):
        self.out, self.case = out, case
        self.normal_duration_ns = round(normal_duration_s * NS)
        self.bench = HoldBench(out, stack, positions=BOARD_POSITIONS)
        self.child_env = {**os.environ, "OPENBLAS_NUM_THREADS": "1"}
        stack.callback(self.bench.close)
        self.board_ids = self.bench.boards
        self.plants = {key: Plant(self.bench.configs[key].hold_position_enu_m) for key in self.board_ids}
        self.judge = JsonlLog(stack.enter_context((out / "judge.jsonl").open("x", encoding="utf-8")))
        self.faults = JsonlLog(stack.enter_context((out / "faults.jsonl").open("x", encoding="utf-8")))
        self.sensor = _loopback_socket(stack)
        with ExitStack() as reserve:
            self.input_addrs = {key: _loopback_socket(reserve).getsockname() for key in self.board_ids}
            peers = {key: _loopback_socket(reserve).getsockname() for key in self.board_ids}
        self.inputs = {}
        self.services = {}
        self.options = {}
        self.infer_endpoints = []
        golden = out / "replay-candidates.npz"
        digest = _write_fixture(golden)
        for board in self.board_ids:
            home = out / board
            self.inputs[board] = JsonlLog(stack.enter_context((home / "input.jsonl").open("x", encoding="utf-8")))
            endpoint = socket_root / (board + ".sock")
            self.infer_endpoints.append(endpoint)
            identity = {"backend": "replay-fp32", "model_sha256": digest, "engine_sha256": digest}
            options = {
                "camera": "sim-D435/" + board, "sensor_bind": self.input_addrs[board],
                "sensor_sender": self.sensor.getsockname(), "infer_socket": str(endpoint),
                "peer_bind": peers[board], "peers": {name: addr for name, addr in peers.items() if name != board},
                "history_age_s": 2.0, "inference_identity": identity,
            }
            self.options[board] = home / "backend.json"
            _json(self.options[board], options)
            stderr = stack.enter_context((home / "inference_stderr.log").open("wb"))
            child = subprocess.Popen(
                [sys.executable, "-m", "yopo_l0.stage_c_sim", "replay",
                 "--fixture", str(golden), "--socket", str(endpoint),
                 "--log", str(home / "inference_service.jsonl")],
                stdout=subprocess.DEVNULL, stderr=stderr, env=self.child_env,
            )
            self.services[board] = child
            stack.callback(_stop_service, child)
        self.started_ns = monotonic_ns()
        self.prev_ns = self.started_ns
        self.depth_due_ns = self.prev_ns
        self.pose_due_ns = self.prev_ns
        self.arm_ns = None
        self.injection_ns = None
        self.restarted = False
        self.resume = False

        build = subprocess.run(["git", "rev-parse", "HEAD"], check=False,
                               capture_output=True, text=True).stdout.strip()
        _json(out / "manifest.json", {
            "schema": 1, "scenario": case, "boards": self.board_ids, "build_commit": build or None,
            "started_ns": self.started_ns,
            "source": "host-wall-clock-software-bench",
            "clock_mapping": {"clock": "host_monotonic_ns", "offset_ns": 0, "scale": 1.0, "error_ns": 0},
            "odometry_source": "host-plant-sensor-mm-quantization",
            "judge_source": "host-plant-truth-separate-stream",
            "depth_source": "software-planar-wall-rays-with-measured-approach-fixture",
            "inference_backend": "replay-fp32", "openblas_threads_per_subprocess": 1,
            "dt_nominal_ns": POSE_PERIOD_NS, "normal_duration_s": normal_duration_s,
            "host_brake": "receiver-local-viscous-stop-when-no-valid-automation",
            "plant": "pva-point-mass-ctbr-body-rates-and-normalized-thrust",
        })

    def _start(self, board):
        self.bench.start(board, extra_args=("--backend", "yopo_l0.stage_c_backend",
                                          "--backend-config", str(self.options[board])),
                         env=self.child_env)

    def _inject(self, now):
        self.injection_ns = now
        case = self.case
        if case == "normal":
            return
        board = "uav2" if case in ("one-way-drop", "restart") else "uav1"
        self.faults("fault_injected", board_id=board, fault=case, timestamp_ns=now)
        if case == "inference-timeout":
            self.services[board].send_signal(signal.SIGSTOP)
        if case == "restart":
            self.bench.processes[board].send_signal(signal.SIGSTOP)

    def _fault_value(self, board, now):
        if self.injection_ns is None:
            return None
        age = now - self.injection_ns
        if self.case == "all-blind" and board == "uav1" and age < 270_000_000:
            return "all-blind"
        if self.case == "one-way-drop" and age < 110_000_000:
            return {"uav1": "drop-probe", "uav2": "drop"}.get(board)
        return None

    def _update_fault(self, now):
        if self.injection_ns is None:
            return
        age = now - self.injection_ns
        if self.case == "inference-timeout" and age >= 320_000_000 and not self.resume:
            self.services["uav1"].send_signal(signal.SIGCONT)
            self.faults("fault_cleared", board_id="uav1", fault=self.case, timestamp_ns=now)
            self.resume = True
        if self.case == "restart" and age >= 300_000_000 and not self.restarted:
            stale = self.bench.processes["uav2"]
            stale.kill()
            stale.wait(2)
            self._start("uav2")
            self.faults("board_restarted", board_id="uav2", fault=self.case, timestamp_ns=now)
            self.restarted = True

    def _update_plant(self, now):
        dt = now - self.prev_ns
        take_depth = now >= self.depth_due_ns and dt >= 4_000_000
        if take_depth:
            self.depth_due_ns = now + DEPTH_PERIOD_NS
        for board, plant in self.plants.items():
            plant.advance(self.prev_ns, now, self.bench.flight_controllers[board])
            command = plant.used_command
            self.judge("plant_pose", board_id=board, source="sim-plant-truth/" + board, stamp_ns=now,
                       position_enu_m=plant.position.tolist(), velocity_enu_mps=plant.velocity.tolist(),
                       consumed_authority=command["authority"] if command else "NONE",
                       consumed_sequence=command["sequence"] if command else None,
                       consumed_session_id=command["session_id"] if command else None)
            distance = max(0.0, 5.0 - float(plant.position[0]) - .18)
            row = {
                "stamp_ns": now, "position_enu_m": np.round(plant.position, 3).tolist(),
                "velocity_enu_mps": plant.velocity.tolist(), "acceleration_enu_mps2": plant.acceleration.tolist(),
                "rotation_enu_flu": plant.rotation.tolist(), "depth_stamp_ns": now - 2_000_000 if take_depth else None,
                "plane_range_m": distance, "enabled": self.arm_ns is not None,
                "fault": self._fault_value(board, now),
            }
            data = json.dumps(row, allow_nan=False).encode("utf-8")
            self.sensor.sendto(data, self.input_addrs[board])
            self.inputs[board]("sensor_sent", board_id=board, **row)
        self.prev_ns = now

    def run(self):
        until = monotonic_ns() + 6 * NS
        while not all(path.is_socket() for path in self.infer_endpoints):
            if monotonic_ns() >= until:
                raise TimeoutError("replay candidate service did not bind its socket")
            self.bench.collect(.01)
        for board in self.board_ids:
            self._start(board)
        startup_deadline = monotonic_ns() + 8 * NS
        while True:
            self.bench.collect(.004)
            now = monotonic_ns()
            if now >= self.pose_due_ns:
                self._update_plant(now)
                self.pose_due_ns = now + POSE_PERIOD_NS
            self._update_fault(now)
            for child in self.bench.processes.values():
                if child.poll() is not None:
                    raise RuntimeError("board runtime exited before the scenario ended")
            if self.arm_ns is None and all(
                records and records[-1]["authority"] == "PVA"
                and records[-1]["disposition"] == "hold" and records[-1]["peer_hold_ready"] is True
                for records in self.bench.received.values()
            ):
                self.arm_ns = now
                self.faults("all_receivers_holding", timestamp_ns=now)
            if self.injection_ns is None:
                tracked = [sum(row["authority"] == "CTBR" and row["disposition"] == "tracking" for row in rows)
                           for rows in self.bench.received.values()]
                if min(tracked) >= 5:
                    self._inject(now)
                elif now > startup_deadline:
                    raise TimeoutError("all three receivers did not establish active CTBR tracking")
            else:
                duration = self.normal_duration_ns if self.case == "normal" else 1_350_000_000
                if now - self.injection_ns >= duration:
                    break
        peak = {key: plant.peak_speed for key, plant in self.plants.items()}
        if max(peak.values()) > 2.2:
            raise RuntimeError(f"host speed exceeded the declared peer physical bound: {peak}")
        self.faults("run_complete", peak_speed_mps=peak)


def run_case(out: Path, case: str, *, normal_duration_s: float = .50) -> dict:
    out.mkdir(parents=True)
    with ExitStack() as stack:
        socket_root = Path(stack.enter_context(TemporaryDirectory(prefix="yopo-c-")))
        simulation = Simulation(out, case, stack, socket_root, normal_duration_s)
        simulation.run()
        simulation.bench.close()
    result = recompute(out)
    _json(out / "result.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    run = commands.add_parser("run")
    run.add_argument("--out", type=Path, required=True)
    run.add_argument("--scenario", choices=(*CASES, "faults"), default="normal")
    run.add_argument("--duration-s", type=float, default=.50, help="normal-case observation time after CTBR startup")
    replay = commands.add_parser("replay")
    replay.add_argument("--fixture", type=Path, required=True)
    replay.add_argument("--socket", type=Path, required=True)
    replay.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "replay":
        _run_service(args.fixture, args.socket, args.log)
        return 0
    if args.scenario == "faults":
        args.out.mkdir(parents=True)
        results = [run_case(args.out / case, case) for case in CASES[1:]]
        return int(not all(row["software_scenario_observed"] for row in results))
    else:
        if not np.isfinite(args.duration_s) or args.duration_s <= 0.0:
            parser.error("--duration-s must be finite and positive")
        result = run_case(args.out, args.scenario, normal_duration_s=args.duration_s)
        return int(not result["software_scenario_observed"])


if __name__ == "__main__":
    raise SystemExit(main())
