"""Replay stage C authority transitions at a separate UDP flight-control receiver."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from pathlib import Path
from threading import Event, Thread
import time

from .board_control import BoardControl, BrakeTail, ControlPlan, ControlReference, Odometry
from .board_runtime import BoardBackend, BoardConfig, ControlTick, JsonlLog
from .stage_c_hold import HoldBench


def _handover(config: BoardConfig, log: JsonlLog) -> BoardBackend:
    gate = BoardControl(log=log)
    position = config.hold_position_enu_m
    forward = ControlReference(position, (0.3, 0.0, 0.0))
    hold = ControlReference(position)

    def step(tick: ControlTick):
        match tick.sequence:
            case 0:
                gate.request_mode("PVA", shadow=True)
            case 8:
                gate.request_mode("CTBR")
            case 16:
                gate.request_mode("PVA", terminal=True)
            case 24:
                gate.request_mode("MANUAL")
            case 26:
                gate.request_mode("CTBR")
        if tick.sequence in (0, 8, 16):
            ticket = gate.begin_plan(tick.now_ns + 50_000_000)
            reference = hold if tick.sequence == 16 else forward
            gate.offer_plan(ticket, ControlPlan(lambda _now: reference, tick.now_ns + 400_000_000),
                            received_ns=tick.now_ns)
        return gate.step(tick, Odometry(tick.now_ns, position))

    return BoardBackend(step, gate.invalidate)


def _deadlines(config: BoardConfig, log: JsonlLog) -> BoardBackend:
    gate = BoardControl(log=log)
    position = config.hold_position_enu_m
    forward = ControlReference(position, (0.3, 0.0, 0.0))
    brake = ControlReference(position, acceleration_enu_mps2=(-0.3, 0.0, 0.0))
    unblock = Event()
    localization_ticket = None

    def blocked_replan(deadline_ns: int) -> None:
        ticket = gate.begin_plan(deadline_ns)
        log("planner_blocked", plan_sequence=ticket.sequence, deadline_ns=ticket.deadline_ns)
        if unblock.wait(2):
            gate.offer_plan(ticket, ControlPlan(lambda _now: forward, time.monotonic_ns() + 200_000_000))

    def step(tick: ControlTick):
        nonlocal localization_ticket
        now = tick.now_ns
        match tick.sequence:
            case 0:
                gate.request_mode("CTBR")
                ticket = gate.begin_plan(now + 55_000_000)
                tail = BrakeTail(lambda _now: brake, now + 115_000_000, now + 265_000_000)
                gate.offer_plan(ticket, ControlPlan(lambda _now: forward, now + 145_000_000, tail))
            case 2:
                worker = Thread(target=blocked_replan, args=(now + 90_000_000,), daemon=True)
                worker.start()
            case 16:
                unblock.set()
            case 20 | 36:
                ticket = gate.begin_plan(now + 50_000_000)
                gate.offer_plan(ticket, ControlPlan(lambda _now: forward, now + 500_000_000))
            case 23:
                localization_ticket = gate.begin_plan(now + 400_000_000)
            case 29:
                gate.offer_plan(localization_ticket, ControlPlan(lambda _now: forward, now + 200_000_000))
            case 44:
                time.sleep(0.14)
        pose = Odometry(now, position, (0.20, 0.0, 0.0))
        return gate.step(tick, None if 25 <= tick.sequence < 34 else pose)

    return BoardBackend(step, gate.invalidate)


SCENARIOS = {"handover": _handover, "deadlines": _deadlines}


def build_backend(config, log, stopping, backend_config):
    scenario = backend_config.read_text(encoding="utf-8").strip()
    log("sim_input_bound", source_kind="authority_fixture", scenario=scenario)
    return SCENARIOS[scenario](config, log)


def run_authority(out: Path, scenario: str) -> None:
    out.mkdir(parents=True)
    scenario_file = out / "scenario.txt"
    scenario_file.write_text(scenario + "\n", encoding="utf-8")
    with ExitStack() as stack:
        bench = HoldBench(out, stack)
        stack.callback(bench.close)
        bench.start("uav1", extra_args=("--backend", "yopo_l0.stage_c_authority",
                                       "--backend-config", str(scenario_file)))
        receiver = bench.flight_controllers["uav1"]
        if scenario == "handover":
            bench.until(lambda: receiver.effective_authority == "MANUAL")
            until = time.monotonic_ns() + 100_000_000
        else:
            bench.until(lambda: bool(bench.received["uav1"]))
            until = bench.received["uav1"][0]["received_ns"] + 1_300_000_000
        bench.until(lambda: time.monotonic_ns() >= until)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="new run output directory")
    parser.add_argument("--scenario", choices=tuple(SCENARIOS), default="handover")
    args = parser.parse_args()
    run_authority(args.out, args.scenario)


if __name__ == "__main__":
    main()
