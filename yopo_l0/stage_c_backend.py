"""Board backend shared by all three-process stage C software scenarios."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import socket
from threading import Thread
from time import monotonic_ns

import numpy as np

from yopo_swarm_core.candidates import UPSTREAM_DECODER, decode_params
from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.infer_ipc import body_obs
from yopo_swarm_core.plan import KinematicLimits, hold_traj, quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from .board_control import BoardControl, BrakeTail, ControlPlan, ControlReference, Odometry, PlanEvidence
from .board_inference import BoardInference, InferenceIdentity
from .board_peers import PeerBudget, PeerGuard
from .board_perception import BoardPerception, ClockMapping, DepthExposure
from .board_runtime import BoardBackend
from .board_stopping import StoppingConstraints


NS = 1_000_000_000
INTRINSICS = (80.0, 80.0, 80.0, 48.0)
MOUNT = (0.18, 0.04, 0.0)
POSE_ROTATION = np.eye(3)
ZERO = np.zeros(3)
BOUNDS = KinematicLimits(2.2, 2.0, 30.0)
PEER_BUDGET = PeerBudget(peer_speed_bound_mps=2.2, tracking_error_m=0.12)


def _sample(track, stamp_ns):
    stamp = min(track.t_end, max(track.t_start, stamp_ns / NS))
    return ControlReference(*(tuple(float(x) for x in track.evaluate(stamp, i)) for i in range(4)))


def _points(track):
    return np.array([track.evaluate(t) for t in np.linspace(track.t_start, track.t_end, 13)])


def _stop(head, duration):
    p, v, a = head
    final = np.array([p + v * duration / 2 + a * duration**2 / 12, ZERO, ZERO])
    return quintic(head, final, duration)


@dataclass(frozen=True)
class Snapshot:
    pose: Odometry | None = None
    can_plan: bool = False
    reason: str | None = None
    clearance_m: float | None = None
    frame_id: int | None = None


class SoftwareBoard:
    def __init__(self, config, log, stopping, options, archive_path):
        self.config, self.log, self.stopping = config, log, stopping
        self.snapshot = Snapshot()
        self.gate = BoardControl(log=log)
        self.stopping_gate = StoppingConstraints()
        self.last_invalidation = None
        self.motion_requested = False
        self.perception = BoardPerception(
            options["camera"], mount_xyz_body=MOUNT, camera_clock=ClockMapping(),
            history_age_s=options["history_age_s"], log=log, archive_dir=archive_path / "depth",
        )
        self.peer = PeerGuard(
            config.board_id, int(log.identity["session_id"], 16), tuple(options["peer_bind"]),
            {key: tuple(addr) for key, addr in options["peers"].items()}, budget=PEER_BUDGET, log=log,
        )
        self.sensor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor.bind(tuple(options["sensor_bind"]))
        self.sensor.setblocking(False)
        self.sensor_sender = tuple(options["sensor_sender"])
        self.clock = None
        self.frame_id = 0
        self.frame = None
        self.plant_stamp_ns = None
        self.clearance_m = None
        self.last_fault = None
        self.fault_done = set()
        self.enabled = False
        self.initial_hold = False
        self.startup_peers_ready = False
        self.active = None
        self.probe_fault = None
        self.checked_until_s = 0.
        self.checked_fault = None
        self.pending = None
        self.last_requested_frame = None
        self.last_probe_ns = 0
        self.rejoins = set()
        self.ready = False
        self.inference = BoardInference(
            options["infer_socket"], session_id=log.identity["session_id"],
            identity=InferenceIdentity(**options["inference_identity"]), log=log,
        )
        self.worker = Thread(target=self.run, name="stage-c-planner", daemon=True)
        self.worker.start()

    def _depth(self, stamp_s, range_m, now_s, *, replay=False):
        self.frame_id += 1
        frame = DepthExposure(
            self.perception.camera_serial, self.frame_id, stamp_s,
            np.full((96, 160), range_m, dtype=np.float32), INTRINSICS,
        )
        if replay:
            self.log("preroll_depth", frame_id=self.frame_id, source="archived-approach-fixture")
        self.frame = self.perception.observe(frame, now_s=now_s)
        self.clearance_m = None if self.frame is None else min(3.0, max(0.0, range_m - 0.50))

    def _preroll(self, stamp_s, p, range_m):
        self.log("preroll_started", source="synthetic-measured-approach", stamp_s=stamp_s,
                 path_start_enu_m=(p - np.array([1.55, 0.0, 0.0])).tolist(), plant_start_enu_m=p.tolist())
        for delta in (-0.62, -0.55, -0.45, -0.35, -0.25, -0.15, -0.07):
            q = p + np.array([2.5 * delta, 0., 0.])
            self.perception.push_odometry(stamp_s + delta, q, (2.5, 0., 0.), POSE_ROTATION)
            if delta == -0.55:
                self._depth(stamp_s - 0.59, range_m + 1.475, stamp_s + delta, replay=True)

    def _read_sensor(self, now_ns):
        for _ in range(32):
            try:
                raw, sender = self.sensor.recvfrom(8192)
            except BlockingIOError:
                return
            if sender != self.sensor_sender:
                continue
            data = json.loads(raw)
            stamp = data["stamp_ns"]
            if stamp > now_ns or self.plant_stamp_ns is not None and stamp <= self.plant_stamp_ns:
                continue
            p = np.asarray(data["position_enu_m"], dtype=float)
            if self.plant_stamp_ns is None:
                self._preroll(stamp / NS, p, data["plane_range_m"])
            self.plant_stamp_ns = stamp
            self.enabled, self.last_fault = data["enabled"], data["fault"]
            self.perception.push_odometry(stamp / NS, p, data["velocity_enu_mps"],
                                          data["rotation_enu_flu"], data["acceleration_enu_mps2"])
            if data["depth_stamp_ns"] is not None:
                self._depth(data["depth_stamp_ns"] / NS,
                            0.0 if data["fault"] == "all-blind" else data["plane_range_m"], now_ns / NS)

    def _broadcast(self, now, pose):
        measured = (float(pose.stamp_s), tuple(float(x) for x in pose.position_enu))
        moving = np.linalg.norm(pose.velocity_enu) > 0.05
        if self.last_fault == "drop" and "drop" not in self.fault_done:
            tail = pose.position_enu + np.array([0., -2.7, 0.])
            fake = PiecewisePolynomial(now, [quintic(
                [pose.position_enu, pose.velocity_enu, ZERO], [tail, ZERO, ZERO], 2.6,
            )])
            sequence = self.peer.broadcast(fake, now_s=now, measured_at_s=measured[0],
                                           position_enu_m=measured[1], omit=("uav1",))
            self.log("conflicting_replan_injected", sequence=sequence, peer_id="uav1", sent_s=now)
            self.fault_done.add("drop")
            return
        if self.last_fault == "drop" and "drop" in self.fault_done:
            return  # preserve the subject's old fresh packet for the one-way probe
        active = self.active[0] if self.active is not None else None
        if active is not None and now + PEER_BUDGET.clock_error_s < active.t_end:
            self.peer.broadcast(active, now_s=now, measured_at_s=measured[0],
                                position_enu_m=measured[1])
        elif not moving:
            hold = hold_traj(pose.position_enu, now, now + 0.8)
            self.peer.broadcast(hold, now_s=now, measured_at_s=measured[0],
                                position_enu_m=measured[1], holding=True)
            self.initial_hold = True

    def _waiting_rejoin(self, now):
        for owner in self.config.peers:
            session = self.peer.pending_session(owner)
            if session is None:
                continue
            key = owner, session
            if key in self.rejoins:
                self.peer.verify_rejoin(owner, session, now_s=now)
            else:
                self.rejoins.add(key)

    def _long_probe(self, now, current):
        if monotonic_ns() < self.last_probe_ns + 15_000_000:
            return
        self.last_probe_ns = monotonic_ns()
        local_track = hold_traj(current.position_enu, now, now + 2.0)
        decision = self.peer.check(local_track, now_s=now, brake_start_s=now + 0.30)
        self.log("conflict_probe", peer_id=decision.peer_id, allowed=decision.allowed, reason=decision.reason)
        return decision

    def _recheck_active(self, now):
        if self.active is None or now >= self.active[0].t_end:
            self.checked_fault = None
        elif now >= self.checked_until_s:
            track, brake_at_s = self.active
            verdict = self.peer.check(track, now_s=now, brake_start_s=brake_at_s)
            self.checked_until_s = now + 1 / 15
            self.checked_fault = None if verdict.allowed else verdict.reason
            self.log("active_peer_recheck", allowed=verdict.allowed, reason=verdict.reason,
                     peer_id=verdict.peer_id, brake_at_s=brake_at_s, brake_end_s=track.t_end)
        return self.checked_fault

    def _admit(self, result, ticket, track_pose, now_ns):
        now = now_ns / NS
        start_ns = monotonic_ns()
        frame = self.frame
        try:
            best = int(np.argmin(result.score[0]))
            _, tail, _ = decode_params(result.traj_params[0], UPSTREAM_DECODER)
            velocity = frame.pose.rotation_enu_flu @ tail[best, 1]
            speed = float(np.linalg.norm(velocity))
            current_speed = float(np.linalg.norm(track_pose.velocity_enu))
            stopped = self.stopping_gate.evaluate(speed, current_speed, self.clearance_m)
            self.log("stopping_decision", stage="plan", frame_id=frame.frame_id,
                     request_id=result.request_id, **stopped.evidence())
            if not stopped.plan_allowed or speed <= 0.0:
                self.gate.decline_plan(ticket, stopped.reason if speed > 0.0 else "NO_FORWARD_CANDIDATE")
                return
            # The control lease can track a short prefix, and its fallback follows
            # the same prefix until the verified continuous stop branch begins.
            # An early fault therefore has at most 130 ms of committed prefix left.
            duration = 0.13
            p = track_pose.position_enu
            v = track_pose.velocity_enu
            delta = velocity - v
            v_next = v + delta * min(1.0, .045 / max(1e-9, float(np.linalg.norm(delta))))
            projected = p + (v + v_next) * (duration / 2)
            head = np.array([p, v, ZERO])
            switch = np.array([projected, v_next, ZERO])
            track = PiecewisePolynomial(now, [quintic(head, switch, duration), _stop(switch, .40)])
            brake_start_ns = now_ns + int(duration * NS)
            if any(check_derivative_bound(track, order, limit, max_nodes=128).verdict is not Verdict.SAFE
                   for order, limit in BOUNDS.by_order()):
                self.gate.decline_plan(ticket, "KINEMATIC_BOUND")
                return
            geometry = self.perception.check_first_segment(_points(track), now_s=now)
            if not geometry.allowed:
                self.gate.decline_plan(ticket, geometry.reason)
                return
            separation = self.peer.check(track, now_s=now, brake_start_s=brake_start_ns / NS)
            if not separation.allowed:
                self.gate.decline_plan(ticket, separation.reason)
                return
            evidence = PlanEvidence(frame.frame_id, round(frame.exposure_s * NS), brake_start_ns)
            plan = ControlPlan(
                lambda ns: _sample(track, ns), brake_start_ns,
                BrakeTail(lambda ns: _sample(track, ns), now_ns, int(track.t_end * NS), evidence),
                evidence,
            )
            if self.gate.offer_plan(ticket, plan):
                self.active = (track, brake_start_ns / NS)
                self.checked_until_s = now + 1 / 15
                self.checked_fault = None
                self.log("forward_plan", frame_id=frame.frame_id, request_id=result.request_id,
                         exposure_ns=round(frame.exposure_s * NS), target_speed_mps=speed,
                         valid_until_ns=plan.valid_until_ns, brake_valid_from_ns=now_ns,
                         brake_start_ns=brake_start_ns,
                         brake_valid_until_ns=plan.verified_brake.valid_until_ns)
        finally:
            self.log("planning_compute", frame_id=frame.frame_id,
                     duration_ns=monotonic_ns()-start_ns)

    def _plan(self, pose, now_ns):
        if self.frame is None:
            self.inference.invalidate("all_blind" if self.perception.last_reason == "ALL_BLIND" else "depth_unavailable")
        result = self.inference.poll(self.frame.frame_id if self.frame is not None else -1)
        if result is not None and self.pending is not None:
            if self.ready and self.motion_requested and pose is not None:
                self._admit(result, self.pending, pose, now_ns)
            else:
                self.gate.decline_plan(self.pending, "READINESS_REVOKED")
            self.pending = None
        if (not self.ready or not self.motion_requested or pose is None or self.frame is None
                or self.last_requested_frame == self.frame.frame_id):
            return
        frame = self.frame
        obs = body_obs(frame.pose.velocity_enu, frame.pose.acceleration_enu,
                       frame.pose.position_enu + (5.0, 0.0, 0.0),
                       frame.pose.position_enu, frame.pose.rotation_enu_flu)
        deadline = now_ns + 56_000_000
        self.last_requested_frame = frame.frame_id
        if self.inference.submit(frame.frame_id, frame.network_depth, obs, deadline_ns=deadline):
            self.pending = self.gate.begin_plan(deadline)

    def run(self):
        next_broadcast = self.peer.next_broadcast_s(monotonic_ns() / NS)
        try:
            while not self.stopping.wait(0.002):
                now_ns = monotonic_ns()
                self._read_sensor(now_ns)
                now_ns = monotonic_ns()
                now = now_ns / NS
                self.peer.poll(now)
                pose = self.perception.tracking_state(now)
                availability = self.peer.availability(now)
                self.startup_peers_ready = availability.allowed
                if pose is not None and now >= next_broadcast:
                    self._broadcast(now, pose)
                    next_broadcast = self.peer.next_broadcast_s(now)
                guard_fault = self._recheck_active(now) if availability.allowed else None
                if self.last_fault != "drop-probe":
                    self.probe_fault = None
                elif pose is not None and availability.allowed:
                    probe = self._long_probe(now, pose)
                    if probe is not None:
                        self.probe_fault = None if probe.allowed else probe.reason
                hazard = None
                if self.enabled and self.initial_hold:
                    if self.frame is None:
                        hazard = self.perception.last_reason
                    elif now - self.frame.exposure_s > self.perception.max_frame_age_s:
                        hazard = "STALE_DEPTH"
                    elif not availability.allowed:
                        hazard = availability.reason
                    else:
                        hazard = guard_fault or self.probe_fault
                self.ready = bool(self.enabled and availability.allowed and hazard is None
                                  and self.frame is not None)
                self.snapshot = Snapshot(
                    Odometry(self.plant_stamp_ns, tuple(pose.position_enu), tuple(pose.velocity_enu),
                             tuple(tuple(row) for row in pose.rotation_enu_flu)) if pose is not None else None,
                    self.ready, hazard, self.clearance_m,
                    self.frame.frame_id if self.frame is not None else None,
                )
                self._waiting_rejoin(now)
                self._plan(pose, now_ns)
        except Exception as error:
            self.log("board_backend_error", reason=f"{type(error).__name__}: {error}")
            self.snapshot = Snapshot(reason="BACKEND_FAILED")
            raise
        finally:
            self.inference.close()
            self.peer.close()
            self.sensor.close()

    def step(self, tick):
        snapshot = self.snapshot
        if snapshot.reason is not None and snapshot.reason != self.last_invalidation:
            self.gate.invalidate(snapshot.reason)
        self.last_invalidation = snapshot.reason
        if snapshot.can_plan and not self.motion_requested:
            self.gate.request_mode("CTBR")
            self.motion_requested = True
        if snapshot.pose is not None and snapshot.can_plan:
            active = self.active
            current = math.hypot(*snapshot.pose.velocity_enu_mps)
            target = current
            if active is not None and active[0].t_start <= tick.now_ns / NS < active[1]:
                target = math.hypot(*active[0].evaluate(tick.now_ns / NS, 1))
            stop = self.stopping_gate.evaluate(target, current, snapshot.clearance_m)
            self.log("stopping_decision", stage="control", frame_id=snapshot.frame_id,
                     sequence=tick.sequence, **stop.evidence())
            if stop.action != "allow":
                self.gate.invalidate(stop.reason, preserve_brake=stop.brake_room_verified)
                self.last_invalidation = stop.reason
        output = self.gate.step(tick, snapshot.pose)
        return replace(output, peer_hold_ready=self.startup_peers_ready)

    def close(self):
        self.worker.join(2.0)


def build_backend(config, log, stopping, backend_config):
    directory = Path(backend_config).parent
    options = json.loads(Path(backend_config).read_text(encoding="utf-8"))
    board = SoftwareBoard(config, log, stopping, options, directory)
    log("sim_input_bound", source_kind="host-plant-odometry-and-depth", source=options["camera"])
    return BoardBackend(board.step, board.gate.invalidate, board.close)
