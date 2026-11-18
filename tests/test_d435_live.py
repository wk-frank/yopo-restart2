import math
import threading
import numpy as np
import pytest
from yopo_swarm_core import baseline as B
from yopo_swarm_core.infer_ipc import serve
from yopo_l0.d435_live import IpcCandidateSource, PoseRingBuffer, StoppingDistanceGate


def test_pose_ring_buffer_interpolates_at_exposure_and_invalidates_on_ekf_jump():
    buf = PoseRingBuffer(max_pos_jump_m=0.50, max_time_gap_s=0.20)
    assert buf.push(1.00, [0.0, 0.0, 2.0], [2.0, 0.0, 0.0], np.eye(3))
    assert buf.push(1.02, [0.04, 0.0, 2.0], [2.0, 0.0, 0.0], np.eye(3))
    mid = buf.at_exposure(1.01)
    assert mid is not None
    assert np.allclose(mid.position_enu, [0.02, 0.0, 2.0])
    assert np.allclose(mid.rotation_enu_flu, np.eye(3))

    # Inject a 1.5 m EKF reset jump -> must invalidate and refuse further interpolation.
    assert not buf.push(1.04, [1.60, 0.0, 2.0], [2.0, 0.0, 0.0], np.eye(3))
    assert not buf.valid
    assert buf.invalid_reason == "EKF_POSITION_JUMP"
    assert buf.at_exposure(1.01) is None

    buf.reset()
    assert buf.push(2.00, [1.60, 0.0, 2.0], [0.0, 0.0, 0.0], np.eye(3))
    assert buf.valid


def test_stopping_distance_gate_matches_worked_example():
    gate = StoppingDistanceGate(response_delay_s=0.15, effective_decel_mps2=2.0, margin_m=0.70)
    # At 3 m/s: 3*0.15 + 2.25 + 0.70 = 3.40 m required clearance.
    assert gate.required_clearance_m(3.0, measured_brake_m=2.25) == pytest.approx(3.40)
    assert not gate.allow_speed(3.0, reliable_clearance_m=3.00, measured_brake_m=2.25)
    assert gate.allow_speed(3.0, reliable_clearance_m=3.45, measured_brake_m=2.25)
    v_safe = gate.max_safe_speed_mps(3.00)
    assert v_safe == pytest.approx(2.747950130825634)
    assert gate.allow_speed(2.74, reliable_clearance_m=3.00)
    assert not gate.allow_speed(2.75, reliable_clearance_m=3.00)
    assert gate.allow_speed(v_safe, reliable_clearance_m=3.00)


def test_ipc_candidate_source_roundtrips_and_records_latency(tmp_path):
    sock_path = tmp_path / "uav0_test.sock"
    golden = np.load(B.GOLDEN_PATH)

    def fake_forward(depth: np.ndarray, obs: np.ndarray):
        assert depth.shape == (1, 1, 96, 160)
        assert obs.shape == (1, 9)
        return (
            np.asarray(golden["traj_params"], dtype=np.float32),
            np.asarray(golden["score"], dtype=np.float32),
            np.asarray(golden["radius"], dtype=np.float32),
        )

    th = threading.Thread(
        target=serve,
        args=(sock_path, fake_forward, "trt-fp32-mock"),
        kwargs={"max_requests": 2},
        daemon=True,
    )
    th.start()
    import time

    for _ in range(50):
        if sock_path.exists():
            break
        time.sleep(0.01)

    src = IpcCandidateSource(str(sock_path))
    raw_depth = np.full((96, 160), 10.0, dtype=np.float32)
    p, s, r = src.forward_raw(
        raw_depth,
        np.array([1.0, 0.0, 0.0]),
        np.zeros(3),
        np.array([10.0, 0.0, 2.0]),
        np.array([0.0, 0.0, 2.0]),
        np.eye(3),
    )
    assert p.shape == (15, 14) and s.shape == (15,) and r.shape == (15, 20)
    assert np.allclose(p, golden["traj_params"][0], rtol=1e-4, atol=1e-5)
    summary = src.summary()
    assert summary["backend"] == "trt-fp32-mock"
    assert summary["infer"]["count"] == 1 and summary["rtt"]["count"] == 1
    src.close()
