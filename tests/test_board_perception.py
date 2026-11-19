import numpy as np
import pytest

from yopo_l0.board_perception import BoardPerception, ClockMapping, DepthExposure


K = (80.0, 80.0, 80.0, 48.0)


def test_exposure_pose_and_tri_state_masks_are_reproducible_with_motion(tmp_path):
    evidence = []
    board = BoardPerception(
        "435-a", mount_xyz_body=(0.18, 0.04, 0.0),
        camera_clock=ClockMapping(offset_s=3.0, error_s=0.005),
        log=lambda event, **row: evidence.append((event, row)), archive_dir=tmp_path,
    )
    assert board.push_odometry(4.00, [0., 0., 2.], [2., 0., 0.], np.eye(3))
    assert board.push_odometry(4.10, [0.2, 0., 2.], [2., 0., 0.], np.eye(3))

    raw = np.full((96, 160), 5.0, dtype=np.float32)
    raw[40:50, 78:80] = 0.0
    raw[44, 81] = np.inf
    raw[45, 81] = np.inf
    verified = np.zeros(raw.shape, dtype=bool)
    verified[45, 81] = True
    frame = DepthExposure("435-a", 1, 1.04, raw, K, verified, "calibrated replay", (6, 154))
    processed = board.observe(frame, now_s=4.12)
    assert processed is not None
    assert processed.exposure_s == pytest.approx(4.04)
    assert processed.pose.position_enu == pytest.approx([0.08, 0.0, 2.0])
    assert board.tracking_state(4.12).position_enu == pytest.approx([0.2, 0.0, 2.0])
    assert processed.network_depth.shape == (1, 1, 96, 160)
    assert np.isnan(processed.safety_depth_m[:, :6]).all()
    assert np.isnan(processed.safety_depth_m[40:50, 78:80]).all()
    assert np.isnan(processed.safety_depth_m[44, 81])
    assert np.isposinf(processed.safety_depth_m[45, 81])
    event, row = evidence[-1]
    assert event == "depth_observed"
    assert row["pose_before"]["stamp_s"] == 4.00
    assert row["pose_after"]["stamp_s"] == 4.10
    assert row["camera_clock"]["error_s"] == 0.005
    assert row["camera_origin_enu"] == pytest.approx([0.26, 0.04, 2.0])
    with np.load(tmp_path / row["archive"]) as record:
        assert record["depth_m"][44, 81] == np.inf
        assert not record["verified_far_free"][44, 81]
        assert record["verified_far_free"][45, 81]
    replay = DepthExposure.from_archive(tmp_path / row["archive"], row)
    fresh_board = BoardPerception("435-a", camera_clock=ClockMapping(**row["camera_clock"]))
    for sample in (row["pose_before"], row["pose_after"]):
        assert fresh_board.push_odometry(**sample)
    reproduced = fresh_board.observe(replay, now_s=row["received_s"])
    np.testing.assert_array_equal(reproduced.safety_depth_m, processed.safety_depth_m)
    np.testing.assert_array_equal(reproduced.network_depth, processed.network_depth)


def test_body_near_blind_band_requires_recent_measured_coverage_and_current_evidence():
    depth = np.full((96, 160), 5.0, dtype=np.float32)

    def prepare_with_motion(keep_previous_frame):
        board = BoardPerception("435-a", mount_xyz_body=(0.18, 0.04, 0.0))
        for stamp in (9.98, 10.05, 10.15, 10.25, 10.35, 10.45, 10.53, 10.60):
            x = 2.5 * (stamp - 10.60)
            assert board.push_odometry(stamp, [x, 0., 2.], [2.5, 0., 0.], np.eye(3))
            if stamp == 10.05 and keep_previous_frame:
                assert board.observe(DepthExposure("435-a", 1, 10.01, depth, K), now_s=10.05)
        assert board.observe(DepthExposure("435-a", 2, 10.55, depth, K), now_s=10.60)
        return board

    first_segment = [[0., 0., 2.], [0.75, 0., 2.]]
    without_history = prepare_with_motion(False)
    assert without_history.check_first_segment([[0., 0., 2.]], now_s=10.60).allowed
    denied = without_history.check_first_segment(first_segment, now_s=10.60)
    assert (denied.allowed, denied.reason) == (False, "NEAR_BLIND_BAND")

    after_history_expires = prepare_with_motion(True)
    assert after_history_expires.push_odometry(10.82, [0., 0., 2.], [0., 0., 0.], np.eye(3))
    assert after_history_expires.observe(DepthExposure("435-a", 3, 10.78, depth, K), now_s=10.82)
    assert after_history_expires.check_first_segment(first_segment, now_s=10.82).reason == "NEAR_BLIND_BAND"

    with_history = prepare_with_motion(True)
    approved = with_history.check_first_segment(first_segment, now_s=10.60)
    assert approved.allowed
    assert 10.01 in approved.coverage_exposures_s
    epoch = with_history.generation
    assert with_history.push_odometry(10.67, [0., 0., 2.], [0., 0., 0.], np.eye(3))
    unproven = np.full((96, 160), np.inf, dtype=np.float32)
    assert with_history.observe(DepthExposure("435-a", 3, 10.63, unproven, K), now_s=10.67) is None
    assert with_history.generation > epoch
    blocked = with_history.check_first_segment(first_segment, now_s=10.67)
    assert (blocked.allowed, blocked.reason) == (False, "ALL_BLIND")


def test_brackets_and_pose_resets_fail_closed_at_the_board_entry():
    board = BoardPerception("435-a", camera_clock=ClockMapping(error_s=.005))
    depth = np.full((96, 160), 5.0, dtype=np.float32)

    def frame(frame_id, stamp):
        return DepthExposure("435-a", frame_id, stamp, depth, K)

    assert board.push_odometry(5.000, [0., 0., 2.], [0., 0., 0.], np.eye(3))
    assert board.push_odometry(5.047, [0., 0., 2.], [0., 0., 0.], np.eye(3))
    assert board.observe(frame(1, 5.045), now_s=5.052) is None
    assert board.last_reason == "EXPOSURE_POSE_BRACKET_MISSING"

    assert board.push_odometry(5.10, [0., 0., 2.], [0., 0., 0.], np.eye(3))
    accepted = board.observe(frame(2, 5.060), now_s=5.10)
    assert accepted
    assert not board.push_odometry(5.11, [1.5, 0., 2.], [0., 0., 0.], np.eye(3))
    assert board.last_reason == "EKF_POSITION_JUMP"
    assert board.generation > accepted.generation
    assert board.tracking_state(5.11) is None
    assert not board.check_first_segment([[0., 0., 2.]], now_s=5.11).allowed

    assert board.push_odometry(5.14, [1.5, 0., 2.], [0., 0., 0.], np.eye(3))
    assert board.push_odometry(5.24, [1.5, 0., 2.], [0., 0., 0.], np.eye(3))
    fresh = board.observe(frame(3, 5.18), now_s=5.24)
    assert fresh
    assert board.observe(frame(4, 5.17), now_s=5.25) is None
    assert board.last_reason == "CAMERA_CLOCK_REGRESSION"
    assert board.generation > fresh.generation
    assert board.observe(frame(5, 5.19), now_s=5.25) is None
    assert board.last_reason == "CAMERA_CLOCK_UNSYNCHRONIZED"
    board.synchronize_camera_clock(ClockMapping(error_s=.005))
    assert board.observe(frame(1, 5.20), now_s=5.25)
    assert not board.push_odometry(5.25, [1.5, 0., 2.], [0., 0., 0.], np.eye(3), coordinate_version=2)
    assert board.last_reason == "COORDINATE_VERSION_CHANGED"
    assert board.tracking_state(5.25) is None


def test_uncertain_exposure_cannot_clear_a_sweep_outside_a_possible_frustum():
    depth = np.full((96, 160), 5., dtype=np.float32)

    def decision(offset, error):
        board = BoardPerception(
            "cam", mount_xyz_body=(.18, 0, 0),
            camera_clock=ClockMapping(offset_s=offset, error_s=error),
        )
        for t in (10.12, 10.175, 10.26, 10.38, 10.50, 10.57, 10.60):
            assert board.push_odometry(t, [2.5 * (t - 10.60), 0, 2], [2.5, 0, 0], np.eye(3))
            if t == 10.26:
                assert board.observe(DepthExposure("cam", 1, 10.20, depth, K), now_s=t)
        assert board.observe(DepthExposure("cam", 2, 10.536, depth, K), now_s=10.60)
        return board.check_first_segment([[0, 0, 2], [.10, 0, 2]], now_s=10.60)

    assert decision(0., 0.).allowed
    assert not decision(.024, 0.).allowed
    assert not decision(0., .025).allowed
