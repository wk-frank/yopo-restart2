import math
import numpy as np
import pytest
from conftest import ROOT
from yopo_l0.depth_accel import conservative_foreground_downsample, prepare_d435_dual_depth
from yopo_l0.depth_local import DepthLocalField
from yopo_l0.scene import load_card
from yopo_l0.sensor import DepthFrame
from yopo_l0.sitl_bridge import LiveFleetConfig, make_live_session


def test_planning_is_stripped_of_map_and_has_four_times_obstacles():
    card = load_card(ROOT / "scenes/explore-6-depth-only-density4-900.json")
    assert len(card.card["geometry"]["cylinders"]) == 12100
    session = make_live_session(card, policy="explore", live=LiveFleetConfig(depth_source="gazebo"))
    assert session.fleet.depth_only
    assert session.card.obstacles == ()
    assert "wide_field" not in session.card.card
    assert session.card.card["geometry"] == {}
    assert all(not s.bridge.camera._obstacles for s in session.slots)


def test_observed_surface_and_unknown_space():
    field = DepthLocalField((0, 0, 0))
    assert field.min_distance([[4, 0, 0]]) == 0
    frame = DepthFrame(0, np.full((96, 160), 5.0, np.float32), (80, 80, 80, 48))
    field.observe(frame, np.zeros(3), np.eye(3))
    assert field.min_distance([[3, 0, 0]]) > 1.9
    assert field.min_distance([[5, 0, 0]]) == 0
    assert field.min_distance([[3, 8, 0]]) == 0


def test_tri_state_mask_rejects_nan_blind_spot_and_bounds_swept_radius():
    field = DepthLocalField((0, 0, 0), strict_unknown=True, swept_radius_m=0.35)
    depth = np.full((96, 160), np.inf, dtype=np.float32)
    # Center patch has a stereo hole / <MinZ blind spot (NaN).
    depth[40:56, 70:90] = np.nan
    # Left sector has a real obstacle at 6.0 m.
    depth[40:56, 20:40] = 6.0
    field.observe(DepthFrame(0.0, depth, (80.0, 80.0, 80.0, 48.0)), np.zeros(3), np.eye(3))

    # Optical axis ray hits the NaN blind spot -> must be rejected as unknown (0.0), not far-clip free!
    assert field.min_distance([[3.0, 0.0, 0.0]]) == 0.0
    # Right sector has verified +inf -> admitted as known free space.
    assert field.min_distance([[3.0, -1.5, 0.0]]) > 2.0
    # Lateral point at 0.8 m outside frustum is outside swept_radius_m=0.35 -> rejected!
    assert field.min_distance([[0.0, 0.8, 0.0]]) == 0.0
    # Point within 0.30 m of body origin is inside verified swept sphere -> admitted.
    assert field.min_distance([[0.0, 0.25, 0.0]]) > 2.0


def test_extrinsic_camera_rotation_shifts_projected_surface_correctly():
    pitch_rad = math.radians(10.0)
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    # Pitch camera up by 10 deg around +Y body axis.
    R_bc = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    field = DepthLocalField((0, 0, 0), strict_unknown=True, cam_rotation=R_bc)
    depth = np.full((96, 160), 5.0, dtype=np.float32)
    field.observe(DepthFrame(0.0, depth, (80.0, 80.0, 80.0, 48.0)), np.zeros(3), np.eye(3))
    # Center pixel ray is at (5*cos(10°), 0, -5*sin(10°)) in body frame.
    surf_pt = np.array([[5.0 * c, 0.0, -5.0 * s]])
    assert float(field.tree.query(surf_pt)[0][0]) < 0.15


def test_conservative_downsample_and_dual_depth_border_semantics():
    hi = np.full((192, 320), np.inf, dtype=np.float32)
    # Single thin pole pixel at 2.4 m inside bin (48, 80).
    hi[96, 160] = 2.4
    # Stereo hole pixel inside bin (20, 20).
    hi[40, 40] = np.nan
    lo = conservative_foreground_downsample(hi, (96, 160))
    assert lo[48, 80] == pytest.approx(2.4)
    assert np.isnan(lo[20, 20])
    assert np.isposinf(lo[10, 10])

    net_in, safety = prepare_d435_dual_depth(lo, valid_col_span=(6, 154))
    assert net_in.shape == (1, 1, 96, 160)
    # Narrow-FOV outer columns must be NaN in safety map, but replicated (non-zero) in network tensor.
    assert np.isnan(safety[:, :6]).all() and np.isnan(safety[:, 154:]).all()
    assert float(np.min(net_in[:, :, :, :6])) > 0.5
