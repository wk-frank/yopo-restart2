import numpy as np
import pytest
from conftest import ROOT
from yopo_l0.flight_evidence import acceptance
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import LiveFleetConfig, make_live_session


def test_three_uav_d435_card_strips_prior_map_and_scales_lateral_cohesion():
    card = load_card(ROOT / "scenes/explore-3-d435-center900.json")
    ids = [d["id"] for d in card.card["drones"]]
    assert ids == ["uav0", "uav1", "uav2"]
    assert len(card.card["geometry"]["cylinders"]) == 75616
    session = make_live_session(card, policy="explore", live=LiveFleetConfig(depth_source="gazebo"))
    fleet = session.fleet
    assert fleet.depth_only
    assert session.card.obstacles == ()
    assert session.card.card["geometry"] == {}
    # Cluttered sector uses low lateral gain; open sector ramps up to pull back into line.
    assert fleet.effective_lateral_gain(4.5) == pytest.approx(0.12)
    assert fleet.effective_lateral_gain(9.0) == pytest.approx(0.24)
    assert fleet.effective_lateral_gain(14.0) == pytest.approx(0.36)


def test_three_uav_acceptance_verifies_expected_id_set_and_rejects_mismatch():
    card = load_card(ROOT / "scenes/explore-3-d435-center900.json")
    ids = [d["id"] for d in card.card["drones"]]
    result = {
        "ok": True,
        "adapter": "px4-ros",
        "fake_px4": False,
        "arrived": 3,
        "control_mode": "ctbr",
        "px4_parameters_written": [],
        "scene_world": {"n_cylinders": 75616},
        "depth_sources": ["gazebo-native-depth"],
        "perception_contract": {
            "mode": "depth-only",
            "planning_geometry_count": 0,
            "scene_geometry_abort_enabled": False,
        },
        "sim_time_s": 240.0,
        "flight_record": {
            k: {
                "ctbr_ticks": 100,
                "max_horizontal_displacement_m": 940.0,
                "samples": [{"control": "ctbr", "p": [940.0, 0.0, 2.5], "truth_p": [940.0, 0.0, 2.5]}],
            }
            for k in ids
        },
        "truth_flight": {
            "max_horizontal_displacement_m": {k: 940.0 for k in ids},
            "collision_samples": 0,
            "samples": {k: 100 for k in ids},
            "missing_plan_samples": {k: 0 for k in ids},
        },
        "native_depth": {
            "source": "gazebo-native-depth",
            "fallback_used": False,
            "decode_errors": [],
            "vehicles": {
                f"px4_{i}": {"unique_consumed": 3000, "max_age_s": 0.08, "finite_pixels": 10000}
                for i in range(1, 4)
            },
        },
        "gazebo_video": {
            "source": "gazebo-native-rgb-camera",
            "frames": 2200,
            "first_gazebo_stamp_s": 10.0,
            "last_gazebo_stamp_s": 230.0,
            "ffmpeg_exit_code": 0,
            "errors": [],
            "mp4_sha256": "b" * 64,
        },
    }
    verdict = acceptance(result, card)
    assert verdict["passed"]
    assert verdict["checks"]["all_arrived"] is True
    assert verdict["checks"]["all_native_depth_streams_consumed"] is True

    # Spoofing arrived count or wrong namespace must fail closed.
    result["native_depth"]["vehicles"]["px4_9"] = result["native_depth"]["vehicles"].pop("px4_3")
    assert not acceptance(result, card)["passed"]
