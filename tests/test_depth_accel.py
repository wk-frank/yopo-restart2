import numpy as np
import pytest
from conftest import ROOT
from yopo_swarm_core.depth import network_depth as core_depth
from yopo_l0.depth_accel import network_depth
from yopo_l0.scene import load_card
from yopo_l0.sensor import DepthCamera,CameraConfig


def test_default_depth_path_remains_core_byte_for_byte(monkeypatch):
    monkeypatch.delenv('YOPO_DEPTH_BACKEND',raising=False)
    raw=np.random.default_rng(4).uniform(0,25,(48,80)).astype(np.float32)
    raw[::3,::2]=np.nan
    assert np.array_equal(network_depth(raw),core_depth(raw))
    monkeypatch.setenv('YOPO_DEPTH_BACKEND','invalid')
    with pytest.raises(ValueError):network_depth(raw)


def test_optional_opencv_matches_wide_scene_frames(monkeypatch):
    pytest.importorskip('cv2')
    monkeypatch.setenv('YOPO_DEPTH_BACKEND','opencv')
    monkeypatch.setenv('YOPO_SENSOR_SPATIAL','1')
    monkeypatch.setenv('YOPO_SENSOR_ACCEL','1')
    card=load_card(ROOT/'scenes/explore-6-center900-ctbr.json')
    cam=DepthCamera(CameraConfig.from_card(card.card['camera']),card.obstacles,np.random.default_rng(0))
    for i,x in enumerate([0.,60.,200.,400.,600.,800.,920.,940.]):
        raw=cam.capture(np.array([x,2.5,2.5]),0.,float(i)).depth
        assert np.array_equal(network_depth(raw),core_depth(raw))
    for raw in [np.full((96,160),np.nan,dtype=np.float32),np.full((27,49),12.,dtype=np.float32)]:
        assert np.array_equal(network_depth(raw),core_depth(raw))


def test_optional_dependency_is_checked_before_flight(monkeypatch):
    import sys
    from yopo_l0.depth_accel import validate_depth_backend
    monkeypatch.setitem(sys.modules,'cv2',None)
    monkeypatch.setenv('YOPO_DEPTH_BACKEND','core')
    assert validate_depth_backend()=='core'
    monkeypatch.setenv('YOPO_DEPTH_BACKEND','opencv')
    with pytest.raises(ImportError):validate_depth_backend()
