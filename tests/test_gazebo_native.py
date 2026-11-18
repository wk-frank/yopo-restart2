"""Native depth contract: units/stride, strict sensor input, and disposable SDF changes."""
from dataclasses import replace
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from yopo_l0.gazebo_native import decode_gz_depth, prepare_native_scene
from yopo_l0.sensor import CameraConfig


def test_gazebo_depth_metres_stride_stamp_and_invalid_format():
    cfg = CameraConfig(width=2, height=2)
    raw = np.array([[1.25, np.inf, 99], [2.75, -np.inf, 99]], dtype='<f4').tobytes()
    msg = NS(width=2, height=2, step=12, data=raw, pixel_format_type=13,
             header=NS(stamp=NS(sec=17, nsec=250000000)))
    frame = decode_gz_depth(msg, cfg)
    assert frame.stamp == 17.25 and frame.depth[0, 0] == 1.25 and frame.depth[1, 0] == 2.75
    assert np.isnan(frame.depth[:, 1]).all()
    strict = decode_gz_depth(msg, replace(cfg, strict_unknown=True))
    assert np.isposinf(strict.depth[0, 1]) and np.isnan(strict.depth[1, 1])
    msg.pixel_format_type = 3
    with pytest.raises(ValueError, match='pixel format'):
        decode_gz_depth(msg, cfg)
    msg.pixel_format_type, msg.step = 13, 3
    with pytest.raises(ValueError, match='stride'):
        decode_gz_depth(msg, cfg)


def test_explore_consumes_external_depth_and_never_renders_scene_card(monkeypatch):
    from conftest import ROOT
    from yopo_l0.scene import load_card
    from yopo_l0.sensor import DepthCamera, DepthFrame
    from yopo_l0.sitl_bridge import (LiveFleetConfig, make_live_session, VehicleInbox,
                                    FakePx4Vehicle, odometry_message_from_odometry)
    card = load_card(ROOT / 'scenes/explore-6-center900-gz-depth.json')
    session = make_live_session(card, seed=0, policy='explore',
                                live=LiveFleetConfig(initial_phase='PLAN', depth_source='gazebo'))
    def forbidden(*_args, **_kwargs):
        raise AssertionError('Synthetic camera fallback used')
    monkeypatch.setattr(DepthCamera, 'capture', forbidden)
    frames = {s.ident['namespace']: DepthFrame(1.0, np.full((96, 160), 11.0+i, np.float32), s.bridge.camera.cfg.intrinsics)
              for i, s in enumerate(session.slots)}
    boxes = {s.ident['namespace']: VehicleInbox(
        odometry=odometry_message_from_odometry(FakePx4Vehicle(np.array([0., 0., 2.5])).odometry(), 1000000),
        depth=frames[s.ident['namespace']], armed=True, offboard=True, landed=False, preflight_ok=True)
        for s in session.slots}
    session.step(1.0, boxes)
    for s in session.slots:
        frame = session.fleet._sense[s.bridge.drone_id][0]
        assert frame.depth is frames[s.ident['namespace']].depth
        assert s.depth_source == 'gazebo-native-depth'
    for box in boxes.values():
        box.depth = None
    session.step(1.02, boxes)
    assert 'DEPTH_MISSING' in session.fail_reason


def test_native_sdf_preserves_inertia_and_collisions_and_is_idempotent(tmp_path):
    world, model = tmp_path/'world.sdf', tmp_path/'model.sdf'
    world.write_text('<sdf><world name="default"><model name="obstacle"><static>true</static></model></world></sdf>')
    model.write_text('<sdf><model name="x500_base"><link name="base_link"><inertial><mass>2</mass></inertial><collision name="body"/></link></model></sdf>')
    cfg = CameraConfig()
    first = prepare_native_scene(world, model, cfg)
    second = prepare_native_scene(world, model, cfg)
    assert first == second
    link = ET.fromstring(model.read_text()).find('model/link')
    assert link.findtext('inertial/mass') == '2'
    assert len(link.findall('collision')) == 1 and len(link.findall('sensor')) == 1
    assert ET.fromstring(world.read_text()).find("world/model[@name='obstacle']") is not None
    assert link.findtext('sensor/camera/image/format') == 'R_FLOAT32'


def test_native_depth_rejects_unsupported_policy_before_ros_or_arming(tmp_path, monkeypatch):
    import importlib.util
    import sys
    from conftest import ROOT
    spec=importlib.util.spec_from_file_location('native_gazebo_cli_guard', ROOT/'scripts/sitl/yopo_l0_live.py')
    module=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    def forbidden(*args, **kwargs):
        raise AssertionError('ROS/arming must not be reached for an unsupported policy')
    monkeypatch.setattr(module, '_run_ros', forbidden)
    code=module.main(['--scene',str(ROOT/'scenes/g2-pillar.json'),'--out',str(tmp_path/'result.json'),
        '--n','1','--policy','upstream','--lockstep','--scene-world','--record-dt','.2','--depth-source','gazebo'])
    assert code==2
