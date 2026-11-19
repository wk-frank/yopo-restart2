import os
import numpy as np
from yopo_l0.wide_scene import generate_wide_geometry
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_l0.explore import _FastObstacleField
from conftest import ROOT


def test_every_map_tile_covered_and_launch_centre_safe():
    card=load_card(ROOT/'scenes/explore-6-center900-ctbr.json')
    trees=card.card['geometry']['cylinders']
    assert len(trees)==55*55
    cells={(int((t['center_xy'][0]+1100)//40),int((t['center_xy'][1]+1100)//40)) for t in trees}
    assert len(cells)==55*55
    assert sum(t['center_xy'][0]<0 for t in trees)>1000
    assert card.card['criteria']['frozen'] is False
    for drone in card.card['drones']:
        assert np.linalg.norm(np.asarray(drone['start_enu'])[:2]) < 18
        assert card.min_distance(np.asarray(drone['start_enu'])) >= 1.5
        assert card.card['task']['x_goal_m']-drone['start_enu'][0] >= 900
    assert generate_wide_geometry(card.card['wide_field']) == card.card['geometry']


def test_large_scene_distance_and_candidate_broadphase_equal_bruteforce():
    card=load_card(ROOT/'scenes/explore-6-center900-ctbr.json')
    field=_FastObstacleField(card.obstacles)
    rng=np.random.default_rng(3)
    pts=rng.uniform([-1100,-1100,.5],[1100,1100,10],(20,3))
    expected=[min(o.distance(p) for o in card.obstacles) for p in pts]
    assert np.allclose([card.min_distance(p) for p in pts],expected,atol=1e-12)
    assert field.min_distance(pts) == min(expected)


def test_spatial_sensor_culling_preserves_depth_and_noise(monkeypatch):
    card=load_card(ROOT/'scenes/explore-6-center900-ctbr.json')
    cfg=CameraConfig(width=24,height=16,noise_std_m=.02)
    monkeypatch.setenv('YOPO_SENSOR_ACCEL','1')
    for pos in ([0.,0.,2.5],[432.,7.,2.5],[-400.,-50.,2.5]):
        monkeypatch.setenv('YOPO_SENSOR_SPATIAL','0')
        a=DepthCamera(cfg,card.obstacles,np.random.default_rng(2)).capture(np.array(pos),.2,0.)
        monkeypatch.setenv('YOPO_SENSOR_SPATIAL','1')
        b=DepthCamera(cfg,card.obstacles,np.random.default_rng(2)).capture(np.array(pos),.2,0.)
        assert np.array_equal(a.depth,b.depth,equal_nan=True)
