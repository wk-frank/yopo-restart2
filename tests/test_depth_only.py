import numpy as np
from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.sensor import DepthFrame
from yopo_l0.depth_local import DepthLocalField
from yopo_l0.sitl_bridge import make_live_session,LiveFleetConfig

def test_planning_is_stripped_of_map_and_has_four_times_obstacles():
    card=load_card(ROOT/'scenes/explore-6-depth-only-density4-900.json')
    assert len(card.card['geometry']['cylinders'])==12100
    session=make_live_session(card,policy='explore',live=LiveFleetConfig(depth_source='gazebo'))
    assert session.fleet.depth_only
    assert session.card.obstacles==()
    assert 'wide_field' not in session.card.card
    assert session.card.card['geometry']=={}
    assert all(not s.bridge.camera._obstacles for s in session.slots)

def test_observed_surface_and_unknown_space():
    field=DepthLocalField((0,0,0))
    assert field.min_distance([[4,0,0]])==0
    frame=DepthFrame(0,np.full((96,160),5.,np.float32),(80,80,80,48))
    field.observe(frame,np.zeros(3),np.eye(3))
    assert field.min_distance([[3,0,0]])>1.9
    assert field.min_distance([[5,0,0]])==0
    assert field.min_distance([[3,8,0]])==0
