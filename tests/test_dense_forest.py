import xml.etree.ElementTree as ET
from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.forest_mesh import batch_trunk_visuals

def test_density_exceeds_five_times_previous_without_shrinking_trunks():
    card=load_card(ROOT/'scenes/explore-6-depth-only-dense-forest900.json')
    trees=card.card['geometry']['cylinders']
    assert len(trees)==75616 and len(trees)>=5*12100
    assert all(.4<=c['radius']<=.85 and 5<=c['z_hi']<=9 for c in trees)

def test_mesh_batching_preserves_collision_geometry(tmp_path):
    world=ET.fromstring('<world><model name="observed_obstacle_field"><link name="field"><collision name="c"><geometry><cylinder><radius>.6</radius><length>6</length></cylinder></geometry></collision><visual name="v"><pose>10 20 3 0 0 0</pose><geometry><cylinder><radius>.6</radius><length>6</length></cylinder></geometry></visual></link></model></world>')
    collision=ET.tostring(world.find('.//collision'))
    assert batch_trunk_visuals(world,tmp_path)==1
    assert ET.tostring(world.find('.//collision'))==collision
    lines=(tmp_path/'trunks-0000.obj').read_text().splitlines()
    vertices=[list(map(float,l.split()[1:])) for l in lines if l.startswith('v ')]
    assert len(vertices)==32
    assert min(p[2] for p in vertices)==0 and max(p[2] for p in vertices)==6
    assert abs(max(p[0] for p in vertices)-10.6)<1e-6
