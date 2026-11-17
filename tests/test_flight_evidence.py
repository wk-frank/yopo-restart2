import numpy as np
from yopo_l0.flight_evidence import FlightTruthMeter, acceptance
from yopo_l0.scene import load_card
from conftest import ROOT


def _success_fixture():
    card=load_card(ROOT/'scenes/explore-6-center900-ctbr.json')
    ids=[d['id'] for d in card.card['drones']]
    result={'ok':True,'adapter':'px4-ros','fake_px4':False,'arrived':6,'control_mode':'ctbr',
            'px4_parameters_written':[], 'scene_world':{'n_cylinders':3025},
            'flight_record':{k:{'ctbr_ticks':100,'max_horizontal_displacement_m':940.} for k in ids},
            'truth_flight':{'max_horizontal_displacement_m':{k:940. for k in ids},
                'collision_samples':0,'samples':{k:100 for k in ids},'missing_plan_samples':{k:0 for k in ids}}}
    for d in card.card['drones']:
        point=[940.,d['start_enu'][1],2.5]
        result['flight_record'][d['id']]['samples']=[{'control':'ctbr','p':point,'truth_p':point}]
    return card,result


def test_acceptance_requires_every_vehicle_and_both_measured_distance_sources():
    card,result=_success_fixture()
    assert acceptance(result,card)['passed']
    result['flight_record']['uav5']['samples'][0]['control']='pva'
    assert not acceptance(result,card)['passed']
    result['flight_record']['uav5']['samples'][0]['control']='ctbr'
    result['truth_flight']['max_horizontal_displacement_m']['uav5']=899.9
    assert not acceptance(result,card)['passed']
    result['truth_flight']['max_horizontal_displacement_m']['uav5']=940.
    result['truth_flight']['collision_samples']=1
    assert not acceptance(result,card)['passed']
    assert not acceptance({},card)['passed']


def test_native_acceptance_requires_six_consumed_streams_and_real_video():
    _,result=_success_fixture()
    card=load_card(ROOT/'scenes/explore-6-center900-gz-depth.json')
    card.card['criteria']['require_gazebo_native_depth']=True
    assert not acceptance(result,card)['passed']
    result['depth_sources']=['gazebo-native-depth']
    result['native_depth']={'source':'gazebo-native-depth','fallback_used':False,'decode_errors':[],
        'vehicles':{f'px4_{i}':{'unique_consumed':3000,'max_age_s':.08,'finite_pixels':10000} for i in range(1,7)}}
    result['gazebo_video']={'source':'gazebo-native-rgb-camera','frames':2200,
        'first_gazebo_stamp_s':10.,'last_gazebo_stamp_s':230.,'ffmpeg_exit_code':0,'errors':[], 'mp4_sha256':'a'*64}
    assert acceptance(result,card)['passed']
    result['native_depth']['vehicles']['px4_6']['max_age_s']=.3
    assert not acceptance(result,card)['passed']
    result['native_depth']['vehicles']['px4_6']['max_age_s']=.08
    result['gazebo_video']['frames']=0
    assert not acceptance(result,card)['passed']


def test_truth_meter_detects_ground_collision_and_missing_vehicle():
    card=load_card(ROOT/'scenes/explore-6-center900-ctbr.json')
    meter=FlightTruthMeter(card)
    meter.note({'uav0':np.array([0.,0.,.1])},{'uav0':'PLAN','uav1':'PLAN'})
    report=meter.summary()
    assert report['collision_samples']==1
    assert report['missing_plan_samples']['uav1']==1
    meter.note({'uav0':np.array([0.,0.,.1])},{'uav0':'LAND'})
    assert meter.summary()['collision_samples']==1
