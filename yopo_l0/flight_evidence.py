"""Read-only Gazebo evidence for long-range CTBR acceptance."""
import math
import numpy as np


class FlightTruthMeter:
    def __init__(self, card):
        self.card=card
        self.starts={d['id']:np.asarray(d['start_enu'],dtype=float) for d in card.card['drones']}
        self.displacement={key:0.0 for key in self.starts}
        self.samples={key:0 for key in self.starts}
        self.missing={key:0 for key in self.starts}
        self.min_clearance=math.inf
        self.min_peer=math.inf
        self.collisions=0
        self.body=float(card.card['criteria']['body_radius_m'])

    def note(self, positions, phases):
        active={key:np.asarray(p,dtype=float) for key,p in positions.items() if phases.get(key)=='PLAN'}
        for key,phase in phases.items():
            if phase!='PLAN': continue
            if key not in active:
                self.missing[key]+=1
                continue
            p=active[key]
            self.samples[key]+=1
            self.displacement[key]=max(self.displacement[key],float(np.linalg.norm(p[:2]-self.starts[key][:2])))
            clearance=self.card.min_distance(p)-self.body
            self.min_clearance=min(self.min_clearance,clearance)
            if clearance<=0.: self.collisions+=1
        keys=list(active)
        for i,key in enumerate(keys):
            for other in keys[i+1:]:
                dist=float(np.linalg.norm(active[key]-active[other]))
                self.min_peer=min(self.min_peer,dist)
                if dist<=2.*self.body: self.collisions+=1

    def summary(self):
        return {'source':'gazebo-dynamic-pose','max_horizontal_displacement_m':dict(self.displacement),
                'samples':dict(self.samples),'missing_plan_samples':dict(self.missing),
                'min_body_surface_clearance_plan_m':None if math.isinf(self.min_clearance) else self.min_clearance,
                'min_centre_peer_distance_plan_m':None if math.isinf(self.min_peer) else self.min_peer,
                'collision_samples':self.collisions}


def acceptance(result, card):
    """Fail closed if any requested evidence is missing; distance is per vehicle."""
    target=float(card.card.get('criteria',{}).get('min_horizontal_displacement_m',900.))
    records=result.get('flight_record',{})
    truth=result.get('truth_flight',{})
    ids=[d['id'] for d in card.card['drones']]
    def ctbr_distance(did):
        start=np.asarray(next(d['start_enu'] for d in card.card['drones'] if d['id']==did),dtype=float)
        return any(s.get('control')=='ctbr' and s.get('truth_p') is not None
                   and np.linalg.norm(np.asarray(s['p'])[:2]-start[:2])>=target
                   and np.linalg.norm(np.asarray(s['truth_p'])[:2]-start[:2])>=target
                   for s in records.get(did,{}).get('samples',[]))
    checks={
        'px4_sitl_success':result.get('ok') is True and result.get('adapter')=='px4-ros' and result.get('fake_px4') is False,
        'six_arrived':len(ids)==6 and result.get('arrived')==6,
        'ctbr_used_by_every_vehicle':result.get('control_mode')=='ctbr' and all(records.get(k,{}).get('ctbr_ticks',0)>0 for k in ids),
        'per_vehicle_ctbr_distance':all(ctbr_distance(k) for k in ids),
        'no_px4_parameter_writes':result.get('px4_parameters_written')==[],
        'all_world_obstacles_present':result.get('scene_world',{}).get('n_cylinders')==len(card.card['geometry']['cylinders']),
        'per_vehicle_odometry_900m':all(records.get(k,{}).get('max_horizontal_displacement_m',0.)>=target for k in ids),
        'per_vehicle_gazebo_900m':all(truth.get('max_horizontal_displacement_m',{}).get(k,0.)>=target for k in ids),
        'gazebo_collision_free':truth.get('collision_samples')==0 and all(truth.get('samples',{}).get(k,0)>0 for k in ids),
        'gazebo_coverage_complete':all(truth.get('samples',{}).get(k,0)>0 and truth.get('missing_plan_samples',{}).get(k,1)==0 for k in ids),
    }
    if card.card.get('criteria',{}).get('require_gazebo_native_depth',False):
        native=result.get('native_depth',{})
        vehicles=native.get('vehicles',{})
        checks['gazebo_native_depth_only']=(native.get('source')=='gazebo-native-depth'
            and native.get('fallback_used') is False and native.get('decode_errors')==[]
            and result.get('depth_sources')==['gazebo-native-depth'])
        checks['six_native_depth_streams_consumed']=(len(vehicles)==6 and all(
            value.get('unique_consumed',0)>=100 and value.get('max_age_s',1.)<=0.20
            and value.get('finite_pixels',0)>0 for value in vehicles.values()))
        video=result.get('gazebo_video') or {}
        first,last=video.get('first_gazebo_stamp_s'),video.get('last_gazebo_stamp_s')
        span=0. if first is None or last is None else last-first
        checks['gazebo_rendered_video']=(video.get('source')=='gazebo-native-rgb-camera'
            and video.get('ffmpeg_exit_code')==0 and video.get('errors')==[]
            and span>=0.90*float(result.get('sim_s',0.))
            and video.get('frames',0)>=0.90*10.*span and bool(video.get('mp4_sha256')))
    if (card.card.get('task') or {}).get('depth_only',False):
        contract=result.get('perception_contract',{})
        checks['no_prior_map_planning']=(contract.get('mode')=='depth-only' and contract.get('planning_geometry_count')==0
            and contract.get('scene_geometry_abort_enabled') is False)
        checks['four_times_obstacle_count']=len(card.card['geometry']['cylinders'])==12100
    return {'passed':all(checks.values()),'required_per_vehicle_horizontal_displacement_m':target,'checks':checks}
