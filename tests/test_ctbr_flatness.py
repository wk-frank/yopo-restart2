import math
import numpy as np
import pytest
from yopo_l0.ctbr import (GRAVITY, FlatnessCTBRController, HoverThrustEstimator,
                           enu_flu_to_ned_frd_rates, flatness_attitude)
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import LiveFleetConfig, make_live_session, FakePx4Vehicle, VehicleInbox, odometry_message_from_odometry
from conftest import ROOT


def test_flatness_matches_numeric_rotation_derivative_with_tilt_and_yaw():
    # Nonzero jerk and yaw rate expose the omitted heading/tilt coupling.
    for t in np.linspace(0.1, 2.0, 25):
        def inputs(s):
            return np.array([2*s+s*s, -1+s**3, .4*s*s]), np.array([2+2*s, 3*s*s, .8*s])
        a,j=inputs(t)
        R,w=flatness_attitude(a,j,.6*t*t,1.2*t)
        eps=1e-6
        Rp=flatness_attitude(inputs(t+eps)[0],inputs(t+eps)[1],.6*(t+eps)**2,1.2*(t+eps))[0]
        Rm=flatness_attitude(inputs(t-eps)[0],inputs(t-eps)[1],.6*(t-eps)**2,1.2*(t-eps))[0]
        skew=R.T@((Rp-Rm)/(2*eps))
        assert np.max(np.abs(w-np.array([skew[2,1],skew[0,2],skew[1,0]]))) < 1e-5
        assert np.linalg.det(R) == pytest.approx(1.0)


def test_hover_sign_limits_and_singular_fallback():
    c=FlatnessCTBRController()
    z=np.zeros(3)
    cmd=c.command(z,z,z,z,0.,0.,z,z,np.eye(3))
    assert cmd.thrust == pytest.approx(.5)
    assert cmd.fields(1.)['thrust_body'][2] < 0
    assert np.allclose(cmd.rates_frd,0.)
    assert np.allclose(enu_flu_to_ned_frd_rates([1,2,3]),[1,-2,-3])
    for a in ([0,0,-GRAVITY],[10,0,-GRAVITY]):
        R,w=flatness_attitude(a,[1,2,3])
        assert np.isfinite(R).all() and np.isfinite(w).all()
        assert np.allclose(R.T@R,np.eye(3))
    cmd=c.command([100,100,100],[100,100,100],[100,100,100],[1000,1000,1000],2.,10.,z,z,np.eye(3))
    assert 0.05 <= cmd.thrust <= .98
    assert np.max(np.abs(cmd.rates_frd)) <= 3.0


def test_hover_calibration_rejects_nonsteady_samples():
    h=HoverThrustEstimator()
    for _ in range(40): h.observe_hover(.66,.01,.02)
    assert h.value == pytest.approx(.66)
    h.observe_hover(.2,1.,0.)
    assert h.value == pytest.approx(.66)


def test_ctbr_phase_modes_revert_for_terminal_and_land():
    card=load_card(ROOT/'scenes/explore-6-open-sitl80.json')
    session=make_live_session(card,policy='explore',live=LiveFleetConfig(initial_phase='PLAN',control_mode='ctbr',record_dt_s=.2))
    vehicles=[FakePx4Vehicle(d['start_enu']) for d in card.card['drones']]
    inbox={s.ident['namespace']:VehicleInbox(odometry=odometry_message_from_odometry(v.odometry(),0),armed=True,offboard=True,landed=False)
           for s,v in zip(session.slots,vehicles)}
    cmds=session.step(0.,inbox)
    assert all(c.rates_setpoint is not None and c.offboard['body_rate'] and not c.offboard['position'] for c in cmds)
    for s in session.slots: s.phase='LAND'
    cmds=session.step(.02,inbox)
    assert all(c.rates_setpoint is None and (c.offboard is None or not c.offboard['body_rate']) for c in cmds)
    assert session.result()['px4_parameters_written'] == []


def test_ctbr_requires_known_mode_and_supported_policy():
    with pytest.raises(ValueError): LiveFleetConfig(control_mode='invalid')
    with pytest.raises(ValueError): LiveFleetConfig(record_dt_s=float('nan'))
    card=load_card(ROOT/'scenes/g2-pillar.json')
    with pytest.raises(ValueError,match='requires policy=explore'):
        make_live_session(card,live=LiveFleetConfig(control_mode='ctbr'))


def test_ctbr_does_not_switch_to_pva_at_start_of_terminal_braking():
    card=load_card(ROOT/'scenes/explore-6-open-sitl80.json')
    session=make_live_session(card,policy='explore',live=LiveFleetConfig(initial_phase='PLAN',control_mode='ctbr'))
    vehicles=[FakePx4Vehicle(d['start_enu']) for d in card.card['drones']]
    inbox={s.ident['namespace']:VehicleInbox(odometry=odometry_message_from_odometry(v.odometry(),0),armed=True,offboard=True,landed=False)
           for s,v in zip(session.slots,vehicles)}
    session.step(0.,inbox)
    slot=session.slots[0]
    session.fleet._terminal_goal[slot.bridge.drone_id]=slot.bridge.goal.copy()
    session.fleet._last_code[slot.bridge.drone_id]='ARRIVED_HOLD'
    slot.position=np.array([15.,-7.5,1.5]);slot.velocity=np.array([5.,0.,0.])
    cmd=session._pack(slot,.02,heartbeat=True,setpoint=session._last_cmds[0].setpoint,
                      commands=[],planned=True,depth_source='scene-card',roundtrip=0.,position=slot.position)
    assert cmd.rates_setpoint is not None
    assert not slot.ctbr_finished
