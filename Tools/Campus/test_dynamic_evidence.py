import json
from dynamic_evidence import measured_encounter


def event(t,x):
    return dict(type='obstacle-pose',id='cart',simulationTime=t,
                detail=json.dumps(dict(position=dict(x=x,y=.5,z=0),colliderEnabled=True)))


def robot(t,x=0):
    return dict(simulationTime=t,position=dict(x=x,y=.01,z=0))


def test_actual_pose_interpolation_measures_encounter():
    d=measured_encounter('cart',[event(1,0),event(1.1,.2)],[robot(1.05)])
    assert abs(d['minimumCenterDistanceMeters']-.1)<1e-8
    assert d['measuredCartDisplacementMeters']==.2


def test_missing_actual_poses_cannot_be_replaced_by_activation_or_commands():
    d=measured_encounter('cart',[dict(type='obstacle-activated',id='cart',simulationTime=0)],[robot(1)])
    assert d['minimumCenterDistanceMeters'] is None


def test_no_extrapolation_or_interpolation_across_stale_samples():
    d=measured_encounter('cart',[event(1,0),event(2,.2)],[robot(.9),robot(1.5),robot(2.1)])
    assert d['matchedRobotSamples']==0


def test_far_actual_cart_and_duplicate_timestamps_are_not_hidden():
    d=measured_encounter('cart',[event(1,5),event(1.1,5.2)],[robot(1)])
    assert d['minimumCenterDistanceMeters']==5
    d=measured_encounter('cart',[event(1,0),event(1,0)],[robot(1)])
    assert not d['monotonicAcquisitionTimes']
    assert d['matchedRobotSamples']==0
