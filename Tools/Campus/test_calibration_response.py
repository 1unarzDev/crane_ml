from calibration_evidence import initial_response


def row(t,linear=.2,angular=0,speed=0,yaw_rate=0):
    return dict(simulationTime=t,commandLinear=linear,commandAngular=angular,
                velocity=dict(x=0,z=speed),angularVelocity=dict(y=yaw_rate))


def test_later_surface_spike_does_not_change_initial_rise_time():
    rows=[row(3.02,speed=.1),row(3.12,speed=.19),row(12,speed=.5)]
    d=initial_response(rows,3.01)
    assert abs(d['riseTime90PercentSeconds']-.11)<1e-9
    assert d['responseChannel']=='linear'


def test_rotation_uses_yaw_response_instead_of_translation_creep():
    d=initial_response([row(3.02,0,-.4,.2,-.1),row(3.2,0,-.4,0,-.38)],3.01)
    assert abs(d['riseTime90PercentSeconds']-.19)<1e-9
    assert d['responseChannel']=='angular'


def test_next_command_cannot_supply_a_missing_initial_response():
    d=initial_response([row(3.02,speed=.1),row(4,linear=.3,speed=.3)],3.01)
    assert d['riseTime90PercentSeconds'] is None
