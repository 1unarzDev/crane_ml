from copy import deepcopy
import json

from calibration_evidence import command_origin
from compare_physics_runs import compare


def trial():
    rows = [dict(simulationTime=i*.1, fixedDeltaTime=.01,
                 position=dict(x=0,y=.01,z=min(max(i-10,0),10)*.02),
                 yaw=359., velocity=dict(x=0,z=min(max(i-8,0),2,max(20-i,0))*.1),
                 angularVelocity=dict(y=0),slipProxy=0,collisionCount=0)
            for i in range(31)]
    return dict(rows=rows,program=dict(trial='CAL-02',rotationDegrees=180),
                commands=[(0,dict(linear=0,angular=0)),(1,dict(linear=.2,angular=0)),
                          (2,dict(linear=0,angular=0))],
                origin=1,precision='fixed-step-command-event',scenario=dict(seed=41),
                robotModel='nominal',physicsManifest=dict(boxes=[],surfaceProfiles=[]))


def test_same_motion_with_different_sampling_rate_passes():
    a=trial();b=deepcopy(a);b['rows']=b['rows'][::2]
    assert compare(a,b,'parity')['state']=='PHYSICS_PASS'


def test_translated_trajectory_does_not_get_registered_into_a_pass():
    a=trial();b=deepcopy(a)
    for row in b['rows']:row['position']['x']+=.1
    assert not compare(a,b,'parity')['checks']['positionRmse']


def test_different_commands_or_truncated_trial_fail():
    a=trial();b=deepcopy(a);b['commands'][1][1]['linear']=.3
    assert not compare(a,b,'parity')['checks']['commandEdgesMatchWithinFixedStep']
    b=deepcopy(a);b['rows']=b['rows'][:19]
    assert not compare(a,b,'parity')['checks']['fullCommandProgramObserved']


def test_heading_wrap_does_not_create_false_error():
    a=trial();b=deepcopy(a)
    for row in b['rows']:row['yaw']=-1
    assert compare(a,b,'parity')['state']=='PHYSICS_PASS'


def test_exact_command_origin_wins_over_later_telemetry_sample(tmp_path):
    event=dict(type='calibration-command',simulationTime=3.01,
               detail=json.dumps(dict(linear=.2,angular=0)))
    (tmp_path/'events.jsonl').write_text(json.dumps(event)+'\n')
    assert command_origin(tmp_path,[dict(simulationTime=3.08,commandLinear=.2)]) == (3.01,'fixed-step-command-event')


def test_transient_heading_error_fails_even_when_final_heading_matches():
    a=trial();b=deepcopy(a)
    for row in b['rows'][10:20]:row['yaw']+=10
    report=compare(a,b,'parity')
    assert report['checks']['finalYawError']
    assert not report['checks']['maximumYawError']
    assert report['state']=='PARTIAL'
