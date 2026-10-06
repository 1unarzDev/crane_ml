"""Recorded-trial gates reject missing transitions and obstacle impacts."""
import json
import subprocess
import sys
from pathlib import Path

SCRIPT=Path(__file__).with_name('check_contact_calibration.py')


def sample(time, z, surface, command=0):
    return dict(simulationTime=time,position=dict(x=24,y=.02,z=z),yaw=0,
                velocity=dict(x=0,y=0,z=command),angularVelocity=dict(x=0,y=0,z=0),
                commandLinear=command,commandAngular=0,leftSurface=surface,
                rightSurface=surface,wheelOdometryPosition=dict(x=24,y=.02,z=z))


def trial(tmp_path, rows, name='CAL-05', contact=None):
    (tmp_path/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (tmp_path/'manifest.json').write_text(json.dumps(dict(boxes=[dict(id='pad',kind='floor'),dict(id='machine',kind='machine')])))
    (tmp_path/'scenario.json').write_text(json.dumps(dict(obstacles=[])))
    (tmp_path/'events.jsonl').write_text(json.dumps(dict(type='contact-enter',id=contact))+'\n' if contact else '')
    result=subprocess.run([sys.executable,str(SCRIPT),str(tmp_path),'--trial',name],capture_output=True,text=True)
    return result.returncode,json.loads(result.stdout)


def crossing():
    return [sample(i,17.4+max(0,i-2)*.15,'normal-concrete' if i<4 else 'compacted-dirt',.15 if 2<=i<7 else 0) for i in range(9)]


def test_both_wheels_cross_concrete_to_dirt(tmp_path):
    code,report=trial(tmp_path,crossing(),contact='pad')
    assert code==0 and report['state']=='PHYSICS_PASS'
    assert report['metrics']['contactEvidence']['terrainContactCallbacks']==1


def test_one_wheel_remaining_on_concrete_fails_transition(tmp_path):
    rows=crossing()
    for row in rows:row['rightSurface']='normal-concrete'
    code,report=trial(tmp_path,rows)
    assert code==1 and not report['checks']['rightSurfaceConcreteToDirt']


def test_reversed_surface_order_fails(tmp_path):
    rows=crossing()
    for row in rows:
        row['leftSurface']=row['rightSurface']='compacted-dirt' if row['simulationTime']<4 else 'normal-concrete'
    code,report=trial(tmp_path,rows)
    assert code==1 and not report['checks']['leftSurfaceConcreteToDirt']


def test_successful_transition_with_machine_contact_fails(tmp_path):
    code,report=trial(tmp_path,crossing(),contact='machine')
    assert code==1 and not report['checks']['noProhibitedObstacleContacts']


def test_straight_drive_requires_several_meters_and_small_heading_error(tmp_path):
    rows=[sample(0,2,'normal-concrete'),sample(3,2,'normal-concrete',.2),sample(20,5.2,'normal-concrete')]
    assert trial(tmp_path,rows,'CAL-01')[0]==0
    rows[-1]['yaw']=30
    assert trial(tmp_path,rows,'CAL-01')[0]==1
    rows[-1]['yaw']=0;rows[-1]['position']['z']=2.7
    assert trial(tmp_path,rows,'CAL-01')[0]==1


def test_arc_is_compared_to_commanded_circle_without_refitting(tmp_path):
    import math
    def arc(radius):
        rows=[sample(0,2,'normal-concrete')]
        for t in [3,5,8,12,16]:
            theta=(t-3)*.2
            row=sample(t,2+radius*math.sin(theta),'normal-concrete',.15)
            row['position']['x']=24+radius*(1-math.cos(theta))
            row['yaw']=math.degrees(theta);row['angularVelocity']['y']=.2
            rows.append(row)
        return rows
    assert trial(tmp_path,arc(.75),'CAL-04')[0]==0
    code,report=trial(tmp_path,arc(1.2),'CAL-04')
    assert code==1 and not report['checks']['arcRadiusConsistent']


def test_unknown_contact_is_not_excused_as_terrain(tmp_path):
    code,report=trial(tmp_path,crossing(),contact='unregistered-object')
    assert code==1 and report['metrics']['contactEvidence']['unknownContactCallbacks']==1


def test_calibration_analysis_reports_known_wheel_odometry_disagreement(tmp_path):
    import math
    rows=[sample(0,2,'normal-concrete'),sample(3,2,'normal-concrete',.2),sample(20,5.2,'normal-concrete')]
    for row,error in zip(rows,[0,.002,.005]):
        row.update(fixedDeltaTime=.01,slipProxy=0,collisionCount=0,minimumClearance=1)
        row['wheelOdometryPosition']['x']+=error
    trial(tmp_path,rows,'CAL-01')
    (tmp_path/'scenario.json').write_text(json.dumps(dict(seed=4101,obstacles=[])))
    analyzer=SCRIPT.with_name('analyze_calibration.py')
    result=subprocess.run([sys.executable,str(analyzer),str(tmp_path),'--trial','CAL-01'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    report=json.loads(result.stdout)
    assert math.isclose(report['finalWheelOdometryPositionErrorMeters'],.005,abs_tol=1e-9)
    assert math.isclose(report['maximumWheelOdometryPositionErrorMeters'],.005,abs_tol=1e-9)
    assert math.isclose(report['wheelOdometryPositionRmseMeters'],math.sqrt((.002**2+.005**2)/3),abs_tol=1e-9)
    assert report['physicalFitAvailable'] is False
