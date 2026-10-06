#!/usr/bin/env python3
"""Simulation/real comparison requires matching frame, command program and acquisition stamps."""
import argparse,json,math,pathlib,statistics
from calibration_evidence import command_origin as read_command_origin, initial_response
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);p.add_argument('--trial',default='CAL-02');p.add_argument('--real',type=pathlib.Path);args=p.parse_args()
rows=[json.loads(x) for x in (args.run/'telemetry.jsonl').read_text().splitlines()];moving=[r for r in rows if r['commandLinear']!=0 or r['commandAngular']!=0];assert rows and moving
command_origin,origin_precision=read_command_origin(args.run,rows)
yaw_steps=[(b['yaw']-a['yaw']+180)%360-180 for a,b in zip(rows,rows[1:])]
unwrapped=[0.]
for step in yaw_steps:unwrapped.append(unwrapped[-1]+step)
pos=lambda r:(r['position']['x'],r['position']['z'])
dist=lambda a,b:math.dist(pos(a),pos(b))
path=sum(dist(a,b) for a,b in zip(rows,rows[1:]));release=next((r for r in rows if r['simulationTime']>moving[-1]['simulationTime'] and r['commandLinear']==0),rows[-1]);stop=next((r for r in rows if r['simulationTime']>=release['simulationTime'] and math.hypot(r['velocity']['x'],r['velocity']['z'])<.01),rows[-1])
result={'schema':'crane-turtlebot-calibration-v1','trial':args.trial,'dataSource':'simulation','seed':json.loads((args.run/'scenario.json').read_text())['seed'],'physicalFitAvailable':False,'initialPosition':rows[0]['position'],'maximumHeightMeters':max(x['position']['y'] for x in rows),'maximumUncommandedSettlingDisplacementMeters':max(dist(rows[0],x) for x in rows if x['simulationTime']<command_origin),'commandProgramOriginUnitySeconds':command_origin,'comparisonTimeOrigin':'first nonzero command acquisition','fixedStepSeconds':rows[0]['fixedDeltaTime'],'durationSeconds':rows[-1]['simulationTime']-rows[0]['simulationTime'],'endpointDistanceMeters':dist(rows[0],rows[-1]),'sampledPathMeters':path,'finalYawDegrees':rows[-1]['yaw'],'maximumYawExcursionDegrees':max(unwrapped)-min(unwrapped),'sampledAbsoluteYawTravelDegrees':sum(abs(x) for x in yaw_steps),'maximumAngularSpeedRadiansPerSecond':max(abs(r['angularVelocity']['y']) for r in rows),'commandedAbsoluteAngularTravelDegrees':sum(abs(r['commandAngular'])*(n['simulationTime']-r['simulationTime'])*180/math.pi for r,n in zip(rows,rows[1:])),'maximumSpeedMetersPerSecond':max(math.hypot(r['velocity']['x'],r['velocity']['z']) for r in rows),'stoppingDistanceMeters':dist(release,stop),'meanWheelSlipProxy':statistics.mean(r['slipProxy'] for r in moving),'collisionCount':rows[-1]['collisionCount'],'minimumRadialClearanceProxyMeters':min(r['minimumClearance'] for r in rows),'realComparison':None}
result.update(initial_response(rows,command_origin))
result['commandTimeOriginPrecision']=origin_precision
if args.real:
 real=[json.loads(x) for x in args.real.read_text().splitlines()];assert len(real)>1
 # Require timestamp-aligned samples; no silent rescaling/registration to improve apparent fit.
 pairs=[]
 for sim in rows:
  trial_time=sim['simulationTime']-command_origin
  measured=min(real,key=lambda x:abs(x['simulationTime']-trial_time))
  if abs(measured['simulationTime']-trial_time)<=.05:pairs.append((sim,measured))
 assert pairs,'No acquisition-aligned real samples';result['physicalFitAvailable']=True;result['realComparison']={'samples':len(pairs),'trajectoryRmseMeters':math.sqrt(statistics.mean(dist(a,b)**2 for a,b in pairs)),'finalPositionErrorMeters':dist(rows[-1],real[-1]),'finalYawErrorDegrees':(rows[-1]['yaw']-real[-1]['yaw']+180)%360-180}
(args.run/'calibration-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
