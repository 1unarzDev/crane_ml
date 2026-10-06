#!/usr/bin/env python3
"""Gate real production runs; these checks do not claim hardware calibration."""
import argparse,json,math,pathlib,statistics
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);p.add_argument('--trial',choices=['CAL-01','CAL-03','CAL-04','CAL-05','CAL-06','CAL-07','CAL-08'],required=True);a=p.parse_args()
rows=[json.loads(l) for l in (a.run/'telemetry.jsonl').read_text().splitlines()]
moving=[r for r in rows if r['commandLinear'] or r['commandAngular']]
assert moving,'No commands observed'
settle=max(math.hypot(r['position']['x']-rows[0]['position']['x'],r['position']['z']-rows[0]['position']['z']) for r in rows if r['simulationTime']<moving[0]['simulationTime'])
checks={'stableStartup':settle<.05,'noFall':min(r['position']['y'] for r in rows)>-.1}
metrics={'startupDisplacementMeters':settle,'finalPosition':rows[-1]['position']}
if a.trial=='CAL-01':
 distance=math.hypot(rows[-1]['position']['x']-rows[0]['position']['x'],rows[-1]['position']['z']-rows[0]['position']['z'])
 yaw=(rows[-1]['yaw']-rows[0]['yaw']+180)%360-180
 checks.update(straightDistance=3.0<distance<3.4,straightHeading=abs(yaw)<5)
 metrics.update(distanceMeters=distance,headingChangeDegrees=yaw)
elif a.trial=='CAL-04':
 # Compare with the declared v/omega circle; do not fit/align a better circle.
 origin=moving[0]; heading=math.radians(origin['yaw']); radius=.15/.2
 cx=origin['position']['x']+radius*math.cos(heading);cz=origin['position']['z']-radius*math.sin(heading)
 late=[r for r in moving if r['simulationTime']>moving[0]['simulationTime']+1]
 radial=[abs(math.hypot(r['position']['x']-cx,r['position']['z']-cz)-radius) for r in late]
 mean_rate=statistics.mean(r['angularVelocity']['y'] for r in late)
 checks.update(arcRadiusConsistent=max(radial)<.10,arcYawResponse=.18<mean_rate<.22)
 metrics.update(commandedRadiusMeters=radius,maximumRadialErrorMeters=max(radial),meanYawRate=mean_rate)
elif a.trial=='CAL-05':
 sequences={}
 for wheel in ['leftSurface','rightSurface']:
  sequence=[]
  for r in rows:
   if not sequence or sequence[-1]!=r[wheel]:sequence.append(r[wheel])
  sequences[wheel]=sequence
  concrete=[i for i,r in enumerate(rows) if r[wheel]=='normal-concrete']
  dirt=[i for i,r in enumerate(rows) if r[wheel]=='compacted-dirt']
  checks[wheel+'ConcreteToDirt']=len(concrete)>=3 and len(dirt)>=3 and min(concrete)<min(dirt)
 metrics['surfaceSequences']=sequences
elif a.trial=='CAL-08':
 checks['repeatedProgramObserved']=sum(1 for previous,current in zip(rows,rows[1:]) if previous['commandLinear']==0 and current['commandLinear']>.1)>=2
 checks['meaningfulRepeatedTravel']=sum(math.hypot(b['position']['x']-a['position']['x'],b['position']['z']-a['position']['z']) for a,b in zip(rows,rows[1:]))>12
 checks['stoppedAtEnd']=math.hypot(rows[-1]['velocity']['x'],rows[-1]['velocity']['z'])<.01
 metrics['finalWheelOdometryPositionErrorMeters']=math.hypot(rows[-1]['wheelOdometryPosition']['x']-rows[-1]['position']['x'],rows[-1]['wheelOdometryPosition']['z']-rows[-1]['position']['z'])
elif a.trial=='CAL-03':
 for sign in [-1,1]:
  segment=[r for r in moving if r['commandAngular']*sign>.3]
  late=[r for r in segment if r['simulationTime']>segment[0]['simulationTime']+1] if segment else []
  response=statistics.mean(r['angularVelocity']['y'] for r in late) if late else 0
  checks['yawResponse'+str(sign)]=.32<sign*response<.48
  metrics['yawResponse'+str(sign)]=response
elif a.trial=='CAL-06':
 checks['crossedSeam']=rows[-1]['position']['z']>30.5
elif a.trial=='CAL-07':
 checks['reachedRaisedSlab']=37.5<rows[-1]['position']['z']<40 and .34<rows[-1]['position']['y']<.38
from contact_evidence import classified_contacts
manifest=json.loads((a.run/'manifest.json').read_text());scenario=json.loads((a.run/'scenario.json').read_text())
events=[json.loads(l) for l in (a.run/'events.jsonl').read_text().splitlines()]
contacts=classified_contacts(manifest,scenario,events)
checks['noProhibitedObstacleContacts']=contacts['noProhibitedObstacleContacts']
metrics['contactEvidence']=contacts
report={'state':'PHYSICS_PASS' if all(checks.values()) else 'PARTIAL','trial':a.trial,'checks':checks,'metrics':metrics,'physicalFitAvailable':False}
(a.run/'contact-calibration-validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2));raise SystemExit(0 if all(checks.values()) else 1)
