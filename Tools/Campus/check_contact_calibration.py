#!/usr/bin/env python3
"""Gate real production runs; these checks do not claim hardware calibration."""
import argparse,json,math,pathlib,statistics
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);p.add_argument('--trial',choices=['CAL-03','CAL-06','CAL-07'],required=True);a=p.parse_args()
rows=[json.loads(l) for l in (a.run/'telemetry.jsonl').read_text().splitlines()]
moving=[r for r in rows if r['commandLinear'] or r['commandAngular']]
assert moving,'No commands observed'
settle=max(math.hypot(r['position']['x']-rows[0]['position']['x'],r['position']['z']-rows[0]['position']['z']) for r in rows if r['simulationTime']<moving[0]['simulationTime'])
checks={'stableStartup':settle<.05,'noFall':min(r['position']['y'] for r in rows)>-.1}
metrics={'startupDisplacementMeters':settle,'finalPosition':rows[-1]['position']}
if a.trial=='CAL-03':
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
report={'state':'PHYSICS_PASS' if all(checks.values()) else 'PARTIAL','trial':a.trial,'checks':checks,'metrics':metrics,'physicalFitAvailable':False}
(a.run/'contact-calibration-validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2));raise SystemExit(0 if all(checks.values()) else 1)
