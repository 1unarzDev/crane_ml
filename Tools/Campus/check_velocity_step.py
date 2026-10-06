#!/usr/bin/env python3
"""Reject startup instability or poor steady body response in the actual CAL-02 path."""
import argparse,json,pathlib,math,statistics
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);a=p.parse_args()
r=[json.loads(l) for l in (a.run/'telemetry.jsonl').read_text().splitlines()]
moving=[x for x in r if x['commandLinear']>0];assert moving
last=[x for x in moving if x['simulationTime']>moving[-1]['simulationTime']-1]
speed=statistics.mean(math.hypot(x['velocity']['x'],x['velocity']['z']) for x in last);command=statistics.mean(x['commandLinear'] for x in last)
settle=max(math.hypot(x['position']['x']-r[0]['position']['x'],x['position']['z']-r[0]['position']['z']) for x in r if x['simulationTime']<moving[0]['simulationTime'])
report={'state':'PHYSICS_PASS' if .8*command<speed<1.2*command and settle<.05 else 'PARTIAL','steadySpeed':speed,'command':command,'startupDisplacement':settle};print(json.dumps(report,indent=2));raise SystemExit(0 if report['state']=='PHYSICS_PASS' else 1)
