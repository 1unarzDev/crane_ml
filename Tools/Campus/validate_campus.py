#!/usr/bin/env python3
"""Structural checks against authoritative campus geometry and scenario contracts."""
import argparse,json,math,pathlib,hashlib
p=argparse.ArgumentParser();p.add_argument('--manifest',type=pathlib.Path,default=pathlib.Path(__file__).parents[2]/'Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json');args=p.parse_args();m=json.loads(args.manifest.read_text())
assert m['schema']=='crane-industrial-campus-v1'
assert len(m['zones'])==4
surfaces={s['id']:s for s in m['surfaceProfiles']};assert len(surfaces)>=8
ids=[b['id'] for b in m['boxes']];assert len(ids)==len(set(ids))
for s in surfaces.values():
 assert 0<s['dynamicFriction']<=s['staticFriction']<=1.5
 assert 0<=s['rollingResistance']<=.1
 assert s['frictionCombine']=='Minimum' # explicit tire model must not double count contact friction
 assert s['calibrationStatus']=='engineering-prior-unmeasured'
scenario_ids=[s['id'] for s in m['scenarios']];assert len(scenario_ids)==len(set(scenario_ids))>=20
for s in m['scenarios']:
 assert s['seed']>0 and s['robot']=='turtlebot3-waffle-reference'
 assert s['approximateLengthMeters']>=8
 assert s['expectedOutcome'] in ['succeeded','aborted']
 for field in ['start','goal']:
  x,z=s[field];assert -16<x<40 and 0<z<62
  for b in m['boxes']:
   if b['kind'] in ['floor','threshold','ramp'] or b['center'][1]-b['size'][1]/2>.4:continue
   cx,cy,cz=b['center'];sx,sy,sz=b['size'];assert not(abs(x-cx)<sx/2+.19 and abs(z-cz)<sz/2+.19),f"{s['id']} {field} overlaps {b['id']}"
 for b in s['obstacles']:
  assert b['surface'] in surfaces
  assert b['remove']<0 or b['remove']>b['activate']
for b in m['boxes']:
 assert b['surface'] in surfaces and all(x>0 for x in b['size'])
print(json.dumps({'state':'STRUCTURAL_PASS','manifestSha256':hashlib.sha256(args.manifest.read_bytes()).hexdigest(),'zones':4,'scenarios':len(scenario_ids),'surfaceProfiles':len(surfaces),'canonicalBoxes':len(ids)},indent=2))
