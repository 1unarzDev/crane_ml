"""Meaningful geometry/fidelity invariants; runtime ROS QA remains required."""
import json,pathlib,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
MANIFEST=ROOT/'Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json'
def test_manifest_structure():
 subprocess.run([sys.executable,str(ROOT/'Tools/Campus/validate_campus.py')],check=True,capture_output=True)
def test_start_in_doorway_wall_is_rejected(tmp_path):
 m=json.loads(MANIFEST.read_text())
 next(s for s in m['scenarios'] if s['id']=='dock_ramp')['start']=[9,30]
 source=tmp_path/'invalid-spawn.json';source.write_text(json.dumps(m))
 result=subprocess.run([sys.executable,str(ROOT/'Tools/Campus/validate_campus.py'),'--manifest',str(source)],capture_output=True,text=True)
 assert result.returncode!=0 and 'dock_ramp start overlaps warehouse-door-east' in result.stderr
def test_low_geometry_has_collision_and_distinct_surfaces():
 m=json.loads(MANIFEST.read_text());b={b['id']:b for b in m['boxes']}
 assert b['threshold']['size'][1]==.003
 assert b['loading-ramp']['kind']=='ramp'
 assert len({x['surface'] for x in m['boxes'] if x['kind']=='floor'})>=6
 assert all(x['calibrationStatus']=='engineering-prior-unmeasured' for x in m['surfaceProfiles'])
def test_narrow_route_has_meaningful_clear_gap():
 m=json.loads(MANIFEST.read_text());s=next(x for x in m['scenarios'] if x['id']=='warehouse_narrow_aisle')
 left,right=sorted(s['obstacles'],key=lambda b:b['center'][0]);gap=right['center'][0]-right['size'][0]/2-(left['center'][0]+left['size'][0]/2)
 assert .9<=gap<=1.2
 assert left['size'][2]>=8 and right['size'][2]>=8
 assert left['center'][0]+left['size'][0]/2<s['start'][0]<right['center'][0]-right['size'][0]/2

def test_ramp_entry_is_flush_and_exit_matches_platform():
 import math
 m=json.loads(MANIFEST.read_text());b={v['id']:v for v in m['boxes']};r=b['loading-ramp'];a=math.radians(abs(r['pitch']));low=r['center'][1]+r['size'][1]/2*math.cos(a)-r['size'][2]/2*math.sin(a);high=r['center'][1]+r['size'][1]/2*math.cos(a)+r['size'][2]/2*math.sin(a)
 assert abs(low)<.001
 p=b['loading-raised-slab'];assert abs(high-(p['center'][1]+p['size'][1]/2))<.002

def test_annex_task_variation_is_reproducible_and_stays_in_task_areas():
 from generate_annex_trial import resolve
 a=resolve(741,'warehouse_compact_switchbacks')
 assert a==resolve(741,'warehouse_compact_switchbacks')
 assert a!=resolve(742,'warehouse_compact_switchbacks')
 for seed in range(1,101):
  trial=next(s for s in resolve(seed,'warehouse_compact_switchbacks')['scenarios'] if s['id']=='warehouse_compact_switchbacks')
  assert -12.72<=trial['start'][0]<=-12.58 and 5.1<=trial['start'][1]<=5.8
  assert -12.48<=trial['goal'][0]<=-12.32 and 22.2<=trial['goal'][1]<=23.5
  assert trial['goal'][1]-trial['start'][1]>16
