#!/usr/bin/env python3
"""Conservative planar connectivity check of authoritative manifest geometry.

Uses a 0.1 m grid and 0.24 m circular footprint. This is an independent geometry
check, not a replacement for LiDAR/Nav2/controller/contact runtime validation.
Ramps/thresholds/surfaces are traversable here; traversal needs physical trials.
"""
import argparse,collections,json,math,pathlib
def blocked(x,z,boxes):
 for b in boxes:
  if b['kind'] in ('floor','threshold','ramp') or b['center'][1]-b['size'][1]/2>.4:continue
  a=math.radians(-b['yaw']);dx=x-b['center'][0];dz=z-b['center'][2]
  lx=dx*math.cos(a)+dz*math.sin(a);lz=-dx*math.sin(a)+dz*math.cos(a)
  if abs(lx)<=b['size'][0]/2+.24 and abs(lz)<=b['size'][2]/2+.24:return True
 return False
def connectivity(m,s):
 step=.1;bounds=(-16,40,0,62)
 def point(i,j):return bounds[0]+i*step,bounds[2]+j*step
 def cell(p):return round((p[0]-bounds[0])/step),round((p[1]-bounds[2])/step)
 # Final stable obstacle state. Timed moving obstacles require runtime verification.
 boxes=m['boxes']+[b for b in s['obstacles'] if b['remove']<0 and not any(b['velocity'])]
 # Rasterize obstacles once rather than evaluating all boxes for every BFS neighbor.
 occupied=set()
 for b in boxes:
  if b['kind'] in ('floor','threshold','ramp') or b['center'][1]-b['size'][1]/2>.4:continue
  radius=math.hypot(b['size'][0],b['size'][2])/2+.3
  lo=cell([b['center'][0]-radius,b['center'][2]-radius]);hi=cell([b['center'][0]+radius,b['center'][2]+radius])
  for i in range(max(0,lo[0]),min(560,hi[0])+1):
   for j in range(max(0,lo[1]),min(620,hi[1])+1):
    if blocked(*point(i,j),[b]):occupied.add((i,j))
 start=cell(s['start']);goal=cell(s['goal']);q=collections.deque([start] if start not in occupied else []);seen={start:0};found=False
 while q:
  v=q.popleft()
  if v==goal:found=True;break
  for dx,dz in ((1,0),(-1,0),(0,1),(0,-1)):
   n=(v[0]+dx,v[1]+dz)
   if 0<=n[0]<=560 and 0<=n[1]<=620 and n not in seen and n not in occupied:seen[n]=seen[v]+1;q.append(n)
 expected=s['expectedOutcome']=='succeeded'
 return dict(scenario=s['id'],reachable=found,expectedReachable=expected,passCheck=found==expected,startBlocked=start in occupied,goalBlocked=goal in occupied,gridPathLengthMeters=seen[goal]*step if found else None)


def main():
 root=pathlib.Path(__file__).resolve().parents[2]
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--manifest',type=pathlib.Path,default=root/'Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json')
 p.add_argument('--scenario',help='Check only this resolved scenario; omit to check all scenarios.')
 args=p.parse_args();m=json.loads(args.manifest.read_text())
 if args.scenario and not any(s['id']==args.scenario for s in m['scenarios']):p.error('Unknown scenario: '+args.scenario)
 results=[connectivity(m,s) for s in m['scenarios'] if not args.scenario or s['id']==args.scenario]
 valid=all(r['passCheck'] and not r['startBlocked'] for r in results)
 print(json.dumps({'schema':'crane-campus-planar-connectivity-v1','state':'STRUCTURAL_PASS' if valid else 'PARTIAL','footprintRadiusMeters':.24,'gridResolutionMeters':.1,'results':results},indent=2))
 return 0 if valid else 1


if __name__=='__main__':raise SystemExit(main())
