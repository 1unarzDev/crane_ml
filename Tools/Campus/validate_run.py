#!/usr/bin/env python3
"""Keep action success, measured behavior, transport, sensing and benchmark validity distinct."""
import argparse,json,pathlib,gzip,math,statistics,hashlib,subprocess,sys
from no_path_evidence import planner_failure_evidence
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);p.add_argument('--require-recovery',action='store_true');p.add_argument('--require-detour',action='store_true');p.add_argument('--require-dynamic',action='store_true');p.add_argument('--require-no-path',action='store_true');a=p.parse_args();r=a.run
f=json.loads((r/'fixture-summary.json').read_text());s=json.loads((r/'scenario.json').read_text());h=json.loads((r/'navigation-reset-summary.json').read_text());t=[json.loads(x) for x in (r/'telemetry.jsonl').read_text().splitlines()];events=[json.loads(x) for x in (r/'events.jsonl').read_text().splitlines()] if (r/'events.jsonl').exists() else []
paths=f.get('planHistory',[]);ids={x.get('pathId') for x in paths};motion=f.get('pathMetrics',{});topic_counts={};stamps={};frames=set();scan_valid=scan_total=0;scan_min=math.inf;scan_max=0;observed_plans=[]
with gzip.open(r/'ros-evidence.jsonl.gz','rt') as stream:
 for line in stream:
  x=json.loads(line);topic=x['topic'];topic_counts[topic]=topic_counts.get(topic,0)+1;m=x['message']
  if 'header' in m:
   hstamp=m['header']['stamp'];stamps.setdefault(topic,[]).append(hstamp['sec']+hstamp['nanosec']*1e-9);frames.add(m['header']['frame_id'])
  if topic=='/tf':frames.update(v['child_frame_id'] for v in m['transforms'])
  if topic=='/plan':observed_plans.append(m)
  if topic=='/scan':
   scan_total+=len(m['ranges']);finite=[v for v in m['ranges'] if v is not None];scan_valid+=len(finite)
   if finite:scan_min=min(scan_min,min(finite));scan_max=max(scan_max,max(finite))
rates={k:(len(v)-1)/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>1 and v[-1]>v[0]}
checks={'expectedNav2Outcome':f['status']==s['expectedOutcome'],'strictHarnessValid':h['valid'],'receivedCommandsAndMotion':f['returnedCommands']>0 and (s['expectedOutcome']=='aborted' or f['displacementMeters']>7),'receivedPlanOrDocumentedPlannerFailure':bool(paths) or (s['expectedOutcome']=='aborted' and 'failed' in (r/'controller.log').read_text().lower()),'occupiedCostmap':f['maximumOccupiedCostmapCells']>0,'receivedScan':topic_counts.get('/scan',0)>0,'correctFrames':{'odom','base_link','base_scan'}.issubset(frames),'scanCadence':4.5<=rates.get('/scan',0)<=5.5,'imuCadence':45<=rates.get('/campus/imu',0)<=55,'receivedWheelOdometry':topic_counts.get('/campus/wheel_odom',0)>0,'finiteRangesWithinProfile':.119<=scan_min<=scan_max<=3.501,'noRobotFall':min(x['position']['y'] for x in t)>-.1,'stableUncommandedStartup':max(math.hypot(x['position']['x']-t[0]['position']['x'],x['position']['z']-t[0]['position']['z']) for x in t if x['simulationTime']<=next((v['simulationTime'] for v in t if v['commandLinear'] or v['commandAngular']),t[-1]['simulationTime']))<.05,'surfaceMeasured':any(x['leftSurface'] not in ['unknown','legacy'] for x in t),'nonpenetratingNominal':t[-1]['collisionCount']==0 if s['expectedOutcome']=='succeeded' and s['id'] not in ['dock_threshold','dock_ramp'] else True}
if a.require_recovery:checks['observedRecovery']=f['maximumRecoveryCount']>0
no_path_evidence=None
if a.require_no_path:
 topology_run=subprocess.run([sys.executable,str(pathlib.Path(__file__).with_name('check_topology.py')),'--manifest',str(r/'manifest.json'),'--scenario',s['id']],capture_output=True,text=True)
 topology=json.loads(topology_run.stdout)
 no_path_evidence=planner_failure_evidence((r/'controller.log').read_text(),f['status'],topology['results'][0])
 checks.update(no_path_evidence['checks'])
if a.require_detour:
 dx=s['goal'][0]-s['start'][0];dz=s['goal'][1]-s['start'][1];length=math.hypot(dx,dz);deviations=[abs(dx*(v['position']['z']-s['start'][1])-dz*(v['position']['x']-s['start'][0]))/length for v in t];checks['measuredAlternateRoute']=max(deviations)>.6;checks['timedBlockageActivated']=any(x['type']=='obstacle-activated' for x in events);checks['changedPlansAfterStart']=len(ids)>1
def intersects(plan, obstacle):
 cx,_,cz=obstacle['center'];sx,_,sz=obstacle['size']
 return any(abs(-p['pose']['position']['y']-cx)<sx/2+.24 and abs(p['pose']['position']['x']-cz)<sz/2+.24 for p in plan['poses'])
detour_evidence=[]
if a.require_detour:
 for obstacle in s['obstacles']:
  event=next((x for x in events if x['type']=='obstacle-activated' and x['id']==obstacle['id']),None)
  if event is None:continue
  epoch=event.get('rosTime')
  if epoch is None:continue
  before=[];after=[]
  for plan in observed_plans:
   stamp=plan['header']['stamp'];acquired=stamp['sec']+stamp['nanosec']*1e-9
   (before if acquired<epoch else after).append(plan)
  evidence={'obstacle':obstacle['id'],'activationRosSeconds':epoch,'initialRouteIntersectedNewObstacle':any(intersects(p,obstacle) for p in before),'laterRouteAvoidedObstacle':any(not intersects(p,obstacle) for p in after),'beforePlans':len(before),'afterPlans':len(after)}
  detour_evidence.append(evidence)
 checks['routeReplannedAroundActivatedCollider']=any(e['initialRouteIntersectedNewObstacle'] and e['laterRouteAvoidedObstacle'] for e in detour_evidence)
encounters=[]
if a.require_dynamic:
 for obstacle in s['obstacles']:
  if not any(obstacle['velocity']):continue
  event=next((e for e in events if e['type']=='obstacle-activated' and e['id']==obstacle['id']),None)
  if event is None:continue
  distances=[]
  for row in t:
   elapsed=row['simulationTime']-event['simulationTime']
   if 0<=elapsed<=obstacle['remove']-obstacle['activate']:
    cx=obstacle['center'][0]+obstacle['velocity'][0]*elapsed;cz=obstacle['center'][2]+obstacle['velocity'][2]*elapsed
    distances.append((math.hypot(row['position']['x']-cx,row['position']['z']-cz),row['simulationTime'],row['position'],[cx,cz]))
  closest=min(distances,key=lambda x:x[0]) if distances else None
  encounters.append({'obstacle':obstacle['id'],'minimumCenterDistanceMeters':closest[0] if closest else None,'closestSimulationTime':closest[1] if closest else None,'measuredRobotPosition':closest[2] if closest else None,'expectedCartCenterXZ':closest[3] if closest else None,'provenance':'Robot telemetry and configured fixed-step kinematic trajectory; cart pose inferred, not independently sampled.'})
 checks['dynamicObstacleEncounterWithinTwoMeters']=any(e['minimumCenterDistanceMeters'] is not None and e['minimumCenterDistanceMeters']<2 for e in encounters)
worker=json.loads((r/'worker-0/result.json').read_text());physics=next(x for x in worker['markers'] if x['name']=='Physics.Simulate')
report={'schema':'crane-campus-runtime-qa-v1','state':'NAVIGATION_PASS' if all(checks.values()) else 'PARTIAL','checks':checks,'scenario':s['id'],'seed':s['seed'],'nav2Status':f['status'],'displacementMeters':f['displacementMeters'],'pathMetrics':motion,'sampledExecutedPathMeters':sum(math.hypot(v['position']['x']-u['position']['x'],v['position']['z']-u['position']['z']) for u,v in zip(t,t[1:])),'timedDetourEvidence':detour_evidence,'distinctPlanCount':len(ids),'maximumRecoveryFeedback':f['maximumRecoveryCount'],'observedRecoveryInvocationStarts':f.get('behaviorTreeCapture',{}).get('observedRecoveryInvocationStartCount'),'exactRecoveryCountEstablished':f.get('behaviorTreeCapture',{}).get('completeness',{}).get('exactRecoveryCountEligible',False),'topicCounts':topic_counts,'acquisitionRatesHz':rates,'scanFiniteFraction':scan_valid/max(1,scan_total),'frames':sorted(frames),'collisionCount':t[-1]['collisionCount'],'wheelSlipProxyMean':statistics.mean(x['slipProxy'] for x in t),'minimumRadialClearanceProxyMeters':min(x['minimumClearance'] for x in t),'physicsSimulateTotalMilliseconds':physics['totalMilliseconds'],'physicsMillisecondsPerFixedStepEstimate':physics['totalMilliseconds']/max(1,worker['simulatedSeconds']/t[0]['fixedDeltaTime']),'gpuFrameMilliseconds':worker['meanGpuFrameMilliseconds'],'realTimeFactor':worker['realTimeFactor'],'allocatedBytes':worker['totalAllocatedMemoryBytes'],'boundaries':['Observed plan changes are not exact planner-invocation counts.','Recovery feedback and subscriber-observed BT invocation starts do not exclude message loss.','Wheel slip and radial clearance are explicit proxies; no physical calibration or exact geometric clearance claim.']}
report['noPathEvidence']=no_path_evidence
report['dynamicEncounters']=encounters
(r/'campus-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'state':report['state'],'checks':checks},indent=2));raise SystemExit(0 if all(checks.values()) else 1)
