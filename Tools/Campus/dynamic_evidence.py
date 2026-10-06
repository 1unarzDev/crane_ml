"""Measure a cart encounter from physics poses, rather than its commanded path."""
import bisect
import json
import math


def measured_encounter(obstacle_id, events, robot_rows, maximum_gap=.15):
    poses=[]
    for event in events:
        if event['type']=='obstacle-pose' and event['id']==obstacle_id:
            detail=json.loads(event['detail'])
            if detail['colliderEnabled']:
                poses.append((event['simulationTime'],detail['position']))
    times=[t for t,_ in poses]
    monotonic=all(a<b for a,b in zip(times,times[1:]))
    distances=[]
    if monotonic:
        for row in robot_rows:
            stamp=row['simulationTime'];index=bisect.bisect_left(times,stamp)
            if index<len(times) and abs(times[index]-stamp)<1e-8:
                position=poses[index][1]
            elif 0<index<len(times) and times[index]-times[index-1]<=maximum_gap:
                fraction=(stamp-times[index-1])/(times[index]-times[index-1])
                position={axis:poses[index-1][1][axis]+fraction*(poses[index][1][axis]-poses[index-1][1][axis]) for axis in 'xyz'}
            else:
                continue
            distances.append((math.hypot(row['position']['x']-position['x'],row['position']['z']-position['z']),stamp,row['position'],position))
    closest=min(distances,key=lambda item:item[0]) if distances else None
    moved=math.dist([poses[0][1][a] for a in 'xyz'],[poses[-1][1][a] for a in 'xyz']) if len(poses)>1 else 0
    return dict(obstacle=obstacle_id,actualPoseSamples=len(poses),matchedRobotSamples=len(distances),
                monotonicAcquisitionTimes=monotonic,measuredCartDisplacementMeters=moved,
                minimumCenterDistanceMeters=closest[0] if closest else None,
                closestSimulationTime=closest[1] if closest else None,
                measuredRobotPosition=closest[2] if closest else None,
                measuredCartPosition=closest[3] if closest else None,
                provenance='Actual Rigidbody pose and robot telemetry sampled before fixed-step integration; cart interpolation bounded to 150 ms, no extrapolation.')
