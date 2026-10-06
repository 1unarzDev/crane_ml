"""Require measured elevation traversal, independently of a planar action result."""
import math


def ramp_traversal(manifest,scenario,rows,radius=.24):
    def inside(position,box,margin=0):
        angle=math.radians(box.get('yaw',0));dx=position[0]-box['center'][0];dz=position[1]-box['center'][2]
        x=math.cos(angle)*dx-math.sin(angle)*dz;z=math.sin(angle)*dx+math.cos(angle)*dz
        return abs(x)<=box['size'][0]/2-margin and abs(z)<=box['size'][2]/2-margin
    floors=[box for box in manifest['boxes'] if box['kind']=='floor' and not box.get('pitch',0)]
    goal_floors=[box for box in floors if inside(scenario['goal'],box)]
    start_floors=[box for box in floors if inside(scenario['start'],box)]
    if not goal_floors or not start_floors:
        return dict(checks={'raisedSurfacePriorAvailable':False},boundary='Cannot prove elevation traversal without supporting start/goal floor priors.')
    height=lambda box:box['center'][1]+box['size'][1]/2
    platform=max(goal_floors,key=height);start_height=max(map(height,start_floors))
    expected=height(platform)+rows[0]['position']['y']-start_height
    supported=[r for r in rows if inside((r['position']['x'],r['position']['z']),platform,radius) and abs(r['position']['y']-expected)<.04]
    final=rows[-1]['position']
    checks=dict(raisedSurfacePriorAvailable=height(platform)-start_height>.05,
                measuredRaisedPlatformTraversal=len(supported)>=3,
                robotFinishedOnRaisedGoalSurface=inside((final['x'],final['z']),platform,radius) and abs(final['y']-expected)<.04)
    return dict(checks=checks,platform=platform['id'],platformTopMeters=height(platform),expectedBodyHeightMeters=expected,
                maximumBodyHeightMeters=max(r['position']['y'] for r in rows),supportedPlatformSamples=len(supported),
                boundary='Body center inside the platform footprint with the chassis radius margin and height within 40 mm; contact impulses are separate evidence.')
