#!/usr/bin/env python3
"""Render recorded geometry, an observed plan and measured robot motion."""
import argparse,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);args=parser.parse_args()
    run=args.run;m=json.loads((run/'manifest.json').read_text());s=json.loads((run/'scenario.json').read_text())
    rows=[json.loads(line) for line in (run/'telemetry.jsonl').read_text().splitlines()]
    fixture=json.loads((run/'fixture-summary.json').read_text()) if (run/'fixture-summary.json').exists() else {}
    fig,axis=plt.subplots(figsize=(7,8),constrained_layout=True)
    for box in m['boxes']+s['obstacles']:
        if box['kind']=='floor' or box['center'][1]-box['size'][1]/2>.4:continue
        angle=math.radians(box.get('yaw',0));sx,_,sz=box['size'];cx,_,cz=box['center']
        corners=[(cx+math.cos(angle)*x+math.sin(angle)*z,cz-math.sin(angle)*x+math.cos(angle)*z)
                 for x,z in [(-sx/2,-sz/2),(-sx/2,sz/2),(sx/2,sz/2),(sx/2,-sz/2)]]
        axis.add_patch(Polygon(corners,facecolor='#d0d4d8',edgecolor='#6b7280',linewidth=.5))
    plans=fixture.get('planHistory',[])
    if plans:
        points=plans[0]['poses'];axis.plot([-p['y'] for p in points],[p['x'] for p in points],'--',color='#db8b28',linewidth=1.5,label='First observed plan')
    x=[r['position']['x'] for r in rows];z=[r['position']['z'] for r in rows]
    axis.plot(x,z,color='#167a94',linewidth=2,label='Measured body trajectory')
    axis.scatter(*s['start'],marker='o',s=50,color='#207a49',label='Start',zorder=4)
    axis.scatter(*s['goal'],marker='*',s=100,color='#963d50',label='Goal',zorder=4)
    axis.set_xlim(min(x+[s['goal'][0]])-3,max(x+[s['goal'][0]])+3)
    axis.set_ylim(min(z+[s['goal'][1]])-1.5,max(z+[s['goal'][1]])+1.5)
    axis.set_aspect('equal');axis.set_xlabel('Unity map X (m)');axis.set_ylabel('Unity map Z (m)');axis.grid(alpha=.15)
    axis.legend(loc='best',fontsize=8);axis.set_title(f"{s['routeName']} · seed {s['seed']}\nNav2: {fixture.get('status','capture in progress')}",fontsize=11)
    fig.savefig(run/'navigation-evidence.png',dpi=160)
    fig.savefig(run/'navigation-evidence.svg')


if __name__=='__main__':main()
