#!/usr/bin/env python3
"""Plot command/body response from streamed calibration evidence."""
import argparse,json,pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);a=p.parse_args()
r=[json.loads(x) for x in (a.run/'telemetry.jsonl').read_text().splitlines()];origin=next(x['simulationTime'] for x in r if x['commandLinear'] or x['commandAngular']);t=[x['simulationTime']-origin for x in r];yaw=[0.]
for u,v in zip(r,r[1:]):yaw.append(yaw[-1]+(v['yaw']-u['yaw']+180)%360-180)
fig,axes=plt.subplots(3,1,figsize=(9,7),sharex=True,constrained_layout=True)
axes[0].plot(t,[x['commandLinear'] for x in r],label='command');axes[0].plot(t,[(x['velocity']['x']**2+x['velocity']['z']**2)**.5 for x in r],label='body speed');axes[0].set_ylabel('Linear (m/s)')
axes[1].plot(t,[x['commandAngular'] for x in r],label='command');axes[1].plot(t,[x['angularVelocity']['y'] for x in r],label='body yaw rate');axes[1].set_ylabel('Angular (rad/s)')
axes[2].plot(t,yaw,label='unwrapped body yaw');axes[2].set_ylabel('Yaw (degrees)');axes[2].set_xlabel('Seconds from first nonzero command')
for ax in axes:ax.grid(alpha=.2);ax.legend(loc='upper right')
fig.suptitle('TurtleBot simulation response • '+a.run.name+'\nEngineering model; no physical fit claimed',fontsize=12)
fig.savefig(a.run/'calibration-response.png',dpi=160)
