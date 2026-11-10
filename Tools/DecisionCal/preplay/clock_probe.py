import sys, json, glob, re
sys.path.insert(0,'/home/lunarz/worktrees/decisioncal')
import numpy as np
from plantcard.marine import mcap_lite as M
D='/home/lunarz/astro_dock/bags/field_trials_20261018/'
out=[]
for jf in sorted(glob.glob(D+'agent_*.json')):
    if '.' in jf.split('/')[-1][:-5]: continue
    d=json.load(open(jf))
    if d.get('status') not in ('MEASURED','INCONCLUSIVE'): continue
    bag=jf[:-5]+'.files/bag/bag_0.mcap'
    b=M.read(bag,['/imu/raw','/thruster_pwm_node/test_pulses'])
    wz_b=np.array([M.imu(x)['w'][2] for _,x in b['/imu/raw']]); st_b=np.array([M.imu(x)['stamp'] for _,x in b['/imu/raw']])
    lt_b=np.array([t for t,_ in b['/imu/raw']])
    imuj=d['histories']['IMU']; tj=np.array([a for a,_ in imuj]); wj=np.array([r['wz'] for _,r in imuj])
    # match exact wz values
    offs=[]
    for i in range(0,len(wz_b),5):
        k=np.flatnonzero(np.abs(wj-wz_b[i])<1e-9)
        if len(k)==1: offs.append(st_b[i]-tj[k[0]])
    offs=np.array(offs)
    pw=[(t,M.uint16_array(x)) for t,x in b['/thruster_pwm_node/test_pulses']]
    on=[t for t,v in pw if any(a!=1500 for a in v)][0]
    ev=dict((e[1].split(' ')[0],e[0]) for e in d['telemetry']['events'])
    ps=ev.get('pulse_start')
    o=np.median(offs) if len(offs) else np.nan
    out.append((jf.split('/')[-1][6:-5],len(offs),o,offs.std() if len(offs)>1 else np.nan,(on-o-ps) if ps else np.nan))
    print('%-38s nmatch=%3d offset=%.4f sd=%.4f  bag_on - json_pulse_start (offset-corrected) = %.3f'%out[-1])
