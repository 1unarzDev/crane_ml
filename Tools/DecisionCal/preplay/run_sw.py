import sys; sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal/preplay'); sys.path.insert(0,'/home/lunarz/worktrees/decisioncal')
import numpy as np, json
from raw import load_all
from sw import *
from plantcard.marine import fieldtrials as F
R=load_all(); ev=F.load(); SEG={s.name:s for s in ev.segments}
def kind(r):
    a,b,_=r['labels'][:3]; 
    if a=='thruster': return 'single'
    n=r['stem']
    if 'pairs_' in n and 'surge' in n: return 'pair_surge'
    if 'pairs_' in n and 'sway' in n: return 'pair_sway'
    return b
rows=[]
for n,r in R.items():
    k=kind(r); t_off=r['t_off']-r['t_on']
    out=dict(name=n[6:],session=r['session'],kind=k,pulse=t_off)
    # yaw: raw 20 Hz gyro
    y=sw=stopwatch(r['ti'],r['wz'],t_off,seed=1)
    out['yaw_raw']=dict(delay=y['delay'],se=y['se'],peak=y['peak'],sd=y['sd_noise'],tpk_off=y['t_peak']-t_off,sgn=y['sgn'],mcf=y['mc_fail'])
    # yaw: as loaded 10 Hz grid (box5 smoothed + interp), onset = t_event
    s=SEG[n]; tt=s.t-s.t_event
    y2=stopwatch(tt,s.x[:,5],t_off,seed=2)
    out['yaw_grid']=dict(delay=y2['delay'],se=y2['se'],peak=y2['peak'],sd=y2['sd_noise'],tpk_off=y2['t_peak']-t_off,sgn=y2['sgn'])
    # no-smooth raw
    y3=stopwatch(r['ti'],r['wz'],t_off,seed=3,smooth=False)
    out['yaw_raw_nosm']=dict(delay=y3['delay'],se=y3['se'])
    # accel ax body
    y4=stopwatch(r['ti'],r['a'][:,0],t_off,seed=4)
    out['ax_raw']=dict(delay=y4['delay'],se=y4['se'],peak=y4['peak'],sd=y4['sd_noise'],sgn=y4['sgn'])
    # surge speed from GPS
    psi0=float(np.interp(0.0,r['to'],r['yaw']))
    e=np.array([np.cos(psi0),np.sin(psi0)]); ee=np.array([-np.sin(psi0),np.cos(psi0)])
    tg=r['tg']; P=r['P']
    sx=P@e; sy=P@ee
    # raw central difference (5 Hz), time = midpoint
    tm=0.5*(tg[1:]+tg[:-1]); vx=np.diff(sx)/np.diff(tg); vy=np.diff(sy)/np.diff(tg)
    ysg=stopwatch(tm,vx,t_off,seed=5,smooth=False,quant=0.2)
    out['u_gps_diff']=dict(delay=ysg['delay'],se=ysg['se'],peak=ysg['peak'],sd=ysg['sd_noise'],sgn=ysg['sgn'])
    # smoothed 3-pt of diff
    ysg2=stopwatch(tm,vx,t_off,seed=6,smooth=True,quant=0.2)
    out['u_gps_diff3']=dict(delay=ysg2['delay'],se=ysg2['se'],peak=ysg2['peak'],sgn=ysg2['sgn'])
    # as loaded u (SG +-1s, 10 Hz grid)
    y5=stopwatch(tt,s.x[:,3],t_off,seed=7)
    out['u_grid']=dict(delay=y5['delay'],se=y5['se'],peak=y5['peak'],sd=y5['sd_noise'],sgn=y5['sgn'])
    y6=stopwatch(tt,s.x[:,4],t_off,seed=8)
    out['v_grid']=dict(delay=y6['delay'],se=y6['se'],peak=y6['peak'],sgn=y6['sgn'])
    out['psi_resp_deg']=float(np.degrees(np.interp(t_off+8,r['to'],r['yaw'])-psi0))
    out['wrench_sign']=None
    rows.append(out)
json.dump(rows,open('sw_rows.json','w'),indent=1,default=float)
def f(d): return '%6.3f±%.3f'%(d['delay'],d['se']) if np.isfinite(d['delay']) else '   --      '
print('%-28s %-9s %-10s | %-12s %-12s %-12s sgn pk(rad/s) sd | ax %-12s | u_gps_diff %-12s u_grid %-12s'%('trial','block','kind','yaw raw20','yaw grid10','yaw nosm','', '', ''))
for o in rows:
    print('%-28s %-11s %-10s | %s %s %s %+d %.3f %.4f | ax %s pk %.2f | ud %s pk%+.2f sd%.2f | ug %s pk %.2f'%(o['name'][:28],o['session'],o['kind'],f(o['yaw_raw']),f(o['yaw_grid']),f(o['yaw_raw_nosm']),o['yaw_raw']['sgn'],o['yaw_raw']['peak'],o['yaw_raw']['sd'],f(o['ax_raw']),o['ax_raw']['peak'],f(o['u_gps_diff']),o['u_gps_diff']['peak']*o['u_gps_diff']['sgn'],o['u_gps_diff']['sd'],f(o['u_grid']),o['u_grid']['peak']))
