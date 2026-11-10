import sys; sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal/preplay')
import numpy as np, collections
from raw import load_all
R=load_all()
print(len(R))
for n,r in R.items():
    dti=np.diff(r['ti']); dtg=np.diff(r['tg']); dtp=np.diff(r['pwm_t']); dto=np.diff(r['to'])
    # trial-time: baseline length, post
    pre=-min(r['ti'][0],r['tg'][0]); post=min(r['ti'][-1],r['tg'][-1])-(r['t_off']-r['t_on'])
    print('%-34s imu %.1fHz(max gap %.2f) gps %.2fHz(max gap %.2f) odo %.1fHz pwm %.1fHz t_prev=%.3f pre=%.1f post=%.1f stamp-log gps %.3f imu %.3f ax_med=%.2f az_med=%.2f'%(
      n[6:],1/np.median(dti),dti.max(),1/np.median(dtg),dtg.max(),1/np.median(dto),1/np.median(dtp),r['t_prev']-r['t_on'],pre,post,
      np.mean(r['tg_stamp']-r['tg']),np.mean(r['ti_stamp']-r['ti']),np.median(r['a'][:,0]),np.median(r['a'][:,2])))
