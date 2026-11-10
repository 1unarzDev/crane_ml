import sys; sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal/preplay')
import numpy as np
from raw import load_all
from sw import smooth3
R=load_all()
for n,r in R.items():
    lab=r['labels']; 
    if not (lab[0]=='thruster' or lab[1]=='yaw'): continue
    t=r['ti']; y=r['wz']; to=r['t_off']-r['t_on']
    b=(t>-10)&(t<-0.3); y0=np.median(y[b]); ys=smooth3(y-y0)
    sg=np.sign(np.median(ys[(t>0.6*to)&(t<to)])); ys*=sg
    L=ys[(t>=to-0.6)&(t<=to-0.1)].mean(); sd=y[b].std()
    # sustained (2 samples) fall below 0.9L after t_off
    out=np.nan
    idx=np.flatnonzero(t>=to)
    for i in idx[:-1]:
        if ys[i]<=0.9*L and ys[i+1]<=0.9*L:
            lo=ys[i-1]; f=(0.9*L-lo)/(ys[i]-lo) if ys[i]!=lo else 0; out=t[i-1]+f*(t[i]-t[i-1])-to; break
    # also 50% decay time
    out50=np.nan
    for i in idx[:-1]:
        if ys[i]<=0.5*L and ys[i+1]<=0.5*L: out50=t[i]-to; break
    # rise: time to 90% of L from onset
    ons=np.nan
    for i in np.flatnonzero(t>=0)[:-1]:
        if ys[i]>=0.9*L and ys[i+1]>=0.9*L: ons=t[i]; break
    print('%-26s L=%.3f sd=%.4f fall90-after-off=%.3f  fall50=%.3f  rise90-from-on=%.3f'%(n[6:],L,sd,out,out50,ons))
