"""How much apparent 10% delay does the fieldtrials gyro pipeline (interp 20Hz->? box5 'same' on raw samples, interp to 10 Hz grid) add on a known delayed response?
Truth: r(t)=A*(1-exp(-(t-d)/tau)) for t>d (step of command at 0), pulse long enough. Apparent delay from the true signal = d + (-tau ln(0.9)) for 10% of peak (peak = A)."""
import sys; sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal/preplay')
import numpy as np
from raw import load_all
from sw import prep, step, stopwatch
R=load_all()
r0=R['agent_battery_03_01_yaw_pos']
ti=r0['ti']  # real jittered IMU times
t_off=1.5
print('median IMU dt %.4f; box5 spans %.3f s'%(np.median(np.diff(ti)),5*np.median(np.diff(ti))))
def truth(t,d,tau,A=0.4,tend=1.5+1.0):
    x=np.where(t>d,A*(1-np.exp(-(t-d)/max(tau,1e-9))) if tau>0 else A,0.0)
    return x
rows=[]
for d in (0.0,0.1,0.2,0.3):
  for tau in (0.0,0.15,0.3):
    for noise in (0.0,):
        # ideal continuous truth: 10% crossing
        grid=np.arange(-10,t_off+3,0.001)
        y=truth(grid,d,tau)
        ideal=step(grid,y,0.0,t_off)['delay'] if True else None
        # raw 20Hz samples
        yi=truth(ti,d,tau); m=(ti>-10)&(ti<t_off+3)
        raw=stopwatch(ti[m],yi[m],t_off,mc=0,smooth=False)['delay'] if False else None
        # raw20 no smoothing: manual
        tt,yy=ti[m],yi[m]
        def dl(t,y,sm):
            ys=np.convolve(np.pad(y,1,mode='edge'),np.ones(3)/3,'valid') if sm else y
            lvl=ys[(t>=0)&(t<=t_off+1.5)].max(); return step(t,ys,1e-6,t_off)['delay'] if lvl>0 else np.nan
        d_raw=dl(tt,yy,False); d_raw3=dl(tt,yy,True)
        # fieldtrials pipeline: np.convolve(wz, ones(5)/5,'same') on raw samples then interp to 0.1 s grid; study _smooth3 on top
        k=np.convolve(yi,np.ones(5)/5,'same')
        g=np.arange(-10,t_off+3,0.1)
        yg=np.interp(g,ti,k)
        d_grid=dl(g,yg,True)
        rows.append((d,tau,ideal,d_raw,d_raw3,d_grid))
print('d tau | ideal(10%) raw20 raw20+sm3 | fieldtrials box5+10Hz grid+sm3 | pipeline bias vs ideal')
for d,tau,i,a,b,c in rows: print('%.2f %.2f | %.3f %.3f %.3f | %.3f | %+.3f'%(d,tau,i,a,b,c,c-i))
# Also noise-free step through gridded SG velocity (surge): positions 5 Hz from a scalar surge plant
from plantcard.marine.fieldtrials import sg_velocity
print('\nGPS->SG(+-1s) surge speed through the loader: apparent 10%-of-peak delay for a synthetic surge response')
tg=r0['tg']; 
def surge_u(t,d,Tau,umax=1.0,t_off=3.0):
    # speed: after delay d, first-order rise with time const Tau toward umax until t_off+d, then decay with Tau
    u=np.zeros_like(t); 
    # simple ODE integration
    dt=0.001; tt=np.arange(-12,15,dt); uu=np.zeros_like(tt)
    for i in range(1,len(tt)):
        cmd=umax if (tt[i]-d>=0 and tt[i]-d<t_off) else 0.0
        uu[i]=uu[i-1]+(cmd-uu[i-1])/Tau*dt
    return tt,uu
for d in (0.0,0.2,0.4):
  for Tau in (1.0,2.0,3.0):
    tt,uu=surge_u(None,d,Tau)
    pos=np.cumsum(uu)*0.001
    tgg=np.arange(-12,14,0.2)+np.random.default_rng(0).normal(0,0.003,130)
    P=np.interp(tgg,tt,pos)[:,None]
    grid=np.arange(-10,12,0.1)
    v=sg_velocity(tgg,P,grid)[:,0]
    ok=np.isfinite(v)
    ideal=step(tt,uu,1e-6,3.0)['delay']
    sgd=step(grid[ok],v[ok],1e-6,3.0)['delay']
    tm=0.5*(tgg[1:]+tgg[:-1]); vd=np.diff(P[:,0])/np.diff(tgg)
    dd=step(tm,vd,1e-6,3.0)['delay']
    print('d=%.1f Tau=%.1f : ideal %.3f | loader SG %.3f (%+.3f) | 5Hz central diff %.3f (%+.3f)'%(d,Tau,ideal,sgd,sgd-ideal,dd,dd-ideal))
