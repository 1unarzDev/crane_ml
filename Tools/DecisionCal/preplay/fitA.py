import sys, time, json
sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal'); 
import _paths; _paths.setup()
import numpy as np
from plantcard.marine import fieldtrials as F
from plantcard.family import Evidence
import twin
from plantcard.marine.fossen3 import Fossen3
f3=twin.Regression.__init__
fam=Fossen3(twin.CONFIG)
Bf=[a for a in dir(fam) if a.upper()=='B' or 'geom' in a.lower() or 'mount' in a.lower()]
print('Fossen3 geometry attrs',Bf)
for a in Bf:
    v=getattr(fam,a); print(a, np.round(np.asarray(v),4) if not callable(v) else v)
print('canonical order check: F default',('front_left','front_right','rear_right','rear_left'))
import copy
def snap(ev,dt):
    segs=[]
    for s in ev.segments:
        s2=copy.copy(s); s2.meta=dict(s.meta); s2.t=np.arange(len(s.t))*dt; segs.append(s2)
    return Evidence(segs,ev.provenance)
ev=F.load()
try:
    twin.Regression(ev)
except ValueError as e: print('UNMODIFIED loader+twin ->',e)
ev=snap(ev,0.1)
t0=time.time()
reg=twin.Regression(ev)
print('regression built %.0fs'%(time.time()-t0),'smoothed(gps)=',reg.smoothed,'rows',reg.n_rows,'grid pts',len(reg.points))
for arm in ('rigid','rigid+delay','rigid+lag','joint'):
    p,rep=twin.fit(reg,arm,512,1)
    print(arm,json.dumps({k:rep[k] for k in ('map','grid_entropy','block_map','rows','frac_clipped')}))
    print('   particle mean:',{k:round(float(np.mean(v)),3) for k,v in p.items() if k in twin.GRID})
