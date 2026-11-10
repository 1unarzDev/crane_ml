import sys, os
sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal')
import _paths; _paths.setup()
import numpy as np, collections
from plantcard.marine import fieldtrials as F
import twin
from plantcard.marine.fossen3 import Fossen3
ev=F.load()
print(len(ev.segments), ev.provenance['skipped'][:3], len(ev.provenance['skipped']))
print('B_GEOM\n',twin.B_GEOM)
print('Fossen3 B', getattr(Fossen3,'B',None))
rows=[]
for s in ev.segments:
    q=s.u.max(0)+s.u.min(0)
    nz=np.abs(s.u).max(0)
    print(f"{s.name:42s} {s.session:12s} {s.labels[0]:9s} {str(s.labels[1]):6s} {s.labels[2]:11s} pulse={s.meta['pulse_s']:.2f} T={s.t[-1]:.0f} u={np.round(s.u[np.argmax(np.abs(s.u).sum(1))],2)} tev={s.t_event:.1f} cur_sd={s.meta['current_sd']:.3f}")
