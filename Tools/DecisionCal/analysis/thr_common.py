"""Shared loader for the threshold analysis (numpy only, read-only on stored runs)."""
import json, os
import numpy as np

CACHE = os.path.expanduser('~/.cache/decisioncal')
CAP = 60.0
# library tables (copied from library_v2.py/library_v3.py parameters; v3 regenerated with same RNG draws)
V2 = [(1.5,1.0,1.5,0.50,2.0,'sqrt@0.6',0.0,1),(1.5,1.0,1.5,0.50,1.0,'sqrt@0.6',0.1,1),(1.0,1.0,1.3,0.50,0.8,'sqrt@0.3',0.1,1),
      (1.0,1.0,1.2,0.50,0.6,'sqrt',0.1,1),(1.5,1.5,1.2,0.25,0.6,'const',0.2,1),(1.0,1.0,1.0,0.25,0.4,'two_stage',0.1,1),
      (1.0,0.5,1.0,0.25,0.4,'sqrt',0.2,1),(1.0,0.5,1.3,0.50,0.6,'sqrt@0.3',0.1,1),(1.0,1.0,1.0,0.25,0.4,'sqrt',0.1,0),
      (0.5,0.5,0.8,0.12,0.2,'sqrt@0.08',0.2,1),(1.0,1.0,0.8,0.12,0.2,'two_stage',0.0,1),(1.5,0.5,1.1,0.25,0.3,'sqrt',0.2,1),
      (1.5,1.0,0.8,0.12,0.2,'two_stage',0.1,0),(1.0,0.5,1.0,0.12,0.4,'two_stage',0.2,1),(1.5,0.5,1.2,0.25,0.4,'sqrt',0.2,1),
      (0.5,1.0,1.2,0.50,0.4,'const',0.2,1)]
FIELDS = ('kp','ki','cruise','turn','brake','dock','latcomp','ff')

def v3_table():
    rng = np.random.default_rng(31); out = []
    DOCKS = ('const','sqrt','two_stage','sqrt@0.3','sqrt@0.6')
    for i in range(16):
        kp=float(rng.choice([0.5,1.0,1.5])); ki=float(rng.choice([0.5,1.0,1.5]))
        cr=round(float(rng.uniform(0.8,1.5)),2); tu=float(rng.choice([0.12,0.25,0.5]))
        br=round(float(np.exp(rng.uniform(np.log(0.2),np.log(2.0)))),2); dk=str(rng.choice(DOCKS))
        lc=float(rng.choice([0.0,0.1,0.2])); ff=bool(rng.uniform()<0.875)
        out.append((kp,ki,cr,tu,br,dk,lc,int(ff)))
    return out

TABLES = {'v2': V2, 'v3': v3_table()}

def load(run):
    p = os.path.join(CACHE, run, 'plants.jsonl')
    rows = [json.loads(l) for l in open(p)]
    for r in rows: r['id'] = run.split('-')[0] + ':' + r['id']        # ids repeat across runs (timing-000 ...)
    return rows

def boot_ci(x, fn=np.mean, n=4000, seed=0, q=(5, 95)):
    x = np.asarray(x, float)
    if len(x) == 0: return (np.nan, np.nan)
    idx = np.random.default_rng(seed).integers(0, len(x), (n, len(x)))
    v = np.array([fn(x[i]) for i in idx]) if fn is not np.mean else x[idx].mean(1)
    return tuple(np.percentile(v, q))

def records(rows, libs=('v2','v3')):
    """One record per (boat, lib, arm). Delay = twin map delay_s (carried); tau/rev_hold also kept."""
    recs = []
    for r in rows:
        s = r['spec']
        L1 = s['dead_time'] + s['odom_latency']
        for lib in libs:
            Ld = r['libs'][lib]
            Jh = np.array(Ld['J_h'], float)
            for arm, sc in Ld['scores']['arms'].items():
                if 'map' not in r['fits'][arm]: continue
                f = r['fits'][arm]['map']
                sel = sc['sel']
                recs.append(dict(
                    id=r['id'], stratum=r['stratum'], lib=lib, arm=arm, sel=sel, oracle=Ld['scores']['oracle'],
                    d=f['delay_s'], tau_fit=f['tau_s'], rh_fit=f['rev_hold_s'],
                    kind=('fixed' if '@d' in arm else 'fitted'),
                    L1=L1, dead=s['dead_time'], odom=s['odom_latency'], tau_up=s['tau_up'], tau_down=s['tau_down'],
                    rev_hold=s['rev_hold'], slew=s['slew'], gamma=s['gamma'], mass=s['mass'],
                    pos_sd=s['pos_sd'], vel_sd=s['vel_sd'], yaw_sd=s['yaw_sd'], gyro_sd=s['gyro_sd'],
                    kmean=float(np.mean(s['k_fwd'])), gust=s['gust_sigma'], wind=float(np.hypot(*s['wind'])),
                    cur=float(np.hypot(*s['current'])),
                    reg=sc['regret'], regraw=sc['regret_raw'], pred_J=sc['pred_J'], hid_J=sc['hidden_J'],
                    dock=Ld['hidden_docked'][sel], Jo=Ld['scores']['J_oracle'],
                    dock_or=Ld['hidden_docked'][Ld['scores']['oracle']]))
    return recs

def arr(recs, k):
    return np.array([r[k] for r in recs], float)
