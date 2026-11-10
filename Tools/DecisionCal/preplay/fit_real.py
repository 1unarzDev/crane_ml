"""PREPLAY: the study's twin.Regression/fit on the real 10-18 evidence, two adapters, profiles, leave-one-block-out."""
import sys, time, json, copy
sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal'); sys.path.insert(0,'/home/lunarz/worktrees/decisioncal/Tools/DecisionCal/preplay')
import _paths; _paths.setup()
import numpy as np
from plantcard.marine import fieldtrials as F
from plantcard.family import Evidence
import twin
from twin import FREE, GRID, BLOCKS, NOMINAL, batched_posterior, nig_prior, BOUNDS
from raw import load_all
RAW = load_all()

def snap(ev, dt):
    segs = []
    for s in ev.segments:
        s2 = copy.copy(s); s2.meta = dict(s.meta); s2.t = np.arange(len(s.t)) * dt; segs.append(s2)
    return Evidence(segs, ev.provenance)

def evidence(kind):
    if kind == 'A':                       # shipped loader (0.1 s grid), only t snapped to exact multiples of 0.1
        return snap(F.load(), 0.1)
    F.DT = 0.02                           # runtime override of the module constant, file untouched
    try:
        ev = F.load()
    finally:
        F.DT = 0.1
    ev = snap(ev, 0.02)
    for s in ev.segments:
        ti = RAW[s.name]['ti'] + s.t_event
        s.meta['gyro_t'] = ti[(ti >= 0) & (ti <= s.t[-1])]
    return ev

def profile(reg, arm):
    """log evidence over the free grid (mirrors twin.fit's loop) -> (logp, points, posts)"""
    grid_free, nig_free, rows = FREE[arm]
    fixed = {k: NOMINAL[k] for k in GRID if k not in grid_free}
    gidx = reg.index(fixed)
    passes = twin.REF_PASSES if 'm' in nig_free else 1
    for it in range(passes):
        logp = np.zeros(len(gidx)); posts = {}
        for b, names in BLOCKS.items():
            Phi, Y, steady = reg.blocks[b]
            Y = Y[gidx]; pinned = {}
            if 'm' not in nig_free: pinned[0] = NOMINAL[names[0]]
            if 'd' not in nig_free: pinned[1], pinned[2] = NOMINAL[names[1]], NOMINAL[names[2]]
            post = batched_posterior(nig_prior(names), Phi, Y, pinned)
            posts[b] = post; logp += post['logev']
        if it + 1 < passes:
            g = int(np.argmax(logp)); ref = dict(reg.ref)
            for b, names in BLOCKS.items():
                post = posts[b]
                if 0 in post['free']: ref[names[0]] = float(max(post['mu'][g][post['free'].index(0)], BOUNDS[names[0]]))
            reg = reg.with_ref(ref)
    return logp, [reg.points[i] for i in gidx], posts, {b: posts[b]['logev'] for b in BLOCKS}

def marg(logp, pts, axis):
    names = list(GRID); j = names.index(axis); out = {}
    for v in GRID[axis]:
        m = np.array([abs(p[j] - v) < 1e-12 for p in pts])
        if m.any(): out[v] = float(np.logaddexp.reduce(logp[m]))
    mx = max(out.values()); return {k: round(v - mx, 2) for k, v in out.items()}

def summary(reg, arm, seed=1):
    p, rep = twin.fit(reg, arm, 512, seed)
    return rep

if __name__ == '__main__':
    kind = sys.argv[1]
    t0 = time.time(); ev = evidence(kind)
    print('== adapter', kind, 'segments', len(ev.segments), 'dt', np.median(np.diff(ev.segments[0].t)), 'gyro_t' in ev.segments[0].meta)
    reg = twin.Regression(ev); print('regression %.0fs' % (time.time() - t0), reg.n_rows)
    out = {'kind': kind, 'arms': {}}
    for arm in ('rigid', 'rigid+delay', 'rigid+lag', 'joint'):
        rep = summary(reg, arm)
        logp, pts, posts, byb = profile(reg, arm)
        d = {'map': rep['map'], 'block_map': rep['block_map'], 'frac_clipped': rep['frac_clipped'], 'rows': rep['rows']}
        free = FREE[arm][0]
        d['marg_logev'] = {ax: marg(logp, pts, ax) for ax in free if ax in ('delay_s', 'tau_s', 'rev_hold_s', 'rev_ratio', 'deadband_q')}
        d['max_logev'] = float(logp.max()); d['logev_by_block_at_map'] = {b: float(v[int(np.argmax(logp))]) for b, v in byb.items()}
        if arm == 'rigid+delay':
            d['delay_profile_by_block'] = {}
            for b in BLOCKS:
                lb = byb[b]; names = list(GRID); j = names.index('delay_s'); o = {}
                for v in GRID['delay_s']:
                    m = np.array([abs(pp[j] - v) < 1e-12 for pp in pts]); o[v] = float(np.logaddexp.reduce(lb[m]))
                mx = max(o.values()); d['delay_profile_by_block'][b] = {k: round(x - mx, 1) for k, x in o.items()}
        out['arms'][arm] = d
        print(arm, json.dumps(d))
    # LOBO
    sess = ev.sessions(); out['lobo'] = {}
    for held in sess:
        tr, te = ev.split([held])
        try:
            rg = twin.Regression(tr)
        except ValueError as e:
            out['lobo'][held] = str(e); print(held, e); continue
        row = {}
        for arm in ('rigid+delay', 'rigid+lag', 'joint'):
            rep = summary(rg, arm); row[arm] = {k: rep['map'][k] for k in ('delay_s', 'tau_s', 'rev_ratio', 'deadband_q')}
            row[arm]['m'] = [round(rep['block_map'][b][0], 1) for b in ('surge', 'sway', 'yaw')]
            row[arm]['n_train'] = len(tr.segments)
        out['lobo'][held] = row
        print('LOBO -%-12s' % held, json.dumps(row))
    json.dump(out, open(f'fit_real_{kind}.json', 'w'), indent=1, default=float)
