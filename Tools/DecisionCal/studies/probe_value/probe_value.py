"""Probe value at equal calibration cost (EXPLORATORY; development seeds; label HSUR, numpy plant only).

Question (owner): which short maneuver or timing measurement most improves controller selection at equal
calibration cost? Passive data vs a fixed pulse or reversal vs a probe chosen for the docking task, with a
stratum where delay and lag are hard to distinguish. Full design, cost model and caveats: README.md here.

Two phases, run in this order by ``run`` (or separately):

  design   development boats (seed --design-seed, never the evaluation seed): preposterior analysis by
           simulation. Every candidate design in CANDIDATES is simulated on every development boat, fitted
           (twin.fit 'joint'), flown against both libraries and scored. Chosen from those numbers, before any
           evaluation boat exists: the task-directed fixed design (lowest mean capped regret), the two-stage
           adaptive policy (per stopwatch bin), the D-optimal design (lowest mean posterior entropy of the
           (delay, lag, hold) grid marginal), and the stopwatch pad. Writes design.json.
  eval     evaluation boats (seed --seed): every arm at every budget on the same boats with common random
           numbers (same segment realizations, same fit seed, same twin and hidden mission seeds).

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \\
        Tools/DecisionCal/studies/probe_value/probe_value.py run --out DIR [--n-per 20] [--n-dev 4] [--procs 4]
    ... design --out DIR [--n-dev 4]        ... eval --out DIR [--n-per 20]       ... summary --out DIR
    ... smoke --out DIR                       (1 process, 2 dev + 2 eval boats, reduced candidates, M = 4)

Resumable: dev_plants.jsonl and plants.jsonl keep finished boats (ids encode the per-stratum count).
"""

import argparse
import copy
import hashlib
import itertools
import json
import math
import os
import sys
import time
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
HERE = Path(__file__).resolve().parent
DC = HERE.parents[1]
sys.dont_write_bytecode = True
if str(DC) not in sys.path:
    sys.path.insert(0, str(DC))

import numpy as np  # noqa: E402

import _paths  # noqa: E402
import baseline_selector as bsel  # noqa: E402
import hidden  # noqa: E402
import mission  # noqa: E402
import probes  # noqa: E402
import record  # noqa: E402
import score  # noqa: E402
import twin  # noqa: E402
from library_v2 import LIBRARY_V2  # noqa: E402
from library_v3 import LIBRARY_V3  # noqa: E402
from plant import DT, N_THR, Plant, PlantSpec  # noqa: E402
from plantcard.family import Evidence, Segment  # noqa: E402
from plantcard.marine.fieldtrials import sg_velocity  # noqa: E402
from study import _seed  # noqa: E402

LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
SEED, DESIGN_SEED = 3131, 3137                       # development seeds (exploratory)
FORBIDDEN = range(20261109, 20261117)                # confirmatory seeds M0..M6 (+1 reserve): refused
CAP = 60.0
M, K = 16, 6                                         # evaluation particles / hidden repeats (as M5)
M_DEV, K_DEV = 8, 4                                  # design phase (cheaper; stated in README)
BUDGETS = (60, 180)                                  # short budgets; 624 s = the full probe log (reference)
FULL_S = probes.probe_seconds()                      # 624
STRATA = ('timing', 'transient', 'steady', 'mixed', 'ambiguous')
ADAPT_THR = 0.20                                     # s, stopwatch bin edge (fixed a priori: crude-rule theta ~0.17)
PADS = tuple(np.round(np.arange(-0.10, 0.3001, 0.025), 3))
LEVELS = tuple(twin.GRID['delay_s'])                 # assumed-delay levels a stopwatch maps to

# ------------------------------------------------------------------------------------------------ segments
BASE_S, EDGE_S, GRID_DT = 8.0, 1.0, probes.GRID_DT  # settle/baseline (counted), grid trim at both ends
AX = probes._axis
PASSIVE_EFFORT, TRANSIT_S, DRIFT_S, PREROLL_S = 0.3, 40.0, 20.0, 10.0
EFFORTS = {'ystep': (0.3, 0.6), 'yrev': (0.4, 0.7), 'sstep': (0.4, 0.8), 'srev': (0.5, 0.8),
           'ytrain': (0.4, 0.7), 'coast': (0.5, 0.9), 'turn': (0.2, 0.4)}


def _phases(kind, e, sg):
    """(phases [(dur, q)], coast s) of one maneuver; sg = +-1 alternates the sign by repeat."""
    if kind == 'ystep':
        return [(2.0, AX(0, 0, sg * e))], 6.0
    if kind == 'yrev':
        return [(1.5, AX(0, 0, sg * e)), (1.5, AX(0, 0, -sg * e))], 6.0
    if kind == 'sstep':
        return [(3.0, AX(sg * e, 0, 0))], 6.0
    if kind == 'srev':
        return [(2.0, AX(sg * e, 0, 0)), (2.0, AX(-sg * e, 0, 0))], 6.0
    if kind == 'ytrain':                               # 5 alternating 0.3 s yaw pulses, 0.3 s apart
        ph = []
        for j in range(5):
            ph += [(0.3, AX(0, 0, sg * e * (-1) ** j)), (0.3, np.zeros(N_THR))]
        return ph[:-1], 6.3
    if kind == 'coast':                                # forward surge, long coast-down
        return [(4.0, AX(e, 0, 0))], 10.0
    if kind == 'turn':                                 # turn pair at surge 0.6
        return [(3.0, AX(0.6, 0, sg * e)), (3.0, AX(0.6, 0, -sg * e))], 6.0
    raise KeyError(kind)


def seg_info(key):
    """Static description of a segment key (name, rep): duration (boat seconds, all counted), preroll
    (seconds before the log starts; only the passive transit has one, see README), command function."""
    name, rep = key
    sg = 1.0 if rep % 2 == 0 else -1.0
    if name == 'drift':
        return {'name': name, 'group': 'passive', 'dur': DRIFT_S, 'pre': 0.0, 'phases': [], 'const': None,
                'pulse_s': 0.0}
    if name == 'transit':
        return {'name': name, 'group': 'passive', 'dur': TRANSIT_S, 'pre': PREROLL_S, 'phases': [],
                'const': AX(PASSIVE_EFFORT, 0, 0), 'pulse_s': 0.0}
    kind, e = name.rstrip('0123456789.'), float(name[len(name.rstrip('0123456789.')):])
    if e not in EFFORTS.get(kind, ()):
        raise KeyError(f'unknown segment {name!r}')
    ph, coast = _phases(kind, e, sg)
    pulse = sum(d for d, _ in ph)
    return {'name': name, 'group': kind, 'dur': round(BASE_S + pulse + coast, 6), 'pre': 0.0, 'phases': ph,
            'const': None, 'pulse_s': pulse}


def seg_seconds(key):
    return seg_info(key)['dur']


def design_seconds(design):
    return float(sum(seg_seconds(k) for k in design))


def _command(info, tt):
    q = np.zeros((len(tt), N_THR))
    if info['const'] is not None:
        q[:] = info['const']
        return q
    t0 = info['pre'] + BASE_S
    for dur, v in info['phases']:
        q[(tt >= t0 - 1e-9) & (tt < t0 + dur - 1e-9)] = v
        t0 += dur
    return q


def simulate(spec_row, keys, seed, pid):
    """{key: Segment} of one boat. Each segment is its own plant env with a seed derived from (seed, pid, key)
    (counter-based noise), so a realization does not depend on which other segments share the batch: every
    arm that uses a key sees the same data (common random numbers). Measurement pipeline as probes.generate
    (GPS 5 Hz -> SG velocity, heading 10 Hz, gyro box-smoothed, stamped on arrival); only samples inside the
    segment's own window are kept."""
    keys = list(dict.fromkeys(keys))
    info = [seg_info(k) for k in keys]
    n = len(keys)
    t_run = max(i['pre'] + i['dur'] for i in info)
    ticks = int(round(t_run / DT))
    tt = np.arange(ticks) * DT
    Q = np.stack([_command(i, tt) for i in info], 1)
    seeds = np.array([_seed(seed, pid, 'seg', k[0], k[1]) for k in keys], np.uint64)
    plant = Plant(spec_row.repeat(n), seeds, t_max=t_run)
    gps_every, head_every, gyro_every = (int(round(x / DT)) for x in (probes.GPS_DT, probes.HEAD_DT, probes.GYRO_DT))
    tg, P, th, H, tr, R = [], [], [], [], [], []
    for k in range(ticks):
        plant.step(Q[k])
        pose, vel, gyro = plant.measure()
        if plant.tick % gps_every == 0:
            tg.append(plant.t)
            P.append(pose[:, :2])
        if plant.tick % head_every == 0:
            th.append(plant.t)
            H.append(pose[:, 2])
        if plant.tick % gyro_every == 0:
            tr.append(plant.t)
            R.append(gyro)
    tg, P, th, H, tr, R = map(np.asarray, (tg, P, th, H, tr, R))
    out = {}
    for i, (key, inf) in enumerate(zip(keys, info)):
        lo, hi = inf['pre'], inf['pre'] + inf['dur']
        mg, mh, mr = ((x >= lo - 1e-9) & (x <= hi + 1e-9) for x in (tg, th, tr))
        tgi, Pi, thi, Hi, tri, Ri = tg[mg], P[mg, i], th[mh], H[mh, i], tr[mr], R[mr, i]
        ng = int(round((inf['dur'] - 2 * EDGE_S) / GRID_DT))
        grid = lo + EDGE_S + np.arange(ng) * GRID_DT
        pos = np.stack([np.interp(grid, tgi, Pi[:, 0]), np.interp(grid, tgi, Pi[:, 1])], -1)
        psi = np.interp(grid, thi, np.unwrap(Hi))
        r = np.interp(grid, tri, np.convolve(Ri, np.ones(5) / 5, 'same'))
        vw = sg_velocity(tgi, Pi, grid)
        c, s = np.cos(psi), np.sin(psi)
        X = np.stack([pos[:, 0], pos[:, 1], psi, c * vw[:, 0] + s * vw[:, 1], -s * vw[:, 0] + c * vw[:, 1], r], -1)
        U = _command(inf, grid)
        t0 = grid[0]
        if inf['name'] == 'transit':
            # start: the twin's spin-up transient (lag <= 0.45 s) is gone; end: twin.Regression clips the integrated
            # thrust at the last grid tick while GPS runs 1 s longer, harmless only when thrust is off by then
            bw, fw = None, (5.0, grid[-1] - t0 - 1.5)
        elif inf['name'] == 'drift':
            bw, fw = (grid >= lo + 4.0) & (grid < hi - EDGE_S), (0.0, grid[-1] - t0)
        else:
            bw = (grid >= lo + BASE_S / 2) & (grid < lo + BASE_S - 0.2)
            fw = (lo + BASE_S - 1.0 - t0, min(grid[-1], lo + BASE_S + inf['pulse_s'] + 8.0) - t0)
        if bw is not None:
            k0, k1 = np.flatnonzero(bw)[[0, -1]]
            cur = tuple(float(x) for x in (pos[k1] - pos[k0]) / (grid[k1] - grid[k0]))
            cur_sd = float(np.sqrt(vw[bw].var(0).mean()))
        else:
            cur, cur_sd = None, None
        h = hashlib.sha256(np.ascontiguousarray(X).tobytes() + np.ascontiguousarray(U).tobytes()).hexdigest()
        out[key] = Segment(
            session=f'probe:{inf["group"]}', tier='synthetic', t=grid - t0, x=X, u=U, u_kind='pwm_q',
            quality={'synthetic': True}, labels=[inf['group'], inf['name']],
            meta={'current': cur, 'current_sd': cur_sd, 'current_measured': cur is not None, 'fit_window': fw,
                  'pulse_s': inf['pulse_s'], 'boat_s': inf['dur'], 'key': [key[0], int(key[1])],
                  'flags': ['synthetic'], 'gps_t': tgi - t0, 'gyro_t': tri - t0},
            name=f'{key[0]}#{key[1]}', source_sha=h, t_event=(lo + BASE_S - t0) if inf['phases'] else 0.0)
    drift = [s.meta['current'] for s in out.values() if s.meta['current_measured'] and s.labels[1] == 'drift']
    for s in out.values():                             # the transit uses the drift's current estimate
        if not s.meta['current_measured']:
            cd = [x.meta['current'] for x in out.values() if x.meta['current_measured']]
            cs = [x.meta['current_sd'] for x in out.values() if x.meta['current_measured']]
            s.meta['current'] = tuple(np.mean(drift or cd or [(0.0, 0.0)], 0))
            s.meta['current_sd'] = float(np.mean(cs or [0.0]))
    return out


def evidence_of(segs, design, tag):
    ss = [segs[k] for k in design]
    return Evidence(ss, {'kind': 'probe_value', 'design': tag, 'boat_s': design_seconds(design),
                         'sha256': hashlib.sha256(''.join(s.source_sha for s in ss).encode()).hexdigest()})


# ------------------------------------------------------------------------------------------------ designs
def _with_reps(names, start=None):
    """Keys for a list of names, numbering repeats of each name from ``start`` (dict name -> next rep)."""
    cnt = dict(start or {})
    out = []
    for n in names:
        out.append((n, cnt.get(n, 0)))
        cnt[n] = cnt.get(n, 0) + 1
    return tuple(out)


def seq_design(seq, budget):
    """Cycle a sequence of segment names, stop at the first one that does not fit the budget."""
    names, used = [], 0.0
    for n in itertools.cycle(seq):
        d = seg_seconds((n, 0))
        if used + d > budget + 1e-9:
            break
        names.append(n)
        used += d
    return _with_reps(names)


PULSE_SEQ = ('ystep0.3', 'sstep0.8', 'ystep0.3', 'sstep0.4', 'ystep0.6', 'sstep0.8')
REV_SEQ = ('yrev0.4', 'srev0.8', 'yrev0.7', 'srev0.5')
PAIRS = {'P1': ('ystep0.3', 'sstep0.8'), 'P2': ('yrev0.4', 'srev0.8'), 'P3': ('ytrain0.7', 'srev0.5'),
         'P4': ('ystep0.6', 'sstep0.4'), 'P5': ('yrev0.7', 'coast0.9'), 'P6': ('turn0.2', 'turn0.4')}
FILLERS = ('ystep0.3', 'ytrain0.4')                 # 'coast0.5' filler trimmed for compute (README)
STAGE1 = ('ystep0.3', 0)                             # the adaptive design's first segment (16 s)


def passive_design(budget):
    n = int((budget - DRIFT_S) // TRANSIT_S)
    return (('drift', 0),) + tuple(('transit', i) for i in range(n))


def fixed_designs(budget):
    return {'pulse': seq_design(PULSE_SEQ, budget), 'reversal': seq_design(REV_SEQ, budget),
            'passive': passive_design(budget)}


def canon(design):
    return tuple(sorted(design))


def candidates(budget, small=False):
    """The searched candidate designs at a budget: {label: design}. 60 s: one pair + one filler; 180 s: three
    repeats of a 60 s pair+filler, or five of the six pairs (+ filler if it fits); the fixed pulse and
    reversal designs are always candidates. Duplicates (same multiset of keys) are merged."""
    out = {}
    base60 = {f'{p}+{f}': PAIRS[p] + (f,) for p in PAIRS for f in FILLERS}
    if budget <= 60:
        for lab, names in base60.items():
            d = _with_reps(names)
            if design_seconds(d) <= budget:
                out[lab] = d
    else:
        for lab, names in base60.items():
            d = _with_reps(names * 3)
            if design_seconds(_with_reps(names)) <= 60 and design_seconds(d) <= budget:
                out[f'3x({lab})'] = d
        for drop in PAIRS:
            names = sum((PAIRS[p] for p in PAIRS if p != drop), ())
            d = _with_reps(names)
            if design_seconds(d) <= budget:
                for f in FILLERS:
                    d2 = _with_reps(names + (f,))
                    if design_seconds(d2) <= budget:
                        d = d2
                        break
                out[f'all-but-{drop}'] = d
    fx = fixed_designs(budget)
    out = {'pulse': fx['pulse'], 'reversal': fx['reversal'], **out}      # fixed designs keep their names on merge
    seen, uniq = {}, {}
    for lab, d in out.items():
        c = canon(d)
        if c in seen:
            continue
        seen[c] = lab
        uniq[lab] = d
    if small:                                          # smoke: the fixed designs and two others
        keep = ['pulse', 'reversal'] + [k for k in uniq if k not in ('pulse', 'reversal')][:2]
        uniq = {k: uniq[k] for k in keep}
    for d in uniq.values():
        assert design_seconds(d) <= budget + 1e-9
    return uniq


# ------------------------------------------------------------------------------------------------ fitting
class Reg(twin.Regression):
    """twin.Regression that (1) tolerates a block no segment excites (zero rows: that block stays at the
    prior card, its evidence is constant over the grid), and (2) gives cheap per-design views of one union
    regression (rows are per segment, so a view = the same cumulative thrust restricted to some segments)."""

    def _build(self, ref):
        segs, cum = self.segs, self.cum
        Ys, Ps = {b: [] for b in twin.BLOCKS}, {b: [] for b in twin.BLOCKS}
        G = cum.shape[1]
        for i, s in enumerate(segs):
            s.meta.pop('_cache', None)
            bb = self.fam._base(s, ref, *self._win)
            k1, k2 = bb['k1'], bb['k2']
            y = cum[i][:, k2] - cum[i][:, k1] - bb['icorr'][None]
            wrench = twin.static_thrust(s.u, twin.K_REF, twin.K_REF, 2.0, 0.0) @ twin.B_GEOM.T
            excited = np.abs(wrench).max(0) >= np.asarray(twin.EXCITES)
            st = np.all(np.abs(bb['dnu']) < twin.STEADY_DNU, axis=1)
            for j, b in enumerate(twin.BLOCKS):
                if not excited[j]:
                    continue
                Ps[b].append((np.stack([bb['dnu'][:, j], bb['ir'][:, j], bb['iq'][:, j]], -1), st))
                Ys[b].append(y[:, :, j])
        blocks = {}
        for b in twin.BLOCKS:
            if not Ps[b]:
                blocks[b] = (np.zeros((0, 3)), np.zeros((G, 0)), np.zeros(0, bool))
            else:
                blocks[b] = (np.concatenate([p for p, _ in Ps[b]]), np.concatenate(Ys[b], axis=1),
                             np.concatenate([m for _, m in Ps[b]]))
        return blocks

    def view(self, keys):
        """Regression of the segments with these keys only (keys: list of (name, rep))."""
        pos = {tuple(s.meta['key']): i for i, s in enumerate(self.segs)}
        idx = [pos[tuple(k)] for k in keys]
        r = copy.copy(self)
        r.segs = [self.segs[i] for i in idx]
        r.cum = self.cum[idx]
        r.ref = dict(twin.NOMINAL)
        r.blocks = r._build(r.ref)
        r.n_rows = {b: len(v[0]) for b, v in r.blocks.items()}
        return r


def grid_posterior(reg, arm):
    """Grid indices and posterior weights of ``arm`` (the evidence part of twin.fit, replicated so the timing
    marginal can be read; test_probe_value checks the MAP agrees with twin.fit's report)."""
    grid_free, nig_free, rows = twin.FREE[arm]
    fixed = {k: twin.NOMINAL[k] for k in twin.GRID if k not in grid_free}
    gidx = reg.index(fixed)
    passes = twin.REF_PASSES if 'm' in nig_free else 1
    for it in range(passes):
        logp = np.zeros(len(gidx))
        posts = {}
        for b, names in twin.BLOCKS.items():
            Phi, Y, steady = reg.blocks[b]
            if rows == 'steady':
                Phi, Y = Phi[steady], Y[:, steady]
            Y = Y[gidx]
            pinned = {}
            if 'm' not in nig_free:
                pinned[0] = twin.NOMINAL[names[0]]
            if 'd' not in nig_free:
                pinned[1], pinned[2] = twin.NOMINAL[names[1]], twin.NOMINAL[names[2]]
            post = twin.batched_posterior(twin.nig_prior(names), Phi, Y, pinned)
            posts[b] = post
            logp += post['logev']
        if it + 1 < passes:
            g = int(np.argmax(logp))
            ref = dict(reg.ref)
            for b, names in twin.BLOCKS.items():
                post = posts[b]
                if 0 in post['free']:
                    ref[names[0]] = float(max(post['mu'][g][post['free'].index(0)], twin.BOUNDS[names[0]]))
            reg = reg.with_ref(ref)
    w = np.exp(logp - logp.max())
    return gidx, w / w.sum()


def timing_marginals(reg, gidx, w):
    """Entropy (nats) of the (delay, lag, hold) marginal and the per-axis marginals."""
    P = np.array(reg.points)[gidx]
    names = list(twin.GRID)
    out = {}
    for ax in ('delay_s', 'tau_s', 'rev_hold_s'):
        vals = twin.GRID[ax]
        j = names.index(ax)
        out[ax] = [float(w[np.isclose(P[:, j], v)].sum()) for v in vals]
    joint = {}
    for pt, wi in zip(map(tuple, P[:, :3]), w):
        joint[pt] = joint.get(pt, 0.0) + wi
    pj = np.array([v for v in joint.values() if v > 0])
    out['H_timing'] = float(-(pj * np.log(pj)).sum())
    out['H_timing_prior'] = float(math.log(np.prod([len(twin.GRID[a]) for a in ('delay_s', 'tau_s', 'rev_hold_s')])))
    return out


def stopwatch(segs, design, first_only=False):
    """Apparent yaw-step delay (command onset to 10% of the yaw-rate response; baseline_selector._step) averaged
    over the design's yaw-step segments (NaN if none responds above the noise)."""
    keys = [k for k in design if k[0].startswith('ystep')]
    if first_only:
        keys = keys[:1]
    vals = [bsel._step(bsel._channel(segs[k], 5))['delay'] for k in keys]
    vals = [v for v in vals if np.isfinite(v)]
    return float(np.mean(vals)) if vals else float('nan')


def stopwatch_level(s1, pad, nan_level):
    if not np.isfinite(s1):
        return float(nan_level)
    for g in LEVELS:
        if g >= s1 + pad - 1e-9:
            return float(g)
    return float(LEVELS[-1])


def twin_current(ev):
    cs = [s.meta['current'] for s in ev.segments if s.meta.get('current_measured', True)]
    if not cs:
        cs = [s.meta['current'] for s in ev.segments]
    return tuple(np.mean(cs, 0)), float(np.std(cs, axis=0).mean())


# ------------------------------------------------------------------------------------------------ boats
def check_seeds(seed, design_seed=None):
    for s in (seed, design_seed):
        if s is not None and int(s) in FORBIDDEN:
            raise SystemExit(f'seed {s} belongs to the confirmatory range 20261109-20261116; refused')
    if design_seed is not None and int(seed) == int(design_seed):
        raise SystemExit('design (development) and evaluation seeds must differ')


def sample_ambiguous(n, seed):
    """Delay/lag-ambiguous boats: steady-stratum draws (own hashed stream) with the timing overwritten.
    Total sluggishness dead + latency + lag ~ U[0.30, 0.40] s; delay share stratified over [0, 1] (Latin
    hypercube, so every pilot covers all-lag to all-delay); latency 20-50% of the pure delay; spin-down
    ratio U[0.8, 1.2]; reversal hold U[0, 0.3] s. hidden.py is not edited."""
    s2 = _seed(seed, 'ambiguous') % (2 ** 31)
    base = hidden.sample_hidden(n, s2, 'steady')
    rng = np.random.default_rng([s2, 77])
    total = rng.uniform(0.30, 0.40, n)
    share = (rng.permutation(n) + rng.uniform(0, 1, n)) / n
    lat_frac = rng.uniform(0.2, 0.5, n)
    L = share * total
    tau = (1 - share) * total
    spec = base.with_(dead_time=L * (1 - lat_frac), odom_latency=L * lat_frac, tau_up=tau,
                      tau_down=tau * rng.uniform(0.8, 1.2, n), rev_hold=rng.uniform(0.0, 0.3, n))
    return spec, share, total


def boats(n_per, seed, prefix=''):
    out = []
    for st in STRATA:
        if st == 'ambiguous':
            spec, share, total = sample_ambiguous(n_per, seed)
        else:
            spec, share, total = hidden.sample_hidden(n_per, seed, st), [None] * n_per, [None] * n_per
        for i in range(n_per):
            out.append({'spec': spec.take([i]), 'id': f'{prefix}{st}-n{n_per}-{i:03d}', 'stratum': st, 'seed': seed,
                        'share': None if share[i] is None else float(share[i]),
                        'total': None if total[i] is None else float(total[i])})
    return out


def truth_of(spec_row):
    r = spec_row.row(0)
    L = r['dead_time'] + r['odom_latency']
    return {'L': float(L), 'dead_time': r['dead_time'], 'odom_latency': r['odom_latency'], 'tau_up': r['tau_up'],
            'tau_down': r['tau_down'], 'rev_hold': r['rev_hold'], 'thrust_mean': float(np.mean(r['k_fwd']))}


# ------------------------------------------------------------------------------------------------ flights
def fly(spec_row, pid, seed, twins, M_, K_):
    """twins: {arm: PlantSpec of M_ particles}. One mission batch per library (twins + hidden boat)."""
    out = {}
    arms = list(twins)
    for lname, lib in LIBS.items():
        C = len(lib)
        sp, ct, sd = [], [], []
        for a in arms:
            sp.append(twins[a].repeat(C))
            ct.append(np.tile(np.arange(C), M_))
            sd.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M_)], C))
        sp.append(spec_row.repeat(C * K_))
        ct.append(np.repeat(np.arange(C), K_))
        sd.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K_)], C))
        met, _ = mission.run(PlantSpec.concat(sp), np.concatenate(ct), np.concatenate(sd).astype(np.uint64), lib=lib)
        J, off, J_arms = met['J'], 0, {}
        for a in arms:
            J_arms[a] = J[off:off + M_ * C].reshape(M_, C).T
            off += M_ * C
        J_h = J[off:].reshape(C, K_)
        docked = met['docked'][off:].reshape(C, K_).mean(1)
        sc = score.plant_scores(J_h, J_arms, default_arm=arms[0])
        out[lname] = {'J_h': np.round(J_h, 2).tolist(), 'hidden_docked': docked.tolist(), 'oracle': sc['oracle'],
                      'arms': {a: {'sel': sc['arms'][a]['sel'], 'regret': float(sc['arms'][a]['regret']),
                                   'regret_c': float(min(sc['arms'][a]['regret'], CAP)),
                                   'dock': float(docked[sc['arms'][a]['sel']]),
                                   'J_twin': np.round(score.finite_scores(J_arms[a]), 1).tolist()}
                               for a in arms}}
    return out


def make_twin(particles, ev, seed, pid):
    cur, cur_sd = twin_current(ev) if ev is not None else ((0.0, 0.0), 0.0)
    return twin.to_spec(particles, current=cur, current_sd=cur_sd,
                        rng=np.random.default_rng(_seed(seed, pid, 'cur') % (2 ** 31)))


def _fit(reg, arm, M_, seed, pid):
    return twin.fit(reg, arm, M_, _seed(seed, pid, 'fit') % (2 ** 31))      # one fit seed for every arm (CRN)


def _pinfo(p, rep, marg=None):
    d = {'particles': {k: np.round(np.asarray(v, float), 5).tolist() for k, v in p.items()},
         'carried_delay': float(np.mean(p['delay_s'])), 'carried_lag': float(np.mean(p['tau_s'])),
         'carried_hold': float(np.mean(p['rev_hold_s'])),
         'fit': {k: rep.get(k) for k in ('arm', 'map', 'grid_entropy', 'block_map', 'rows', 'frac_clipped')}}
    if marg is not None:
        d['timing'] = marg
    return d


# ------------------------------------------------------------------------------------------------ design phase
def design_space(small=False):
    cand = {b: candidates(b, small) for b in BUDGETS}
    return cand


def dev_evaluate(job):
    spec_row, pid, seed, small = job['spec'], job['id'], job['seed'], job['small']
    M_, K_ = job['M'], job['K']
    t0 = time.time()
    cand = design_space(small)
    keys = sorted({k for b in cand for d in cand[b].values() for k in d})
    segs = simulate(spec_row, keys, seed, pid)
    reg = Reg(Evidence([segs[k] for k in keys], {'sha256': ''}))
    t_reg = time.time() - t0
    twins, info = {}, {}
    for b in BUDGETS:
        for lab, d in cand[b].items():
            name = f'{b}:{lab}'
            v = reg.view(d)
            p, rep = _fit(v, 'joint', M_, seed, pid)
            gidx, w = grid_posterior(v, 'joint')
            ev = evidence_of(segs, d, name)
            twins[name] = make_twin(p, ev, seed, pid)
            info[name] = {'design': [list(k) for k in d], 'seconds': design_seconds(d), 'budget': b,
                          'H_timing': timing_marginals(v, gidx, w)['H_timing'],
                          'carried_delay': float(np.mean(p['delay_s'])), 'map': rep['map']}
    pulse60 = cand[BUDGETS[0]]['pulse']
    v = reg.view(pulse60)
    ev60 = evidence_of(segs, pulse60, 'pulse60')
    for g in LEVELS:
        p, _ = _fit(v, f'rigid@d{g}', M_, seed, pid)
        twins[f'sw@{g}'] = make_twin(p, ev60, seed, pid)
    t_fit = time.time() - t0 - t_reg
    res = fly(spec_row, pid, seed, twins, M_, K_)
    return {'id': pid, 'stratum': job['stratum'], 'seed': seed, 'truth': truth_of(spec_row), 'share': job['share'],
            'S1_stage1': stopwatch(segs, (STAGE1,), first_only=True), 'S1_pulse60': stopwatch(segs, pulse60),
            'designs': info, 'libs': {l: {'arms': {a: {k: r['arms'][a][k] for k in ('sel', 'regret_c', 'dock')}
                                                    for a in r['arms']}, 'J_h': r['J_h']} for l, r in res.items()},
            'wall_s': {'reg': t_reg, 'fit': t_fit, 'total': time.time() - t0, 'n_segments': len(keys)}}


def choose(rows, small=False):
    """All design choices from development rows only."""
    cand = design_space(small)
    reg_of = lambda r, a: np.mean([r['libs'][l]['arms'][a]['regret_c'] for l in LIBS])
    out = {'candidates': {}, 'chosen': {}, 'dev_ids': [r['id'] for r in rows], 'n_dev': len(rows)}
    for b in BUDGETS:
        tab = {}
        for lab, d in cand[b].items():
            a = f'{b}:{lab}'
            tab[lab] = {'design': [list(k) for k in d], 'seconds': design_seconds(d),
                        'regret_pooled': float(np.mean([reg_of(r, a) for r in rows])),
                        'regret': {l: float(np.mean([r['libs'][l]['arms'][a]['regret_c'] for r in rows])) for l in LIBS},
                        'dock': {l: float(np.mean([r['libs'][l]['arms'][a]['dock'] for r in rows])) for l in LIBS},
                        'H_timing': float(np.mean([r['designs'][a]['H_timing'] for r in rows])),
                        'has_stage1': list(STAGE1) in [list(k) for k in d]}
        out['candidates'][str(b)] = tab
        order = lambda key: sorted(tab, key=lambda k: (tab[k][key], tab[k]['seconds']))
        task = order('regret_pooled')[0]
        dopt = order('H_timing')[0]
        out['chosen'][f'task@{b}'] = {'label': task, 'design': tab[task]['design']}
        out['chosen'][f'dopt@{b}'] = {'label': dopt, 'design': tab[dopt]['design']}
        s1 = np.array([r['S1_stage1'] for r in rows], float)
        hi_bin = ~(s1 < ADAPT_THR)                     # NaN (no detectable response) counts as sluggish
        branches = {}
        stage = [k for k in tab if tab[k]['has_stage1']]
        for bname, mask in (('lo', ~hi_bin), ('hi', hi_bin)):
            R = [r for r, m in zip(rows, mask) if m]
            if len(R) < 3:                             # too few development boats in a bin: pooled choice
                R, fb = rows, True
            else:
                fb = False
            best = min(stage, key=lambda k: (np.mean([reg_of(r, f'{b}:{k}') for r in R]), tab[k]['seconds']))
            branches[bname] = {'label': best, 'design': tab[best]['design'], 'n_dev': len(R), 'pooled_fallback': fb,
                               'dev_regret': float(np.mean([reg_of(r, f'{b}:{best}') for r in R]))}
        out['chosen'][f'adaptive@{b}'] = {'threshold_s': ADAPT_THR, 'stage1': list(STAGE1), 'branches': branches,
                                          'searched': stage}
    # stopwatch pad: level = smallest grid delay >= S1 + pad (rigid@d twin on the 60 s pulse evidence)
    s1 = np.array([r['S1_pulse60'] for r in rows], float)
    lvl_reg = lambda r, g: reg_of(r, f'sw@{g}')
    nanr = [r for r, s in zip(rows, s1) if not np.isfinite(s)]
    nan_level = min(LEVELS, key=lambda g: np.mean([lvl_reg(r, g) for r in nanr])) if nanr else 0.47
    pads = {}
    for pad in PADS:
        pads[float(pad)] = float(np.mean([lvl_reg(r, stopwatch_level(s, pad, nan_level)) for r, s in zip(rows, s1)]))
    best_pad = float(min(pads, key=lambda p: (pads[p], abs(p)))) + 0.0     # + 0.0: no '-0.000'
    out['stopwatch'] = {'pad': best_pad, 'nan_level': float(nan_level), 'pad_table': pads, 'n_nan': len(nanr),
                        'level_regret': {str(g): float(np.mean([lvl_reg(r, g) for r in rows])) for g in LEVELS},
                        'rule': 'assumed delay = smallest twin.GRID delay >= S1 + pad; rigid@d body'}
    return out


# ------------------------------------------------------------------------------------------------ evaluation
def eval_arms(design):
    """{arm: (regression name, design, fit arm, adaptive?)}; adaptive and stopwatch levels resolved per boat."""
    ch = design['chosen']
    arms = {'nominal': ('none', (), 'nominal')}
    for b in BUDGETS:
        fx = fixed_designs(b)
        arms[f'passive@{b}'] = ('passive', fx['passive'], 'joint')
        arms[f'pulse@{b}'] = ('short', fx['pulse'], 'joint')
        arms[f'reversal@{b}'] = ('short', fx['reversal'], 'joint')
        arms[f'stopwatch@{b}'] = ('short', fx['pulse'], 'stopwatch')
        arms[f'task@{b}'] = ('short', tuple(map(tuple, ch[f'task@{b}']['design'])), 'joint')
        arms[f'adaptive@{b}'] = ('short', None, 'adaptive')
        arms[f'dopt@{b}'] = ('short', tuple(map(tuple, ch[f'dopt@{b}']['design'])), 'joint')
    arms['full@624'] = ('full', None, 'joint')
    arms['rigid@624'] = ('full', None, 'rigid')
    return arms


def arm_budget(arm):
    return 0 if arm == 'nominal' else int(arm.split('@')[1])


def validate_design(design):
    """Refuse a design.json that breaks cost accounting or seed separation."""
    for b in BUDGETS:
        for kind in ('task', 'dopt'):
            d = tuple(map(tuple, design['chosen'][f'{kind}@{b}']['design']))
            if design_seconds(d) > b + 1e-9:
                raise SystemExit(f'{kind}@{b}: {design_seconds(d)} s exceeds the budget')
        for br in design['chosen'][f'adaptive@{b}']['branches'].values():
            d = tuple(map(tuple, br['design']))
            if design_seconds(d) > b + 1e-9 or list(STAGE1) not in [list(k) for k in d]:
                raise SystemExit(f'adaptive@{b}: branch breaks the budget or lacks the stage-1 yaw step')


def evaluate(job):
    spec_row, pid, seed, design = job['spec'], job['id'], job['seed'], job['design']
    M_, K_ = job['M'], job['K']
    t0 = time.time()
    arms = eval_arms(design)
    ch, sw = design['chosen'], design['stopwatch']
    # resolve the adaptive branch from the stage-1 segment only, then the union of short segments
    s1_stage = None
    adapt = {}
    short_keys = {STAGE1}
    for a, (rn, d, fa) in arms.items():
        if rn == 'short' and d is not None:
            short_keys |= set(d)
    for b in BUDGETS:
        for br in ch[f'adaptive@{b}']['branches'].values():
            short_keys |= set(map(tuple, br['design']))
    passive_keys = set().union(*(set(arms[f'passive@{b}'][1]) for b in BUDGETS))
    segs = simulate(spec_row, sorted(short_keys), seed, pid)
    psegs = simulate(spec_row, sorted(passive_keys), seed, pid)
    s1_stage = stopwatch(segs, (STAGE1,), first_only=True)
    for b in BUDGETS:
        bname = 'lo' if s1_stage < ADAPT_THR else 'hi'
        br = ch[f'adaptive@{b}']['branches'][bname]
        adapt[b] = (bname, tuple(map(tuple, br['design'])), br['label'])
    t_sim = time.time() - t0
    regs = {'short': Reg(Evidence([segs[k] for k in sorted(short_keys)], {'sha256': ''})),
            'passive': Reg(Evidence([psegs[k] for k in sorted(passive_keys)], {'sha256': ''}))}
    ev_full, _ = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    regs['full'] = twin.Regression(ev_full)
    t_reg = time.time() - t0 - t_sim
    twins, info, cache, ckey = {}, {}, {}, {}
    for a, (rn, d, fa) in arms.items():
        b = arm_budget(a)
        extra = {}
        if fa == 'adaptive':
            extra['branch'], d, extra['branch_label'] = adapt[b]
            fa = 'joint'
        if rn == 'none':
            p, rep = twin.fit(None, 'nominal', M_, 0)
            ckey[a] = ('none', (), 'nominal')
            cache[ckey[a]] = (p, rep, None, make_twin(p, None, seed, pid))
            twins[a] = cache[ckey[a]][3]
            info[a] = {'design': [], 'seconds': 0.0, 'fit_arm': 'nominal', 'same_as': None, **_pinfo(p, rep)}
            continue
        if rn == 'full':
            reg, ev, secs, dl = regs['full'], ev_full, float(FULL_S), 'probes.PROTOCOL'
        else:
            src = segs if rn == 'short' else psegs
            reg, ev, secs, dl = regs[rn].view(d), evidence_of(src, d, a), design_seconds(d), [list(k) for k in d]
        if fa == 'stopwatch':
            extra['S1'] = stopwatch(segs, d)
            extra['assumed_delay'] = stopwatch_level(extra['S1'], sw['pad'], sw['nan_level'])
            fa = f'rigid@d{extra["assumed_delay"]}'
        ck = (rn, canon(tuple(map(tuple, d))) if d is not None else None, fa)
        ckey[a] = ck
        if ck not in cache:
            p, rep = _fit(reg, fa, M_, seed, pid)
            marg = None
            if fa == 'joint':
                gidx, w = grid_posterior(reg, 'joint')
                marg = timing_marginals(reg, gidx, w)
            cache[ck] = (p, rep, marg, make_twin(p, ev, seed, pid))
        p, rep, marg, ts = cache[ck]
        twins[a] = ts
        info[a] = {'design': dl, 'seconds': secs, 'fit_arm': fa, 'same_as': None, **extra, **_pinfo(p, rep, marg)}
        if secs > b + 1e-9:
            raise AssertionError(f'{a}: {secs} s of evidence exceeds the {b} s budget')
    # identical twins (same regression, same multiset of segments, same fit arm, same fit seed) fly once
    uniq, alias = {}, {}
    for a in twins:
        if ckey[a] in uniq:
            alias[a] = uniq[ckey[a]]
            info[a]['same_as'] = uniq[ckey[a]]
        else:
            uniq[ckey[a]] = a
    t_fit = time.time() - t0 - t_sim - t_reg
    res = fly(spec_row, pid, seed, {a: twins[a] for a in twins if a not in alias}, M_, K_)
    for l in res:
        for a, src in alias.items():
            res[l]['arms'][a] = res[l]['arms'][src]
    tr = truth_of(spec_row)
    for a in info:
        info[a]['delay_err'] = info[a]['carried_delay'] - tr['L']
    return {'id': pid, 'stratum': job['stratum'], 'seed': seed, 'truth': tr, 'share': job['share'],
            'total_sluggish': job['total'], 'spec': spec_row.row(0), 'S1_stage1': s1_stage, 'arms': info,
            'libs': res, 'n_flown': len(uniq),
            'wall_s': {'sim': t_sim, 'reg': t_reg, 'fit': t_fit, 'total': time.time() - t0,
                       'n_short_segments': len(short_keys)}}


# ------------------------------------------------------------------------------------------------ summary
def _boot(x, seed=0, n=4000):
    x = np.asarray(x, float)
    if len(x) == 0:
        return [float('nan')] * 2
    idx = np.random.default_rng(seed).integers(0, len(x), (n, len(x)))
    return [float(v) for v in np.quantile(x[idx].mean(1), [0.05, 0.95])]


def ref_arm(arm):
    b = arm_budget(arm)
    return f'pulse@{b}' if b in BUDGETS else f'pulse@{BUDGETS[-1]}'


def summarize(rows, design):
    arms = list(eval_arms(design))
    out = {'n': len(rows), 'arms': arms, 'tables': {}, 'ambiguous': {}, 'design': design['chosen'],
           'stopwatch': {k: design['stopwatch'][k] for k in ('pad', 'nan_level')}}
    for l in LIBS:
        T = {}
        for st in STRATA + ('all',):
            R = [r for r in rows if st == 'all' or r['stratum'] == st]
            if not R:
                continue
            T[st] = {}
            nom = np.array([r['libs'][l]['arms']['nominal']['regret_c'] for r in R])
            for a in arms:
                reg = np.array([r['libs'][l]['arms'][a]['regret_c'] for r in R])
                ref = np.array([r['libs'][l]['arms'][ref_arm(a)]['regret_c'] for r in R])
                secs = float(np.mean([r['arms'][a]['seconds'] for r in R]))
                T[st][a] = {'n': len(R), 'regret': float(reg.mean()), 'ci90': _boot(reg),
                            'median': float(np.median(reg)),
                            'dock': float(np.mean([r['libs'][l]['arms'][a]['dock'] for r in R])),
                            'delay_err': float(np.mean([r['arms'][a]['delay_err'] for r in R])),
                            'diff_vs_ref': float((reg - ref).mean()), 'diff_ci90': _boot(reg - ref), 'ref': ref_arm(a),
                            'seconds': secs, 'gain_vs_nominal': float((nom - reg).mean()),
                            'gain_per_min': float((nom - reg).mean() / (secs / 60.0)) if secs > 0 else None}
        out['tables'][l] = T
        R = [r for r in rows if r['stratum'] == 'ambiguous']
        if R:
            sh = np.array([r['share'] for r in R])
            bins = {'lag-dominated (share<1/3)': sh < 1 / 3, 'mixed (1/3-2/3)': (sh >= 1 / 3) & (sh < 2 / 3),
                    'delay-dominated (share>=2/3)': sh >= 2 / 3}
            out['ambiguous'][l] = {bn: {'n': int(m.sum()), **{a: {
                'regret': float(np.mean([R[i]['libs'][l]['arms'][a]['regret_c'] for i in np.flatnonzero(m)])) if m.any() else None,
                'dock': float(np.mean([R[i]['libs'][l]['arms'][a]['dock'] for i in np.flatnonzero(m)])) if m.any() else None,
                'carried_delay': float(np.mean([R[i]['arms'][a]['carried_delay'] for i in np.flatnonzero(m)])) if m.any() else None}
                for a in arms}} for bn, m in bins.items()}
            out['ambiguous'][l]['corr_regret_share'] = {
                a: float(np.corrcoef(sh, [r['libs'][l]['arms'][a]['regret_c'] for r in R])[0, 1])
                if len(R) > 2 and np.std([r['libs'][l]['arms'][a]['regret_c'] for r in R]) > 0 else None for a in arms}
    out['adaptive_branches'] = {f'adaptive@{b}': {bn: int(sum(r['arms'][f'adaptive@{b}'].get('branch') == bn for r in rows))
                                                  for bn in ('lo', 'hi')} for b in BUDGETS}
    return out


def print_summary(s):
    arms = s['arms']
    print(f'== probe value at equal cost: n={s["n"]} boats; mean capped ({CAP:.0f} s) regret; EXPLORATORY, HSUR')
    for b in BUDGETS:
        print(f'  design @{b}s: task={s["design"][f"task@{b}"]["label"]}  dopt={s["design"][f"dopt@{b}"]["label"]}  '
              f'adaptive lo={s["design"][f"adaptive@{b}"]["branches"]["lo"]["label"]} '
              f'hi={s["design"][f"adaptive@{b}"]["branches"]["hi"]["label"]}  (branch counts {s["adaptive_branches"][f"adaptive@{b}"]})')
    print(f'  stopwatch: assumed delay = grid level >= S1 + {s["stopwatch"]["pad"]:+.3f} s (no response -> {s["stopwatch"]["nan_level"]})')
    for l, T in s['tables'].items():
        st_cols = [st for st in STRATA + ('all',) if st in T]
        print(f'\n-- library {l}: regret / dock rate of the pick per stratum')
        print('  ' + f'{"arm":14s}' + ''.join(f'{c:>14s}' for c in st_cols))
        for a in arms:
            print('  ' + f'{a:14s}' + ''.join(f'{T[c][a]["regret"]:7.1f}/{T[c][a]["dock"]:4.2f} ' for c in st_cols))
        A = T['all']
        print(f'  pooled: {"arm":14s} {"s":>5s} {"regret [90% CI]":>20s} {"d(delay-L)":>10s} {"vs ref [90% CI]":>26s} {"gain/min vs nominal":>20s}')
        for a in arms:
            v = A[a]
            gpm = '' if v['gain_per_min'] is None else f'{v["gain_per_min"]:8.2f}'
            print(f'          {a:14s} {v["seconds"]:5.0f} {v["regret"]:6.1f} [{v["ci90"][0]:5.1f},{v["ci90"][1]:5.1f}] '
                  f'{v["delay_err"]:+10.3f} {v["diff_vs_ref"]:+6.1f} [{v["diff_ci90"][0]:+5.1f},{v["diff_ci90"][1]:+5.1f}] {v["ref"]:>10s} {gpm:>12s}')
    for l, A in s['ambiguous'].items():
        print(f'\n-- ambiguous stratum, library {l}: regret / dock / carried delay by true delay share')
        bins = [k for k in A if k != 'corr_regret_share']
        print('  ' + f'{"arm":14s}' + ''.join(f'{b[:28]:>30s}' for b in bins) + f'{"corr(regret,share)":>20s}')
        for a in arms:
            cells = ''.join('                          --  ' if A[b][a]['regret'] is None else
                            f'          {A[b][a]["regret"]:6.1f}/{A[b][a]["dock"]:4.2f}/{A[b][a]["carried_delay"]:4.2f}' for b in bins)
            c = A['corr_regret_share'][a]
            print('  ' + f'{a:14s}' + cells + ('' if c is None else f'{c:+20.2f}'))
        print('  ' + ' ' * 14 + ''.join(f'{"n=" + str(A[b]["n"]):>30s}' for b in bins))


# ------------------------------------------------------------------------------------------------ driver
def _load(pj):
    done = {}
    if pj.exists():
        for line in pj.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                done[r['id']] = r
    return done


def _pool_map(fn, todo, procs):
    if procs <= 1:
        yield from map(fn, todo)
        return
    with get_context('fork').Pool(procs) as pool:
        yield from pool.imap_unordered(fn, todo)


def run_design(out, design_seed, n_dev, procs, small=False, M_=M_DEV, K_=K_DEV, seed=None):
    check_seeds(seed if seed is not None else SEED, design_seed)
    out.mkdir(parents=True, exist_ok=True)
    jobs = [dict(j, M=M_, K=K_, small=small) for j in boats(n_dev, design_seed, prefix='dev-')]
    pj = out / 'dev_plants.jsonl'
    done = _load(pj)
    todo = [j for j in jobs if j['id'] not in done]
    t0 = time.time()
    with open(pj, 'a') as fp:
        for k, r in enumerate(_pool_map(dev_evaluate, todo, procs)):
            done[r['id']] = r
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            print(f'[design {k + 1}/{len(todo)}] {r["id"]} ({r["wall_s"]["total"]:.0f}s boat, {time.time() - t0:.0f}s)',
                  flush=True)
    rows = [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs]
    d = choose(rows, small)
    d.update({'design_seed': design_seed, 'small': small, 'M_dev': M_, 'K_dev': K_,
              'candidate_counts': {str(b): len(v) for b, v in design_space(small).items()},
              'created': time.strftime('%Y-%m-%dT%H:%M:%S'), 'elapsed_s': time.time() - t0})
    (out / 'design.json').write_text(json.dumps(record._clean(d), indent=1, sort_keys=True))
    for b in BUDGETS:
        tab = d['candidates'][str(b)]
        print(f'  candidates @{b}s (dev pooled regret / H_timing):')
        for lab in sorted(tab, key=lambda k: tab[k]['regret_pooled']):
            print(f'    {lab:24s} {tab[lab]["seconds"]:5.0f}s  {tab[lab]["regret_pooled"]:6.1f}  {tab[lab]["H_timing"]:5.2f}')
        print(f'  -> task {d["chosen"][f"task@{b}"]["label"]}, dopt {d["chosen"][f"dopt@{b}"]["label"]}, adaptive '
              + ', '.join(f'{k}: {v["label"]} (n={v["n_dev"]})' for k, v in d['chosen'][f'adaptive@{b}']['branches'].items()))
    print(f'  stopwatch pad {d["stopwatch"]["pad"]:+.3f} s, no-response level {d["stopwatch"]["nan_level"]}')
    return d


def run_eval(out, seed, n_per, procs, M_=M, K_=K):
    dpath = out / 'design.json'
    if not dpath.exists():
        raise SystemExit(f'{dpath} missing: run the design phase first')
    design = json.loads(dpath.read_text())
    check_seeds(seed, design['design_seed'])
    validate_design(design)
    jobs = [dict(j, M=M_, K=K_, design=design) for j in boats(n_per, seed)]
    manifest = {'seed': seed, 'n_per': n_per, 'M': M_, 'K': K_, 'procs': procs, 'arms': list(eval_arms(design)),
                'design_sha256': hashlib.sha256(dpath.read_bytes()).hexdigest(), 'design_seed': design['design_seed'],
                'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(), 'evidence_label': 'HSUR',
                'exploratory': True, 'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    (out / 'manifest.json').write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
    pj = out / 'plants.jsonl'
    done = _load(pj)
    todo = [j for j in jobs if j['id'] not in done]
    t0 = time.time()
    with open(pj, 'a') as fp:
        for k, r in enumerate(_pool_map(evaluate, todo, procs)):
            done[r['id']] = r
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            v2 = r['libs']['v2']['arms']
            print(f'[eval {k + 1}/{len(todo)}] {r["id"]} v2 ' + ' '.join(
                f'{a}={v2[a]["regret_c"]:.0f}' for a in ('nominal', 'pulse@60', 'task@60', 'pulse@180', 'full@624'))
                + f' ({r["wall_s"]["total"]:.0f}s boat, {r["n_flown"]} twins flown, {time.time() - t0:.0f}s)', flush=True)
    return summary(out)


def summary(out):
    design = json.loads((out / 'design.json').read_text())
    rows = list(_load(out / 'plants.jsonl').values())
    s = summarize(rows, design)
    (out / 'summary.json').write_text(json.dumps(record._clean(s), indent=1, sort_keys=True))
    print_summary(s)
    return s


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('cmd', choices=('design', 'eval', 'run', 'summary', 'smoke'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--design-seed', type=int, default=DESIGN_SEED)
    ap.add_argument('--n-per', type=int, default=20)
    ap.add_argument('--n-dev', type=int, default=4)
    ap.add_argument('--procs', type=int, default=4)
    a = ap.parse_args(argv)
    if a.procs > 4:
        raise SystemExit('compute policy: at most 4 processes')
    check_seeds(a.seed, a.design_seed)
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    if a.cmd == 'smoke':
        # 1 process; design on 1 dev boat per stratum would exceed the smoke cap, so: 2 dev boats (timing,
        # ambiguous), 2 eval boats, reduced candidates, M = 4, K = 2
        global STRATA
        keep = STRATA
        STRATA = ('timing', 'ambiguous')
        try:
            run_design(out, a.design_seed, 1, 1, small=True, M_=4, K_=2, seed=a.seed)
            run_eval(out, a.seed, 1, 1, M_=4, K_=2)
        finally:
            STRATA = keep
        return
    if a.cmd in ('design', 'run'):
        run_design(out, a.design_seed, a.n_dev, a.procs, seed=a.seed)
    if a.cmd in ('eval', 'run'):
        run_eval(out, a.seed, a.n_per, a.procs)
    if a.cmd == 'summary':
        summary(out)


if __name__ == '__main__':
    main()
