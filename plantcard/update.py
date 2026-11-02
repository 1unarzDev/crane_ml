"""Evidence -> a proposed child card (``status: fitted``), robot-agnostic.

1. For every point g of the parent's grid (delay, thrust shape, ...) and every NIG block, update
   the parent's block with the family's regressors at g (forgetting ``lam``) and keep the
   closed-form log evidence. Two passes: the second recomputes the coupling terms the regressors
   hold fixed (``ref``) at the first pass's posterior.
2. Grid posterior = parent grid x exp(sum of block evidences).
3. Particles: grid point ~ grid posterior, then theta ~ that point's NIG posterior (exact joint),
   truncated to the family bounds on the pooled draw (the mass removed is recorded).
4. Disturbance: the per-session spread of the mean regression residual (slow force bias), and the
   evidence's current uncertainty.
5. Envelope: the evidence's z rows, standardised, kNN distance with a leave-one-session-out
   conformal threshold.
"""

import datetime as _dt

import numpy as np

from plantcard.blocks import NIG, Grid
from plantcard.family import ParamBatch


def _collect(family, segments, g, ref):
    rows = {b: ([], []) for b in family.blocks}
    for seg in segments:
        for b, (Phi, y) in family.regressors(seg, g, ref).items():
            rows[b][0].append(Phi)
            rows[b][1].append(y)
    return {b: (np.concatenate(P), np.concatenate(Y)) for b, (P, Y) in rows.items()}


class BlockFit:
    """A block's NIG with some parameters pinned at a physical bound (active set): the free
    parameters are conditioned on the pinned values, the regressor columns of pinned ones move to
    the target. ``pinned`` = {} is the plain NIG."""

    def __init__(self, nig, pinned=None):
        self.full, self.pinned = nig, dict(pinned or {})
        d = len(nig.mu)
        self.free = [j for j in range(d) if j not in self.pinned]
        if self.pinned:
            P = sorted(self.pinned)
            L = nig.Lam
            Lff = L[np.ix_(self.free, self.free)]
            dv = np.array([self.pinned[j] for j in P]) - nig.mu[P]
            mu = nig.mu[self.free] - np.linalg.solve(Lff, L[np.ix_(self.free, P)] @ dv)
            self.nig = NIG(mu, Lff, nig.a, nig.b)
        else:
            self.nig = nig

    def _reduce(self, Phi, y):
        if not self.pinned:
            return Phi, y
        P = sorted(self.pinned)
        return Phi[:, self.free], y - Phi[:, P] @ np.array([self.pinned[j] for j in P])

    def update(self, Phi, y, lam=1.0):
        post = BlockFit(self.full, self.pinned)
        post.nig = self.nig.update(*self._reduce(Phi, y), lam)
        return post

    def log_evidence(self, Phi, y, lam=1.0):
        return self.nig.log_evidence(*self._reduce(Phi, y), lam)

    def _expand(self, th):
        out = np.empty(th.shape[:-1] + (len(self.full.mu),))
        out[..., self.free] = th
        for j, v in self.pinned.items():
            out[..., j] = v
        return out

    @property
    def mu(self):
        return self._expand(self.nig.mu)

    @property
    def noise_var(self):
        return self.nig.noise_var

    @property
    def a(self):
        return self.nig.a

    def cov(self):
        d = len(self.full.mu)
        C = np.zeros((d, d))
        C[np.ix_(self.free, self.free)] = self.nig.cov()
        return C

    def sample(self, n, rng):
        th, s2 = self.nig.sample(n, rng)
        return self._expand(th), s2

    def full_nig(self, pin_precision=1e12):
        """The posterior in all dimensions (a pinned parameter as a near-delta), so a child card
        stays a valid prior for the next update."""
        d = len(self.full.mu)
        L = np.zeros((d, d))
        L[np.ix_(self.free, self.free)] = self.nig.Lam
        for j in self.pinned:
            L[j, j] = pin_precision
        return NIG(self.mu, L, self.nig.a, self.nig.b)


def _violations(family, names, mu):
    specs = {p.name: p for p in family.params}
    out = {}
    for j, (k, v) in enumerate(zip(names, mu)):
        lo, hi = specs[k].bounds
        if lo is not None and v < lo:
            out[j] = float(lo)
        elif hi is not None and v > hi:
            out[j] = float(hi)
    return out


def fit_blocks(parent, evidence, lam=1.0, passes=2, max_pins=3):
    """Posterior per block at every grid point, the grid posterior, the MAP index, the refreshed
    reference parameters and the pins. A block whose MAP mean leaves a physical bound is refitted
    with that parameter pinned at the bound (the evidence then compares like with like)."""
    fam = parent.family
    priors = {b: NIG.from_dict(parent.blocks[b]['nig']) for b in fam.blocks}
    pins = {b: {} for b in fam.blocks}
    grid = Grid.from_dict(parent.blocks['act']['grid'])
    ref = dict(parent.doc['nominal'])
    for it in range(passes + max_pins):
        fits = {b: BlockFit(priors[b], pins[b]) for b in fam.blocks}
        loglik = np.zeros(grid.logp.shape)
        posts = {}
        for idx, g in grid.points():
            data = _collect(fam, evidence.segments, g, ref)
            posts[idx] = {b: fits[b].update(*data[b], lam) for b in fam.blocks}
            loglik[idx] = sum(fits[b].log_evidence(*data[b], lam) for b in fam.blocks)
        gpost = grid.posterior(loglik)
        idx_map = np.unravel_index(int(np.argmax(gpost.logp)), gpost.logp.shape)
        ref = dict(ref)
        for b, names in fam.blocks.items():
            ref.update(dict(zip(names, (float(v) for v in posts[idx_map][b].mu))))
        ref.update(gpost.map())
        new = False
        for b, names in fam.blocks.items():
            v = _violations(fam, names, posts[idx_map][b].mu)
            v = {j: x for j, x in v.items() if j not in pins[b]}
            if v:
                worst = min(v, key=lambda j: posts[idx_map][b].mu[j] - v[j])   # pin one at a time
                pins[b][worst] = v[worst]
                new = True
        if not new and it + 1 >= passes:
            break
    return posts, gpost, idx_map, ref, pins


def draw_particles(family, posts, gpost, n, seed):
    """Joint draws, truncated to the family's bounds. Returns (ParamBatch, removed mass)."""
    rng = np.random.default_rng(seed)
    specs = {p.name: p for p in family.params}
    rows, tried = [], 0
    while len(rows) < n and tried < 50 * n:
        need = n - len(rows)
        for idx in gpost.sample_idx(need, rng):
            g = {k: gpost.axes[k][i] for k, i in zip(gpost.axes, idx)}
            row = dict(g)
            for b, names in family.blocks.items():
                th, _ = posts[tuple(idx)][b].sample(1, rng)
                row.update(dict(zip(names, th[0])))
            tried += 1
            ok = all((specs[k].bounds[0] is None or v >= specs[k].bounds[0])
                     and (specs[k].bounds[1] is None or v <= specs[k].bounds[1]) for k, v in row.items())
            if ok:
                rows.append(row)
    if len(rows) < n:
        raise ValueError(f'bounds removed too much posterior mass ({len(rows)}/{tried} kept)')
    return ParamBatch.from_rows(rows), 1.0 - len(rows) / tried


def residual_bias(family, segments, g, ref, theta):
    """Per-segment mean residual per block (the slow force bias the dynamics miss), over every
    segment: a block's unmodelled response in segments it does not learn from is disturbance."""
    out = []
    for seg in segments:
        r = family.regressors(seg, g, ref, select=False)
        out.append([float(np.mean(y - Phi @ theta[b])) if len(y) else np.nan for b, (Phi, y) in r.items()])
    return np.asarray(out)


def envelope(family, segments, every=5, k=5, q=0.95):
    Z, sess, names = [], [], None
    for seg in segments:
        z, names = family.envelope_features(seg)
        Z.append(z[::every])
        sess += [seg.session] * len(z[::every])
    Z, sess = np.concatenate(Z), np.asarray(sess)
    center, scale = Z.mean(0), np.maximum(Z.std(0), 1e-6)
    Zs = (Z - center) / scale
    d = np.sqrt(((Zs[:, None] - Zs[None]) ** 2).sum(-1))
    d[sess[:, None] == sess[None, :]] = np.inf          # distance to OTHER sessions' support
    kth = np.sort(d, axis=1)[:, k - 1]
    kth = kth[np.isfinite(kth)]
    thr = float(np.quantile(kth, q)) if len(kth) else float('inf')
    ranges = {nm: [float(Z[:, j].min()), float(Z[:, j].max())] for j, nm in enumerate(names)}
    return ({'z': names, 'ranges': ranges, 'center': center.tolist(), 'scale': scale.tolist(),
             'support': {'file': 'support.npz', 'kind': 'knn', 'k': k, 'threshold': thr,
                         'calibrated_alpha': round(1 - q, 3), 'calibration': 'leave-one-session-out'}},
            {'z': Z})


def propose(parent, evidence, *, lam=1.0, n_particles=512, seed=7, procedure='', note=''):
    """(doc, particles, support, fit report) of a fitted child of ``parent``."""
    fam = parent.family
    posts, gpost, idx_map, ref, pins = fit_blocks(parent, evidence, lam)
    particles, removed = draw_particles(fam, posts, gpost, n_particles, seed)
    g_map = gpost.map()
    theta = {b: posts[idx_map][b].mu for b in fam.blocks}
    bias = residual_bias(fam, evidence.segments, g_map, ref, theta)
    pdist = parent.blocks.get('dist', {})
    dist = dict(pdist)
    dist.update({'kind': 'constant_bias',
                 'force_sd': [float(v) for v in np.nan_to_num(np.sqrt(np.nanmean(bias ** 2, 0)))],
                 'force_mean': [float(v) for v in np.nan_to_num(np.nanmean(bias, 0))],
                 'current_sd': float(np.median([s.meta.get('current_sd', pdist.get('current_sd', 0.0))
                                                for s in evidence.segments])),
                 'status': 'fitted', 'n_segments': len(evidence.segments),
                 'procedure': 'rms of per-segment mean residual (all segments); baseline-window current spread'})
    env, support = envelope(fam, evidence.segments)
    sources = {}
    for s in evidence.segments:
        sources.setdefault(s.session, []).append(s)
    blocks = {}
    for b, names in fam.blocks.items():
        post = posts[idx_map][b]
        cov = post.cov()
        sd = np.sqrt(np.diag(cov))
        free = post.free
        cf = cov[np.ix_(free, free)]
        corr = cf / np.outer(sd[free], sd[free])
        pinned = {names[j]: v for j, v in pins[b].items()}
        prev = parent.blocks[b]
        blocks[b] = {
            'kind': 'nig', 'params': names, 'nig': post.full_nig().to_dict(), 'forget': lam,
            'pinned': pinned,
            'mean': [float(v) for v in post.mu], 'sd': [float(v) for v in sd],
            'noise_std': float(np.sqrt(post.noise_var)),
            'n_eff': float(prev.get('n_eff', 0.0) * lam + 2 * (post.a - lam * NIG.from_dict(prev['nig']).a)),
            'n_windows': int(round(2 * (post.a - lam * NIG.from_dict(prev['nig']).a))),
            'status': 'fitted', 'procedure': procedure,
            'flags': sorted((set(prev.get('flags', [])) - {'prior_dominated', 'eiv_biased'})
                            | ({'pinned_at_bound'} if pinned else set())),
            'identifiability': {'status': 'COMBINATION_ONLY', 'with': ['gauge'],
                                'max_abs_corr': float(np.max(np.abs(corr - np.eye(len(free))))) if len(free) > 1 else 0.0,
                                'cond': float(np.linalg.cond(post.nig.Lam))},
            'bounds': {p.name: list(p.bounds) for p in fam.params if p.name in names},
            'sources': [{'kind': 'session', 'ref': k, 'segments': len(v),
                         'sha256': sorted({s.source_sha for s in v})} for k, v in sorted(sources.items())],
            'representable_in': prev.get('representable_in', []),
        }
    pgrid = parent.blocks['act']
    act = dict(pgrid, grid=gpost.to_dict(), status='fitted', procedure='NIG evidence on the grid',
               posterior={k: dict(zip(('mean', 'sd'), gpost.mean_sd(k))) for k in gpost.axes},
               map=g_map, flags=sorted(set(pgrid.get('flags', [])) | {'lumped'}))
    blocks['act'] = act
    blocks['dist'] = dist
    nominal = dict(ref)
    doc = {
        'schema': parent.doc['schema'], 'robot': parent.robot, 'version': parent.doc.get('version', 0) + 1, 'parent': parent.sha,
        'status': 'fitted', 'family': parent.doc['family'], 'rig': parent.doc.get('rig', {}),
        'gauge': parent.doc['gauge'], 'blocks': blocks, 'nominal': {k: float(v) for k, v in nominal.items()},
        'particles': {'file': 'particles.npz', 'n': n_particles, 'seed': seed,
                      'joint_over': ['dyn.*', 'act.*'], 'bounds_removed_mass': round(removed, 4)},
        'envelope': env, 'validation': [], 'views': {},
        'provenance': {'evidence': evidence.provenance, 'by': 'plantcard.update.propose',
                       'created': _dt.date.today().isoformat(), 'accepted_by': None, 'note': note,
                       'forget': lam},
    }
    report = {'grid_map': g_map, 'bounds_removed_mass': removed,
              'blocks': {b: {'mean': blocks[b]['mean'], 'sd': blocks[b]['sd']} for b in fam.blocks}}
    return doc, dict(particles), support, report
