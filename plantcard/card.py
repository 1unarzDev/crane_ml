"""The plant card (``crane.plantcard/1``): an immutable, content-addressed bundle of a family
reference, its blocks (conjugate NIG states, grids, disturbance), a particle set (the uncertainty
interchange every consumer reads), an envelope of the evidence's regime and validation records.

    card = plantcard.load('<family>/<robot>@HEAD')
    card.nominal(); card.sample(64, seed=0); card.rollout(x0, t, u, n=64, seed=0)
    card.view('unity'); card.view('nlc', n=256); card.view('fastsim')

Posterior spread is not coverage: ``predict_set`` scales the raw spread by the conformal multiplier
of a validation record (tier, event) when one exists and says ``uncalibrated`` otherwise.
"""

import hashlib
import importlib
import io
import json
import os
from pathlib import Path

import numpy as np

from plantcard.family import ParamBatch

SCHEMA = 'crane.plantcard/1'
STATUSES = ('prior', 'fitted', 'provisional', 'validated')
HERE = Path(__file__).resolve().parent
REPO_CARDS = HERE / 'cards'
REPO_REFS = HERE / 'refs.json'


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def card_sha(doc):
    return hashlib.sha256(canonical({k: v for k, v in doc.items() if k != 'sha'}).encode()).hexdigest()


def npz_bytes(arrays):
    buf = io.BytesIO()
    np.savez(buf, **{k: np.asarray(v, np.float64) for k, v in sorted(arrays.items())})
    return buf.getvalue()


def load_ref(ref):
    """A class/function from 'module:attr'."""
    mod, _, attr = ref.partition(':')
    obj = importlib.import_module(mod)
    for part in attr.split('.'):
        obj = getattr(obj, part)
    return obj


def validate_doc(doc):
    """Raise on a malformed card document (stdlib checks only)."""
    for key in ('schema', 'robot', 'status', 'family', 'gauge', 'blocks', 'particles', 'provenance'):
        if key not in doc:
            raise ValueError(f'plant card: missing {key!r}')
    if doc['schema'] != SCHEMA:
        raise ValueError(f'plant card: schema {doc["schema"]!r}, expected {SCHEMA}')
    if doc['status'] not in STATUSES:
        raise ValueError(f'plant card: status {doc["status"]!r} not in {STATUSES}')
    if 'sha' in doc and doc['sha'] != card_sha(doc):
        raise ValueError('plant card: sha does not match its content')


class Card:
    def __init__(self, doc, particles, support=None, path=None):
        validate_doc(doc)
        self.doc, self.path = doc, path
        self.particles = ParamBatch({k: np.asarray(v, float) for k, v in particles.items()})
        self.support = support
        fam = doc['family']
        self.family = load_ref(fam['ref'])(fam.get('config'))

    sha = property(lambda self: self.doc.get('sha', ''))
    robot = property(lambda self: self.doc['robot'])
    status = property(lambda self: self.doc['status'])
    blocks = property(lambda self: self.doc['blocks'])

    def __repr__(self):
        return f'<plant card {self.robot} {self.sha[:12]} {self.status} n={self.particles.n}>'

    # ---- draws ----------------------------------------------------------------------------
    def nominal(self):
        """Posterior point (NIG means, grid MAP) as a one-particle batch."""
        return ParamBatch({k: np.array([float(v)]) for k, v in self.doc['nominal'].items()})

    def sample(self, n, seed=0):
        """n particles, reproducible: a seeded resample of the stored set (no replacement while
        n <= its size)."""
        rng = np.random.default_rng(seed)
        N = self.particles.n
        idx = rng.permutation(N)[:n] if n <= N else rng.integers(0, N, n)
        return self.particles.take(idx)

    def rollout(self, x0, t, u, n=64, seed=0, seg=None, disturbance=True, params=None):
        """[n, T, nx] trajectories: one particle and one disturbance draw per rollout."""
        p = params if params is not None else self.sample(n, seed)
        rng = np.random.default_rng([seed, 1])
        w = self.family.disturbance(p.n, rng, seg, self.doc['blocks'].get('dist') if disturbance else None)
        return self.family.rollout(x0, np.asarray(t, float), np.asarray(u, float), p, w)

    def predict_set(self, x0, t, u, event='pos_err_max', horizon_s=5.0, alpha=0.1, tier='real', n=256,
                    seed=0, seg=None):
        """Predicted mean path and a radius for ``event``: the raw particle quantile times the
        conformal multiplier of the matching validation record ('uncalibrated' without one)."""
        from plantcard import validate as V
        tr = self.rollout(x0, t, u, n=n, seed=seed, seg=seg)
        k = int(np.searchsorted(t, t[0] + horizon_s, side='right'))
        mean, spread = V.spread_scores(tr[:, :k], self.family.position)
        q = float(np.quantile(spread, 1 - alpha))
        rec = next((r for r in self.doc.get('validation', []) if r.get('tier') == tier
                    and r.get('event') == f'{event}@H{horizon_s:g}s' and r.get('alpha') == alpha), None)
        if rec is None:
            return {'mean': mean, 'radius': q, 'calibrated': False, 'label': 'uncalibrated'}
        return {'mean': mean, 'radius': q * rec['conformal_multiplier'], 'calibrated': True,
                'label': f'conformal {rec["event"]} alpha={alpha} ({rec["n_units"]} units)'}

    # ---- envelope -------------------------------------------------------------------------
    def envelope_score(self, z):
        """Standardised kNN distance of rows z to the evidence support; > threshold = outside."""
        env = self.doc.get('envelope')
        if not env or self.support is None:
            return np.zeros(len(np.atleast_2d(z)))
        S = self.support['z']
        mu, sd = np.asarray(env['center']), np.asarray(env['scale'])
        Zs, zs = (S - mu) / sd, (np.atleast_2d(z) - mu) / sd
        d = np.sqrt(((zs[:, None, :] - Zs[None, :, :]) ** 2).sum(-1))
        k = env['support']['k']
        return np.sort(d, axis=1)[:, k - 1]

    def in_envelope(self, z):
        return self.envelope_score(z) <= self.doc['envelope']['support']['threshold']

    # ---- views ----------------------------------------------------------------------------
    def view(self, name, **kw):
        ref = self.family.views.get(name)
        if ref is None:
            raise ValueError(f'{self.family.name} has no view {name!r}; known: {sorted(self.family.views)}')
        return load_ref(ref)(self, **kw)


# ======================================================================================
# Store: content-addressed directories, refs per robot
# ======================================================================================

def default_store():
    return Path(os.environ.get('PLANTCARD_STORE', Path.home() / '.cache' / 'plantcards'))


def _stores(store=None):
    out = [Path(store)] if store else [default_store()]
    return out + [HERE]   # the repo copy: cards/<sha12>/ and refs.json, git-tracked


def save(doc, particles, support=None, store=None):
    """Write a card (sha computed here) to the store; returns the loaded Card."""
    doc = dict(doc)
    pbytes = npz_bytes(particles)
    doc['particles'] = dict(doc['particles'], sha256=hashlib.sha256(pbytes).hexdigest(),
                            n=int(len(next(iter(particles.values())))))
    if support is not None:
        sbytes = npz_bytes(support)
        doc['envelope'] = dict(doc['envelope'])
        doc['envelope']['support'] = dict(doc['envelope']['support'], sha256=hashlib.sha256(sbytes).hexdigest())
    doc['schema'] = SCHEMA
    doc.pop('sha', None)
    doc['sha'] = card_sha(doc)
    root = Path(store) if store else default_store()
    d = root / 'cards' / doc['sha'][:12]
    d.mkdir(parents=True, exist_ok=True)
    (d / 'particles.npz').write_bytes(pbytes)
    if support is not None:
        (d / 'support.npz').write_bytes(sbytes)
    (d / 'card.json').write_text(json.dumps(doc, indent=1, sort_keys=True) + '\n')
    return Card(doc, particles, support, d)


def read_dir(d):
    d = Path(d)
    doc = json.loads((d / 'card.json').read_text())
    pz = d / 'particles.npz'
    pbytes = pz.read_bytes()
    if hashlib.sha256(pbytes).hexdigest() != doc['particles']['sha256']:
        raise ValueError(f'{pz}: particles do not match the card sha256')
    with np.load(io.BytesIO(pbytes)) as z:
        particles = {k: z[k] for k in z.files}
    support = None
    if (d / 'support.npz').exists():
        with np.load(d / 'support.npz') as z:
            support = {k: z[k] for k in z.files}
    return Card(doc, particles, support, d)


def refs(store=None):
    """{robot: sha12} merged over the stores (the user store wins)."""
    out = {}
    for root in reversed(_stores(store)):
        f = root / 'refs.json'
        if f.exists():
            out.update(json.loads(f.read_text()))
    return out


def set_ref(robot, sha, store=None, name='HEAD'):
    """Point ``robot`` (HEAD) or ``robot@name`` at a card in the store's refs.json."""
    root = Path(store) if store else default_store()
    root.mkdir(parents=True, exist_ok=True)
    f = root / 'refs.json'
    cur = json.loads(f.read_text()) if f.exists() else {}
    cur[robot if name == 'HEAD' else f'{robot}@{name}'] = sha[:12]
    f.write_text(json.dumps(cur, indent=1, sort_keys=True) + '\n')


def load(ref, store=None):
    """A card from a directory path, 'robot@HEAD' (or 'robot'), or a sha prefix."""
    p = Path(str(ref)).expanduser()
    if p.is_dir() and (p / 'card.json').exists():
        return read_dir(p)
    ref = str(ref)
    robot, _, rev = ref.partition('@')
    if '/' in robot:
        known = refs(store)
        sha = known.get(robot) if rev in ('', 'HEAD') else known.get(f'{robot}@{rev}', rev)
        if sha is None:
            raise KeyError(f'no HEAD for {robot!r} in {[str(s) for s in _stores(store)]}')
    else:
        sha = ref
    for root in _stores(store):
        hits = sorted((root / 'cards').glob(sha[:12] + '*')) if (root / 'cards').is_dir() else []
        if hits:
            return read_dir(hits[0])
    raise KeyError(f'plant card {sha!r} not found in {[str(s) for s in _stores(store)]}')
