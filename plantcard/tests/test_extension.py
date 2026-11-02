"""Extension proof: a planar quadrotor is fitted, validated and stored by the same core."""
import numpy as np

import plantcard
from plantcard import validate as V
from plantcard.blocks import NIG, Grid
from plantcard.card import Card
from plantcard.families.quad2d import Quad2D
from plantcard.family import Evidence, ParamBatch, Segment
from plantcard.update import draw_particles

TRUTH = {'m_x': 1.2, 'dx': 0.3, 'm_z': 1.2, 'dz': 0.5, 'W': 11.77, 'I': 0.02, 'dth': 0.01, 'delay_s': 0.05}


def _prior():
    fam = Quad2D()
    blocks = {
        'dyn.x': {'kind': 'nig', 'nig': NIG.prior([1.0, 0.2], 0.5, 0.5).to_dict()},
        'dyn.z': {'kind': 'nig', 'nig': NIG.prior([1.0, 0.2, 10.0], 0.5, 0.5).to_dict()},
        'dyn.th': {'kind': 'nig', 'nig': NIG.prior([0.03, 0.02], 0.5, 0.05).to_dict()},
        'act': {'kind': 'grid', 'grid': Grid.uniform({'delay_s': [0.0, 0.05, 0.1, 0.15]}).to_dict()},
        'dist': {'kind': 'constant_bias', 'force_sd': [0.05, 0.05, 0.0]},
    }
    posts = {idx: {b: NIG.from_dict(blocks[b]['nig']) for b in fam.blocks}
             for idx, _ in Grid.from_dict(blocks['act']['grid']).points()}
    parts, _ = draw_particles(fam, posts, Grid.from_dict(blocks['act']['grid']), 128, 0)
    nominal = {'m_x': 1.0, 'dx': 0.2, 'm_z': 1.0, 'dz': 0.2, 'W': 10.0, 'I': 0.03, 'dth': 0.02, 'delay_s': 0.05}
    doc = {'schema': 'crane.plantcard/1', 'robot': 'toy/quad2d', 'status': 'prior',
           'family': {'ref': 'plantcard.families.quad2d:Quad2D', 'config': fam.config}, 'gauge': 'absolute',
           'blocks': blocks, 'nominal': nominal, 'particles': {'n': 128}, 'provenance': {}}
    return Card(doc, dict(parts))


def _evidence(n=6, seed=0):
    fam = Quad2D()
    rng = np.random.default_rng(seed)
    p = ParamBatch({k: np.array([v]) for k, v in TRUTH.items()})
    hover = TRUTH['W'] / (2 * fam.k_ref)
    segs = []
    for i in range(n):
        t = np.arange(0, 6, 0.02)
        u = np.full((len(t), 2), hover)
        a, b = rng.uniform(-0.05, 0.05, 2)
        u[(t > 1) & (t < 1.3)] += [a, -a]
        u[(t > 2) & (t < 3)] += [b, b]
        x = fam.rollout(np.zeros(6), t, u, p)[0] + rng.normal(size=(len(t), 6)) * 0.002
        segs.append(Segment(session=f's{i}', tier='synthetic', t=t, x=x, u=u, name=f'q{i}', t_event=1.0))
    return Evidence(segs, {'kind': 'synthetic'})


def test_quad2d_fit_validate_store(tmp_path):
    c0 = _prior()
    ev = _evidence()
    doc, parts, support, _ = plantcard.propose(c0, ev, n_particles=128)
    assert abs(doc['nominal']['m_z'] - TRUTH['m_z']) / TRUTH['m_z'] < 0.1
    assert abs(doc['nominal']['W'] - TRUTH['W']) / TRUTH['W'] < 0.05
    child_rows, parent_rows = V.cross_validate(c0, ev, horizons=(2.0,), n=32, n_particles=128, log=lambda *_: None)
    s = V.summarize(child_rows, 0.1, 2.0)
    assert s['n_units'] == 6
    assert s['mean_path_err_max_median_m'] < V.summarize(parent_rows, 0.1, 2.0)['mean_path_err_max_median_m']
    saved = plantcard.save(doc, parts, support, tmp_path)
    assert plantcard.load(saved.sha[:12], store=tmp_path).family.name == 'quad2d'
