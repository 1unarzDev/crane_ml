"""Prior card c0 from a characterization profile (``Tools/Characterization/profiles/*.json``).

The profile's numbers become NIG priors centred on its effective values with the NLC study's
prior strength (rel_std 0.5, noise_std 2.0: ``nlc/calib/loop.py``), a grid over the lumped
actuation delay and the reverse/forward thrust ratio, and a disturbance prior. Every block is
``prior_dominated`` and the gauge is ``effective`` (no bollard pull).
"""

import datetime as _dt
import hashlib
import json
from pathlib import Path

import numpy as np

from plantcard.blocks import NIG, Grid
from plantcard.card import Card
from plantcard.marine.fossen3 import Fossen3, profile_config, profile_params
from plantcard.update import draw_particles

CRANE_ROOT = Path(__file__).resolve().parents[2]
CHARACTERIZED = CRANE_ROOT / 'Tools' / 'Characterization' / 'profiles' / 'characterized.json'
DELAY_GRID = [round(0.15 + 0.05 * i, 3) for i in range(9)]      # 0.15 .. 0.55 s
DB_GRID = [0.0, 0.025, 0.05, 0.075, 0.1, 0.125]                 # thruster deadband in q (25 us = 0.0625)
REV_GRID = [round(0.5 + 0.05 * i, 3) for i in range(11)]       # 0.50 .. 1.00
REL_STD, NOISE_STD = 0.5, 2.0


def prior_doc(profile_path=CHARACTERIZED, *, robot, n_particles=512, seed=7):
    profile_path = Path(profile_path)
    raw = profile_path.read_bytes()
    profile = json.loads(raw)
    config = profile_config(profile)
    fam = Fossen3(config)
    p = profile_params(profile)
    blocks = {}
    for b, names in fam.blocks.items():
        nig = NIG.prior([p[k] for k in names], REL_STD, NOISE_STD)
        blocks[b] = {'kind': 'nig', 'params': names, 'nig': nig.to_dict(), 'forget': 1.0,
                     'mean': [p[k] for k in names], 'sd': [float(v) for v in nig.std()],
                     'noise_std': NOISE_STD, 'n_eff': 0.0, 'status': 'prior',
                     'procedure': f'profile {profile_path.name} (NLC prior strength)',
                     'flags': ['prior_dominated'],
                     'identifiability': {'status': 'COMBINATION_ONLY', 'with': ['gauge']},
                     'bounds': {s.name: list(s.bounds) for s in fam.params if s.name in names},
                     'sources': [{'kind': 'profile', 'ref': profile_path.name,
                                  'sha256': hashlib.sha256(raw).hexdigest()}],
                     'representable_in': ['fast', 'unity', 'nlc']}
    grid = Grid.uniform({'delay_s': DELAY_GRID, 'rev_ratio': REV_GRID, 'deadband_q': DB_GRID})
    blocks['act'] = {'kind': 'grid', 'params': ['delay_s', 'rev_ratio', 'deadband_q'], 'grid': grid.to_dict(),
                     'thrust_table': 'k sign(q) e^2, e = (|q| - deadband_q)/(1 - deadband_q); (k_fwd + k_rev)/2 = k_ref; k_rev = rev_ratio k_fwd',
                     'from_evidence_of': list(fam.blocks), 'status': 'prior', 'flags': ['lumped', 'prior_dominated'],
                     'representable_in': ['fast', 'unity', 'nlc']}
    blocks['dist'] = {'kind': 'constant_bias', 'current_sd': 0.1, 'force_sd': [3.0, 3.0, 1.0], 'status': 'prior',
                      'representable_in': ['fast', 'nlc']}
    # the prior's own particles: grid prior x NIG prior (exact joint)
    posts = {idx: {b: NIG.from_dict(blocks[b]['nig']) for b in fam.blocks} for idx, _ in grid.points()}
    nominal = dict(p)
    nominal['delay_s'] = float(np.mean(DELAY_GRID))
    nominal['rev_ratio'] = p['rev_ratio']
    nominal['deadband_q'] = p['deadband_q']
    doc = {
        'schema': 'crane.plantcard/1', 'robot': robot, 'version': 0, 'parent': None, 'status': 'prior',
        'family': {'ref': 'plantcard.marine.fossen3:Fossen3', 'version': Fossen3.version,
                   'frames': 'world ENU, body FLU at CG', 'config': config},
        'rig': {'profile': profile_path.name, 'epoch': 'hw-2026-10-18',
                'mass_kg': {'value': profile['mass'], 'status': 'prior', 'reason': 'no weigh-in'}},
        'gauge': 'effective', 'blocks': blocks, 'nominal': nominal,
        'particles': {'file': 'particles.npz', 'n': n_particles, 'seed': seed, 'joint_over': ['dyn.*', 'act.*']},
        'envelope': None, 'validation': [], 'views': {},
        'provenance': {'evidence': [], 'priors': f'{profile_path.name} sha256 {hashlib.sha256(raw).hexdigest()[:12]}',
                       'by': 'plantcard.marine.seed', 'created': _dt.date.today().isoformat(), 'accepted_by': None,
                       'note': 'c0: the characterization profile wrapped as a card (P0); no field fit'},
    }
    parts, _ = draw_particles(fam, posts, grid, n_particles, seed)
    return doc, dict(parts)


def prior_card(profile_path=CHARACTERIZED, **kw):
    doc, parts = prior_doc(profile_path, **kw)
    return Card(doc, parts)
