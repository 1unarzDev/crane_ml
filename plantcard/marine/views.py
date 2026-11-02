"""Derived views of a Fossen3 card. Every view is stamped with the card sha so a run can name the
plant it ran.

* ``unity``  : a ``profiles/*.json`` profile (Unity ``MarinePlanarModel``; ``Tools/Characterization``
               ``ProfilePlant`` is its exact Python twin), nominal or one particle;
* ``nlc``    : keyword arguments for ``nlc.sim.vessel.VesselParams`` (scalars, or per-env arrays
               of particles: the study's batched hidden parameters);
* ``fastsim``: a fast-tier vehicle spec using ``plantcard.marine.fastsim_plant:CardPlant``;
* ``control``: feedforward D(nu) nu and steady speed curves in effort units (never auto-applied).

Gauge: the card is effective per axis (thrust-normalised). Views fix the thrust at the card's
k_ref and put each axis's effective inertia and damping into the profile; the rigid mass stays the
configured one and the rigid yaw inertia is lowered to the effective total when needed, so added
masses stay >= 0. ``provenance.gauge`` says so.
"""

import copy
import json
from pathlib import Path

import numpy as np

from plantcard.marine.seed import CHARACTERIZED

STRUCTURE_GAP = ['thruster_lag_lumped_into_dead_time', 'deadband_rescaled_thrust_curve (Unity: hard deadband, q^2)',
                 'sway/surge-thrust-induced yaw (thruster-hull interaction, even in thrust)', 'current_and_force_bias']


def _params(card, particle=None):
    if particle is None:
        return {k: float(v) for k, v in card.doc['nominal'].items()}
    return card.particles.row(int(particle))


def _thrust_split(card, p):
    k_ref = card.family.k_ref
    rho = p['rev_ratio']
    kf = 2.0 * k_ref / (1.0 + rho)
    return kf, rho * kf


def unity_profile(card, particle=None, base=CHARACTERIZED, fixed_dt=None):
    """A characterization profile dict for MarinePlanarModel from the card's nominal (or particle k)."""
    prof = copy.deepcopy(json.loads(Path(base).read_text()))
    p = _params(card, particle)
    cfg = card.family.config
    m_rb = min(cfg['m_rb'], p['m11'], p['m22'])
    izz = min(cfg['izz_rb'], p['m66'])
    prof['name'] = f'PlantCard-{card.sha[:12]}' + ('' if particle is None else f'-p{particle}')
    if fixed_dt:
        prof['fixedDeltaTime'] = fixed_dt
    prof['mass'], prof['izz'] = m_rb, izz
    prof['addedMass'].update({'a11': p['m11'] - m_rb, 'a22': p['m22'] - m_rb, 'a26': cfg.get('a26', 0.0),
                              'a66': p['m66'] - izz})
    for ax, k in (('surge', 'u'), ('sway', 'v'), ('yaw', 'r')):
        prof[ax] = {'d1Pos': p['d1_' + k], 'd1Neg': p['d1_' + k], 'd2Pos': p['d2_' + k], 'd2Neg': p['d2_' + k]}
    kf, kr = _thrust_split(card, p)
    names = [t['name'] for t in cfg['thrusters']]
    for t in prof['thrusters']:
        if t['name'] not in names:
            raise ValueError(f'base profile thruster {t["name"]} is not in the card family')
        t.update({'kForward': kf, 'kReverse': kr, 'deadTime': p['delay_s'], 'tauT': 0.0,
                  'deadbandUs': p.get('deadband_q', 0.0) * 400.0})
    prof['provenance'] = {
        'kind': 'plantcard', 'card_sha': card.sha, 'card_status': card.status, 'robot': card.robot,
        'particle': particle, 'gauge': card.doc['gauge'] + ' (thrust fixed at k_ref; effective per-axis inertia)',
        'source': f'plantcard view unity of {card.sha[:12]}, base {Path(base).name}',
        'structure_gap': STRUCTURE_GAP,
        'summary': f'card status {card.status}; '
                   + '; '.join(f'{v["event"]} cov {v["covered_frac"]:.2f}' for v in card.doc.get('validation', [])
                               if v.get('tier') == 'real'),
        'parameters': [{'name': b, 'status': card.blocks[b].get('status', 'prior'),
                        'reason': ', '.join(card.blocks[b].get('flags', []))}
                       for b in ('dyn.surge', 'dyn.sway', 'dyn.yaw', 'act')],
    }
    return prof


def nlc_params(card, n=None, seed=0):
    """``VesselParams`` keyword arguments: the nominal (n=None) or n particles as per-env arrays."""
    cfg = card.family.config
    if n is None:
        P = {k: np.array([v]) for k, v in _params(card).items()}
    else:
        P = {k: np.asarray(v) for k, v in card.sample(n, seed).items()}
    m_rb = np.minimum(np.minimum(cfg['m_rb'], P['m11']), P['m22'])
    izz = np.minimum(cfg['izz_rb'], P['m66'])
    kf, kr = _thrust_split(card, _params(card))
    one = (lambda a: float(a[0])) if n is None else (lambda a: a)
    return {
        'mass': one(m_rb), 'izz': one(izz), 'a11': one(P['m11'] - m_rb), 'a22': one(P['m22'] - m_rb),
        'a26': cfg.get('a26', 0.0), 'a66': one(P['m66'] - izz),
        'd1': tuple(one(P['d1_' + k]) for k in 'uvr'), 'd2': tuple(one(P['d2_' + k]) for k in 'uvr'),
        'thr_xy': tuple((t['x'], t['y']) for t in cfg['thrusters']),
        'thr_alpha_deg': tuple(t['alphaDeg'] for t in cfg['thrusters']),
        'k_fwd': kf, 'k_rev': kr, 'dead_time': one(P['delay_s']), 'tau_thr': 1e-3, 'thrust_scale': 1.0,
        'source': f'plantcard {card.sha[:12]} ({card.status}{"" if n is None else f", {n} particles"})',
    }


def control(card, speeds=None):
    """Feedforward and steady speed curves (effort units = fraction of k_ref per axis column)."""
    p = _params(card)
    out = {'card_sha': card.sha, 'feedforward': {}, 'speed_curves': {}}
    B = card.family.B
    k_ref = card.family.k_ref
    full = {'u': np.abs(B[0]).sum() * k_ref, 'v': np.abs(B[1]).sum() * k_ref, 'r': np.abs(B[2]).sum() * k_ref}
    for k, ax in zip('uvr', ('surge', 'sway', 'yaw')):
        d1, d2 = p['d1_' + k], p['d2_' + k]
        out['feedforward'][ax] = {'d1': d1, 'd2': d2, 'form': 'tau = (d1 + d2 |nu|) nu'}
        grid = np.linspace(0.1, 1.0, 10) if speeds is None else np.asarray(speeds)
        tau = grid * full[k]                     # effort -> force at q|q| = effort (a linearised map)
        v = np.where(d2 > 0, (-d1 + np.sqrt(d1 * d1 + 4 * d2 * tau)) / (2 * d2 + 1e-12), tau / max(d1, 1e-9))
        out['speed_curves'][ax] = {'effort': grid.tolist(), 'steady': v.tolist(), 'full_force': float(full[k])}
    return out


def fastsim_spec(card, gain=1.5, particle=None):
    """A fastsim vehicle spec (``vehicle:`` in a domain YAML) that runs this card."""
    return {'ref': 'plantcard.marine.fastsim_plant:CardPlant', 'gain': gain,
            'card': str(card.path) if card.path else card.sha, 'particle': particle}
