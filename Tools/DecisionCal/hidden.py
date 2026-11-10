"""Hidden boats: the evaluation plants, outside the twin family.

Every hidden boat differs from any twin in ways the twin cannot represent: per-thruster forward
and reverse gains, a thrust-curve exponent gamma != 2, per-thruster deadbands, spin-up and spin-down
lags that differ, a slew limit, an OU + spatial current and gusty wind. The rigid body and sensors
share the twin's model class (stated limitation; the Unity check is the independent-implementation
test).

Strata (25 boats each in the full study), FROZEN 2026-11-09 before any scored run (the timing
stratum was narrowed from dead 0.12-0.4 / latency 0.05-0.25 s after an unscored shakedown in which
58% of its missions never docked):

* steady    -- transients and timing small; the steady thrust/drag differs from the prior card.
* transient -- slow spin-up/down, long reversal holds, heavier effective inertia.
* timing    -- long actuator dead time and odometry latency.
* mixed     -- moderate transient + timing, stronger current.

Common to all: thrust scale s ~ U[0.35, 1.0] (the 10-18 field fit put the effective inertia at ~3x
the prior card, i.e. thrust ~1/3 of the profile's), per-thruster factors U[0.85, 1.15], reverse
ratio per thruster U[0.6, 0.95], gamma U[1.7, 2.2], deadband U[0, 0.06], damping x U[0.7, 1.6] per
axis, added mass x U[0.7, 1.4].
"""

import hashlib
import json

import numpy as np

import _paths  # noqa: F401
from plant import N_THR, PlantSpec
from twin import CONFIG, NOMINAL, K_REF

STRATA = ('steady', 'transient', 'timing', 'mixed')
FROZEN = '2026-11-09'

COMMON = {'thrust_scale': (0.35, 1.0), 'thr_factor': (0.85, 1.15), 'rev_ratio': (0.6, 0.95), 'gamma': (1.7, 2.2),
          'deadband': (0.0, 0.06), 'damp': (0.7, 1.6), 'added': (0.7, 1.4), 'slew_frac': (2.0, 8.0),
          'current_speed': (0.0, 0.15), 'ou_sigma': (0.0, 0.04), 'spatial_amp': (0.0, 0.04), 'wind': (0.0, 3.0),
          'gust_sigma': (0.0, 0.8), 'pos_sd': (0.03, 0.12), 'vel_sd': (0.02, 0.05), 'yaw_sd': (0.005, 0.02),
          'gyro_sd': (0.005, 0.02)}
STRATUM = {
    'steady':    {'dead_time': (0.0, 0.05), 'odom_latency': (0.0, 0.04), 'tau_up': (0.02, 0.08),
                  'down_ratio': (0.8, 1.5), 'rev_hold': (0.0, 0.05), 'inertia': (1.0, 1.0)},
    'transient': {'dead_time': (0.0, 0.05), 'odom_latency': (0.0, 0.04), 'tau_up': (0.15, 0.5),
                  'down_ratio': (0.8, 2.0), 'rev_hold': (0.2, 0.6), 'inertia': (1.0, 1.6)},
    'timing':    {'dead_time': (0.1, 0.3), 'odom_latency': (0.05, 0.15), 'tau_up': (0.02, 0.08),
                  'down_ratio': (0.8, 1.5), 'rev_hold': (0.0, 0.05), 'inertia': (1.0, 1.0)},
    'mixed':     {'dead_time': (0.05, 0.2), 'odom_latency': (0.03, 0.12), 'tau_up': (0.08, 0.3),
                  'down_ratio': (0.8, 1.8), 'rev_hold': (0.05, 0.3), 'inertia': (1.0, 1.3),
                  'current_speed': (0.1, 0.3)},
}


def ranges_sha():
    return hashlib.sha256(json.dumps({'common': COMMON, 'strata': STRATUM}, sort_keys=True).encode()).hexdigest()


def sample_hidden(n, seed, stratum):
    """PlantSpec of n hidden boats of one stratum, and their latent draws (for the record)."""
    if stratum not in STRATUM:
        raise ValueError(f'unknown stratum {stratum!r}')
    rng = np.random.default_rng([seed, STRATA.index(stratum)])
    R = {**COMMON, **STRATUM[stratum]}
    U = lambda k, size=None: rng.uniform(*R[k], n if size is None else (n, size))
    s = U('thrust_scale')
    k_base = K_REF * s
    fac = U('thr_factor', N_THR)
    rev = U('rev_ratio', N_THR)
    k_fwd = k_base[:, None] * fac * 2.0 / (1.0 + rev)
    k_rev = rev * k_fwd
    m_rb, izz = CONFIG['m_rb'], CONFIG['izz_rb']
    a = {k: NOMINAL[m] - (m_rb if m != 'm66' else izz) for k, m in (('a11', 'm11'), ('a22', 'm22'), ('a66', 'm66'))}
    inertia = U('inertia')
    added = U('added', 3)
    d1n = np.array([NOMINAL['d1_u'], NOMINAL['d1_v'], NOMINAL['d1_r']])
    d2n = np.array([NOMINAL['d2_u'], NOMINAL['d2_v'], NOMINAL['d2_r']])
    damp = U('damp', 3)
    tau_up = U('tau_up')
    ang = rng.uniform(0, 2 * np.pi, n)
    cs = U('current_speed')
    wang = rng.uniform(0, 2 * np.pi, n)
    ws = U('wind')
    spec = PlantSpec.build(
        n, mass=m_rb * inertia, izz=izz * inertia, a11=a['a11'] * added[:, 0] * inertia,
        a22=a['a22'] * added[:, 1] * inertia, a26=CONFIG['a26'], a66=a['a66'] * added[:, 2] * inertia,
        d1=d1n * damp, d2=d2n * damp, k_fwd=k_fwd, k_rev=k_rev, gamma=U('gamma'), deadband=U('deadband', N_THR),
        dead_time=U('dead_time'), tau_up=tau_up, tau_down=tau_up * U('down_ratio'), rev_hold=U('rev_hold'),
        slew=k_base * U('slew_frac'), odom_latency=U('odom_latency'), pos_sd=U('pos_sd'), vel_sd=U('vel_sd'),
        yaw_sd=U('yaw_sd'), gyro_sd=U('gyro_sd'), current=np.stack([cs * np.cos(ang), cs * np.sin(ang)], -1),
        ou_sigma=U('ou_sigma'), spatial_amp=U('spatial_amp'),
        wind=np.stack([ws * np.cos(wang), ws * np.sin(wang)], -1), gust_sigma=U('gust_sigma'))
    return spec


def sample_all(n_per, seed):
    """(PlantSpec of len(STRATA) * n_per boats, stratum label per boat, plant ids)."""
    specs, labels, ids = [], [], []
    for st in STRATA:
        specs.append(sample_hidden(n_per, seed, st))
        labels += [st] * n_per
        ids += [f'{st}-{seed}-{i:03d}' for i in range(n_per)]
    return PlantSpec.concat(specs), labels, ids
