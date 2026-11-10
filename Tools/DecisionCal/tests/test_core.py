"""Core invariants of the DecisionCal harness (fast: < 1 min total)."""
import ast
import math
from pathlib import Path

import numpy as np
import pytest

import _paths
import law
import library
import mission
import probes
import score
import twin
from plant import B, DT, Plant, PlantSpec, ThrusterState, crn_normal, static_thrust
from plantcard.blocks.nig import NIG
from vendor.thruster_mixer import mix_thrusters
from vendor.velocity_law import Plant as ScalarPlant, VelocityLaw

HERE = Path(__file__).resolve().parents[1]


def _spec(n=1, **kw):
    base = dict(mass=30.0, izz=4.1, a11=1.5, a22=15.0, a66=1.23, d1=[10, 15, 10], d2=[19.4, 41.7, 19.3],
                k_fwd=51.5, k_rev=51.5)
    base.update(kw)
    return PlantSpec.build(n, **base)


# ---- isolation from the autonomy stack ----------------------------------------------------------

def test_no_mhseals_imports():
    for f in sorted(HERE.rglob('*.py')):
        tree = ast.parse(f.read_text())
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            for nm in names:
                assert not nm.startswith(('mhseals', 'crane_hardware', 'loop', 'study_tuning')), f'{f.name}: {nm}'


def test_nlc_vessel_pinned():
    assert _paths.provenance()['nlc_vessel_sha256'] == _paths.NLC_VESSEL_SHA256


# ---- vendored law and mixer parity --------------------------------------------------------------

def test_mix_matches_vendored():
    rng = np.random.default_rng(0)
    e = rng.uniform(-1.5, 1.5, (200, 3))
    ours = law.mix(e)
    ref = np.array([mix_thrusters(*row) for row in e])
    assert np.max(np.abs(ours - ref)) < 1e-12


def test_batched_law_matches_vendored_scalar():
    g = law.GAIN
    cfg = {'characterization': {'velocity_gain': {a: {b: float(g[i, j]) for j, b in enumerate(('surge', 'sway', 'yaw'))}
                                                  for i, a in enumerate(('surge', 'sway', 'yaw'))},
                                'motion_limits': {'velocity': dict(zip(('surge', 'sway', 'yaw'), law.LIMITS))}}}
    rng = np.random.default_rng(1)
    n = 5
    scal = [VelocityLaw(ScalarPlant(cfg)) for _ in range(n)]
    bat = law.BatchedLaw(n)
    t = 0.0
    for k in range(300):
        t += 0.04
        vel = rng.normal(0, 0.5, (n, 3))
        yaw = rng.uniform(-3, 3, n)
        des = rng.normal(0, 0.4, (n, 3)) * (k % 50 > 10)          # include hold-still stretches (drift learning)
        bat.observe(vel, yaw, t)
        eb = bat.step(des, t)
        for i in range(n):
            scal[i].observe(vel[i], yaw[i], t)
            es = np.array(scal[i].step(des[i], t))
            assert np.max(np.abs(es - eb[i])) < 1e-12
            assert np.max(np.abs(scal[i].drift - bat.drift[i])) < 1e-12


# ---- plant ----------------------------------------------------------------------------------------

def test_crn_depends_only_on_key():
    a = crn_normal(np.array([5, 9], np.uint64), 17, 3, 4)
    b = crn_normal(np.array([9], np.uint64), 17, 3, 4)
    assert np.array_equal(a[1], b[0])
    z = crn_normal(np.arange(20000, dtype=np.uint64), 1, 1, 1)
    assert abs(z.mean()) < 0.03 and abs(z.std() - 1) < 0.03


def test_batch_equals_singles():
    spec = _spec(3, dead_time=[0.0, 0.1, 0.2], tau_up=[0, 0.2, 0.3], tau_down=[0, 0.3, 0.2], rev_hold=[0, 0.1, 0.3],
                 odom_latency=0.04, pos_sd=0.05, ou_sigma=0.03, spatial_amp=0.03, wind=[1.0, 0.5], gust_sigma=0.5)
    seeds = np.array([3, 4, 5], np.uint64)
    q = np.array(mix_thrusters(0.7, 0.2, 0.1))
    pb = Plant(spec, seeds, t_max=4)
    singles = [Plant(spec.take([i]), seeds[[i]], t_max=4) for i in range(3)]
    for k in range(150):
        qq = q * (1 if k < 80 else -1)
        pb.step(np.tile(qq, (3, 1)))
        for s in singles:
            s.step(qq[None])
    for i, s in enumerate(singles):
        assert np.max(np.abs(pb.st[i] - s.st[0])) < 1e-12


def test_thruster_reversal_hold_and_static_limit():
    st = ThrusterState((1,), np.array([50.0]), DT, np.array([0.0]), np.array([0.0]), np.array([0.3]), np.array([np.inf]))
    out = [st.step(np.full((1, 4), 40.0))[0, 0] for _ in range(5)]
    assert out[-1] == 40.0                                         # tau = 0: the static map
    rev = [st.step(np.full((1, 4), -30.0))[0, 0] for _ in range(40)]
    zeros = sum(1 for v in rev if v == 0.0)
    assert abs(zeros * DT - 0.3) <= 2 * DT                         # held at zero for rev_hold
    assert rev[-1] == -30.0


def test_twin_family_is_hidden_subfamily():
    """A hidden boat built from twin parameters is the twin plant exactly (metamorphic base)."""
    p = twin.nominal_particles(2)
    p['delay_s'][:] = 0.16
    p['tau_s'][:] = 0.2
    spec = twin.to_spec(p)
    q = np.array(mix_thrusters(0.5, 0, 0.2))
    a = Plant(spec, np.array([1, 2], np.uint64))
    b = Plant(spec.take([0, 1]), np.array([1, 2], np.uint64))
    for _ in range(100):
        a.step(np.tile(q, (2, 1)))
        b.step(np.tile(q, (2, 1)))
    assert np.array_equal(a.st, b.st)


# ---- ID-07: dead time vs odometry latency ---------------------------------------------------------

def test_id07_dead_time_and_latency_give_identical_logs():
    """With arrival-stamped logs, 0.16 s of actuator dead time and 0.16 s of odometry latency
    produce the same evidence, so no fit on such logs can split them."""
    a = _spec(1, dead_time=0.16, odom_latency=0.0, pos_sd=0.05, yaw_sd=0.01, gyro_sd=0.01)
    b = _spec(1, dead_time=0.0, odom_latency=0.16, pos_sd=0.05, yaw_sd=0.01, gyro_sd=0.01)
    ea, _ = probes.generate(a, 7)
    eb, _ = probes.generate(b, 7)
    for sa, sb in zip(ea.segments, eb.segments):
        # compare away from the segment start (the log grid starts 1 s into the run)
        m = sa.t >= 1.0
        assert np.max(np.abs(sa.x[m] - sb.x[m])) < 1e-9
        assert np.array_equal(sa.u, sb.u)


# ---- fitter ---------------------------------------------------------------------------------------

def test_batched_evidence_matches_plantcard_nig():
    rng = np.random.default_rng(2)
    Phi = rng.normal(size=(60, 3))
    Y = rng.normal(size=(4, 60)) * 3 + (Phi @ np.array([30, 10, 20]))[None]
    prior = twin.nig_prior(['m11', 'd1_u', 'd2_u'])
    post = twin.batched_posterior(prior, Phi, Y, {})
    for g in range(4):
        ref = prior.log_evidence(Phi, Y[g])
        assert abs(post['logev'][g] - ref) < 1e-8
        assert np.allclose(post['mu'][g], prior.update(Phi, Y[g]).mu)


@pytest.fixture(scope='module')
def fitted():
    p = twin.nominal_particles(1)
    p.update(m11=np.array([90.]), m22=np.array([120.]), m66=np.array([12.]), delay_s=np.array([0.25]),
             tau_s=np.array([0.2]), rev_hold_s=np.array([0.2]), rev_ratio=np.array([0.75]), deadband_q=np.array([0.04]))
    spec = twin.to_spec(p, sensors=dict(pos_sd=0.0, vel_sd=0.0, yaw_sd=0.0, gyro_sd=0.0))
    ev, raw = probes.generate(spec, 1)
    return ev, raw


def test_joint_fit_recovers_grid_on_exact_states(fitted):
    ev, raw = fitted
    for i, s in enumerate(ev.segments):
        k = np.round((s.t + 1.0) / DT).astype(int) - 1
        s.x = raw['truth'][k, i].copy()
        s.x[:, 2] = np.unwrap(s.x[:, 2])
        s.meta.pop('_cache', None)
        s.meta.pop('gps_t', None)                                  # exact states: no measurement operator
        s.meta.pop('gyro_t', None)
    reg = twin.Regression(ev)
    assert not reg.smoothed
    _, rep = twin.fit(reg, 'joint', 32, 0)
    assert rep['map'] == {'delay_s': 0.25, 'tau_s': 0.2, 'rev_hold_s': 0.2, 'rev_ratio': 0.75, 'deadband_q': 0.04}
    assert abs(rep['block_map']['sway'][0] - 120) < 6 and abs(rep['block_map']['yaw'][0] - 12) < 1.5


@pytest.mark.parametrize('seed', [1, 2, 3])
def test_joint_fit_through_processed_log_recovers_what_is_identifiable(seed):
    """Review B1 falsifier, stated honestly. From the processed log (5 Hz GPS, SG-smoothed over +-1 s,
    default noise) the joint fit recovers the static thrust shape and the reversal hold exactly, but
    delay, lag and inertia trade off: depending on the noise seed the MAP is (delay 0.25, tau 0.2) -- the
    truth -- or (delay 0.16, tau 0.45) with ~17% lower surge/sway inertia and ~35% lower yaw inertia.
    The test pins the identifiable part and bounds the rest; the trade-off is reported in the paper as a
    limit of this measurement set, not hidden."""
    p = twin.nominal_particles(1)
    p.update(m11=np.array([90.]), m22=np.array([120.]), m66=np.array([12.]), delay_s=np.array([0.25]),
             tau_s=np.array([0.2]), rev_hold_s=np.array([0.2]), rev_ratio=np.array([0.75]), deadband_q=np.array([0.04]))
    ev, _ = probes.generate(twin.to_spec(p), seed)                 # default twin sensor noise
    reg = twin.Regression(ev)
    assert reg.smoothed
    _, rep = twin.fit(reg, 'joint', 32, 0)
    m = rep['map']
    assert (m['rev_hold_s'], m['rev_ratio'], m['deadband_q']) == (0.2, 0.75, 0.04)
    assert 0.40 <= m['delay_s'] + m['tau_s'] <= 0.65, m
    for b, true in (('surge', 90.0), ('sway', 120.0)):
        assert abs(rep['block_map'][b][0] - true) / true < 0.25, (b, rep['block_map'][b])
    assert abs(rep['block_map']['yaw'][0] - 12.0) / 12.0 < 0.45


@pytest.mark.parametrize('scale,seed', [(1.0, 6), (2.0, 2), (3.0, 5), (3.0, 1)])
def test_rigid_fit_recovers_inertia_on_heavy_noisy_boats(scale, seed):
    """REVIEW_verifier_wave1 finding 1: recovery must hold across seeds and 1-3x inertia/damping,
    with the twin's default sensor noise (coupling terms refitted at the fitted masses)."""
    p = twin.nominal_particles(1)
    for k in ('m11', 'm22', 'm66', 'd1_u', 'd2_u', 'd1_v', 'd2_v', 'd1_r', 'd2_r'):
        p[k] = p[k] * scale
    ev, _ = probes.generate(twin.to_spec(p), seed)
    part, rep = twin.fit(twin.Regression(ev), 'rigid', 32, 0)
    for k in ('m11', 'm22', 'm66'):
        assert abs(np.median(part[k]) / (twin.NOMINAL[k] * scale) - 1) < 0.15, (k, np.median(part[k]))
    assert max(rep['frac_clipped'].values()) < 0.1


def test_abstain_rule_falls_back_to_default():
    J_h = np.tile(np.linspace(40, 70, 16)[:, None], (1, 6))
    ok = np.tile(np.linspace(40, 70, 16)[:, None], (1, 8))
    allfail = np.full((16, 8), 240.0)
    allfail[5] = 230.0                                             # an arbitrary tie-break
    sc = score.plant_scores(J_h, {'nominal': ok, 'bad': allfail})
    assert sc['arms']['bad']['sel'] == 5 and sc['arms']['bad']['abstained']
    assert sc['arms']['bad']['sel_abst'] == 0 and sc['arms']['bad']['regret_abst'] == 0.0
    assert not sc['arms']['nominal']['abstained']


def test_equal_budget_across_arms(fitted):
    ev, _ = fitted
    fits, reg = twin.fit_all(ev, 16, 3)
    for arm, (p, rep) in fits.items():
        assert all(len(v) == 16 for v in p.values()), arm
        assert set(p) == set(twin.NOMINAL), arm
    assert reg.evidence_sha == ev.provenance['sha256']


# ---- library and mission --------------------------------------------------------------------------

def test_library_balanced_and_distinct():
    F = library.factor_matrix()
    assert len({c.sha() for c in library.LIBRARY}) == 16
    for i in range(F.shape[1]):
        for j in range(i + 1, F.shape[1]):
            pairs = {}
            for a, b in zip(F[:, i], F[:, j]):
                pairs[(a, b)] = pairs.get((a, b), 0) + 1
            assert len(set(pairs.values())) == 1, (i, j, pairs)   # every level pair equally often


def test_library_outcomes_distinct_on_nominal():
    spec = twin.to_spec(twin.nominal_particles(16))
    m, _ = mission.run(spec, np.arange(16), np.arange(16))
    assert len(np.unique(np.round(m['J'], 3))) == 16
    assert m['docked'].all()


def test_broad_twin_selects_several_members():
    """Guard against the degenerate roster (E1 v1): the argmin over broad-prior particles is not one
    member for all of them."""
    p, _ = twin.fit(None, 'broad', 12, 5)
    spec = twin.to_spec(p).repeat(16)
    m, _ = mission.run(spec, np.tile(np.arange(16), 12), np.repeat(np.arange(12), 16))
    J = m['J'].reshape(12, 16)
    assert len(set(J.argmin(1))) >= 3


def test_mission_deterministic():
    spec = twin.to_spec(twin.nominal_particles(4)).with_(pos_sd=np.full(4, 0.05))
    a, _ = mission.run(spec, [0, 5, 8, 12], np.arange(4), t_max=30)
    b, _ = mission.run(spec, [0, 5, 8, 12], np.arange(4), t_max=30)
    for k in a:
        assert np.array_equal(np.nan_to_num(a[k]), np.nan_to_num(b[k]))


# ---- scoring --------------------------------------------------------------------------------------

def test_selection_is_nan_safe():
    J = np.array([[np.nan, np.nan], [50.0, 52.0], [60.0, 61.0]])
    assert score.selection(J) == 1


def test_holm_and_sign_flip():
    assert score.sign_flip_p(np.zeros(8)) == 1.0
    assert score.sign_flip_p(np.ones(10)) < 0.01
    adj = score.holm({'a': 0.01, 'b': 0.02, 'c': 0.5})
    assert abs(adj['a'] - 0.03) < 1e-12 and abs(adj['b'] - 0.04) < 1e-12 and adj['c'] == 0.5


def test_regret_zero_when_twin_equals_hidden_ranking():
    rng = np.random.default_rng(0)
    base = np.linspace(40, 80, 16)
    J_h = base[:, None] + rng.normal(0, 0.5, (16, 6))
    J_t = base[:, None] + rng.normal(0, 0.5, (16, 8))
    sc = score.plant_scores(J_h, {'nominal': J_t})
    assert sc['arms']['nominal']['sel'] == 0 and sc['arms']['nominal']['within_band']
