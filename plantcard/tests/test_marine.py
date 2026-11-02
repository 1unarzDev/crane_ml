"""Marine pack: Fossen3 is the characterization ODE; the Unity view round-trips the profile; a
synthetic boat is recovered; the card stores, loads and runs in the fast tier."""
import json
from pathlib import Path

import numpy as np
import pytest

import plant as charz                                    # Tools/Characterization/plant.py
import plantcard
from plantcard import card as C
from plantcard import validate as V
from plantcard.family import Evidence, ParamBatch, Segment
from plantcard.marine import seed, views
from plantcard.marine.fossen3 import Fossen3, profile_config, profile_params

PROFILE = json.loads(seed.CHARACTERIZED.read_text())


def _family():
    return Fossen3(profile_config(PROFILE))


def test_rhs_is_the_characterization_ode():
    fam = _family()
    p = ParamBatch({k: np.array([v]) for k, v in profile_params(PROFILE).items()})
    rng = np.random.default_rng(0)
    for _ in range(25):
        st = rng.normal(size=6) * [5, 5, 3, 1, 0.6, 0.5]
        tau = rng.normal(size=3) * [60, 40, 10]
        cur = tuple(rng.normal(size=2) * 0.3)
        ref = charz.ode_rhs(PROFILE, st, tau, cur)
        got = fam.rhs_wrench(st[None], tau[None], p, {'cx': cur[0], 'cy': cur[1]})[0]
        assert np.allclose(got, ref, rtol=1e-12, atol=1e-12)


def test_thrust_is_the_profile_thruster():
    fam = _family()
    for pwm in ([1820, 1180, 1500, 1660], [1300, 1700, 1900, 1100]):
        q = (np.array(pwm, float) - 1500) / 400
        ref = np.zeros(3)
        for t, pw in zip(PROFILE['thrusters'], pwm):
            ref += charz.ThrusterModel(t, 0.02).step(pw)
        got = fam.thrust(q[None], np.array([1.0]), np.array([0.0]))[0]
        assert np.allclose(got, ref, rtol=1e-12)


def test_unity_view_round_trips_the_profile():
    c0 = seed.prior_card()
    prof = views.unity_profile(c0)
    charz.validate_profile(prof)
    for k in ('mass', 'izz', 'addedMass', 'surge', 'sway', 'yaw', 'envelope', 'fixedDeltaTime'):
        assert prof[k] == pytest.approx(PROFILE[k]), k
    for a, b in zip(prof['thrusters'], PROFILE['thrusters']):
        assert a['kForward'] == pytest.approx(b['kForward']) and a['kReverse'] == pytest.approx(b['kReverse'])
    assert prof['provenance']['kind'] == 'plantcard'


def test_nlc_view_shapes():
    c0 = seed.prior_card()
    one, many = views.nlc_params(c0), views.nlc_params(c0, n=16)
    assert isinstance(one['mass'], float) and len(one['d1']) == 3
    assert many['d1'][0].shape == (16,) and many['dead_time'].shape == (16,)


def _synthetic(fam, truth, n_seg=8, seed_=0, noise=0.02):
    rng = np.random.default_rng(seed_)
    p = ParamBatch({k: np.array([v]) for k, v in truth.items()})
    patterns = [[-1, -1, 1, 1], [1, 1, -1, -1], [1, -1, -1, 1], [-1, 1, 1, -1], [1, -1, 1, -1],
                [-1, 1, -1, 1], [1, 0, 0, 0], [0, 0, 1, 0]]
    segs = []
    for i in range(n_seg):
        tf = np.arange(0, 16, 0.01)                       # simulate fine, sample at 10 Hz
        q = np.array(patterns[i % len(patterns)], float) * rng.choice([0.4, 0.8])
        uf = np.zeros((len(tf), 4))
        uf[(tf >= 4) & (tf < 7)] = q
        x = fam.rollout(np.zeros(6), tf, uf, p, {'cx': np.zeros(1), 'cy': np.zeros(1)})[0][::10]
        t, u = tf[::10], uf[::10]
        x = x + rng.normal(size=x.shape) * [noise, noise, 0.002, noise, noise, 0.005]
        segs.append(Segment(session=f's{i // 2}', tier='synthetic', t=t, x=x, u=u, name=f'seg{i}',
                            meta={'fit_window': (3.0, 15.0)}, t_event=4.0))
    return Evidence(segs, {'kind': 'synthetic'})


def test_recovers_a_synthetic_boat():
    c0 = seed.prior_card()
    fam = c0.family
    truth = profile_params(PROFILE)
    truth.update({'m11': 60.0, 'd1_u': 5.0, 'd2_u': 80.0, 'm22': 55.0, 'd2_v': 120.0, 'm66': 3.0, 'd2_r': 9.0,
                  'delay_s': 0.35, 'rev_ratio': 0.75})
    ev = _synthetic(fam, truth)
    doc, parts, support, rep = plantcard.propose(c0, ev, n_particles=256)
    assert doc['nominal']['delay_s'] == pytest.approx(0.35, abs=0.051)
    assert doc['nominal']['rev_ratio'] == pytest.approx(0.75, abs=0.051)
    for k, tol in (('m11', 0.15), ('d2_u', 0.15), ('m66', 0.15), ('d2_r', 0.2)):
        assert doc['nominal'][k] == pytest.approx(truth[k], rel=tol), k
    card = C.Card(doc, parts, support)
    rows = []
    for s in ev.segments:
        rows += V.score_segment(card, s, horizons=(5.0,), n=64)
    assert np.median([r['e_real'] for r in rows]) < 0.5


def test_store_round_trip_and_fastsim_plant(tmp_path):
    doc, parts = seed.prior_doc(n_particles=64)
    saved = C.save(doc, parts, None, tmp_path)
    C.set_ref(saved.robot, saved.sha, tmp_path)
    again = plantcard.load('roboboat/blastoise@HEAD', store=tmp_path)
    assert again.sha == saved.sha and again.particles.n == 64
    (Path(again.path) / 'particles.npz').write_bytes(b'tampered')
    with pytest.raises(Exception):
        C.read_dir(again.path)
    fastsim = pytest.importorskip('fastsim.plant')
    from plantcard.marine.fastsim_plant import CardPlant
    lim = fastsim.Limits(vmax=(1.5, 0.5, 0.8), vmin=(-0.5, -0.5, -0.8), accel=(0.5, 0.5, 0.5),
                         decel=(-0.5, -0.5, -0.5), goal_tol=(0.5, 0.2), precise_tol=(0.2, 0.1), rotate_above=1.0,
                         rotate_wz=0.5, progress_radius=0.3, progress_s=10.0, pass_radius=1.0,
                         footprint=((0.5, 0.35), (-0.5, -0.35)))
    plant = CardPlant(lim, card=seed.prior_card(), seed=3)
    for _ in range(50):
        plant.step((1.0, 0.0, 0.0), 0.1)
    assert 0.3 < plant.vel[0] < 2.5 and plant.pose[0] > 1.0
    assert plant.params()['particle'] is not None


def test_unity_export_runs_like_the_card():
    """The exported profile in the Unity twin (ProfilePlant: implicit damping, tick-rounded dead
    time) follows the card's own RK4 rollout within 0.3 m over an 8 s pulse-and-coast."""
    c1 = plantcard.load('roboboat/blastoise@candidate')
    prof = c1.view('unity')
    for th in prof['thrusters']:
        th['deadbandUs'] = 0.0                     # Unity's hard deadband is a known structure gap
    pwm = [1820, 1820, 1180, 1180]
    rows = np.array(charz.ProfilePlant(prof, 0.02).run_pwm(lambda k, t: pwm if t < 3 - 1e-9 else [1500] * 4, 400))
    t = np.arange(0, 8.0001, 0.02)
    U = np.where((t < 3)[:, None], (np.array(pwm) - 1500) / 400, 0.0)
    p = c1.nominal()
    p['deadband_q'] = np.array([0.0])
    tr = c1.family.rollout(np.zeros(6), t, U, p, None)[0]
    i = np.searchsorted(t, rows[:, 0] - 1e-9)
    assert np.max(np.hypot(rows[:, 1] - tr[i, 0], rows[:, 2] - tr[i, 1])) < 0.3
