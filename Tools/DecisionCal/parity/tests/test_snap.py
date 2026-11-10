"""The thrust-snap flag (plant.SNAP_FRAC): default behaviour, the artifact it creates, and the reversal
hold with the snap removed.  nice -n 19 python3 -I -m pytest Tools/DecisionCal/parity/tests -q"""
import sys
from pathlib import Path

DC = Path(__file__).resolve().parents[2]
if str(DC) not in sys.path:
    sys.path.insert(0, str(DC))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

import plant  # noqa: E402
from plant import DT, Plant, PlantSpec, ThrusterState, static_thrust  # noqa: E402


@pytest.fixture
def snap():
    """Restore SNAP_FRAC after each test."""
    old = plant.SNAP_FRAC
    yield
    plant.SNAP_FRAC = old


def _thr(tau=0.3, hold=0.3, k=50.0):
    return ThrusterState((1,), np.array([k]), DT, np.array([tau]), np.array([tau]), np.array([hold]), np.array([np.inf]))


def test_default_is_1e3(snap):
    assert plant.SNAP_FRAC == 1e-3
    assert np.allclose(_thr().eps, 1e-3 * 50.0)


def test_snap_blocks_spin_up_from_rest(snap):
    """q = 0.12 with tau = 0.3 s: a_up f* = (1 - e^(-0.02/0.3)) 50 0.12^2 = 0.046 N < eps = 0.05 N, so with the
    snap the thruster never leaves zero; without it the thrust builds to f*."""
    f = static_thrust(np.full((1, 4), 0.12), 50.0, 50.0, np.array([2.0]), 0.0)
    a = 1 - np.exp(-DT / 0.3)
    assert a * f[0, 0] < 1e-3 * 50.0                     # the condition the artifact needs
    plant.SNAP_FRAC = 1e-3
    s = _thr()
    on = [s.step(f)[0, 0] for _ in range(250)]
    assert max(on) == 0.0
    plant.SNAP_FRAC = 0.0
    s = _thr()
    off = [s.step(f)[0, 0] for _ in range(250)]
    assert off[-1] == pytest.approx(f[0, 0], rel=1e-6)


def test_snap_on_full_plant_q012(snap):
    """The same artifact on a whole plant: a constant q = 0.12 surge command leaves the boat at rest."""
    spec = PlantSpec.build(1, mass=30.0, izz=4.1, a11=1.5, a22=15.0, a66=1.23, d1=[10, 15, 10], d2=[19.4, 41.7, 19.3],
                           k_fwd=50.0, k_rev=50.0, tau_up=0.3, tau_down=0.3)
    q = 0.12 * np.sign(plant.B[0])[None, :]               # pure surge pattern
    for frac, moves in ((1e-3, False), (0.0, True)):
        plant.SNAP_FRAC = frac
        p = Plant(spec, np.array([1], np.uint64), t_max=6)
        for _ in range(250):
            p.step(q)
        assert (np.abs(p.thr.T).max() > 0) == moves
        assert (np.abs(p.st).max() > 1e-6) == moves


@pytest.mark.parametrize('frac', [1e-3, 0.0])
def test_reversal_stops_at_zero_and_holds(snap, frac):
    """A thruster driven from +40 N to -30 N (lagged) stops at exactly zero at the crossing, holds zero for
    rev_hold, then builds negative thrust: with and without the snap."""
    plant.SNAP_FRAC = frac
    s = _thr(tau=0.1, hold=0.3)
    for _ in range(100):
        s.step(np.full((1, 4), 40.0))
    assert s.T[0, 0] > 39.0
    rev = np.array([s.step(np.full((1, 4), -30.0))[0, 0] for _ in range(120)])
    assert rev.min() < -29.0                               # it does reverse in the end
    first_zero = int(np.argmax(rev == 0.0))
    assert np.all(rev[:first_zero] > 0)                    # positive until the crossing
    n_zero = int((rev == 0.0).sum())
    assert np.all(rev[first_zero:first_zero + n_zero] == 0.0)   # one contiguous hold
    assert abs(n_zero * DT - 0.3) <= 2 * DT
    assert np.all(rev[first_zero + n_zero:] < 0)


def test_reversal_hold_without_snap_tau0(snap):
    """The static-limit case of test_core with SNAP_FRAC = 0."""
    plant.SNAP_FRAC = 0.0
    st = _thr(tau=0.0, hold=0.3)
    out = [st.step(np.full((1, 4), 40.0))[0, 0] for _ in range(5)]
    assert out[-1] == 40.0
    rev = [st.step(np.full((1, 4), -30.0))[0, 0] for _ in range(40)]
    assert abs(sum(v == 0.0 for v in rev) * DT - 0.3) <= 2 * DT
    assert rev[-1] == -30.0


def test_decay_to_command_zero_then_reverse_without_snap(snap):
    """Without the snap a thrust decaying toward f* = 0 never reaches exactly zero; a later reversal must still
    pass through an exact zero and the hold (the hold then starts at the crossing, not at the decay)."""
    plant.SNAP_FRAC = 0.0
    s = _thr(tau=0.2, hold=0.3)
    for _ in range(50):
        s.step(np.full((1, 4), 40.0))
    for _ in range(100):
        s.step(np.zeros((1, 4)))
    assert 0 < s.T[0, 0] < 0.5
    rev = np.array([s.step(np.full((1, 4), -30.0))[0, 0] for _ in range(60)])
    assert rev[0] == 0.0
    assert abs((rev == 0.0).sum() * DT - 0.3) <= 2 * DT
    assert rev[-1] < -25
