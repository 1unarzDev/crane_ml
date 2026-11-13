"""Offline regression for the legacy FossenDynamics added-mass sign (I1 V1, F5 bug 1).

Mirrors the C# update: Unity integrates m a = T + F_am, with F_am = sgn * M_A * a_prev (nuDot is a one-tick
backward difference, so it lags one tick). The analytic requirement is a = T/(m + M_A) at steady state, so
sgn must be -1. With the old +1 the response is T/(m - M_A) (1.69x for m = 14.6, M_A = 6 is ~ (m+MA)/(m-MA)).
Pure-double Python; no Unity.
"""
import pytest

M, MA, T, DT = 20.0, 6.0, 10.0, 0.02


def steady_accel(sgn, steps=4000):
    v, a_prev = 0.0, 0.0
    for _ in range(steps):
        a = (T + sgn * MA * a_prev) / M
        v_new = v + DT * a
        a_prev = (v_new - v) / DT
        v = v_new
    return a_prev


def test_fixed_sign_gives_effective_mass_response():
    # fixed point: a = T/(M + MA)
    assert steady_accel(-1.0) == pytest.approx(T / (M + MA), rel=1e-9)


def test_old_sign_reproduces_the_bug():
    assert steady_accel(+1.0) == pytest.approx(T / (M - MA), rel=1e-6)
    assert steady_accel(+1.0) / steady_accel(-1.0) == pytest.approx((M + MA) / (M - MA), rel=1e-6)
