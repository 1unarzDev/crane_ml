"""Library v2 invariants and the mission.run backward-compatibility pin (fast: < 1 min)."""
import numpy as np

import library
import library_v2
import mission
import twin

# mission.run J of the 16 v1 controllers on the nominal twin, seeds arange(16), computed BEFORE the
# mission.py edit that added kp_scale / ki_scale / per-library profiles.
V1_REFERENCE_J = [
    42.26000000000101, 57.800000000003436, 51.74000000000249, 51.340000000002426, 46.16000000000162,
    95.83999999999807, 92.47999999999874, 51.22000000000241, 40.44000000000072, 90.01999999999923,
    84.08000000000041, 48.520000000001986, 60.540000000003864, 71.44000000000293, 67.26000000000376,
    65.98000000000401]


def _nominal_run(lib):
    C = len(lib)
    spec = twin.to_spec(twin.nominal_particles(C))
    return mission.run(spec, np.arange(C), np.arange(C), lib=lib)[0]


def test_v1_default_library_unchanged():
    spec = twin.to_spec(twin.nominal_particles(16))
    m, _ = mission.run(spec, np.arange(16), np.arange(16))
    assert np.allclose(m['J'], V1_REFERENCE_J, rtol=0, atol=1e-9)
    assert mission.profiles_for(library.LIBRARY) is mission.PROFILES


def test_v2_size_and_unique_hashes():
    lib = library_v2.LIBRARY_V2
    assert 12 <= len(lib) <= 16
    assert len({c.sha() for c in lib}) == len(lib)
    assert [c.idx for c in lib] == list(range(len(lib)))


def test_v2_axes_graded_and_ff_mostly_on():
    lib = library_v2.LIBRARY_V2
    for field in ('kp_scale', 'ki_scale'):
        assert {getattr(c, field) for c in lib} >= {0.5, 1.0, 1.5}
    assert len({c.brake for c in lib}) >= 3 and len({c.latcomp for c in lib}) >= 3
    n_pi = sum(not c.feedforward for c in lib)
    assert 1 <= n_pi <= 2


def test_v2_nominal_docks_and_outcomes_distinct():
    m = _nominal_run(library_v2.LIBRARY_V2)
    assert m['docked'].all()
    assert len(np.unique(np.round(m['J'], 3))) == len(library_v2.LIBRARY_V2)


def test_v2_run_uses_v2_profiles():
    lib = library_v2.LIBRARY_V2
    prof = mission.profiles_for(lib)
    assert prof.shape == (len(lib), len(mission.ARC))
    assert prof is mission.profiles_for(lib)                      # cached
    assert not np.array_equal(prof, mission.PROFILES[:len(lib)])
    # graded dock decel: the aggressive sqrt@0.6 member allows a higher final-approach speed than sqrt@0.08
    fast = next(i for i, c in enumerate(lib) if c.dock == 'sqrt@0.6')
    slow = next(i for i, c in enumerate(lib) if c.dock == 'sqrt@0.08')
    k = int(np.argmin(np.abs(mission.S_END - mission.ARC - 2.0)))
    assert prof[fast, k] > prof[slow, k]
    # a v2 run must differ from the same indices flown with v1 profiles
    a = _nominal_run(lib)['J']
    spec = twin.to_spec(twin.nominal_particles(len(lib)))
    saved = mission._PROFILE_CACHE.copy()
    try:
        mission._PROFILE_CACHE[tuple(lib)] = mission.PROFILES[:len(lib)]
        b = mission.run(spec, np.arange(len(lib)), np.arange(len(lib)), lib=lib)[0]['J']
    finally:
        mission._PROFILE_CACHE.clear()
        mission._PROFILE_CACHE.update(saved)
    assert not np.allclose(a, b)


def test_gain_scales_change_behaviour():
    """kp_scale / ki_scale reach the velocity law (otherwise v2 gains would be silently ignored)."""
    from dataclasses import replace
    import hidden
    base = library_v2.LIBRARY_V2[3]
    lib = (base, replace(base, idx=1, kp_scale=0.5), replace(base, idx=2, ki_scale=0.5))
    spec = hidden.sample_hidden(1, 4242, 'transient').repeat(3)       # a laggy boat: gains matter
    J = mission.run(spec, np.arange(3), np.zeros(3, np.uint64), t_max=40, lib=lib)[0]['J']
    assert len(np.unique(np.round(J, 3))) == 3
