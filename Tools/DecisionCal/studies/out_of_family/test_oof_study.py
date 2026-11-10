"""Driver guard rails and analysis helpers (no missions flown)."""
import numpy as np
import pytest

import oof_study as S


@pytest.mark.parametrize('seed', [20261109, 20261112, 20261116])
def test_refuses_confirmatory_seeds(seed, tmp_path):
    with pytest.raises(SystemExit):
        S.main(['pilot', '--out', str(tmp_path), '--seed', str(seed), '--smoke'])
    with pytest.raises(SystemExit):
        S.main(['diag', '--out', str(tmp_path), '--tune-seed', str(seed)])


def test_refuses_shared_seeds_and_too_many_procs(tmp_path):
    with pytest.raises(SystemExit):
        S.main(['pilot', '--out', str(tmp_path), '--seed', '4142', '--smoke'])        # = tune seed
    with pytest.raises(SystemExit):
        S.main(['pilot', '--out', str(tmp_path), '--procs', '5', '--smoke'])


def test_split_delay():
    assert S.split_delay(0.2, 0.1, 0.05) == (0.25, 0.1)
    d, lat = S.split_delay(0.05, 0.1, -0.10)
    assert d == 0.0 and abs(lat - 0.05) < 1e-12


def test_parse_config():
    assert S.parse_config('none:0') == (S.MECHS, 0.0)
    assert S.parse_config('link+waves:0.5') == (('link', 'waves'), 0.5)


def test_verdicts():
    assert S.verdict_baseline([-2.0, 2.5]).startswith('EQUIVALENT')
    assert S.verdict_baseline([-8.0, -4.0]) == 'fitted delay better (> margin)'
    assert S.verdict_baseline([-4.0, -1.0]) == 'fitted delay better'
    assert S.verdict_baseline([0.5, 6.0]) == 'baseline better'
    assert S.verdict_baseline([-5.0, 4.0]) == 'inconclusive'


def test_fixed_pick_is_best_mean_on_dev_boats():
    J1 = np.full((16, 6), 50.0)
    J1[3] = 30.0
    J2 = np.full((16, 6), 50.0)
    J2[3], J2[5] = 40.0, 25.0
    rows = [{'level': 'far', 'libs': {lib: {'J_h': J.tolist()} for lib in S.LIBS}} for J in (J1, J2)]
    assert S.fixed_picks(rows)['far'] == {lib: 3 for lib in S.LIBS}          # means 35 (c3) vs 37.5 (c5)
