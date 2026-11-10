# SPDX-License-Identifier: GPL-3.0-or-later
"""Re-render the result figures (PNG + PDF with Type 1 core fonts) from saved metrics/params, without re-running the
benchmark. Usage: python -I make_figures.py"""
import json, sys
from pathlib import Path

T = Path(__file__).resolve().parent
sys.path.insert(0, str(T))
import analyze, fit_obs  # noqa: E402

for sub, params in (('', 'obs_params_tau03.json'), ('tau05', 'obs_params_tau05.json')):
    out = T / 'results' / sub
    M = json.load(open(out / 'metrics.json'))
    blurs = sorted(int(k.split('_')[1]) for k in M if k.startswith('blur_') and k[5:].isdigit())
    analyze.figures(None, M, out, M['tau'], blurs)
    fit_obs.calib_fig(json.load(open(T / params)), out)
    print('figures', out)
