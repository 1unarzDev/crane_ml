# SPDX-License-Identifier: GPL-3.0-or-later
"""Leave-one-site-out comparison of blur feature forms for the per-frame model. Usage: python -I blur_features.py BENCH_DIR"""
import csv, sys, numpy as np
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import fit_obs as F
from scipy.optimize import minimize
rows = [r for r in csv.DictReader(open(sys.argv[1] + '/gt.csv')) if r['mode'] == 'tiled']
a = lambda k: np.array([float(r[k]) for r in rows])
h, vis, bl = a('h_px'), a('visibility') / 100, a('blur_px')
oc = np.array([r['sky'] == 'overcast' for r in rows], float); ls = np.array([r['sun'] == 'low_sun' for r in rows], float)
loc = np.array([r['location'] for r in rows])
for tau in (0.3, 0.5):
    y = (a('scoreloc') >= tau).astype(float)
    he = np.maximum(h, 4)
    for name, bf in (('log1p(b/h)', np.log1p(bl / he)), ('log1p(b/4)', np.log1p(bl / 4)), ('log1p(b/sqrt(h))', np.log1p(bl / np.sqrt(he))), ('b/h', bl / he)):
        X = np.stack([np.ones_like(h), np.log2(he) - 5, vis - 1, bf, oc, ls], 1)
        p = np.zeros_like(y)
        for L in set(loc):
            tr, te = loc != L, loc == L
            p[te] = F.predict(F.fit_detect(X[tr], y[tr]), X[te])
        m = F.metrics(p, y)
        byb = [(int(b), round(p[bl == b].mean(), 3), round(y[bl == b].mean(), 3)) for b in np.unique(bl)]
        print(tau, name, round(m['logloss'], 4), round(m['ece'], 4), byb, flush=True)
