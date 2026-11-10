"""Model-free selectors (trained on 200 development boats, seed 7373) scored on the M4 boats, next to the
twin arms on the same boats. Exploratory. Features of M4 probe logs come from analysis/crude_rule.py's cache
(regenerated logs; evidence hashes matched the stored rows)."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import baseline_selector as bs
import score

train = bs.load(str(Path.home() / '.cache/decisioncal/baseline-train/train.npz'))
feats = json.load(open('/tmp/dcr/feats.json'))
rows = [json.loads(l) for l in open(Path.home() / '.cache/decisioncal/m4-20261114/plants.jsonl')]
X = np.array([feats['m4/' + r['id']]['f'] for r in rows], float)
rng = np.random.default_rng(0)
def ci(d):
    d = np.asarray(d, float); idx = rng.integers(0, len(d), (4000, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95]); return '%+.1f [%+.1f, %+.1f]' % (d.mean(), lo, hi)
for li, lib in enumerate(['v2', 'v3']):
    sels = bs.train_selectors(train, li)
    picks = {n: m.select(X) for n, m in sels.items() if not n.startswith('stratum')}
    reg = {}
    for n, sel in picks.items():
        reg[n] = np.array([min(score.cross_fit_regret(np.array(r['libs'][lib]['J_h']), int(s)), 60.0) for r, s in zip(rows, sel)])
        reg[n + ':dock'] = np.array([r['libs'][lib]['hidden_docked'][int(s)] for r, s in zip(rows, sel)])
    for a in ('rigid', 'rigid+delay', 'joint', 'rigid@d0.47'):
        reg[a] = np.array([min(r['libs'][lib]['scores']['arms'][a]['regret'], 60.0) for r in rows])
        reg[a + ':dock'] = np.array([r['libs'][lib]['hidden_docked'][r['libs'][lib]['scores']['arms'][a]['sel']] for r in rows])
    st = np.array([r['stratum'] for r in rows])
    print(f'== library {lib}: mean capped regret (s) / dock rate on M4 boats; best-fixed controller = c{sels["best-fixed"].c:02d}, knn k={sels["knn"].k}')
    names = ['rigid', 'rigid@d0.47', 'rigid+delay', 'joint', 'best-fixed', 'knn', 'ridge', 'ridge-quad']
    for s in ('timing', 'transient', 'steady', 'all'):
        m = np.ones(len(rows), bool) if s == 'all' else st == s
        print(f'  {s:9s} ' + ' | '.join(f'{n} {reg[n][m].mean():.1f}/{reg[n + ":dock"][m].mean():.2f}' for n in names))
    for n in ('best-fixed', 'knn', 'ridge-quad'):
        print(f'  {n} minus joint (all): {ci(reg[n] - reg["joint"])};  minus fitted delay: {ci(reg[n] - reg["rigid+delay"])}')
