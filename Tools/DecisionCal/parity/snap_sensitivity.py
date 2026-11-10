#!/usr/bin/env python3
"""EXPLORATORY sensitivity of the frozen M5 / M2 headlines to the NumPy thrust-snap artifact (PARITY.md, task A).

The snap flag is set on ``plant`` BEFORE any other harness module is imported, so it reaches probe
generation, the twin fitter (twin.py builds its own ThrusterState), twin missions and hidden missions; the
fork pool inherits it. Development seed only: confirmatory seeds 20261109-20261116 are refused.

    nice -n 19 python3 -I Tools/DecisionCal/parity/snap_sensitivity.py --design m5 --snap off --out DIR
    nice -n 19 python3 -I Tools/DecisionCal/parity/snap_sensitivity.py --design m2 --snap on  --out DIR

m5: study_m5.evaluate on N timing boats (twins with true L + e on fitted and true bodies, libraries v2, v3).
m2: study_m1.evaluate with the M2 arm set (nominal, rigid, rigid+delay, rigid+lag, joint; regression fitter)
    on N transient boats, libraries v2, v3.
Common seeds: the boats and every noise key depend only on (--seed, boat id), so the on/off runs are paired.
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True
DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))

CONFIRMATORY = range(20261109, 20261117)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--design', choices=('m5', 'm2'), required=True)
    ap.add_argument('--snap', choices=('on', 'off'), required=True)
    ap.add_argument('--seed', type=int, default=5353)
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--procs', type=int, default=2)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    if a.seed in CONFIRMATORY:
        raise SystemExit(f'seed {a.seed} is a spent confirmatory seed (20261109-20261116); refused')
    if a.procs > 2:
        raise SystemExit('parity stream compute budget: at most 2 NumPy processes')

    import plant                                   # first harness import: set the flag before anything else
    plant.SNAP_FRAC = 1e-3 if a.snap == 'on' else 0.0
    from multiprocessing import get_context
    import _paths
    import hidden
    import record
    import twin
    assert twin.ThrusterState is plant.ThrusterState

    if a.design == 'm5':
        import study_m5 as S
        stratum, fn = 'timing', S.evaluate
        extra = {}
    else:
        import study_m2  # noqa: F401  (sets study_m1's arm globals exactly as the M2 driver does)
        import study_m1 as S
        stratum, fn = 'transient', S.evaluate
        extra = {'K': 6, 'M': 16, 'n_prop': 0}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    spec = hidden.sample_hidden(a.n, a.seed, stratum)
    jobs = [{'spec': spec.take([i]), 'id': f'{stratum}-{i:03d}', 'stratum': stratum, 'seed': a.seed, **extra}
            for i in range(a.n)]
    pj = out / 'plants.jsonl'
    done = {}
    if pj.exists():
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    (out / 'manifest.json').write_text(json.dumps(record._clean(
        {'argv': vars(a), 'snap_frac': plant.SNAP_FRAC, 'provenance': _paths.provenance(),
         'hidden_ranges_sha256': hidden.ranges_sha(), 'evidence_label': 'EXPLORATORY (development seed)',
         'started': time.strftime('%Y-%m-%dT%H:%M:%S')}), indent=1, sort_keys=True))
    t0 = time.time()

    def init():
        assert plant.SNAP_FRAC == (1e-3 if a.snap == 'on' else 0.0)

    with get_context('fork').Pool(a.procs, initializer=init) as pool, open(pj, 'a') as fp:
        for k, r in enumerate(pool.imap_unordered(fn, todo)):
            r['snap_frac'] = plant.SNAP_FRAC
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            print(f'[{k + 1}/{len(todo)}] {r["id"]} ({time.time() - t0:.0f}s)', flush=True)
    print('done', time.time() - t0)


if __name__ == '__main__':
    main()
