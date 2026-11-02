"""Build the RoboBoat plant card from the characterization profile and the 10-18 field trials.

    python3 -m plantcard.marine.build [--trials DIR] [--store DIR] [--repo] [--n 128]

1. c0: ``profiles/characterized.json`` wrapped as a prior card (P0).
2. Evidence: the open-loop pulse trials (GPS + gyro truth; see ``fieldtrials``).
3. Leave-one-session-out: c0 scores every trial before any fit (prequential); each fold's child
   scores only the session it never saw. Events pos_err_max@{2,5,10}s, alpha 0.1.
4. c1 = propose(c0, all evidence) + validation records + the promotion check. c1 stays
   ``fitted`` (a human accepts promotion) and becomes ref ``<robot>@candidate``; HEAD stays c0
   until then.
5. Exports next to the card: unity_profile.json, nlc_params.json, control.json, validation_rows.json.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from plantcard import card as C
from plantcard import validate as V
from plantcard.marine import fieldtrials, seed, views
from plantcard.update import propose

HORIZONS = (2.0, 5.0, 10.0)
ALPHA = 0.1


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        o = o.item()
    if isinstance(o, float) and not np.isfinite(o):
        return None
    return o


def build(trials=fieldtrials.DEFAULT_DIR, stores=(None,), n=128, log=print):
    doc0, parts0 = seed.prior_doc()
    c0 = C.Card(doc0, parts0)
    ev = fieldtrials.load(trials, [t['name'] for t in c0.family.config['thrusters']], log=log)
    log('cross-validating (leave one session out) ...')
    child_rows, parent_rows = V.cross_validate(c0, ev, horizons=HORIZONS, alphas=(ALPHA,), n=n, log=log)
    summ = {H: V.summarize(child_rows, ALPHA, H) for H in HORIZONS}
    psumm = {H: V.summarize(parent_rows, ALPHA, H) for H in HORIZONS}
    doc1, parts1, support1, report = propose(c0, ev, procedure='integral-window NIG (1 s), GPS+gyro, 10-18 pulses',
                                             note='first field fit: 10-18 open-loop pulse trials')
    records = []
    for H in HORIZONS:
        if summ[H] is None:
            continue
        rec = dict(summ[H], H=H, tier='real', heldout='leave-one-session-out', card_scored_before_fit=True,
                   parent={k: psumm[H][k] for k in ('covered_frac', 'lcb95_cp', 'width_median_m', 'crps_final_m',
                                                     'mean_path_err_max_median_m', 'conformal_multiplier')},
                   parent_raw=psumm[H]['raw'])
        rec['promotion'] = V.promotion(summ[H], psumm[H], ALPHA)
        records.append(rec)
    doc1['validation'] = _clean(records)
    doc1['promotion'] = {'policy': 'design 5.6 (frozen): CP LCB95 >= 1 - alpha - 0.05, width <= 1.1 parent, '
                                   'CRPS <= parent; >= 5 sessions on >= 2 days for validated; human accepts',
                         'event': 'pos_err_max@H5s', 'passed': bool(next(r['promotion']['passed'] for r in records
                                                                         if r['H'] == 5.0)) if records else False,
                         'days': 1, 'awaiting': 'owner acceptance (accepted_by)'}
    out = {}
    for store in stores:
        saved0 = C.save(doc0, parts0, None, store)
        doc1['parent'] = saved0.sha
        saved1 = C.save(doc1, parts1, support1, store)
        C.set_ref(c0.robot, saved0.sha, store)
        C.set_ref(c0.robot, saved1.sha, store, name='candidate')
        ex = Path(saved1.path) / 'exports'
        ex.mkdir(exist_ok=True)
        (ex / 'unity_profile.json').write_text(json.dumps(views.unity_profile(saved1), indent=4) + '\n')
        (ex / 'nlc_params.json').write_text(json.dumps(_clean(views.nlc_params(saved1)), indent=1) + '\n')
        (ex / 'control.json').write_text(json.dumps(_clean(views.control(saved1)), indent=1) + '\n')
        (ex / 'validation_rows.json').write_text(json.dumps(_clean({'child': child_rows, 'parent': parent_rows,
                                                                    'evidence': ev.provenance}), indent=1) + '\n')
        out[str(store)] = (saved0, saved1)
        log(f'saved c0 {saved0.sha[:12]} and c1 {saved1.sha[:12]} in {saved1.path.parent.parent}')
    return out, summ, psumm, report, ev


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trials', default=str(fieldtrials.DEFAULT_DIR))
    ap.add_argument('--store', default=None, help='card store (default $PLANTCARD_STORE or ~/.cache/plantcards)')
    ap.add_argument('--repo', action='store_true', help='also write the cards into plantcard/cards (git-tracked)')
    ap.add_argument('--n', type=int, default=128, help='rollouts per held-out segment')
    a = ap.parse_args(argv)
    stores = [a.store or C.default_store()] + ([C.HERE] if a.repo else [])
    out, summ, psumm, report, ev = build(a.trials, stores, a.n)
    print(json.dumps(_clean({'fit': report, 'child': summ, 'parent': psumm}), indent=1))


if __name__ == '__main__':
    main()
