"""python3 -m judge.cli_layers (from the crane root): outcome scoring, layer divergence, selection.

  score RECORD_DIR...            per-run outcome table (+ validity when judge.validity exists)
  diverge A B [--out DIR]        divergence.json, divergence.txt and evidence.csv for one matched pair
  select ROOT [--out DIR]        selection / regret / screening tables over every record under ROOT

A RECORD_DIR (or ROOT) is searched recursively for ``record.json``. The task pack is named by plugin
reference (``--pack MODULE``, default the dock-mission pack), so this module imports no pack.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

from judge import record as JR
from judge import selection as SEL
from judge.insight import layers as LY
from judge.pack import evaluate, load as load_ref

DEFAULT_PACK = 'judge.packs.dock_mission'


def _pack(mod):
    return (load_ref(f'{mod}:PACK'), load_ref(f'{mod}:outcome_time'), load_ref(f'{mod}:event_insight'))


def find_records(paths):
    out = []
    for p in paths:
        p = Path(p)
        if (p / 'record.json').is_file():
            out.append(p)
        else:
            out += sorted(q.parent for q in p.rglob('record.json'))
    return out


def load_all(paths, errors=None):
    recs = []
    for d in find_records(paths):
        try:
            recs.append(JR.load(d))
        except Exception as e:          # a bad record is reported, never silently dropped
            if errors is not None:
                errors.append({'path': str(d), 'error': f'{type(e).__name__}: {e}'})
    return recs


def _validity(rec):
    try:
        from judge import validity
    except ImportError:
        return None
    try:
        return validity.check(rec)
    except Exception as e:
        return {'error': f'{type(e).__name__}: {e}'}


def _fmt(v, nd=2):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '-'
    return f'{v:.{nd}f}' if isinstance(v, float) else str(v)


def score_rows(recs, pack, outcome_time, event_insight):
    rows = []
    for r in recs:
        vals = evaluate(r, None, [pack])
        cat = pack.outcome(r, vals)
        vr = _validity(r)
        ev = event_insight(r)
        g = lambda k: vals[k].value if k in vals and vals[k].usable else None
        row = {'run_id': r.run_id, 'backend': LY.header_block(r, 'backend').get('name') or r.header.source,
               'controller': LY.header_block(r, 'controller').get('id'), 'seed': r.header.scenario.seed,
               'outcome': cat, 'docked': g('dock.docked'), 't_dock': g('dock.time_to_dock'),
               'retries': g('dock.retries'), 'contacts': g('dock.contacts'), 'gate_miss': g('dock.gate_miss'),
               'pose_err': g('dock.pose_error'), 'J': g('dock.J'),
               'metrics': {k: v.as_dict() for k, v in vals.items()},
               'events_disagree': ev.claim if ev else None}
        if vr is None:
            row['valid'] = None
        elif isinstance(vr, dict):
            row['valid'], row['validity'] = None, vr
        else:
            row['valid'], row['validity'] = vr.valid, vr.as_dict()
            row['validity_codes'] = vr.codes
        rows.append(row)
    return rows


def score_text(rows):
    hdr = (f'{"run":44s} {"backend":24s} {"ctrl":5s} {"seed":>8s} {"outcome":20s} {"t_dock":>7s} {"retry":>5s} '
           f'{"cont":>4s} {"gate":>4s} {"pose":>5s} {"J":>7s} valid')
    L = [hdr]
    for r in rows:
        v = '-' if r['valid'] is None else ('yes' if r['valid'] else 'NO ' + ','.join(r.get('validity_codes', [])))
        L.append(f'{r["run_id"][:44]:44s} {str(r["backend"])[:24]:24s} {str(r["controller"])[:5]:5s} '
                 f'{_fmt(r["seed"]):>8s} {r["outcome"]:20s} {_fmt(r["t_dock"]):>7s} {_fmt(r["retries"], 0):>5s} '
                 f'{_fmt(r["contacts"], 0):>4s} {_fmt(r["gate_miss"], 0):>4s} {_fmt(r["pose_err"]):>5s} '
                 f'{_fmt(r["J"]):>7s} {v}')
        if r['events_disagree']:
            L.append(f'    events: {r["events_disagree"]}')
    return '\n'.join(L)


def cmd_score(a):
    pack, ot, ei = _pack(a.pack)
    errors = []
    recs = load_all(a.paths, errors)
    rows = score_rows(recs, pack, ot, ei)
    print(score_text(rows))
    for e in errors:
        print(f'LOAD ERROR {e["path"]}: {e["error"]}')
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'score.json').write_text(json.dumps(LY._jsonable({'rows': rows, 'load_errors': errors}),
                                                   sort_keys=True, indent=1) + '\n')
        (out / 'score.txt').write_text(score_text(rows) + '\n')
    return 0


def write_divergence(res, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'divergence.json').write_text(json.dumps(res.as_dict(), sort_keys=True, indent=1) + '\n')
    (out / 'divergence.txt').write_text(res.explanation + '\n\nattribution procedure:\n' + LY.ATTRIBUTION_DOC + '\n')
    if res.evidence:
        (out / 'evidence.csv').write_text(LY.evidence_csv(res))
    return out


def cmd_diverge(a):
    _, ot, _ = _pack(a.pack)
    ra, rb = JR.load(a.a), JR.load(a.b)
    res = LY.diverge(ra, rb, allow_mismatch=a.allow_mismatch, outcome=ot)
    print(res.explanation)
    if a.out:
        write_divergence(res, a.out)
    return 0 if res.verdict != 'refused' else 2


def cmd_select(a):
    pack, _, _ = _pack(a.pack)
    errors = []
    recs = load_all([a.root], errors)

    def score(r):
        v = evaluate(r, None, [pack]).get('dock.J')
        return v.value if v is not None and v.usable else None

    def failed(r):
        v = evaluate(r, None, [pack]).get('dock.docked')
        return not (v is not None and v.usable and v.value == 1.0)
    table = SEL.table_from_records(recs, score, failed)
    res = SEL.analyze(table, n_boot=a.n_boot, seed=a.seed, margin=a.margin)
    txt = SEL.table_text(res)
    skipped = {sc: v['skipped'] for sc, v in table.items() if v['skipped']}
    print(txt)
    if skipped:
        print('unscorable runs (no J): ' + json.dumps(skipped))
    for e in errors:
        print(f'LOAD ERROR {e["path"]}: {e["error"]}')
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'select.json').write_text(json.dumps(LY._jsonable({**res, 'skipped': skipped, 'load_errors': errors}),
                                                    sort_keys=True, indent=1) + '\n')
        (out / 'select.txt').write_text(txt + '\n')
    return 0


def add_subcommands(sub):
    """Register score / diverge / select on an argparse subparsers object."""
    p = sub.add_parser('score', help='per-run outcome table')
    p.add_argument('paths', nargs='+')
    p.add_argument('--out')
    p.add_argument('--pack', default=DEFAULT_PACK)
    p.set_defaults(fn=cmd_score)
    p = sub.add_parser('diverge', help='divergence between two matched records')
    p.add_argument('a')
    p.add_argument('b')
    p.add_argument('--out')
    p.add_argument('--allow-mismatch', action='store_true')
    p.add_argument('--pack', default=DEFAULT_PACK)
    p.set_defaults(fn=cmd_diverge)
    p = sub.add_parser('select', help='selection, regret and screening over a directory of records')
    p.add_argument('root')
    p.add_argument('--out')
    p.add_argument('--n-boot', type=int, default=500)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--margin', type=float, default=0.0)
    p.add_argument('--pack', default=DEFAULT_PACK)
    p.set_defaults(fn=cmd_select)
    return sub


def main(argv=None):
    ap = argparse.ArgumentParser(prog='judge.cli_layers', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_subcommands(ap.add_subparsers(dest='cmd', required=True))
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
