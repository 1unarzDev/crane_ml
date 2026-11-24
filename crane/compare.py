"""crane compare: two run sets side by side, as a Markdown table and JSON.

    crane compare A B                          variant A/B: matched runs on one frame and clock, metric deltas
    crane compare LOW HIGH --tiers             tier agreement (G3) and tier gap (G4) per metric
    crane compare --field BUNDLE RUNS          per astro test: the result's metrics, field vs sim

**A B.** Runs are crane.run/1 records found under each root, matched by their path below it. B is put in
A's frame (``crane.frames.to_common``, refused when the origins differ) and on A's clock (``align``,
refused when one is sim and the other wall); the table gives the position RMS, final and max distance and
the heading RMS, then each shared numeric metric's delta.

**--tiers.** LOW and HIGH hold variants as their first directory level. Per metric: ``judge.compare.
tier_agreement`` over the variants' means (regret, MMRV, top-k, Kendall) and ``tier_gap`` over runs paired
by path (bias low - high with a unit bootstrap CI, ``--band`` for a verdict). ``--fidelity CONFIG`` also
replays HIGH's goal windows (events with ``kind`` ``goal*`` and x, y) on CONFIG's vehicle
(``fastsim.fidelity.score``).

**--field.** The bundle's results (per test the applied one, else the newest) against the runs of
``crane run --astro-test`` (``metrics.json`` holds the same fields). Metrics are the result's numbers under
the names astro writes (``changes`` keys without ``characterization.``, then ``measured``); units follow
the name. Delta is sim - field. The CI (95 %, ``judge.stats.bootstrap_ci``) resamples the field's per-hold
samples when the metric has them (t95), else the sim runs when there are two or more.

**--bounds.** With ``--field``: pass/fail per row against ``config/fidelity/<domain>.yaml`` (``--bounds FILE``
names another file), the gate of the fit (``--fit DIR``, else ``fit-<bundle>``) and a verdict line. Exit 0
pass, 1 fail, 2 usage. ``--bounds card`` (tolerances from the plant card's spread) is not yet available.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SAMPLE_LISTS = ('measured.t95',)


class CompareError(ValueError):
    pass


# ---- run sets -------------------------------------------------------------------------------------------

def find_runs(root):
    """{relative path: RunRecord} of every crane.run/1 record under ``root`` (a run dir counts as one)."""
    from crane import record
    root = Path(root)
    out = {}
    metas = sorted({p.parent for name in (record.SIDECAR, record.META) for p in root.rglob(name)})
    for d in metas:
        try:
            rec = record.read_run(d)
        except (record.RecordError, json.JSONDecodeError):
            continue
        rel = str(d.relative_to(root)) if d != root else '.'
        out[rel] = rec
    return out


# ---- astro result metrics -------------------------------------------------------------------------------

def unit_of(name):
    leaf = name.rsplit('.', 1)[-1]
    yaw = 'yaw' in name
    if leaf in ('d1',):
        return 'e^2 s/m' if not yaw else 'e^2 s/rad'
    if leaf in ('d2',):
        return 'e^2 s^2/m^2' if not yaw else 'e^2 s^2/rad^2'
    if 't95' in name or leaf in ('rise_time', 'settling_time'):
        return 's'
    if 'acceleration' in name:
        return 'rad/s^2' if yaw else 'm/s^2'
    if 'cross' in name or leaf in ('overshoot', 'samples'):
        return '1'
    if leaf == 'heading_deg':
        return 'deg'
    if leaf == 'iae':
        return 'rad' if yaw else 'm'
    return 'rad/s' if yaw else 'm/s'


def flatten(result, axes=('surge', 'sway', 'yaw')):
    """{name: number or [samples]} of an astro result (or a sim run's metrics.json, which has its fields)."""
    out = {}
    test = str(result.get('test', ''))
    axis = test.split('-', 1)[1] if '-' in test else ''

    def walk(prefix, v):
        if isinstance(v, bool) or v is None:
            return
        if isinstance(v, (int, float)):
            if math.isfinite(v):
                out[prefix] = float(v)
        elif isinstance(v, dict):
            for k, x in v.items():
                walk(f'{prefix}.{k}', x)
        elif isinstance(v, list) and v and all(isinstance(x, (int, float)) for x in v):
            if prefix in SAMPLE_LISTS:
                out[prefix] = [float(x) for x in v]
            else:
                names = axes if len(v) == len(axes) else range(len(v))
                for k, x in zip(names, v):
                    walk(f'{prefix}.{k}', x)
    for c in result.get('changes') or []:
        if 'column' in c or not c.get('key') or c['key'].endswith('.version'):
            continue
        walk(c['key'].removeprefix('characterization.'), c['value'])
    measured = result.get('measured') or {}
    kind = result.get('type')
    if kind == 'speed':
        for sign, pts in (measured.get('points') or {}).items():
            for v, e in pts:
                out[f'measured.speed.{axis}{sign}@{e:g}'] = float(v)
        if measured.get('t95'):
            out['measured.t95'] = [float(x) for x in measured['t95']]
    elif kind == 'couple':
        seen = set()
        for h in measured.get('holds') or []:
            if h['sign'] in seen:          # the first hold of each sign: the column in force
                continue
            seen.add(h['sign'])
            d = f'{axis}{"+" if h["sign"] > 0 else "-"}'
            others = [a for a in axes if a != axis]
            for a, x in zip(others, h.get('cross') or []):
                out[f'measured.cross.{d}.{a}'] = float(x)
    elif kind in ('still', 'steps'):
        walk('measured', {k: v for k, v in measured.items() if k != 'samples'})
    return out


def _mean(v):
    return float(np.mean(v)) if isinstance(v, list) else v


def field_rows(field_metrics, sim_metrics, only=None):
    """Rows {metric, unit, field, sim, delta, ci, n_field, n_sim} for the metrics both sides have."""
    from judge import stats
    rows = []
    names = sorted(set(field_metrics) & set().union(*[set(m) for m in sim_metrics])) if sim_metrics else []
    for name in names:
        if only and not any(name == o or name.startswith(o + '.') for o in only):
            continue
        f = field_metrics[name]
        sims = [m[name] for m in sim_metrics if name in m]
        sim = float(np.mean([_mean(s) for s in sims]))
        fv = _mean(f)
        row = {'metric': name, 'unit': unit_of(name), 'field': fv, 'sim': sim, 'delta': sim - fv,
               'n_field': len(f) if isinstance(f, list) else 1, 'n_sim': len(sims), 'ci': None, 'ci_over': None}
        if isinstance(f, list) and len(f) >= 2:
            _, ci = stats.bootstrap_ci(f, lambda u: sim - float(np.mean(u)))
            row.update(ci=list(ci), ci_over='field samples')
        elif len(sims) >= 2:
            _, ci = stats.bootstrap_ci([_mean(s) for s in sims], lambda u: float(np.mean(u)) - fv)
            row.update(ci=list(ci), ci_over='sim runs')
        rows.append(row)
    return rows


def compare_field(bundle, runs_root, only=None):
    """{test: {variant: rows}} of a bundle against astro-test runs."""
    import tempfile
    from crane import fit
    B, _ = fit.characterization()
    with tempfile.TemporaryDirectory(prefix='crane-compare-') as tmp:
        b = fit.open_bundle(bundle, tmp)
        picked = B.latest_by_test(b.results)
    runs = find_runs(runs_root)
    by = {}
    for rel, rec in runs.items():
        m = rec.metrics or {}
        if m.get('test'):
            variant = rel.split('/', 1)[0] if '/' in rel else '.'
            by.setdefault((m['test'], variant), []).append(flatten(m))
    out = {}
    for test, (path, doc) in sorted(picked.items()):
        fm = flatten(doc)
        variants = sorted(v for t, v in by if t == test)
        if not variants:
            out[test] = {None: {'field_result': path.name, 'note': 'no sim run of this test'}}
            continue
        out[test] = {v: {'field_result': path.name, 'rows': field_rows(fm, by[(test, v)], only)} for v in variants}
    return out


# ---- fidelity bounds ----------------------------------------------------------------------------------------

def bounds_path(domain):
    from crane import paths
    return paths.CONFIG / 'fidelity' / f'{domain}.yaml'


def load_bounds(where):
    """{'gate': {...}, 'rules': [...]} from a bounds YAML; CompareError when missing or malformed."""
    import yaml
    p = Path(where)
    if not p.is_file():
        raise CompareError(f'no bounds for this domain: create {p} (rules: [{{metric: GLOB, rel|abs: X, '
                           'ci_zero_ok: bool, unit: U}], gate: {require: identified})')
    doc = yaml.safe_load(p.read_text()) or {}
    for r in doc.get('rules') or []:
        if 'metric' not in r or ('rel' in r) == ('abs' in r):
            raise CompareError(f'{p}: each rule needs `metric` and exactly one of `rel`, `abs`: {r}')
    doc['rules'] = doc.get('rules') or []
    return doc


def _rule_for(rules, metric):
    from fnmatch import fnmatchcase
    return next((r for r in rules if fnmatchcase(metric, r['metric'])), None)


def judge_row(row, rule):
    """The row with ``result`` (pass, FAIL, n/a), ``bound`` (+- in the row's unit) and ``rule`` set."""
    if rule is None:
        return {**row, 'result': 'n/a', 'bound': None, 'rule': None}
    bound = float(rule['abs']) if 'abs' in rule else float(rule['rel']) * abs(row['field'])
    ci = row.get('ci')
    in_ci = bool(rule.get('ci_zero_ok')) and bool(ci) and ci[0] <= 0.0 <= ci[1]
    ok = abs(row['delta']) <= bound or in_ci
    why = f"{rule['rel'] * 100:g}%" if 'rel' in rule else f"{rule['abs']:g} abs"
    if rule.get('ci_zero_ok'):
        why += ', or CI contains 0'
    return {**row, 'result': 'pass' if ok else 'FAIL', 'bound': bound, 'rule': why, 'hint': rule.get('hint'),
            'ci_zero': in_ci and abs(row['delta']) > bound}


def read_gate(bundle, fit_dir=None):
    """{'checked', 'ok', 'not_identified', 'note'} from ``crane fit``'s fit.json (``fit-<bundle>`` by default)."""
    d = Path(fit_dir) if fit_dir else Path(f'fit-{Path(bundle).name.split(".")[0]}')
    f = d / 'fit.json'
    if not f.is_file():
        return {'checked': False, 'ok': None, 'not_identified': [],
                'note': f'no {f}; crane fit {bundle} --out {d}, or --fit DIR'}
    doc = json.loads(f.read_text())
    from crane import fit
    _, G = fit.characterization()
    rows = [q for q in (doc.get('gate') or {}).get('parameters') or []
            if q['name'] in G.DIRECTIONS or q['name'].startswith('addedMass.')]
    bad = [q['name'] for q in rows if q['status'] != 'identified']
    return {'checked': True, 'ok': not bad, 'not_identified': bad, 'note': str(f)}


def field_bounds(bundle, runs_root, bounds, only=None, fit_dir=None):
    """Judged rows, the gate and the verdict of a bundle against astro-test runs."""
    res = compare_field(bundle, runs_root, only)
    rows, notes = [], []
    for test, variants in res.items():
        for variant, body in variants.items():
            if 'rows' not in body:
                notes.append({'test': test, 'note': body.get('note')})
                continue
            for r in body['rows']:
                rows.append({'test': test, 'variant': variant, **judge_row(r, _rule_for(bounds['rules'], r['metric']))})
    if (bounds.get('gate') or {}).get('require') == 'identified':
        gate = read_gate(bundle, fit_dir)
    else:
        gate = {'checked': False, 'ok': None, 'not_identified': [], 'note': 'the bounds file has no gate rule'}
    failed = sum(r['result'] == 'FAIL' for r in rows)
    n_bound = sum(r['result'] != 'n/a' for r in rows)
    gate_fail = gate['checked'] and not gate['ok']
    verdict = {'pass': not failed and not gate_fail and n_bound > 0, 'rows': len(rows), 'bounds': n_bound,
               'failed': failed, 'gate': 'not checked' if not gate['checked'] else
               ('identified' if gate['ok'] else 'not identified'), 'gate_not_identified': gate['not_identified']}
    return {'rows': rows, 'notes': notes, 'gate': gate, 'verdict': verdict, 'bundle': str(bundle)}


def bounds_text(res):
    rows, v, g = res['rows'], res['verdict'], res['gate']
    lines = []
    for test, variant in sorted({(r['test'], r['variant']) for r in rows}):
        sel = [r for r in rows if (r['test'], r['variant']) == (test, variant)]
        w = max(len(r['metric']) for r in sel)
        lines.append(f'{test} vs sim `{variant}`')
        lines.append(f"  {'metric':<{w}}  {'unit':<8} {'field':>10} {'sim':>10} {'delta':>10}  {'bound':<30} result")
        for r in sel:
            bound = '-' if r['bound'] is None else f"+-{_f(r['bound'])} ({r['rule']})"
            lines.append(f"  {r['metric']:<{w}}  {r['unit']:<8} {_f(r['field']):>10} {_f(r['sim']):>10} "
                         f"{_f(r['delta']):>10}  {bound:<30} {r['result']}")
        lines.append('')
    lines += [f"{n['test']}: {n['note']}" for n in res['notes']]
    for r in rows:
        if r['result'] == 'FAIL':
            ci = f", CI [{_f(r['ci'][0])}, {_f(r['ci'][1])}]" if r['ci'] else ''
            hint = (r.get('hint') or 'refit: crane fit {bundle}').format(bundle=res['bundle'])
            lines.append(f"FAIL {r['test']} {r['metric']}: delta {r['delta']:+.4g} {r['unit']} outside "
                         f"+-{_f(r['bound'])} {r['unit']} ({r['rule']}){ci}; fix: {hint}")
    if not g['checked']:
        lines.append(f"gate: not checked ({g['note']})")
    else:
        lines.append('gate: all identified' if g['ok'] else 'gate: not identified: ' + ', '.join(g['not_identified']))
    if v['pass']:
        lines.append(f"verdict: pass ({v['rows']} rows, {v['bounds']} bounds)")
        lines.append('next: the sim stands in for the field on these tests')
    else:
        extra = ''
        if v['gate_not_identified']:
            extra = f"; gate: {', '.join(v['gate_not_identified'])} not identified"
        if not v['bounds']:
            extra += '; no row matched a bound'
        lines.append(f"verdict: FAIL ({v['failed']} of {v['rows']} outside bounds{extra})")
        lines.append(f"next: refit with crane fit {res['bundle']} --out DIR, then rerun the sim with --card DIR/profile.json"
                     if v['failed'] or v['gate_not_identified'] else
                     f"next: run the same tests in sim first: crane run --astro-test all --tests {res['bundle']}")
    return '\n'.join(lines) + '\n'


# ---- A/B and tiers ----------------------------------------------------------------------------------------

def pose_diff(a, b):
    """Position RMS, final, max (m) and heading RMS (rad) of B against A on A's frame and clock."""
    from crane import frames
    pa = frames.project_planar(a)
    pb = frames.to_common(frames.project_planar(b), pa.frame)
    pa, pb = frames.align(pa, pb)
    out = {}
    for agent in pa.agents():
        if f'{agent}.x' not in pb.columns:
            continue
        d = np.hypot(pa.columns[f'{agent}.x'] - pb.columns[f'{agent}.x'],
                     pa.columns[f'{agent}.y'] - pb.columns[f'{agent}.y'])
        dy = frames.wrap(pa.columns[f'{agent}.yaw'] - pb.columns[f'{agent}.yaw'])
        out[agent] = {'pos_rms_m': float(np.sqrt(np.mean(d * d))), 'final_m': float(d[-1]),
                      'max_m': float(d.max()), 'yaw_rms_rad': float(np.sqrt(np.mean(dy * dy))),
                      'overlap_s': float(pa.t[-1] - pa.t[0]), 'samples': int(len(pa.t))}
    return out


def numeric(metrics):
    if not metrics:
        return {}
    if metrics.get('test') and metrics.get('type'):
        return {k: _mean(v) for k, v in flatten(metrics).items()}
    return {k: float(v) for k, v in metrics.items() if isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(v)}


def compare_ab(a_root, b_root, only=None):
    from crane.frames import FrameError
    A, B = find_runs(a_root), find_runs(b_root)
    if len(A) == 1 and len(B) == 1:
        A, B = {'.': next(iter(A.values()))}, {'.': next(iter(B.values()))}
    rows = []
    for rel in sorted(set(A) & set(B)):
        a, b = A[rel], B[rel]
        row = {'run': rel, 'a': a.run_id, 'b': b.run_id}
        if a.has_traj and b.has_traj:
            try:
                row['pose'] = pose_diff(a, b)
            except FrameError as error:
                row['pose_refused'] = str(error)
        ma, mb = numeric(a.metrics), numeric(b.metrics)
        row['metrics'] = {k: {'a': ma[k], 'b': mb[k], 'delta': mb[k] - ma[k]} for k in sorted(set(ma) & set(mb))
                          if not only or any(k == o or k.startswith(o + '.') for o in only)}
        rows.append(row)
    if not rows:
        raise CompareError(f'no run under {a_root} has a match (same path) under {b_root}')
    return {'pairs': rows, 'unmatched': {'a': sorted(set(A) - set(B)), 'b': sorted(set(B) - set(A))}}


def _scored(rec, values):
    from judge.metric import MetricValue
    valid = rec.metrics is not None
    shim = SimpleNamespace(validity=SimpleNamespace(valid=valid, excluded_from=frozenset(), flags={}),
                           tier=rec.meta.get('tier'))
    from judge.compare import Scored
    return Scored(shim, {k: MetricValue(v) for k, v in values.items()}, outcome=str((rec.metrics or {}).get(
        'outcome', 'unknown')))


def compare_tiers(low_root, high_root, only=None, direction='higher', band=None, fidelity=None):
    from judge.compare import tier_agreement, tier_gap
    from judge.task import Objective
    L, H = find_runs(low_root), find_runs(high_root)
    lv, hv = {}, {}
    for runs, into in ((L, lv), (H, hv)):
        for rel, rec in runs.items():
            variant = rel.split('/', 1)[0]
            for k, v in numeric(rec.metrics).items():
                into.setdefault(k, {}).setdefault(variant, []).append(v)
    names = sorted(set(lv) & set(hv))
    if only:
        names = [n for n in names if any(n == o or n.startswith(o + '.') for o in only)]
    if not names:
        raise CompareError(f'no metric is on both tiers ({low_root}, {high_root})')
    agreement = {n: tier_agreement(lv[n], hv[n], direction) for n in names}
    pairs = [(rel, _scored(L[rel], numeric(L[rel].metrics)), _scored(H[rel], numeric(H[rel].metrics)))
             for rel in sorted(set(L) & set(H))]
    gap = tier_gap(pairs, [Objective(n, direction=direction, band=band) for n in names],
                   unit=lambda s: id(s)) if pairs else {'n_pairs': 0}
    out = {'tiers': {'low': _tier_of(L), 'high': _tier_of(H)}, 'agreement': agreement, 'gap': gap}
    if fidelity:
        out['fidelity'] = fidelity_score(H, fidelity)
    return out


def _tier_of(runs):
    return sorted({str(r.meta.get('tier')) for r in runs.values()})


def fidelity_score(high_runs, config):
    """``fastsim.fidelity.score`` of CONFIG's vehicle on the HIGH runs' goal windows."""
    from fastsim import config as fsconfig, fidelity
    from fastsim.plant import Limits
    from crane import frames
    cfg, base = fsconfig.load(Path(config))
    segments = []
    for rec in high_runs.values():
        if not rec.has_traj:
            continue
        ps = frames.project_planar(rec)
        agent = ps.agents()[0]
        x, y, yaw = (np.asarray(ps.columns[f'{agent}.{k}']) for k in ('x', 'y', 'yaw'))
        vx, vy = np.gradient(x, ps.t), np.gradient(y, ps.t)
        wz = np.gradient(np.unwrap(yaw), ps.t)
        c, s = np.cos(yaw), np.sin(yaw)
        track = np.stack([ps.t, x, y, yaw, c * vx + s * vy, -s * vx + c * vy, wz], -1).tolist()
        goals = [e for e in rec.events if str(e.get('kind', '')).startswith('goal') and 'x' in e and 'y' in e]
        stops = [e['t'] for e in rec.events if e.get('kind') in ('pause', 'override', 'stop')]
        segments += fidelity.windows(track, [{**g, 'yaw': g.get('yaw', 0.0)} for g in goals], stops)
    if not segments:
        return {'n': 0, 'note': 'no goal events with x, y in the high runs'}
    return fidelity.score(segments, Limits.from_dict(cfg['limits']), cfg['vehicle'])


# ---- output ---------------------------------------------------------------------------------------------

def _f(v, nd=4):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.{nd}g}'
    return str(v)


def markdown(kind, res):
    lines = []
    if kind == 'field':
        for test, variants in res.items():
            for variant, body in variants.items():
                if 'rows' not in body:
                    lines.append(f"\n### {test} ({body.get('field_result', '')}): {body.get('note', '')}")
                    continue
                lines.append(f"\n### {test} ({body.get('field_result', '')}) vs sim `{variant}`\n")
                lines.append('| metric | unit | field | sim | delta (sim - field) | 95% CI | over |')
                lines.append('|---|---|---|---|---|---|---|')
                for r in body['rows']:
                    ci = f"[{_f(r['ci'][0])}, {_f(r['ci'][1])}]" if r['ci'] else '-'
                    lines.append(f"| {r['metric']} | {r['unit']} | {_f(r['field'])} | {_f(r['sim'])} | "
                                 f"{_f(r['delta'])} | {ci} | {r['ci_over'] or '-'} |")
    elif kind == 'tiers':
        lines.append(f"tiers: low {res['tiers']['low']}, high {res['tiers']['high']}\n")
        lines.append('| metric | variants | low pick | high best | regret | MMRV | top-k | Kendall | bias (low - high) | '
                     '95% CI | verdict |')
        lines.append('|---|---|---|---|---|---|---|---|---|---|---|')
        for n, a in res['agreement'].items():
            g = (res['gap'].get('metrics') or {}).get(n, {})
            ci = g.get('bias_ci95')
            lines.append(f"| {n} | {a.get('n_variants')} | {a.get('low_pick', '-')} | {a.get('high_best', '-')} | "
                         f"{_f(a.get('regret'))} | {_f(a.get('mmrv'))} | {_f(a.get('topk_recall'))} | "
                         f"{_f(a.get('kendall_tau'))} | {_f(g.get('bias'))} | "
                         f"{f'[{_f(ci[0])}, {_f(ci[1])}]' if ci else '-'} | {g.get('verdict', a.get('verdict', '-'))} |")
        if 'fidelity' in res:
            lines.append(f"\nfidelity (replayed goal windows): {json.dumps(res['fidelity'])}")
    else:
        lines.append('| run | agent | pos RMS m | final m | max m | yaw RMS rad | overlap s |')
        lines.append('|---|---|---|---|---|---|---|')
        for p in res['pairs']:
            if 'pose_refused' in p:
                lines.append(f"| {p['run']} | - | refused: {p['pose_refused']} | | | | |")
            for agent, d in (p.get('pose') or {}).items():
                lines.append(f"| {p['run']} | {agent} | {_f(d['pos_rms_m'])} | {_f(d['final_m'])} | {_f(d['max_m'])} | "
                             f"{_f(d['yaw_rms_rad'])} | {_f(d['overlap_s'])} |")
        lines.append('\n| run | metric | A | B | delta (B - A) |')
        lines.append('|---|---|---|---|---|')
        for p in res['pairs']:
            for k, d in p['metrics'].items():
                lines.append(f"| {p['run']} | {k} | {_f(d['a'])} | {_f(d['b'])} | {_f(d['delta'])} |")
    return '\n'.join(lines).strip() + '\n'


def main(args):
    from crane.frames import FrameError
    bounds_arg = getattr(args, 'bounds', None)
    domain = getattr(args, 'domain', None) or 'boat'
    as_json = getattr(args, 'json', False)
    try:
        if bounds_arg and not args.field:
            raise CompareError('--bounds needs --field BUNDLE')
        if bounds_arg == 'card':
            raise CompareError('--bounds card (tolerances from the plant card\'s spread) is not yet available; '
                               f'use the file bounds: --bounds (config/fidelity/{domain}.yaml)')
        if args.field:
            if len(args.runs) != 1:
                raise CompareError('--field BUNDLE takes one RUNS root')
            if bounds_arg:
                bounds = load_bounds(bounds_path(domain) if bounds_arg == 'file' else bounds_arg)
                kind, res = 'bounds', field_bounds(args.field, args.runs[0], bounds, args.metric, getattr(args, 'fit', None))
            else:
                kind, res = 'field', compare_field(args.field, args.runs[0], args.metric)
        elif len(args.runs) != 2:
            raise CompareError('give two run sets: A B')
        elif args.tiers:
            kind, res = 'tiers', compare_tiers(*args.runs, args.metric, args.direction, args.band, args.fidelity)
        else:
            kind, res = 'ab', compare_ab(*args.runs, args.metric)
    except (CompareError, FrameError, LookupError) as error:
        print(f'crane compare: {error}', file=sys.stderr)
        return 2
    md = bounds_text(res) if kind == 'bounds' else markdown(kind, res)
    doc = {'schema': 'crane.compare/1', 'kind': kind, 'result': res}
    if as_json:
        from crane.output import dumps
        print(dumps(json.loads(json.dumps(doc, default=str))))
    else:
        sys.stdout.write(md)
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'compare.json').write_text(json.dumps(doc, indent=1, default=str) + '\n')
        (out / 'compare.md').write_text(md)
        print(f'wrote {out / "compare.json"} and {out / "compare.md"}', file=sys.stderr if as_json else sys.stdout)
    return 0 if kind != 'bounds' or res['verdict']['pass'] else 1
