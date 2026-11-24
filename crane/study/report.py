"""A study's status (progress, failures, interim dev means) and its paired report (judge.stats).

    doc = status(study, out)          # crane study status
    doc = report(out)                 # crane study report: report.json + report.md beside the rows

The report pairs each arm with the baseline on the seed within every cell (rows sharing ``pair``), per metric:
mean difference (arm - baseline), a bootstrap CI over seeds, a sign-flip permutation p, and Holm across the cells
of one (stage, arm, metric). The pooled line per (stage, block, arm, metric) averages each seed's differences over
the cells first, so the seed stays the unit. A held-out block enters only once all its units are done.

Population scorers (the spec's ``scorers:``) get the rows of one group (e.g. one card x block: condition columns
and sim columns) and return named parts. ``report.population`` then ranks the cards by a calibration part and
gives the Spearman correlation of that ranking with the same part on the held-out rows, with a cluster bootstrap CI
(clusters from a condition column, e.g. cond.session_id; judge.stats.cluster_bootstrap_ci).
"""

import json
import math
from pathlib import Path

from crane.cache import UnitCache
from crane.study import exec as ex, guard, plan

IDENT = set(ex.KEYS) | set(ex.END) | set(ex.TAIL)


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _metric_names(rows, wanted=None):
    if wanted:
        return list(wanted)
    names = sorted({k for r in rows for k, v in r.items() if k not in IDENT and not k.startswith('cond.')
                    and _number(v)})
    return names


# ---- status --------------------------------------------------------------------------------------------------

def status(study, out, show=4):
    from crane import progress
    out = Path(out)
    commit = plan.code_commit()
    cache = UnitCache(out / 'cache')
    cells = plan.cells(study, None, None, out)
    stages, dev_rows = {}, []
    for cell in cells:
        st = stages.setdefault(cell.stage, {'cells': 0, 'units': 0, 'done': 0, 'blocks': {}})
        st['cells'] += 1
        name = ex._block_name(study, cell)
        b = st['blocks'].setdefault(name, {'role': cell.role, 'units': 0, 'done': 0})
        for seed in cell.seeds:
            rec = cache.get(cell.unit(seed, commit))
            st['units'] += 1
            b['units'] += 1
            if rec is not None:
                st['done'] += 1
                b['done'] += 1
                if guard.shows_means(cell.role):        # interim means: dev blocks only
                    dev_rows.append(rec['row'])
    failures = ex._failures(out, cache, commit)
    other = sum(1 for e in cache if (e.get('unit') or {}).get('commit') not in (commit, None))
    names = _metric_names(dev_rows, study.report.get('metrics') or (study.metrics or [])[:show] or None)[:show]
    means = {}
    for r in dev_rows:
        for m in names:
            if _number(r.get(m)):
                s = means.setdefault(r['stage'], {}).setdefault(r['arm'], {}).setdefault(m, [0.0, 0])
                s[0] += r[m]
                s[1] += 1
    interim = {stage: {arm: {m: {'mean': round(t / n, 6), 'n': n} for m, (t, n) in ms.items()}
                       for arm, ms in arms.items()} for stage, arms in means.items()}
    sample = None
    if failures:
        shown = [f for f in failures if guard.shows_outcomes(f.get('role'))]
        f = (shown or failures)[-1]
        if not guard.shows_outcomes(f.get('role')):   # its message may carry outcome numbers: progress only
            sample = {'cell': f.get('cell'), 'seed': f.get('seed'), 'held_out': guard.is_held_out(f.get('role')),
                      'error': f'{f.get("role") or "role-less"} unit failed (its error is in failures.jsonl)',
                      'traceback': ''}
        else:
            sample = {'cell': f.get('cell'), 'seed': f.get('seed'), 'error': f.get('error'),
                      'traceback': '\n'.join([ln for ln in (f.get('traceback') or '').rstrip().splitlines()
                                               if ln.strip(' ~^')][-5:])}
    return {'schema': 'crane.study.status/1', 'study': study.name, 'spec': str(study.path), 'out': str(out),
            'commit': commit, 'stages': stages, 'pending': plan.pending(study, out),
            'progress': progress.read(out), 'failed': len(failures), 'failure': sample,
            'cached_other_commits': other, 'interim': interim, 'interim_metrics': names}


# ---- report --------------------------------------------------------------------------------------------------

def _complete(manifests):
    """{(stage, block): done == total} summed over every manifest (a fetched cluster output has one per job)."""
    sums = {}
    for m in manifests:
        for stage, blocks in (m.get('units') or {}).items():
            for name, b in blocks.items():
                s = sums.setdefault((stage, name), [0, 0, b.get('role')])
                s[0] += b.get('done', 0)
                s[1] += b.get('total', 0)
    return sums


def _manifests(out):
    out = Path(out)
    from crane.data.rows import MANIFEST, dataset_dirs
    return [json.loads((d / MANIFEST).read_text()) for d in dataset_dirs(out)]


def analyse(rows, baseline, metrics=None, axes=(), B=2000, perm_B=10000, alpha=0.05, seed=0):
    """Paired per-cell contrasts and the pooled lines (see the module doc). Pure: rows in, dict out."""
    from judge import stats
    names = _metric_names(rows, metrics)
    cells = {}
    for r in rows:
        c = cells.setdefault((r['stage'], r['pair']), {'rows': {}, 'context': r})
        c['rows'].setdefault(r['arm'], {})[r['seed']] = r
    per_cell, families = [], {}
    pooled = {}
    for (stage, pair), c in sorted(cells.items(), key=lambda kv: kv[0]):
        base = c['rows'].get(baseline)
        if not base:
            continue
        ctxrow = c['context']
        block = plan.label(ctxrow['block'], ctxrow.get('split'))
        for arm, by_seed in sorted(c['rows'].items()):
            if arm == baseline:
                continue
            for m in names:
                seeds = sorted(s for s in by_seed if s in base and _number(by_seed[s].get(m))
                               and _number(base[s].get(m)))
                if not seeds:
                    continue
                diffs = [by_seed[s][m] - base[s][m] for s in seeds]
                mean, (lo, hi) = stats.bootstrap_ci(diffs, lambda d: sum(d) / len(d), B=B, alpha=alpha, seed=seed)
                _, p = stats.permutation_paired(diffs, B=perm_B, seed=seed)
                entry = {'stage': stage, 'pair': pair, 'block': block, 'role': ctxrow.get('role'),
                         'tier': ctxrow.get('tier'), 'condition': ctxrow.get('condition'),
                         'axes': {k: ctxrow.get(k) for k in axes},
                         'arm': arm, 'baseline': baseline, 'metric': m, 'n': len(diffs), 'diff': mean,
                         'ci': [lo, hi], 'p': p,
                         'arm_mean': sum(by_seed[s][m] for s in seeds) / len(seeds),
                         'base_mean': sum(base[s][m] for s in seeds) / len(seeds)}
                per_cell.append(entry)
                families.setdefault((stage, arm, m), {})[len(per_cell) - 1] = p
                pool = pooled.setdefault((stage, block, arm, m), {})
                for s, d in zip(seeds, diffs):
                    pool.setdefault(s, []).append(d)
    for key, ps in families.items():
        adjusted = stats.holm({str(i): (p if math.isfinite(p) else 1.0) for i, p in ps.items()})
        for i in ps:
            per_cell[i]['p_holm'] = adjusted[str(i)]
    lines = []
    for (stage, block, arm, m), by_seed in sorted(pooled.items()):
        units = [sum(v) / len(v) for _, v in sorted(by_seed.items())]
        mean, (lo, hi) = stats.bootstrap_ci(units, lambda d: sum(d) / len(d), B=B, alpha=alpha, seed=seed)
        _, p = stats.permutation_paired(units, B=perm_B, seed=seed)
        fam = [c for c in per_cell if (c['stage'], c['block'], c['arm'], c['metric']) == (stage, block, arm, m)]
        lines.append({'stage': stage, 'block': block, 'role': fam[0]['role'] if fam else None, 'arm': arm,
                      'baseline': baseline, 'metric': m,
                      'seeds': len(units), 'cells': len(fam), 'diff': mean, 'ci': [lo, hi], 'p': p,
                      'cells_holm_below_alpha': sum(1 for c in fam if c.get('p_holm', 1.0) < alpha)})
    return {'baseline': baseline, 'metrics': names, 'alpha': alpha, 'B': B, 'pooled': lines, 'cells': per_cell}


def verdicts(targets, pooled):
    """Each target ({contrast: A - B, metric, direction}) against the pooled held-out lines that pair A with B:
    met / not met / inconclusive (the 95% CI decides), or no data. ``lower`` and ``higher`` need the CI on that
    side of 0; ``not lower`` and ``not higher`` fail only when the CI is wholly on the wrong side."""
    result = []
    for t in targets:
        lines = []
        for x in pooled:
            if x.get('role') != 'held-out' or x['metric'] != t['metric']:
                continue
            if (x['arm'], x['baseline']) == (t['a'], t['b']):
                diff, lo, hi = x['diff'], x['ci'][0], x['ci'][1]
            elif (x['arm'], x['baseline']) == (t['b'], t['a']):
                diff, lo, hi = -x['diff'], -x['ci'][1], -x['ci'][0]
            else:
                continue
            below, above = hi < 0, lo > 0
            verdict = {'lower': 'met' if below else 'not met' if above else 'inconclusive',
                       'higher': 'met' if above else 'not met' if below else 'inconclusive',
                       'not lower': 'not met' if below else 'met',
                       'not higher': 'not met' if above else 'met'}[t['direction']]
            lines.append({'stage': x['stage'], 'block': x['block'], 'diff': diff, 'ci': [lo, hi],
                          'seeds': x['seeds'], 'verdict': verdict})
        why = None if lines else (f"no done held-out line pairs {t['a']} with {t['b']} on {t['metric']} (the report "
                                  f"pairs arms with the baseline; --baseline {t['b']})")
        result.append({'contrast': t['contrast'], 'metric': t['metric'], 'direction': t['direction'],
                       'lines': lines, 'verdict': 'no data' if not lines else lines[0]['verdict']
                       if len({x['verdict'] for x in lines}) == 1 else 'mixed', 'why': why})
    return result


def load_ref(study, ref):
    """A 'module:attr' or 'file.py:attr' ref of the spec, its plugin dirs on sys.path (for the enclosing
    exec.plugin_scope)."""
    import sys
    from fastsim import plugin
    for d in [str(study.base), *(str(p) for p in reversed(study.plugins))]:
        if d not in sys.path:
            sys.path.insert(0, d)
    return plugin.load(ref, str(study.base))


def _gvalue(r, key):
    if key == 'card':
        return r.get('card_sha') or f"arm:{r['arm']}"
    if key == 'block':
        return plan.label(r['block'], r.get('split'))
    return r.get(key)


def card_label(r, cards):
    sha = r.get('card_sha')
    if sha and sha in (cards or {}):
        return ','.join(f'{k}={v}' for k, v in cards[sha]['levels'].items())
    return r['arm'] + (f'@{sha[:8]}' if sha else '')


def _parts(fn, sc, rows, group):
    got = fn(rows, group, **sc['args'])
    if not isinstance(got, dict):
        raise TypeError(f'population scorer {sc["fn"]} returned {type(got).__name__}, not a dict of named parts')
    return {k: float(v) for k, v in got.items() if _number(v)}


@ex.plugin_scope
def population(rows, study, cards=None, B=200, alpha=0.05, seed=0):
    """{groups: [...], ranking: {...} | None} of the spec's population scorers over ``rows`` (see the module doc)."""
    from judge import stats
    if not study.scorers:
        return None
    fns = {sc['name']: load_ref(study, sc['fn']) for sc in study.scorers}
    groups = []
    for sc in study.scorers:
        table = {}
        for r in rows:
            table.setdefault(tuple(_gvalue(r, k) for k in sc['over']), []).append(r)
        for key, members in sorted(table.items(), key=lambda kv: tuple(map(str, kv[0]))):
            roles = {m.get('role') for m in members}
            group = {'scorer': sc['name'], 'key': dict(zip(sc['over'], key)), 'label': card_label(members[0], cards),
                     'card_sha': members[0].get('card_sha') if 'card' in sc['over'] else None,
                     'role': roles.pop() if len(roles) == 1 else None, 'n': len(members)}
            group['parts'] = _parts(fns[sc['name']], sc, members, dict(group))
            groups.append(group)
    cfg = dict(study.report.get('population') or {})
    first = study.scorers[0]
    name, _, part = str(cfg.get('part') or '').partition('.')
    if not name:
        name = first['name']
        part = next((next(iter(g['parts']), None) for g in groups if g['scorer'] == name and g['parts']), None)
    sc = next((x for x in study.scorers if x['name'] == name), None)
    if sc is None or part is None:
        return {'groups': groups, 'ranking': None}
    calib, test = cfg.get('calib', 'calib'), cfg.get('test', 'held-out')
    cluster = cfg.get('cluster', 'condition')
    order = cfg.get('order', 'min')
    fn = fns[name]
    sides = [r for r in rows if r.get('role') in (calib, test)]
    keys = [k for k in sc['over'] if k not in ('block', 'split', 'role')] or ['card']   # what is ranked

    def ident(r):
        return tuple(_gvalue(r, k) for k in keys)

    def label(r):
        return ' '.join(card_label(r, cards) if k == 'card' else str(_gvalue(r, k)) for k in keys)

    def scores(sample):
        by = {}
        for r in sample:
            by.setdefault((ident(r), 'test' if r.get('role') == test else 'calib'), []).append(r)
        got = {}
        for (card, side), members in by.items():
            v = _parts(fn, sc, members, {'scorer': name, 'key': dict(zip(keys, card)), 'role': side,
                                         'card_sha': members[0].get('card_sha'), 'label': label(members[0]),
                                         'n': len(members)}).get(part)
            if v is not None:
                got.setdefault(card, {})[side] = v
        return got

    def rho(sample):
        got = scores(sample)
        both = [c for c, v in got.items() if 'calib' in v and 'test' in v]
        if len(both) < 3:
            return float('nan')
        return stats.spearman([got[c]['calib'] for c in both], [got[c]['test'] for c in both])
    got = scores(sides)
    labels = {ident(r): label(r) for r in sides}
    sign = 1.0 if order == 'min' else -1.0
    ranked = sorted((c for c in got if 'calib' in got[c]), key=lambda c: (sign * got[c]['calib'], str(c)))
    table = [{'rank': i + 1, 'card': list(c), 'label': labels.get(c), 'calib': got[c]['calib'],
              'test': got[c].get('test')} for i, c in enumerate(ranked)]
    ranking = {'part': f'{name}.{part}', 'by': keys, 'order': order, 'calib': calib, 'test': test, 'cluster': cluster,
               'cards': table, 'spearman': None, 'ci': None, 'B': B}
    if any(t['test'] is not None for t in table):
        clusters = [(r.get('role'), r.get(cluster)) for r in sides]
        est, (lo, hi) = stats.cluster_bootstrap_ci(sides, clusters, rho, B=B, alpha=alpha, seed=seed,
                                                   strata=[r.get('role') for r in sides])
        ranking.update(spearman=est if _number(est) else None, ci=[lo, hi] if _number(lo) else None,
                       clusters=len(set(clusters)))
    else:
        ranking['why'] = f'no done {test} rows yet: the ranking only'
    return {'groups': groups, 'ranking': ranking}


def report(out, baseline=None, metrics=None, B=2000, alpha=0.05, write=True, study=None):
    """The paired report of OUT's rows (one study output, or a dir of job outputs). Held-out blocks enter only when
    done. ``study`` (its spec) adds the population scorers. Writes OUT/report.json and OUT/report.md; returns the
    JSON document."""
    from crane.data.rows import read_rows
    out = Path(out)
    manifests = _manifests(out)
    if not manifests:
        raise FileNotFoundError(f'{out} holds no manifest.json')
    complete = _complete(manifests)
    waiting = {f'{stage}:{name}': {'done': done, 'total': total} for (stage, name), (done, total, role)
               in sorted(complete.items()) if role == 'held-out' and done < total}
    rows = []
    for r in read_rows(out):
        if r.get('role') == 'held-out' and f"{r['stage']}:{plan.label(r['block'], r.get('split'))}" in waiting:
            continue
        rows.append(r)
    study_name = manifests[0].get('study')
    baseline = baseline or manifests[0].get('baseline') or (rows[0]['arm'] if rows else None)
    doc = {'schema': 'crane.study.report/1', 'study': study_name, 'out': str(out), 'rows': len(rows),
           'held_out_waiting': waiting,
           'touched_held_out': any(m.get('touched_held_out') for m in manifests),
           **analyse(rows, baseline, metrics, manifests[0].get('axes') or (), B=B, alpha=alpha)}
    held = next((m['held_out'] for m in manifests if (m.get('held_out') or {}).get('targets')), None)
    doc['targets'] = verdicts(held['targets'].get('entries') or [], doc['pooled']) if held else []
    if study is not None and study.scorers:
        cards = {}
        for m in manifests:
            cards.update(m.get('cards') or {})
        cfg = study.report.get('population') or {}
        doc['population'] = population(rows, study, cards, B=int(cfg.get('B', 200)), alpha=alpha)
    if write:
        from crane import progress
        progress.write_atomic(out / 'report.json', json.dumps(doc, indent=1, default=str) + '\n')
        progress.write_atomic(out / 'report.md', markdown(doc))
    return doc


def fmt(v, digits=4):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '-'
    return f'{v:.{digits}g}'


def pooled_rows(doc):
    head = ('stage', 'block', 'arm', 'metric', 'seeds', 'cells', 'diff', '95% CI', 'p', 'cells p_holm<a')
    body = [(x['stage'], x['block'], x['arm'], x['metric'], x['seeds'], x['cells'],
             fmt(x['diff']), f"[{fmt(x['ci'][0])}, {fmt(x['ci'][1])}]", fmt(x['p'], 3),
             f"{x['cells_holm_below_alpha']}/{x['cells']}") for x in doc['pooled']]
    return [head, *body]


def markdown(doc):
    lines = [f"# {doc['study']}: paired report", '',
             f"Baseline `{doc['baseline']}`; each arm minus the baseline, paired on the seed within each cell. "
             f"{doc['rows']} rows. Bootstrap B={doc['B']}, sign-flip permutation, Holm across cells.", '']
    if doc['held_out_waiting']:
        lines += ['Held-out blocks not done (progress only): ' + ', '.join(
            f"{k} {v['done']}/{v['total']}" for k, v in doc['held_out_waiting'].items()), '']
    if doc.get('targets'):
        lines += ['## Held-out targets', '', '| contrast | metric | direction | verdict |', '|---|---|---|---|']
        lines += [f"| {t['contrast']} | {t['metric']} | {t['direction']} | {t['verdict']} |" for t in doc['targets']]
        lines += ['']
    rk = (doc.get('population') or {}).get('ranking')
    if rk:
        rho = (f"Spearman {fmt(rk['spearman'])} [{fmt(rk['ci'][0])}, {fmt(rk['ci'][1])}] over {rk.get('clusters')} "
               f"{rk['cluster']} clusters (B={rk['B']})" if rk.get('ci') else rk.get('why') or 'Spearman -')
        lines += [f"## Cards by {rk['part']} ({rk['order']} first on {rk['calib']})", '', rho, '',
                  f"| rank | card | {rk['calib']} | {rk['test']} |", '|---|---|---|---|']
        lines += [f"| {t['rank']} | {t['label']} | {fmt(t['calib'])} | {fmt(t['test'])} |" for t in rk['cards']]
        lines += ['']
    table = pooled_rows(doc)
    lines += ['## Pooled', '', '| ' + ' | '.join(table[0]) + ' |', '|' + '---|' * len(table[0])]
    lines += ['| ' + ' | '.join(str(c) for c in row) + ' |' for row in table[1:]]
    lines += ['', '## Cells', '', '| stage | block | cell | arm | metric | n | diff | 95% CI | p | p_holm |',
              '|---|---|---|---|---|---|---|---|---|---|']
    for c in doc['cells']:
        where = ' '.join([*(f'{k}={v}' for k, v in c['axes'].items()), *([f"cond={c['condition']}"]
                                                                          if c['condition'] else [])]) or c['pair']
        lines.append(f"| {c['stage']} | {c['block']} | {where} | {c['arm']} | {c['metric']} | {c['n']} | "
                     f"{fmt(c['diff'])} | [{fmt(c['ci'][0])}, {fmt(c['ci'][1])}] | {fmt(c['p'], 3)} | "
                     f"{fmt(c.get('p_holm'), 3)} |")
    return '\n'.join(lines) + '\n'
