"""Fidelity loop against a twin (another simulator or the field): score a calibration epoch, refit.

One pass: the domain turns new twin sessions into goal segments -> the fixed epoch is scored on
them before they enter any fit (prequential: each batch is scored once, by an epoch fit only on
strictly older sessions) -> optionally a new epoch is fitted on every scored session -> a
trust-trend row is appended. A segment is one goal: the vehicle replays it from the twin's state
with the goal as a one-pose route over its window; its score is the RMS position error against
the twin's track. Epoch files are never edited: a refit writes the next epoch file beside it.

A segment is {t0, goal: [x, y, yaw], precise, track: [[t, x, y, yaw, vx, vy, wz], ...]}; a
batch is {session, segments, cuts}. Session ids must sort in time order (e.g. YYYYMMDD_HHMMSS).
``windows`` cuts a twin's track into segments; reading the twin's logs is the domain's part.
"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import statistics

import yaml

from fastsim.identity import sha256
from fastsim.plant import Goal, vehicle

HORIZON_S, DT, MIN_SAMPLES = 30.0, 0.05, 20


def windows(track, goals, stops, horizon_s=HORIZON_S, min_samples=MIN_SAMPLES):
    """Goal segments of one twin run: a goal's window ends at the next goal, the next stop
    (pause, override, suspended task ...) or ``horizon_s`` after it was sent. ``goals`` are
    [{t, x, y, yaw, precise, ...}] and ``track`` rows [t, x, y, yaw, vx, vy, wz], one clock."""
    track, stops, segments = sorted(track), sorted(stops), []
    for k, g in enumerate(goals):
        t0 = g['t']
        t1 = min([t0 + horizon_s] + [s for s in stops if s > t0]
                 + ([goals[k + 1]['t']] if k + 1 < len(goals) else []))
        window = [list(r) for r in track if t0 <= r[0] <= t1]
        if len(window) >= min_samples:
            segments.append({'task': g.get('task'), 't0': t0, 'goal': [g['x'], g['y'], g.get('yaw', 0.0)],
                             'precise': bool(g.get('precise')), 'track': window})
    return segments


def replay(segment, limits, spec, dt=DT):
    """(RMS position error m, fast path m, twin path m) of one segment under vehicle ``spec``."""
    t, x, y, yaw, vx, vy, wz = segment['track'][0]
    tracker, plant = vehicle(spec, limits, (x, y, yaw), (vx, vy, wz))
    goal = Goal([segment['goal']], segment['precise'])
    err2, fast = 0.0, 0.0
    for ts, ux, uy, *_ in segment['track']:
        while t < ts - 1e-9:
            moved = plant.step(tracker.command(goal, plant.pose, t), dt)
            fast += math.hypot(*moved)
            t += dt
        err2 += (plant.pose[0] - ux) ** 2 + (plant.pose[1] - uy) ** 2
    twin = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(segment['track'], segment['track'][1:]))
    return math.sqrt(err2 / len(segment['track'])), fast, twin


def score(segments, limits, spec, dt=DT):
    rows = [replay(s, limits, spec, dt) for s in segments]
    if not rows:
        return {'n': 0}
    rms = [r[0] for r in rows]
    return {'n': len(rows), 'rms_mean': round(statistics.fmean(rms), 3),
            'rms_median': round(statistics.median(rms), 3),
            'path_abslog_median': round(statistics.median(
                abs(math.log(max(f, 0.05) / max(u, 0.05))) for _, f, u in rows), 3),
            'per_segment_rms': [round(v, 3) for v in rms]}


def with_values(spec, names, values):
    """A copy of a vehicle spec with dotted parameters set (``delay_s.0`` is an index)."""
    out = json.loads(json.dumps(spec))
    for name, value in zip(names, values):
        *path, last = name.split('.')
        node = out
        for part in path:
            node = node[part]
        if isinstance(node, list):
            node[int(last)] = value
        else:
            node[last] = value
    return out


def _get(spec, name):
    node = spec
    for part in name.split('.'):
        node = node[int(part)] if isinstance(node, list) else node[part]
    return float(node)


def _pattern_search(f, x0, bounds, step=0.25, tol=0.01, iters=200):
    """Bounded coordinate pattern search (deterministic; no dependencies)."""
    x, fx = list(x0), f(x0)
    steps = [step * (hi - lo) for lo, hi in bounds]
    for _ in range(iters):
        if max(s / (hi - lo) for s, (lo, hi) in zip(steps, bounds)) < tol:
            break
        improved = False
        for i, (lo, hi) in enumerate(bounds):
            for sign in (1, -1):
                trial = list(x)
                trial[i] = min(hi, max(lo, x[i] + sign * steps[i]))
                ft = f(trial)
                if ft < fx - 1e-9:
                    x, fx, improved = trial, ft, True
                    break
        if not improved:
            steps = [s / 2 for s in steps]
    return x, fx


def refit(segments, limits, start, bounds, dt=DT):
    """Fit the ``bounds`` parameters ({dotted name: [lo, hi]}) on mean segment RMS from ``start``:
    bounded Powell where scipy is installed, else a bounded pattern search."""
    names = list(bounds)
    box = [tuple(bounds[n]) for n in names]
    x0 = [min(hi, max(lo, _get(start, n))) for n, (lo, hi) in zip(names, box)]

    def cost(v):
        return score(segments, limits, with_values(start, names, v), dt)['rms_mean']
    try:
        from scipy.optimize import minimize
        res = minimize(cost, x0, method='Powell', bounds=box, options={'xtol': 0.01, 'ftol': 1e-3})
        x, fx, method = list(res.x), float(res.fun), 'powell'
    except ImportError:
        (x, fx), method = _pattern_search(cost, x0, box), 'pattern'
    v = [round(float(a), 3) for a in x]
    edges = [n for n, a, b in zip(names, v, box) if min(abs(a - e) for e in b) < 0.01]
    return with_values(start, names, v), edges, {'start': cost(x0), 'fit': round(fx, 3), 'method': method}


def _binom_cdf(k, n, p):
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k + 1))


def _bisect(f, lo=0.0, hi=1.0):
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) else (lo, mid)
    return (lo + hi) / 2


def clopper_pearson(k, n, alpha):
    """Exact two-sided (1 - alpha) interval of a binomial proportion k/n."""
    lower = 0.0 if k == 0 else _bisect(lambda p: 1 - _binom_cdf(k - 1, n, p) < alpha / 2)
    upper = 1.0 if k == n else _bisect(lambda p: _binom_cdf(k, n, p) > alpha / 2)
    return lower, upper


def duration_band(ratios, look, band=(0.67, 1.5)):
    """T_n: the share of fast/twin duration ratios outside ``band``, with Clopper-Pearson at
    alpha 0.05 / (n (n + 1)) for look n (an always-valid sequence of looks)."""
    out = sum(not band[0] <= r <= band[1] for r in ratios)
    return {'pairs': len(ratios), 'outside': out, 'look': look,
            'bounds': clopper_pearson(out, len(ratios), 0.05 / (look * (look + 1))) if ratios else None}


# ---- the prequential loop: epochs, the ledger, the trust trend ----------------------------------

def _lines(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()] if path.exists() else []


def next_epoch_path(path):
    """``name-eN.yaml`` -> ``name-e(N+1).yaml`` beside it."""
    path = Path(path)
    stem = re.sub(r'\d+$', lambda m: str(int(m.group()) + 1), path.stem)
    return path.with_name(stem + path.suffix)


def write_epoch(path, epoch, basis, plant, fit):
    if Path(path).exists():
        raise FileExistsError(f'{path}: an epoch file is never rewritten')
    head = (f'# Fast-sim plant calibration against the {basis} twin, epoch {epoch} (fidelity refit).\n'
            '# Never edit a fitted file: a refit is a new epoch. EFFECTIVE lumped controller + plant\n'
            '# numbers for the fast tier\'s stand-in tracker, not physical constants.\n')
    Path(path).write_text(head + yaml.safe_dump({'epoch': epoch, 'basis': basis, 'fit': fit, 'plant': plant},
                                                sort_keys=False, width=110))


def pair_ratios(root, sessions, epoch_sha, session_of, match=None):
    """fast/twin ``run.duration_s`` of the pairs archived under ``root`` (pairs.py) whose twin
    session (``session_of(index row)``) is in ``sessions`` and whose fast run used this epoch;
    ``match`` {session: predicate(row)} keeps one twin per session (e.g. the fast run on the
    controller code the twin ran)."""
    ratios = []
    for row in _lines(Path(root) / 'index.jsonl'):
        name = session_of(row)
        cal = ((row.get('fast') or {}).get('calibration') or {})
        keep = (match or {}).get(name, lambda r: True)
        if name not in sessions or cal.get('sha256') != epoch_sha or not keep(row):
            continue
        diff = Path(root) / 'pairs' / row['pair_id'] / 'diff.json'
        duration = json.loads(diff.read_text()).get('run.duration_s', {})
        if duration.get('twin'):
            ratios.append(duration['fast'] / duration['twin'])
    return ratios


def keep(store, batch):
    """Store sessions' segments (an epoch's own fit sessions go in this way, never scored)."""
    (Path(store) / 'segments').mkdir(parents=True, exist_ok=True)
    for b in batch:
        (Path(store) / 'segments' / f"{b['session']}.json").write_text(json.dumps(b))


def score_batch(epoch_path, store, batch, limits, uncalibrated, bounds=None, ratios=None,
                session_key=str, note='', dt=DT):
    """Score the epoch on a new batch [{session, segments, cuts}] once; with ``bounds`` refit the
    next epoch on every scored session (their segments kept in ``store``). Returns the ledger row;
    raises ValueError for a batch already scored or not newer than the epoch's own data."""
    epoch_path, store = Path(epoch_path), Path(store)
    epoch = yaml.safe_load(epoch_path.read_text())
    epoch_sha = sha256(epoch_path.read_bytes())
    basis = [session_key(s) for s in (epoch.get('fit') or {}).get('sessions', [])]
    ledger = store / 'ledger.jsonl'
    scored = _lines(ledger)
    seen = {s for r in scored for s in r['sessions']} | set(basis)
    newest = max(basis) if basis else ''
    for b in batch:
        if b['session'] in seen:
            raise ValueError(f"{b['session']} was already scored or fitted on; a batch is scored once")
        if b['session'] <= newest:
            raise ValueError(f"{b['session']} is not newer than epoch {epoch['epoch']}'s data ({newest})")
    keep(store, batch)
    segments = [s for b in batch for s in b['segments']]
    names = [b['session'] for b in batch]
    look = 1 + sum(r['epoch_sha256'] == epoch_sha for r in scored)
    row = {'scored_at': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%MZ'), 'sessions': names,
           'epoch': epoch['epoch'], 'epoch_sha256': epoch_sha,
           'score': score(segments, limits, epoch['plant'], dt),
           'uncalibrated': score(segments, limits, uncalibrated, dt),
           'duration_band': duration_band(ratios or [], look),
           'cuts': {b['session']: b['cuts'] for b in batch if b.get('cuts')}, 'note': note}
    if bounds:
        fit_sessions = sorted(seen | set(names))
        missing = [n for n in fit_sessions if not (store / 'segments' / f'{n}.json').exists()]
        if missing:
            raise ValueError(f'ingest {missing} into the store before a refit')
        fit_segments = [s for n in fit_sessions
                        for s in json.loads((store / 'segments' / f'{n}.json').read_text())['segments']]
        plant, edges, rms = refit(fit_segments, limits, epoch['plant'], bounds, dt)
        out = next_epoch_path(epoch_path)
        write_epoch(out, epoch['epoch'] + 1, epoch.get('basis'), plant, {
            'method': f"goal-segment replay, {rms['method']}, mean RMS position error over each goal window",
            'sessions': fit_sessions, 'segments': len(fit_segments), 'rms_mean_m': rms,
            'bounds': {k: list(v) for k, v in bounds.items()}, 'on_bound': edges})
        row['refit'] = f'{out.name} {plant}' + (f' on bounds {edges}' if edges else '')
    with ledger.open('a') as f:
        f.write(json.dumps(row) + '\n')
    return row


def trend_row(note, row):
    """Append the row to a markdown note's trust-trend table (created on first use)."""
    note = Path(note)
    rt, base, sc = row['duration_band'], row['uncalibrated'], row['score']
    band = ('no pairs' if not rt['pairs'] else f"{rt['outside']}/{rt['pairs']} [{rt['bounds'][0]:.2f}, "
            f"{rt['bounds'][1]:.2f}] look {rt['look']}")
    text = note.read_text() if note.exists() else ''
    if '## Fidelity trust trend' not in text:
        text += ('\n## Fidelity trust trend (fastsim.fidelity; prequential, one row per scored batch)\n'
                 '| scored (UTC) | sessions | epoch | segments | RMS m mean/median (uncal.) '
                 '| path abs log median (uncal.) | duration band T_n | refit, note |\n|---|---|---|---|---|---|---|---|\n')
    text += (f"| {row['scored_at']} | {' '.join(row['sessions'])} | {row['epoch']} | {sc.get('n', 0)} "
             f"| {sc.get('rms_mean')}/{sc.get('rms_median')} ({base.get('rms_mean')}/{base.get('rms_median')}) "
             f"| {sc.get('path_abslog_median')} ({base.get('path_abslog_median')}) | {band} "
             f"| {row.get('refit') or '-'}{' ' + row['note'] if row['note'] else ''} |\n")
    note.write_text(text)
