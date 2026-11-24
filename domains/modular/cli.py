"""crane run --domain modular: the waypoint route per seed, or logged command timelines replayed (--replay).

    crane run --domain modular --seeds 0:64                                   # route:square, mecanum-1
    crane run --domain modular --robot mecanum-pair --env route:random --seeds 0:256
    crane run --domain modular --replay TIMELINES.npz --seeds 0:24 --card example#plated \\
        --set faults.p_healthy=0.25 --set faults.p_double=0.2 --set dr=true

``--set`` keys: card.KEY (profile overrides, e.g. card.scale.stall=1.1), faults.KEY (the fault prior,
domains/modular/batch.py), record.obs.KEY (channels, hz, dtype, window), dr (true: per-seed draws in the card's
dr ranges), payload.mass_kg / payload.at. Outputs under --out: ``metrics/`` (one row per env) and, for a replay or
with record set, ``obs/`` (one row per env: the T x C channels), both crane.data datasets, plus ``run.json``.
"""

import json
import time
from collections import Counter
from pathlib import Path

import numpy as np

from crane import output, select
from domains.modular import assembly as AS, batch as B, profile as PR, route as RT

SCHEMA = 'crane.modular.run/1'


def _flat(d, prefix=''):
    out = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            out.update(_flat(v, f'{prefix}{k}.'))
        else:
            out[f'{prefix}{k}'] = v
    return out


def _resolve(args, cfg, tier):
    t = cfg['tiers'][tier]
    dt, tick_hz = float(t.get('dt_s', 0.005)), float(t.get('tick_hz', 50.0))
    nsub = int(round(1.0 / (tick_hz * dt)))
    if abs(nsub * dt * tick_hz - 1.0) > 1e-9:
        raise B.BatchError(f'tiers.{tier}: tick_hz {tick_hz} is not a whole number of dt_s {dt} steps')
    sets = select.parse_sets(args.set or [])
    unknown = sorted(set(sets) - {'card', 'faults', 'record', 'dr', 'payload'})
    if unknown:
        raise B.BatchError(f'--set {unknown[0]}: unknown here; keys: card.*, faults.*, record.*, dr, payload.*')
    robots = (cfg.get('robot') or {}).get('robots') or {}
    robot = getattr(args, 'robot', None) or (cfg.get('robot') or {}).get('default')
    if robot not in robots:
        raise B.BatchError(f'no robot {robot!r} in the modular domain; robots: {", ".join(robots)}')
    cards = list(getattr(args, 'card', None) or [])
    ref = cards[0] if cards else (cfg.get('card') or {}).get('default')
    prof = PR.load(ref, _flat(sets.get('card')))
    record = sets.get('record')
    spec = cfg.get('record') if record is None else {'obs': {**((cfg.get('record') or {}).get('obs') or {}),
                                                             **(record.get('obs') or {})}}
    return {'dt': dt, 'tick_hz': tick_hz, 'nsub': nsub, 'robot': robot, 'assembly': robots[robot].get('assembly', robot),
            'card': ref, 'prof': prof, 'faults': sets.get('faults'), 'dr': bool(sets.get('dr', False)),
            'payload': sets.get('payload'), 'record': spec, 'record_asked': record is not None}


def _write_metrics(out, rows):
    from crane.data import rows as R
    if not rows:
        return None
    w = R.RowWriter(out / 'metrics', R.infer_schema(rows), fmt='auto', part='part-0000')
    for r in rows:
        w.write(r)
    return R.write_manifest(out / 'metrics', w.schema, w.close(), {'kind': 'metrics'})


def _write_obs(out, rec, arrays, t0, envs, keys, lengths):
    from crane.data import recorder
    store = recorder.Store(out / 'obs', part='part-0000', fmt='npz')
    for e, key in enumerate(keys):
        L = None if lengths is None else int(np.ceil(lengths[e] / rec.every))
        store.add(key, rec.env(arrays[e], t0, envs.labels[e], L))
    return store.close()['obs']


def run(args, out_, cfg, tier):
    """The runner of config/domains/modular.yaml's tier: --replay FILE, else the route task. Exit code."""
    try:
        r = _resolve(args, cfg, tier)
        seeds = select.parse_seeds(args.seeds or '0', args.domain)
        backend = args.backend or 'numba'
        replay = getattr(args, 'replay', None)
        if backend == 'jax' and not replay:
            raise B.BatchError('--backend jax runs --replay only; the route task runs numpy or numba')
        if r['record_asked'] and not replay:
            raise B.BatchError('--set record.*: channels are recorded on --replay (the route task writes metrics)')
        if replay:
            tl = B.timelines(Path(replay).expanduser(), r['tick_hz'])
            env_names = tl.get('assembly') or r['assembly']
        else:
            kind, _, name = (getattr(args, 'env', None) or (cfg.get('env') or {}).get('default', 'route:square')).partition(':')
            if kind != 'route':
                raise B.BatchError(f'--env {kind}:{name}: the modular domain has route:NAME (square, random)')
            routes = [RT.load(name, s) for s in seeds]
    except (B.BatchError, PR.ProfileError, AS.AssemblyError, select.SelectError, FileNotFoundError, ValueError) as error:
        return out_.usage('run', str(error), 'crane run --domain modular --help')
    dest = Path(args.out or select.default_out('modular', tier)).expanduser()
    plan = {'schema': 'crane.run.plan/1', 'domain': 'modular', 'tier': tier, 'task': 'replay' if replay else 'route',
            'robot': r['robot'], 'card': r['card'], 'seeds': seeds, 'backend': backend, 'dt_s': r['dt'],
            'tick_hz': r['tick_hz'], 'dr': r['dr'], 'faults': r['faults'] or 'none', 'out': str(dest)}
    if replay:
        plan.update(timelines=str(replay), M=int(tl['cmd'].shape[0]), ticks=int(tl['length'].max()),
                    envs=int(tl['cmd'].shape[0]) * len(seeds))
    if out_.dry_run:
        if not out_.doc(plan):
            out_.say('crane run (dry run): nothing runs')
            out_.table([(k, v if not isinstance(v, list) or len(v) <= 12 else f'{v[:6]} ... {len(v)} seeds')
                        for k, v in plan.items() if k != 'schema'])
        return output.OK
    try:
        if replay:
            envs = B.Envs.build(env_names, r['prof'], seeds, tl['cmd'].shape[0], r['faults'], r['dr'], r['dt'],
                                logged=tl if 'mode' in tl else None, payload=r['payload'])
        else:
            envs = B.Envs.build(r['assembly'], r['prof'], seeds, 1, r['faults'], r['dr'], r['dt'],
                                payload=r['payload'])
    except (B.BatchError, PR.ProfileError, AS.AssemblyError) as error:
        return out_.usage('run', str(error), 'crane run --domain modular --help')
    from engine.kernels import mecanum as K, mecanum_run as KR
    dest.mkdir(parents=True, exist_ok=True)
    E, W = envs.p.shape[0], K.wheels_of(envs.p)
    started = time.perf_counter()
    if replay:
        res = KR.replay(envs.p, tl['cmd'], envs.which, tl['length'], envs.delay, r['dt'], r['nsub'], backend,
                        args.dtype or 'float64')
        T = res['cmd'].shape[1]
        keys = [{'timeline': int(envs.which[e]), 'timeline_id': tl['id'][envs.which[e]], 'seed': int(seeds[e % len(seeds)])}
                for e in range(E)]
        disp = np.linalg.norm(res['pose'][np.arange(E), tl['length'][envs.which] - 1, :2], axis=1)
        metrics = [{**k, 'ticks': int(tl['length'][envs.which[e]]), 'displacement_m': float(disp[e]),
                    'faulty_wheels': sum(m != 'ok' for m in envs.labels[e]['fault_mode'])} for e, k in enumerate(keys)]
        lengths = tl['length'][envs.which]
    else:
        res, T = RT.drive(envs, routes, r['dt'], r['nsub'], backend)
        keys = [{'seed': int(s)} for s in seeds]
        metrics = [{**keys[e], 'outcome': str(res['outcome'][e]), **{k: float(res[k][e]) for k in
                    ('time_s', 'path_m', 'track_rms_m', 'corridor_excess_m', 'corridor_out_s')}} for e in range(E)]
        lengths = None
    wall = time.perf_counter() - started
    files = {'metrics': _write_metrics(dest, metrics)}
    channels = None
    if replay:
        rec = B.Recording(r['record'], r['tick_hz'], W)
        a, t0 = rec.arrays(res)
        files['obs'] = _write_obs(dest, rec, a, t0, envs, keys, lengths)
        channels = rec.meta()['obs']
    doc = {'schema': SCHEMA, **{k: v for k, v in plan.items() if k not in ('schema', 'out')}, 'out': str(dest),
           'n': E, 'wheels': W, 'wall_s': round(wall, 3), 'envs_per_s': round(E / wall, 3) if wall > 0 else 0.0,
           'sim_s': round(T * r['nsub'] * r['dt'], 3), 'card_numbers': PR.card(r['prof']),
           'provenance': {'kernel': 'engine/kernels/mecanum.py', 'reference': 'domains/modular/truth.py'},
           'channels': channels, 'outcomes': dict(Counter(m.get('outcome', 'REPLAYED') for m in metrics)),
           'files': {k: (v or {}).get('files') for k, v in files.items()}}
    (dest / 'run.json').write_text(json.dumps(output.finite(doc), indent=1, default=output._default) + '\n')
    if out_.doc(doc):
        return output.OK
    sim = T * r['nsub'] * r['dt']
    if replay:
        out_.line(f"replay   {plan['M']} timelines x {len(seeds)} seeds = {E} envs, {T} ticks at {r['tick_hz']:g} Hz "
                  f"({sim:.1f} s), {backend} {args.dtype or 'float64'}")
    else:
        out_.line(f"route    {name} x {E} seeds, {backend}: {doc['outcomes']}")
    out_.line(f"card     {r['card']}   robot {r['robot']} ({W} wheels)   latency {int(envs.delay.max())} steps   "
              f"dr {'on' if r['dr'] else 'off'}   faults {'prior' if r['faults'] else ('logged' if replay and 'mode' in tl else 'none')}")
    out_.say(f'done     {E} envs in {wall:.1f} s wall: {E / wall:.1f} envs/s ({E * sim / wall:.0f} sim-s/s)')
    out_.line(f'wrote    {dest}/metrics' + (f', {dest}/obs ({len(channels["channels"])} channels at '
                                           f'{channels["hz"]:g} Hz, {channels["dtype"]})' if channels else '')
              + f', {dest}/run.json')
    out_.next(f'crane data schema {dest}/{"obs" if channels else "metrics"}')
    return output.OK
