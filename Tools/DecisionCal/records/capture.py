#!/usr/bin/env python3
"""Capture DecisionCal runs as judge run records (``crane.judge.run/2`` layout).

    python3 Tools/DecisionCal/records/capture.py numpy    --stratum transient --boat-index 0 --hidden-seed 4242 \
        --controller 0 --seed 1 [--projection hidden|representable|both] [--verify] --out DIR
    python3 Tools/DecisionCal/records/capture.py lockstep --fake|--live --stratum steady --boat-index 0 \
        --hidden-seed 4242 --controller 8 --seed 1 [--time-scale 4] --out DIR
    python3 Tools/DecisionCal/records/capture.py physical [--trial STEM ...] --out DIR      (default: all 37)
    python3 Tools/DecisionCal/records/capture.py convert DIR/<run_id> ... [--target v1|v2]
    python3 Tools/DecisionCal/records/capture.py index DIR          (DIR/INDEX.json from the manifests)

Each run is written to ``DIR/<run_id>/`` (run id ``<layer>-<scenario>-<controller>-s<seed>-<shortsha>``) with
the record files and ``manifest.json`` (sha256 of every other file in the directory). ``--live`` launches
Unity only through ``heavy`` admission (``heavy probe unity`` must exit 0; HEAVY_WHO=decisioncal-records).
"""

import argparse
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))     # Tools/DecisionCal

from records import _compat  # noqa: E402


def manifest(run_dir, rec, schema_path, extra=None):
    run_dir = Path(run_dir)
    files = {}
    for p in sorted(run_dir.rglob('*')):
        if p.is_file() and p.name != 'manifest.json':
            files[str(p.relative_to(run_dir))] = hashlib.sha256(p.read_bytes()).hexdigest()
    h = rec['header']
    out = {'run_id': h['run_id'], 'schema_path': schema_path, 'schema': rec['schema'], 'layer': h['backend']['layer'],
           'backend': h['backend']['name'], 'scenario': h['scenario']['id'], 'seed': h['scenario']['seed'],
           'controller': h['controller']['id'], 'window': h['window'],
           'channels': sorted(rec['channels']), 'absent': sorted(rec['absent']),
           'outcome': h['provenance'].get('outcome_from_events'), 'files': files, **(extra or {})}
    (run_dir / 'manifest.json').write_text(json.dumps(_compat.clean(out), sort_keys=True, indent=1) + '\n')
    return out


def write(rec, out_root, extra_dir=None):
    run_dir = Path(out_root) / rec['header']['run_id']
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True)
    if extra_dir is not None:
        shutil.move(str(extra_dir), str(run_dir / 'player'))
    path, _ = _compat.save(rec, run_dir)
    man = manifest(run_dir, rec, path)
    return run_dir, man


def _cal(a):
    return json.loads(Path(a.calibration_json).read_text()) if getattr(a, 'calibration_json', None) else None


def _ev(a):
    ev = {}
    for k in ('kind', 'prereg', 'seed_class'):
        v = getattr(a, f'evidence_{k}', None)
        if v is not None:
            ev[k] = v
    return ev


def summary(run_dir, man):
    o = man.get('outcome') or {}
    keys = ('docked', 't_dock', 'contacts', 'gate_miss', 'J', 'terminal')
    print(f'{run_dir}  [{man["schema_path"]}]  ' + ' '.join(f'{k}={o.get(k)}' for k in keys if k in o), flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def mission_args(p):
        p.add_argument('--stratum', default='steady')
        p.add_argument('--boat-index', type=int, default=0)
        p.add_argument('--hidden-seed', type=int, default=4242)
        p.add_argument('--controller', type=int, default=8)
        p.add_argument('--seed', type=int, default=1, help='noise seed (CRN key; also the player --crane-seed)')
        p.add_argument('--library', default='v1', choices=('v1', 'v2', 'v3'))
        p.add_argument('--t-max', type=float, default=120.0)
        p.add_argument('--calibration-json', default=None, help='{twin, arm, evidence_sha, lineage} of the selecting twin')
        p.add_argument('--evidence-kind', default=None, choices=('confirmatory', 'exploratory', 'demo', 'replay'))
        p.add_argument('--evidence-prereg', default=None)
        p.add_argument('--evidence-seed-class', default=None)
        p.add_argument('--out', required=True)

    pn = sub.add_parser('numpy', help='fast numpy plant')
    mission_args(pn)
    pn.add_argument('--projection', default='hidden', choices=('hidden', 'representable', 'both'),
                    help="'representable' = unity_check.representable_numpy(hidden boat)")
    pn.add_argument('--verify', action='store_true', help='also run mission.run directly and assert equal metrics')

    pl = sub.add_parser('lockstep', help='Unity player over the lockstep link (or the fake player)')
    mission_args(pl)
    g = pl.add_mutually_exclusive_group(required=True)
    g.add_argument('--fake', action='store_true', help="unity_check's fake player (numpy Plant behind TCP), no Unity")
    g.add_argument('--live', action='store_true', help='the real player through heavy admission')
    pl.add_argument('--start', default='0,0,0', help='--fake: world start pose x,y,yaw')
    pl.add_argument('--time-scale', type=float, default=4.0)
    pl.add_argument('--player-wall', type=float, default=400.0)
    pl.add_argument('--mission-wall', type=float, default=150.0)
    pl.add_argument('--accept-timeout', type=float, default=400.0)

    pp = sub.add_parser('physical', help='2026-10-18 open-loop pulse trials')
    pp.add_argument('--trial', action='append', default=None, help='trial stem (agent_...); repeatable; default all')
    pp.add_argument('--out', required=True)

    pi = sub.add_parser('index', help='write DIR/INDEX.json from the manifests under DIR')
    pi.add_argument('dir')

    pc = sub.add_parser('convert', help='intermediate capture dir(s) -> judge record files in place')
    pc.add_argument('dirs', nargs='+')
    pc.add_argument('--target', default=None, choices=('v1', 'v2'))

    a = ap.parse_args(argv)
    done = []
    if a.cmd == 'numpy':
        from records import numpy_capture as N
        projs = ('hidden', 'representable') if a.projection == 'both' else (a.projection,)
        for pr in projs:
            rec, _ = N.capture(a.stratum, a.boat_index, a.hidden_seed, a.controller, a.seed, library=a.library,
                               projection=pr, t_max=a.t_max, calibration=_cal(a), evidence=_ev(a), verify=a.verify)
            done.append(write(rec, a.out))
    elif a.cmd == 'lockstep':
        from records import lockstep_capture as L
        if a.fake:
            start = tuple(float(v) for v in a.start.split(','))
            rec, _ = L.capture_fake(a.stratum, a.boat_index, a.hidden_seed, a.controller, a.seed, start=start,
                                    library=a.library, t_max=a.t_max, calibration=_cal(a), evidence=_ev(a))
            done.append(write(rec, a.out))
        else:
            work = Path(a.out) / f'.work-{int(time.time() * 1000)}'
            rec, _ = L.capture_live(a.stratum, a.boat_index, a.hidden_seed, a.controller, a.seed, work, library=a.library,
                                    t_max=a.t_max, time_scale=a.time_scale, player_wall=a.player_wall,
                                    mission_wall=a.mission_wall, accept_timeout=a.accept_timeout,
                                    calibration=_cal(a), evidence=_ev(a))
            done.append(write(rec, a.out, extra_dir=work))
    elif a.cmd == 'physical':
        from records import physical_pulse as Ph
        T = Ph.trials()
        for stem in a.trial or sorted(T):
            rec, _ = Ph.build(stem, T.get(stem))
            done.append(write(rec, a.out))
    elif a.cmd == 'index':
        rows = []
        for mf in sorted(Path(a.dir).glob('*/manifest.json')):
            m = json.loads(mf.read_text())
            o = m.get('outcome') or {}
            rows.append({'path': str(mf.parent), 'run_id': m['run_id'], 'layer': m['layer'], 'backend': m['backend'],
                         'scenario': m['scenario'], 'controller': m['controller'], 'seed': m['seed'],
                         'schema_path': m['schema_path'], 'absent': m['absent'],
                         **{k: o.get(k) for k in ('docked', 't_dock', 'contacts', 'gate_miss', 'J', 'terminal')}})
        (Path(a.dir) / 'INDEX.json').write_text(json.dumps(rows, indent=1, sort_keys=True) + '\n')
        for r in rows:
            print(f"{r['layer']:10s} {r['backend']:50s} {r['scenario']:22s} {r['controller']} J={r['J']} {r['terminal']}")
        return rows
    elif a.cmd == 'convert':
        for d in a.dirs:
            out = _compat.convert(d, target=a.target)
            rec = _compat.load_intermediate(d)
            man = manifest(out, rec, 'converted-' + (a.target or ('v2' if _compat.V2 else 'v1')))
            print(out, man['schema_path'])
        return
    for run_dir, man in done:
        summary(run_dir, man)
    return done


if __name__ == '__main__':
    main()
