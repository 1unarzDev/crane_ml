"""crane run --astro-test: astro's tests (still, couple, speed, steps, nav2) as T0/T1 scenarios.

    crane run --domain boat --tier T1 --astro-test speed-surge --card CARD --out runs/sim [--space small]

One run record per variant, test and seed (the vessel is deterministic: seeds repeat a run, and
``--set current=[cx,cy]`` sets a steady world current): ``OUT/<variant>/<test>/s<seed>/``. Its ``metrics.json`` has the
fields of an astro result file (``test, type, layer, space, amp, summary, measured, changes``; same names and
units) and ``result.yaml`` is the whole result as astro writes it, holds included, so ``crane compare --field``
lines the two up key by key. The vessel lives through the tests of one variant in order, and each result's
changes are applied to the session state as ``astro apply`` would (``--astro-test all`` runs the ladder).

The tier picks the vessel (Tools/Characterization/astro_tests.py): T0 ``FirstOrderVessel`` (per-axis lag and
steady speeds from the card, no forces), T1 ``ProfileVessel`` (the MarinePlanarModel twin). T2 runs astro's
own tests on the Unity world (``crane sim up`` then ``astro test``).

``--card`` takes a profile JSON (``Tools/Characterization/profiles/*.json``, ``crane fit``'s profile.json), a
plant card (directory, sha or ``robot@ref``: its ``unity`` view), or a ``crane fit --factorial`` manifest (one
variant per card). Without it, the domain's tier config names the profile (``vehicle.profile``).
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path


from crane import paths, tiers

CHAR = paths.ROOT / 'Tools' / 'Characterization'
DEFAULT_TESTS = paths.ASTRO_DOCK / 'src' / 'mhseals_hardware' / 'config' / 'tests.yaml'
HZ = 50.0


def characterization():
    if str(CHAR) not in sys.path:
        sys.path.insert(0, str(CHAR))
    import astro_tests
    return astro_tests


def load_spec(where):
    """astro's tests file from a YAML path or from a bundle's state/tests.yaml."""
    A = characterization()
    p = Path(where).expanduser()
    if p.suffix in ('.tgz', '.gz'):
        import tarfile
        import yaml
        with tarfile.open(p) as tar:
            member = next((m for m in tar.getmembers() if m.name.endswith('tests.yaml')), None)
            if member is None:
                raise LookupError(f'{p}: no tests.yaml in the bundle')
            return A.Spec(yaml.safe_load(tar.extractfile(member).read()) or {}, f'{p}:{member.name}')
    if not p.is_file():
        raise LookupError(f'no tests file at {p} (give --tests FILE, a bundle, or set ASTRO_DOCK)')
    return A.Spec.load(p)


def cards(refs, default_profile):
    """[(variant, profile dict, card record)] for each --card (a profile JSON, a plant card or a factorial
    manifest); the domain's profile when none is given."""
    out = []
    for ref in refs or [str(default_profile)]:
        p = Path(ref).expanduser()
        if p.is_file() and p.suffix == '.json':
            raw = p.read_bytes()
            doc = json.loads(raw)
            if doc.get('schema') == 'crane.factorial/1':
                for v in doc['variants']:
                    vp = (p.parent / v['profile']).resolve()
                    vraw = vp.read_bytes()
                    out.append((v['name'], json.loads(vraw), {'ref': str(vp), 'sha256': hashlib.sha256(vraw).hexdigest(),
                                                              'factorial': str(p), 'levels': v['levels']}))
                continue
            if 'thrusters' in doc and 'mass' in doc:
                out.append((p.stem, doc, {'ref': str(p), 'sha256': hashlib.sha256(raw).hexdigest(),
                                          'name': doc.get('name')}))
                continue
            raise LookupError(f'{p}: neither a profile JSON, a plant card nor a factorial manifest')
        import plantcard
        card = plantcard.load(ref)
        out.append((card.sha[:12] if p.is_dir() else str(ref).replace('/', '_').replace('@', '-'),
                    card.view('unity'), {'ref': str(ref), 'card_sha': card.sha, 'status': card.status}))
    return out


def _write(out, rows, t0, t1, events, result, meta_kw):
    from crane import record
    from theater import traj as ctraj
    import yaml
    sel = [r for r in rows if t0 - 1e-9 <= r[0] <= t1 + 1e-9]
    step = max(1, int(round(1.0 / (HZ * (sel[1][0] - sel[0][0])))) if len(sel) > 1 else 1)
    sel = sel[::step]
    hz = 1.0 / (sel[1][0] - sel[0][0]) if len(sel) > 1 else HZ
    cols = {'boat.x': [r[5] for r in sel], 'boat.y': [r[6] for r in sel],
            'boat.yaw': [(r[4] + math.pi) % (2 * math.pi) - math.pi for r in sel]}
    header = {'pose_dims': 'planar', 'robot': {'model': 'boat', 'agents': ['boat']},
              'frame': {'type': 'enu', 'origin': 'start', 'body': 'flu'}}
    data = ctraj.encode(header, cols, [], hz=round(hz, 6), t0_s=0.0)
    evs = [{**e, 't': e['t'] - t0} for e in events if t0 - 1e-9 <= e['t'] <= t1 + 1e-9]
    metrics = {k: v for k, v in result.items() if k != 'holds'}
    rec = record.write_run(out, record.meta(**meta_kw, clock={'source': 'sim', 't0': round(t0, 6),
                                                                  'time_multiplier': None},
                                            frame={'world': 'enu', 'origin': 'start', 'body': 'flu'}),
                           traj=data, events=evs, metrics=json.loads(json.dumps(metrics)))
    (Path(out) / 'result.yaml').write_text(yaml.safe_dump(json.loads(json.dumps(result)), sort_keys=False, width=110))
    return rec


def main(args):
    A = characterization()
    try:
        tier = tiers.normalise(args.tier) if args.tier else 'T1'
    except ValueError as error:
        print(f'crane run: {error}', file=sys.stderr)
        return 2
    if tier == 'T2':
        print('crane run: astro tests at T2 run on the Unity world: `crane sim up --headless`, then '
              f'`astro test {args.astro_test}`.', file=sys.stderr)
        return 2
    import yaml
    from crane import run as crane_run
    try:
        if args.config:
            config, merged = Path(args.config).resolve(), {}
        else:
            tier, config, merged = crane_run.tier_config(args.domain, tier)
        cfg = yaml.safe_load(Path(config).read_text()) or {}
        overrides = {**merged, **crane_run.overrides(args.set)}
        profile = ((cfg.get('vehicle') or {}).get('profile') or ((cfg.get('adapter') or {}).get('options') or {})
                   .get('profile') or 'Tools/Characterization/profiles/characterized.json')
        variants = cards(args.card, paths.ROOT / profile)
        spec = load_spec(args.tests or DEFAULT_TESTS)
        names = spec.expand(args.astro_test)
    except (LookupError, KeyError, ValueError) as error:
        print(f'crane run: {error}', file=sys.stderr)
        return 2
    if tier not in A.VESSELS:
        print(f'crane run: no astro-test vessel at {tier}', file=sys.stderr)
        return 2
    seeds = _seeds(args.seeds or '0')
    out = Path(args.out or 'runs/astro')
    current = tuple(overrides.get('current') or (0.0, 0.0))
    space = args.space or 'large'
    done = 0
    for variant, prof, card in variants:
        for seed in seeds:
            vessel = A.VESSELS[tier](prof, current=current)
            session = A.Session(spec, vessel)
            for name in names:
                t0 = vessel.now()
                try:
                    result = session.run(name, space=space, amp=args.amp)
                except A.Abort as error:
                    print(f'crane run: {variant} {name}: aborted ({error})', file=sys.stderr)
                    return 2
                rec = _write(out / variant / name / f's{seed}', vessel.rows, t0, vessel.now(), session.events,
                             result, dict(run_id=f'{variant}-{name}-s{seed}', domain=args.domain or 'boat',
                                          tier=tier, kind=vessel.kind, card=card,
                                          params={'astro_test': name, 'space': space, 'amp': result.get('amp'),
                                                  'current': list(current)},
                                          seed=seed, backend='python', dtype='float64',
                                          provenance={'tests': spec.path, 'config': str(config), 'tier': tier,
                                                      'vessel': type(vessel).__name__}))
                done += 1
                print(f'{rec.path}: {result["summary"]}')
    print(f'{done} astro-test runs at {tier} ({tiers.NAMES[tier]}) in {out}')
    return 0


def _seeds(text):
    from crane import select
    return select.parse_seeds(text)

