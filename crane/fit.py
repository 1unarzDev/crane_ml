"""crane fit: an astro bundle to a profile (through the gate) and a plant card (through the evidence).

    crane fit BUNDLE.tgz [BUNDLE2.tgz ...] [--bags DIR] [--out DIR]   (OUT: BUNDLE.fit/ beside the first bundle)
    crane fit --factorial SPEC.yaml [--out DIR]

1. **Gate.** ``Tools/Characterization/bundle.py`` turns the bundle's state and speed results into the gate's
   inputs; ``import_yaml`` writes ``OUT/profile.json`` (parameters the gate identifies or marks provisional
   overwrite the base; the rest stay prior) and ``OUT/profile.gate.json``. On abstain the summary names the
   exact missing fields.
2. **Evidence.** Each hold of each result (speed, couple, steps) with a bag covering it becomes a
   ``plantcard`` Segment (``plantcard.marine.fieldtrials.hold_segment``): odometry from the bag, the thruster
   commands from ``/thrusters/pwm`` (physical outputs, mapped to the canonical thrusters by the state's
   ``channel_map`` and ``reversed``). A bundle is one session; a sim-made state gives tier ``unity`` segments.
3. **Card.** ``plantcard.update.propose`` fits a child of the prior card made from ``OUT/profile.json``; both
   go to ``OUT/cards`` (``<robot>`` is the prior, ``<robot>@candidate`` the fit). The card's provenance names
   the bundles (sha256), their results and the software commits.
4. **Validation.** With two or more sessions, ``validate.cross_validate`` (leave one session out) and
   ``validate.promotion`` on ``pos_err_max@H5s``; with one, the card stays ``fitted`` and is not promotable.

``OUT/fit.json`` holds the machine-readable summary. ``--factorial`` writes card variants (``factorial``).
"""

from __future__ import annotations

import hashlib
import json
import sys
import tempfile
from contextlib import ExitStack
from pathlib import Path

import numpy as np

from crane import paths

CHAR = paths.ROOT / 'Tools' / 'Characterization'
CANONICAL = ('front_left', 'front_right', 'rear_right', 'rear_left')   # FL, FR, RR, RL: the mixer rows
PWM_TOPIC, ODOM_TOPIC = '/thrusters/pwm', '/odom/local'
MIN_SESSIONS = 2
HORIZON = 5.0


def characterization():
    if str(CHAR) not in sys.path:
        sys.path.insert(0, str(CHAR))
    import bundle as B
    import gate as G
    return B, G


def open_bundle(path, into):
    B, _ = characterization()
    return B.open_bundle(path, into)


# ---- the gate ---------------------------------------------------------------------------------------------

def abstentions(report):
    """{reason: [(scope, missing or evidence summary)]} of the gate's abstaining checks."""
    out = {}
    for c in report.get('checks') or []:
        if c.get('status') != 'ABSTAIN':
            continue
        ev = c.get('evidence') or {}
        what = ev.get('missing') or ev.get('why') or {k: v for k, v in ev.items() if k in ('max_gap_s', 'coverage',
                                                                                          'path', 'hardware_hash')}
        out.setdefault(c['reason'], []).append((c['scope'], what))
    return out


def gate_lines(prof, report, state_source):
    lines = [f"gate: {report['summary']} ({len([p for p in prof['provenance']['parameters']])} rows)",
             f"  {'parameter':<14} {'status':<11} why"]
    B, G = characterization()
    for p in prof['provenance']['parameters']:
        if p['name'] in G.DIRECTIONS or p['name'].startswith('addedMass.'):
            lines.append(f"  {p['name']:<14} {p['status']:<11} {p['reason']}")
    ab = abstentions(report)
    if ab:
        lines.append('  abstained because:')
    for reason, rows in sorted(ab.items()):
        if reason == 'SIM_DERIVED':
            lines.append(f'    {reason}: characterization.source is {state_source!r}; a sim-made state is never '
                         'imported (its directions stay prior)')
        elif reason == 'PROVENANCE_MISSING':
            missing = sorted({m for _, what in rows for m in (what if isinstance(what, list) else [str(what)])})
            scopes = sorted({s.split(':', 1)[-1] for s, _ in rows})
            lines.append(f'    {reason} ({", ".join(scopes)}): missing {"; ".join(missing)}')
        elif reason.startswith('HOLD_INVALID'):
            lines.append(f'    {reason}: {len(rows)} hold(s), e.g. {rows[0][0]} {rows[0][1]}')
        else:
            scopes = sorted({s for s, _ in rows})
            whys = sorted({json.dumps(w, default=str) for _, w in rows if w})[:2]
            lines.append(f'    {reason}: {", ".join(scopes)}' + (f' ({"; ".join(whys)})' if whys else ''))
    m = report.get('characterization_mapping')
    if m:
        lines.append(f"  characterization version {m['from']} mapped to 2: t95 from the + holds "
                     f"{m.get('t95')} (version 1 had {m.get('t95_v1')}); not in version 1: "
                     f"{', '.join(m.get('not_in_v1') or [])}" if m.get('mapped') else f"  {m.get('missing')}")
    return lines


# ---- evidence ---------------------------------------------------------------------------------------------

def _q_canonical(pwm_us, state):
    """[n, 4] normalised commands in canonical order from physical-output PWM (channel_map, reversed)."""
    cm = list(state.get('channel_map') or [1, 2, 3, 4])
    rev = list(state.get('reversed') or [False] * 4)
    q = np.stack([(pwm_us[:, cm[i] - 1] - 1500.0) / 400.0 for i in range(4)], -1)
    return np.clip(q * np.array([-1.0 if r else 1.0 for r in rev]), -1.0, 1.0)


def bag_streams(bag):
    """Odometry and PWM of one bag on its header clock, and the receive-time map (wall -> header clock s)."""
    from crane import field
    streams, files = field.read_bag(bag, [ODOM_TOPIC, PWM_TOPIC])
    o = field.odometry(streams[ODOM_TOPIC])
    clock, to_record, _ = field.clock_of(o['t_log'], o['t_hdr'])
    t = o['t_hdr'] - clock['t0']
    pt, pus = field.pwm(streams[PWM_TOPIC]) if streams[PWM_TOPIC] else (np.zeros(0), np.zeros((0, 4)))
    sha = hashlib.sha256(b''.join(hashlib.sha256(f.read_bytes()).digest() for f in files)).hexdigest()
    return {'odom': {'t': t, **{k: o[k] for k in ('x', 'y', 'yaw', 'vx', 'vy', 'wz')}},
            'pwm_t': to_record(pt), 'pwm_us': pus, 'to_record': to_record, 'clock': clock,
            'wall_span': (float(o['t_log'][0]), float(o['t_log'][-1])), 'sha': sha, 'bag': str(bag)}


def evidence(bundles, extra_bags, names, log=print):
    """plantcard Evidence from every hold covered by a bag, and the skipped holds with reasons."""
    from plantcard.family import Evidence
    from plantcard.marine import fieldtrials
    B, G = characterization()
    segs, skipped, used_bags, prov = [], [], set(), []
    order = [CANONICAL.index(n) for n in names]
    for b in bundles:
        streams = []
        for bag in [*b.bags, *[Path(x).expanduser() for x in extra_bags or []]]:
            try:
                streams.append(bag_streams(bag))
            except Exception as error:      # noqa: BLE001 - one unreadable bag must not stop the rest
                skipped.append((str(bag), f'unreadable: {error}'))
        tier = 'unity' if b.sim_derived else 'real'
        results = []
        for path, doc in b.results:
            results.append(path.name)
            test = b.test(doc['test'])
            for i, hold in enumerate(doc.get('holds') or []):
                name = f'{path.stem}#{i}'
                start, end = float(hold.get('start', 0)), float(hold.get('end', 0))
                base_s = float(test.get('baseline_s', 2.0))
                s = next((s for s in streams if (start < 1e9 or s['wall_span'][0] <= start - base_s)
                          and (start < 1e9 or s['wall_span'][1] >= end)), None)
                if s is None:
                    skipped.append((name, 'no bag covers it'))
                    continue
                if not len(s['pwm_t']):
                    skipped.append((name, f'no {PWM_TOPIC} in {Path(s["bag"]).name}'))
                    continue
                conv = (lambda x: x - s['clock']['t0']) if start < 1e9 else (lambda x: float(s['to_record'](x)))
                q = _q_canonical(s['pwm_us'], b.state)[:, order]
                seg, why = fieldtrials.hold_segment(
                    session=b.name, tier=tier, name=name, odom=s['odom'], pwm_t=s['pwm_t'], q=q,
                    t_on=conv(start), t_cut=conv(end), baseline_s=base_s,
                    coast_s=float(test.get('coast_s', 2.0)), source_sha=s['sha'])
                if seg is None:
                    skipped.append((name, why))
                else:
                    segs.append(seg)
                    used_bags.add(s['bag'])
        sw = {}
        for _, doc in b.results:
            sw.update(B.software(doc))
        prov.append({'bundle': b.name, 'sha256': b.sha256, 'state': b.state_path.name if b.state_path else None,
                     'results': results, 'software': sw, 'tier': tier})
    ev = Evidence(segs, {'kind': 'astro_bundles', 'bundles': prov, 'bags': sorted(used_bags),
                         'n_segments': len(segs), 'sessions': sorted({s.session for s in segs}),
                         'truth': 'odometry pose and twist (translation_from_odometry)',
                         'skipped': [f'{a}: {w}' for a, w in skipped]})
    return ev, skipped


# ---- the card ---------------------------------------------------------------------------------------------

def card_lines(doc):
    lines = [f"  {'block':<10} {'status':<8} parameters (mean +- sd)"]
    for b, blk in sorted(doc['blocks'].items()):
        if blk.get('kind') == 'nig':
            ps = ', '.join(f'{n} {m:.4g}+-{s:.2g}' for n, m, s in zip(blk['params'], blk['mean'], blk['sd']))
            lines.append(f"  {b:<10} {blk.get('status', ''):<8} {ps}"
                         + (f"  pinned {blk['pinned']}" if blk.get('pinned') else ''))
        elif blk.get('kind') == 'grid':
            post = blk.get('posterior') or {}
            ps = ', '.join(f'{k} {v["mean"]:.3g}+-{v["sd"]:.2g}' for k, v in post.items())
            lines.append(f"  {b:<10} {blk.get('status', ''):<8} {ps}")
    return lines


def fit_card(profile_path, ev, robot, store, n_rollouts, log=print):
    from plantcard import card as C, validate as V
    from plantcard.marine import seed
    from plantcard.update import propose
    doc0, parts0 = seed.prior_doc(profile_path, robot=robot)
    c0 = C.Card(doc0, parts0)
    note = f"astro bundles {', '.join(b['bundle'] for b in ev.provenance['bundles'])}"
    doc1, parts1, support1, report = propose(c0, ev, procedure='integral-window NIG (1 s), odometry, astro holds',
                                             note=note)
    sessions = ev.sessions()
    validation = {'sessions': len(sessions), 'needed': MIN_SESSIONS}
    sup = doc1['envelope']['support']
    if not np.isfinite(sup['threshold']):      # one session: no other session's support to calibrate on
        sup.update(threshold=None, calibration=f'none: {len(sessions)} session (leave-one-session-out needs two)')
    if len(sessions) >= MIN_SESSIONS:
        child, parent = V.cross_validate(c0, ev, horizons=(HORIZON,), n=n_rollouts, log=log)
        cs, ps = V.summarize(child, 0.1, HORIZON), V.summarize(parent, 0.1, HORIZON)
        if cs and ps:
            promo = V.promotion(cs, ps)
            doc1['validation'] = [_clean({**cs, 'H': HORIZON, 'tier': ev.segments[0].tier,
                                          'heldout': 'leave-one-session-out', 'promotion': promo})]
            validation.update(child=_clean(cs), parent=_clean(ps), promotion=promo)
    saved0 = C.save(doc0, parts0, None, store)
    doc1['parent'] = saved0.sha
    saved1 = C.save(doc1, parts1, support1, store)
    C.set_ref(robot, saved0.sha, store)
    C.set_ref(robot, saved1.sha, store, name='candidate')
    return saved0, saved1, validation


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


def run_fit(bundle_paths, out, extra_bags=(), base=None, robot=None, n_rollouts=64, card=True, log=print):
    """The whole fit; returns the summary dict (also OUT/fit.json)."""
    B, G = characterization()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        bundles = [B.open_bundle(p, stack.enter_context(tempfile.TemporaryDirectory(prefix='crane-fit-')))
                   for p in bundle_paths]
        last = bundles[-1]
        sw = {}
        for _, doc in last.results:
            sw.update(B.software(doc))
        log(f"crane fit: {', '.join(b.name for b in bundles)}; state "
            f"{last.state_path.name if last.state_path else 'none'}, {len(last.results)} results, software "
            + (', '.join(f'{k}@{v[:8]}' for k, v in sw.items()) or 'none recorded'))
        if not (G.characterization_block(last.state) or {}).get('speed_curves'):
            raise SystemExit(f'crane fit: {last.name} has no characterization.speed_curves in its state; '
                             'run and apply astro test speed-* first')
        prof, report, notes = B.import_profile(last, out / 'profile.json', base=base, log=lambda *_: None)
        source = (G.characterization_block(last.state) or {}).get('source', '')
        for line in gate_lines(prof, report, source):
            log(line)
        for n in notes:
            log(f'  note: {n}')
        summary = {'schema': 'crane.fit/1', 'bundles': [{'name': b.name, 'sha256': b.sha256} for b in bundles],
                   'profile': str(out / 'profile.json'), 'gate': {'summary': report['summary'],
                   'parameters': prof['provenance']['parameters'], 'abstained': {
                       k: [list(r) for r in v] for k, v in abstentions(report).items()},
                   'mapping': report.get('characterization_mapping')}, 'software': sw}
        if card:
            names = [t['name'] for t in json.loads((out / 'profile.json').read_text())['thrusters']]
            ev, skipped = evidence(bundles, extra_bags, names, log)
            reasons = {}
            for _, why in skipped:
                reasons[why] = reasons.get(why, 0) + 1
            log(f"evidence: {len(ev.segments)} segments from {len(ev.provenance['bags'])} bag(s) in "
                f"{len(ev.sessions())} session(s)" + (f"; skipped {', '.join(f'{v} ({k})' for k, v in reasons.items())}"
                                                       if reasons else ''))
            summary['evidence'] = _clean({k: v for k, v in ev.provenance.items() if k != 'skipped'})
            if ev.segments:
                robot = robot or f"astro/{last.state_path.stem if last.state_path else 'vessel'}"
                c0, c1, validation = fit_card(out / 'profile.json', ev, robot, out / 'cards', n_rollouts, log)
                log(f'card: {c1.sha[:12]} {c1.status} ({robot}@candidate), parent {c0.sha[:12]} prior from '
                    f'profile.json; store {out / "cards"}')
                for line in card_lines(c1.doc):
                    log(line)
                if 'promotion' in validation:
                    p = validation['promotion']
                    log(f"validation (leave one session out, pos_err_max@H{HORIZON:g}s): "
                        f"{'passes' if p['passed'] else 'fails'} {p['checks']}; a person still accepts it")
                else:
                    log(f"validation: needs {MIN_SESSIONS} or more sessions (bundles), have {validation['sessions']}; "
                        'the card stays fitted and is not promotable')
                summary['card'] = {'candidate': c1.sha, 'prior': c0.sha, 'robot': robot, 'store': str(out / 'cards'),
                                   'validation': validation}
            else:
                log('card: no segments, so no card (bags: --bags DIR, or astro bundle --bags)')
    (out / 'fit.json').write_text(json.dumps(_clean(summary), indent=1, default=str) + '\n')
    log(f'wrote {out / "fit.json"}')
    return summary


# ---- factorials -------------------------------------------------------------------------------------------

def _source_params(ref, base_dir):
    """(params, card or None, record) of a level source: a plant card (its nominal) or a profile JSON."""
    from plantcard.marine.fossen3 import profile_params
    p = (base_dir / Path(ref).expanduser()) if not Path(ref).expanduser().is_absolute() else Path(ref)
    if p.is_file() and p.suffix == '.json':
        raw = p.read_bytes()
        return profile_params(json.loads(raw)), None, {'profile': str(p), 'sha256': hashlib.sha256(raw).hexdigest()}
    import plantcard
    card = plantcard.load(str(p) if p.is_dir() else ref)
    return {k: float(v) for k, v in card.doc['nominal'].items()}, card, {'card': str(ref), 'sha': card.sha}


def factorial(spec_path, out, log=print):
    """Card variants (each a ``unity`` view, a profile JSON with provenance) over every combination of the
    properties' levels, and ``OUT/manifest.json`` (schema crane.factorial/1) with the one command that runs
    and compares them.

    SPEC.yaml::

        measured: CARD_OR_PROFILE        # the measured level (a plant card ref/dir or a profile JSON)
        prior: CARD_OR_PROFILE           # the prior level
        properties:                      # property -> the card parameters it sets
          surge: [m11, d1_u, d2_u]
          actuation: [delay_s, rev_ratio, deadband_q]
        hypotheses:                      # optional extra levels: property -> {level: {param: value}}
          actuation: {slow_esc: {delay_s: 0.45}}
        run: {domain: boat, tier: T1, astro_test: speed-surge, space: small, bundle: B.tgz}
    """
    import itertools
    import yaml
    from plantcard import card as C
    from plantcard.marine import seed, views
    spec_path = Path(spec_path)
    raw = spec_path.read_bytes()
    spec = yaml.safe_load(raw) or {}
    here = spec_path.parent
    measured, mcard, mrec = _source_params(spec['measured'], here)
    prior, _, prec = _source_params(spec['prior'], here)
    if mcard is None:
        pref = Path(mrec['profile'])
        doc, parts = seed.prior_doc(pref)
        mcard = C.Card(doc, parts)
    props = spec.get('properties') or {}
    hyps = spec.get('hypotheses') or {}
    unknown = sorted({p for ps in props.values() for p in ps} - set(measured))
    if unknown:
        raise SystemExit(f'crane fit --factorial: {spec_path}: parameters {unknown} are not card parameters '
                         f'({", ".join(sorted(measured))})')
    levels = {prop: ['measured', 'prior', *sorted((hyps.get(prop) or {}))] for prop in props}
    out = Path(out)
    (out / 'variants').mkdir(parents=True, exist_ok=True)
    variants = []
    for combo in itertools.product(*[[(prop, lv) for lv in lvs] for prop, lvs in levels.items()]):
        params = dict(measured)
        for prop, lv in combo:
            for name in props[prop]:
                params[name] = prior[name] if lv == 'prior' else measured[name]
            if lv not in ('measured', 'prior'):
                params.update({k: float(v) for k, v in hyps[prop][lv].items()})
        name = '_'.join(f'{prop}-{lv}' for prop, lv in combo) or 'measured'
        doc = json.loads(json.dumps(mcard.doc))
        doc['nominal'] = params
        doc['provenance'] = {**doc.get('provenance', {}), 'factorial': {'spec': str(spec_path), 'levels': dict(combo)}}
        doc.pop('sha', None)
        doc['sha'] = C.card_sha(doc)
        view = views.unity_profile(C.Card(doc, mcard.particles))
        view['name'] = f'Factorial-{name}'
        view['provenance']['factorial'] = {'levels': dict(combo), 'measured': mrec, 'prior': prec,
                                           'spec_sha256': hashlib.sha256(raw).hexdigest(), 'view_of': doc['sha']}
        path = out / 'variants' / f'{name}.json'
        text = json.dumps(view, indent=2) + '\n'
        path.write_text(text)
        variants.append({'name': name, 'levels': dict(combo), 'profile': f'variants/{name}.json',
                         'sha256': hashlib.sha256(text.encode()).hexdigest()})
    run = spec.get('run') or {}
    command = (f"crane run --domain {run.get('domain', 'boat')} --tier {run.get('tier', 'T1')} --astro-test "
               f"{run.get('astro_test', 'all')} --space {run.get('space', 'large')} --card {out / 'manifest.json'} "
               f"--out {out / 'runs'}")
    if run.get('tests'):
        command += f" --tests {run['tests']}"
    if run.get('bundle'):
        command += f" && crane compare --field {run['bundle']} {out / 'runs'} --out {out / 'compare'}"
    manifest = {'schema': 'crane.factorial/1', 'spec': str(spec_path), 'spec_sha256': hashlib.sha256(raw).hexdigest(),
                'measured': mrec, 'prior': prec, 'levels': levels, 'variants': variants, 'command': command}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')
    log(f"{len(variants)} variants ({' x '.join(f'{p} {len(v)}' for p, v in levels.items())}) in {out / 'variants'}")
    log(f"manifest {out / 'manifest.json'}; run and compare them with:\n  {command}")
    return manifest


def main(args):
    if args.factorial:
        factorial(args.factorial, args.out or f'factorial-{Path(args.factorial).stem}')
        return 0
    if not args.bundles:
        print('crane fit: give an astro bundle (BUNDLE.tgz), or --factorial SPEC.yaml', file=sys.stderr)
        return 2
    out = args.out or paths.fit_out(args.bundles[0])
    run_fit(args.bundles, out, args.bags, args.base, args.robot, args.n, card=not args.no_card)
    print(f'next: crane sim up --card {Path(out) / "profile.json"}, then crane compare --field {args.bundles[0]} RUNS '
          f'--bounds file --fit {out}')
    return 0
