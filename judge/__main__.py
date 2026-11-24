"""python3 -m judge (from the crane root)

  demo [--out DIR]           the toy quadrotor on two tiers, a decision fault and a clean control
  fastsim ROOT [--out DIR]   judge every fastsim run under ROOT with the generic motion pack
  oneline REPORT.json        print a report's one-line verdict
  markdown REPORT.json       print a report as Markdown

--json prints the report as one JSON document instead. Exit 1 when the report FAILs (a fail-level insight, a
tier gap verdict FAIL, or a variant comparison that is inferior or worse); demo always exits 0.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from judge import render


def failures(report):
    """Why a report FAILs ([] when it passes): fail-level insights, tier-gap FAIL rows, inferior/worse variants."""
    out = [f"insight {i['kind']}" for i in report.get('insights', []) if i.get('severity') == 'fail']
    g4 = report.get('g4') or {}
    out += [f'tier gap {m} FAIL' for m, row in (g4.get('metrics') or {}).items() if row.get('verdict') == 'FAIL']
    out += [f"variant {c.get('metric')} {c.get('verdict')}" for c in report.get('comparisons', [])
            if c.get('verdict') in ('inferior', 'worse')]
    return out


def _demo(out):
    from judge.packs import generic_motion, toy_quad as tq
    from judge.packs.toy_quad.sim import variant
    from judge.run import judge_tiers
    packs = [tq.PACK, generic_motion.make('pose.xz')]
    for name, hp in (('plant', tq.PARAMS_HIGH), ('decision', variant(tq.PARAMS_LOW, skip_gate=2)),
                     ('clean', tq.PARAMS_LOW)):
        pairs = [(s, tq.simulate(tq.PARAMS_LOW, s, 'low'), tq.simulate(hp, s, 'high')) for s in range(8)]
        rep = judge_tiers(pairs, tq.TASK, packs, replayer=tq.QuadReplayer(), decision_kinds=('gate.target',),
                          segmenter=tq.segments, pose='pose.xz')
        print(render.oneline(rep, f'toy-{name}'))
        if out:
            render.write(rep, Path(out) / name, f'Toy quadrotor: {name} fault')


def _fastsim(root, out, as_json=False):
    from judge.packs import generic_motion
    from judge.run import judge_runs
    from judge.sources.fastsim import FastsimSource
    refused = []
    recs = FastsimSource().load_all(root, refused)
    rep = judge_runs(recs, None, [generic_motion.PACK])
    rep['refused'] = refused
    print(json.dumps(rep, indent=1, default=str) if as_json else render.oneline(rep))
    if out:
        render.write(rep, out, 'fastsim runs')
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(prog='judge', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=('demo', 'fastsim', 'oneline', 'markdown'))
    ap.add_argument('path', nargs='?')
    ap.add_argument('--out')
    ap.add_argument('--json', action='store_true', help='the report as one JSON document')
    a = ap.parse_args(argv)
    if a.cmd == 'demo':
        _demo(a.out)
        return 0
    if not a.path:
        ap.error(f'{a.cmd} needs a path')
    if a.cmd == 'fastsim':
        rep = _fastsim(a.path, a.out, a.json)
    else:
        rep = json.loads(Path(a.path).read_text())
        print(json.dumps(rep, indent=1) if a.json else render.oneline(rep) if a.cmd == 'oneline'
              else render.markdown(rep))
    why = failures(rep)
    if why and not a.json:
        print(f'judge: FAIL ({", ".join(why[:3])}{", ..." if len(why) > 3 else ""})', file=sys.stderr)
    return 1 if why else 0


if __name__ == '__main__':
    sys.exit(main())
