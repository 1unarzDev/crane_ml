"""python3 -m judge (from the crane root)

  demo [--out DIR]           the toy quadrotor on two tiers, a decision fault and a clean control
  fastsim ROOT [--out DIR]   judge every fastsim run under ROOT with the generic motion pack
  oneline REPORT.json        print a report's one-line verdict
  markdown REPORT.json       print a report as Markdown
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from judge import render


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


def _fastsim(root, out):
    from judge.packs import generic_motion
    from judge.run import judge_runs
    from judge.sources.fastsim import FastsimSource
    refused = []
    recs = FastsimSource().load_all(root, refused)
    rep = judge_runs(recs, None, [generic_motion.PACK])
    rep['refused'] = refused
    print(render.oneline(rep))
    if out:
        render.write(rep, out, 'fastsim runs')


def main(argv=None):
    ap = argparse.ArgumentParser(prog='judge', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=('demo', 'fastsim', 'oneline', 'markdown'))
    ap.add_argument('path', nargs='?')
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    if a.cmd == 'demo':
        _demo(a.out)
    elif a.cmd == 'fastsim':
        _fastsim(a.path, a.out)
    else:
        rep = json.loads(Path(a.path).read_text())
        print(render.oneline(rep) if a.cmd == 'oneline' else render.markdown(rep))


if __name__ == '__main__':
    main()
