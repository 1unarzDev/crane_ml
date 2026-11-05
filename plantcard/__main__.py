"""plantcard CLI.

    python3 -m plantcard show   REF                   summary of a card (REF: robot[@name], sha or dir)
    python3 -m plantcard export VIEW REF [--particle K] [--n N] [--out FILE]
                                                      VIEW: unity | nlc | control | fastsim
    python3 -m plantcard.marine.build --robot R [--repo]   refit a marine card from field trials
"""

import argparse
import json
import sys

import numpy as np

from plantcard.card import load


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(prog='plantcard', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('show')
    s.add_argument('ref')
    e = sub.add_parser('export')
    e.add_argument('view')
    e.add_argument('ref')
    e.add_argument('--particle', type=int, default=None)
    e.add_argument('--n', type=int, default=None)
    e.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    card = load(a.ref)
    if a.cmd == 'show':
        d = card.doc
        print(f'{card.robot} {card.sha[:12]} status={card.status} gauge={d["gauge"]} parent={str(d.get("parent"))[:12]}')
        for b, blk in d['blocks'].items():
            if blk.get('kind') == 'nig':
                print(f'  {b:10s} ' + '  '.join(f'{k}={m:.3g}+-{s:.2g}' for k, m, s in zip(blk['params'], blk['mean'], blk['sd']))
                      + f'  [{blk["status"]}; {", ".join(blk.get("flags", []))}]')
            elif blk.get('kind') == 'grid':
                post = blk.get('posterior') or {}
                print(f'  {b:10s} map={blk.get("map", "-")} ' + ' '.join(f'{k}={v["mean"]:.3g}+-{v["sd"]:.2g}'
                                                                     for k, v in post.items()))
            else:
                print(f'  {b:10s} {json.dumps({k: v for k, v in blk.items() if k not in ("kind",)})[:160]}')
        for v in d.get('validation', []):
            print(f'  validation {v["tier"]} {v["event"]} alpha={v["alpha"]}: covered {v["covered_frac"]:.2f} '
                  f'(CP LCB95 {v["lcb95_cp"]:.2f}), width {v["width_median_m"]:.2f} m, CRPS {v["crps_final_m"]:.2f} m; '
                  f'parent covered {v["parent"]["covered_frac"]:.2f}, width {v["parent"]["width_median_m"]:.2f} m')
        return
    kw = {}
    if a.view == 'unity' or a.view == 'fastsim':
        kw['particle'] = a.particle
    if a.view == 'nlc':
        kw['n'] = a.n
    out = json.dumps(_clean(card.view(a.view, **kw)), indent=1)
    if a.out:
        open(a.out, 'w').write(out + '\n')
    else:
        sys.stdout.write(out + '\n')


if __name__ == '__main__':
    main()
