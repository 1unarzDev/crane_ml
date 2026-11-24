"""python3 -m Tools.SimCheck CHECK INPUTS... (or python3 Tools/SimCheck CHECK ...); see __init__.py."""

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ''):          # run as a script path: python3 Tools/SimCheck ...
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = 'Tools.SimCheck'

from . import axis, docking, sitl, smoke, thresholds  # noqa: E402


def _json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _trajectory(path):
    text = Path(path).read_text(encoding='utf-8')
    if str(path).endswith('.jsonl'):
        return [json.loads(line) for line in text.splitlines() if line.strip()], None
    doc = json.loads(text)
    return (doc, None) if isinstance(doc, list) else (doc.get('trajectory', []), doc.get('goal'))


def parser():
    p = argparse.ArgumentParser(prog='python3 -m Tools.SimCheck', description=__doc__)
    p.add_argument('--thresholds', help='a thresholds.yaml in place of the one beside this package')
    sub = p.add_subparsers(dest='check', required=True)

    a = sub.add_parser('axis', help='FLU command axes and yaw feedback from a command-response result')
    a.add_argument('result')
    a.add_argument('--dominance-ratio', type=float)

    peer = sub.add_parser('sitl-peer', help="drive the player's SITL UDP port as a flight controller")
    peer.add_argument('--output', type=Path, required=True)
    peer.add_argument('--host', default='127.0.0.1')
    peer.add_argument('--port', type=int, default=9002)
    peer.add_argument('--duration', type=float, default=4.0)
    peer.add_argument('--frame-rate', type=int, default=50)
    peer.add_argument('--pwm', type=int, default=1600)
    peer.add_argument('--mode', choices=sorted(sitl.LIMITATIONS), default='aquatic')

    s = sub.add_parser('sitl', help='SITL UDP acceptance over RESULT_ROOT (fixture-summary.json, worker-N/)')
    s.add_argument('result_root', type=Path)
    s.add_argument('--worker-id', type=int, default=0)
    s.add_argument('--maximum-lag-ticks', type=int)
    s.add_argument('--scope', choices=sorted(sitl.LIMITATIONS), default='aquatic')

    d = sub.add_parser('docking', help='rotate-to-path and settling over a run trajectory (.json or .jsonl)')
    d.add_argument('result')
    d.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                   help='settling goal; else the result\'s "goal", else rotate-to-path only')

    m = sub.add_parser('smoke', help='a recorded session flowed: SESSION_DIR (bag/, course_truth.json) or a bag')
    m.add_argument('session')
    m.add_argument('--duration', type=float, help='recorded seconds the span topics must cover')
    m.add_argument('--truth', help='course truth JSON (default SESSION_DIR/course_truth.json)')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    limits = thresholds(args.thresholds)
    if args.check == 'axis':
        ratio = args.dominance_ratio if args.dominance_ratio is not None else limits['axis']['dominance_ratio']
        result = axis.check(_json(args.result), ratio)
    elif args.check == 'sitl-peer':
        result = sitl.peer(args.host, args.port, args.duration, args.frame_rate, args.pwm, args.mode)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        result = dict(result, check='sitl-peer', passed=result['valid'])
    elif args.check == 'sitl':
        lag = args.maximum_lag_ticks if args.maximum_lag_ticks is not None else limits['sitl']['maximum_lag_ticks']
        result = sitl.acceptance_dir(args.result_root, lag, args.scope, args.worker_id,
                                     limits['sitl']['aerial_min_climb_m'])
        (args.result_root / 'sitl-summary.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    elif args.check == 'docking':
        trajectory, goal = _trajectory(args.result)
        result = docking.check(trajectory, limits['docking'], args.goal or goal)
    else:
        result = smoke.check(args.session, limits['smoke'], args.duration, args.truth)
    print(json.dumps(result, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
