#!/usr/bin/env python3
"""Lockstep check of a rebuilt player: the demo capture (../capture.py, unedited) of the reference case with the
player path swapped, then a comparison with the reference capture.

    HEAVY_WHO=decisioncal-player nice -n 10 python3 lockstep_check.py [--player DIR] [--reference RUN_DIR] [--run-id ID]

capture.py reads ``unity_check.PLAYER`` when it builds the argv and stamps ``capture.PLAYER_COMMIT``; both are set
here from DIR/CRANE.x86_64 and DIR/commit before ``capture.main`` runs (module attributes only; no file is edited).
The capture goes through ``heavy unity`` as capture.py always does. Outputs: OUT_ROOT/<run id>/ (capture.py's run dir)
and OUT_ROOT/<run id>/lockstep_check.json (outcome, metric deltas, per-tick state deltas against the reference).
Default case: steady boat, hidden seed 4242 index 0, controller 8, noise seed 1 (reference docked at 43.08 s).
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import capture  # noqa: E402  (imports unity_check as capture.U)

PLAYER_DIR = Path.home() / '.cache' / 'decisioncal' / 'demo-player' / 'player-linux'
REFERENCE = Path.home() / '.cache' / 'decisioncal' / 'demo' / 'runs' / 'unity-ls-20261109-steady4242b0-c08-s1'
OUT_ROOT = Path.home() / '.cache' / 'decisioncal' / 'demo-player' / 'runs'


def rows(path):
    with open(path) as f:
        r = list(csv.reader(f))
    return r[0], [[float(x) for x in row] for row in r[1:]]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--player', default=str(PLAYER_DIR))
    ap.add_argument('--reference', default=str(REFERENCE))
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--out-root', default=str(OUT_ROOT))
    ap.add_argument('--compare-only', metavar='RUN_DIR', help='skip the capture; compare an existing run dir')
    a, rest = ap.parse_known_args(argv)
    player = Path(a.player)
    commit = (player / 'commit').read_text().strip()
    if a.compare_only:
        out = Path(a.compare_only)
    else:
        capture.U.PLAYER = player / 'CRANE.x86_64'
        capture.PLAYER_COMMIT = commit
        run_id = a.run_id or f"unity-ls-{time.strftime('%Y%m%d')}-steady4242b0-c08-s1-p{commit[:7]}"
        capture.main(['--run-id', run_id, '--out-root', a.out_root] + rest)
        out = Path(a.out_root) / run_id
        man = json.loads((out / 'run_manifest.json').read_text())
        man['backend']['player_branch'] = 'research/demo-player'      # capture.py writes nlc-lockstep literally
        (out / 'run_manifest.json').write_text(json.dumps(man, indent=1))
    ref = Path(a.reference)
    s_new, s_ref = json.loads((out / 'summary.json').read_text()), json.loads((ref / 'summary.json').read_text())
    h_new, r_new = rows(out / 'steps.csv')
    h_ref, r_ref = rows(ref / 'steps.csv')
    assert h_new == h_ref, 'steps.csv columns differ'
    n = min(len(r_new), len(r_ref))
    cols = ['world_x', 'world_y', 'world_yaw', 'x', 'y', 'psi', 'u', 'v', 'r', 'pulse_fl', 'pulse_fr', 'pulse_rr', 'pulse_rl']
    deltas = {c: max(abs(r_new[i][h_new.index(c)] - r_ref[i][h_ref.index(c)]) for i in range(n)) for c in cols}
    m_new, m_ref = s_new.get('metrics') or {}, s_ref['metrics']
    rep = {'run': str(out), 'player': str(player / 'CRANE.x86_64'), 'player_commit': commit, 'reference': str(ref),
           'metrics': m_new, 'reference_metrics': m_ref,
           'metric_deltas': {k: (m_new.get(k, float('nan')) - v) for k, v in m_ref.items()},
           'ticks': [len(r_new), len(r_ref)], 'max_abs_step_delta_first_n': deltas, 'n_compared': n,
           'identical_steps_except_wall': len(r_new) == len(r_ref) and all(
               x[:-1] == y[:-1] for x, y in zip(r_new, r_ref)),   # last column is wall_s
           'same_outcome': bool(m_new.get('docked') == m_ref['docked'] and abs(m_new.get('t_dock', 1e9) - m_ref['t_dock']) < 0.5
                                and m_new.get('contacts') == m_ref['contacts'])}
    (out / 'lockstep_check.json').write_text(json.dumps(rep, indent=1) + '\n')
    print(json.dumps(rep, indent=1))
    return 0 if rep['same_outcome'] else 1


if __name__ == '__main__':
    sys.exit(main())
