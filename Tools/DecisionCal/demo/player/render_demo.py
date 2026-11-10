#!/usr/bin/env python3
"""Render a demo shot script (make_demo_scripts.py) with the demo player through Theater, under ``heavy unity``.

    python3 render_demo.py SCRIPT [--pack PACK] [--out OUT] [--player DIR] [--name NAME] [-- EXTRA PLAYER ARGS]

Runs (also saved to OUT/command.txt)::

    HEAVY_WHO=decisioncal-player heavy unity python3 -m theater.render local PACK --player DIR/CRANE.x86_64 --out OUT \
        --size 1920x1080 --fps 30 --frames 15:N --stills 30 -- --crane-theater-script SCRIPT [EXTRA]

Every act of a demo script is frozen (speed 0.01) and 1 s long: frame 15 of each act is kept as a PNG (Theater
pre-rolls 2 s before frame 15), copied to OUT/<NAME>_<act>.png. Nothing is drawn on the frames afterwards.
"""
import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]          # the decisioncal worktree (theater/ package)
PACK = Path.home() / '.cache' / 'decisioncal' / 'demo' / 'packs' / 'unity-ls-20261109-steady4242b0-c08-s1'
PLAYER = Path.home() / '.cache' / 'decisioncal' / 'demo-player' / 'player-linux'
RENDERS = Path.home() / '.cache' / 'decisioncal' / 'demo-player' / 'renders'
HEAVY = Path.home() / 'astro_dock' / 'src' / 'mhseals_agents' / 'tools' / 'heavy'


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    extra = argv[argv.index('--') + 1:] if '--' in argv else []
    argv = argv[:argv.index('--')] if '--' in argv else argv
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('script')
    ap.add_argument('--pack', default=str(PACK))
    ap.add_argument('--player', default=str(PLAYER))
    ap.add_argument('--out', default=None)
    ap.add_argument('--name', default=None)
    ap.add_argument('--size', default='1920x1080')
    ap.add_argument('--timeout', type=float, default=3600)
    a = ap.parse_args(argv)
    script = Path(a.script).resolve()
    name = a.name or script.stem.replace('script_', '')
    out = Path(a.out).resolve() if a.out else RENDERS / f"{name}-{time.strftime('%Y%m%d-%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    doc = json.loads(script.read_text())
    fps = int(doc['fps'])
    total = sum(round(x['duration_s'] * fps) for x in doc['acts'])
    player = Path(a.player)
    cmd = [str(HEAVY), 'unity', sys.executable, '-m', 'theater.render', 'local', str(Path(a.pack).resolve()),
           '--player', str(player / 'CRANE.x86_64'), '--out', str(out), '--size', a.size, '--fps', str(fps),
           '--frames', f'{fps // 2}:{total}', '--stills', str(fps), '--timeout', str(int(a.timeout)),
           '--', '--crane-theater-script', str(script), *extra]
    env = dict(os.environ, HEAVY_WHO=os.environ.get('HEAVY_WHO', 'decisioncal-player'), HEAVY_MAX_S=str(int(a.timeout)))
    for k in ('DISPLAY', 'WAYLAND_DISPLAY'):
        env.pop(k, None)
    env['SDL_VIDEODRIVER'] = 'dummy'
    (out / 'command.txt').write_text(f'cd {ROOT} && HEAVY_WHO={env["HEAVY_WHO"]} ' + ' '.join(shlex.quote(c) for c in cmd) + '\n')
    shutil.copy(script, out / script.name)
    t0 = time.time()
    rc = subprocess.call(cmd, cwd=ROOT, env=env)
    wall = time.time() - t0
    stills = sorted((out / 'stills').glob('frame_*.png')) if (out / 'stills').is_dir() else []
    kept = []
    for i, p in enumerate(stills):
        dst = out / f'{name}_{i}.png'
        shutil.copy(p, dst)
        kept.append(str(dst))
    rep = json.loads((out / 'render_report.json').read_text()) if (out / 'render_report.json').is_file() else {}
    meta = {'rc': rc, 'wall_s_incl_admission': wall, 'player_wall_s': rep.get('wall_s'), 'errors': rep.get('errors'),
            'error_lines': rep.get('error_lines'), 'player': str(player), 'player_commit':
            (player / 'commit').read_text().strip() if (player / 'commit').is_file() else None,
            'script': str(script), 'stills': kept, 'notes': [x.get('note') for x in doc['acts']]}
    (out / 'render_meta.json').write_text(json.dumps(meta, indent=1) + '\n')
    print(json.dumps(meta, indent=1))
    return rc


if __name__ == '__main__':
    sys.exit(main())
