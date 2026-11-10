#!/usr/bin/env python3
"""Theater shot scripts for a single-episode demo using the demo player's options (research/demo-player):
``hud: minimal`` (title + backend/run/time caption, no fleet panels), hero_tint 0 (the boat's own materials),
no ring, a thin trail, and fixed cameras (``wide``: explicit position/look-at; ``dock``: a fixed target on the slip).

    python3 make_demo_scripts.py PACK [--out DIR] [--player-commit SHA] [--look plain|theater]

Writes DIR/script_demo_wide.json, DIR/script_demo_dock.json and DIR/script_demo_candidates.json (several dock times
and angles in one render, for picking). Reads only PACK (manifest.json, course_truth.json); writes nothing there.
Coordinates are the trajectory's world frame (ENU, metres; z above the waterline).

Defaults are tuned for the decisioncal demo pack (unity-ls-20261109-steady4242b0-c08-s1):
* wide: the viewpoint of the earlier wide still (``top`` shot, pitch 58), read from that render's frames.jsonl
  (Unity camera (14.4637, 12.0196, 6.4745) = world (6.4745, -14.4637, 12.0196)) looking at the course bounds centre;
* dock: target = the slip's centre from the course truth (slip_wall elements) shifted 0.175 m towards the mouth, a
  steep three-quarter view from the south-east at t = 38.5 s (boat half-way in, 0.45 m/s). Steep (pitch 43 deg) so the
  HUD panel sits on dark water, not on sky glare; south-east so the trail leaves to the right and neither leg of the
  route passes under the panel (checked by projection and by eye; bearings 120-235 put a leg under the caption).
"""
import argparse
import json
from pathlib import Path

DEFAULT_PACK = Path.home() / '.cache' / 'decisioncal' / 'demo' / 'packs' / 'unity-ls-20261109-steady4242b0-c08-s1'
OUT = Path.home() / '.cache' / 'decisioncal' / 'demo-player' / 'scripts'


def slip_centre(truth):
    walls = [e for e in truth['elements'] if e.get('role') == 'slip_wall']
    if not walls:
        return None
    sides = [e for e in walls if e['id'].endswith(('north', 'south'))] or walls
    return (sum(e['x'] for e in sides) / len(sides), sum(e['y'] for e in sides) / len(sides))


def bounds_centre(man):
    b = man['world']['bounds']
    return ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)


def act(t0, caption, camera, hero, note, **kw):
    a = {'type': 'spotlight', 'duration_s': 1.0, 'speed': 0.01, 't0': t0, 'caption': caption, 'note': note,
         'colour': 'progress', 'trails': 'none', 'markers': False, 'hero': hero, 'previous': 'none',
         'iterations': [0, 0], 'hero_tint': 0.0, 'hero_ring': False, 'hero_trail': True,
         'trail_width': 0.07, 'trail_colour': [0.96, 0.97, 1.0, 0.8], 'camera': camera}
    a.update(kw)
    return a


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pack', nargs='?', default=str(DEFAULT_PACK))
    ap.add_argument('--out', default=str(OUT))
    ap.add_argument('--player-commit', default=None, help='shown in the caption line (default: the demo player commit file)')
    ap.add_argument('--hero', default='Unity c08 s1', help='the run (manifest id) to show')
    ap.add_argument('--t-wide', type=float, default=None, help='run time of the wide still (default: t_dock)')
    ap.add_argument('--t-dock', type=float, default=38.5, help='run time of the dock still')
    ap.add_argument('--dock-bearing', type=float, default=300.0, help='deg from east, CCW, target -> camera')
    ap.add_argument('--dock-distance', type=float, default=4.4)
    ap.add_argument('--dock-height', type=float, default=4.2)
    ap.add_argument('--dock-fov', type=float, default=36.0)
    ap.add_argument('--dock-target-dx', type=float, default=0.175, help='shift of the dock target along +x (east), m')
    ap.add_argument('--wide-pos', default='6.4745,-14.4637,12.0196')
    ap.add_argument('--wide-fov', type=float, default=50.0)
    ap.add_argument('--look', default=None, help='look preset name written as the scripts\' "look" (player >= look merge)')
    a = ap.parse_args(argv)
    pack, out = Path(a.pack), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    man = json.loads((pack / 'manifest.json').read_text())
    truth = json.loads((pack / man['world'].get('truth', 'course_truth.json')).read_text())
    dc = man.get('decisioncal', {})
    run_id = dc.get('run_ids', {}).get(a.hero, a.hero)
    t_dock = float(dc.get('metrics', {}).get('t_dock', 43.08))
    commit = a.player_commit
    if commit is None:
        f = Path.home() / '.cache' / 'decisioncal' / 'demo-player' / 'player-linux' / 'commit'
        commit = f.read_text().strip() if f.is_file() else ''
    backend = 'Unity lockstep plant (CRANE C#, HDRP water)' if run_id.startswith('unity') else 'NumPy plant'
    cx, cy = bounds_centre(man)
    sx, sy = slip_centre(truth) or (cx, cy)
    wpos = [float(v) for v in a.wide_pos.split(',')]
    title = 'Gate, U-turn, slip dock: one recorded run'
    head = {'fps': 30, 'size': [1920, 1080], 'title': title, 'hud': 'minimal', 'backend': backend,
            'note': f'make_demo_scripts.py; replay of the run\'s own 50 Hz trajectory; player {commit[:7]}'}
    if a.look:
        head['look'] = a.look
    wide = act(a.t_wide if a.t_wide is not None else t_dock, run_id,
               {'shot': 'wide', 'pos': wpos, 'look': [cx, cy, 0.0], 'fov': a.wide_fov}, a.hero,
               'wide: the whole route from the south (the earlier wide still\'s viewpoint), boat docked')
    dock_cam = {'shot': 'dock', 'target': [sx + a.dock_target_dx, sy, 0.0], 'bearing': a.dock_bearing, 'distance': a.dock_distance,
                'height': a.dock_height, 'look_height': 0.15, 'fov': a.dock_fov}
    dock = act(a.t_dock, run_id, dock_cam, a.hero, 'dock: fixed camera on the slip, three-quarter view; the boat enters the frame',
               trail_width=0.05)
    (out / 'script_demo_wide.json').write_text(json.dumps(dict(head, acts=[wide]), indent=1) + '\n')
    (out / 'script_demo_dock.json').write_text(json.dumps(dict(head, acts=[dock]), indent=1) + '\n')
    cands = [wide]
    for t in (37.8, a.t_dock, 40.0, t_dock + 1.0):
        for b, d, h in ((a.dock_bearing, a.dock_distance, a.dock_height), (335.0, 4.4, 4.2), (105.0, 3.6, 3.6)):
            c = dict(dock_cam, bearing=b, distance=d, height=h)
            cands.append(act(t, run_id, c, a.hero, f'dock candidate t={t:.2f} bearing={b:g} distance={d:g} height={h:g}',
                             trail_width=0.05))
    (out / 'script_demo_candidates.json').write_text(json.dumps(dict(head, acts=cands), indent=1) + '\n')
    print(json.dumps({'out': str(out), 'slip_centre': [sx, sy], 'bounds_centre': [cx, cy], 'run_id': run_id,
                      't_dock': t_dock, 'candidates': len(cands)}, indent=1))


if __name__ == '__main__':
    main()
