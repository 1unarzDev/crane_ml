#!/usr/bin/env python3
"""Theater input for the demo: manifest + track pack + course truth + one script per shot.

    python3 make_pack.py --unity-run RUN [--numpy RUN/numpy-....ctraj] [--out PACK] [--hz 50] [--water still|free]

RUN is a capture.py run dir. The pack holds the Unity run (RUN/run.mission.ctraj, the mission window of the
player's 50 Hz trajectory, 6-DOF) and optionally the NumPy run (planar), resampled to ``--hz`` (default 50,
the capture rate). The course truth is RUN/course_truth.in.json, the same file the capture replayed, so the
Theater scene has the same gate buoys and slip walls in the same place as the live run.

The world's centre line is the mission route (mission.PATH mapped course -> world with the run's hello), so
the pack's progress channel is route progress; the world bounds are the route's box plus a margin, so the
``top`` shot frames the route rather than the whole 130 m course area.

Shot scripts (``--crane-theater-script``; frame times at 30 fps; Theater's time is per frame, never a clock):
  script_stills.json   six 1 s frozen acts (speed 0.01): three wide candidates, three close candidates
  script_clip.json     <= 15 s: a wide establishing act, then a chase into the slip
  script_compare.json  two 1 s frozen acts with gold = Unity, silver = NumPy (needs --numpy)
The run ids and backends go in the title; per-act captions say what the shot is.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import _paths  # noqa: E402,F401
import mission  # noqa: E402
from theater import pack as P  # noqa: E402
from theater import traj as T  # noqa: E402
from theater.worlds import RoboBoatWorld  # noqa: E402

FPS = 30


class RouteWorld(RoboBoatWorld):
    """The RoboBoat world plug-in with the DecisionCal route as its centre line and its box as the bounds."""

    def __init__(self, truth, start, margin=3.0):
        super().__init__(truth)
        x0, y0, yaw0 = start
        c, s = math.cos(yaw0), math.sin(yaw0)
        pts = mission.PATH[::10]
        self.line = [(x0 + c * px - s * py, y0 + s * px + c * py) for px, py in pts.tolist()]
        S = mission.SLIP
        extra = [(S['x0'] - S['wall'], S['y'] - S['half'] - S['wall']), (mission.GATE_X, mission.GATE_Y + 0.3)]
        allp = self.line + [(x0 + c * px - s * py, y0 + s * px + c * py) for px, py in extra]
        xs, ys = [p[0] for p in allp], [p[1] for p in allp]
        self.box = (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)

    def start(self):
        t = self.truth
        return float(t['startX']), float(t['startY']), float(t['startYaw'])

    def centreline(self):
        return self.line

    def bounds(self, margin=None):
        return self.box


def title_for(unity_id, numpy_id=None):
    # The HUD's title panel and the centre gold/silver panel overlap when the title is wider than ~600 px, so
    # the title is short; the full run ids go in the act captions and in the caption bar below the frame.
    if numpy_id:
        return 'Unity (gold) vs NumPy (silver)'
    return 'Unity plant, external controller c08'


def display_id(header):
    """Short id the HUD prints in the gold/silver row (the full run id overflows it)."""
    rec = header['build']['recorder']
    p = header.get('params') or {}
    tag = 'Unity' if rec == 'unity' else 'NumPy' if rec == 'numpy-plant' else rec
    return f"{tag} c{int(p.get('controller', 0)):02d} s{p.get('noise_seed', '?')}"


def act(note, t0, shot, duration=1.0, speed=0.01, kind='spotlight', hero=None, previous='none', caption=None, **extra):
    a = {'type': kind, 'duration_s': duration, 'speed': speed, 't0': round(t0, 3), 'caption': caption or note, 'note': note,
         'colour': 'progress', 'trails': 'none', 'markers': True, 'hero': hero, 'previous': previous, 'camera': shot}
    a.update(extra)
    return a


def scripts(unity_id, numpy_id, t_dock, t_end, size=(1920, 1080), full_ids=None):
    """Shot scripts. ``unity_id``/``numpy_id`` are the pack's (display) run ids; ``full_ids`` maps them to the
    capture's run ids, which go in the HUD caption line."""
    out = {}
    full_ids = full_ids or {}
    hero = unity_id
    run_cap = f"run {full_ids.get(unity_id, unity_id)}"
    only = {'iterations': [0, 0]}          # the Unity run only (the NumPy run is iteration 1)
    t_close = max(0.0, t_dock - 3.0)

    def a1(note, t0, shot, **kw):
        return act(note, t0, shot, hero=hero, caption=run_cap, **only, **kw)

    stills = [
        a1('wide: whole route from the south (gate, U-turn, slip), boat docked', t_dock,
           {'shot': 'top', 'pitch': 58, 'margin': 1.0, 'push': 0.0}),
        a1('wide: whole route, steep view, boat docked', t_dock, {'shot': 'top', 'pitch': 75, 'margin': 1.0, 'push': 0.0}),
        a1('wide: oblique from the east, return leg', t_dock - 12.0,
           {'shot': 'orbit', 'angle': 345, 'radius': 16, 'height': 9, 'deg_per_s': 0}),
        a1('close: final approach into the slip, from behind', t_close,
           {'shot': 'chase', 'back': 3.2, 'height': 1.8, 'lead': 1.2, 'fov': 50}),
        a1('close: final approach into the slip, three-quarter view', t_close,
           {'shot': 'chase', 'back': 2.6, 'height': 2.0, 'side': 2.4, 'lead': 0.6, 'fov': 50}),
        a1('close: docked, seen from beyond the end wall', t_dock + 1.0,
           {'shot': 'chase', 'back': -3.6, 'height': 1.6, 'lead': -1.2, 'fov': 50}),
    ]
    title = title_for(unity_id)
    out['stills'] = {'fps': FPS, 'size': list(size), 'title': title, 'acts': stills}
    # clip (13 s): 3 s wide over run time t_dock-24 .. t_dock-15, then 10 s chase from t_dock-15 to the episode end
    t_mid = max(0.0, t_dock - 24.0)
    clip = [
        a1('return leg toward the slip (wide)', t_mid, {'shot': 'top', 'pitch': 60, 'margin': 1.0, 'push': 0.05},
           duration=3.0, speed=3.0),
        a1('final approach into the slip (chase)', t_mid + 9.0, {'shot': 'chase', 'back': 3.4, 'height': 2.0, 'lead': 1.2},
           duration=10.0, speed=round(max(0.5, (t_end - (t_mid + 9.0)) / 10.0), 3)),
    ]
    out['clip'] = {'fps': FPS, 'size': list(size), 'title': title, 'acts': clip}
    if numpy_id:
        cap = 'NumPy is planar: seated flat, no heave or roll'
        kw = dict(kind='compare', hero=unity_id, previous=numpy_id, caption=cap, trails='top', trail_k=0)
        cmp_ = [
            act('compare: wide, return leg', t_dock - 12.0, {'shot': 'top', 'pitch': 62, 'margin': 1.0, 'push': 0.0}, **kw),
            act('compare: close, final approach', t_dock - 2.0, {'shot': 'chase', 'back': 3.6, 'height': 2.4, 'side': 1.5, 'lead': 1.0}, **kw),
            act('compare: wide, U-turn', 20.0, {'shot': 'top', 'pitch': 62, 'margin': 1.0, 'push': 0.0}, **kw),
        ]
        out['compare'] = {'fps': FPS, 'size': list(size), 'title': title_for(unity_id, numpy_id), 'acts': cmp_}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--unity-run', required=True)
    ap.add_argument('--numpy', default=None, help='a numpy_to_ctraj.py output')
    ap.add_argument('--out', default=None, help='default: ~/.cache/decisioncal/demo/packs/<run id>')
    ap.add_argument('--hz', type=float, default=50.0)
    ap.add_argument('--water', choices=('still', 'free'), default='still')
    a = ap.parse_args(argv)
    run = Path(a.unity_run)
    summ = json.loads((run / 'summary.json').read_text())
    out = Path(a.out) if a.out else Path.home() / '.cache' / 'decisioncal' / 'demo' / 'packs' / summ['run_id']
    trajs = out / 'trajs'
    if trajs.exists():
        for f in trajs.glob('*.ctraj'):
            f.unlink()
    trajs.mkdir(parents=True, exist_ok=True)
    runs, full = [], {}
    for it, src in enumerate([run / 'run.mission.ctraj'] + ([Path(a.numpy)] if a.numpy else [])):
        t = T.read(src)
        rid = display_id(t.header)
        full[rid] = t.header['run_id']
        (trajs / f'{rid}.ctraj').write_bytes(src.read_bytes())
        # HUD score = J (s; lower is better). Theater ranks higher scores as better; nothing here relies on that
        # (heroes are named explicitly), but the HUD's "BEST SCORE" panel is then just the J of the run(s) shown.
        runs.append({'run_id': rid, 'iteration': it, 'outcome': t.header.get('outcome'), 'score': t.header.get('J'),
                     'progress': t.header.get('progress'), 'valid': True,
                     # params: only fields that are equal across runs, so the HUD's "WHAT CHANGED" panel stays empty
                     # (it would print J as if it were a changed parameter)
                     'params': {**{k: v for k, v in (t.header.get('params') or {}).items() if isinstance(v, (int, float))}}})
    unity_id = runs[0]['run_id']
    numpy_id = runs[1]['run_id'] if len(runs) > 1 else None
    truth = json.loads((run / 'course_truth.in.json').read_text())
    world = RouteWorld(truth, tuple(summ['hello']))
    select = {'schema': 'decisioncal-demo-select/1', 'study': title_for(unity_id, numpy_id), 'runs': runs,
              'water': {'mode': a.water, 't0_s': 0.0}}
    doc = P.from_select(select, trajs, world, out, hz=a.hz)
    doc['robot'] = {'plugin': 'roboboat', 'model': 'blastoise'}
    doc['decisioncal'] = {'unity_run': str(run), 'unity_run_id': summ['run_id'], 'run_ids': full,
                          'route_world_start': summ['hello'], 'metrics': summ.get('metrics'),
                          'note': 'gate buoys and slip walls are scene objects built from course_truth.json (the truth the live run '
                                  'replayed); gold/silver shells, rings and trails are Theater overlays; HUD score = J'}
    (out / 'manifest.json').write_text(json.dumps(doc, indent=1, sort_keys=True) + '\n')
    met = summ.get('metrics') or {}
    t_dock = float(met.get('t_dock') or summ.get('t_final', 40.0))
    t_end = float(summ.get('t_final', t_dock + 2.0))
    for name, s in scripts(unity_id, numpy_id, t_dock, t_end, full_ids=full).items():
        (out / f'script_{name}.json').write_text(json.dumps(s, indent=1) + '\n')
        frames = sum(round(x['duration_s'] * s['fps']) for x in s['acts'])
        print(f'script_{name}.json: {len(s["acts"])} acts, {frames} frames')
    print(f"{len(doc['runs'])} runs {full}, hz {a.hz}, water {a.water}, pose_dims {doc.get('pose_dims')} -> {out}")


if __name__ == '__main__':
    main()
