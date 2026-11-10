#!/usr/bin/env python3
"""Course truth that makes the DecisionCal route visible in the Unity 'Roboboat Course' scene.

The study's mission (``mission.py``) is defined in a COURSE frame: start (0, 0) heading +x, a gate of two
buoys at (10, +-1), a right U-turn, then west along y = -5 into a slip whose walls are y = -5 +- 0.6 for
x in [2, 5] (0.1 m thick) and whose end wall is at x = 2 (0.1 m thick, behind it). In the lockstep runs the
gate and the slip are VIRTUAL: the harness counts contacts against them in Python.

This writes a ``crane-roboboat-course-truth-v1`` document (frame ``world`` = /crane/odom ENU, what
``--crane-course-replay`` reads; ``ReplayCourseSource.InWorldFrame`` is true, so element poses are used as
given) whose ONLY elements are the route's own geometry, mapped course -> world with the player's start pose
(``RoboBoatCourse.ToWorld``: w = start + R(yaw0) c, yaw_w = yaw_c + yaw0):

  * gate: ``roboboat/taylor_red`` at course (10, +1) (port of +x travel) and ``roboboat/taylor_green`` at
    (10, -1). Catalog radius 0.229 m (the harness's virtual buoy radius is 0.15 m, so the real pole is
    slightly fatter than the scored one).
  * slip: three ``roboboat/dock_tine`` boxes (the catalog's Box placeholder; size from the element's
    ``width`` x ``length``, height 0.406 m from the waterline up), placed EXACTLY on the virtual wall
    rectangles of ``mission.SLIP``: two side walls 3.0 x 0.1 m centred at y = -5 +- 0.65 and the end wall
    1.4 x 0.1 m centred at x = 1.95.

These are real scene objects with colliders (CreatePrimitive boxes; template buoys made solid). The
seed-1 generated course the earlier lockstep runs used (entry gate at course x = 3.06, marina, ...) is NOT
included, so the scene shows the route and nothing else from the course family (the family's scene groups
are hidden by the builder as usual).

    python3 route_truth.py --out course_truth.in.json [--start -2.275,-4.753,0]
"""

import argparse
import copy
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import mission  # noqa: E402

# The 'Roboboat Course' scene's boat start (every lockstep run so far reported this in `hello`; course_truth
# written by the player carries the same startX/startY/startYaw). capture.py checks hello against it.
SCENE_START = (-2.275, -4.753, 0.0)
# The envelope of a truth written by the e27629b player for course seed 1 (schema, family, handbook,
# boundary, shore, vessel; see truth_envelope.json). Elements, gates, bays, tasks, states and
# constraints are replaced.
TEMPLATE = HERE / 'truth_envelope.json'


def to_world(start, cx, cy, cyaw=0.0):
    x0, y0, yaw0 = start
    c, s = math.cos(yaw0), math.sin(yaw0)
    return x0 + c * cx - s * cy, y0 + s * cx + c * cy, (cyaw + yaw0 + math.pi) % (2 * math.pi) - math.pi


def route_elements(start):
    """(elements, gates) in the world frame. Geometry from mission.py's constants."""
    gx, gy = mission.GATE_X, mission.GATE_Y
    S = mission.SLIP
    w = S['wall']
    els = []

    def el(id_, kind, label, role, cx, cy, cyaw=0.0, radius=0.229, height=0.99, **extra):
        x, y, yaw = to_world(start, cx, cy, cyaw)
        d = {'task': 'decisioncal_route', 'label': label, 'x': round(x, 4), 'y': round(y, 4), 'z': 0, 'yaw': round(yaw, 4),
             'radius': radius, 'height': height, 'id': id_, 'role': role, 'kind': kind, 'state': None,
             'course_xy': [cx, cy]}
        d.update(extra)
        els.append(d)
        return d

    el('route.gate.port', 'roboboat/taylor_red', 'red_pole_buoy', 'gate', gx, +gy, template='RedPoleBuoy0')
    el('route.gate.starboard', 'roboboat/taylor_green', 'green_pole_buoy', 'gate', gx, -gy, template='GreenPoleBuoy0')
    L = S['x1'] - S['x0']
    xm = 0.5 * (S['x0'] + S['x1'])
    for side, sgn in (('north', +1), ('south', -1)):
        el(f'route.slip.wall_{side}', 'roboboat/dock_tine', '', 'slip_wall', xm, S['y'] + sgn * (S['half'] + w / 2), 0.0,
           radius=0.254, height=0.406, width=w, length=L)
    el('route.slip.end_wall', 'roboboat/dock_tine', '', 'slip_wall', S['x0'] - w / 2, S['y'], math.pi / 2,
       radius=0.254, height=0.406, width=w, length=2 * (S['half'] + w))
    mx, my, myaw = to_world(start, gx, 0.0, 0.0)
    gates = [{'id': 'route.gate', 'task': 'decisioncal_route', 'port': 'route.gate.port', 'starboard': 'route.gate.starboard',
              'order': 1, 'x': round(mx, 4), 'y': round(my, 4), 'cc': 2 * gy, 'clear': round(2 * gy - 2 * 0.229, 4),
              'normal_yaw': round(myaw, 4)}]
    return els, gates


def build(start=SCENE_START, template=TEMPLATE):
    t = copy.deepcopy(json.loads(Path(template).read_text()))
    els, gates = route_elements(start)
    t['name'] = 'decisioncal-route-v1'
    t['startX'], t['startY'], t['startYaw'] = start
    t['course_frame'] = {'x': start[0], 'y': start[1], 'yaw': start[2]}
    gen = t.get('generator') or {}
    t['generator'] = {'seed': 1, 'mode': 'authored', 'overrides': {},
                      'authored_by': 'Tools/DecisionCal/demo/route_truth.py',
                      'note': 'route geometry of Tools/DecisionCal/mission.py; envelope from a seed-1 roboboat_2026 truth',
                      'crane_commit': gen.get('crane_commit')}
    t['elements'], t['gates'] = els, gates
    t['bays'], t['tasks'], t['states'], t['constraints'] = [], [], [], []
    t['decisioncal_route'] = {
        'frame_note': 'course -> world: w = start + R(yaw0) c; mission.py constants',
        'start_world': list(start), 'gate_x': mission.GATE_X, 'gate_y': mission.GATE_Y, 'virtual_buoy_r': mission.BUOY_R,
        'slip': mission.SLIP, 'target_course': mission.TARGET.tolist()}
    return t


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--start', default=','.join(str(v) for v in SCENE_START), help='world start pose x,y,yaw (rad)')
    ap.add_argument('--template', default=str(TEMPLATE))
    a = ap.parse_args(argv)
    start = tuple(float(v) for v in a.start.split(','))
    t = build(start, a.template)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(t, indent=1) + '\n')
    for e in t['elements']:
        print(f"{e['id']:24s} {e['kind']:22s} world ({e['x']:8.3f}, {e['y']:8.3f}) yaw {e['yaw']:+.3f}  course {e['course_xy']}")
    print('->', a.out)


if __name__ == '__main__':
    main()
