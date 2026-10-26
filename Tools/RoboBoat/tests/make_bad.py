#!/usr/bin/env python3
"""Writes tests/bad/*.json: hand-built bad layouts, each one mutation of the good base truth
(fixtures/roboboat_2026-s1.json) that the validator must reject on exactly one row, named in
``expect.fail``. The mutation helpers here are shared with test_course.py's per-row limit table.
Run after the base fixture or the spec changes; the outputs are committed."""

import copy
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = json.loads((HERE / 'fixtures' / 'roboboat_2026-s1.json').read_text())


def el(t, ident):
    return next(e for e in t['elements'] if e['id'] == ident)


def gate(t, ident):
    return next(g for g in t['gates'] if g['id'] == ident)


def bay(t, ident):
    return next(b for b in t['bays'] if b['id'] == ident)


def task(t, ident):
    return next(x for x in t['tasks'] if x['id'] == ident)


def anchor(t, ref):
    """'task.name' -> the anchor dict."""
    task_id, _, name = ref.partition('.')
    return task(t, task_id)['anchors'][name]


def state(t, ident):
    return next(s for s in t['states'] if s['id'] == ident)


def xy(o):
    return (o['x'], o['y'])


def unit(yaw):
    return (math.cos(yaw), math.sin(yaw))


def transform(t, ids=(), gates=(), bays=(), anchors=(), footprints=(), centre=(0.0, 0.0), deg=0.0, dx=0.0, dy=0.0):
    """Rigidly moves objects: rotate by ``deg`` about ``centre``, then translate by (dx, dy)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)

    def move(o):
        x, y = o['x'] - centre[0], o['y'] - centre[1]
        o['x'], o['y'] = round(centre[0] + c * x - s * y + dx, 6), round(centre[1] + s * x + c * y + dy, 6)

    for e in t['elements']:
        if e['id'] in ids:
            move(e)
            if 'yaw' in e:
                e['yaw'] += a
    for g in t['gates']:
        if g['id'] in gates:
            move(g)
            g['normal_yaw'] += a
    for b in t['bays']:
        if b['id'] in bays:
            move(b)
            b['yaw'] += a
    for ref in anchors:
        move(anchor(t, ref))
    for x in t['tasks']:
        if x['id'] in footprints:
            pts = [{'x': p[0], 'y': p[1]} for p in x['footprint']]
            for p in pts:
                move(p)
            x['footprint'] = [[p['x'], p['y']] for p in pts]


def gate_posts(t, gate_id):
    g = gate(t, gate_id)
    return el(t, g['port']), el(t, g['starboard'])


def set_cc(t, gate_id, cc):
    """Post centre-to-centre distance about the gate centre, along the gate line."""
    g = gate(t, gate_id)
    p, s = gate_posts(t, gate_id)
    mx, my = (p['x'] + s['x']) / 2, (p['y'] + s['y']) / 2
    lx, ly = -math.sin(g['normal_yaw']), math.cos(g['normal_yaw'])
    p['x'], p['y'] = mx + lx * cc / 2, my + ly * cc / 2
    s['x'], s['y'] = mx - lx * cc / 2, my - ly * cc / 2


def set_clear(t, gate_id, clear):
    """Post radii grown evenly until the clear width (cc minus radii) is ``clear``."""
    p, s = gate_posts(t, gate_id)
    r = (math.dist(xy(p), xy(s)) - clear) / 2
    p['radius'] = s['radius'] = r


def rotate_gate(t, gate_id, deg):
    g = gate(t, gate_id)
    transform(t, ids={g['port'], g['starboard']}, gates={gate_id}, centre=xy(g), deg=deg)


def move_gate(t, gate_id, dx, dy):
    g = gate(t, gate_id)
    transform(t, ids={g['port'], g['starboard']}, gates={gate_id}, dx=dx, dy=dy)


def move_el(t, ident, dx, dy):
    transform(t, ids={ident}, dx=dx, dy=dy)


def gate_frame(t, gate_id, point):
    """(along, lateral) of a point in a gate's frame (lateral positive to port)."""
    g = gate(t, gate_id)
    f = unit(g['normal_yaw'])
    d = (point[0] - g['x'], point[1] - g['y'])
    return d[0] * f[0] + d[1] * f[1], -d[0] * f[1] + d[1] * f[0]


def place_in_gate_frame(t, gate_id, ident, along=None, lateral=None):
    """Moves an element to the given along/lateral of a gate (None keeps that coordinate)."""
    g = gate(t, gate_id)
    e = el(t, ident)
    a0, l0 = gate_frame(t, gate_id, xy(e))
    a, lat = a0 if along is None else along, l0 if lateral is None else lateral
    f = unit(g['normal_yaw'])
    return (g['x'] + f[0] * a - f[1] * lat - e['x'], g['y'] + f[1] * a + f[0] * lat - e['y'])


# ---- object groups -------------------------------------------------------------------------

SPEED_ANCHORS = ('speed_challenge.approach', 'speed_challenge.mark', 'harbor_alert.zone1')


def speed_group(t, deg=0.0, dx=0.0, dy=0.0):
    """The whole speed challenge (gate, mark, marker, sign, its anchors and harbor zone 1, which
    sits on the mark), rotated about the gate centre and/or translated."""
    g = gate(t, 'speed_challenge.gate')
    transform(t, ids={e['id'] for e in t['elements'] if e['task'] == 'speed_challenge'}, gates={g['id']},
              anchors=SPEED_ANCHORS, footprints={'speed_challenge'}, centre=xy(g), deg=deg, dx=dx, dy=dy)


def hulls_in(t, bays):
    """Ids of the supply hulls whose centre lies in one of ``bays``."""
    out = set()
    for e in t['elements']:
        if e['task'] != 'supply_drop':
            continue
        for b in bays:
            f = unit(b['yaw'])
            d = (e['x'] - b['x'], e['y'] - b['y'])
            if abs(d[0] * f[0] + d[1] * f[1]) <= b['depth'] / 2 and abs(-d[0] * f[1] + d[1] * f[0]) <= b['width'] / 2:
                out.add(e['id'])
    return out


def pier_group(t, pier, deg=0.0, dx=0.0, dy=0.0, centre=None):
    """One pier (spine, tines, slip signs/indicators, moored and docked boats, bays, its tip anchor)."""
    bays = [b for b in t['bays'] if b['task'] == 'marina' and b['pier'] == pier]
    ids = {e['id'] for e in t['elements'] if e['id'].startswith(f'marina.{pier}.') or
           any(e['id'].startswith(b['id'] + '.') for b in bays)} | hulls_in(t, bays)
    spine = el(t, f'marina.{pier}.spine')
    transform(t, ids=ids, bays={b['id'] for b in bays}, anchors=(f'marina.tips_{pier}',),
              centre=centre or xy(spine), deg=deg, dx=dx, dy=dy)


def marina_group(t, dx=0.0, dy=0.0):
    """The whole marina with its docked hulls, anchors and footprint."""
    bays = [b for b in t['bays'] if b['task'] == 'marina']
    ids = {e['id'] for e in t['elements'] if e['task'] == 'marina'} | hulls_in(t, bays)
    transform(t, ids=ids, bays={b['id'] for b in bays}, anchors=[f'marina.{k}' for k in task(t, 'marina')['anchors']],
              footprints={'marina'}, dx=dx, dy=dy)


def marina_turn(t, deg):
    """The whole marina with its docked hulls, anchors and footprint, rotated about the middle of its spines."""
    bays = [b for b in t['bays'] if b['task'] == 'marina']
    ids = {e['id'] for e in t['elements'] if e['task'] == 'marina'} | hulls_in(t, bays)
    spines = [el(t, f'marina.{p}.spine') for p in 'NS']
    transform(t, ids=ids, bays={b['id'] for b in bays}, anchors=[f'marina.{k}' for k in task(t, 'marina')['anchors']],
              footprints={'marina'}, centre=(sum(s['x'] for s in spines) / 2, sum(s['y'] for s in spines) / 2), deg=deg)


def hull_by_lane(t, off):
    """The free hull ``off`` metres off the middle of the lead-in leg from entry gate 2 to channel gate 1."""
    a, b = xy(gate(t, 'entry_gates.gate2')), xy(gate(t, 'debris_clearance.gate1'))
    d = math.dist(a, b)
    u = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
    el(t, 'supply_drop.water1').update(x=round((a[0] + b[0]) / 2 + u[1] * off, 6), y=round((a[1] + b[1]) / 2 - u[0] * off, 6))


def zone2_group(t, x, y):
    """Harbor zone 2 (buoy, zone anchor, its approach) moved so the buoy sits at (x, y)."""
    b = el(t, 'harbor_alert.zone2_buoy')
    dx, dy = x - b['x'], y - b['y']
    transform(t, ids={b['id']}, anchors=('harbor_alert.zone2', 'harbor_alert.approach'), dx=dx, dy=dy)


def add_hull(t, ident, x, y, kind='roboboat/hull_ball_black'):
    h = dict(copy.deepcopy(el(t, 'supply_drop.water1')), id=ident, x=x, y=y, kind=kind)
    t['elements'].append(h)
    return h


def slip_hull(t, n=0):
    return [e for e in t['elements'] if e['task'] == 'supply_drop' and e['role'] == 'slip_hull'][n]


def dock_hull(t, bay_id, depth=0.0):
    """Moves the docked hull into a slip, ``depth`` metres back from the slip centre (toward the spine)."""
    b = bay(t, bay_id)
    f = unit(b['yaw'])
    slip_hull(t).update(x=b['x'] - f[0] * depth, y=b['y'] - f[1] * depth)


# ---- bad layouts ---------------------------------------------------------------------------

def gate_cc_3_3(t):
    set_cc(t, 'entry_gates.gate1', 3.3)


def gate_cc_narrow(t):
    set_cc(t, 'debris_clearance.gate2', 1.6)


def gate1_posts_fat(t):
    # entry gate 1's posts swell until 1.10 m is clear, below beam + 0.3 (no path runs through T1)
    set_clear(t, 'entry_gates.gate1', 1.10)


def slip_narrow_for_vessel(t):
    # a red slip at the handbook minimum width and a 1.1 m beam: the slip passes fig 60 but not the vessel
    bay(t, 'marina.S3')['width'] = 1.38
    t['vessel']['beam'] = 1.10


def red_to_starboard(t):
    p, s = gate_posts(t, 'entry_gates.gate1')
    (p['x'], p['y']), (s['x'], s['y']) = (s['x'], s['y']), (p['x'], p['y'])


def two_lowest_greens(t):
    # the other pier's slip with the lowest green number turns green too (it holds no boat)
    low = min((b for b in t['bays'] if b['indicator']['color'] == 'green'), key=lambda b: b['sign']['number'])
    twin = next(b for b in t['bays'] if b['pier'] != low['pier'] and b['sign']['number'] == low['sign']['number'])
    assert not twin['occupied'] and not hulls_in(t, [twin])
    twin['indicator']['color'] = 'green'


def green_occupied(t):
    # the green slip gets a moored boat flag but stays green: green_free and occupied_red are one rule
    bay(t, 'marina.S1')['occupied'] = True


def numbers_duplicate(t):
    # S1 repeats S2's number; the lowest green (N3, number 1) stays unique
    bay(t, 'marina.S1')['sign']['number'] = bay(t, 'marina.S2')['sign']['number']


def marker_beyond_mark(t):
    mark = el(t, 'speed_challenge.mark')
    marker = el(t, 'speed_challenge.marker')
    g = gate(t, 'speed_challenge.gate')
    marker['x'] = mark['x'] + math.cos(g['normal_yaw']) * 4
    marker['y'] = mark['y'] + math.sin(g['normal_yaw']) * 4


def leg_too_short(t):
    # the turn mark (and harbor zone 1 on it) comes in to 11 m
    a, _ = gate_frame(t, 'speed_challenge.gate', xy(el(t, 'speed_challenge.mark')))
    g = gate(t, 'speed_challenge.gate')
    f = unit(g['normal_yaw'])
    d = 11.0 - a
    transform(t, ids={'speed_challenge.mark'}, anchors=('speed_challenge.mark', 'harbor_alert.zone1'), dx=f[0] * d, dy=f[1] * d)


def direction_wrong(t):
    # the marker turns green (cw) but the task still says ccw
    color = 'green' if state(t, 'speed_challenge.marker')['initial'] == 'red' else 'red'
    state(t, 'speed_challenge.marker').update(initial=color, program={'type': 'fixed', 'value': color})


def footprints_overlap(t):
    task(t, 'supply_drop')['footprint'] = copy.deepcopy(task(t, 'marina')['footprint'])


def channel_blocked(t):
    g = gate(t, 'debris_clearance.gate2')
    t['elements'].append(dict(copy.deepcopy(el(t, 'debris_clearance.channel_black1')),
                              id='debris_clearance.blocker', x=g['x'], y=g['y']))


def no_room_at_green(t):
    green = el(t, 'debris_clearance.green_beacon1')
    t['elements'].append(dict(copy.deepcopy(el(t, 'debris_clearance.black1')), id='debris_clearance.crowd',
                              x=green['x'] + 1.75, y=green['y']))


def gate2_yaw_16(t):
    # gate 1 turns 9 deg away from gate 2 (gate 2 and the channel stay put)
    rotate_gate(t, 'entry_gates.gate1', -9.0)


def separation_short(t):
    g1, g2 = gate(t, 'entry_gates.gate1'), gate(t, 'entry_gates.gate2')
    move_gate(t, g2['id'], g1['x'] + 5 - g2['x'], g1['y'] - g2['y'])


def horn_1200(t):
    task(t, 'harbor_alert')['semantics']['horn']['freq_hz'] = 1200.0


def zone1_far(t):
    anchor(t, 'harbor_alert.zone1')['x'] += 5.0


def outside_boundary(t):
    # a field black drifts past the south edge of the course area
    e = el(t, 'debris_clearance.black8')
    e['y'] = -65.0 + e['radius'] - 0.2


def debris_heading_16(t):
    # entry gate 2 turns 7 deg further from the channel's first gate (still within 15 deg of gate 1)
    rotate_gate(t, 'entry_gates.gate2', 7.0)


def marina_unrooted(t):
    # the marina slides 2.5 m off the shore line (zone 2 follows so it stays 7-12 m off the mouth)
    marina_group(t, dx=2.5)


def hull_in_green_slip(t):
    # docked at the back of a green slip, clear of the path into it
    dock_hull(t, 'marina.N2', depth=1.17)


def hull_with_moored_boat(t):
    dock_hull(t, 'marina.S2', depth=1.17)


def hull_beside_moored(t):
    # S3 is red and free but sits next to the moored boat in S2
    dock_hull(t, 'marina.S3')


def none_docked(t):
    t['elements'].remove(slip_hull(t))


def hulls_clumped(t):
    # two more free hulls 4 m apart, one by harbor zone 2 and one by the marina (still two tasks)
    add_hull(t, 'supply_drop.ball2', 4.75, 23.5)
    add_hull(t, 'supply_drop.water2', 4.75 + 4.0 * math.cos(-1.2), 23.5 + 4.0 * math.sin(-1.2), kind='roboboat/hull_water_orange')


def hulls_one_task(t):
    # a second free hull 21 m from the first, but both nearest the debris field
    add_hull(t, 'supply_drop.ball2', 45.0, -12.0)


def hull_far_from_tasks(t):
    # the free hull drifts to the empty south-east corner
    el(t, 'supply_drop.water1').update(x=110.0, y=-55.0)


def speed_gate_turned(t):
    # the speed challenge turns 45 deg about its gate: it faces away from entry gate 2 (62 deg)
    # while still facing the channel exit (37 deg)
    speed_group(t, deg=45.0)


def zone2_by_lane(t):
    # zone 2 sits 13.5 m off the entry lane (15 m from the start, 10 m off the marina mouth)
    zone2_group(t, 9.0, 13.5)


def hull_in_lane(t):
    # the free hull 0.65 m off the leg from entry gate 2 to channel gate 1
    hull_by_lane(t, 0.65)


def marina_into_shore(t):
    # the marina turns 40 deg about its middle: its piers now point 94 deg off the shore normal, toward the shore
    marina_turn(t, 40.0)


def no_light_sign(t):
    t['elements'].remove(el(t, 'speed_challenge.sign'))


CASES = {
    # name: (mutation, the one row that must fail)
    'gate_cc_3_3': (gate_cc_3_3, 'T1.cc'),
    'gate_cc_narrow': (gate_cc_narrow, 'T2.cc'),
    'gate1_posts_fat': (gate1_posts_fat, 'T1.clear'),
    'slip_narrow_for_vessel': (slip_narrow_for_vessel, 'T5.slip_vessel'),
    'red_to_starboard': (red_to_starboard, 'T1.side'),
    'two_lowest_greens': (two_lowest_greens, 'T5.lowest_green'),
    'numbers_duplicate': (numbers_duplicate, 'T5.numbers'),
    'marker_beyond_mark': (marker_beyond_mark, 'T3.marker_between'),
    'leg_too_short': (leg_too_short, 'T3.leg'),
    'direction_wrong': (direction_wrong, 'T3.direction_marker'),
    'footprints_overlap': (footprints_overlap, 'G4.no_overlap'),
    'channel_blocked': (channel_blocked, 'P1.channel'),
    'no_room_at_green': (no_room_at_green, 'P3.rescue_circle'),
    'gate2_yaw_16': (gate2_yaw_16, 'T1.gate2_yaw'),
    'separation_short': (separation_short, 'T1.separation'),
    'horn_1200': (horn_1200, 'T6.freq'),
    'zone1_far': (zone1_far, 'T6.zone1'),
    'outside_boundary': (outside_boundary, 'G1.inside'),
    'debris_heading_16': (debris_heading_16, 'T2.heading'),
    'marina_unrooted': (marina_unrooted, 'T5.shore'),
    'hull_in_green_slip': (hull_in_green_slip, 'T4.dock_green_free'),
    'hull_with_moored_boat': (hull_with_moored_boat, 'T4.dock_moored_free'),
    'hull_beside_moored': (hull_beside_moored, 'T4.dock_beside_moored'),
    'none_docked': (none_docked, 'T4.docked'),
    'hulls_clumped': (hulls_clumped, 'T4.spacing'),
    'hulls_one_task': (hulls_one_task, 'T4.scatter'),
    'hull_far_from_tasks': (hull_far_from_tasks, 'T4.near_task'),
    'speed_gate_turned': (speed_gate_turned, 'T3.approach_gate2'),
    'zone2_by_lane': (zone2_by_lane, 'T6.zone2_lane'),
    'no_light_sign': (no_light_sign, 'T3.sign'),
    'hull_in_lane': (hull_in_lane, 'T4.lane'),
    'marina_into_shore': (marina_into_shore, 'T5.yaw'),
}

if __name__ == '__main__':
    out = HERE / 'bad'
    out.mkdir(exist_ok=True)
    for f in out.glob('*.json'):
        f.unlink()
    for name, (mutate, row) in CASES.items():
        t = copy.deepcopy(BASE)
        mutate(t)
        t['expect'] = {'fail': [row], 'exact': True}
        (out / f'{name}.json').write_text(json.dumps(t, indent=1) + '\n')
    print(f'{len(CASES)} bad layouts in {out}')
