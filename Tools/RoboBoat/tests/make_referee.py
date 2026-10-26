#!/usr/bin/env python3
"""Writes tests/referee/<case>/: referee fixtures per scoring tier for the testing stream, in
session-directory shape (course_truth.json, track.jsonl, events.jsonl) plus expected.json,
the verdict the case is built to produce (tier, direction, sweep, zone, contacts), written from
the case's intent rather than copied from ``course_truth.py explain``. Tracks are scripted
waypoint paths at 1 m/s on the base course (fixtures/roboboat_2026-s1.json); a case that needs a
different course (another direction source, a toggling sign) carries its own truth."""

import copy
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = json.loads((HERE / 'fixtures' / 'roboboat_2026-s1.json').read_text())
STEP = 0.2


def el(t, i):
    return next(e for e in t['elements'] if e['id'] == i)


def task(t, i):
    return next(x for x in t['tasks'] if x['id'] == i)


def gate(t, i):
    return next(g for g in t['gates'] if g['id'] == i)


def bay(t, i):
    return next(b for b in t['bays'] if b['id'] == i)


def along(p, yaw, d):
    return (p[0] + math.cos(yaw) * d, p[1] + math.sin(yaw) * d)


def xy(o):
    return (o['x'], o['y'])


def path(points, t0=0.0, speed=1.0, hold=()):
    """Samples (t, x, y) along the polyline; ``hold`` = {index: seconds} pauses at a waypoint."""
    out, t = [(t0, *points[0])], t0
    hold = dict(hold)
    for i, (a, b) in enumerate(zip(points, points[1:])):
        n = max(1, int(math.dist(a, b) / (speed * STEP)))
        for k in range(1, n + 1):
            t += math.dist(a, b) / n / speed
            out.append((round(t, 3), a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
        for _ in range(int(hold.get(i + 1, 0) / STEP)):
            t += STEP
            out.append((round(t, 3), *b))
    return out


def ring(centre, r, start_angle, degrees, n=48):
    sign = 1 if degrees > 0 else -1
    count = max(2, int(abs(degrees) / 360 * n))
    return [(centre[0] + r * math.cos(start_angle + sign * math.radians(abs(degrees)) * k / count),
             centre[1] + r * math.sin(start_angle + sign * math.radians(abs(degrees)) * k / count)) for k in range(count + 1)]


def entry_out(t):
    g1, g2 = gate(t, 'entry_gates.gate1'), gate(t, 'entry_gates.gate2')
    return [(0.0, 0.0), xy(g1), xy(g2), along(xy(g2), g2['normal_yaw'], 3)]


def speed_loop(t, direction):
    sem = task(t, 'speed_challenge')['semantics']
    g = gate(t, sem['gate'])
    mark = xy(el(t, sem['turn_mark']))
    approach = along(xy(g), g['normal_yaw'], -6)
    start = math.atan2(g['y'] - mark[1], g['x'] - mark[0])
    loop = ring(mark, 3.0, start, 380 if direction == 'ccw' else -380)
    return [approach, xy(g)] + loop + [xy(g), approach]


def dock(t, bay_id):
    b = bay(t, bay_id)
    approach = xy(task(t, 'marina')['anchors']['approach'])
    aisle = along(xy(b), b['yaw'], b['depth'] / 2 + 2)
    return path([approach, aisle, xy(b)], hold={2: 3.0})


def debris_circle(t, ident, degrees):
    c = xy(el(t, ident))
    r = 2.2
    entry = (c[0] - 6, c[1])
    return path([entry, (c[0] - r, c[1])] + ring(c, r, math.pi, degrees))


def approx(value, tol):
    """An expected number explain must reproduce within ``tol``."""
    return {'approx': round(value, 3), 'tol': tol}


def short_of(target, origin, metres):
    """The point ``metres`` before ``target`` on the way from ``origin``."""
    d = math.dist(origin, target)
    return (target[0] + (origin[0] - target[0]) * metres / d, target[1] + (origin[1] - target[1]) * metres / d)


def write(name, truth, track, expected, events=()):
    """``expected`` is what the case is built to produce, stated independently of explain: dict
    leaves must equal explain's, lists match item by item, and {'approx', 'tol'} bounds a number."""
    d = HERE / 'referee' / name
    d.mkdir(parents=True, exist_ok=True)
    target = d / 'course_truth.json'
    if target.is_symlink() or target.exists():
        target.unlink()
    if truth is BASE:
        target.symlink_to('../../fixtures/roboboat_2026-s1.json')
    else:
        target.write_text(json.dumps(truth, indent=1) + '\n')
    (d / 'track.jsonl').write_text(''.join(json.dumps({'t': t, 'x': round(x, 3), 'y': round(y, 3), 'yaw': 0.0}) + '\n' for t, x, y in track))
    (d / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    (d / 'expected.json').write_text(json.dumps(expected, indent=1) + '\n')


def main():
    t = BASE
    beam = t['vessel']['beam']
    circle_r = 8.0  # course_truth.CIRCLE_RADIUS: sweeps count within 8 m of the centre
    sem = task(t, 'speed_challenge')['semantics']
    assert sem['direction_source'] == 'marker' and sem['source_color'] == 'red'
    marker = sem['marker']['id']

    # entry gates out and back at 1 m/s: each gate crossed out, then in on the way back (t = metres)
    out = entry_out(t)
    legs = [math.dist(a, b) for a, b in zip(out, out[1:])]
    total = 2 * sum(legs)
    g1_out, g2_out = legs[0], legs[0] + legs[1]
    write('entry_out_and_back', t, path(out + out[::-1]),
          {'tasks': {'entry_gates': {'all_out': True, 'all_in': True, 'gates': [
              {'id': 'entry_gates.gate1', 'crossings': [[approx(g1_out, 0.25), 'out'], [approx(total - g1_out, 0.25), 'in']]},
              {'id': 'entry_gates.gate2', 'crossings': [[approx(g2_out, 0.25), 'out'], [approx(total - g2_out, 0.25), 'in']]}]}},
           'contacts': []})
    # straight through gate 1's port post: one contact, the hull centre on the post centre
    g1 = gate(t, 'entry_gates.gate1')
    post = el(t, g1['port'])
    write('contact_strike', t, path([(0.0, 0.0), xy(post), along(xy(g1), 0, 5)]),
          {'contacts': [g1['port']], 'min_clearance': {g1['port']: approx(-(post['radius'] + beam / 2), 0.01)}})

    # speed challenge: red marker -> ccw is right
    speed = {'source': marker, 'source_color_at_entry': 'red', 'expected': 'ccw'}
    write('speed_marker_correct', t, path(speed_loop(t, 'ccw')),
          {'tasks': {'speed_challenge': dict(speed, direction='ccw', correct=True)}, 'contacts': []})
    write('speed_marker_wrong', t, path(speed_loop(t, 'cw')),
          {'tasks': {'speed_challenge': dict(speed, direction='cw', correct=False)}, 'contacts': []})
    # the light sign is the source and shows green -> cw is right
    sign_truth = copy.deepcopy(t)
    s = task(sign_truth, 'speed_challenge')['semantics']
    s.update(direction_source='sign', source_color='green', direction='cw')
    s['sign']['program'] = {'type': 'fixed', 'value': 'green'}
    next(x for x in sign_truth['states'] if x['id'] == s['sign']['id']).update(initial='green', program=s['sign']['program'])
    sign = {'source': s['sign']['id'], 'source_color_at_entry': 'green', 'expected': 'cw'}
    write('speed_sign_correct', sign_truth, path(speed_loop(sign_truth, 'cw')),
          {'tasks': {'speed_challenge': dict(sign, direction='cw', correct=True)}, 'contacts': []})
    write('speed_sign_wrong', sign_truth, path(speed_loop(sign_truth, 'ccw')),
          {'tasks': {'speed_challenge': dict(sign, direction='ccw', correct=False)}, 'contacts': []})
    # the sign starts red but toggles to green before the boat crosses the gate (t = 6 s):
    # the colour at entry rules, so cw is right
    toggle_truth = copy.deepcopy(sign_truth)
    s = task(toggle_truth, 'speed_challenge')['semantics']
    entry_t = 6.0  # the approach point is 6 m before the gate at 1 m/s
    program = {'type': 'toggle', 'values': ['red', 'green'], 'period_s': entry_t * 0.75, 'phase_s': 0.0}
    s['sign']['program'] = program
    next(x for x in toggle_truth['states'] if x['id'] == s['sign']['id']).update(initial='red', program=program)
    write('speed_sign_toggled_mid_approach', toggle_truth, path(speed_loop(toggle_truth, 'cw')),
          {'tasks': {'speed_challenge': dict(sign, direction='cw', correct=True)}, 'contacts': []})

    # debris: rescue = full circle round a green beacon, avoid = a red beacon not circled
    deb = task(t, 'debris_clearance')['semantics']
    green, red = deb['rescue'][0], deb['avoid'][0]
    write('debris_green_full_circle', t, debris_circle(t, green, 380),
          {'tasks': {'debris_clearance': {'rescue': {green: approx(380, 1)}}}, 'contacts': []})
    write('debris_green_partial', t, debris_circle(t, green, 250),
          {'tasks': {'debris_clearance': {'rescue': {green: approx(250, 1)}}}, 'contacts': []})
    rc = xy(el(t, red))
    # a straight pass 3 m north of it: the part inside 8 m sweeps 2 atan(sqrt(8^2 - 3^2) / 3) cw
    write('debris_red_avoided', t, path([(rc[0] - 8, rc[1] + 3), (rc[0] + 8, rc[1] + 3)]),
          {'tasks': {'debris_clearance': {'avoid': {red: {
              'sweep_deg': approx(-math.degrees(2 * math.atan(math.sqrt(circle_r ** 2 - 9) / 3)), 1),
              'min_clearance': approx(3 - el(t, red)['radius'] - beam / 2, 0.01)}}}}})
    write('debris_red_circled', t, debris_circle(t, red, 380),
          {'tasks': {'debris_clearance': {'avoid': {red: {
              'sweep_deg': approx(380, 1), 'min_clearance': approx(2.2 - el(t, red)['radius'] - beam / 2, 0.01)}}}}})

    # marina: N3 is the lowest green (1), N2 another green, S3 red and free (N1 holds the docked
    # supply hull), S2 holds a moored boat
    tiers = task(t, 'marina')['semantics']['tiers']
    assert tiers['lowest_green'] == 'marina.N3' and 'marina.N2' in tiers['green']
    assert bay(t, 'marina.S3')['indicator']['color'] == 'red' and not bay(t, 'marina.S3')['occupied']
    assert bay(t, 'marina.S2')['occupied']
    for name, slip, tier, contacts in (('dock_lowest_green', 'marina.N3', 'lowest_green', []),
                                       ('dock_green_not_lowest', 'marina.N2', 'green', []),
                                       ('dock_any_unoccupied', 'marina.S3', 'unoccupied', []),
                                       # into the moored boat: the occupied tier, and a contact with it
                                       ('dock_wrong_slip_occupied', 'marina.S2', 'occupied', ['marina.S2.vessel'])):
        write(name, t, dock(t, slip), {'tasks': {'marina': {'entered': slip, 'tier': tier}}, 'contacts': contacts})

    # harbor alert: the horn calls zone 2; the boat waits at the entry exit, then runs to a zone
    # after the trigger and stops 2 m short of its centre (inside the 3 m zone, clear of buoys)
    harbor = task(t, 'harbor_alert')
    horn = harbor['semantics']['horn']
    assert horn['target_zone'] == 'zone2'
    trig = horn['program']['trigger_s']
    start = xy(task(t, 'entry_gates')['anchors']['exit'])
    idle = [(round(k * STEP, 3), *start) for k in range(int(trig / STEP))]
    z2 = xy(harbor['anchors']['zone2'])
    z1 = xy(harbor['anchors']['zone1'])
    east = (z1[0] + 10, 20.0)  # round the speed challenge's east side, clear of its gate and marker
    for name, route, zone, ok in (('horn_zone_correct', [start, short_of(z2, start, 2.0)], 'zone2', True),
                                  ('horn_zone_wrong', [start, east, short_of(z1, east, 2.0)], 'zone1', False)):
        write(name, t, idle + path(route, t0=trig),
              {'tasks': {'harbor_alert': {'target_zone': 'zone2', 'reached': zone, 'correct': ok}}, 'contacts': []},
              events=[{'event': 'horn_heard', 'time': trig + 1.0, 'pattern': horn['pattern']}])
    print(f"referee fixtures in {HERE / 'referee'}")


if __name__ == '__main__':
    main()
