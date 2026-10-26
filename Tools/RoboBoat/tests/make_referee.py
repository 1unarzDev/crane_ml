#!/usr/bin/env python3
"""Writes tests/referee/<case>/: referee fixtures per scoring tier for the testing stream, in
session-directory shape (course_truth.json, track.jsonl, events.jsonl) plus expected.json,
the part of ``course_truth.py explain`` the case pins. Tracks are scripted waypoint paths at
1 m/s on the base course (fixtures/roboboat_2026-s1.json); a case that needs a different
course (another direction source, a toggling sign, an occupied slip) carries its own truth."""

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


def write(name, truth, track, expected, events=()):
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
    sem = task(t, 'speed_challenge')['semantics']
    right = 'ccw' if sem['source_color'] == 'red' else 'cw'
    wrong = 'cw' if right == 'ccw' else 'ccw'

    out = entry_out(t)
    write('entry_out_and_back', t, path(out + out[::-1]),
          {'tasks': {'entry_gates': {'all_out': True, 'all_in': True}}, 'contacts': []})
    g1 = gate(t, 'entry_gates.gate1')
    write('contact_strike', t, path([(0.0, 0.0), xy(el(t, g1['port'])), along(xy(g1), 0, 5)]),
          {'contacts': [g1['port']]})

    write('speed_marker_correct', t, path(speed_loop(t, right)), {'tasks': {'speed_challenge': {'direction': right, 'correct': True}}})
    write('speed_marker_wrong', t, path(speed_loop(t, wrong)), {'tasks': {'speed_challenge': {'direction': wrong, 'correct': False}}})
    sign_truth = copy.deepcopy(t)
    s = task(sign_truth, 'speed_challenge')['semantics']
    sign_color = 'green' if sem['source_color'] == 'red' else 'red'
    s.update(direction_source='sign', source_color=sign_color, direction='ccw' if sign_color == 'red' else 'cw')
    s['sign']['program'] = {'type': 'fixed', 'value': sign_color}
    next(x for x in sign_truth['states'] if x['id'] == s['sign']['id']).update(initial=sign_color, program=s['sign']['program'])
    sign_right = s['direction']
    write('speed_sign_correct', sign_truth, path(speed_loop(sign_truth, sign_right)),
          {'tasks': {'speed_challenge': {'source_color_at_entry': sign_color, 'correct': True}}})
    write('speed_sign_wrong', sign_truth, path(speed_loop(sign_truth, 'cw' if sign_right == 'ccw' else 'ccw')),
          {'tasks': {'speed_challenge': {'correct': False}}})
    toggle_truth = copy.deepcopy(sign_truth)
    s = task(toggle_truth, 'speed_challenge')['semantics']
    loop = path(speed_loop(toggle_truth, 'cw'))
    entry_t = 6.0  # the approach point is 6 m before the gate at 1 m/s
    program = {'type': 'toggle', 'values': ['red', 'green'], 'period_s': entry_t * 0.75, 'phase_s': 0.0}
    s['sign']['program'] = program
    next(x for x in toggle_truth['states'] if x['id'] == s['sign']['id']).update(initial='red', program=program)
    write('speed_sign_toggled_mid_approach', toggle_truth, loop,
          {'tasks': {'speed_challenge': {'source_color_at_entry': 'green', 'expected': 'cw', 'correct': True}}})

    deb = task(t, 'debris_clearance')['semantics']
    green, red = deb['rescue'][0], deb['avoid'][0]
    write('debris_green_full_circle', t, debris_circle(t, green, 380), {'tasks': {'debris_clearance': {'rescue': {green: '>=360'}}}})
    write('debris_green_partial', t, debris_circle(t, green, 250), {'tasks': {'debris_clearance': {'rescue': {green: '<360'}}}})
    rc = xy(el(t, red))
    write('debris_red_avoided', t, path([(rc[0] - 8, rc[1] + 3), (rc[0] + 8, rc[1] + 3)]),
          {'tasks': {'debris_clearance': {'avoid': {red: {'sweep_deg': '<180'}}}}})
    write('debris_red_circled', t, debris_circle(t, red, 380), {'tasks': {'debris_clearance': {'avoid': {red: {'sweep_deg': '>=360'}}}}})

    tiers = task(t, 'marina')['semantics']['tiers']
    lowest = tiers['lowest_green']
    other_green = next((b for b in tiers['green'] if b != lowest), None)
    red_free = next(b['id'] for b in t['bays'] if b['indicator']['color'] == 'red' and not b['occupied'])
    write('dock_lowest_green', t, dock(t, lowest), {'tasks': {'marina': {'entered': lowest, 'tier': 'lowest_green'}}})
    if other_green:
        write('dock_green_not_lowest', t, dock(t, other_green), {'tasks': {'marina': {'entered': other_green, 'tier': 'green'}}})
    write('dock_any_unoccupied', t, dock(t, red_free), {'tasks': {'marina': {'entered': red_free, 'tier': 'unoccupied'}}})
    occupied = copy.deepcopy(t)
    bay(occupied, red_free)['occupied'] = True
    write('dock_wrong_slip_occupied', occupied, dock(occupied, red_free), {'tasks': {'marina': {'tier': 'occupied'}}})

    harbor = task(t, 'harbor_alert')
    horn = harbor['semantics']['horn']
    trig = horn['program']['trigger_s']
    target, other = horn['target_zone'], ('zone2' if horn['target_zone'] == 'zone1' else 'zone1')
    start = xy(task(t, 'entry_gates')['anchors']['exit'])
    for name, zone, ok in (('horn_zone_correct', target, True), ('horn_zone_wrong', other, False)):
        z = harbor['anchors'][zone]
        idle = [(round(k * STEP, 3), *start) for k in range(int(trig / STEP))]
        write(name, t, idle + path([start, (z['x'], z['y'])], t0=trig),
              {'tasks': {'harbor_alert': {'reached': zone, 'correct': ok}}},
              events=[{'event': 'horn_heard', 'time': trig + 1.0, 'pattern': horn['pattern']}])
    print(f"referee fixtures in {HERE / 'referee'}")


if __name__ == '__main__':
    main()
