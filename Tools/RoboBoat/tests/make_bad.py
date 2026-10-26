#!/usr/bin/env python3
"""Writes tests/bad/*.json: hand-built bad layouts, each one mutation of the good base truth
(fixtures/roboboat_2026-s1.json) that the validator must reject. Each file carries
``expect_fail`` (rows that must fail) and ``exact`` (true when no other row may fail).
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


def move_along(t, gate_id, ident, metres):
    g = gate(t, gate_id)
    e = el(t, ident)
    e['x'] += math.cos(g['normal_yaw']) * metres
    e['y'] += math.sin(g['normal_yaw']) * metres


def set_cc(t, gate_id, cc):
    g = gate(t, gate_id)
    p, s = el(t, g['port']), el(t, g['starboard'])
    mx, my = (p['x'] + s['x']) / 2, (p['y'] + s['y']) / 2
    lx, ly = -math.sin(g['normal_yaw']), math.cos(g['normal_yaw'])
    p['x'], p['y'] = mx + lx * cc / 2, my + ly * cc / 2
    s['x'], s['y'] = mx - lx * cc / 2, my - ly * cc / 2


def gate_cc_3_3(t):
    set_cc(t, 'entry_gates.gate1', 3.3)


def gate_cc_narrow(t):
    set_cc(t, 'debris_clearance.gate2', 1.6)


def clear_below_beam(t):
    t['vessel']['beam'] = 1.6


def red_to_starboard(t):
    g = gate(t, 'entry_gates.gate1')
    p, s = el(t, g['port']), el(t, g['starboard'])
    (p['x'], p['y']), (s['x'], s['y']) = (s['x'], s['y']), (p['x'], p['y'])


def two_lowest_greens(t):
    # the other pier's slip with the lowest green number turns green too (it may hold a docked hull)
    low = min((b for b in t['bays'] if b['indicator']['color'] == 'green'), key=lambda b: b['sign']['number'])
    twin = next(b for b in t['bays'] if b['pier'] != low['pier'] and b['sign']['number'] == low['sign']['number'])
    twin['indicator']['color'] = 'green'


def green_occupied(t):
    bay(t, 'marina.S1')['occupied'] = True


def numbers_duplicate(t):
    # a red slip repeats another slip's number on its pier (the lowest green stays unique)
    red = next(b for b in t['bays'] if b['indicator']['color'] == 'red')
    other = next(b for b in t['bays'] if b['pier'] == red['pier'] and b is not red and b['indicator']['color'] == 'red' or
                 b['pier'] == red['pier'] and b is not red and b['sign']['number'] != 1)
    red['sign']['number'] = other['sign']['number']


def marker_beyond_mark(t):
    mark = el(t, 'speed_challenge.mark')
    marker = el(t, 'speed_challenge.marker')
    g = gate(t, 'speed_challenge.gate')
    marker['x'] = mark['x'] + math.cos(g['normal_yaw']) * 4
    marker['y'] = mark['y'] + math.sin(g['normal_yaw']) * 4


def leg_too_short(t):
    mark = el(t, 'speed_challenge.mark')
    g = gate(t, 'speed_challenge.gate')
    along = (mark['x'] - g['x']) * math.cos(g['normal_yaw']) + (mark['y'] - g['y']) * math.sin(g['normal_yaw'])
    move_along(t, 'speed_challenge.gate', 'speed_challenge.mark', 11.0 - along)


def direction_wrong(t):
    s = task(t, 'speed_challenge')['semantics']
    s['direction'] = 'cw' if s['direction'] == 'ccw' else 'ccw'


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


def gate2_yaw_20(t):
    g = gate(t, 'entry_gates.gate2')
    g['normal_yaw'] += math.radians(20)
    set_cc(t, 'entry_gates.gate2', math.dist(*[(el(t, g[k])['x'], el(t, g[k])['y']) for k in ('port', 'starboard')]))


def separation_short(t):
    g1, g2 = gate(t, 'entry_gates.gate1'), gate(t, 'entry_gates.gate2')
    dx, dy = g1['x'] + 5 - g2['x'], g1['y'] - g2['y']
    for k in ('port', 'starboard'):
        el(t, g2[k])['x'] += dx
        el(t, g2[k])['y'] += dy
    g2['x'], g2['y'] = g2['x'] + dx, g2['y'] + dy


def horn_1200(t):
    task(t, 'harbor_alert')['semantics']['horn']['freq_hz'] = 1200.0


def zone1_far(t):
    task(t, 'harbor_alert')['anchors']['zone1']['x'] += 5.0


def outside_boundary(t):
    el(t, 'supply_drop.ball1')['y'] = 80.0


def debris_heading_25(t):
    g = gate(t, 'debris_clearance.gate1')
    cc = math.dist(*[(el(t, g[k])['x'], el(t, g[k])['y']) for k in ('port', 'starboard')])
    g['normal_yaw'] += math.radians(25)
    set_cc(t, 'debris_clearance.gate1', cc)


def marina_yaw_100(t):
    task(t, 'marina')['params']['yaw_deg'] = 100.0


def marina_unrooted(t):
    for e in t['elements']:
        if e['task'] == 'marina':
            e['x'] += 6.0


def slip_hull(t, n=0):
    return [e for e in t['elements'] if e['task'] == 'supply_drop' and e['role'] == 'slip_hull'][n]


def hull_in_green_slip(t):
    b = next(b for b in t['bays'] if b['indicator']['color'] == 'green')
    slip_hull(t).update(x=b['x'], y=b['y'])


def hull_beside_moored(t):
    b = next(b for b in t['bays'] if b['occupied'])
    slip_hull(t).update(x=b['x'], y=b['y'])


def none_docked(t):
    free = next(e for e in t['elements'] if e['task'] == 'supply_drop' and e['role'] == 'hull')
    for i, e in enumerate([e for e in t['elements'] if e['role'] == 'slip_hull']):
        e.update(x=free['x'] + 4.0 * (i + 1), y=free['y'], role='hull')


def no_free_red(t):
    for b in t['bays']:
        if b['indicator']['color'] == 'red':
            b['occupied'] = True


CASES = {
    # name: (mutation, rows that must fail, exact)
    'gate_cc_3_3': (gate_cc_3_3, ['T1.cc'], True),
    'gate_cc_narrow': (gate_cc_narrow, ['T2.cc'], False),
    'clear_below_beam': (clear_below_beam, ['T1.clear', 'T5.slip_vessel'], False),
    'red_to_starboard': (red_to_starboard, ['T1.side'], True),
    'two_lowest_greens': (two_lowest_greens, ['T5.lowest_green'], False),
    'green_occupied': (green_occupied, ['T5.green_free', 'T5.occupied_red'], True),
    'numbers_duplicate': (numbers_duplicate, ['T5.numbers'], True),
    'marker_beyond_mark': (marker_beyond_mark, ['T3.marker_between'], False),
    'leg_too_short': (leg_too_short, ['T3.leg'], False),
    'direction_wrong': (direction_wrong, ['T3.direction_red', 'T3.direction_green'], False),
    'footprints_overlap': (footprints_overlap, ['G4.no_overlap'], True),
    'channel_blocked': (channel_blocked, ['P1.channel'], False),
    'no_room_at_green': (no_room_at_green, ['P3.rescue_circle'], False),
    'gate2_yaw_20': (gate2_yaw_20, ['T1.gate2_yaw'], False),
    'separation_short': (separation_short, ['T1.separation'], False),
    'horn_1200': (horn_1200, ['T6.freq'], True),
    'zone1_far': (zone1_far, ['T6.zone1'], True),
    'outside_boundary': (outside_boundary, ['G1.inside'], False),
    'debris_heading_25': (debris_heading_25, ['T2.heading'], False),
    'marina_yaw_100': (marina_yaw_100, ['T5.yaw'], True),
    'marina_unrooted': (marina_unrooted, ['T5.shore'], False),
    'hull_in_green_slip': (hull_in_green_slip, ['T4.dock_green_free'], False),
    'hull_beside_moored': (hull_beside_moored, ['T4.dock_moored_free'], False),
    'none_docked': (none_docked, ['T4.docked'], False),
    'no_free_red': (no_free_red, ['T5.free_red'], False),
}

if __name__ == '__main__':
    out = HERE / 'bad'
    out.mkdir(exist_ok=True)
    for name, (mutate, rows, exact) in CASES.items():
        t = copy.deepcopy(BASE)
        mutate(t)
        # One of direction_red/green applies; the test requires at least one listed row to fail
        # when ``any_of`` is set.
        t['expect'] = {'fail': rows, 'exact': exact, 'any_of': name == 'direction_wrong'}
        (out / f'{name}.json').write_text(json.dumps(t, indent=1) + '\n')
    print(f'{len(CASES)} bad layouts in {out}')
