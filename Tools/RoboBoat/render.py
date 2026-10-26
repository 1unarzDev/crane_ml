#!/usr/bin/env python3
"""SVG of a course truth: boundary, task footprints, structures, objects coloured by label,
gates, zones and an optional track. ``render.py TRUTH.json [--track track.jsonl] > course.svg``."""

import argparse
import json
import math
import sys
from pathlib import Path

COLOURS = {'red': '#d62728', 'green': '#2ca02c', 'yellow': '#e6b800', 'orange': '#ff7f0e', 'black': '#222222',
           'banner': '#ffffff', 'boat': '#9e9e9e'}


def colour(label):
    for key, value in COLOURS.items():
        if key in (label or ''):
            return value
    return '#7f7f7f'


def svg(truth, track=()):
    pts = [tuple(p) for p in truth.get('boundary', [])] or [(0, 0), (1, 1)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    pad, scale = 2.0, 6.0
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad

    def tx(x, y):  # course x up the page, y (port) to the left
        return ((y1 - y) * scale, (x1 - x) * scale)

    def poly(points, **attrs):
        d = ' '.join(f'{a:.1f},{b:.1f}' for a, b in (tx(*p) for p in points))
        extra = ' '.join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
        return f'<polygon points="{d}" {extra}/>'

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{(y1 - y0) * scale:.0f}" height="{(x1 - x0) * scale:.0f}" '
           f'font-family="sans-serif" font-size="9">', '<rect width="100%" height="100%" fill="#eaf4fb"/>',
           poly(pts, fill='none', stroke='#4a90c2', stroke_width=2)]
    for t in truth.get('tasks', []):
        if len(t.get('footprint', [])) > 2:
            out.append(poly(t['footprint'], fill='none', stroke='#999', stroke_dasharray='4 3'))
            cx = sum(p[0] for p in t['footprint']) / len(t['footprint'])
            cy = sum(p[1] for p in t['footprint']) / len(t['footprint'])
            a, b = tx(cx, cy)
            out.append(f'<text x="{a:.1f}" y="{b:.1f}" fill="#555">{t["id"]}</text>')
        for name, z in (t.get('anchors') or {}).items():
            if 'r' in z:
                a, b = tx(z['x'], z['y'])
                out.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="{z["r"] * scale:.1f}" fill="#ffd70033" stroke="#e6b800"/>')
    byid = {e['id']: e for e in truth.get('elements', [])}
    for g in truth.get('gates', []):
        (a, b), (c, d) = tx(byid[g['port']]['x'], byid[g['port']]['y']), tx(byid[g['starboard']]['x'], byid[g['starboard']]['y'])
        out.append(f'<line x1="{a:.1f}" y1="{b:.1f}" x2="{c:.1f}" y2="{d:.1f}" stroke="#bbb" stroke-dasharray="2 2"/>')
    for e in truth.get('elements', []):
        if 'length' in e:
            f, l = (math.cos(e['yaw']), math.sin(e['yaw'])), (-math.sin(e['yaw']), math.cos(e['yaw']))
            corners = [(e['x'] + f[0] * u * e['length'] / 2 + l[0] * v * e['width'] / 2,
                        e['y'] + f[1] * u * e['length'] / 2 + l[1] * v * e['width'] / 2) for u, v in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
            out.append(poly(corners, fill='#8c8c8c'))
        else:
            a, b = tx(e['x'], e['y'])
            out.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="{max(e["radius"] * scale, 1.5):.1f}" fill="{colour(e.get("label"))}" '
                       f'stroke="#000" stroke-width="0.4"><title>{e["id"]} {e.get("label", "")}</title></circle>')
    if track:
        d = ' '.join(f'{a:.1f},{b:.1f}' for a, b in (tx(p[1], p[2]) for p in track))
        out.append(f'<polyline points="{d}" fill="none" stroke="#1f77b4" stroke-width="1.2"/>')
    out.append('</svg>')
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('truth')
    ap.add_argument('--track')
    args = ap.parse_args(argv)
    truth = json.loads(Path(args.truth).read_text())
    track = []
    if args.track:
        for line in Path(args.track).read_text().splitlines():
            r = json.loads(line)
            track.append((r['t'], r['x'], r['y']))
    sys.stdout.write(svg(truth, track) + '\n')


if __name__ == '__main__':
    main()
