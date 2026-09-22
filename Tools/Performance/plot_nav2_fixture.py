#!/usr/bin/env python3
"""Render dependency-free SVG diagnostics from a Nav2 fixture summary."""

import argparse
import html
import json
import math
from pathlib import Path


WIDTH = 960
HEIGHT = 620
MARGIN = 70
COLORS = ('#2563eb', '#dc2626', '#16a34a', '#9333ea')


def wrapped_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def downsample(values, maximum=1800):
    if len(values) <= maximum:
        return values
    stride = math.ceil(len(values) / maximum)
    sampled = values[::stride]
    return sampled + ([values[-1]] if sampled[-1] is not values[-1] else [])


def bounds(series, padding=0.05):
    points = [point for values in series for point in values]
    if not points:
        return 0.0, 1.0, 0.0, 1.0
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    dx = max(x1 - x0, 1e-6)
    dy = max(y1 - y0, 1e-6)
    return x0 - dx * padding, x1 + dx * padding, y0 - dy * padding, y1 + dy * padding


def transform(point, extents, viewport=(MARGIN, 45, WIDTH - 2 * MARGIN, HEIGHT - 2 * MARGIN)):
    x0, x1, y0, y1 = extents
    left, top, width, height = viewport
    x = left + (point[0] - x0) / max(x1 - x0, 1e-9) * width
    y = top + height - (point[1] - y0) / max(y1 - y0, 1e-9) * height
    return x, y


def polyline(points, extents, color, viewport=None, width=2.0, dash=None):
    if not points:
        return ''
    mapped = [transform(point, extents, viewport or (MARGIN, 45, WIDTH - 2 * MARGIN,
                                                       HEIGHT - 2 * MARGIN))
              for point in points]
    coords = ' '.join(f'{x:.2f},{y:.2f}' for x, y in mapped)
    dashed = f' stroke-dasharray="{dash}"' if dash else ''
    return (f'<polyline points="{coords}" fill="none" stroke="{color}" '
            f'stroke-width="{width}"{dashed}/>' )


def svg_start(title):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" '
        f'viewBox="0 0 {WIDTH} {HEIGHT}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{WIDTH / 2}" y="27" text-anchor="middle" '
        f'font-family="sans-serif" font-size="18">{html.escape(title)}</text>',
    ]


def axes(extents, x_label, y_label, viewport=(MARGIN, 45, WIDTH - 2 * MARGIN,
                                              HEIGHT - 2 * MARGIN)):
    left, top, width, height = viewport
    x0, x1, y0, y1 = extents
    values = [
        f'<rect x="{left}" y="{top}" width="{width}" height="{height}" '
        'fill="none" stroke="#64748b"/>',
        f'<text x="{left + width / 2}" y="{top + height + 42}" text-anchor="middle" '
        f'font-family="sans-serif" font-size="13">{html.escape(x_label)}</text>',
        f'<text x="18" y="{top + height / 2}" text-anchor="middle" '
        f'transform="rotate(-90 18 {top + height / 2})" font-family="sans-serif" '
        f'font-size="13">{html.escape(y_label)}</text>',
    ]
    for index in range(6):
        fraction = index / 5
        x = left + fraction * width
        y = top + height - fraction * height
        xv = x0 + fraction * (x1 - x0)
        yv = y0 + fraction * (y1 - y0)
        values.extend([
            f'<line x1="{x}" y1="{top}" x2="{x}" y2="{top + height}" '
            'stroke="#e2e8f0"/>',
            f'<text x="{x}" y="{top + height + 19}" text-anchor="middle" '
            f'font-family="monospace" font-size="11">{xv:.2f}</text>',
            f'<line x1="{left}" y1="{y}" x2="{left + width}" y2="{y}" '
            'stroke="#e2e8f0"/>',
            f'<text x="{left - 8}" y="{y + 4}" text-anchor="end" '
            f'font-family="monospace" font-size="11">{yv:.2f}</text>',
        ])
    return values


def legend(labels, y=51):
    values = []
    start = WIDTH - MARGIN - 145 * len(labels)
    for index, label in enumerate(labels):
        x = start + index * 145
        values.extend([
            f'<line x1="{x}" y1="{y}" x2="{x + 25}" y2="{y}" '
            f'stroke="{COLORS[index]}" stroke-width="3"/>',
            f'<text x="{x + 31}" y="{y + 4}" font-family="sans-serif" '
            f'font-size="11">{html.escape(label)}</text>',
        ])
    return values


def write_xy_plot(path, title, series, labels, x_label, y_label, equal=False):
    extents = bounds(series)
    if equal:
        x0, x1, y0, y1 = extents
        span = max(x1 - x0, y1 - y0)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        extents = cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2
    svg = svg_start(title)
    svg.extend(axes(extents, x_label, y_label))
    for index, values in enumerate(series):
        svg.append(polyline(downsample(values), extents, COLORS[index],
                            width=2.4 if index else 2.0,
                            dash='7 5' if index == 0 and len(series) > 1 else None))
    svg.extend(legend(labels))
    svg.append('</svg>')
    path.write_text('\n'.join(svg) + '\n', encoding='utf-8')


def nearest_path_error(sample, planned):
    best = None
    for first, second in zip(planned, planned[1:]):
        dx, dy = second['x'] - first['x'], second['y'] - first['y']
        length2 = dx * dx + dy * dy
        fraction = 0.0 if length2 == 0 else max(0.0, min(
            1.0, ((sample['x'] - first['x']) * dx +
                  (sample['y'] - first['y']) * dy) / length2))
        px, py = first['x'] + fraction * dx, first['y'] + fraction * dy
        distance = math.hypot(sample['x'] - px, sample['y'] - py)
        heading = math.atan2(dy, dx) if length2 else first['yaw']
        candidate = distance, abs(wrapped_angle(sample['yaw'] - heading))
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best or (0.0, 0.0)


def write_two_panel(path, title, x_values, panels):
    svg = svg_start(title)
    panel_height = 220
    for panel_index, (label, series, labels) in enumerate(panels):
        top = 55 + panel_index * 270
        viewport = (MARGIN, top, WIDTH - 2 * MARGIN, panel_height)
        packed = [[(x, y) for x, y in zip(x_values, values)] for values in series]
        extents = bounds(packed, padding=0.03)
        svg.extend(axes(extents, 'wall time (s)', label, viewport))
        for index, values in enumerate(packed):
            svg.append(polyline(downsample(values), extents, COLORS[index], viewport))
        svg.extend(legend(labels, y=top + 12))
    svg.append('</svg>')
    path.write_text('\n'.join(svg) + '\n', encoding='utf-8')


def dock_polygon(goal_x, goal_y, goal_yaw, depth=3.0, width=2.0):
    forward = (math.cos(goal_yaw), math.sin(goal_yaw))
    left = (-math.sin(goal_yaw), math.cos(goal_yaw))
    return [
        (goal_x + forward[0] * a + left[0] * b,
         goal_y + forward[1] * a + left[1] * b)
        for a, b in ((-depth / 2, -width / 2), (depth / 2, -width / 2),
                     (depth / 2, width / 2), (-depth / 2, width / 2),
                     (-depth / 2, -width / 2))
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('summary')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    parser.add_argument('--goal-yaw', type=float)
    args = parser.parse_args()

    summary = json.loads(Path(args.summary).read_text(encoding='utf-8'))
    trajectory = summary.get('trajectory') or []
    planned = summary.get('plannedPath') or []
    if not trajectory:
        raise SystemExit('fixture summary contains no trajectory')
    if planned:
        goal_x, goal_y, goal_yaw = (planned[-1][field] for field in ('x', 'y', 'yaw'))
    elif None not in (args.goal_x, args.goal_y, args.goal_yaw):
        goal_x, goal_y, goal_yaw = args.goal_x, args.goal_y, args.goal_yaw
    else:
        raise SystemExit('goal is required when plannedPath is unavailable')

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    actual_xy = [(row['x'], row['y']) for row in trajectory]
    planned_xy = [(row['x'], row['y']) for row in planned]
    if planned:
        write_xy_plot(output / 'planned-vs-actual.svg', 'Planned vs actual trajectory',
                      [planned_xy, actual_xy], ['planned', 'actual'],
                      'ROS odom X (m)', 'ROS odom Y (m)', equal=True)
    else:
        write_xy_plot(output / 'actual-trajectory.svg', 'Actual NavigateToPose trajectory',
                      [actual_xy], ['actual'], 'ROS odom X (m)', 'ROS odom Y (m)',
                      equal=True)

    t0 = trajectory[0]['wallSeconds']
    times = [row['wallSeconds'] - t0 for row in trajectory]
    if planned:
        errors = [nearest_path_error(row, planned) for row in trajectory]
        write_two_panel(output / 'tracking-heading.svg', 'Tracking and heading error', times,
                        [('cross-track error (m)', [[row[0] for row in errors]], ['CTE']),
                         ('heading error (rad)', [[row[1] for row in errors]], ['heading'])])

    write_two_panel(output / 'commanded-vs-measured.svg',
                    'Commanded vs measured body motion', times,
                    [('surge / sway (m/s)',
                      [[row['commandSurge'] for row in trajectory],
                       [row['bodySurge'] for row in trajectory],
                       [row['commandSway'] for row in trajectory],
                       [row['bodySway'] for row in trajectory]],
                      ['cmd surge', 'body surge', 'cmd sway', 'body sway']),
                     ('yaw rate (rad/s)',
                      [[row['commandYaw'] for row in trajectory],
                       [row['bodyYawRate'] for row in trajectory]],
                      ['cmd yaw', 'body yaw'])])

    distance = [math.hypot(row['x'] - goal_x, row['y'] - goal_y)
                for row in trajectory]
    write_xy_plot(output / 'distance-to-goal.svg', 'Distance to dock goal',
                  [[(time, value) for time, value in zip(times, distance)]], ['distance'],
                  'wall time (s)', 'distance (m)')

    final_rows = [row for row, value in zip(trajectory, distance) if value <= 3.0]
    final_xy = [(row['x'], row['y']) for row in final_rows]
    tolerance = [(goal_x + 0.4 * math.cos(angle), goal_y + 0.4 * math.sin(angle))
                 for angle in [index * 2 * math.pi / 80 for index in range(81)]]
    region = dock_polygon(goal_x, goal_y, goal_yaw)
    write_xy_plot(output / 'final-approach.svg', 'Final 3 m approach and dock region',
                  [region, tolerance, final_xy], ['dock region', '0.40 m tolerance', 'actual'],
                  'ROS odom X (m)', 'ROS odom Y (m)', equal=True)

    manifest = {
        'schema': 'crane-nav2-fixture-plots-v1',
        'source': str(Path(args.summary)),
        'goal': {'x': goal_x, 'y': goal_y, 'yaw': goal_yaw},
        'plots': sorted(path.name for path in output.glob('*.svg')),
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n',
                                          encoding='utf-8')


if __name__ == '__main__':
    main()
