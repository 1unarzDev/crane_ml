#!/usr/bin/env python3
"""Audit a retained Nav2 costmap against a dock goal and straight final approach."""

import argparse
import base64
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import zlib


def decode_snapshot(snapshot):
    if snapshot.get('dataEncoding') != 'base64+zlib+uint8-row-major':
        raise ValueError(f"unsupported costmap encoding: {snapshot.get('dataEncoding')}")
    raw = zlib.decompress(base64.b64decode(snapshot['data']))
    expected = int(snapshot['sizeX']) * int(snapshot['sizeY'])
    if len(raw) != expected:
        raise ValueError(f'costmap has {len(raw)} cells, expected {expected}')
    digest = hashlib.sha256(raw).hexdigest()
    if digest != snapshot['dataSha256']:
        raise ValueError('costmap SHA-256 does not match retained data')
    return raw


def world_to_grid(snapshot, x, y):
    origin = snapshot['origin']
    dx = x - float(origin['x'])
    dy = y - float(origin['y'])
    yaw = float(origin['yaw'])
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return (math.floor(local_x / float(snapshot['resolution'])),
            math.floor(local_y / float(snapshot['resolution'])))


def grid_to_world(snapshot, cell_x, cell_y):
    resolution = float(snapshot['resolution'])
    local_x = (cell_x + 0.5) * resolution
    local_y = (cell_y + 0.5) * resolution
    yaw = float(snapshot['origin']['yaw'])
    return (float(snapshot['origin']['x']) +
            math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            float(snapshot['origin']['y']) +
            math.sin(yaw) * local_x + math.cos(yaw) * local_y)


def cell_cost(snapshot, data, x, y):
    cell_x, cell_y = world_to_grid(snapshot, x, y)
    size_x = int(snapshot['sizeX'])
    size_y = int(snapshot['sizeY'])
    if not 0 <= cell_x < size_x or not 0 <= cell_y < size_y:
        return None, (cell_x, cell_y)
    return int(data[cell_y * size_x + cell_x]), (cell_x, cell_y)


def obstacle_centers(snapshot, data, predicate):
    size_x = int(snapshot['sizeX'])
    return [grid_to_world(snapshot, index % size_x, index // size_x)
            for index, value in enumerate(data) if predicate(int(value))]


def point_clearance(x, y, centers, resolution):
    if not centers:
        return None
    # Treat each occupied grid cell as an area, not a dimensionless point.
    cell_radius = resolution * math.sqrt(2.0) / 2.0
    return max(0.0, min(math.hypot(x - ox, y - oy) for ox, oy in centers) - cell_radius)


def approach_samples(goal_x, goal_y, goal_yaw, length, spacing):
    count = max(1, math.ceil(length / spacing))
    for index in range(count + 1):
        distance = length * index / count
        yield {
            'distanceBeforeGoal': distance,
            'x': goal_x - math.cos(goal_yaw) * distance,
            'y': goal_y - math.sin(goal_yaw) * distance,
        }


def connected(snapshot, data, start, goal, blocked_cost=253):
    start_cell = world_to_grid(snapshot, *start)
    goal_cell = world_to_grid(snapshot, *goal)
    size_x = int(snapshot['sizeX'])
    size_y = int(snapshot['sizeY'])

    def passable(cell):
        x, y = cell
        return (0 <= x < size_x and 0 <= y < size_y and
                int(data[y * size_x + x]) < blocked_cost)

    if not passable(start_cell) or not passable(goal_cell):
        return False
    pending = deque([start_cell])
    visited = {start_cell}
    while pending:
        cell = pending.popleft()
        if cell == goal_cell:
            return True
        for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1),
                       (0, 1), (1, -1), (1, 0), (1, 1)):
            neighbor = (cell[0] + dx, cell[1] + dy)
            if neighbor not in visited and passable(neighbor):
                visited.add(neighbor)
                pending.append(neighbor)
    return False


def audit(summary, goal_x, goal_y, goal_yaw, approach_length,
          physical_half_beam, logical_radius, inflation_radius):
    snapshot = summary.get('latestCostmapSnapshot')
    if snapshot is None:
        raise ValueError('fixture summary has no latestCostmapSnapshot')
    data = decode_snapshot(snapshot)
    resolution = float(snapshot['resolution'])
    lethal = obstacle_centers(snapshot, data, lambda value: value == 254)
    blocked = obstacle_centers(snapshot, data, lambda value: value >= 253)
    samples = list(approach_samples(
        goal_x, goal_y, goal_yaw, approach_length, max(resolution / 2.0, 0.01)))
    for sample in samples:
        cost, cell = cell_cost(snapshot, data, sample['x'], sample['y'])
        sample['cell'] = {'x': cell[0], 'y': cell[1]}
        sample['cost'] = cost
        sample['lethalClearanceMeters'] = point_clearance(
            sample['x'], sample['y'], lethal, resolution)

    goal_cost, goal_cell = cell_cost(snapshot, data, goal_x, goal_y)
    final_pose = summary.get('actionResultPose') or summary.get('finalPose')
    final_xy = (float(final_pose['x']), float(final_pose['y'])) if final_pose else None
    final_cost, final_cell = (cell_cost(snapshot, data, *final_xy)
                              if final_xy else (None, (None, None)))
    clearances = [sample['lethalClearanceMeters'] for sample in samples
                  if sample['lethalClearanceMeters'] is not None]
    minimum_clearance = min(clearances) if clearances else None
    minimum_sample = (min(samples, key=lambda sample: sample['lethalClearanceMeters'])
                      if clearances else None)
    return {
        'schema': 'crane-nav2-costmap-clearance-audit-v1',
        'sourceSummary': summary.get('runId') or summary.get('scope'),
        'snapshot': {
            'frameId': snapshot['frameId'],
            'stamp': snapshot['stamp'],
            'resolution': resolution,
            'sizeX': int(snapshot['sizeX']),
            'sizeY': int(snapshot['sizeY']),
            'origin': snapshot['origin'],
            'dataSha256': snapshot['dataSha256'],
            'lethalCellCount': len(lethal),
            'blockedCellCountAtOrAbove253': len(blocked),
        },
        'goal': {
            'x': goal_x,
            'y': goal_y,
            'yaw': goal_yaw,
            'cell': {'x': goal_cell[0], 'y': goal_cell[1]},
            'cost': goal_cost,
            'blockedAt253': goal_cost is None or goal_cost >= 253,
        },
        'actionResult': {
            'pose': final_pose,
            'cell': {'x': final_cell[0], 'y': final_cell[1]},
            'cost': final_cost,
        },
        'approach': {
            'lengthMeters': approach_length,
            'sampleCount': len(samples),
            'minimumLethalClearanceMeters': minimum_clearance,
            'minimumClearanceSample': minimum_sample,
            'physicalHalfBeamMeters': physical_half_beam,
            'logicalRadiusMeters': logical_radius,
            'inflationRadiusMeters': inflation_radius,
            'clearsPhysicalHalfBeam': (
                minimum_clearance is not None and minimum_clearance >= physical_half_beam),
            'clearsLogicalRadius': (
                minimum_clearance is not None and minimum_clearance >= logical_radius),
            'clearsInflationRadius': (
                minimum_clearance is not None and minimum_clearance >= inflation_radius),
            'samples': samples,
        },
        'connectedBelowCost253': (
            connected(snapshot, data, final_xy, (goal_x, goal_y)) if final_xy else None),
        'interpretation': (
            'Clearance is measured from query points to the conservative edge of lethal costmap '
            'cells. The current cost field already includes the configured robot radius and '
            'inflation; connectedBelowCost253 tests the actual retained planner grid.'),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('summary', type=Path)
    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--goal-yaw', type=float, required=True)
    parser.add_argument('--approach-length', type=float, default=5.0)
    parser.add_argument('--physical-half-beam', type=float, default=0.4475)
    parser.add_argument('--logical-radius', type=float, default=0.80)
    parser.add_argument('--inflation-radius', type=float, default=1.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    with args.summary.open(encoding='utf-8') as stream:
        result = audit(json.load(stream), args.goal_x, args.goal_y, args.goal_yaw,
                       args.approach_length, args.physical_half_beam,
                       args.logical_radius, args.inflation_radius)
    rendered = json.dumps(result, indent=2, sort_keys=True) + '\n'
    if args.output:
        args.output.write_text(rendered, encoding='utf-8')
    print(rendered, end='')


if __name__ == '__main__':
    main()
