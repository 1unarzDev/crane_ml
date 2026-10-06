#!/usr/bin/env python3
"""Seeded task endpoints and tightly bounded cart placement; retain resolved manifest."""
import argparse, copy, json, pathlib, random

ROOT = pathlib.Path(__file__).resolve().parents[2]

def resolve(seed, scenario):
    manifest = json.loads((ROOT / 'Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json').read_text())
    trial = next(s for s in manifest['scenarios'] if s['id'] == scenario)
    rng = random.Random(seed)
    trial['seed'] = seed
    trial['start'] = [rng.uniform(-12.72, -12.58), rng.uniform(5.1, 5.8)]
    trial['goal'] = [rng.uniform(-12.48, -12.32), rng.uniform(22.2, 23.5)]
    trial['startYaw'] = rng.uniform(-4, 4)
    for obstacle in trial['obstacles']:
        if obstacle['id'] == 'annex-tote-cart':
            obstacle['center'][2] += rng.uniform(-.12, .12)
    return manifest

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--scenario', default='warehouse_compact_switchbacks', choices=['warehouse_compact_switchbacks', 'warehouse_compact_no_path'])
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.seed <= 0:
        parser.error('seed must be positive')
    manifest = resolve(args.seed, args.scenario)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + '\n')
    print(args.output.resolve())

if __name__ == '__main__':
    main()
