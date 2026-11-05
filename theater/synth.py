"""Synthetic fleets for tests and render spikes: ghosts that follow a world's centre line with seeded
lateral wander, a speed that grows with the iteration, and failures (a stop at a hit) that get rarer.

    python3 -m theater.synth --truth course_truth.json --tasks entry_gates,debris_clearance \
        --runs 500 --iterations 20 --seed 1 --out DIR

writes DIR/manifest.json, DIR/tracks.pack and DIR/course_truth.json (the world's input).
"""

import argparse
import json
import math
import random
from pathlib import Path

from . import pack
from .worlds import RoboBoatWorld

HZ = 10


def _arc(line):
    s = [0.0]
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        s.append(s[-1] + math.hypot(bx - ax, by - ay))
    return s


def _at(line, arc, d):
    d = max(0.0, min(arc[-1], d))
    for i in range(len(arc) - 1):
        if arc[i + 1] >= d:
            u = (d - arc[i]) / max(arc[i + 1] - arc[i], 1e-9)
            (ax, ay), (bx, by) = line[i], line[i + 1]
            return ax + u * (bx - ax), ay + u * (by - ay), math.atan2(by - ay, bx - ax)
    (ax, ay), (bx, by) = line[-2], line[-1]
    return bx, by, math.atan2(by - ay, bx - ax)


def _smooth(points, k=6):
    """Chaikin corner cutting (k rounds): the boat turns, it doesn't pivot on a vertex."""
    for _ in range(k):
        out = [points[0]]
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            out += [(0.75 * ax + 0.25 * bx, 0.75 * ay + 0.25 * by), (0.25 * ax + 0.75 * bx, 0.25 * ay + 0.75 * by)]
        out.append(points[-1])
        points = out
    return points


def run(world_line, iteration, n_iter, rng, run_id):
    """One synthetic run: samples at HZ and its metadata."""
    skill = (iteration + 1) / n_iter                        # 0..1 over the study
    line = _smooth(world_line)
    arc = _arc(line)
    L = arc[-1]
    v = 0.6 + 1.0 * skill + rng.uniform(-0.15, 0.15)        # m/s
    wander = (1.6 - 1.3 * skill) * rng.uniform(0.6, 1.2)    # m
    waves = [(rng.uniform(0.02, 0.12), rng.uniform(0, 2 * math.pi), rng.uniform(0.3, 1.0)) for _ in range(3)]
    p_fail = max(0.03, 0.6 * (1 - skill) ** 1.5)
    fail_at = rng.uniform(0.1, 0.95) * L if rng.random() < p_fail else None
    timeout = 300.0
    samples, events = [], []
    d, t, prev = 0.0, 0.0, None
    gate_s = [a for a in _arc(world_line)[1:]]
    next_gate = 0
    outcome, reason = 'finished', None
    while True:
        lat = wander * sum(a * math.sin(2 * math.pi * f * d / 10 + ph) for f, ph, a in waves)
        lat *= min(1.0, d / 3.0, (L - d) / 3.0) if L > 6 else 1.0
        x, y, h = _at(line, arc, d)
        x += -math.sin(h) * lat
        y += math.cos(h) * lat
        if prev is not None:
            yaw = math.degrees(math.atan2(y - prev[1], x - prev[0])) if (x, y) != prev[:2] else prev[2]
        else:
            yaw = math.degrees(h)
        roll = 4.0 * math.sin(t * 1.3 + run_id) * (v / 2)
        samples.append({'x': x, 'y': y, 'z': 0.0, 'yaw': pack.wrap_deg(yaw), 'pitch': 0.0, 'roll': roll,
                        'speed': v, 'progress': min(1.0, d / L)})
        prev = (x, y, yaw)
        while next_gate < len(gate_s) and d >= gate_s[next_gate] * arc[-1] / max(gate_s[-1], 1e-9):
            events.append({'t': round(t, 2), 'kind': 'gate', 'index': next_gate, 'x': round(x, 2), 'y': round(y, 2)})
            next_gate += 1
        if fail_at is not None and d >= fail_at:
            outcome, reason = 'failed', rng.choice(['buoy_hit', 'buoy_hit', 'stalled', 'fence'])
            events.append({'t': round(t, 2), 'kind': 'failure', 'reason': reason, 'x': round(x, 2), 'y': round(y, 2)})
            break
        if d >= L:
            events.append({'t': round(t, 2), 'kind': 'finish', 'x': round(x, 2), 'y': round(y, 2)})
            break
        if t >= timeout:
            outcome, reason = 'timeout', 'timeout'
            break
        t += 1.0 / HZ
        d += v / HZ
    progress = samples[-1]['progress']
    score = progress * (1.0 if outcome == 'finished' else 0.7) + (0.3 * max(0.0, 1 - t / 200) if outcome == 'finished' else 0)
    meta = {'id': f'synth-{run_id:05d}', 'iteration': iteration, 'outcome': outcome, 'failure_reason': reason,
            'score': round(score, 4), 'progress': round(progress, 4), 'valid': True, 'duration_s': round(t, 2),
            'params': {'FollowPath.desired_linear_vel': round(v, 3), 'synthetic.wander_m': round(wander, 3)},
            'events': events}
    return samples, meta


def fleet(world, runs, iterations, seed):
    line = world.centreline()
    rng = random.Random(seed)
    per = [runs // iterations + (1 if i < runs % iterations else 0) for i in range(iterations)]
    tracks, metas, k = [], [], 0
    for it, n in enumerate(per):
        for _ in range(n):
            s, m = run(line, it, iterations, random.Random(rng.getrandbits(64)), k)
            tracks.append(s)
            metas.append(m)
            k += 1
    insights = []
    for it in range(iterations):
        ms = [m for m in metas if m['iteration'] == it]
        best = max(ms, key=lambda m: m['score'])
        insights.append({'iteration': it, 'runs': len(ms), 'best': best['id'], 'best_score': best['score'],
                         'insight': f"synthetic: speed {best['params']['FollowPath.desired_linear_vel']} m/s, "
                                    f"wander {best['params']['synthetic.wander_m']} m"})
    return tracks, metas, insights


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--truth', required=True)
    ap.add_argument('--tasks', default='entry_gates,debris_clearance')
    ap.add_argument('--runs', type=int, default=500)
    ap.add_argument('--iterations', type=int, default=20)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    truth = json.loads(Path(a.truth).read_text())
    if a.tasks:
        truth = RoboBoatWorld.only_tasks(truth, a.tasks.split(','))
    (out / 'course_truth.json').write_text(json.dumps(truth, indent=1) + '\n')
    world = RoboBoatWorld(truth)
    tracks, metas, insights = fleet(world, a.runs, a.iterations, a.seed)
    sx, sy, _ = world.start()
    pack.write_pack(out / 'tracks.pack', tracks, HZ, origin=(round(sx), round(sy), 0.0))
    pack.write_manifest(out / 'manifest.json', world=world.manifest_world(), robot={'plugin': 'roboboat'},
                        runs=metas, study=f'synthetic-{a.seed}', iterations=insights,
                        water={'mode': 'still'}, extra={'synthetic': {'seed': a.seed, 'runs': a.runs, 'iterations': a.iterations}})
    print(f'{len(tracks)} runs, {sum(len(t) for t in tracks)} samples -> {out}')


if __name__ == '__main__':
    main()
