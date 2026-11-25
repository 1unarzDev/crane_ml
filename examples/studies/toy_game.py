"""A toy team game for examples/studies/teamgame_like.yaml: sub-millisecond units, so the arm is a batch task.

Each episode a team of ``world.team`` agents chases a drifting target. ``fixed`` never retunes, ``oracle`` knows the
drift, ``adaptive`` estimates it from noisy looks and closes most of the gap. The loss is the mean miss distance.
"""

import random


def play(args, seed, cell):
    """One episode: {loss, caught, events}."""
    w, policy = args.get('world', {}), args.get('policy', 'fixed')
    gain, noise, team = float(w.get('gain', 1.0)), float(w.get('noise', 0.1)), int(w.get('team', 4))
    r = random.Random(seed)
    drift = r.gauss(0.0, 1.0) * gain
    if policy == 'oracle':
        guess = drift
    elif policy == 'adaptive':
        looks = [drift + r.gauss(0.0, 0.3 + noise) for _ in range(team)]
        guess = 0.85 * sum(looks) / team
    else:
        guess = 0.0
    misses = [abs(drift - guess + r.gauss(0.0, noise)) for _ in range(team)]
    loss = sum(misses) / team
    return {'loss': loss, 'caught': float(loss < 0.5),
            'events': [{'t': i, 'agent': i, 'miss': round(m, 4)} for i, m in enumerate(misses)]}


def play_batch(args, seeds, cell):
    """``batch: true``: one call per seed chunk, one result per seed in order (the rows `play` would give)."""
    return [play(args, s, cell) for s in seeds]
