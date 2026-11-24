"""Example scorers and tasks for examples/studies/*.yaml. A scorer returns named parts (numbers); a task returns a
dict of numbers (and optionally 'outcome')."""

import math


def toy(result, cell):
    """Success, a time part (1 at t=0, 0 at the time limit) and their sum."""
    m = result['metrics']
    success = m.get('succeeded', 0.0)
    time = 1.0 - min(1.0, m.get('run.sim_s', 240.0) / 240.0)
    return {'success': success, 'time': time, 'total': success + time}


def boat(result, cell):
    """The boat mission: success, completion time and clearance kept."""
    m = result['metrics']
    success = m.get('succeeded', 0.0)
    time = 1.0 - min(1.0, m.get('run.sim_s', 300.0) / 300.0)
    clearance = min(1.0, max(0.0, m.get('run.min_clearance_m', 0.0)))
    return {'success': success, 'time': time, 'clearance': clearance, 'total': success + time + 0.5 * clearance}


def quadratic(args, seed, cell):
    """A task with no sim: a noisy quadratic bowl around (1, -0.5). Seeded, so arms pair on the seed."""
    import random
    r = random.Random(seed)
    x, y = float(args.get('x', 0.0)), float(args.get('y', 0.0))
    loss = (x - 1.0) ** 2 + (y + 0.5) ** 2 + 0.05 * r.gauss(0.0, 1.0) * float(args.get('noise', 1.0))
    return {'loss': loss, 'neg_loss': -loss, 'dist': math.hypot(x - 1.0, y + 0.5)}
