"""One run: a domain controller on a scenario, on a fixed-step sim clock; its run directory.

    <scenario.filename>  the scenario file, byte for byte
    track.jsonl          {t, x, y, yaw} at track_hz, the scenario's world frame
    events.jsonl         the controller's events {t, event, ...} (goals included)
    states.jsonl         the world's state log {t_sim, id, value}
    run.json             outcome, flags, contacts, error, idle, identity, limits, config, provenance
    (the adapter's own files, from DomainAdapter.finalize)

No wall clock is read: equal run identities give byte-equal directories.
"""

import json
import math
from pathlib import Path
import traceback

from fastsim import controller as ctl, identity, perception as perc
from fastsim.plant import Limits, vehicle
from fastsim.world import World, contact_cause, footprint_clearance

SCHEMA = 'crane.fastsim.run/1'
UNHASHED = ('workers', 'max_workers', 'scenarios')  # change no result


def line(record):
    return json.dumps(record, separators=(', ', ': ')) + '\n'


def sightings(world, sensor, pose):
    """Tracks without a tracker: what is perceived now, confidence 1."""
    return [perc.Track(str(k + 1), world.labels[i], float(world.xy[i, 0]), float(world.xy[i, 1]),
                       1.0, True, int(i)) for k, i in enumerate(sensor.see(pose).tolist())]


def simulate(scenario, adapter, cfg, seed, out, base=None, provenance=None):
    """Run ``adapter``'s controller on ``scenario`` with ``seed``; write ``out``; return the summary."""
    out = Path(out)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f'{out}: a run directory is written once (provenance)')
    world = World(scenario)
    lim = adapter.limits() or Limits.from_dict(cfg['limits'])
    dt = float(cfg['dt'])
    steps_per_tick = max(1, round(1.0 / (float(cfg['tick_hz']) * dt)))
    track_every = max(1, round(1.0 / (float(cfg['track_hz']) * dt)))
    tracker, plant = vehicle(cfg['vehicle'], lim, base=base)
    clock = {'now': 0.0}
    nav = ctl.Navigator(lim, clock)
    sensor = perc.perception(cfg['perception'], world, identity.rng(seed, 'perception'), base)
    holdover = perc.HoldoverTracker(world, sensor, **cfg['tracker']) if cfg.get('tracker') else None
    ctx = ctl.RunContext(seed, scenario, world, nav, lim, identity.rng(seed, 'controller'), clock)
    nav.report = ctx.emit
    brain = adapter.controller(ctx)
    time_limit = float(adapter.time_limit(ctx) or cfg['time_limit_s'])
    limit = time_limit + float(cfg['grace_s'])
    track, contacts, touching, least, crash = [], [], False, math.inf, None
    idle = {'first_t': None, 's': 0.0}
    reach = max(math.hypot(*p) for p in lim.footprint)  # beyond this nothing can touch
    outcome, step = None, 0
    while outcome is None and step * dt <= limit:
        now = clock['now'] = round(step * dt, 9)
        if step % track_every == 0:
            x, y, yaw = world.to_world(plant.pose)
            track.append((now, round(x, 4), round(y, 4), round(yaw, 5)))
            clearance, i = footprint_clearance(world, plant.pose, lim.footprint, reach)
            least = min(least, clearance)
            if clearance < 0 and not touching:
                contacts.append({'t': now, 'id': world.ids[i], 'clearance_m': round(clearance, 3),
                                 'cause': contact_cause(world, i, nav.goal, reach)})
            touching = clearance < 0
        if step % steps_per_tick == 0:
            tracks = holdover.update(plant.pose, now) if holdover else sightings(world, sensor, plant.pose)
            try:
                outcome = brain.tick(ctl.Observation(now, plant.pose, tracks, sensor.fov()))
            except Exception as error:  # noqa: BLE001  the domain's logic raised: this run ends
                frame = traceback.extract_tb(error.__traceback__)[-1]
                crash = {'t': now, 'error': f'{type(error).__name__}: {error}',
                         'at': f'{Path(frame.filename).name}:{frame.lineno}'}
                outcome = 'ERROR'
        goal = nav.goal
        if goal is not None and not goal.done:
            nav.driven = now
        elif outcome is None and now - max(nav.driven or 0.0, nav.held or 0.0) > cfg['mux_idle_s']:
            idle['first_t'] = now if idle['first_t'] is None else idle['first_t']
            idle['s'] += dt
        moved = plant.step(tracker.command(goal, plant.pose, now, nav.cap), dt)
        nav.advance(plant.pose, moved, now)
        step += 1
    brain.close()
    flags = (['oracle_labels'] + (['state_program'] if scenario.changing_states() else [])
             + (['contact'] if contacts else [])
             + sorted({c['cause'] for c in contacts} & {'route_through_obstacle'})
             + (['sensor_at_origin'] if not (cfg['perception'] or {}).get('mount') else [])
             + (['logic_exception'] if crash else []) + (['idle_drift'] if idle['first_t'] is not None else []))
    hashed = {k: v for k, v in cfg.items() if k not in UNHASHED}
    ident = identity.run_identity(scenario.content_sha256, hashed, adapter.identity(), seed,
                                  identity.core_version())
    pair_key, truth_sha = scenario.pair_key
    summary = {'schema': SCHEMA, 'fidelity': cfg['fidelity'], 'outcome': outcome or 'TIMEOUT',
               'seed': int(seed), 'sim_s': round(step * dt, 3), 'time_limit_s': time_limit,
               'condition': {**adapter.condition(ctx), 'time_limit_s': time_limit},
               'flags': flags, 'contacts': contacts, 'error': crash,
               'idle': {'first_t': idle['first_t'], 's': round(idle['s'], 3)},
               'min_clearance_m': round(least, 3) if least < math.inf else None,
               'scenario': {'name': scenario.name, 'file': scenario.filename,
                            'pair_key': pair_key, 'truth_sha256': truth_sha},
               'identity': ident, 'adapter': adapter.identity(), 'limits': lim.record(),
               'plant': {'gain': tracker.gain, **plant.params()}, 'config': cfg,
               'provenance': provenance or {}}
    out.mkdir(parents=True, exist_ok=True)
    (out / scenario.filename).write_bytes(scenario.data)
    (out / 'track.jsonl').write_text(''.join(line({'t': t, 'x': x, 'y': y, 'yaw': a}) for t, x, y, a in track))
    (out / 'events.jsonl').write_text(''.join(line(e) for e in ctx.events))
    (out / 'states.jsonl').write_text(''.join(line(r) for r in scenario.state_log()))
    (out / 'run.json').write_text(json.dumps(summary, indent=2) + '\n')
    adapter.finalize(out, {**summary, 'track': track, 'events': ctx.events, 'context': ctx})
    return summary
