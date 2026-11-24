"""Decoupled mode: any fastsim domain's scalar Python controller on the batched plant and tracker.

    setup = Setup(cfg, base, adapter, seeds, mode='decoupled')
    runs = run(setup, out=DIR)          # one fastsim-style run directory per env (adapter.finalize included)

The plant, tracker, progress checker, track, contacts and perception run batched (engine/episode.py,
numpy float64). At tick_hz each running env's own ``Controller`` (``adapter.controller(ctx)``) gets
the same ``Observation`` the scalar core builds, and acts through a ``Navigator`` whose goals live in
the batch's route buffers. The decision stays scalar, so its cost bounds the speedup.
"""

import json
import math
from pathlib import Path
import traceback

import numpy as np

from engine import geometry as GEO, tracker as TR
from engine.episode import OUT_TIMEOUT, consts, init_state, step_body
from engine.ops import numpy_ops
from fastsim import controller as ctl, identity, perception as perc
from fastsim.run import CRANE_RUN, SCHEMA, UNHASHED, crane_run_meta, line, write_traj

OUTCOME_CODE = {'SUCCEEDED': 1, 'FAILED': 2}
ERROR = 4


class Goal:
    """A goal as the controller holds it, read from the batch (frozen once a newer goal replaces it)."""

    def __init__(self, nav, precise, why, reverse):
        self.nav, self.precise, self.why, self.reverse = nav, precise, why, reverse
        self.frozen = None

    def _G(self):
        return self.nav.S['G']

    @property
    def done(self):
        if self.frozen:
            return True
        return not bool(self._G()['live'][self.nav.e])

    @property
    def status(self):
        if self.frozen:
            return self.frozen
        G, e = self._G(), self.nav.e
        return None if G['live'][e] else TR.STATUS[int(G['status'][e])]

    @property
    def route(self):
        G, e = self._G(), self.nav.e
        return [tuple(float(v) for v in p) for p in G['route'][e, G['head'][e]:G['rlen'][e]]]

    def end(self, status):
        G, e = self._G(), self.nav.e
        if not self.frozen and G['live'][e]:
            G['live'][e], G['status'][e] = False, {'SUCCEEDED': 1, 'ABORTED': 2, 'CANCELED': 3}[status]

    def cancel(self):
        self.end('CANCELED')


class Navigator(ctl.Navigator):
    """fastsim's Navigator with its goal in row ``e`` of the batch."""

    def __init__(self, limits, clock, e, holder):
        super().__init__(limits, clock)
        self.e, self.holder = e, holder

    @property
    def S(self):
        return self.holder['S']

    def send(self, route, precise=False, quiet=False, why='', reverse=None):
        if reverse is not None:
            raise NotImplementedError('engine decoupled mode: reverse goals are not batched')
        if route and not quiet:
            x, y, yaw = route[-1]
            self.last = {'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2), 'why': why}
            if self.report is not None:
                self.report('goal', **self.last)
        if self.goal is not None:
            self.goal.end('ABORTED')
            self.goal.frozen = self.goal.status or 'ABORTED'
        G, e = self.S['G'], self.e
        if len(route) > G['route'].shape[1]:
            grow = np.zeros((G['route'].shape[0], len(route) - G['route'].shape[1], 3))
            G['route'] = np.concatenate([G['route'], grow], axis=1)
        G['route'][e, :len(route)] = [tuple(p) for p in route]
        G['rlen'][e], G['head'][e], G['precise'][e] = max(1, len(route)), 0, bool(precise)
        G['live'][e], G['exists'][e], G['status'][e], G['anchored'][e] = bool(route), True, 0, False
        self.goal = Goal(self, precise, why, reverse)
        return self.goal

    def speed_limit(self, speed):
        super().speed_limit(speed)
        self.S['G']['cap'][self.e] = self.cap if self.cap is not None else np.inf

    def keep_still(self, quiet_s=0.3):
        now = self.clock['now']
        S = self.S
        if S['driven_set'][self.e] and not now - S['driven'][self.e] > quiet_s:
            return
        S['held'][self.e] = now

    def advance(self, pose, moved, now):  # the batch runs the progress checker
        pass


def run(setup, out=None, provenance=None):
    """Run every env; returns a list of (summary, run_dir) (run_dir None without ``out``)."""
    ops = numpy_ops()
    C = consts(setup)
    S = init_state(setup, record=1)
    E, cfg, W = setup.E, setup.cfg, setup.W
    S['G']['cap'] = np.full(E, np.inf)
    holder = {'S': S}
    clocks, ctxs, brains, sensors, crash = [], [], [], [], [None] * E
    for e in range(E):
        clock = {'now': 0.0}
        nav = Navigator(setup.lim, clock, e, holder)
        sensor = perc.perception(cfg['perception'], setup.worlds[e], identity.rng(setup.seeds[e], 'perception'),
                                 setup.base)
        ctx = ctl.RunContext(setup.seeds[e], setup.scenarios[e], setup.worlds[e], nav, setup.lim,
                             identity.rng(setup.seeds[e], 'controller'), clock)
        nav.report = ctx.emit
        clocks.append(clock)
        ctxs.append(ctx)
        sensors.append(sensor)
        brains.append(setup.adapter.controller(ctx))
        ctx.brain = brains[-1]
    allidx = np.repeat(np.arange(W.xy.shape[1])[None], E, axis=0)

    def decide(S, pose, act, t):
        holder['S'] = S
        vis = GEO.visibility(np, C['W'], pose, setup.cam, allidx, True)
        for e in np.flatnonzero(act):
            now = float(setup.now[t[e]])
            clocks[e]['now'] = now
            idx = np.flatnonzero(vis[e, :int(W.count[e])])
            w = setup.worlds[e]
            tracks = [perc.Track(str(k + 1), w.labels[i], float(w.xy[i, 0]), float(w.xy[i, 1]), 1.0, True, int(i))
                      for k, i in enumerate(idx.tolist())]
            p = (float(pose[0][e]), float(pose[1][e]), float(pose[2][e]))
            try:
                outcome = brains[e].tick(ctl.Observation(now, p, tracks, sensors[e].fov()))
            except Exception as error:  # noqa: BLE001  the domain's logic raised: this run ends
                frame = traceback.extract_tb(error.__traceback__)[-1]
                crash[e] = {'t': now, 'error': f'{type(error).__name__}: {error}',
                            'at': f'{Path(frame.filename).name}:{frame.lineno}'}
                outcome = 'ERROR'
            if outcome is not None:
                S['M']['outcome'][e] = OUTCOME_CODE.get(outcome, ERROR)
                S.setdefault('named', {})[e] = outcome
        return S

    step = 0
    with np.errstate(divide='ignore', invalid='ignore'):
        while (S['M']['outcome'] == 0).any():
            S = step_body(ops, setup, C, S, 1, step % setup.track_every == 0, step % setup.steps_per_tick == 0, decide)
            holder['S'] = S
            step += 1
    for b in brains:
        b.close()
    results = []
    for e in range(E):
        results.append(_finish(setup, S, e, ctxs[e], crash[e], (S.get('named') or {}).get(e), out, provenance))
    return results


def _finish(setup, S, e, ctx, crash, named, out, provenance):
    """fastsim.run.simulate's summary and run directory of env e."""
    cfg, dt = setup.cfg, setup.dt
    code = int(S['M']['outcome'][e])
    outcome = named or ('TIMEOUT' if code == OUT_TIMEOUT else None)
    every = setup.track_every
    rows = int(S['end'][e] - 1) // every + 1
    track = [(float(setup.now[j * every]), *(float(v) for v in r)) for j, r in enumerate(S['rec'][e, :rows])]
    ids = setup.W.ids[e]
    n = min(int(S['kcount'][e]), S['contacts'].shape[1])
    contacts = [{'t': float(setup.now[s]), 'id': ids[k], 'clearance_m': round(float(v), 3), 'cause': GEO.CAUSES[c]}
                for (s, k, c), v in zip(S['contacts'][e, :n].tolist(), S['cval'][e, :n].tolist())]
    first = int(S['idle_first'][e])
    idle_s = 0.0
    for _ in range(int(S['idle_n'][e])):
        idle_s += dt
    least = float(S['least'][e])
    scenario, adapter = setup.scenarios[e], setup.adapter
    flags = (['oracle_labels'] + (['state_program'] if scenario.changing_states() else [])
             + (['contact'] if contacts else []) + sorted({c['cause'] for c in contacts} & {'route_through_obstacle'})
             + (['sensor_at_origin'] if not (cfg['perception'] or {}).get('mount') else [])
             + (['logic_exception'] if crash else []) + (['idle_drift'] if first >= 0 else []))
    hashed = {k: v for k, v in cfg.items() if k not in UNHASHED}
    hashed['engine'] = setup.identity('numpy', 'float64')
    ident = identity.run_identity(scenario.content_sha256, hashed, adapter.identity(), setup.seeds[e],
                                  identity.core_version())
    pair_key, truth_sha = scenario.pair_key
    time_limit = setup.time_limit[e]
    summary = {'schema': SCHEMA, 'fidelity': cfg['fidelity'], 'outcome': outcome or 'TIMEOUT',
               'seed': int(setup.seeds[e]), 'sim_s': round(int(S['end'][e]) * dt, 3), 'time_limit_s': time_limit,
               'condition': {**adapter.condition(ctx), 'time_limit_s': time_limit},
               'flags': flags, 'contacts': contacts, 'error': crash,
               'idle': {'first_t': float(setup.now[first]) if first >= 0 else None, 's': round(idle_s, 3)},
               'min_clearance_m': round(least, 3) if least < math.inf else None,
               'scenario': {'name': scenario.name, 'file': scenario.filename, 'pair_key': pair_key,
                            'truth_sha256': truth_sha},
               'identity': ident, 'adapter': adapter.identity(), 'limits': setup.lim.record(),
               'plant': {'gain': setup.vehicle.gain, 'engine': setup.vehicle.ident}, 'config': cfg,
               'provenance': provenance or {}}
    run_dir = None
    if out is not None:
        run_dir = Path(out) / f's{setup.seeds[e]}'
        run_dir.mkdir(parents=True, exist_ok=False)
        (run_dir / scenario.filename).write_bytes(scenario.data)
        (run_dir / 'track.jsonl').write_text(''.join(line({'t': t, 'x': x, 'y': y, 'yaw': a}) for t, x, y, a in track))
        (run_dir / 'sim_events.jsonl').write_text(''.join(line(ev) for ev in ctx.events))
        (run_dir / 'states.jsonl').write_text(''.join(line(r) for r in scenario.state_log()))
        (run_dir / 'run.json').write_text(json.dumps(summary, indent=2) + '\n')
        write_traj(run_dir / 'run.ctraj', summary, track, ctx.events, float(cfg['track_hz']), adapter)
        (run_dir / CRANE_RUN).write_text(json.dumps(crane_run_meta(summary), indent=2) + '\n')
        if hasattr(adapter, 'current'):  # an adapter that keeps its run's controller (the boat's seen buoys)
            adapter.current = ctx.brain
        adapter.finalize(run_dir, {**summary, 'track': track, 'events': ctx.events, 'context': ctx})
    return summary, run_dir

