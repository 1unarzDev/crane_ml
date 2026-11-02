# Moving mhseals_testing's fast tier onto the crane fast-sim core

The core (`crane_ml/fastsim/`) now holds what `mhseals_testing/fastsim/` (branches
`fastsim-p1` and `fastsim-fidelity`) did generically: the world geometry, the camera cone, the
hold-over tracker, the goal tracker and plant, the run loop, the batch runner, the pair archive
and the fidelity loop. mhseals_testing keeps only a RoboBoat adapter. Nothing below is done
yet: the adapter is the owner's mhseals_testing change to make.

## 1. What stays in mhseals_testing

One module, `mhseals_testing/fastsim/adapter.py`, plus one YAML, `config/fastsim_core.yaml`.
`world.limits()` and `camera_mount()` (reading mhseals_nav), `McapSegment`/`ingest` (bag
reading), the harness writers and the referee stay where they are. `run.py`, `batch.py`,
`pairs.py` and the generic halves of `world.py` and `fidelity.py` are deleted once the adapter's
byte-identity test passes.

## 2. The adapter (`fastsim.controller.DomainAdapter`)

```python
from fastsim.controller import Controller, DomainAdapter
from fastsim.plant import Limits

class RoboBoatAdapter(DomainAdapter):
    def __init__(self, base=None, task='mission', mission=None, profile=None,
                 limits_overlay=None, camera_frame='front_camera_link', unknown_static=None):
        super().__init__(base, task=task, mission=mission, profile=profile, ...)
        self.params = load_params(task, mission, profile)            # mhseals_nav.tasks.params
        self.boat = world.limits(limits_overlay, camera_frame)       # the existing reader

    def limits(self):                     # Limits field for field; boat-only numbers in extra
        b = self.boat
        return Limits(b.vmax, b.vmin, b.accel, b.decel, b.goal_tol, b.precise_tol, b.rotate_above,
                      b.rotate_wz, b.progress_radius, b.progress_s, b.pass_radius, b.footprint,
                      extra={'controller': b.controller, 'sources': b.sources})

    def overrides(self):                  # read from the boat's files, so hashed with the run
        b = self.boat
        return {'perception': {'name': 'cone', 'hfov_deg': math.degrees(b.hfov), 'range_m': b.range_m,
                               'occlusion': True, 'mount': list(b.camera_mount) if b.camera_mount else None},
                'tracker': {**b.tracker, 'promote': self.options['unknown_static']}}
        # b.tracker carries class_groups, timeout, tau, miss_in_fov_s, as HoldoverTracker takes them

    def identity(self):                   # what used to be provenance.config
        return {'mhseals_nav': git_state(mhseals_nav.__file__), 'params_sha256': digest(self.params),
                'limits_sources': self.boat.sources, 'task': ..., 'mission': ..., 'profile': ...}

    def time_limit(self, ctx):  return float(self.params['timeout'])
    def condition(self, ctx):   return {'task': self.options['task'], 'profile': self.params.get('profile')}
    def controller(self, ctx):  return MissionController(ctx, self)
    def finalize(self, run_dir, result): ...   # section 3
    def metrics(self, run_dir): ...            # the referee's numbers, as pairs._numbers read them
```

`MissionController.__init__` clears the py_trees blackboard, adds the one boat-only navigator
call to the core navigator and builds the host exactly as `run.py` did:

```python
nav = ctx.navigator
nav.controller_vx_max = lambda: _vx_max(ctx.limits.extra['controller'])  # FollowPath.vx_max check
self.events, self.statuses = [], []
received = lambda rows: (lambda p: rows.append({**p, 'received': ctx.clock['now']}))
self.host = MissionHost(task, params, nav, received(self.events), lambda: ctx.clock['now'],
                        status_sink=received(self.statuses))     # sets nav.report, as before
```

and `tick` converts tracks and returns the host's outcome:

```python
def tick(self, obs):
    tracks = [Track(t.label, t.x, t.y, t.confidence, None, t.static, False, t.id) for t in obs.tracks]
    return self.host.tick(obs.pose, tracks, obs.fov)
```

The core already does everything else the old loop did: map frame from the start pose, contacts
and their causes at `track_hz`, idle time against `mux_idle_s`, an exception in the host ending
the run as `ERROR` with `error: {t, error, at}`, and `keep_still`, `hold`, `speed_limit`,
`send(route, precise, quiet, why, reverse)` and the progress checker with the same semantics.

## 3. `finalize`: the harness session layout, unchanged for the referee

The core writes `course_truth.json` (byte for byte, crane_truth format), `track.jsonl` (the same
`{t, x, y, yaw}` rows), `states.jsonl` (the same rows), `sim_events.jsonl` and `run.json`.
`finalize(run_dir, result)` writes the rest from `result` (run.json's summary plus `track`,
`events` and `context`), moving the tail of the old `run.run()` across unchanged:

1. `run_id = runs.next_run_id(run_dir / 'runs', PROCEDURE['procedure'])`, then
   `keep(run_dir, task, result['track'], ctrl.events, ctrl.statuses, run_id=run_id)`. `keep`
   rewrites `track.jsonl` with identical content. Reach the controller through
   `result['context']`, or keep it on the adapter.
2. The journal (`Journal`, `HornJournal`, `crane.horn_blasts`) into `events.jsonl`, then
   `runs.write_run(...)` with `outcome=result['outcome']` mapped as in step 4.
3. `session.json` (`mhseals.session/1`, kind/source `kinematic`, `course.identity` from
   `ctx.scenario.identity`, `git` and `config` from `result['provenance']` and
   `result['identity']`).
4. `fastsim.json` (`mhseals.fastsim/1`) built from `result`, with three renames that keep old
   readers working: outcome `TIMEOUT` becomes `RUNNING`, `error` becomes `logic_exception`, and
   flag `sensor_at_origin` becomes `camera_at_origin`. Also put `config.calibration` in it.

## 4. The domain YAML (`mhseals_testing/config/fastsim_core.yaml`)

```yaml
adapter: {ref: 'mhseals_testing.fastsim.adapter:RoboBoatAdapter',
          options: {task: mission, unknown_static: {within_m: 8.0, after_s: 2.0}}}
scenarios:
  dir: $FASTSIM_TRUTHS
  pattern: '{family}-s{seed}.json'
  fields: {family: roboboat_2026, crane: $CRANE_DIR, mode: iid}
  generate: [dotnet, '{crane}/Tools/RoboBoat/cli/bin/Release/net10.0/coursegen.dll', --family, '{family}',
             --mode, '{mode}', --seeds, '{seeds_from}:{seeds_to}', --out, '{dir}']
  format: {name: crane_truth, identity_check: '$CRANE_DIR/Tools/RoboBoat/course_truth.py:check_identity'}
vehicle: {model: kinematic, gain: 1.0}
calibration: fastsim_calibration/unity-e1.yaml     # its plant block {gain, delay_s, speed_scale} applies as is
dt: 0.05
tick_hz: 10.0      # the old tick_hz: null meant the mission's tick_hz; put the mission's value here
track_hz: 10.0
grace_s: 5.0
mux_idle_s: 0.5
workers: 4
max_workers: 8
```

`$VARS` in `dir`, `fields` values used by `generate`, and file references are expanded. Set
`CRANE_DIR` to a crane checkout on origin/main (never `~/crane_ml`).

## 5. The commands

| was | now |
|---|---|
| `boat_trial fastsim --crane C --truths D --seeds 1:21 --out O [--variant V] [--calibration F] [--generate] [--allow-dirty]` | `CRANE_DIR=C FASTSIM_TRUTHS=D python3 -m fastsim.batch --config config/fastsim_core.yaml --seeds 1:21 --out O [same flags]`. `boat_trial fastsim` can stay as a wrapper that maps `--profile/--mission/--limits` into `adapter.options` (e.g. `fastsim.config.load(path, overrides)` then `fastsim.batch.run_batch`). |
| `python -m mhseals_testing.fastsim.pairs wait FAST...` | `python3 -m fastsim.pairs wait FAST...` |
| `python -m mhseals_testing.fastsim.pairs pair UNITY...` | `python3 -m fastsim.pairs pair UNITY... --describe mhseals_testing.fastsim.adapter:describe_session --metrics mhseals_testing.fastsim.adapter:referee_numbers` |
| cluster job | backend `fastsim`, job `{config: config/fastsim_core.yaml, workdir: <mhseals_testing checkout>, seeds, out, variant, overrides}` |

`describe_session(dir)` returns `{pair_key, truth_sha256, seed, condition, source, provenance}`
for a Unity session. `pair_key`, `truth_sha256` and `seed` come from `course_truth.json`'s
`generator.identity`. `condition` is `{task, profile, time_limit_s: float(...)}` and must equal
a fast run's `condition` key for key. `referee_numbers(dir)` is the old `_numbers` over
`analysis/score.*.json`, so `diff.json` keys stay `run.duration_s`, `total.*` and so on, with
`fast_minus_twin` in place of `fast_minus_unity`. The archive root is now
`CRANE_FASTSIM_PAIRS` (default `~/.cache/crane-fastsim-pairs`). Leave the old
`~/.cache/fastsim-pairs` read-only: its rows use the `unity` key, and the core never rewrites an
archive.

## 6. The fidelity loop

`fidelity.py` in mhseals_testing keeps only the bag side and calls the core:

```python
from fastsim import fidelity as core
batch = [{'session': s.name[:15], 'cuts': cuts,
          'segments': core.windows(track, goals, stops)} ...]   # goals: [{t: log_t, x, y, yaw, precise, task}]
core.keep(store, basis_batches)                                # --basis sessions, never scored
row = core.score_batch(epoch, store, batch, adapter.limits(),
                       uncalibrated={'model': 'kinematic', 'gain': 1.0, 'delay_s': [0, 0, 0], 'speed_scale': 1.0},
                       bounds={'gain': [0.05, 2.0], 'delay_s.0': [0.0, 2.5], 'delay_s.2': [0.0, 1.5],
                               'speed_scale': [0.4, 1.2]} if refit else None,
                       ratios=core.pair_ratios(pairs_root, set(names), epoch_sha,
                                               session_of=lambda r: Path(r['twin_from']).name[:15],
                                               match={s: (lambda r, c=c: r['fast']['identity']...startswith(c))
                                                      for s, c in navs.items()}),
                       session_key=lambda v: _session_id(v), note=note)   # YAML 1.1 reads 20261027_161837 as an int
core.trend_row(trend_note, row)
```

`score_batch` enforces the same rules as before. It raises `ValueError` for a session that was
already scored or fitted on, or that is not newer than the epoch's newest session. A refit
writes `<name>-e(N+1).yaml` and refuses to overwrite one. scipy's bounded Powell is used when
scipy is installed, otherwise a deterministic bounded pattern search. The method used is
recorded in the epoch's `fit.rms_mean_m.method`.

## 7. Tests to move or keep

- Keep as adapter tests: the harness layout, the byte-identical run, the profile setting the
  time limit, the logic-exception ending, and the limits and URDF mount reading (they exercise
  mhseals_nav).
- `LOGIC_WORDS` and the line caps now apply to `adapter.py`. The core enforces its own rule in
  `fastsim/tests/test_core.py::test_the_core_names_no_domain`.
- Run the core's tests with `python3 -m pytest fastsim/tests` from the crane root (needs numpy,
  pyyaml and pytest; scipy is optional).
