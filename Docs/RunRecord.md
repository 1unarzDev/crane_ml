# Run record: `crane.run/1`

```
runs/u7/
  run.json       the meta below
  traj.ctraj     poses and events, crane-traj/1 (theater/traj.py); absent for an event-kind run
  events.jsonl   {"t": 12.5, "kind": "gate", ...}, t in seconds from clock.t0
  metrics.json   the run's numbers
```

```json
{
  "schema": "crane.run/1",
  "run_id": "u7",
  "domain": "boat",
  "tier": "T2",
  "kind": "unity",
  "card": {"ref": "Tools/Characterization/profiles/characterized.json", "view": "unity"},
  "params": {"gain": 0.15},
  "seed": 7,
  "backend": "unity",
  "dtype": "float64",
  "identity": {"traj_sha256": "…"},
  "provenance": {"source": "…/run.ctraj"},
  "frame": {"world": "enu", "origin": "world", "body": "flu"},
  "clock": {"source": "sim", "t0": 0.0, "time_multiplier": null},
  "timing": {"t_src": "sim", "injected_delay_s": 0.12, "derate": [1.0, 0.8]}
}
```

```python
from crane import record, frames
r = record.read_run('runs/u7')                     # r.meta, r.events, r.metrics; r.traj loads on first use
ned = frames.to_common(r, {'world': 'ned', 'origin': 'world', 'body': 'frd'})
flat = frames.project_planar(r)                     # 6-DOF -> x, y, yaw, said so in flat.notes
a, b = frames.align(r, record.read_run('runs/f3'))  # one time grid over the overlap, on r's clock
```

## Fields
| Key | Value |
|---|---|
| `tier`, `kind` | `T0` with `event` or `kinematic`; `T1` with `dynamic`; `T2` with `unity`; `field` (tier may be null) |
| `frame` | **required.** `world` `enu`\|`ned`, `origin` a name, `body` `flu`\|`frd` |
| `clock` | **required.** `source` `sim`\|`wall`; `t0` the clock's time at run t = 0 (unix seconds on wall); `time_multiplier` sim s per wall s, null when unpaced |
| `timing` | optional: `injected_delay_s`, per-thruster `derate`, `t_src` (the clock the stamps come from) |
| `files` | optional: `{traj, events, metrics}` names when they differ from the defaults |

Angles are Z-Y-X (yaw, pitch, roll). ENU/FLU yaw is counter-clockwise from east, NED/FRD yaw clockwise from
north: converting maps yaw to `pi/2 - yaw` and pitch to `-pitch`, and keeps roll.

## Rules
- `validate(meta)` names every missing or bad key; `write_run` and `read_run` call it.
- `to_common` refuses different origins unless a transform `{from, to, xyz, yaw}` is given. A planar pose
  cannot change world without body (it would tilt).
- `align` refuses a sim run against a wall run unless `offset_s` places one in the other.
- A `.ctraj` written through `write_run` carries the same frame in its header (`{type, origin, body}`).

## Writers
| Writer | How |
|---|---|
| any engine | `record.write_run(dir, meta, traj, events, metrics)` |
| fastsim | `simulate()` writes `crane.run.json` beside its own `run.json` (`crane.fastsim.run/1`): T0 kinematic, planar, `enu`/`scenario`/`flu`, sim clock. `record.from_fastsim(dir)` adds it to an older run dir. |
| Unity T2 (`--crane-traj`) | `record.from_unity_ctraj(path, out_dir, domain=…)`: 6-DOF, `enu`/`world`/`flu`, sim clock |
| any other `.ctraj` | `record.from_ctraj(path, out_dir, tier=…, kind=…)`, frame from the header (`body` defaults to `flu` for enu, `frd` for ned) |

`read_run` reads `crane.run.json` first, then a `run.json` whose schema is `crane.run/1`.
