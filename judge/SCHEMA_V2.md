# `crane.judge.run/2` — contract for the sprint build (2026-11-09)

One run record for every backend: the fast array-based layer, the integrated game-engine layer driven over
the lockstep link, and physical log replay. `/1` records stay loadable. This file is the contract the
schema, sources and analysis owners build against; change it only through the integration owner.

## Header additions (all optional in `/1`, required in `/2`; unknown = explicit `null`, never omitted)

| field | type | meaning |
|---|---|---|
| `backend` | `{layer: 'fast'|'integrated'|'physical', name: str, build: str|null, mechanisms: [str]}` | which layer produced the run and the mechanisms it represents (free vocabulary, e.g. `thrust.lag.first_order`, `thrust.reversal_hold`, `water.heave_roll`) |
| `controller` | `{id: str, library: str|null, spec_sha: str|null, params: dict}` | the controller under test |
| `calibration` | `{twin: str|null, arm: str|null, evidence_sha: str|null, lineage: [str]}` | which twin (if any) selected this controller and from what evidence |
| `clock_mode` | `{mode: 'fixed_step'|'lockstep'|'realtime'|'recorded', dt_s: float|null, time_scale: float|null, rtf: float|null, episode: int|null}` | how time advanced |
| `code` | `{repo: str, commit: str, dirty: bool, files_sha: dict}` | code revision |
| `evidence` | `{kind: 'confirmatory'|'exploratory'|'demo'|'replay', prereg: str|null, seed_class: str|null}` | keeps confirmatory runs and exploration apart |
| `reset` | `{method: str|null, verified: bool|null, pose_error: [float]|null}` | how the episode started and whether the start state was checked |

`Scenario` (existing) carries the scenario id, seed, family and condition. `Validity` (existing) is written
only by `judge.validity`, never by a source: a source reports facts, the judge decides.

## Standard channels (names are fixed; a source provides what it has, and declares the rest absent)

Time base: seconds since episode start on the backend's own clock; every channel also carries `tick` when
the backend has ticks.

| channel | fields | observability | notes |
|---|---|---|---|
| `truth.pose` | x, y, yaw | exact / surveyed | evaluator truth, scenario frame; never an input to a controller |
| `truth.twist` | u, v, r (body) | exact / estimated | |
| `obs.pose`, `obs.twist` | as above | estimated | what the controller received |
| `obs.stamp` | t_acq, t_recv | exact / absent | acquisition time and receipt time of each observation the controller used |
| `cmd.requested` | q0..q3 (normalised) | commanded | what the controller asked for, at the time it asked |
| `cmd.sent` | p0..p3 (pulse, us) + t_send | commanded / absent | what left the controller side |
| `cmd.applied` | q0..q3 + t_apply | exact / absent | what the plant applied and when; absent when the backend does not expose it |
| `act.thrust` | f0..f3 (N) | exact / absent | actuator output |
| `env.current`, `env.wind` | x, y | exact / absent | |

Events (existing `Event` type; vocabulary for the docking task): `episode_start`, `gate_cross{y}`,
`contact{with}`, `dock_enter`, `dock_hold_start`, `docked`, `timeout`, `link_lost`, `reset`, `abort{why}`.
Terminal reason = `window[2]` ∈ `docked | timeout | abort | link_lost | infra_error | track_end`.

## Three questions, three outputs

1. `judge.validity.check(record) -> ValidityReport{valid, reasons:[{code, severity, evidence}]}`. Codes
   (shared across backends): `clock.nonmonotonic`, `clock.dt_mismatch`, `clock.offset`, `clock.rtf_starved`,
   `frame.mismatch`, `telemetry.missing_channel`, `telemetry.gap`, `telemetry.rate`, `reset.failed`,
   `reset.unverified`, `cmd.delivery_gap`, `cmd.rejected`, `infra.crash`, `infra.link_lost`,
   `infra.backend_flag`, `record.schema`. A mission failure (no dock, contact, timeout) is never a validity
   reason. Invalid runs stay in every report with their reasons.
2. Outcome: a task pack scores the same quantities on every backend from `truth.*` and events only:
   docked, time to dock, retries, terminal pose error, constraint violations (contacts, gate miss), and the
   study's scalar score. Selection regret is computed across records with a stated uncertainty and the
   runs that drive it.
3. Divergence between matched records (same scenario, seed, controller): tick-matched trajectory and event
   timeline comparison; the first meaningful disagreement with the evidence around it; an attribution in
   `{clock_or_timestamp, command_delivery, plant, sensor, controller, unknown}` with the test that
   supports it. `unknown` is a first-class answer when the logs cannot separate the candidates.

## Rules

- Core imports numpy and the standard library only (`tests/test_domain_guard.py`).
- Writers are deterministic (sorted keys, fixed zip timestamps); equal runs hash equally.
- A record never mixes evaluator truth into `obs.*`.

## Deviations (schema + validity owner, 2026-11-09)

Nearest workable implementation where the contract was silent or underspecified; all additive.

1. **`backend.flags`** (`{name: detail}`, default `{}`) added to the backend block: the contract asks
   validity to read "backend-reported flags carried in the header" but had no field for them. Flag
   `cmd_rejected` maps to `cmd.rejected`, `crash` to `infra.crash`, `link_lost` to `infra.link_lost`, every
   other flag to `infra.backend_flag` (invalid unless the task's `Expect.benign_flags` lists it).
2. **Channel spec gains `note` and `obs_is_truth`** (strings, written only when non-empty, so `/1` bytes are
   unchanged). `declare_absent(name, why)` = an empty channel with observability `absent` and `note=why`.
   An `obs.*` channel whose values are its `truth.*` counterpart (same object, shared memory or equal bytes)
   is refused by `save` unless `obs_is_truth` holds the reason.
3. **Time semantics of `obs.*`**: row `t` = when the controller used the observation; the value corresponds
   to truth at `obs.stamp.t_acq` (pairing by tick, then time, then index). `obs.stamp` row `t` = use time.
4. **Declared start**: `Scenario.condition['start_pose']` = `[x, y, yaw]` and optionally
   `['start_twist']` = `[u, v, r]` (or `Expect.start_pose/start_twist`). A `reset.method` equal to `rest` or
   ending in `.rest` (e.g. `teleport.rest`) claims the episode starts at rest.
5. **`reset.verified`**: `true` = the source checked the start state. `reset.unverified` (warning) fires only
   when it is not `true` and the judge had no declared start to check against itself.
6. **Save version selection**: all seven blocks -> `/2`; none -> `/1` (byte-identical to the old writer);
   some -> refused. `save(..., schema=SCHEMA_V2)` insists on `/2`. Loading a `/2` file missing a block fails.
   Inside blocks, nullable fields are written as explicit `null` (lists as `[]`).
7. **Report shape**: `ValidityReport` also carries `run_id`, `checked` and `skipped{check: why}` so "no
   finding" and "not checkable" stay apart; each reason has a one-line `message`. `apply` writes
   `Validity.flags = {code: worst severity}`, `excluded_from = {G3, G4}` when invalid, and keeps the full
   report in `attachments['validity_report']` (in memory, not saved).
8. **Severities chosen by the judge**: `record.schema` on a `/1` record is a warning; `reset.unverified`,
   duplicated timestamps, unlogged-and-undeclared optional channels, out-of-range command values or pulses,
   `rtf_starved` in lockstep mode, and short gaps (< 0.5 s) or scattered (< 0.2 s runs) delivery misses are
   warnings; everything else is invalid. Non-finite truth samples are `infra.crash`.
