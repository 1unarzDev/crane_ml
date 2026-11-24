# Field to sim

```bash
B=~/astro_dock/bags/sim/2026-11-24/bundle-173031.tgz          # astro bundle [--bags]
crane fit $B --bags ~/astro_dock/bags/sim/2026-11-24/bags --out /tmp/fit
crane run --domain boat --tier T0 --astro-test all --space small --tests $B --card /tmp/fit/profile.json --out /tmp/sim/t0
crane run --domain boat --tier T1 --astro-test all --space small --tests $B --card /tmp/fit/profile.json --out /tmp/sim/t1
crane sim up --headless --profile-json /tmp/fit/profile.json && astro test all     # T2: astro's own tests on Unity
crane compare --field $B /tmp/sim/t1 --out /tmp/cmp                                 # per test: field, sim, delta, CI
crane compare --field $B /tmp/sim/t1 --bounds file --fit /tmp/fit                   # pass/FAIL per row, verdict, exit 0/1
crane compare /tmp/sim/t0 /tmp/sim/t1 --tiers                                       # T0 vs T1
crane field import $B --out /tmp/field                                              # the bags as crane.run/1
```

`crane fit` prints the gate's verdict per parameter (identified, provisional or prior, and why), then the
evidence, the fitted card and its validation. `crane compare --field` prints one table per test:

```
### speed-surge (speed-surge-171550.yaml) vs sim `profile`
| metric | unit | field | sim | delta (sim - field) | 95% CI | over |
| measured.t95 | s | 2.885 | 3.027 | 0.1418 | [-0.3025, 0.6163] | field samples |
| speed_curves.surge+.d1 | e^2 s/m | 0.06758 | 0.05649 | -0.01109 | - | - |
```

## crane fit

`crane fit BUNDLE… [--bags DIR] [--out DIR] [--base PROFILE] [--robot NAME] [--no-card]`

1. **Gate.** `Tools/Characterization/bundle.py` reads the bundle; `gate.py` checks it and `import_yaml.py`
   writes `OUT/profile.json` (for `sim_up --profile-json`) and `OUT/profile.gate.json`. Identified and
   provisional parameters overwrite the base (Ideal); the rest stay prior. On abstain the summary lists the
   exact missing fields.
2. **Evidence.** Every hold of every result (speed, couple, steps) that a bag covers becomes a plant-card
   segment (`plantcard.marine.fieldtrials.hold_segment`). Odometry comes from `/odom/local`. Thruster commands
   come from `/thrusters/pwm` (physical outputs), mapped to the thrusters by the state's `channel_map` and
   `reversed`. A bundle is one session. A sim-made state gives tier `unity` segments.
3. **Card.** `plantcard.update.propose` fits a child of the prior card built from `OUT/profile.json`. Both
   go to `OUT/cards`: `<robot>` is the prior and `<robot>@candidate` is the fit. The card's provenance names
   each bundle (sha256), its result files and the software commits.
4. **Validation.** With two or more bundles, the fit runs leave-one-session-out and then
   `validate.promotion` on `pos_err_max@H5s`. With one bundle the card stays `fitted` and cannot be promoted.

`OUT/fit.json` holds the same summary as JSON.

### What the gate reads from a bundle

| Gate input | From |
|---|---|
| characterization | `state/hardware*.yaml` `characterization` |
| software sha | `software: {repo: sha}` of the results (`mhseals_testing`/`mhseals_hardware` keys read the same) |
| per direction | the applied (else newest) `speed-<axis>` result: id, session, file, state hash |
| hardware hashes | the state's `thruster_matrix`, `reversed`, `hardware_epoch` and `channel_map`, `pwm_channels` |
| holds | the speed results' `holds: [{command, start, end, samples: {t, vx, vy, wz[, yaw, t_sensor]}}]` |
| hold windows | baseline = the test's `baseline_s` before `start`; steady = the last `measure_s` before `end` |
| conditions | the `still` result's `measured.conditions` (`verdict, ceiling, drift_trust, state`), when present |

A state whose characterization `source` starts with `sim` is SIM_DERIVED. Its directions stay prior, and its
card is labelled tier `unity`.

**Version 1.** astro's speed tests write `characterization.version: 1`. Version 1 maps to version 2 without
loss, with one exception. Both versions use the same signed curves (`e^2 = d1 v + d2 v|v|` per direction) and
the same `motion_limits.velocity`. The exception is `t95`: version 1 keeps the median over every hold, while
version 2 keeps the rise of the high-amplitude `+` hold. `gate.characterization_v2` recomputes that rise from
the holds' samples. An axis with no such hold loses its `t95`, and M_eff then stays prior. Version 2 also has
`wave_floor`, `identified_at`, `sample_count` and `motion_limits.measured_at_effort`. If a version-1 block
whose curves are not signed `d1/d2` pairs reaches the gate, the gate abstains with `PROVENANCE_MISSING`, and
the reason lists what version 2 adds.

### Card factorials

`crane fit --factorial SPEC.yaml [--out DIR]` writes one card view per combination of property levels, plus
`manifest.json`:

```yaml
measured: /tmp/fit/cards/cards/5d10b8bc4c28     # a plant card (dir, sha, robot@ref) or a profile JSON
prior: Tools/Characterization/profiles/characterized.json
properties: {surge: [m11, d1_u, d2_u], yaw: [m66, d1_r, d2_r]}
hypotheses: {surge: {draggy: {d2_u: 20.0}}}       # extra levels: property -> {level: {param: value}}
run: {domain: boat, tier: T1, astro_test: speed-surge, space: small, tests: B.tgz, bundle: B.tgz}
```

Each property's levels are `measured`, `prior` and the listed hypotheses. Each variant is the card's `unity`
view at the mixed parameters. Its provenance records the levels, both sources and the spec's sha. The
manifest's `command` runs every variant (`crane run --card manifest.json`) and compares the runs with the
field (`crane compare --field`).

## The same tests in sim

`crane run --domain boat --tier T0|T1 --astro-test TEST|GROUP [--card CARD] [--tests FILE] [--space S] [--amp A]`

- `Tools/Characterization/astro_tests.py` runs astro's test types (`still`, `couple`, `speed`, `steps`,
  `nav2`) on a simulated vessel. The command programs come from `excitation.astro_holds`, using the tests
  file's timing, levels and signs. The maths is astro's: response with world-frame drift removal, speed
  curve, `speed_at`, `t95`, step metrics and Nav2 caps.
- **T0** (`FirstOrderVessel`): per axis, a lag toward the steady speed the card gives an effort, with the
  damping time constant at that speed. No forces.
- **T1** (`ProfileVessel`): the MarinePlanarModel twin. Efforts reach the thrusters through the linear mixer
  `q = clip(M e)`, which matches the hardware for single-axis commands. Steps use a feedforward + P velocity
  loop.
- **T2** is astro itself on the Unity world (`crane sim up`, then `astro test`).
- Runs land in `OUT/<card>/<test>/s<seed>/`. Each run's `metrics.json` has the astro result's fields
  (`test, type, layer, space, amp, summary, measured, changes`). `result.yaml` is the whole result,
  including the holds.
- A variant's tests share one vessel and one state. Each result's changes are applied as `astro apply`
  would, so `nav2` and `steps` read what the speed tests measured.
- `couple` runs only the first hold of each sign, with the column in force; there is no column search.
- `--set current=[cx,cy]` adds a steady current.

## crane compare

```
$ crane compare --field $B /tmp/sim/t1 --bounds file --fit /tmp/fit
speed-surge vs sim `profile`
  metric                     unit          field        sim      delta  bound                          result
  measured.speed.surge+@0.5  m/s           0.931      0.934     0.003  +-0.0931 (10%)                 pass
  measured.t95               s             2.885      3.027    0.1418  +-0.7213 (25%, or CI contains 0) pass
  t95.surge                  s             2.885      3.027    0.1418  -                              n/a
FAIL speed-surge measured.speed.surge-@0.5: delta -0.1 m/s outside +-0.08 m/s (10%); fix: refit the axis curve: crane fit B.tgz
gate: yaw+ not identified
verdict: FAIL (1 of 3 outside bounds; gate: yaw+ not identified)
next: refit with crane fit B.tgz --out DIR, then rerun the sim with --card DIR/profile.json
```

Exit 0 on pass, 1 on FAIL, 2 on usage.

- `crane compare A B` matches runs by path. It puts B in A's frame (`crane.frames.to_common`) and on A's
  clock (`align`), then gives position and heading errors plus metric deltas. Different origins, or a sim
  clock against a wall clock, are refused.
- `--tiers`: per metric, `judge.compare.tier_agreement` (regret, MMRV, top-k, Kendall) over the variants
  (the first directory level), and `tier_gap` over runs paired by path. `--fidelity CONFIG` also scores
  CONFIG's vehicle on B's goal windows (`fastsim.fidelity.score`).
- `--field BUNDLE RUNS`: per astro test, each result metric for field and sim, with the delta (sim − field)
  and a 95 % bootstrap CI (`judge.stats.bootstrap_ci`). The CI resamples the field's per-hold samples when
  the metric has them (`measured.t95`), otherwise the sim runs when there are two or more.
- Metric names are astro's: change keys without `characterization.`, then `measured.*`. For example
  `speed_curves.surge+.d1`, `t95.surge`, `measured.speed.surge+@0.35` (m/s at effort 0.35),
  `measured.cross.surge+.sway`, `measured.surge+.rise_time`.
- The output is a Markdown table; `--out DIR` also writes `compare.json`.

### Fidelity bounds

`--field BUNDLE RUNS --bounds file|FILE [--domain D] [--fit DIR] [--json]`

`file` reads `config/fidelity/<domain>.yaml` (`--domain`, default `boat`); a FILE path reads that file. A
missing file is refused in one line that names it. The file is a first-match list of rules, and the same
format serves every domain:

```yaml
gate: {require: identified}
rules:
  - {metric: 'measured.speed.*', rel: 0.10, ci_zero_ok: false, unit: m/s, hint: 'refit the axis curve: crane fit {bundle}'}
  - {metric: 'measured.t95', rel: 0.25, ci_zero_ok: true, unit: s}
  - {metric: 'measured.cross.*', abs: 0.02, unit: '1'}
```

- `metric` is a glob over the row's metric names. `rel` is a fraction of the field value, `abs` is in the
  row's unit. A row passes when `|sim − field|` is within the bound, or when `ci_zero_ok` is set and the
  row's 95 % CI contains 0. A row no rule matches is `n/a`.
- Boat: speed at each level within 10 %; `measured.t95` within 25 % or a CI that contains 0; the couple
  cross response (a ratio, unit 1) within 0.02. Steps rows have no rule.
- Gate: `--fit DIR` (default `fit-<bundle>`, `crane fit`'s default output) is read for `fit.json`. Every
  direction and added-mass row must be `identified`. Without a fit it prints `gate: not checked` and does not
  fail.
- The verdict is `pass (N rows, M bounds)` or `FAIL (k of N outside bounds; gate: X not identified)`.
  `--json` prints the rows (each with `result`, `bound`, `rule`) and the verdict.
- `--bounds card` (tolerances from the plant card's particle spread) is not yet available.

## crane field import

`crane field import BUNDLE|BAG_DIR|FILE.mcap --out RUN_DIR [--odom /odom/local] [--events /events] [--frame ID=enu|ned] [--hz 20]`

Each bag becomes one crane.run/1 record of `kind: field`:

- `traj.ctraj`: planar `boat.x, boat.y, boat.yaw` from the odometry, resampled on the header stamps.
- `frame` comes from the odometry `frame_id`. REP 103/105 names (`odom`, `map`, `world`, …) are ENU with a
  FLU body, and a `*_ned` name is NED with an FRD body. Any other frame id is refused unless `--frame` names
  it. The origin is the frame id.
- `clock`: header stamps on the wall clock give `source: wall`. Stamps on a simulator clock give
  `source: sim`, with the fitted sim seconds per wall second as `time_multiplier`. `timing.t_src` is
  `header.stamp`.
- `events.jsonl` holds the `/events` lines (std_msgs/String) and the bundle's `log.txt` lines as
  `{t, kind: log, log_kind, text, source}`, each line once. `log.txt` times are local; their offset comes from
  the lines also seen on `/events`. When none match, this machine's time zone is assumed and recorded.
- MCAP is read with `plantcard/marine/mcap_lite.py`. Compressed chunks need the `mcap` package
  (`[field]` extra).
