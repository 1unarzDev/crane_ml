# plantcard: one identified plant with uncertainty, for every tier

A plant card (`crane.plantcard/1`) is an immutable, content-addressed bundle:

- a **family** (a parametric model plugin: `marine.fossen3:Fossen3`, `families.quad2d:Quad2D`);
- **blocks**: conjugate NIG states for the parameters that enter linearly (with forgetting, closed-form
  evidence and EIG), a **grid** posterior for the nonlinear ones (delay, thrust shape), a disturbance block;
- **particles** (`particles.npz`, 512 joint draws): the uncertainty interchange every consumer reads;
- an **envelope** (kNN support of the evidence with a leave-one-session-out threshold);
- **validation** records: held-out prediction coverage, computed with sessions as units.

The design, with the alternatives and the adversarial review, is
`~/.cache/research/sim2real/design-plantcard.md` ("Contract + conjugate core"). The core (`card.py`,
`family.py`, `update.py`, `validate.py`, `blocks/`) names no robot; `marine/` is the RoboBoat pack.

| module | role |
|---|---|
| `card.py` | `Card` (nominal, sample, rollout, predict_set, envelope_score, view), store, refs, `load` |
| `family.py` | `Family` base (batched RK4 rollouts, input delay), `ParamSpec`, `ParamBatch`, `Segment`, `Evidence` |
| `blocks/nig.py` | NIG with forgetting; identical to the NLC study's `nlc/calib/blr.py` (parity test) plus evidence, EIG, draws |
| `blocks/grid.py` | grid posteriors from NIG evidence |
| `update.py` | `propose(parent, evidence)`: fit every grid point, grid posterior, pin parameters that leave physical bounds, joint particles, disturbance, envelope |
| `validate.py` | `pos_err_max@H` events, leave-one-session-out, cross-conformal multipliers, Clopper-Pearson and cluster-bootstrap bounds, CRPS, the promotion policy |
| `marine/fossen3.py` | 3-DOF Fossen + 4 vectored thrusters: the `Tools/Characterization` ODE (tested equal), integral-form regressors |
| `marine/fieldtrials.py` | 10-18 open-loop pulse trials: GPS + gyro truth (odometry translation there is dead reckoning) |
| `marine/mcap_lite.py` | stdlib reader for uncompressed rosbag2 MCAP + CDR decoders (NavSatFix, Imu, Odometry, UInt16MultiArray) |
| `marine/seed.py` | prior card c0 from `profiles/characterized.json` |
| `marine/views.py` | `unity` profile, `nlc` VesselParams kwargs, `control` feedforward, `fastsim` spec |
| `marine/fastsim_plant.py` | `CardPlant`: fastsim `PlantModel` plugin (one particle per plant) |
| `marine/build.py` | the full fit: c0, evidence, LOSO validation, c1, exports |
| `families/quad2d.py` | planar quadrotor: the extension proof (fit, validate, store with no core edits) |

## Use

```python
import sys; sys.path.insert(0, '<crane root>')
import plantcard
card = plantcard.load('roboboat/blastoise@candidate')   # HEAD = accepted card; @candidate = latest fit
card.view('unity')                     # profiles/*.json dict for MarinePlanarModel (provenance.card_sha)
card.view('nlc')                       # VesselParams(**kw) for the NLC study; view('nlc', n=256): per-env particles
card.rollout(x0, t, u, n=64, seed=0)   # [n, T, 6], one particle per rollout
card.predict_set(x0, t, u, horizon_s=5.0, alpha=0.1)   # conformal radius from the card's validation record
```

CLI: `python3 -m plantcard show REF`, `python3 -m plantcard export unity|nlc|control|fastsim REF`,
`python3 -m plantcard.marine.build --robot roboboat/blastoise --repo` (refit; ~2 min on one core). The ref is
always given: plantcard has no default robot.

Fast tier: a vehicle spec `{ref: plantcard.marine.fastsim_plant:CardPlant, gain: 1.5, card: roboboat/blastoise@candidate, seed: 3}`.

Store: `$PLANTCARD_STORE` (default `~/.cache/plantcards`) then the git-tracked copy here (`cards/<sha12>/`,
`refs.json`). A card's `sha` covers its JSON and the particles' sha256; loading re-checks it.

## Status ladder and promotion

`prior` -> `fitted` (a proposal) -> `provisional` -> `validated`. A fitted card is promoted only when the
frozen policy passes on held-out real sessions (CP LCB95 of coverage >= 1 - alpha - 0.05, width <= 1.1 x the
parent's, CRPS <= the parent's) and the owner accepts it (`provenance.accepted_by`). `validated` needs >= 5
sessions on >= 2 days. Posterior spread is never coverage: raw particle quantiles are labelled
`uncalibrated`; `predict_set` scales them by the conformal multiplier of the validation record.

Tests: `python3 -m pytest` from the crane root runs every crane Python suite (this one is `plantcard/tests`; numpy and PyYAML only; the NLC
parity test runs when `~/nl-constraints-bench` exists).
