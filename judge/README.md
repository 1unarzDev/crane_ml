# judge: crane's tier- and robot-agnostic run evaluation

The judge turns runs from any tier (fast sim, Unity, a real robot) into one record and answers
three questions with evidence, not only scores:

| question | function | output |
|---|---|---|
| How did these runs go, and why did the failures fail? | `run.judge_runs` | metrics with status, outcomes, stable failure clusters, model-relative first causes |
| Does the cheaper tier tell the truth about the dearer one? | `run.judge_tiers` | G4 gap per objective (bias CI, band verdict), failure-mode confusion, per-pair divergence, decision diff, gap origin; G3 with a roster |
| Is variant B better than A? | `run.judge_variants` | paired comparisons per objective, Holm, non-inferiority with declared margins |

Each report is `crane.judge.report/1` JSON plus a one-line verdict (at most 160 characters, for the
TUI agent panel, sim templates and cluster job summaries), Markdown, and a viewer feed of insight
anchors `[run, t0, t1, channels]`.

```
JUDGE G4 6/6 usable | quad.time_to_last_gate bias -3.09 [-4.71,-1.25] band 0.5 FAIL | split plant6 | mode agree 50%
```

## Pieces

| module | role |
|---|---|
| `record.py` | `RunRecord`: header (tier, scenario key, clock, window), typed channels with observability and sigma, events, caps, validity; deterministic save/load (G1 hashes) |
| `metric.py` | `Metric` (value + status + spans), `Property` with the NLC three-valued semantics (bounds e and L) |
| `task.py` | `TaskSpec`, `Objective` (safety first, never summed; NI margin and G4 band declared with a cite) |
| `pack.py` | `Pack` (metrics, decision events, outcome, segmenter, dependency graph, hints, replayer) |
| `source.py` | `Source`, `FidelityProfile`, `Refused` |
| `pairing.py` | exact / block / loose pairing; the design label travels with the comparison |
| `stats.py` | one statistics module (golden parity with `cluster/crane_cluster/promotion.py` and mhseals_testing `reliability.py`) |
| `compare.py` | paired comparisons, G3 (regret, MMRV, top-k recall), G4 (gap bands, failure-mode table), exclusion accounting |
| `insight/` | integrity, divergence (time vs path residual, CUSUM, counterfactual replay split), event-timeline diff, failure clusters with bootstrap stability, first cause, hints, LLM citation validator |
| `render.py` | report files, one-liner, Markdown, viewer feed |
| `sources/fastsim.py` | fastsim run directories |
| `packs/generic_motion.py` | robot-agnostic motion metrics |
| `packs/toy_quad/` | a planar quadrotor (sim, source, pack, replayer): the second domain and the seeded-fault fixture |

## A new robot

Write a `Source` (artifact -> `RunRecord`) and a `Pack` (metrics, outcome, decision event kinds, a
dependency graph, optionally a `Replayer` for the plant/interaction split). No core change; the
contract test fails if the core imports a domain. RoboBoat lives in
`mhseals_testing/mhseals_testing/judge_pack/`; the toy quadrotor here is the template.

## How the gap origin is decided

For each segment of the high-tier run, a domain `Replayer` predicts the high tier's track from its
own state: open loop (its commands through the low tier's plant) and reset closed loop (the low
tier's controller and plant). Errors are taken over a local horizon (`Replayer.horizon_s`) so an
open-loop unstable plant is judged on its n-step prediction. If the accumulated local drift explains
under 30% of the free residual, the gap is a decision or upstream divergence (the event-timeline
diff then names the first differing decision); otherwise its origin is the plant (open-loop share
of at least 50%) or the controller-plant interaction.

## Tests

`python3 -m pytest judge/tests` from the crane root. Every insight generator has a seeded-fault test
(plant fault, decision fault, clean control, invalid runs, cluster stability, dependency walk).
