# E4 preregistration: compute-aware speed/compute policy vs the p99 governor (DRAFT, NOT FROZEN)

Status: **DRAFT**, written 2026-11-10 during D3 DEV work. Nothing here is frozen. It must be reviewed and frozen by the
user/orchestrator (sha256 recorded in `results/raw/e4_spec.sha256`) before any held-out layout is generated or run.
Evidence label for every outcome: IMPLEMENTED (2.5D point mass, synthetic sensors/forest/scenarios).

## Question
Does a joint policy that chooses speed AND compute level per planning cycle, using the measured latency tail, raise the
maximum safe speed over the existing p99-age governor (`opt_backup_t`)? H2: dv* >= +1 m/s at P(collision) <= 1 %.
Kill (pre-declared): dv* point estimate < 0.5 m/s on the E4 held-out set => report the D3 diagnosis (H1) as the result.

## Layouts
- **Layout seeds 90600-90615** (16 whole layouts), inside `splits.json`'s held-out block (E2 uses 90200-90207, E3 90400-90407).
  Moved from the ledger's 6000-6015 by the orchestrator (2026-11-10): that range collided with ForestSearch nbeacon's held-out labels
  and lies in the global `train` split, where `guard_seed` would not refuse it. SEEDS.md must be updated at freeze. D3 DEV work never
  touched 90600-90615 (DEV layouts used: 310-313, 320-324, 330-331, 340-341). Below, L_E4 = these 16 layouts.
- Each layout yields the four E3 families (stale S1, drift S3, occl S4, stall S0) by `scenarios.build(seed, family)`; run
  seed = layout seed for every method and speed (common random numbers). Wind `light_breeze`, detector `on`, link up.

## Vehicle profile (DECISION recorded)
- E4 uses the **CAD AOS7 profile** `Tools/Aerial/profiles/aos7_v5_f90_6s.json` (1.843 kg, T/W 4.79), passed explicitly
  (`load_params(profile_path=...)`, cond `profile: "cad"` in `experiments/d3_episode.py`). NavPilot's default stays pinned to
  v1 (`aos7_v5_f90_6s_v1.json`, 1.383 kg, T/W 6.38) so frozen E2/E3 remain reproducible; **all D3 DEV numbers in
  `results/D3_dev.md` are v1** except the labelled v1-vs-CAD row in its section 5. In NavPilot the horizontal authority is
  tilt-limited (8.23 m/s^2 for both profiles), so the profile change enters only through drag/mass: expect a small shift of v*
  in NavPilot (the larger effect, lower thrust margin and 1.6x inertia, lives in the .NET RK4 plant and is invisible here).
  Because of that, DEV tuning of the governor knobs must be repeated on CAD before freezing (cheap: tune+train ~25 min).

## Code (to be hashed at freeze)
`stacks/compute_gov.py`, `experiments/d3_episode.py`, `experiments/d3.py` plus the E3 modules. Tuned parameters come from DEV
only: `results/raw/e3_tuned.json`, `results/raw/d3_tuned.json` (analytic knobs), `results/raw/d3_lp.json` (learned weights).
No parameter is tuned on L_E4. The e3.py arms are NOT rerun (E3 held-out layouts stay untouched).

## Methods (to be fixed by the D3 DEV result; candidates)
| id | description |
|---|---|
| opt_backup_t | p99-age governor (the baseline for H2) |
| clear_only | tau = 0 (clearance and braking envelope only) |
| pas_full | E3 best method; fixed compute level 0 |
| cc_pas | analytic chance-constrained (speed, level) governor on pas_full |
| lp_pas | learned level/delta policy (CEM weights from DEV) |
| (optional) fix_l2 | pas_full pinned to the lightest compute level (compute-only ablation) |
Drop any arm that the D3 DEV report shows is dominated everywhere, before freezing, to cut cost.

## Lessons from D3 DEV that change this draft (to decide before freezing)
- v* from 20 episodes per setpoint is too noisy to carry H2 (one collision at 4 m/s moved a v* by 4 m/s). With 16 layouts x 4 families = 64
  episodes per setpoint it is better but still step-like: add as a co-primary the **pooled collision rate and mean progress over the
  >= 5.5 m/s setpoints** with layout-cluster bootstrap, and the contrast **cc_pas vs pas_full** (same belief, no adaptivity), because
  DEV showed that the large gain over opt_backup_t is the PAS component, not compute adaptivity (D3_dev.md sections 0 and 3).
- Proposed arms: opt_backup_t, pas_full, cc_pas, rr_pas, fix_l2 (drop clear_only and lp_pas: dominated on DEV; keep p99_nb/cc_nb only if the
  "without backup" claim of NOVELTY_D3 is wanted: 2 extra arms).
- The multi-rate acceptance rule in the plan is unattainable (chaos floor 84 % agreement), so E4 must run at 200 Hz.

## Plan and compute
Speeds {4, 5.5, 7, 9} m/s (the E3 grid minus 2.5, where nothing collides; add 2.5 only if a method collides there on DEV).
5 methods x 4 families x 4 speeds x 16 layouts = 1280 episodes at ~3-5 s each = about 1.4 CPU-h on one fsrun slot (cache
via `Tools/Frontier/cache.py` when it exists; held-out rows must carry `split: heldout`).

## Endpoints (decided now)
1. v*_m: speed where isotonic (PAVA, weights n) P(collision | v) crosses **1 %** (primary, H2) and **5 %** (E3-comparable),
   grid linear interpolation, same censoring rules as PREREG_SPEED.md; code = `e3.pava/v_star/curve_stats` (imported).
2. dv* = v*_m - v*_opt_backup_t, paired cluster bootstrap over layouts (2000 resamples, seed 0, all families/speeds of a
   layout move together), 95 % percentile CI. **Primary test:** H2 holds iff the CI of dv* (1 % crossing, pooled families)
   excludes 0 AND the point estimate is >= +1 m/s. Because the 1 % crossing with 16 layouts x 4 families = 64 episodes per
   speed is a coarse estimate, the 5 % dv* is reported alongside; they must agree in sign for a claim.
3. Secondary: dA* (achieved progress at v*), collision rate and progress per setpoint, compute budget used (mean level, detector
   recall), brake_infeasible counts, per-family v* (no CIs claimed), stall-watchdog trips.
4. Completeness guard: only layouts with every planned episode enter the analysis.
5. No multiplicity correction; with 4 contrasts a single marginal win is weak.

## Escalation (only after this prereg is frozen and the E4 run is reported)
Top-2 policies by DEV/E4 result go to the .NET harness at 5-8 m/s (see `results/D3_dev.md`, section "Candidates for .NET"),
via the ops agent; Unity validation only for policies that pass .NET.

## Declared limits (known now)
- The compute-level model tau(c), q(c) is partly assumed (stall scaling, detector share) and only the headings/frames work
  is exercised by real code; the measured planner cost scales weakly with n_headings (level 2 = 0.66x on this CPU).
  Any gain is therefore a statement about this model, not about Orin hardware, until `Tools/Vision/orin_bench.py` runs.
- Counterfactual arms are DEV diagnostics and are not part of E4.
