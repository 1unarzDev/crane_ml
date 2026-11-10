# Job spec: behaviour-cloned reactive policy inside a backup-set safety filter (E3 optional arm, NOT_RUN)

Status: SPEC ONLY. The harness has no training loop, dataset writer or learned-policy hook, and the shared 3-slot fsrun
queue cannot fit data collection, training and an E3-sized evaluation inside this pass. Nothing here has been run.

## Why it matters for the max-safe-speed question
Loquercio 2021 and Zhang et al. 2025 report learned reactive policies at 7-10+ m/s, with privileged-expert supervision.
In our harness the binding constraint is first-detection range of thin branches (E2/E3 mechanism tables). A learned
policy sees no more than the sensors do, so we predict it raises A\* only where the analytic planners waste speed (open
stand, correct maps) and not in `stale` or `occl`. The safety filter is what would make it reportable at all.

## Pipeline (each step is a job file for `experiments/worker.py`)
1. **Expert (privileged).** `AdvStack` with `belief="map"`, prior = truth geometry including branches (S0-style
   rasterisation of `scn["truth"]`), `pas=False`, and v_lim drawn from U(3, 11). This expert knows every branch, so its
   speed is limited only by dynamics and the stopping rule.
2. **Data (TRAIN split only, seeds 2000-2399).** 400 layouts x 4 families x 2 speeds. At each planner cycle, log the
   observation features and the expert's (theta - goal heading, v_cmd / v_lim):
   - 64-bin egocentric range image from the RangeOccMap fresh map (min range per bearing, cap 15 m);
   - 64-bin "confirmed" image (range to the first cell with rmin > R_conf near structure);
   - speed, goal bearing, p99 age, sigma_loc_rate.
   DAgger with 3 rounds: roll out the learner, relabel with the expert, aggregate. About 3 x 3200 episodes at ~3 s
   is ~8 h on one slot, which is why it is not in this pass.
3. **Policy.** A 2-layer MLP (numpy, 128 hidden units, tanh) that outputs a heading-offset bin distribution (48 bins)
   plus speed fraction. Train it with cross-entropy plus MSE, Adam, early stopping on VAL seeds 70000-70099.
4. **Safety filter (backup set).** The learner's (theta, v) is accepted iff it passes the same feasibility test as
   opt_backup_t (the braking stop lies inside fresh known-free space, with the p99-age rule and watchdog). Otherwise
   the filter returns the feasible primitive closest to it in (theta, v) (a minimal-intervention projection, as in
   predictive safety filters), and stops if none exists. Log the intervention rate.
5. **Evaluation.** Add `bc_shield` to `e3.HELD_METHODS` and evaluate it on a further fresh held-out block (seeds
   90500-90509, never touched) under the PREREG_SPEED.md protocol, with opt_backup_t and the best E3 method re-run
   there for pairing.

## Acceptance
Report A\*, v\* and the intervention rate per family. Count it as a gain only if dA\* vs opt_backup_t has a 95 % CI
excluding 0 and the filter intervenes on fewer than 20 % of cycles. Otherwise the filter, not the policy, is doing the
work.
