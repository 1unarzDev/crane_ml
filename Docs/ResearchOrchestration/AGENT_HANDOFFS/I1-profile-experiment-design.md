# I1: unified profile schema + experiment API design (Plan, Opus)

The full design was returned to the orchestrator, and these are its key points.

**Environment:** offline the only packages available are numpy 2.5.4, pytest, PyYAML and Python 3.14. scipy, optuna, botorch, jsonschema and cma are missing, so everything is pure numpy, with optional adapters.

## A. `crane.profile/v2` (Tools/Profiles/)
- **Envelope fields:** kind, domain, version, frames.
  - `calibrationState` (UNCALIBRATED_PRIOR, SIM_TRUTH_RECOVERY, PHYSICALLY_CALIBRATED, FIXTURE_ONLY) is schema-enforced, not set by naming.
  - Also: evidenceLabel, sources (with license), datasets (with origin physical|sim_truth|synthetic-fixture and sites/sessions), components by id@sha, coefficients, claimScope.excluded, migratedFrom, and a canonical sha256 contentHash.
- **Quantity fields:**
  - value and unit, from a registry that is dimension-checked against the key suffix
  - frame, required for vectors
  - provenance: cited|fitted|prior|derived|fixtureOnly
  - uncertainty
  - identifiability, with status and reason (RANK_DEFICIENT, ILL_CONDITIONED, CONFOUNDED_WITH:<path>, INSUFFICIENT_EXCITATION, SINGLE_SITE, NO_PHYSICAL_DATA)
  - alternatives
- **Validator rules R1–R12:**
  - R5: PHYSICALLY_CALIBRATED requires a physical dataset hash plus a held-out split by site or session.
  - R6: sim-truth and physical data may only be mixed if the mix is labelled.
  - R7: a fixtureOnly profile is capped at FIXTURE_VALIDATED.
  - R11: a profile that sources uncommitted user work is capped at IMPLEMENTED.
  - R12: no site, session or runId may overlap between fit and held-out data.
- **Migrations with parity tests:**
  - campus profile split into a robot profile plus dynamics/actuator/sensor/timing, with a v2→v1 exporter so Unity keeps working
  - heldout schema → dataset_manifest_v2
  - bluerov2_heavy
  - 4 aerial profiles
  - 3 terrain profiles (their non-CONTRACTS labels are fixed)

## B. `crane_experiment` (Tools/Experiment/)
- **Spec and lifecycle:**
  - The spec format is `crane.study/v1`.
  - `create` freezes the spec's SHA.
  - The loop is ask → evaluate(worker) → result. `tell` is internal only.
  - The ledger is an append-only, hash-chained JSONL, with evidenceId `ev:<sha12>`.
- **Budget:** charged by worker-reported cost, and invalid, stale or loophole trials are still charged. Optimizer failures count as constraint violations.
- **Randomness:** common random numbers via a SeedSequence.
- **Splits:** splits are by whole site or session, and `ask` never returns held-out contexts until `freeze()`.
- **Abstention:** the report says RECOMMEND only when the valid fraction is sufficient, constraint risk is ≤ α, the lower confidence bound of improvement over the default is > 0, no loophole was found, and the profiles' calibration state matches the claim. Otherwise it says ABSTAIN with one of 7 reasons.
- **Baselines:** default, random, Sobol, CMA-ES, TPE, GP-BO, contextual GP, and a two-fidelity multi-fidelity BO, all in pure numpy.
- **LLM contract:**
  - The LLM uses the same JSON CLI as people.
  - It may not call `tell`, edit the ledger, touch held-out data before freeze, use real fidelity, or edit the preregistration.
  - `claims check` enforces `ev:`/`prof:` citations and the evidence label ceiling, and blocks banned claims.
- **Fake worker:** a quadratic with a known contextual optimum. It has invalid, stale and loophole regions, and the loophole is detected by independent invariants. A meta-test checks that disabling the detector makes the suite go red.
- **CRANE binding (crane_nav2.py):** this is read-only through existing artifacts (navigation-reset-summary, crane-benchmark-v1 counters, inspect_replay). A PROFILE_MISMATCH occurs if `calibrationProfileSha256` ≠ the context hash. It stays NOT_RUN until the preregistration is frozen.
  - **Orchestrator amendment:** the isolated ROS domain must lie in 200–228, the same range as the crane_ros guard, and never 42/142.
