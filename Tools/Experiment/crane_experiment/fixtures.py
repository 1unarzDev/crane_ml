"""Demo specs over the fake worker. FIXTURE ONLY."""
PROFILE_STATE = "FIXTURE_ONLY"


def demo_spec(seed=1, n_search=40, n_train=3, required_state="FIXTURE_ONLY", study_id="fake-demo"):
    feats = [0.2, 0.5, 0.8][:n_train] if n_train <= 3 else [i / (n_train - 1) for i in range(n_train)]
    contexts = []
    profiles = {}
    for i, c in enumerate(feats):
        p = f"fake_profile_t{i}@{'a' * 8}{i}"
        contexts.append({"id": f"train{i}", "site": f"siteT{i}", "session": f"sessT{i}",
                         "split": "train", "profile": p, "features": [c]})
        profiles[p] = {"calibrationState": PROFILE_STATE}
    for i, c in enumerate([0.35, 0.65]):
        p = f"fake_profile_h{i}@{'b' * 8}{i}"
        contexts.append({"id": f"held{i}", "site": f"siteH{i}", "session": f"sessH{i}",
                         "split": "heldout", "profile": p, "features": [c]})
        profiles[p] = {"calibrationState": PROFILE_STATE}
    base_reps = 2
    n_pairs = n_train * base_reps
    return {
        "schema": "crane.study/v1", "study_id": study_id, "seed": int(seed),
        "space": {"x0": {"low": 0.0, "high": 2.0}, "x1": {"low": 0.0, "high": 1.0},
                  "x2": {"low": -1.0, "high": 1.0}},
        "default": {"x0": 1.6, "x1": 0.2, "x2": 0.5},
        "contexts": contexts,
        "fidelities": {"sim_low": {"cost": 0.25}, "sim_high": {"cost": 1.0}, "real": {"cost": 10.0}},
        "default_fidelity": "sim_high",
        "budget": {"total_cost": float(n_search + n_pairs + 1.25 * n_pairs),
                   "reserve_cost": 1.25 * n_pairs},
        "crn_replicates": 2, "baseline_replicates": base_reps,
        "max_obs_age_s": 1.0, "min_valid_fraction": 0.5, "alpha": 0.2,
        "profiles": profiles,
        "claim_scope": {"requiredCalibrationState": required_state},
    }
