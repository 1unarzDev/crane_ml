"""RECOMMEND only when every gate passes; otherwise ABSTAIN with exactly one reason."""

REASONS = ("LOOPHOLE_FLAGGED", "INSUFFICIENT_VALID", "NO_CANDIDATE", "PROFILE_MISMATCH",
           "CALIBRATION_MISMATCH", "CONSTRAINT_RISK", "NO_IMPROVEMENT")

_T95 = {1: 6.314, 2: 2.920, 3: 2.353, 4: 2.132, 5: 2.015, 6: 1.943, 7: 1.895, 8: 1.860,
        9: 1.833, 10: 1.812}


def t_crit(df):
    if df in _T95:
        return _T95[df]
    return 1.70 if df <= 30 else 1.645


def lcb_mean(d):
    """One-sided 95% lower confidence bound of the mean of paired differences."""
    n = len(d)
    if n < 2:
        return float("-inf")
    m = sum(d) / n
    var = sum((x - m) ** 2 for x in d) / (n - 1)
    return m - t_crit(n - 1) * (var / n) ** 0.5


def decide(g):
    """g: dict of gate inputs. Returns (action, reason)."""
    if g["loophole_count"] > 0:
        return "ABSTAIN", "LOOPHOLE_FLAGGED"
    if g["n_trials"] == 0 or g["n_valid"] == 0 or g["valid_fraction"] < g["min_valid_fraction"]:
        return "ABSTAIN", "INSUFFICIENT_VALID"
    if not g["has_candidate"]:
        return "ABSTAIN", "NO_CANDIDATE"
    if g["profile_mismatch"]:
        return "ABSTAIN", "PROFILE_MISMATCH"
    if not g["calibration_ok"]:
        return "ABSTAIN", "CALIBRATION_MISMATCH"
    if g["constraint_risk"] > g["alpha"]:
        return "ABSTAIN", "CONSTRAINT_RISK"
    if not (g["improvement_lcb"] > 0):
        return "ABSTAIN", "NO_IMPROVEMENT"
    return "RECOMMEND", None
