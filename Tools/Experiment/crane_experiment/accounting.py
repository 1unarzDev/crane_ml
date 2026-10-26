"""Cost and count accounting over trials. Every status is charged."""
STATUSES = ("valid", "invalid", "stale", "loophole")


def account(trials, budget):
    done = [t for t in trials if t["status"] != "pending"]
    by = {s: 0 for s in STATUSES}
    cost = {s: 0.0 for s in STATUSES}
    for t in done:
        by[t["status"]] += 1
        cost[t["status"]] += t["cost"]
    n = len(done)
    total = sum(cost.values())
    return {
        "n_trials": n,
        "n_pending": len(trials) - n,
        "by_status": by,
        "cost_by_status": cost,
        "total_charged": total,
        "budget_spent": budget.spent,
        "budget_total": budget.total,
        "charges_reconcile": abs(total - budget.spent) < 1e-9,
        "valid_fraction": (by["valid"] / n) if n else 0.0,
    }
