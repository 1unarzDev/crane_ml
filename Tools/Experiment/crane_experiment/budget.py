"""Budget charged by worker-reported cost. Failed trials are charged too."""
import math

from .errors import BudgetExhausted


class Budget:
    def __init__(self, total, reserve=0.0):
        self.total = float(total)
        self.reserve = float(reserve)   # held back from kind=search for confirmation trials
        self.spent = 0.0
        self.outstanding = {}           # trial_id -> reserved estimate

    @property
    def reserved(self):
        return sum(self.outstanding.values())

    def remaining(self, search=False):
        lim = self.total - (self.reserve if search else 0.0)
        return lim - self.spent - self.reserved

    def reserve_trial(self, trial_id, est, search=False):
        if est > self.remaining(search) + 1e-12:
            raise BudgetExhausted(
                f"estimated cost {est:.3f} exceeds remaining {self.remaining(search):.3f}")
        self.outstanding[trial_id] = float(est)

    def charge(self, trial_id, cost):
        """Charge the worker-reported cost, whatever the trial's status."""
        self.outstanding.pop(trial_id, None)
        if cost is None or not math.isfinite(cost) or cost < 0:
            raise ValueError("worker-reported cost must be finite and >= 0")
        self.spent += float(cost)
