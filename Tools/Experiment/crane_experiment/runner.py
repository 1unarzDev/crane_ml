"""Drive a study to completion with a worker: baseline -> search -> confirm."""
from .errors import BudgetExhausted, StudyError


def run_study(study, worker, role="human", confirm=True, max_search=10_000):
    n = 0
    while n < max_search:
        try:
            t = study.ask(role)
        except BudgetExhausted:
            break
        study.evaluate(t["trial_id"], worker, role)
        n += 1
    if confirm:
        while True:
            try:
                t = study.ask(role, kind="confirm")
            except (BudgetExhausted, StudyError):   # confirm pass is finite
                break
            study.evaluate(t["trial_id"], worker, role)
    return study
