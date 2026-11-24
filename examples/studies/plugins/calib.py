"""Example scorers for examples/studies/boat_cards.yaml (a plugin dir module).

    parts(result, cell)          a unit scorer: named parts of one episode (rows get score.PART)
    gate_fit(rows, group)        a population scorer: named parts of one card x block, from its rows (condition
                                 columns cond.* and sim columns side by side)
"""


def parts(result, cell):
    m = result['metrics']
    return {'success': m.get('succeeded', 0.0), 'gates': m.get('boat.gates_passed', 0.0)}


def gate_fit(rows, group):
    """How well the sim's gate count matches each condition's observed one (cond.observed_gates): the mean absolute
    error (loss), its mean signed part (bias), and the censored share. The gate count covers the whole episode, up to
    the row's time_cap_s (the condition's time_limit_s plus the tier's grace_s); a censored row reached that cap and
    carries censored = true and time_cap_s, never NaN."""
    errs = [r['boat.gates_passed'] - r['cond.observed_gates'] for r in rows
            if r.get('boat.gates_passed') is not None and r.get('cond.observed_gates') is not None]
    if not errs:
        return {}
    return {'loss': sum(abs(e) for e in errs) / len(errs), 'bias': sum(errs) / len(errs),
            'censored': sum(1 for r in rows if r.get('censored')) / len(rows)}
