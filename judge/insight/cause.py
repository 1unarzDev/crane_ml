"""First cause, model-relative: walk the pack's dependency graph upstream from where a failure
shows and name the most upstream stage with an anomaly at or before the failure time.

The output's basis is ``model-relative:<graph id>``: it is only as right as the dependency model,
and says so (observational root-cause analysis, never a causal claim about the world)."""

from __future__ import annotations

from judge.insight import Insight
from judge.metric import Span


def first_cause(rec, values, pack, t_fail=None):
    g, fn = pack.dependency, pack.failure_node
    if g is None or fn is None:
        return None
    node = fn(rec, values)
    if node is None:
        return None
    t_fail = rec.header.window[1] if t_fail is None else t_fail
    found = []
    for name in [node] + g.ancestors(node):
        n = g.node(name)
        if n.check is None:
            continue
        for t, what in n.check(rec) or ():
            if t is None or t <= t_fail + 1e-9:
                found.append((name, t, what))
    if not found:
        return Insight('cause.none_found', 'info',
                       f'{rec.run_id}: failure shows at {node}; no upstream anomaly in {g.id}',
                       scope={'run': rec.run_id}, basis=f'model-relative:{g.id}')
    order = [node] + g.ancestors(node)
    name, t, what = max(found, key=lambda f: (order.index(f[0]), -(f[1] if f[1] is not None else 0)))
    chain = ' <- '.join(order[:order.index(name) + 1])
    ev = (Span(rec.run_id, t, t + 5, g.node(name).channels, name),) if t is not None else ()
    return Insight('cause.first', 'warn',
                   f'{rec.run_id}: failure at {node} traces to {name} ({what}'
                   + (f', t={t - rec.header.window[0]:.1f} s' if t is not None else '') + f'); chain {chain}',
                   evidence=ev, scope={'run': rec.run_id}, basis=f'model-relative:{g.id}',
                   data={'node': name, 't': t, 'what': what, 'failure_node': node})
