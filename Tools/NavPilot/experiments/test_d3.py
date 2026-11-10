# SPDX-License-Identifier: GPL-3.0-or-later
"""D3 episode parity: with no counterfactual, 200 Hz plant and an E3 stack, d3_episode.run == e3_episode.run."""
import os, sys
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
from experiments.scenarios import build
from experiments import e3_episode as E3, d3_episode as D3

KEYS = ["outcome", "time_s", "progress_m", "min_clear", "path_len", "cycles", "watchdog", "contact_speed"]


def test_parity(layout=310, fam="S1", v=7.0, stack="opt_backup_t"):
    c = E3.default_cond(v_lim=v, latency_tail="heavy")
    a = E3.run(build(layout, fam), stack, c, layout, {})
    b = D3.run(build(layout, fam), stack, dict(c), layout, {})
    assert all(a[k] == b[k] for k in KEYS), [(k, a[k], b[k]) for k in KEYS if a[k] != b[k]]


if __name__ == "__main__":
    for st in ("opt_backup_t", "pas_full"):
        test_parity(stack=st)
        print("parity ok", st)
