"""Library v3: 16 controllers drawn AT RANDOM (fixed seed) from the ranges library v2 spans.

Purpose (REVIEW_m1_fresh, objection to the channel x regime map): library v2 was hand-placed on
development boats with knowledge of the strata, so regime dependence could be designed in. v3 has no
hand placement: every member is an independent draw; nothing was evaluated before freezing it.
"""

import numpy as np

from library_v2 import ControllerSpecV2

SEED = 31
DOCKS = ('const', 'sqrt', 'two_stage', 'sqrt@0.3', 'sqrt@0.6')


def _library(seed=SEED, n=16):
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        out.append(ControllerSpecV2(
            idx=i, kp_scale=float(rng.choice([0.5, 1.0, 1.5])), ki_scale=float(rng.choice([0.5, 1.0, 1.5])),
            cruise=round(float(rng.uniform(0.8, 1.5)), 2), turn_alat=float(rng.choice([0.12, 0.25, 0.5])),
            brake=round(float(np.exp(rng.uniform(np.log(0.2), np.log(2.0)))), 2), dock=str(rng.choice(DOCKS)),
            latcomp=float(rng.choice([0.0, 0.1, 0.2])), feedforward=bool(rng.uniform() < 0.875)))
    return tuple(out)


LIBRARY_V3 = _library()
