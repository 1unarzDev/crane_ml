"""SeedSequence-based seeding with common random numbers (CRN).

A trial's world noise depends only on (root seed, context id, replicate k), never on
the config, optimizer or fidelity, so any two configs on the same (context, k) see
the same noise draw.
"""
import hashlib

import numpy as np


def stable_int(text: str) -> int:
    return int.from_bytes(hashlib.sha256(text.encode()).digest()[:8], "big")


def crn_seed(root: int, context_id: str, k: int) -> int:
    ss = np.random.SeedSequence([int(root), stable_int("ctx:" + context_id), int(k)])
    return int(ss.generate_state(1, np.uint64)[0])


def optimizer_seed(root: int, name: str) -> int:
    ss = np.random.SeedSequence([int(root), stable_int("opt:" + name)])
    return int(ss.generate_state(1, np.uint64)[0] >> np.uint64(1))
