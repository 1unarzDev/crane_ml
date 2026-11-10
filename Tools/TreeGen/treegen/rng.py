# SPDX-License-Identifier: GPL-3.0-or-later
"""Deterministic per-tree RNG streams.

Every random draw goes through a Generator built from SeedSequence([forest_seed, tree_id, stream]).
Separate streams mean, e.g., changing the leaf card count never perturbs branch topology.
No global numpy RNG is used anywhere in TreeGen.
"""
import numpy as np

STREAMS = {
    "growth": 0,    # bud fates, tropism noise, phyllotaxis jitter
    "layout": 1,    # stand positions / species draws (tree_id = -1 slot uses 0xFFFF)
    "leaves": 2,    # leaf card placement
    "twigs": 3,     # twig sprays / soft detail
    "variant": 4,   # snag / log post-processing
    "markers": 5,   # space-colonization attractors
    "mesh": 6,      # bark noise in meshing
}


def stream(forest_seed: int, tree_id: int, name: str) -> np.random.Generator:
    if name not in STREAMS:
        raise KeyError(f"unknown RNG stream {name!r}")
    tid = int(tree_id) & 0xFFFFFFFF
    ss = np.random.SeedSequence([int(forest_seed) & 0xFFFFFFFF, tid, STREAMS[name]])
    return np.random.Generator(np.random.PCG64(ss))
