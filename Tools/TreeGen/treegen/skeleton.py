# SPDX-License-Identifier: GPL-3.0-or-later
"""Flat-array (struct-of-arrays) tree skeleton, CSR children, chains, JSON I/O and hashing.

Internal frame: right-handed, metres, Z up, origin at the tree root (stem base).
Export frame ("unity"): x_u = x, y_u = z (up), z_u = y. Swapping y/z maps a right-handed Z-up frame
onto Unity's left-handed Y-up frame without mirroring.

Node i (i > 0) is the distal end of the internode parent[i] -> i. radius[i] is that internode's radius
(at node i); radius[0] is the stem base radius. Topological order holds: parent[i] < i.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field

import numpy as np

from . import GENERATOR_NAME, GENERATOR_VERSION, SCHEMA_VERSION


def to_unity(p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, dtype=np.float64)
    return p[..., [0, 2, 1]]


def from_unity(p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, dtype=np.float64)
    return p[..., [0, 2, 1]]


@dataclass
class Skeleton:
    pos: np.ndarray            # (N,3) float64, internal Z-up frame, tree-local
    parent: np.ndarray         # (N,) int32, -1 for root
    radius: np.ndarray         # (N,) float64 metres
    order: np.ndarray          # (N,) uint8 branch order (0 = stem)
    alive: np.ndarray          # (N,) bool
    is_main: np.ndarray        # (N,) bool: node continues its parent's axis
    birth: np.ndarray          # (N,) int16 season of creation
    dead_since: np.ndarray     # (N,) int16 season of death, -1 if alive
    light: np.ndarray          # (N,) float32 last light exposure Q of the subtree per metamer
    meta: dict = field(default_factory=dict)

    # ------------------------------------------------------------------ topology
    @property
    def n(self) -> int:
        return int(self.pos.shape[0])

    def children_csr(self):
        """Return (indptr, indices): children of i are indices[indptr[i]:indptr[i+1]] in id order."""
        n = self.n
        par = self.parent
        kids = np.nonzero(par >= 0)[0]
        p = par[kids]
        order = np.argsort(p, kind="stable")
        indices = kids[order].astype(np.int32)
        counts = np.bincount(p, minlength=n)
        indptr = np.zeros(n + 1, dtype=np.int64)
        np.cumsum(counts, out=indptr[1:])
        return indptr, indices

    def main_child(self) -> np.ndarray:
        mc = -np.ones(self.n, dtype=np.int64)
        idx = np.nonzero((self.parent >= 0) & self.is_main)[0]
        # if several (should not happen) the lowest id wins: assign in reverse
        mc[self.parent[idx[::-1]]] = idx[::-1]
        return mc

    def chains(self) -> list[np.ndarray]:
        """Maximal axis chains as node-id arrays. A chain starts at the root or a lateral node and
        includes its parent node as the first element (so tubes connect), except for the root chain."""
        mc = self.main_child()
        starts = np.nonzero((self.parent < 0) | (~self.is_main))[0]
        out = []
        for s in starts:
            ids = [] if self.parent[s] < 0 else [int(self.parent[s])]
            c = int(s)
            while c >= 0:
                ids.append(c)
                c = int(mc[c])
            out.append(np.asarray(ids, dtype=np.int64))
        return out

    def depth(self) -> np.ndarray:
        d = np.zeros(self.n, dtype=np.int64)
        par = self.parent
        for i in range(1, self.n):  # topological order
            d[i] = d[par[i]] + 1
        return d

    def tips(self) -> np.ndarray:
        has_child = np.zeros(self.n, dtype=bool)
        has_child[self.parent[self.parent >= 0]] = True
        return ~has_child

    def segment_arrays(self):
        """(a, b, ra, rb, child_id) for every internode parent->child."""
        kid = np.nonzero(self.parent >= 0)[0]
        p = self.parent[kid]
        return self.pos[p], self.pos[kid], self.radius[p], self.radius[kid], kid

    # ------------------------------------------------------------------ editing
    def subset(self, keep: np.ndarray) -> "Skeleton":
        """Keep nodes where keep is True. Kept nodes must have kept parents (or be roots)."""
        keep = np.asarray(keep, dtype=bool)
        newid = -np.ones(self.n, dtype=np.int64)
        newid[keep] = np.arange(int(keep.sum()))
        par = self.parent[keep]
        par = np.where(par >= 0, newid[np.maximum(par, 0)], -1).astype(np.int32)
        if np.any((self.parent[keep] >= 0) & (par < 0)):
            raise ValueError("subset would orphan nodes")
        return Skeleton(self.pos[keep].copy(), par, self.radius[keep].copy(), self.order[keep].copy(),
                        self.alive[keep].copy(), self.is_main[keep].copy(), self.birth[keep].copy(),
                        self.dead_since[keep].copy(), self.light[keep].copy(), dict(self.meta))

    def copy(self) -> "Skeleton":
        return self.subset(np.ones(self.n, dtype=bool))

    # ------------------------------------------------------------------ hashing / IO
    def topology_hash(self) -> str:
        """Golden hash: quantized positions (0.1 mm), radii (1 um), parent, order, alive."""
        h = hashlib.sha256()
        h.update(np.round(self.pos * 1e4).astype(np.int64).tobytes())
        h.update(np.round(self.radius * 1e6).astype(np.int64).tobytes())
        h.update(self.parent.astype(np.int32).tobytes())
        h.update(self.order.astype(np.uint8).tobytes())
        h.update(self.alive.astype(np.uint8).tobytes())
        return h.hexdigest()

    def to_json_dict(self) -> dict:
        pu = to_unity(self.pos)
        return {
            "schema": "crane.treegen.skeleton",
            "schema_version": SCHEMA_VERSION,
            "generator": GENERATOR_NAME,
            "generator_version": GENERATOR_VERSION,
            "frame": "unity_lh_yup_m_tree_local",
            **self.meta,
            "node_count": self.n,
            "skeleton_hash": self.topology_hash(),
            "nodes": {
                "pos": np.round(pu, 5).reshape(-1).tolist(),
                "parent": self.parent.astype(int).tolist(),
                "radius": np.round(self.radius, 6).tolist(),
                "order": self.order.astype(int).tolist(),
                "alive": self.alive.astype(int).tolist(),
                "is_main": self.is_main.astype(int).tolist(),
                "birth": self.birth.astype(int).tolist(),
                "dead_since": self.dead_since.astype(int).tolist(),
            },
        }

    def save_json(self, path) -> None:
        with open(path, "w") as f:
            json.dump(self.to_json_dict(), f, separators=(",", ":"))

    @staticmethod
    def from_json_dict(d: dict) -> "Skeleton":
        nd = d["nodes"]
        n = len(nd["parent"])
        pos = from_unity(np.asarray(nd["pos"], dtype=np.float64).reshape(n, 3))
        meta = {k: v for k, v in d.items() if k not in ("nodes", "node_count", "skeleton_hash", "schema",
                                                        "schema_version", "generator", "generator_version",
                                                        "frame")}
        return Skeleton(pos, np.asarray(nd["parent"], np.int32), np.asarray(nd["radius"], np.float64),
                        np.asarray(nd["order"], np.uint8), np.asarray(nd["alive"], bool),
                        np.asarray(nd["is_main"], bool), np.asarray(nd["birth"], np.int16),
                        np.asarray(nd["dead_since"], np.int16), np.zeros(n, np.float32), meta)

    @staticmethod
    def load_json(path) -> "Skeleton":
        with open(path) as f:
            return Skeleton.from_json_dict(json.load(f))


def params_hash(params: dict) -> str:
    blob = json.dumps(params, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()[:16]
