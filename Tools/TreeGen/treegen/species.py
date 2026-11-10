# SPDX-License-Identifier: GPL-3.0-or-later
"""Species presets (JSON in Tools/TreeGen/presets). Unknown keys are kept; missing keys get defaults."""
from __future__ import annotations

import json
from pathlib import Path

PRESET_DIR = Path(__file__).resolve().parent.parent / "presets"

DEFAULTS = {
    "kind": "broadleaf",
    "seasons": 30,
    "environment": "shadow",
    "alpha": 2.0,
    "lambda": 0.52,
    "order_vigor_decay": 1.0,
    "max_metamers": 4,
    "internode_length": 0.35,
    "internode_order_decay": 0.9,
    "internode_jitter": 0.1,
    "phyllotaxis": "spiral",
    "divergence_deg": 137.5,
    "buds_per_node": 1,
    "max_order": 5,
    "branch_angle_deg_by_order": [0, 50, 45, 40],
    "branch_angle_jitter_deg": 8.0,
    "w_prev": 1.0,
    "w_light": 0.25,
    "w_grav_by_order": [0.35, 0.05, 0.0],
    "w_horiz_by_order": [0.0, 0.1, 0.1],
    "w_noise": 0.06,
    "p_abort": 0.05,
    "p_terminal_abort": 0.0,
    "dormancy_seasons": 3,
    "p_regen": 0.0,
    "regen_light": 0.7,
    "shed_threshold": 0.12,
    "shed_grace": 3,
    "p_stub": 0.5,
    "stub_nodes": 2,
    "stub_decay_seasons": 8,
    "n_pipe": 2.2,
    "r_tip": 0.004,
    "height_max": 25.0,
    "height_exponent": 3.0,
    "crown_radius_max": 8.0,
    "flare": 0.4,
    "flare_height": 0.5,
    "leaf": {"card_size": 0.22, "cards_per_node": 2, "max_age": 2, "max_radius": 0.012,
             "atlas_index": 0, "aspect": 1.0},
    "bark_tile_m": 0.5,
    "bark_color": [0.35, 0.3, 0.26],
    "leaf_color": [0.25, 0.42, 0.15],
    "targets": {},
}


def _merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def list_presets() -> list[str]:
    return sorted(p.stem for p in PRESET_DIR.glob("*.json"))


def load_species(name_or_path, overrides: dict | None = None) -> dict:
    p = Path(name_or_path)
    if not p.suffix:
        p = PRESET_DIR / f"{name_or_path}.json"
    with open(p) as f:
        d = json.load(f)
    if "base" in d:   # variant presets (snag, log) inherit a growth species
        base = load_species(d["base"])
        d = _merge(base, {k: v for k, v in d.items() if k != "base"})
    sp = _merge(DEFAULTS, d)
    if overrides:
        sp = _merge(sp, overrides)
    return sp
