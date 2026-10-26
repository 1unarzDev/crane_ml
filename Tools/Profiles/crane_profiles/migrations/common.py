from __future__ import annotations

import re
from typing import Any

from .. import canonical, model

LEGACY_LABELS = {"PUBLISHED_DESCRIPTION_PRIOR": "IMPLEMENTED"}


def Q(value: Any, unit: str, *, prov: str, source: str | None = None, locator: str | None = None,
      frame: str | None = None, ident: dict | None = None, alternatives: list | None = None,
      legacy: dict | None = None) -> dict:
    q: dict = {"value": value, "unit": unit, "provenance": prov}
    for k, v in (("source", source), ("locator", locator), ("frame", frame), ("identifiability", ident),
                 ("alternatives", alternatives), ("legacy", legacy)):
        if v is not None:
            q[k] = v
    return q


def no_data(reason: str = "Literature/engineering prior; nothing identified on CRANE data or a CRANE system.") -> dict:
    return {"status": "NO_PHYSICAL_DATA", "reason": reason}


def set_path(root: dict, dotted: str, node: Any) -> None:
    parts = dotted.split(".")
    cur = root
    for p in parts[:-1]:
        cur = cur.setdefault(p, {})
    cur[parts[-1]] = node


def get_path(root: dict, dotted: str) -> Any:
    cur: Any = root
    for p in dotted.split("."):
        cur = cur[p]
    return cur


def is_uncommitted(*texts: Any) -> bool:
    return any(re.search(r"uncommitted|working tree|\bWIP\b", str(t), re.I) for t in texts if t)


def normalize_label(raw: str, *, uncommitted: bool = False) -> tuple[str, str | None]:
    """Map a legacy free-form evidence label to a CONTRACTS label; returns (label, downgrade_note)."""
    head = model.label_of(raw)
    label = LEGACY_LABELS.get(head, head)
    if label not in model.LABEL_RANK:
        raise ValueError(f"cannot map legacy evidence label {raw!r}")
    note = None
    if label != head:
        note = f"legacy label {head} mapped to {label}"
    if label == "PHYSICALLY_CALIBRATED":
        label, note = "LIVE_SIM_VALIDATED", "PHYSICALLY_CALIBRATED cannot be claimed by migration"
    if uncommitted and model.LABEL_RANK[label] > model.LABEL_RANK["IMPLEMENTED"]:
        note = f"{label} capped to IMPLEMENTED (R11: sources uncommitted user work)"
        label = "IMPLEMENTED"
    return label, note


def finish(doc: dict) -> dict:
    return canonical.seal(doc)
