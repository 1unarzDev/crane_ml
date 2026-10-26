"""Adapter: an mhseals_testing ``mhseals.physics/1`` survey record (characterization-v2) to the gate's
four inputs (yaml, provenance, run, snapshot). import_yaml.py calls it for such a record.

    <root>/characterization/<fit>-survey.yaml          the record (mhseals_testing physics_record.py)
    <root>/characterization/<fit>-survey.samples.json  odometry [[t, vx, vy, wz, yaw]] and segments
    <root>/config-history/{changes.jsonl, versions/hardware/<sha>.json}

Mapping (review follow-up): samples_file.sha256 is verified over the file bytes (a mismatch raises);
each survey hold segment becomes a gate hold whose direction and amplitude come from its axis, sign
and ``applied`` main-axis effort, whose ``override`` is ``overridden``, and whose odometry is sliced
from the preceding coast (the baseline) to the hold end; coasts become coastdowns, ``neutral``
segments neutral windows; wave_floor {vx, vy, wz} becomes [wz, vx, vy]; hardware.hash and
keys.hardware_epoch go to the provenance with sim_derived (SIM_DERIVED in the gate); the snapshot is
read from config-history by hardware.history.id, never from the record's own keys. The record has
no t95, so M_eff stays at its prior (UNDERDETERMINED).
"""
from __future__ import annotations

import hashlib
import json
import os

import numpy as np

from gate import AXES, TOLERANCE  # gate never imports this module

SCHEMA = "mhseals.physics/1"


def is_record(doc) -> bool:
    return isinstance(doc, dict) and doc.get("schema") == SCHEMA


def snapshot_by_id(history_dir, history):
    """The config-history hardware version that ``history.id`` names, or None when it can't be found."""
    if not history or not history_dir:
        return None
    try:
        with open(os.path.join(history_dir, "changes.jsonl")) as f:
            change = next((json.loads(x) for x in f if x.strip() and json.loads(x).get("id") == history["id"]), None)
        if not change or change.get("subsystem", "hardware") != "hardware":
            return None
        with open(os.path.join(history_dir, "versions", "hardware", f"{change['after']}.json")) as f:
            return json.load(f)
    except (OSError, ValueError, KeyError):
        return None


def adapt(record_path, history_dir=None):
    """(yaml_doc, provenance, run, snapshot) for gate.run_gate from a survey record on disk."""
    import yaml  # the gate's only YAML user, as in import_yaml.py
    with open(record_path) as f:
        rec = yaml.safe_load(f)
    if not is_record(rec) or rec.get("step") != "survey":
        raise ValueError(f"{record_path}: not an {SCHEMA} survey record")
    folder = os.path.dirname(os.path.abspath(record_path))
    history_dir = history_dir or os.path.join(os.path.dirname(folder), "config-history")
    samples = {"segments": [], "odometry": []}
    sf = rec.get("samples_file")
    if sf:
        with open(os.path.join(folder, sf["path"]), "rb") as f:
            raw = f.read()
        got = "sha256:" + hashlib.sha256(raw).hexdigest()
        if got != sf.get("sha256"):
            raise ValueError(f"SAMPLES_SHA_MISMATCH {sf['path']}: recorded {sf.get('sha256')}, file {got}")
        samples = json.loads(raw)
    odo = np.array([[np.nan if x is None else x for x in r] for r in samples["odometry"]], float).reshape(-1, 5)

    def cut(t0, t1):
        rows = odo[(odo[:, 0] >= t0) & (odo[:, 0] <= t1) & np.isfinite(odo[:, 1:4]).all(axis=1)]
        return {k: rows[:, i].tolist() for i, k in enumerate(("t", "vx", "vy", "wz", "yaw"))}

    segs = samples["segments"]
    holds, coasts, neutral = [], [], []
    for i, s in enumerate(segs):
        prev = segs[i - 1] if i else None
        if "axis" in s:
            axis, sign = int(s["axis"]), int(s["sign"])
            command = [0.0, 0.0, 0.0]
            command[axis] = sign * abs(float(s["applied"][axis]))
            settle = s["start"] - prev["start"] if prev and prev["label"] in ("coast", "neutral") else 0.0
            holds.append({"label": s["label"], "start": s["start"], "end": s["end"], "command": command,
                          "settle": settle, "override": bool(s.get("overridden")),
                          "samples": cut(s["start"] - settle, s["end"])})
        elif s["label"] == "coast" and prev and "axis" in prev:
            coasts.append({"axis": AXES[prev["axis"]], "sign": prev["sign"], "start": s["start"], "end": s["end"],
                           "samples": cut(s["start"], s["end"])})
        elif s["label"] == "neutral":
            neutral.append({"start": s["start"], "end": s["end"], "samples": cut(s["start"], s["end"])})
    hold_s = np.median([h["end"] - h["start"] for h in holds]) if holds else 0.0
    settle_s = np.median([h["settle"] for h in holds]) if holds else 0.0
    wf = rec.get("wave_floor") or {}
    wave_floor = [wf.get(k) for k in ("wz", "vx", "vy")]
    char = {"version": 2, "source": f"{SCHEMA} {os.path.abspath(record_path)}",
            "identified_at": rec["fit"].get("time"), "sample_count": len(odo),
            "speed_curves": rec.get("speed_curves") or {},
            "wave_floor": wave_floor if None not in wave_floor else None}
    hw = rec.get("hardware") or {}
    history = hw.get("history")
    entry = {"fit": dict(rec["fit"], config=(history or {}).get("id")),
             "hardware": {"hash": hw.get("hash"), "epoch": (hw.get("keys") or {}).get("hardware_epoch"),
                          "sim_derived": bool(hw.get("sim_derived"))},
             "gate": {"muted": bool(rec.get("gate_muted"))}, "battery_v": rec.get("battery_v"),
             "conditions": rec.get("conditions")}
    provenance = {"schema": "mhseals.calibration/1", "software": rec.get("software") or {},
                  "directions": {a + s: entry for a in AXES for s in "+-"}}
    run = {"procedure": SCHEMA, "holds": holds, "coastdowns": coasts, "neutral": neutral,
           "conditions": rec.get("conditions"), "holds_rejected": [], "directions": cross_responses(rec),
           # the producer's steady window is the last 0.4 of a hold; the baseline the coast's second half
           "settings": {"transient_s": 0.6 * float(hold_s), "coast_s": float(settle_s), "settle_s": float(settle_s)}}
    snap = snapshot_by_id(history_dir, history)
    if snap is not None and history.get("sha") and snap.get("sha") != history["sha"]:
        snap = None  # the id names a different version than the record saw: no fit-time snapshot
    return {"characterization": char}, provenance, run, snap


def cross_responses(rec):
    """Cross/main hold-mean ratios at the highest and lowest level per direction. The survey runs the
    fit-time C+/- without iterating it, so 'converged'/'low_ok' say only whether that C leaves the
    cross responses inside the characterize.yaml tolerance."""
    out = {}
    for axis_i, axis in enumerate(AXES):
        for sign in (1, -1):
            name = axis + ("+" if sign > 0 else "-")
            mine = [h for h in rec.get("holds") or [] if h["name"].split()[0] == name and not h.get("overridden")
                    and None not in (h.get("vx"), h.get("vy"), h.get("wz"))]
            if not mine:
                out[name] = None
                continue
            levels = sorted({abs(h["applied"][axis_i]) for h in mine})

            def ratios(level):
                v = np.mean([[h["vx"], h["vy"], h["wz"]] for h in mine if abs(h["applied"][axis_i]) == level], axis=0)
                main = max(sign * v[axis_i], 1e-6)
                return [float(v[k] / main) for k in range(3) if k != axis_i]
            hi, lo, tol = ratios(levels[-1]), ratios(levels[0]), TOLERANCE[axis]
            out[name] = {"response": hi, "low_response": lo, "source": "survey hold means (no C+/- iteration)",
                         "converged": all(abs(r) <= t for r, t in zip(hi, tol)),
                         "low_ok": all(abs(r) <= t for r, t in zip(lo, tol))}
    return out
