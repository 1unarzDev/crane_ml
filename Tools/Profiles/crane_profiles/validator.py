"""R1-R12 validator for crane.profile/v2 and crane.dataset_manifest/v2.

Rules (the rule id is attached to every error):
 R1  every Quantity has a numeric/array value (or null with ABSTAIN), a parseable unit, and a unit whose
     dimension matches the key-name suffix (massKg -> kg, tauUpS -> s ...)
 R2  non-scalar values (vectors, matrices) name a frame declared in `frames`
 R3  every source is declared with a licence; cited/derived quantities name a declared source and a locator
 R4  fitted quantities carry a CI (lo <= value <= hi) and name datasets declared in the profile
 R5  PHYSICALLY_CALIBRATED needs physical fit + held-out datasets with hashes, no synthetic-only data, no
     fixtureOnly coefficients, and every sim-truth-fitted coefficient listed in claimScope.excluded
 R6  sim-truth and physical data may only be mixed under an explicit `mixLabel`
 R7  fixtureOnly (state or coefficient) caps the evidence label at FIXTURE_VALIDATED
 R8  calibrationState and evidenceLabel are enumerated values (CONTRACTS.md labels only); a label above
     LIVE_SIM_VALIDATED needs state PHYSICALLY_CALIBRATED.  File names are never consulted.
 R9  identifiability status is enumerated; non-IDENTIFIED needs a reason; CONFOUNDED_WITH/COMBINATION_ONLY
     paths must resolve; ABSTAIN needs a reason and is the only status allowing a null value
 R10 contentHash must equal the canonical sha256
 R11 a source flagged `uncommitted` caps the label at IMPLEMENTED
 R12 fit and held-out data share no site, session or runId
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from . import canonical, model, units

M = model


@dataclass
class Report:
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    info: dict = field(default_factory=dict)

    def err(self, rule: str, path: str, msg: str) -> None:
        self.errors.append({"rule": rule, "path": path, "message": msg})

    def warn(self, rule: str, path: str, msg: str) -> None:
        self.warnings.append({"rule": rule, "path": path, "message": msg})

    @property
    def ok(self) -> bool:
        return not self.errors

    def rules(self) -> set:
        return {e["rule"] for e in self.errors}

    def to_dict(self) -> dict:
        return {"ok": self.ok, "errors": self.errors, "warnings": self.warnings, **self.info}


def _nonempty(x: Any) -> bool:
    return isinstance(x, str) and bool(x.strip())


def _norm_status(idf: dict) -> tuple:
    """Return (status, with_paths) accepting the 'CONFOUNDED_WITH:<path>' shorthand."""
    status = idf.get("status")
    paths = list(idf.get("with", []) or [])
    if isinstance(status, str) and ":" in status:
        head, tail = status.split(":", 1)
        status, paths = head, paths + [p for p in tail.split(",") if p]
    return status, paths


def _check_dataset_split(doc: dict, rep: Report, items: list, where: str) -> None:
    """R12: fit vs heldout must not share site, session or runId.  items: dicts with split/sites/sessions/runIds."""
    pools = {"fit": {"sites": set(), "sessions": set(), "runIds": set()},
             "heldout": {"sites": set(), "sessions": set(), "runIds": set()}}
    for it in items:
        pool = pools.get(it.get("split"))
        if pool is None:
            continue
        for key in pool:
            pool[key].update(x for x in (it.get(key) or []) if x is not None)
    for key in ("sites", "sessions", "runIds"):
        shared = pools["fit"][key] & pools["heldout"][key]
        if shared:
            rep.err("R12", where, f"fit and heldout share {key}: {sorted(map(str, shared))}")


def _check_common_envelope(doc: dict, rep: Report) -> None:
    if not _nonempty(doc.get("id")) and doc.get("schema") == M.PROFILE_SCHEMA:
        rep.err("R8", "id", "id must be a non-empty string")
    if doc.get(canonical.HASH_FIELD) is None:
        rep.err("R10", canonical.HASH_FIELD, "contentHash is missing")
    else:
        h = doc[canonical.HASH_FIELD]
        if not isinstance(h, str) or not M.SHA256_RE.match(h):
            rep.err("R10", canonical.HASH_FIELD, "contentHash must be lowercase sha256 hex")
        elif h != canonical.content_hash(doc):
            rep.err("R10", canonical.HASH_FIELD, "contentHash does not match canonical content (tampered or stale)")
    rep.info["contentHash"] = doc.get(canonical.HASH_FIELD)
    rep.info["computedHash"] = canonical.content_hash(doc)


def _check_sources(doc: dict, rep: Report) -> dict:
    sources = doc.get("sources", {})
    if not isinstance(sources, dict):
        rep.err("R3", "sources", "sources must be an object")
        return {}
    for sid, s in sources.items():
        if not isinstance(s, dict) or not _nonempty(s.get("license")):
            rep.err("R3", f"sources.{sid}", "source has no license")
    return sources


def validate_manifest(doc: dict) -> Report:
    rep = Report()
    rep.info["kind"] = "dataset_manifest"
    if doc.get("schema") != M.MANIFEST_SCHEMA:
        rep.err("R8", "schema", f"schema must be {M.MANIFEST_SCHEMA}")
    _check_common_envelope(doc, rep)
    if doc.get("origin") not in M.ORIGINS:
        rep.err("R8", "origin", f"origin must be one of {M.ORIGINS}")
    trials = doc.get("trials")
    if not isinstance(trials, list) or not trials:
        rep.err("R12", "trials", "trials must be a non-empty list")
        return rep
    ids = set()
    unverifiable = False
    for i, t in enumerate(trials):
        p = f"trials[{i}]"
        if not isinstance(t, dict):
            rep.err("R12", p, "trial must be an object")
            continue
        if t.get("id") in ids or not _nonempty(t.get("id")):
            rep.err("R12", p, f"missing or duplicate trial id {t.get('id')!r}")
        ids.add(t.get("id"))
        if t.get("split") not in ("fit", "heldout"):
            rep.err("R12", p, "split must be fit or heldout")
        for key in ("site", "session"):
            if t.get(key) is None:
                unverifiable = True
    pooled = [{"split": t.get("split"), "sites": [t.get("site")], "sessions": [t.get("session")],
               "runIds": [t.get("runId")]} for t in trials if isinstance(t, dict)]
    _check_dataset_split(doc, rep, pooled, "trials")
    for factor in doc.get("heldOutFactors", []) or []:
        fit_v = {str((t.get("factors") or {}).get(factor)) for t in trials if isinstance(t, dict) and t.get("split") == "fit"}
        held_v = {str((t.get("factors") or {}).get(factor)) for t in trials if isinstance(t, dict) and t.get("split") == "heldout"}
        if fit_v & held_v:
            rep.err("R12", f"heldOutFactors.{factor}", f"heldout {factor} values overlap fit split: {sorted(fit_v & held_v)}")
    splits = {t.get("split") for t in trials if isinstance(t, dict)}
    if splits != {"fit", "heldout"}:
        rep.err("R12", "trials", "manifest needs both fit and heldout trials")
    rep.info["independence"] = "UNVERIFIED" if unverifiable else ("VERIFIED" if rep.ok else "VIOLATED")
    if unverifiable:
        rep.warn("R12", "trials", "site/session missing on some trials: split independence cannot be verified")
    return rep


def validate(doc: Any) -> Report:
    """Validate a parsed v2 document.  Never reads the file name; labels come from the document only."""
    rep = Report()
    if not isinstance(doc, dict):
        rep.err("R8", "", "document must be an object")
        return rep
    if doc.get("schema") == M.MANIFEST_SCHEMA:
        return validate_manifest(doc)
    if doc.get("schema") != M.PROFILE_SCHEMA:
        rep.err("R8", "schema", f"schema must be {M.PROFILE_SCHEMA}")
        return rep
    for key in ("kind", "domain", "version"):
        if not _nonempty(doc.get(key)):
            rep.err("R8", key, f"{key} must be a non-empty string")
    frames = doc.get("frames")
    if not isinstance(frames, dict) or not frames:
        rep.err("R2", "frames", "frames must declare at least one frame")
        frames = {}
    _check_common_envelope(doc, rep)

    state, label = doc.get("calibrationState"), doc.get("evidenceLabel")
    if state not in M.STATES:
        rep.err("R8", "calibrationState", f"calibrationState must be one of {M.STATES}")
    if label not in M.LABELS:
        rep.err("R8", "evidenceLabel", f"evidenceLabel {label!r} is not a CONTRACTS.md label")
    rep.info.update({"calibrationState": state, "evidenceLabel": label})

    sources = _check_sources(doc, rep)

    # datasets
    datasets = doc.get("datasets", [])
    ds_by_id: dict = {}
    if not isinstance(datasets, list):
        rep.err("R5", "datasets", "datasets must be a list")
        datasets = []
    for i, d in enumerate(datasets):
        p = f"datasets[{i}]"
        if not isinstance(d, dict) or not _nonempty(d.get("id")):
            rep.err("R5", p, "dataset needs an id")
            continue
        if d["id"] in ds_by_id:
            rep.err("R5", p, f"duplicate dataset id {d['id']}")
        ds_by_id[d["id"]] = d
        if d.get("origin") not in M.ORIGINS:
            rep.err("R5", p, f"dataset origin must be one of {M.ORIGINS}")
        if d.get("split", "none") not in M.SPLITS:
            rep.err("R5", p, f"dataset split must be one of {M.SPLITS}")
        if d.get("sha256") is not None and not M.SHA256_RE.match(str(d["sha256"])):
            rep.err("R5", p, "dataset sha256 must be lowercase sha256 hex")
    for i, c in enumerate(doc.get("components", []) or []):
        cid = c.get("id") if isinstance(c, dict) else c
        if not isinstance(cid, str) or not M.COMPONENT_RE.match(cid):
            rep.err("R10", f"components[{i}]", "component must be id@sha256")
    _check_dataset_split(doc, rep, datasets, "datasets")
    origins = {d.get("origin") for d in ds_by_id.values()}
    if "sim_truth" in origins and "physical" in origins and not _nonempty(doc.get("mixLabel")):
        rep.err("R6", "mixLabel", "sim_truth and physical data are mixed without an explicit mixLabel")

    # quantities
    qpaths = dict(M.iter_quantities(doc))
    uncommitted = [sid for sid, s in sources.items() if isinstance(s, dict) and s.get("uncommitted")]
    fixture_paths, simtruth_paths = [], []
    for path, q in qpaths.items():
        leaf = path.rsplit(".", 1)[-1]
        abstain = isinstance((q.get("identifiability") or {}), dict) and (q.get("identifiability") or {}).get("status") == "ABSTAIN"
        # R1
        unit = None
        if "unit" not in q:
            rep.err("R1", path, "missing unit")
        else:
            try:
                unit = units.parse_unit(q["unit"])
            except units.UnitError as exc:
                rep.err("R1", path, f"bad unit: {exc}")
        if unit is not None:
            exp = units.expected_dimension(leaf)
            if exp is not None and not exp.same_dimension(unit):
                rep.err("R1", path, f"unit {q['unit']!r} ({unit.dimension_name()}) does not match key suffix "
                                    f"{units.key_suffix(leaf)!r} ({exp.dimension_name()})")
        value = q["value"]
        if value is None:
            if not abstain:
                rep.err("R1", path, "null value is only allowed with identifiability.status ABSTAIN")
        else:
            nums = M.flatten_numbers(value)
            if not nums or not all(M.is_number(x) for x in nums):
                rep.err("R1", path, "value must be finite numbers (scalar or nested list)")
            # R2
            if isinstance(value, (list, tuple)):
                fr = q.get("frame")
                if not _nonempty(fr):
                    rep.err("R2", path, "vector/matrix value has no frame")
                elif fr not in frames:
                    rep.err("R2", path, f"frame {fr!r} is not declared in frames")
        # provenance
        prov = q.get("provenance")
        if prov not in M.PROVENANCE:
            rep.err("R3", path, f"provenance must be one of {M.PROVENANCE}")
        src = q.get("source")
        if src is not None and src not in sources:
            rep.err("R3", path, f"unknown source {src!r}")
        if prov == "cited" and src is None:
            rep.err("R3", path, "cited quantity has no source")
        if prov == "cited" and not _nonempty(q.get("locator")):
            rep.err("R3", path, "cited quantity has no locator")
        # R4
        if prov == "fitted":
            _check_fitted(q, path, ds_by_id, rep)
            if any((ds_by_id.get(i) or {}).get("origin") in ("sim_truth", "synthetic-fixture") for i in q.get("datasets", [])):
                simtruth_paths.append(path)
        if q.get("simTruth"):
            simtruth_paths.append(path)
        if prov == "fixtureOnly":
            fixture_paths.append(path)
        # R9
        idf = q.get("identifiability")
        if idf is None:
            if prov == "fitted":
                rep.err("R9", path, "fitted quantity lacks identifiability")
        else:
            _check_ident(idf, path, doc, rep)
        for j, alt in enumerate(q.get("alternatives", []) or []):
            if not isinstance(alt, dict) or "value" not in alt:
                rep.err("R1", f"{path}.alternatives[{j}]", "alternative needs a value")
                continue
            if "unit" in alt and not units.is_unit(alt["unit"]):
                rep.err("R1", f"{path}.alternatives[{j}]", "bad unit")
            if alt.get("source") is not None and alt["source"] not in sources:
                rep.err("R3", f"{path}.alternatives[{j}]", f"unknown source {alt['source']!r}")
    for sname, s in (doc.get("structures") or {}).items():
        sp = f"structures.{sname}"
        if not isinstance(s, dict) or s.get("frame") not in frames:
            rep.err("R2", sp, "structure needs a declared frame")
        elif s.get("source") is not None and s["source"] not in sources:
            rep.err("R3", sp, f"unknown source {s['source']!r}")

    # claimScope.excluded
    excluded = list((doc.get("claimScope") or {}).get("excluded", []) or [])
    for ex in excluded:
        if ex not in qpaths:
            rep.err("R5", "claimScope.excluded", f"excluded path {ex!r} does not exist")

    # label ceiling
    cap, why = M.LABEL_RANK["PHYSICALLY_CALIBRATED"], "none"
    def lower(rank: int, reason: str) -> None:
        nonlocal cap, why
        if rank < cap:
            cap, why = rank, reason
    if state != "PHYSICALLY_CALIBRATED":
        lower(M.LABEL_RANK["LIVE_SIM_VALIDATED"], "R8: label above LIVE_SIM_VALIDATED needs state PHYSICALLY_CALIBRATED")
    if state == "FIXTURE_ONLY" or fixture_paths:
        lower(M.LABEL_RANK["FIXTURE_VALIDATED"], "R7: fixtureOnly content")
    if uncommitted:
        lower(M.LABEL_RANK["IMPLEMENTED"], "R11: sources uncommitted user work " + ",".join(sorted(uncommitted)))
    rep.info["labelCeiling"] = [k for k, v in M.LABEL_RANK.items() if v == cap][-1]
    rep.info["labelCeilingReason"] = why
    if label in M.LABEL_RANK and M.LABEL_RANK[label] > cap:
        rule = why.split(":")[0] if ":" in why else "R8"
        rep.err(rule, "evidenceLabel", f"evidenceLabel {label} exceeds ceiling {rep.info['labelCeiling']} ({why})")
    if label == "PHYSICALLY_CALIBRATED" and state != "PHYSICALLY_CALIBRATED":
        rep.err("R8", "evidenceLabel", "PHYSICALLY_CALIBRATED label requires calibrationState PHYSICALLY_CALIBRATED")

    # R5
    if state == "PHYSICALLY_CALIBRATED":
        phys = [d for d in ds_by_id.values() if d.get("origin") == "physical"]
        fit = [d for d in phys if d.get("split") == "fit" and d.get("sha256")]
        held = [d for d in phys if d.get("split") == "heldout" and d.get("sha256")]
        if not phys:
            rep.err("R5", "datasets", "PHYSICALLY_CALIBRATED requires a physical dataset (none declared)")
        elif not fit:
            rep.err("R5", "datasets", "PHYSICALLY_CALIBRATED requires a physical fit dataset with sha256")
        if phys and not held:
            rep.err("R5", "datasets", "PHYSICALLY_CALIBRATED requires a physical held-out dataset with sha256")
        for d in held:
            if not (d.get("sites") or d.get("sessions")):
                rep.err("R5", f"datasets.{d['id']}", "held-out dataset must identify sites or sessions")
        if origins and origins <= {"synthetic-fixture", "sim_truth"}:
            rep.err("R5", "datasets", "PHYSICALLY_CALIBRATED cannot rest on synthetic or sim-truth data only")
        for p in sorted(set(simtruth_paths) - set(excluded)):
            rep.err("R5", p, "sim-truth coefficient is not listed in claimScope.excluded")
        for p in fixture_paths:
            rep.err("R7", p, "fixtureOnly coefficient under PHYSICALLY_CALIBRATED")
    return rep


def _check_fitted(q: dict, path: str, ds_by_id: dict, rep: Report) -> None:
    u = q.get("uncertainty")
    if not isinstance(u, dict) or u.get("kind") != "ci":
        rep.err("R4", path, "fitted coefficient has no CI uncertainty (kind 'ci' with level, lo, hi)")
    else:
        lvl = u.get("level")
        if not (M.is_number(lvl) and 0 < lvl < 1):
            rep.err("R4", path, "CI level must be in (0,1)")
        lo, hi, v = (M.flatten_numbers(u.get("lo")), M.flatten_numbers(u.get("hi")), M.flatten_numbers(q.get("value")))
        if not (len(lo) == len(hi) == len(v)) or not all(M.is_number(x) for x in lo + hi):
            rep.err("R4", path, "CI lo/hi must match the value shape")
        elif any(not (a <= x <= b) for a, x, b in zip(lo, v, hi)):
            rep.err("R4", path, "value lies outside its CI")
    dsets = q.get("datasets")
    if not dsets or not isinstance(dsets, list):
        rep.err("R4", path, "fitted coefficient names no datasets")
    else:
        for i in dsets:
            if i not in ds_by_id:
                rep.err("R4", path, f"fitted coefficient references undeclared dataset {i!r}")


def _check_ident(idf: Any, path: str, doc: dict, rep: Report) -> None:
    if not isinstance(idf, dict):
        rep.err("R9", path, "identifiability must be an object")
        return
    status, with_paths = _norm_status(idf)
    if status not in M.ID_STATUSES:
        rep.err("R9", path, f"identifiability status {status!r} not in {M.ID_STATUSES}")
        return
    if status != "IDENTIFIED" and not _nonempty(idf.get("reason")):
        rep.err("R9", path, f"{status} requires a reason")
    if status in M.WITH_STATUSES:
        if not with_paths:
            rep.err("R9", path, f"{status} requires at least one path in 'with'")
        for wp in with_paths:
            if not M.is_quantity(M.resolve_path(doc, wp)):
                rep.err("R9", path, f"{status} points at missing path {wp!r}")


def validate_file(path) -> Report:
    import json
    from pathlib import Path
    try:
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        rep = Report()
        rep.err("R8", str(path), f"cannot read document: {exc}")
        return rep
    return validate(doc)
