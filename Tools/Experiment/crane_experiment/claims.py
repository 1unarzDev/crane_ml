"""Claim linter: citations, evidence-label ceiling, banned claims.

A claim is {"text": str, "label": <evidence label>}. Citations are inline tokens
`ev:<12 hex>` (ledger entry) or `prof:<id@sha>` (profile). Every sentence containing a
number must carry a valid citation in the same sentence.
"""
import re

LABELS = ["NOT_RUN", "UNKNOWN", "IMPLEMENTED", "FIXTURE_VALIDATED", "LIVE_SIM_VALIDATED",
          "PHYSICALLY_CALIBRATED"]
RANK = {l: i for i, l in enumerate(LABELS)}
PROFILE_CEILING = {"FIXTURE_ONLY": "FIXTURE_VALIDATED", "UNCALIBRATED_PRIOR": "IMPLEMENTED",
                   "SIM_TRUTH_RECOVERY": "LIVE_SIM_VALIDATED",
                   "PHYSICALLY_CALIBRATED": "PHYSICALLY_CALIBRATED"}

EV_RE = re.compile(r"\bev:[0-9a-f]{12}\b")
PROF_RE = re.compile(r"\bprof:[A-Za-z0-9_.\-/]+@[0-9a-fA-F]+\b|\bprof:[A-Za-z0-9_.\-/]+@[A-Za-z0-9]+\b")
NUM_RE = re.compile(r"(?<![A-Za-z0-9_.:@/-])[-+]?\d+(?:\.\d+)?%?(?![A-Za-z0-9_@])")

# (regex, message, exempt_if_label)
BANNED = [
    (re.compile(r"\bprove[sd]?\b|\bproof\b", re.I), "proof language", None),
    (re.compile(r"\bstate[- ]of[- ]the[- ]art\b", re.I), "state-of-the-art claim", None),
    (re.compile(r"\bguarantee[sd]?\b", re.I), "guarantee language", None),
    (re.compile(r"sim[- ]?to[- ]?real (gap|transfer) (is )?(closed|solved|eliminated)", re.I),
     "sim-to-real gap closed", None),
    (re.compile(r"\bsafe (for|to) (deploy|deployment|hardware|field)", re.I), "deployment-safety claim", None),
    (re.compile(r"\bphysically[- ]calibrated\b", re.I), "physical calibration claim", "PHYSICALLY_CALIBRATED"),
    (re.compile(r"\b(validated on (real )?hardware|real[- ]world (validated|validation|results?))\b", re.I),
     "real-world validation claim", "PHYSICALLY_CALIBRATED"),
    (re.compile(r"\b(null result|falsified|confirmed)\b.*\bP[1-7]\b|\bP[1-7]\b.*\b(confirmed|supported|falsified)\b", re.I),
     "preregistered prediction outcome claimed while NOT_RUN", "PHYSICALLY_CALIBRATED"),
]


def _sentences(text):
    return [s for s in re.split(r"(?<=[.!?])\s+", text.strip()) if s]


def check_claims(claims, evidence, profiles):
    """evidence: {ev_id: ceiling_label}; profiles: {id@sha: calibrationState}. Returns violations."""
    out = []
    for i, cl in enumerate(claims):
        text, label = cl.get("text", ""), cl.get("label")

        def v(code, msg):
            out.append({"claim": i, "code": code, "message": msg})

        if label not in RANK:
            v("BAD_LABEL", f"label {label!r} is not one of {LABELS}")
            continue
        ceilings = []
        for ev in EV_RE.findall(text):
            if ev not in evidence:
                v("UNKNOWN_CITATION", f"{ev} is not in the ledger")
            else:
                ceilings.append(evidence[ev])
        for pr in PROF_RE.findall(text):
            pid = pr[len("prof:"):]
            if pid not in profiles:
                v("UNKNOWN_CITATION", f"{pr} is not a known profile")
            else:
                ceilings.append(PROFILE_CEILING.get(profiles[pid], "UNKNOWN"))
        for s in _sentences(text):
            bare = PROF_RE.sub(" ", EV_RE.sub(" ", s))
            if NUM_RE.search(bare) and not (EV_RE.search(s) or PROF_RE.search(s)):
                v("UNCITED_NUMBER", f"number without citation in: {s!r}")
        ceiling = min(ceilings, key=lambda l: RANK[l]) if ceilings else "UNKNOWN"
        if RANK[label] > RANK[ceiling]:
            v("LABEL_EXCEEDS_CEILING", f"label {label} exceeds ceiling {ceiling} of cited evidence")
        for rx, msg, exempt in BANNED:
            if rx.search(text) and not (exempt and label == exempt and ceiling == exempt):
                v("BANNED_CLAIM", msg)
    return out
