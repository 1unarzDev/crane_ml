"""CRANE Nav2 binding STUB. It never launches anything.

It only parses artifact formats that CRANE already writes:
  * navigation-reset-summary.json  (Tools/Performance/summarize_nav2_reset.py)
  * crane-benchmark-v1 counters    (CraneBenchmarkRunner.cs)
  * inspect_replay.py JSON output  (runtimeMetadata.calibrationProfileSha256)
Execution is NOT_RUN until PREREGISTRATION.md is frozen, and is not implemented here
even then. The isolated ROS domain must be 200-228 (never 42/142).
"""
import json
import os
import re

from .errors import NotRun, StudyError

ROS_DOMAIN_MIN, ROS_DOMAIN_MAX = 200, 228
FORBIDDEN_DOMAINS = (42, 142)
RESET_SCHEMAS = {"crane-nav2-scene-reload-v1", "crane-nav2-worker-fixture-v1",
                 "crane-nav2-external-player-fixture-v1"}
BENCHMARK_SCHEMA = "crane-benchmark-v1"
COUNTER_KEYS = ("staleObservations", "staleActions", "acceptedActions", "rejectedActions",
                "unknownSourceActions", "crossEpisodeActions", "duplicateActions",
                "failedObservations", "loggedErrors", "loggedExceptions")
PREREG = os.path.join(os.path.dirname(__file__), "..", "..", "..", "Docs", "ResearchOrchestration",
                      "PREREGISTRATION.md")


def validate_ros_domain(domain):
    if domain in FORBIDDEN_DOMAINS:
        raise StudyError(f"ROS domain {domain} is reserved and forbidden")
    if not isinstance(domain, int) or not ROS_DOMAIN_MIN <= domain <= ROS_DOMAIN_MAX:
        raise StudyError(f"isolated ROS domain must be {ROS_DOMAIN_MIN}-{ROS_DOMAIN_MAX}, got {domain!r}")
    return domain


def isolated_domain(worker_id):
    return validate_ros_domain(ROS_DOMAIN_MIN + int(worker_id) % (ROS_DOMAIN_MAX - ROS_DOMAIN_MIN + 1))


def preregistration_frozen(path=None):
    path = path or PREREG
    try:
        with open(path, encoding="utf-8") as fh:
            head = fh.read(400)
    except OSError:
        return False
    return "DRAFT" not in head.split("\n", 1)[0] and "FROZEN" in head


def _load(path, what):
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError) as exc:
        raise StudyError(f"cannot parse {what} {path}: {exc}")


def parse_reset_summary(path):
    d = _load(path, "navigation-reset-summary")
    if d.get("schema") not in RESET_SCHEMAS:
        raise StudyError(f"unrecognised reset-summary schema {d.get('schema')!r}")
    transport, actions = d.get("transport", {}), d.get("actions", {})
    if "rosDomainId" in transport and transport["rosDomainId"] is not None:
        validate_ros_domain(transport["rosDomainId"])
    return {"schema": d["schema"], "valid": bool(d.get("valid")), "scope": d.get("scope"),
            "rosDomainId": transport.get("rosDomainId"),
            "actions": {k: actions.get(k) for k in ("accepted", "rejected", "stale", "crossEpisode")},
            "realTimeFactor": d.get("realTimeFactor")}


def parse_benchmark_counters(path):
    d = _load(path, "crane-benchmark-v1")
    if d.get("schema") != BENCHMARK_SCHEMA:
        raise StudyError(f"expected {BENCHMARK_SCHEMA}, got {d.get('schema')!r}")
    return {"schema": d["schema"], "scenario": d.get("scenario"),
            "counters": {k: d.get(k) for k in COUNTER_KEYS}}


def parse_replay_inspect(path):
    d = _load(path, "inspect_replay output")
    meta = d.get("runtimeMetadata") or {}
    return {"frameCount": d.get("frameCount"), "actionCount": d.get("actionCount"),
            "outcomeCount": d.get("outcomeCount"),
            "calibrationProfileSha256": meta.get("calibrationProfileSha256")}


def check_profile_match(replay_info, context_profile):
    """context_profile is an opaque 'id@sha'. Returns 'OK' or 'PROFILE_MISMATCH'."""
    want = context_profile.split("@", 1)[1].lower()
    got = (replay_info.get("calibrationProfileSha256") or "").lower()
    return "OK" if got and got == want else "PROFILE_MISMATCH"


class CraneNav2Worker:
    label_ceiling = "NOT_RUN"

    def run(self, config, context, fidelity, seed, spec):
        if not preregistration_frozen():
            raise NotRun("NOT_RUN: PREREGISTRATION.md is still DRAFT; no CRANE Nav2 run may be launched")
        raise NotRun("NOT_RUN: Nav2 execution is intentionally not implemented in this binding")

    def launch(self, *a, **kw):
        raise NotRun("NOT_RUN: this binding never launches anything")
