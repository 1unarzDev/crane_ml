"""crane-hyp: submit a hypothesis, watch it, and report/apply the promotion decision.

  crane-hyp submit hypothesis.yaml --purpose "..."  # enqueue stage 1 (refs resolved to exact commits)
  crane-hyp status [ID]                  # one hypothesis, or the recent list
  crane-hyp report ID [--apply] [--json] # paired test vs baseline; --apply posts the decision

Every submission says why it runs, in public (the coordinator's /v1/feed): --purpose, 1-2
sentences by the submitting agent on what the hypothesis tests or what it is for; est_seconds
(per run) from --est-seconds, the file or every stage; the submitter from --submitter,
CRANE_SUBMITTER or the user; --campaign optionally links a campaign, boat test or study.

Environment: CRANE_COORDINATOR (URL) and CRANE_TOKEN (admin token for submit and
--apply; a viewer token is enough for status and report).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time

from . import miniyaml, promotion
from .api import ApiError, Backoff, Client, unavailable, wait_until_up

SHA = re.compile(r"^[0-9a-f]{40}$")


def ls_remote(url: str, ref: str) -> str:
    """The commit a branch or tag names on the remote right now."""
    out = subprocess.run(["git", "ls-remote", url, ref, f"refs/heads/{ref}", f"refs/tags/{ref}"],
                         capture_output=True, text=True, timeout=120, env={**os.environ, "GIT_TERMINAL_PROMPT": "0"})
    if out.returncode != 0:
        raise SystemExit(f"git ls-remote {url} {ref}: {out.stderr.strip()}")
    shas = {line.split()[0] for line in out.stdout.splitlines() if line.strip()}
    if len(shas) != 1:
        raise SystemExit(f"ref {ref!r} on {url} is {'missing' if not shas else 'ambiguous'}")
    return shas.pop()


PURPOSE_MIN = 20


def check_purpose(text) -> str:
    """The coordinator's first checks, locally: 1-2 real sentences, not a placeholder."""
    s = " ".join(str(text or "").split())
    if len(s) < PURPOSE_MIN or len([w for w in s.split() if any(ch.isalpha() for ch in w)]) < 4:
        raise SystemExit(f"--purpose needs 1-2 sentences ({PURPOSE_MIN}+ chars) on what this tests or is for; "
                         "it is shown publicly at /v1/feed")
    return s


def normalize(spec: dict, resolve: bool = True, purpose: str | None = None, est_seconds: float | None = None,
              submitter: str | None = None, campaign: str | None = None) -> dict:
    """hypothesis.yaml (+ the CLI's purpose, estimate, submitter, campaign) -> the coordinator's POST /v1/hyp body."""
    out = dict(spec)
    if purpose is not None:
        out["purpose"] = purpose
    if est_seconds is not None:
        out["est_seconds"] = est_seconds
    if campaign is not None:
        out["campaign"] = campaign
    if submitter:
        out["submitted_by"] = submitter
    seeds = out.get("seeds", 10)
    out["seeds"] = list(range(1, seeds + 1)) if isinstance(seeds, int) else [int(s) for s in seeds]
    if "baseline" in out and "baseline_ref" not in out:
        b = out.pop("baseline")
        out["baseline_ref"] = b["ref"] if isinstance(b, dict) else str(b)
    url = out.get("repo_url")
    if resolve and out.get("repo") and url:
        def pin(ref):
            return ref if ref == "@main" or SHA.match(ref) else ls_remote(url, ref)
        out["baseline_ref"] = pin(out["baseline_ref"])
        out["variants"] = [{**v, "ref": pin(v["ref"])} for v in out["variants"]]
    out.setdefault("submitted_by", os.environ.get("CRANE_SUBMITTER") or os.environ.get("USER") or "unknown")
    return out


class WaitingClient(Client):
    """GETs wait out an unavailable coordinator (503 out of quota, 5xx, unreachable): 5 s doubling to 5 min."""

    sleep = staticmethod(time.sleep)

    def get(self, path, **q):
        b = Backoff()
        while True:
            try:
                return super().get(path, **q)
            except (ApiError, OSError) as e:
                why = unavailable(e)
                if why is None:
                    raise
                w = b.step(e)
                print(f"crane-hyp: coordinator unavailable ({why}); retrying in {w:.0f} s", file=sys.stderr)
                self.sleep(w)


def client(a) -> Client:
    url = a.coordinator or os.environ.get("CRANE_COORDINATOR")
    if not url:
        raise SystemExit("set CRANE_COORDINATOR or pass --coordinator")
    return WaitingClient(url, a.token or os.environ.get("CRANE_TOKEN"))


def cmd_submit(a):
    spec = miniyaml.load_file(a.file)
    body = normalize(spec, resolve=not a.no_resolve, purpose=check_purpose(a.purpose), est_seconds=a.est_seconds,
                     submitter=a.submitter, campaign=a.campaign)
    if body.get("est_seconds") is None and not all("est_seconds" in s for s in body.get("stages") or [{}]):
        raise SystemExit("est_seconds required: --est-seconds N (per run), est_seconds in the file, or one per stage")
    c = client(a)
    # A hypothesis runs only on the cluster (staged, paired arms): when the coordinator is down, wait for it
    # (5 s doubling to 5 min) and say why, then submit once, so a retried POST cannot queue it twice.
    wait_until_up(c, log=lambda m: print(f"crane-hyp: {m}; hypotheses run on the cluster only", file=sys.stderr))
    r = c.post("/v1/hyp", body)
    print(json.dumps(r) if a.json else f"{r['id']}: {r['jobs']} jobs queued for {r['stage']} (baseline {r['baseline_ref'][:12]})")


def cmd_status(a):
    c = client(a)
    if not a.id:
        for h in c.get("/v1/hyp")["hypotheses"]:
            print(f"{h['id']}  {h['status']:<9} {h['stage']:<10} {h['name']}")
        return
    h = c.get(f"/v1/hyp/{a.id}")
    if a.json:
        print(json.dumps(h, indent=2))
        return
    print(f"{h['id']}  {h['name']}  [{h['status']}, stage {h['stage']}]\n  claim: {h['claim']}\n"
          f"  metric: {h['metric']} ({h['direction']}), runs used {h['runs_used']}")
    for row in h["jobs"]:
        print(f"  stage {row['stage_index']}  {row['arm']:<12} {row['status']:<9} {row['n']}")


def report(c: Client, hid: str) -> tuple[dict, dict, promotion.Decision]:
    h = c.get(f"/v1/hyp/{hid}")
    res = c.get(f"/v1/hyp/{hid}/results")
    return h, res, promotion.decide(res, h, runs_used=h.get("runs_used"))


def cmd_report(a):
    c = client(a)
    h, res, d = report(c, a.id)
    if h["status"] != "running":
        print(f"{h['id']} is {h['status']}; decisions: {json.dumps(h['decisions'])}")
        return
    applied = None
    if a.apply and d.action != "wait":
        body = {"action": d.action, "evidence": d.evidence()}
        if d.variants is not None:
            body["variants"] = d.variants
        if d.seeds:
            body["seeds"] = d.seeds
        applied = c.post(f"/v1/hyp/{a.id}/advance", body)
    if a.json:
        print(json.dumps({"hypothesis": h["id"], "stage": h["stage"], "decision": d.action, "variants": d.variants,
                          "seeds": d.seeds, **d.evidence(), "applied": applied}, indent=2))
        return
    print(f"{h['id']} {h['name']}: stage {h['stage']}, metric {h['metric']} ({h['direction']}), "
          f"{res['pending']} pending, {res['dead']} dead")
    print(f"  {'variant':<14}{'pairs':>6}{'mean d':>10}{'median d':>10}{'p better':>10}{'p worse':>10}")
    for v, s in d.stats.items():
        print(f"  {v:<14}{s.n:>6}{s.mean_diff:>10.4g}{s.median_diff:>10.4g}{s.p_better_adj:>10.3g}{s.p_worse_adj:>10.3g}")
    print(f"  decision: {d.action} - {d.reason}" + (f"; seeds {d.seeds[0]}..{d.seeds[-1]}" if d.seeds else ""))
    if applied:
        print(f"  applied: now {applied['status']} at {applied['stage']} ({applied['jobs']} jobs queued)")


def main(argv=None):
    p = argparse.ArgumentParser(prog="crane-hyp", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--coordinator")
    p.add_argument("--token")
    p.add_argument("--json", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("submit")
    s.add_argument("file")
    s.add_argument("--no-resolve", action="store_true", help="send branch names as given instead of exact commits")
    s.add_argument("--purpose", required=True, help="1-2 sentences: what this hypothesis tests or is for (public)")
    s.add_argument("--est-seconds", type=float, help="estimated seconds per run (else the file's est_seconds)")
    s.add_argument("--submitter", help="agent or session label (default CRANE_SUBMITTER, else the user)")
    s.add_argument("--campaign", help="optional campaign, boat test or study this belongs to")
    st = sub.add_parser("status")
    st.add_argument("id", nargs="?")
    r = sub.add_parser("report")
    r.add_argument("id")
    r.add_argument("--apply", action="store_true", help="post the decision (promote, extend or reject)")
    a = p.parse_args(argv)
    try:
        {"submit": cmd_submit, "status": cmd_status, "report": cmd_report}[a.cmd](a)
    except ApiError as e:
        sys.exit(f"coordinator: {e}")


if __name__ == "__main__":
    main()
