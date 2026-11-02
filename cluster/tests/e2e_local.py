#!/usr/bin/env python3
"""End-to-end test on this machine: `wrangler dev` (local D1/R2 under a temp dir,
no Cloudflare login, nothing deployed) plus two worker daemons running dummy jobs.

Checks: invites and join; capability matching; a worker SIGKILLed mid-job whose
lease expires so the job is retried and finished by the other worker; artifacts;
the cron sweep; the GitHub webhook; and a hypothesis walked from stage 1 through
successive halving and stage 2 to 'candidate' with `crane-hyp report --apply`,
on arms checked out from a local git repo at exact commits.

  python3 tests/e2e_local.py [--keep] [--server]   (--server: the self-hosted coordinator, cluster/server)
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from crane_cluster.api import Client  # noqa: E402

ADMIN = "e2e-admin-token"
SERVER = "--server" in sys.argv  # run against cluster/server (self-hosted) instead of `wrangler dev`
HOOK = "e2e-hook-secret"
TTL = 6


# What every job submission must carry (the public feed shows it).
META = {"purpose": "End-to-end check of leases, retries and artifacts on dummy jobs.", "est_seconds": 4, "submitter": "e2e"}


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(pred, timeout, what, every=0.5):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(every)
    raise AssertionError(f"timed out waiting for {what}")


def git(*a, cwd):
    return subprocess.run(["git", "-c", "user.email=e2e@local", "-c", "user.name=e2e", *a], cwd=cwd, check=True,
                          capture_output=True, text=True).stdout.strip()


class Procs:
    def __init__(self):
        self.procs: dict[str, subprocess.Popen] = {}

    def start(self, name, argv, log: Path, **kw):
        self.procs[name] = subprocess.Popen(argv, stdout=open(log, "wb"), stderr=subprocess.STDOUT, start_new_session=True, **kw)
        return self.procs[name]

    def kill(self, name, sig=signal.SIGKILL):
        p = self.procs.pop(name)
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            pass
        p.wait(timeout=30)

    def stop_all(self):
        for n in list(self.procs):
            self.kill(n, signal.SIGTERM)


def main():
    keep = "--keep" in sys.argv
    tmp = Path(tempfile.mkdtemp(prefix="crane-e2e-"))
    procs = Procs()
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    coord = HERE / "coordinator"
    env = {**os.environ, "WRANGLER_SEND_METRICS": "false", "CI": "1", "NO_COLOR": "1"}
    state = tmp / "state"
    try:
        print(f"[e2e] temp dir {tmp}")
        if SERVER:  # the self-hosted coordinator (server/dist, built by `npm run build` there)
            procs.start("wrangler", ["node", "--disable-warning=ExperimentalWarning", str(HERE / "server" / "dist" / "main.mjs")],
                        tmp / "wrangler.log", env={**env, "CRANE_DATA_DIR": str(state), "PORT": str(port), "ADMIN_TOKEN": ADMIN,
                                                   "GITHUB_WEBHOOK_SECRET": HOOK, "LEASE_TTL_SECONDS": str(TTL),
                                                   "ARCHIVE_URL": "", "CRANE_TEST_SCHEDULED": "1"})
        else:
            subprocess.run(["npx", "wrangler", "d1", "migrations", "apply", "crane-cluster", "--local", "--persist-to", str(state)],
                           cwd=coord, env=env, check=True, capture_output=True)
            procs.start("wrangler", ["npx", "wrangler", "dev", "--local", "--ip", "127.0.0.1", "--port", str(port),
                                     "--persist-to", str(state), "--test-scheduled", "--show-interactive-dev-session=false",
                                     "--var", f"ADMIN_TOKEN:{ADMIN}", "--var", f"GITHUB_WEBHOOK_SECRET:{HOOK}",
                                     "--var", f"LEASE_TTL_SECONDS:{TTL}"], tmp / "wrangler.log", cwd=coord, env=env)
        adm = Client(url, ADMIN, retries=0)

        def up():
            try:
                return adm.get("/v1/ping")["ok"]
            except Exception:
                return False
        wait_for(up, 120, "wrangler dev")
        print(f"[e2e] coordinator up at {url}")

        # ---- two workers; only B offers the 'gpu-sim' tag
        invites = adm.post("/v1/admin/invite", {"n": 2})["invites"]
        workers = {}
        for name, tags in (("A", ["cpu"]), ("B", ["cpu", "gpu-sim"])):
            d = tmp / f"worker-{name}"
            d.mkdir()
            cfg = d / "worker.yaml"
            cfg.write_text(f"coordinator: {url}\nname: e2e-{name}\npoll_s: 1\nheartbeat_s: 1\ntoken_file: {d / 'token'}\n"
                           f"data_dir: {d / 'data'}\ncaps:\n  tags: {json.dumps(tags)}\n  vulkan: false\n  gpu: null\n"
                           f"policy:\n  nice: 5\nbackends:\n  dummy:\n    type: dummy\n    sleep_s: 0.3\n")
            # the real installer, minus the systemd unit: copies the package, joins with the invite
            subprocess.run(["bash", str(HERE / "install" / "install.sh"), "--coordinator", url, "--invite", invites.pop(),
                            "--config", str(cfg), "--prefix", str(d / "inst"), "--no-service", "--minimal"], check=True, capture_output=True)
            workers[name] = cfg

        def start_worker(name):
            exe = tmp / f"worker-{name}" / "inst" / "bin" / "crane-worker"
            procs.start(name, [str(exe), "--config", str(workers[name]), "run"], tmp / f"worker-{name}.log", cwd=tmp)

        # ---- kill A mid-job: its lease must expire and B must finish the job on attempt 2
        [victim] = adm.post("/v1/admin/jobs", {"jobs": [{"backend": "dummy", "seeds": [1, 2, 3], "spec": {"sleep_s": 4}, **META}]})["job_ids"]
        start_worker("A")
        wait_for(lambda: adm.get(f"/v1/jobs/{victim}/runs")["runs"], 30, "A to finish its first seed of the victim job")
        procs.kill("A")
        print(f"[e2e] worker A killed mid-job {victim}")
        t_kill = time.time()
        others = adm.post("/v1/admin/jobs", {"jobs": [{"backend": "dummy", "seeds": [s], **META} for s in range(10, 16)]
                                             + [{"backend": "dummy", "needs": {"gpu": True}, **META}]})["job_ids"]
        start_worker("B")

        def victim_done():
            j = [x for x in adm.get("/v1/jobs")["jobs"] if x["job_id"] == victim][0]
            return j if j["status"] == "done" else None
        j = wait_for(victim_done, 90, "the victim job to be retried and finished")
        assert j["attempt"] == 2, j
        assert time.time() - t_kill >= TTL - 1, "retried before the lease could have expired"
        runs = adm.get(f"/v1/jobs/{victim}/runs")["runs"]
        assert {r["attempt"] for r in runs} == {1, 2} and len([r for r in runs if r["attempt"] == 2]) == 3, runs
        status = adm.get("/v1/status")
        names = {w["id"]: w["name"] for w in status["workers"]}
        assert names[runs[-1]["worker_id"]] == "e2e-B", runs
        print(f"[e2e] victim retried: attempt 2 by e2e-B ({j['last_error'] or 'ok'})")
        wait_for(lambda: all(x["status"] == "done" for x in adm.get("/v1/jobs")["jobs"] if x["job_id"] in others[:-1]), 60,
                 "the other jobs")
        gpu_job = [x for x in adm.get("/v1/jobs")["jobs"] if x["job_id"] == others[-1]][0]
        assert gpu_job["status"] == "queued", "a GPU job was leased by a worker without a GPU"
        adm.post("/v1/admin/jobs/cancel", {"job_ids": [others[-1]]})
        arts = adm.get("/v1/artifacts", job=victim)["artifacts"]
        assert {"metrics.json", "summary.json", "artifacts/log.txt", "provenance.json"} <= {a["name"] for a in arts}, arts
        blob = adm.request("GET", f"/v1/artifact/{[a for a in arts if a['name'] == 'metrics.json'][0]['key']}")
        assert "score" in json.loads(blob), blob
        cron = subprocess.run(["curl", "-sf", f"{url}/__scheduled?cron=*/5+*+*+*+*"], capture_output=True, text=True)
        assert cron.returncode == 0, cron.stderr
        print(f"[e2e] artifacts ok ({len(arts)} for the victim), cron sweep ran")

        # ---- a local repo with a baseline and two variant branches; webhook sets main_version
        repo = tmp / "demo-repo"
        repo.mkdir()
        git("init", "-q", "-b", "main", cwd=repo)
        (repo / "gain.txt").write_text("1.0\n")
        git("add", ".", cwd=repo)
        git("commit", "-qm", "baseline", cwd=repo)
        main_sha = git("rev-parse", "HEAD", cwd=repo)
        for br, gain in (("hyp/strong", "1.5"), ("hyp/flat", "1.0")):
            git("checkout", "-qb", br, "main", cwd=repo)
            (repo / "gain.txt").write_text(f"{gain}  # {br}\n")
            git("commit", "-qam", br, cwd=repo)
        git("checkout", "-q", "main", cwd=repo)
        push = json.dumps({"ref": "refs/heads/main", "after": main_sha,
                           "repository": {"full_name": "local/demo", "clone_url": str(repo), "default_branch": "main"}}).encode()
        sig = "sha256=" + hmac.new(HOOK.encode(), push, hashlib.sha256).hexdigest()
        r = Client(url).request("POST", "/v1/webhook/github", data=push,
                                headers={"x-github-event": "push", "x-hub-signature-256": sig, "content-type": "application/json"})
        assert r["main_version"] == main_sha, r
        print(f"[e2e] webhook: local/demo main_version {main_sha[:12]}")

        start_worker("A")  # back for the hypothesis
        hyp_yaml = tmp / "hypothesis.yaml"
        hyp_yaml.write_text(f"""\
name: e2e-strong-gain
claim: the strong gain variant raises the score
metric: score
direction: increase
repo: local/demo
repo_url: {repo}
baseline: "@main"
variants:
  - name: strong
    ref: hyp/strong
    params: {{effect: 0.4}}
  - name: flat
    ref: hyp/flat
seeds: 6
budget: {{max_runs: 200}}
stages:
  - name: fast
    backend: dummy
    params: {{sleep_s: 0.1, sd: 0.3, jitter_sd: 0.05}}
  - name: gpu-sim
    backend: dummy
    params: {{sleep_s: 0.1, sd: 0.3, jitter_sd: 0.05}}
    needs: {{tags: [gpu-sim]}}
    seeds_per_job: 3
promotion: {{alpha: 0.05, min_pairs: 6, halving_eta: 2, max_seeds: 24}}
""")
        cli_env = {**os.environ, "CRANE_COORDINATOR": url, "CRANE_TOKEN": ADMIN, "PYTHONPATH": str(HERE)}
        out = subprocess.run([sys.executable, "-m", "crane_cluster.hyp", "--json", "submit", str(hyp_yaml),
                              "--purpose", "End-to-end check that the strong gain branch is promoted over the flat one.",
                              "--est-seconds", "2", "--submitter", "e2e"], env=cli_env,
                             capture_output=True, text=True, check=True).stdout
        hid = json.loads(out)["id"]
        print(f"[e2e] hypothesis {hid} submitted")
        decisions = []
        for _ in range(12):
            st = adm.get(f"/v1/hyp/{hid}")
            if st["status"] != "running":
                break
            wait_for(lambda: adm.get(f"/v1/hyp/{hid}/results")["pending"] == 0, 120, "stage jobs")
            rep = json.loads(subprocess.run([sys.executable, "-m", "crane_cluster.hyp", "--json", "report", hid, "--apply"],
                                            env=cli_env, capture_output=True, text=True, check=True).stdout)
            decisions.append((rep["stage"], rep["decision"], rep["variants"]))
            print(f"[e2e]   stage {rep['stage']}: {rep['decision']} {rep['variants'] or ''} - {rep['reason']}")
        st = adm.get(f"/v1/hyp/{hid}")
        assert st["status"] == "candidate", (st["status"], decisions)
        assert decisions[0][1] == "extend" and decisions[0][2] == ["strong"], decisions
        assert [v["name"] for v in st["variants"] if v["active"]] == ["strong"]
        # stage-2 jobs ran only on the gpu-sim worker, and arms ran at the exact commits
        st2 = [x for x in adm.get("/v1/jobs", hypothesis=hid)["jobs"] if x["stage_index"] == 1]
        assert st2 and all(x["status"] == "done" for x in st2)
        b_id = [w["id"] for w in adm.get("/v1/status")["workers"] if w["name"] == "e2e-B"][0]
        assert all(r["worker_id"] == b_id for x in st2 for r in adm.get(f"/v1/jobs/{x['job_id']}/runs")["runs"])
        strong = [x for x in st2 if x["arm"] == "strong"][0]
        assert strong["repos"][0]["ref"] == git("rev-parse", "hyp/strong", cwd=repo)
        prov = [a for a in adm.get("/v1/artifacts", job=strong["job_id"])["artifacts"] if a["name"] == "provenance.json"][0]
        assert json.loads(adm.request("GET", f"/v1/artifact/{prov['key']}"))["resolved"]["local/demo"] == strong["repos"][0]["ref"]
        baseline = [x for x in st2 if x["arm"] == "baseline"][0]
        assert baseline["repos"][0]["ref"] == main_sha
        print(f"[e2e] hypothesis reached candidate: {[d[:2] for d in decisions]}")

        # ---- the public feed: no token, every job with its purpose, nothing private
        feed_raw = subprocess.run(["curl", "-sf", f"{url}/v1/feed?recent=200"], capture_output=True, text=True, check=True).stdout
        feed = json.loads(feed_raw)
        assert feed["schema"] == "crane-feed/1" and feed["jobs"], feed
        assert all(len(j["purpose"]) >= 20 and j["submitter"] for j in feed["jobs"]), feed["jobs"]
        assert ADMIN not in feed_raw and str(tmp) not in feed_raw and not re.search(r"\bw_[0-9a-f]{16}\b", feed_raw), "the feed leaks"
        sse = subprocess.run(["curl", "-sf", f"{url}/v1/feed/stream?once=1"], capture_output=True, text=True, check=True).stdout
        assert "event: feed" in sse, sse[:200]
        print(f"[e2e] feed: {len(feed['jobs'])} jobs, {len(feed['runs'])} runs, sse ok")
        print("[e2e] PASS")
    except BaseException:
        for log in sorted(tmp.glob("*.log")):
            print(f"----- {log.name} (tail)\n" + "\n".join(log.read_text(errors="replace").splitlines()[-25:]))
        raise
    finally:
        procs.stop_all()
        if not keep:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
