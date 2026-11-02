"""The worker daemon: lease a job, check out its exact refs, run its backend per
seed, report metrics and artifacts, heartbeat throughout. Repos and images are
updated only between jobs. ``crane-worker join|run|caps|on|off|status``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait
import urllib.parse
from pathlib import Path

from . import __version__, backends as backend_mod, config as config_mod, idle
from .api import ApiError, Client
from .gitcache import GitCache

log = logging.getLogger("crane-worker")


class Stop(Exception):
    pass


class Worker:
    def __init__(self, cfg: dict, client: Client | None = None):
        self.cfg = cfg
        self.policy = cfg.get("policy") or {}
        self.backends = backend_mod.build(cfg["backends"])
        self.client = client or Client(cfg["coordinator"], cfg["token_file"].read_text().strip())
        self.git = GitCache(cfg["data_dir"] / "git")
        self.stop = threading.Event()
        self.repo_seen: dict[str, str | None] = {}
        self.repo_urls: dict[str, str] = {}
        self.last_update = 0.0
        self.paused: str | None = None
        self.caps = config_mod.detect_caps(cfg, sorted(self.backends))

    def note(self, state: str, reason: str | None = None, job: str | None = None) -> None:
        """What ``crane-worker status`` reports: <data_dir>/state.json."""
        try:
            path = self.cfg["data_dir"] / "state.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"state": state, "reason": reason, "job": job, "pid": os.getpid(), "at": time.time()}))
            tmp.replace(path)
        except OSError as e:
            log.warning("state file: %s", e)

    # ------------------------------------------------------------ between jobs

    def between_jobs(self, versions: dict[str, str | None]) -> None:
        """Refresh mirrors whose main moved, and (hourly) let backends pull images. Never during a job."""
        for name, ver in versions.items():
            if name in self.repo_urls and self.repo_seen.get(name) != ver:
                log.info("repo %s main -> %s; refreshing mirror", name, ver)
                try:
                    self.git.refresh(name, self.repo_urls[name])
                    self.repo_seen[name] = ver
                except RuntimeError as e:
                    log.warning("refresh %s failed: %s", name, e)
        if time.time() - self.last_update > float(self.cfg.get("update_interval_s", 3600)):
            self.last_update = time.time()
            for b in self.backends.values():
                try:
                    b.update()
                except Exception as e:  # an image pull failure must not stop the worker
                    log.warning("backend %s update failed: %s", b.name, e)

    # ------------------------------------------------------------ main loop

    def loop(self, once: bool = False) -> None:
        idle.lower_priority(self.policy)
        versions: dict[str, str | None] = {}
        while not self.stop.is_set():
            reason = idle.unavailable_reason(self.policy, cfg=self.cfg)
            if reason:
                if reason != self.paused:
                    log.info("paused: %s", reason)
                self.paused = reason
                self.note("paused", reason)
                self._safe_post("/v1/heartbeat", {"job_ids": [], "state": "paused", "stats": {"reason": reason}})
                if once:
                    return
                self.stop.wait(min(float(self.cfg["poll_s"]), float(self.cfg["heartbeat_s"])))
                continue
            if self.paused:
                log.info("resumed (%s cleared)", self.paused)
                self.paused = None
            self.note("idle")
            self.between_jobs(versions)
            try:
                r = self.client.post("/v1/lease", {"caps": self.caps, "policy": self.policy, "max_jobs": 1, "state": "idle",
                                                   "client_version": __version__})
            except ApiError as e:
                if e.status == 426:
                    log.error("coordinator requires a newer client: %s", e.body)
                    self.stop.wait(600)
                    continue
                raise
            except OSError as e:
                log.warning("coordinator unreachable: %s", e)
                self.stop.wait(float(self.cfg["poll_s"]))
                continue
            versions = r.get("repos") or {}
            ttl = float(r.get("lease_ttl_s") or 300)
            for job in r["jobs"]:
                self.run_job(job, ttl)
            if once:
                return
            if not r["jobs"]:
                self.stop.wait(float(self.cfg["poll_s"]))

    def _safe_post(self, path, body):
        try:
            return self.client.post(path, body)
        except (ApiError, OSError) as e:
            log.warning("%s failed: %s", path, e)
            return None

    # ------------------------------------------------------------ one job

    def run_job(self, job: dict, ttl: float) -> str:
        jid = job["job_id"]
        log.info("job %s (%s, arm %s, attempt %s, seeds %s)", jid, job["backend"], job.get("arm"), job["attempt"], job["seeds"])
        backend = self.backends[job["backend"]]
        work = self.cfg["data_dir"] / "jobs" / f"{jid}-a{job['attempt']}"
        shutil.rmtree(work, ignore_errors=True)
        work.mkdir(parents=True)
        iso = self.cfg["isolation"]
        slot = int(iso.get("slot", 0))
        ctx = backend_mod.JobContext(job=job, work_dir=work, isolation={
            "slot": slot, "port_base": int(iso["port_base"]) + slot * int(iso["port_stride"]),
            "isolation_id": int(iso["id_base"]) + slot})
        revoked, released = threading.Event(), threading.Event()
        done = threading.Event()
        self.note("running", job=jid)

        def heartbeat():
            period = max(1.0, min(float(self.cfg["heartbeat_s"]), ttl / 3))
            tick, beat = min(1.0, period), time.monotonic() + period
            while not done.wait(tick):
                now = self.stop.is_set() or self.cfg["off_file"].exists()  # shutdown or `crane-worker off`: at once
                if not now and time.monotonic() < beat:
                    continue
                beat = time.monotonic() + period
                why = "stop" if self.stop.is_set() else "off" if now else \
                    "user active" if self.policy.get("pause_on_activity") and idle.user_active(self.policy) else \
                    "pause_when" if idle.pause_requested(self.cfg.get("pause_when"), float(self.cfg.get("pause_when_timeout_s", 20))) \
                    else None
                if why:
                    log.info("releasing job %s (%s)", jid, why)
                    released.set()
                    backend.cancel()
                    return
                r = self._safe_post("/v1/heartbeat", {"job_ids": [jid], "state": "running"})
                if r and jid in r.get("revoked", []):
                    log.warning("job %s revoked by the coordinator; stopping", jid)
                    revoked.set()
                    backend.cancel()
                    return

        hb = threading.Thread(target=heartbeat, daemon=True)
        hb.start()
        error = None
        # Artifact uploads (about 0.8 s each through the coordinator into R2) run in the background,
        # overlapping the next seeds; the job completes only after they finish.
        uploads = ThreadPoolExecutor(max_workers=int(self.cfg.get("upload_threads", 8)), thread_name_prefix="upload")
        pending = []
        try:
            for repo in job.get("repos") or []:
                self.repo_urls[repo["name"]] = repo["url"]
                dest = work / "src" / repo["name"].replace("/", "__")
                ctx.resolved[repo["name"]] = self.git.checkout(repo["name"], repo["url"], repo["ref"], dest)
                ctx.repos[repo["name"]] = dest
            backend.prepare(ctx)
            for seed in job["seeds"]:
                if revoked.is_set() or released.is_set():
                    break
                t0 = time.time()
                try:
                    run_dir = backend.run(seed)
                    metrics = backend.metrics(run_dir)
                except Exception as e:
                    if revoked.is_set() or released.is_set():
                        break
                    error = f"seed {seed}: {e}"
                    self._safe_post(f"/v1/jobs/{jid}/runs", {"seed": seed, "status": "failed", "error": str(e)[:2000],
                                                            "wall_s": time.time() - t0})
                    break
                run = self.client.post(f"/v1/jobs/{jid}/runs", {"seed": seed, "metrics": metrics, "wall_s": time.time() - t0})
                for path, name, retention in backend.artifacts(run_dir):
                    pending.append(uploads.submit(self._upload_file, jid, run["run_id"], path, name, retention))
                prov = {"resolved": ctx.resolved, "backend": job["backend"], "worker": self.cfg["name"]}
                pending.append(uploads.submit(self._upload_bytes, jid, run["run_id"], "provenance.json",
                                              json.dumps(prov).encode(), "summary"))
        except Exception as e:
            error = error or f"{type(e).__name__}: {e}"
        finally:
            wait(pending)
            uploads.shutdown()
            done.set()
            hb.join(timeout=5)
            try:
                backend.finish()
            except Exception as e:
                log.warning("backend finish: %s", e)
        if revoked.is_set():
            outcome = "revoked"
        elif released.is_set():
            self._safe_post("/v1/release", {"job_ids": [jid]})
            outcome = "released"
        else:
            r = self._safe_post(f"/v1/jobs/{jid}/complete", {"status": "failed" if error else "ok", "error": error})
            outcome = (r or {}).get("status", "unknown")
        log.info("job %s -> %s%s", jid, outcome, f" ({error})" if error else "")
        if not self.cfg.get("keep_work"):
            shutil.rmtree(work, ignore_errors=True)
        return outcome

    def upload_artifacts(self, jid: str, run_id: str, items) -> None:
        for path, name, retention in items:
            self._upload_file(jid, run_id, path, name, retention)

    def _upload_file(self, jid, run_id, path, name, retention):
        try:
            data = Path(path).read_bytes()
        except OSError as e:
            log.warning("artifact %s unreadable: %s", name, e)
            return
        self._upload_bytes(jid, run_id, name, data, retention)

    def _upload_bytes(self, jid, run_id, name, data, retention):
        q = urllib.parse.urlencode({"run": run_id, "retention": retention})
        try:
            self.client.upload(f"/v1/jobs/{jid}/artifacts/{urllib.parse.quote(name)}?{q}", data, hashlib.sha256(data).hexdigest())
        except (ApiError, OSError) as e:
            log.warning("artifact %s upload failed: %s", name, e)


# ---------------------------------------------------------------- CLI

def cmd_join(a, cfg):
    url = a.coordinator or cfg["coordinator"]
    if not url:
        sys.exit("give --coordinator or set coordinator: in the config")
    backends = sorted(backend_mod.build(cfg["backends"]))
    r = Client(url).post("/v1/join", {"invite": a.invite, "name": a.name or cfg["name"], "client_version": __version__,
                                      "caps": config_mod.detect_caps(cfg, backends), "policy": cfg.get("policy") or {}})
    tf: Path = cfg["token_file"]
    tf.parent.mkdir(parents=True, exist_ok=True)
    tf.write_text(r["token"] + "\n")
    tf.chmod(0o600)
    print(f"joined as {r['worker_id']}; token saved to {tf}")


UNIT = "crane-worker.service"


def _systemd_unit() -> bool:
    """True when this user's systemd runs the worker as crane-worker.service (install.sh)."""
    if not shutil.which("systemctl"):
        return False
    r = subprocess.run(["systemctl", "--user", "cat", UNIT], capture_output=True, text=True)
    return r.returncode == 0


def cmd_toggle(a, cfg):
    """on/off: the off flag (honoured by any running worker, at once) plus, under systemd, enable/disable --now."""
    flag: Path = cfg["off_file"]
    if a.cmd == "off":
        flag.parent.mkdir(parents=True, exist_ok=True)
        flag.write_text(f"crane-worker off at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    else:
        flag.unlink(missing_ok=True)
    if _systemd_unit():
        subprocess.run(["systemctl", "--user", "disable" if a.cmd == "off" else "enable", "--now", "--quiet", UNIT], check=False)
    cmd_status(a, cfg)


def cmd_status(a, cfg):
    service = None
    if _systemd_unit():
        service = subprocess.run(["systemctl", "--user", "is-active", UNIT], capture_output=True, text=True).stdout.strip()
    try:
        st = json.loads((cfg["data_dir"] / "state.json").read_text())
    except (OSError, ValueError):
        st = {}
    running = service == "active" if service is not None else bool(st) and time.time() - st.get("at", 0) < 3 * float(cfg["poll_s"]) + 60
    if cfg["off_file"].exists():
        line = "off" + (" (worker still stopping)" if running and st.get("state") == "running" else "")
    elif not running:
        line = "on, but the worker is not running" + (f" (service {service})" if service else "")
    elif st.get("state") == "paused":
        line = f"paused ({st.get('reason')})"
    elif st.get("state") == "running":
        line = f"on, running job {st.get('job')}"
    elif st.get("state") == "idle":
        line = "on, idle"
    else:
        line = "on, starting"
    print(line)


def main(argv=None):
    p = argparse.ArgumentParser(prog="crane-worker", description=__doc__)
    p.add_argument("--config", help="worker YAML (default: <config dir>/worker.yaml if present)")
    sub = p.add_subparsers(dest="cmd", required=True)
    j = sub.add_parser("join", help="register with an invite; saves the worker token")
    j.add_argument("--coordinator")
    j.add_argument("--invite", required=True)
    j.add_argument("--name")
    r = sub.add_parser("run", help="run the daemon")
    r.add_argument("--once", action="store_true", help="one lease round, then exit")
    sub.add_parser("caps", help="print detected capabilities")
    sub.add_parser("on", help="take jobs again (removes the off flag; starts and enables the service)")
    sub.add_parser("off", help="take no jobs until `on`: releases the current job now, stops and disables the service")
    sub.add_parser("status", help="on, off or paused (with the reason), and the current job")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    path = a.config or (config_mod.config_dir() / "worker.yaml")
    cfg = config_mod.load(path if Path(path).exists() else None)
    if a.cmd == "join":
        return cmd_join(a, cfg)
    if a.cmd in ("on", "off"):
        return cmd_toggle(a, cfg)
    if a.cmd == "status":
        return cmd_status(a, cfg)
    if a.cmd == "caps":
        print(json.dumps(config_mod.detect_caps(cfg, sorted(backend_mod.build(cfg["backends"]))), indent=2))
        return
    if not cfg["coordinator"]:
        sys.exit("no coordinator: set coordinator: in the config or CRANE_COORDINATOR")
    w = Worker(cfg)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: w.stop.set())
    w.loop(once=a.once)
    w.note("stopped")


if __name__ == "__main__":
    main()
