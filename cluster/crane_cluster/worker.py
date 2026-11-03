"""The worker daemon: lease jobs up to its free slots (slots.py), check out their exact refs, run each
backend per seed in its own thread, report metrics and artifacts, heartbeat each job throughout. Repos and
images are updated only while no job runs. ``crane-worker join|run|caps|on|off|drain|status``.
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

from . import __version__, backends as backend_mod, config as config_mod, idle, provision
from .slots import Slots
from .api import ApiError, Backoff, Client, unavailable
from .gitcache import GitCache

log = logging.getLogger("crane-worker")


class Stop(Exception):
    pass


def drain_file(cfg: dict) -> Path:
    """`crane-worker drain` creates it beside the off flag; the worker removes it when it exits drained."""
    return Path(cfg["off_file"]).parent / "drain"


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
        self.backoff = Backoff()
        self.caps = config_mod.detect_caps(cfg, sorted(self.backends))
        self.slots = Slots(cfg, self.caps)
        self.jobs: dict[str, threading.Thread] = {}   # running job id -> its thread
        self.freed = threading.Event()                # set when a job ends (a slot came free)
        self.git_lock = threading.Lock()              # the git mirrors are shared between jobs

    def note(self, state: str, reason: str | None = None, job: str | None = None) -> None:
        """What ``crane-worker status`` reports: <data_dir>/state.json."""
        with self._note_lock:
            self._note(state, reason, job)

    _note_lock = threading.Lock()

    def _note(self, state, reason, job) -> None:
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
        try:
            self._loop(once, versions)
        finally:
            for t in list(self.jobs.values()):   # shutdown: running jobs release themselves (heartbeat sees stop)
                t.join()

    def _loop(self, once: bool, versions: dict) -> None:
        drain = drain_file(self.cfg)
        while not self.stop.is_set():
            if drain.exists():
                # `crane-worker drain`: lease nothing, let running jobs finish, then exit for the service to restart.
                if not self.jobs:
                    drain.unlink(missing_ok=True)
                    log.info("drained; exiting for a restart")
                    self.stop.set()
                    return
                self.note("draining", job=", ".join(sorted(self.jobs)))
                self.freed.wait(float(self.cfg["poll_s"]))
                self.freed.clear()
                continue
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
            if not self.jobs:
                self.note("idle")
                self.between_jobs(versions)
            free = self.slots.free()
            if not free:   # every slot that could take a job is busy: wait for one to come free
                self.freed.wait(float(self.cfg["poll_s"]))
                self.freed.clear()
                continue
            # Long poll while idle: the coordinator holds an empty answer up to wait_s until a job arrives.
            wait_s = 0 if once or self.jobs else float(self.cfg.get("long_poll_s", 25))
            t0 = time.monotonic()
            try:
                # max_jobs: any mix of that many fits, since each backend alone fits that many.
                body = {"caps": self.caps, "policy": self.policy, "max_jobs": min(free.values()), "backends": sorted(free),
                        "state": "running" if self.jobs else "idle", "client_version": __version__}
                if wait_s > 0:
                    body["wait_s"] = wait_s
                r = self.client.post("/v1/lease", body)
            except (ApiError, OSError) as e:
                if isinstance(e, ApiError) and e.status == 426:
                    log.error("coordinator requires a newer client: %s", e.body)
                    self.stop.wait(600)
                    continue
                why = unavailable(e)
                if why is None:
                    raise
                # 503 (out of quota), 5xx or unreachable: back off 5 s doubling to 5 min, and say why.
                w = self.backoff.step(e)
                log.warning("coordinator unavailable (%s); next lease in %.0f s", why, w)
                if not self.jobs:
                    self.note("waiting", f"coordinator unavailable: {why}")
                if once:
                    return
                self.stop.wait(w)
                continue
            self.backoff.reset()
            versions = r.get("repos") or {}
            ttl = float(r.get("lease_ttl_s") or 300)
            for job in r["jobs"]:
                self._start(job, ttl)
            if once:
                return
            # Leased some: ask again at once for the slots still free. An empty answer: wait poll_s, less what a
            # long poll already waited (an older coordinator answers at once), or until a slot comes free.
            if not r["jobs"]:
                left = max(0.0, float(self.cfg["poll_s"]) - (time.monotonic() - t0))
                if self.jobs:
                    self.freed.wait(left)
                    self.freed.clear()
                else:
                    self.stop.wait(left)

    def _start(self, job: dict, ttl: float) -> None:
        jid = job["job_id"]
        slot = self.slots.acquire(jid, job["backend"])

        def body():
            try:
                self.run_job(job, ttl, slot=slot)
            except Exception:
                log.exception("job %s crashed", jid)
            finally:
                self.slots.release(jid)
                self.jobs.pop(jid, None)
                self._note_running()
                self.freed.set()

        t = threading.Thread(target=body, name=f"job-{jid}", daemon=True)
        self.jobs[jid] = t
        self._note_running()
        t.start()

    def _note_running(self) -> None:
        ids = sorted(self.jobs)
        if ids:
            self.note("running", job=", ".join(ids))

    def _safe_post(self, path, body):
        try:
            return self.client.post(path, body)
        except (ApiError, OSError) as e:
            log.warning("%s failed: %s", path, e)
            return None

    # ------------------------------------------------------------ one job

    def new_backend(self, name: str):
        """A fresh backend instance per job: concurrent jobs never share a backend's per-job state."""
        if name not in self.backends:
            raise KeyError(f"backend {name} is not configured here")
        return backend_mod.build({name: self.cfg["backends"][name]})[name]

    def run_job(self, job: dict, ttl: float, slot: int = 0) -> str:
        jid = job["job_id"]
        log.info("job %s (%s, arm %s, attempt %s, slot %s, seeds %s)", jid, job["backend"], job.get("arm"), job["attempt"],
                 slot, job["seeds"])
        backend = self.new_backend(job["backend"])
        work = self.cfg["data_dir"] / "jobs" / f"{jid}-a{job['attempt']}"
        shutil.rmtree(work, ignore_errors=True)
        work.mkdir(parents=True)
        iso = self.cfg["isolation"]
        slot = int(iso.get("slot", 0)) + slot   # each concurrent job its own ports and isolation id
        ctx = backend_mod.JobContext(job=job, work_dir=work, isolation={
            "slot": slot, "port_base": int(iso["port_base"]) + slot * int(iso["port_stride"]),
            "isolation_id": int(iso["id_base"]) + slot}, client=self.client)
        revoked, released = threading.Event(), threading.Event()
        done = threading.Event()

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
                with self.git_lock:
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
                run = self._post_run(jid, backend, run_dir, {"seed": seed, "metrics": metrics, "wall_s": time.time() - t0})
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

    def _post_run(self, jid: str, backend, run_dir, body: dict) -> dict:
        """POST the run, with the backend's ``run_meta(run_dir, metrics)`` (the theater index) when it has one.
        A meta the coordinator refuses (or one that fails to build) never costs the run: it is sent without."""
        meta = None
        if callable(getattr(backend, "run_meta", None)):
            try:
                meta = backend.run_meta(run_dir, body["metrics"])
            except Exception as e:  # metadata is optional; the run's numbers are not
                log.warning("run meta: %s", e)
        if not meta:
            return self.client.post(f"/v1/jobs/{jid}/runs", body)
        try:
            return self.client.post(f"/v1/jobs/{jid}/runs", {**body, "meta": meta})
        except ApiError as e:
            if e.status != 400:
                raise
            log.warning("run meta refused (%s); posting the run without it", e)
            return self.client.post(f"/v1/jobs/{jid}/runs", body)

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


TASK = "crane-worker"


def _win_task() -> str | None:
    """The scheduled task's state (Ready, Running, Disabled) when install.ps1 registered it, else None."""
    if sys.platform != "win32":
        return None
    r = subprocess.run(["powershell", "-NoProfile", "-Command", f"(Get-ScheduledTask -TaskName {TASK} -ErrorAction Stop).State"],
                       capture_output=True, text=True, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    return r.stdout.strip() or None if r.returncode == 0 else None


def cmd_toggle(a, cfg):
    """on/off: the off flag (honoured by any running worker, at once) plus, under systemd, enable/disable --now.
    On Windows `off` disables the scheduled task (the running worker idles on the flag and is not restarted)
    and `on` enables and starts it."""
    flag: Path = cfg["off_file"]
    if a.cmd == "off":
        flag.parent.mkdir(parents=True, exist_ok=True)
        flag.write_text(f"crane-worker off at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    else:
        flag.unlink(missing_ok=True)
    if _systemd_unit():
        subprocess.run(["systemctl", "--user", "disable" if a.cmd == "off" else "enable", "--now", "--quiet", UNIT], check=False)
    elif _win_task():
        verbs = f"Disable-ScheduledTask {TASK}" if a.cmd == "off" else f"Enable-ScheduledTask {TASK}; Start-ScheduledTask {TASK}"
        subprocess.run(["powershell", "-NoProfile", "-Command", verbs + " | Out-Null"], check=False,
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    cmd_status(a, cfg)


def cmd_drain(a, cfg):
    """Lease nothing more, let the running jobs finish, then exit (the service restarts on the installed code)."""
    flag = drain_file(cfg)
    flag.parent.mkdir(parents=True, exist_ok=True)
    flag.write_text(f"crane-worker drain at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
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
    elif st.get("state") == "draining" or drain_file(cfg).exists():
        line = f"draining (running {st.get('job') or 'nothing'})"
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
    sub.add_parser("drain", help="lease nothing more, finish the running jobs, then exit (the service restarts it)")
    sub.add_parser("status", help="on, off or paused (with the reason), and the running jobs")
    a = p.parse_args(argv)
    path = a.config or (config_mod.config_dir() / "worker.yaml")
    provision.apply_tools_env(path)  # the installer's tools (git, ffmpeg, dotnet) on PATH for jobs
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = config_mod.load(path if Path(path).exists() else None)
    if a.cmd == "join":
        return cmd_join(a, cfg)
    if a.cmd in ("on", "off"):
        return cmd_toggle(a, cfg)
    if a.cmd == "status":
        return cmd_status(a, cfg)
    if a.cmd == "drain":
        return cmd_drain(a, cfg)
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
