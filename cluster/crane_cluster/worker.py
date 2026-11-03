"""The worker daemon: lease jobs up to its free slots (slots.py), check out their exact refs, run each
backend per seed in its own thread, report metrics and artifacts, heartbeat each job throughout. Repos and
images are updated only while no job runs. ``crane-worker join|run|caps|on|off|drain|stop|status``.

Host guards: a backend whose jobs pass through a host's own admission (a shared machine's heavy-job guard)
names two commands, so the worker never holds a job that guard would keep waiting::

    fastsim_roboboat:
      admit: [~/bin/heavy, probe, sim]     # exit 0: a job started now gets a slot at once (1: not now)
      admit_waited: [~/bin/heavy, waited]  # + the job id: the seconds that job has waited there (1: not waiting)

The backend is leased only while ``admit`` exits 0. A leased job that has still waited ``admit_release_min``
minutes (worker option, default 5) goes back to the queue (a release: no attempt counted). Child
processes get ``CRANE_JOB_ID``, so the guard can tell the jobs apart.

Restarts: ``crane-worker drain`` finishes the running jobs, then exits; ``crane-worker stop`` (and SIGTERM,
which the systemd unit sends to the worker alone) releases them back to the queue at once and exits. Either
way the service or the scheduled task starts it again on the installed code, and no job fails for it.
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

from . import __version__, backends as backend_mod, config as config_mod, gpuwindow, idle, provision
from .slots import Slots
from .api import ApiError, Backoff, Client, unavailable
from .gitcache import GitCache

log = logging.getLogger("crane-worker")


class Stop(Exception):
    pass


def flag_dir(cfg: dict) -> Path:
    """Where drain and stop flags live: beside the off flag (the GPU runner: its gpu/ subdirectory)."""
    return Path(cfg.get("flag_dir") or Path(cfg["off_file"]).parent)


def drain_file(cfg: dict) -> Path:
    """`crane-worker drain` creates it beside the off flag; the worker removes it when it exits drained."""
    return flag_dir(cfg) / "drain"


def stop_file(cfg: dict) -> Path:
    """`crane-worker stop` creates it: release the running jobs now and exit (removed on the way out)."""
    return flag_dir(cfg) / "stop"


GUARD_KEYS = ("admit", "admit_waited")   # host-guard options (module docstring), not the backend's


def guard_cmd(opts: dict | None, key: str) -> list[str] | None:
    cmd = (opts or {}).get(key)
    if not cmd:
        return None
    return [os.path.expanduser(str(c)) for c in ([cmd] if isinstance(cmd, str) else cmd)]


def run_guard(cmd: list[str], timeout: float = 20) -> tuple[int | None, str]:
    """(exit code, stdout) of a host-guard command; (None, why) when it could not run."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        return r.returncode, r.stdout.strip()
    except (OSError, subprocess.SubprocessError) as e:
        return None, str(e)


class Worker:
    gate: gpuwindow.Gate | None = None   # the GPU runner's window and idle gate (set in __init__)

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
        # The GPU runner (--gpu-window): leases only inside its window while the user is away (gpuwindow.py).
        self.gate = gpuwindow.Gate(cfg["gpu"]) if cfg.get("gpu") else None

    def lease_policy(self, waiting: str | None = None) -> dict:
        """The policy sent with a lease; the GPU runner's carries its window for the feed."""
        if not self.gate:
            return self.policy
        return {**self.policy, "window": self.gate.report(waiting=waiting)}

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
        stop_file(self.cfg).unlink(missing_ok=True)   # a stop asked before this process started is done
        versions: dict[str, str | None] = {}
        try:
            self._loop(once, versions)
        finally:
            for t in list(self.jobs.values()):   # shutdown: running jobs release themselves (heartbeat sees stop)
                t.join()

    def _loop(self, once: bool, versions: dict) -> None:
        drain = drain_file(self.cfg)
        while not self.stop.is_set():
            if stop_file(self.cfg).exists():
                # `crane-worker stop`: running jobs go back to the queue at once (heartbeat sees stop), then exit.
                stop_file(self.cfg).unlink(missing_ok=True)
                log.info("stop requested; releasing %s and exiting", ", ".join(sorted(self.jobs)) or "nothing")
                self.stop.set()
                return
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
            if not reason and self.gate:
                reason = self.gate.closed()
            if reason:
                if reason != self.paused:
                    log.info("paused: %s", reason)
                self.paused = reason
                self.note("paused", reason)
                if self.gate:   # a lease for no job: the coordinator keeps the runner's window (feed: "next at")
                    self._safe_post("/v1/lease", {"caps": self.caps, "policy": self.lease_policy(reason), "max_jobs": 0,
                                                  "state": "paused", "client_version": __version__})
                else:
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
            free = self.admitted(self.slots.free())
            if not free:   # every slot that could take a job is busy (or its host guard says not now): wait
                self.freed.wait(float(self.cfg["poll_s"]))
                self.freed.clear()
                continue
            # Long poll while idle: the coordinator holds an empty answer up to wait_s until a job arrives.
            wait_s = 0 if once or self.jobs else float(self.cfg.get("long_poll_s", 25))
            t0 = time.monotonic()
            try:
                # max_jobs: any mix of that many fits, since each backend alone fits that many.
                body = {"caps": self.caps, "policy": self.lease_policy(), "max_jobs": min(free.values()), "backends": sorted(free),
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

    def admitted(self, free: dict[str, int]) -> dict[str, int]:
        """The free backends whose host guard (``admit``) would start a job now; the rest wait for a later round."""
        out, refused = {}, self.__dict__.setdefault("refused", {})   # backend -> why its guard refuses it now
        for b, k in free.items():
            cmd = guard_cmd(self.cfg["backends"].get(b), "admit")
            rc, why = (0, "") if cmd is None else run_guard(cmd)
            if rc is None or rc not in (0, 1):   # a guard that fails to answer never blocks the backend
                log.warning("admit %s: %s (exit %s); leasing it anyway", b, why[-200:], rc)
                rc = 0
            if rc == 0:
                out[b] = k
                if refused.pop(b, None):
                    log.info("host guard admits %s again", b)
            elif refused.get(b) != why:
                refused[b] = why
                log.info("not leasing %s: host guard: %s", b, why or "not now")
        return out

    def guard_waited(self, backend: str, jid: str) -> float | None:
        """Seconds the job has waited in its host guard (``admit_waited``), or None."""
        cmd = guard_cmd(self.cfg["backends"].get(backend), "admit_waited")
        if cmd is None:
            return None
        rc, out = run_guard(cmd + [jid])
        try:
            return float(out.split()[0]) if rc == 0 and out else None
        except ValueError:
            return None

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
            "isolation_id": int(iso["id_base"]) + slot}, client=self.client, env={"CRANE_JOB_ID": jid})
        if self.gate:
            if sys.platform == "win32" and self.gate.opts.get("desktop"):
                ctx.env["CRANE_DESKTOP"] = str(self.gate.opts["desktop"])   # players start there (hiddendesk.py)
            self.gate.arm()
        release_after = 60 * float(self.cfg.get("admit_release_min", 5))
        revoked, released = threading.Event(), threading.Event()
        done = threading.Event()

        def heartbeat():
            period = max(1.0, min(float(self.cfg["heartbeat_s"]), ttl / 3))
            tick, beat = min(1.0, period), time.monotonic() + period
            while not done.wait(tick):
                # The GPU runner: input, an unlock or the window's end gives the job back within a tick.
                why = self.gate.interrupted() if self.gate else None
                if why:
                    log.info("releasing job %s (%s)", jid, why)
                    released.set()
                    backend.cancel()
                    return
                now = self.stop.is_set() or self.cfg["off_file"].exists()  # shutdown or `crane-worker off`: at once
                if not now and time.monotonic() < beat:
                    continue
                beat = time.monotonic() + period
                why = "stop" if self.stop.is_set() else "off" if now else \
                    "user active" if self.policy.get("pause_on_activity") and idle.user_active(self.policy) else \
                    "pause_when" if idle.pause_requested(self.cfg.get("pause_when"), float(self.cfg.get("pause_when_timeout_s", 20))) \
                    else None
                if not why:
                    waited = self.guard_waited(job["backend"], jid)
                    if waited is not None and waited >= release_after:
                        why = f"waited {waited / 60:.0f} min in the host guard"
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


def cmd_stop(a, cfg):
    """Release the running jobs (no attempt counted) and exit now; the service or task restarts the worker."""
    flag = stop_file(cfg)
    flag.parent.mkdir(parents=True, exist_ok=True)
    flag.write_text(f"crane-worker stop at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    print("stop requested: the worker releases its jobs and exits within a few seconds")


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


def no_console_windows() -> None:
    """Windows, the GPU runner: it runs (pythonw, no console) in the user's desktop session, where a console
    program it starts (git, nvidia-smi, a guard) would open a console window on his screen. Every child gets
    CREATE_NO_WINDOW unless it asks for a console of its own."""
    if sys.platform != "win32" or getattr(subprocess.Popen, "_crane_no_window", False):
        return
    orig = subprocess.Popen.__init__
    keep = subprocess.CREATE_NEW_CONSOLE | subprocess.DETACHED_PROCESS

    def init(self, *args, **kw):
        flags = kw.get("creationflags", 0) or 0
        if not flags & keep:
            kw["creationflags"] = flags | subprocess.CREATE_NO_WINDOW
        orig(self, *args, **kw)
    subprocess.Popen.__init__ = init
    subprocess.Popen._crane_no_window = True


def cmd_gpu_check(a, cfg):
    """The GPU runner's one-time check, run in the user's session (task crane-gpu-check) while he is away: a short
    unity benchmark on the hidden desktop, watched every 0.1 s for any window of the player on the visible
    desktop and for the foreground. Writes <data dir>/gpu/gpu-check.json; ok means it rendered unseen."""
    gate = gpuwindow.Gate(cfg["gpu"])
    idle_s, locked = gate.idle_fn(), gate.locked_fn()
    if not a.force and not locked and (idle_s is None or idle_s < 60):
        sys.exit(f"the user is active (idle {idle_s} s, not locked): run it while he is away, or --force")
    if "unity" not in cfg["backends"]:
        sys.exit("gpu-check needs the unity backend in the config")
    from . import hiddendesk
    desk = str(cfg["gpu"].get("desktop") or "")
    work = cfg["data_dir"] / "gpu-check"
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    job = {"job_id": "gpu-check", "backend": "unity", "attempt": 1, "seeds": [1],
           "spec": {"duration": 5, "warmup": 1, "traj": False, "require_valid_water": False}}
    backend = backend_mod.build({"unity": cfg["backends"]["unity"]})["unity"]
    ctx = backend_mod.JobContext(job=job, work_dir=work, isolation={"slot": 16, "port_base": 21600, "isolation_id": 17},
                                 env={"CRANE_JOB_ID": "gpu-check", **({"CRANE_DESKTOP": desk} if desk else {})})
    out = {"desktop": desk or None, "idle_s": idle_s, "locked": locked, "at": time.strftime("%Y-%m-%d %H:%M:%S")}
    result: dict = {}

    def run():
        try:
            backend.prepare(ctx)
            result["run_dir"] = str(backend.run(1))
        except Exception as e:  # reported, not raised: the watch below must finish
            result["error"] = f"{type(e).__name__}: {e}"[:1500]
    t = threading.Thread(target=run, daemon=True)
    t.start()
    pids = lambda: hiddendesk.tree(p.pid) if (p := getattr(backend, "proc", None)) else set()   # noqa: E731
    seen, hidden = {"samples": 0, "visible": 0, "foreground": 0, "errors": 0}, 0
    t0 = time.monotonic()
    while t.is_alive() and time.monotonic() - t0 < 900:
        w = hiddendesk.watch(pids, 1.0)
        for k in seen:
            seen[k] += w[k]
        if desk:   # what the player made where nobody sees it (only the runner's players live there)
            hidden = max(hidden, len(hiddendesk.windows_of(None, desk, visible_only=False)))
    t.join(timeout=5)
    backend.cancel()
    out.update(result, visible_samples=seen["visible"], foreground_samples=seen["foreground"], samples=seen["samples"],
               watch_errors=seen["errors"], hidden_desktop_windows=hidden, player_s=round(time.monotonic() - t0, 1))
    out["ok"] = "run_dir" in result and not seen["visible"] and not seen["foreground"]
    (cfg["data_dir"] / "gpu-check.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


def main(argv=None):
    p = argparse.ArgumentParser(prog="crane-worker", description=__doc__)
    p.add_argument("--config", help="worker YAML (default: <config dir>/worker.yaml if present)")
    p.add_argument("--gpu-window", action="store_true",
                   help="the GPU runner: only the gpu_window backends, in the user's session, inside its window (gpuwindow.py)")
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
    sub.add_parser("stop", help="release the running jobs to the queue now (no attempt counted), then exit (the "
                               "service restarts it)")
    sub.add_parser("status", help="on, off or paused (with the reason), and the running jobs")
    g = sub.add_parser("gpu-check", help="with --gpu-window, in the user's session while he is away: a short player "
                                         "run on the hidden desktop, verified unseen (writes gpu-check.json)")
    g.add_argument("--force", action="store_true", help="run even though the user is active")
    a = p.parse_args(argv)
    path = a.config or (config_mod.config_dir() / "worker.yaml")
    provision.apply_tools_env(path)  # the installer's tools (git, ffmpeg, dotnet) on PATH for jobs
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = config_mod.for_role(config_mod.load(path if Path(path).exists() else None), a.gpu_window)
    if a.gpu_window:
        no_console_windows()
    if a.cmd == "gpu-check":
        if not a.gpu_window:
            sys.exit("gpu-check is the GPU runner's: crane-worker --gpu-window gpu-check")
        return cmd_gpu_check(a, cfg)
    if a.cmd == "join":
        return cmd_join(a, cfg)
    if a.cmd in ("on", "off"):
        return cmd_toggle(a, cfg)
    if a.cmd == "status":
        return cmd_status(a, cfg)
    if a.cmd == "drain":
        return cmd_drain(a, cfg)
    if a.cmd == "stop":
        return cmd_stop(a, cfg)
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
