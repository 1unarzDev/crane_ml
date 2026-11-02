"""Runs a templated command per seed, inside a container image when one is configured.

Options (worker YAML, overridable per job through spec.params where noted):
  command:   string or list; str.format template (required)
  image:     container image; omit to run on the host
  docker:    container CLI (default "docker"; "podman" works too)
  gpus:      pass "--gpus all" to the container (default false)
  run_args:  extra container run arguments (list)
  env:       mapping of extra environment variables (values are templates)
  timeout_s: per-seed timeout (default 3600)
  pull:      pull the image between jobs (default true when image is set)
  metrics_file: JSON file in the run dir with the metrics (default metrics.json)

Template fields: seed, run_dir, work_dir, job_id, arm, scenario, spec (dict,
e.g. {spec[gain]}), repos (dict name -> path, e.g. {repos[org/sim]}),
port_base, isolation_id. Inside a container, run_dir is /crane/run and each
repo is mounted read-only at /crane/src/<last path part of its name>.
"""
from __future__ import annotations

import os
import shlex
import signal
import subprocess
from pathlib import Path

from . import Backend, register


class _Fields(dict):
    def __missing__(self, key):
        raise KeyError(f"unknown template field {{{key}}}")


@register("command")
class CommandBackend(Backend):
    def __init__(self, name, options):
        super().__init__(name, options)
        if not options.get("command"):
            raise ValueError(f"backend {name}: 'command' is required")
        self.proc: subprocess.Popen | None = None
        self.container: str | None = None

    @property
    def image(self):
        return self.options.get("image")

    def update(self):
        if self.image and self.options.get("pull", True):
            subprocess.run([self.options.get("docker", "docker"), "pull", "-q", self.image], check=False, timeout=1800)

    def _fields(self, seed: int, run_dir: Path) -> dict:
        ctx = self.ctx
        if self.image:
            repos = {n: f"/crane/src/{n.rstrip('/').split('/')[-1]}" for n in ctx.repos}
            run, work = "/crane/run", "/crane/run"
        else:
            repos = {n: str(p) for n, p in ctx.repos.items()}
            run, work = str(run_dir), str(ctx.work_dir)
        return _Fields(seed=seed, run_dir=run, work_dir=work, job_id=ctx.job["job_id"], arm=ctx.job.get("arm") or "",
                       scenario=ctx.spec.get("scenario") or "", spec=ctx.spec, repos=repos,
                       port_base=ctx.isolation.get("port_base", 0), isolation_id=ctx.isolation.get("isolation_id", 0))

    def argv(self, seed: int, run_dir: Path) -> list[str]:
        f = self._fields(seed, run_dir)
        cmd = self.options["command"]
        env = {k: str(v).format_map(f) for k, v in (self.options.get("env") or {}).items()}
        env.update({"CRANE_SEED": str(seed), "CRANE_JOB_ID": f["job_id"], "CRANE_RUN_DIR": f["run_dir"],
                    "CRANE_PORT_BASE": str(f["port_base"]), "CRANE_ISOLATION_ID": str(f["isolation_id"])})
        if not self.image:
            self._env = env
            return [c.format_map(f) for c in cmd] if isinstance(cmd, list) else shlex.split(cmd.format_map(f), posix=os.name != "nt")
        self._env = {}
        self.container = f"crane-{self.ctx.job['job_id']}-{seed}"[:120]
        argv = [self.options.get("docker", "docker"), "run", "--rm", "--name", self.container,
                "-v", f"{run_dir}:/crane/run"]
        for n, p in self.ctx.repos.items():
            argv += ["-v", f"{p}:/crane/src/{n.rstrip('/').split('/')[-1]}:ro"]
        for k, v in env.items():
            argv += ["-e", f"{k}={v}"]
        if self.options.get("gpus"):
            argv += ["--gpus", "all"]
        argv += list(self.options.get("run_args") or [])
        argv.append(self.image)
        if isinstance(cmd, list):
            argv += [c.format_map(f) for c in cmd]
        else:
            argv += ["sh", "-c", cmd.format_map(f)]
        return argv

    def run(self, seed: int) -> Path:
        run_dir = self.run_dir(seed)
        argv = self.argv(seed, run_dir)
        log = open(run_dir / "command.log", "wb")
        try:
            kw = {"start_new_session": True} if os.name != "nt" else {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
            self.proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, cwd=self.ctx.work_dir,
                                         env={**os.environ, **self.ctx.env, **self._env}, **kw)
            try:
                rc = self.proc.wait(timeout=float(self.options.get("timeout_s", 3600)))
            except subprocess.TimeoutExpired:
                self.cancel()
                raise RuntimeError(f"seed {seed} timed out")
        finally:
            log.close()
            self.proc = None
        if rc != 0:
            tail = (run_dir / "command.log").read_bytes()[-2000:].decode(errors="replace")
            raise RuntimeError(f"command exited {rc}: {tail}")
        return run_dir

    def artifacts(self, run_dir):
        out = super().artifacts(run_dir)
        log = Path(run_dir) / "command.log"
        return out + ([(log, "command.log", "raw")] if log.is_file() else [])

    def cancel(self):
        if self.container and self.image:
            subprocess.run([self.options.get("docker", "docker"), "kill", self.container], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        p = self.proc
        if p and p.poll() is None:
            try:
                if os.name != "nt":
                    os.killpg(p.pid, signal.SIGTERM)
                else:
                    p.terminate()
            except ProcessLookupError:
                pass
