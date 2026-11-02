"""Git cache: one bare mirror per repo, and per-job checkouts at exact refs.

Mirrors are fetched only between jobs (``refresh``) or when a job names a
commit the mirror lacks; a running job's checkout never changes under it.
Git LFS files stay pointers (the mirror holds no LFS objects to smudge from).
"""
from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
from pathlib import Path

SHA = re.compile(r"^[0-9a-f]{40}$")


def _git(*args, cwd=None, timeout=1800) -> str:
    r = subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, timeout=timeout,
                       env={**_base_env(), "GIT_TERMINAL_PROMPT": "0", "GIT_LFS_SKIP_SMUDGE": "1"})
    if r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()[-500:]}")
    return r.stdout.strip()


def _base_env():
    import os
    return {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}


class GitCache:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def mirror_path(self, name: str) -> Path:
        slug = re.sub(r"[^A-Za-z0-9._-]", "_", name)[-60:]
        return self.root / f"{slug}-{hashlib.sha1(name.encode()).hexdigest()[:8]}.git"

    def ensure_mirror(self, name: str, url: str) -> Path:
        m = self.mirror_path(name)
        if not m.exists():
            tmp = m.with_suffix(".tmp")
            shutil.rmtree(tmp, ignore_errors=True)
            _git("clone", "--mirror", "--quiet", url, str(tmp))
            tmp.rename(m)
        return m

    def refresh(self, name: str, url: str) -> None:
        """Between jobs: fetch everything new into the mirror."""
        _git("remote", "update", "--prune", cwd=self.ensure_mirror(name, url))

    def resolve(self, name: str, url: str, ref: str) -> str:
        """The commit sha for ref, fetching once if the mirror does not have it yet."""
        m = self.ensure_mirror(name, url)
        for attempt in range(2):
            try:
                return _git("rev-parse", "--verify", "--quiet", f"{ref}^{{commit}}", cwd=m)
            except RuntimeError:
                if attempt:
                    raise RuntimeError(f"{name}: ref {ref} not found") from None
                if SHA.match(ref):
                    try:
                        _git("fetch", "--quiet", "origin", ref, cwd=m)  # servers allowing reachable-sha fetch
                        continue
                    except RuntimeError:
                        pass
                _git("remote", "update", "--prune", cwd=m)
        raise AssertionError

    def checkout(self, name: str, url: str, ref: str, dest: Path) -> str:
        """A fresh, detached checkout of ref at dest, sharing objects with the mirror. Returns the sha."""
        sha = self.resolve(name, url, ref)
        dest = Path(dest)
        shutil.rmtree(dest, ignore_errors=True)
        dest.parent.mkdir(parents=True, exist_ok=True)
        _git("clone", "--quiet", "--shared", "--no-checkout", str(self.mirror_path(name)), str(dest))
        _git("checkout", "--quiet", "--detach", sha, cwd=dest)
        return sha
