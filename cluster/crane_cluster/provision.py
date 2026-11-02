"""Provision a worker host with what its backends need, user-level and idempotent (re-running upgrades).

    python -m crane_cluster.provision --plan install/provision.yaml --tools DIR [--config worker.yaml]
                                      [--player ZIP|DIR|URL] [--unity-managed DIR|DLL|URL] [--no-repos]

The installers run it. The plan names tools (git, ffmpeg, dotnet), repos (mirrored in the worker's
git cache and checked out at their ref under <data>/repos/<name>) and backends to add to worker.yaml
once their requirements are met. Tools land in DIR; their PATH entries and environment go to
tools.json beside the worker config, which the worker applies at start (``apply_tools_env``).
Nothing needs an administrator and nothing changes the system PATH.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path

from . import config as config_mod
from . import miniyaml
from .gitcache import GitCache

WIN = sys.platform == "win32"
UA = {"User-Agent": "crane-cluster-installer"}


def apply_tools_env(config_path) -> dict:
    """Puts tools.json (beside the worker config) into os.environ: its PATH entries first, then its env."""
    f = Path(config_path).parent / "tools.json"
    try:
        t = json.loads(f.read_text())
    except (OSError, ValueError):
        return {}
    have = os.environ.get("PATH", "").split(os.pathsep)
    os.environ["PATH"] = os.pathsep.join([p for p in t.get("path", []) if p not in have] + have)
    os.environ.update({k: str(v) for k, v in (t.get("env") or {}).items()})
    return t


def _get(url: str) -> bytes:
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=600) as r:
        return r.read()


def latest_asset(repo: str, pattern: str, tag: str = "latest") -> tuple[str, str]:
    """(name, url) of the matching asset of a GitHub release, the highest version when several match."""
    path = "releases/latest" if tag == "latest" else f"releases/tags/{tag}"
    rel = json.loads(_get(f"https://api.github.com/repos/{repo}/{path}"))
    hits = [(a["name"], a["browser_download_url"]) for a in rel.get("assets", []) if re.fullmatch(pattern, a["name"])]
    if not hits:
        raise RuntimeError(f"{repo} {rel.get('tag_name')}: no asset matches {pattern}")
    key = lambda h: [int(x) for x in re.findall(r"\d+", h[0])]
    return max(hits, key=key)


def install_archive(name: str, url: str, dest: Path) -> bool:
    """Unpacks the archive at url into dest (its single top folder flattened); False when dest already has it."""
    stamp = dest / ".crane-version"
    if stamp.is_file() and stamp.read_text().strip() == name:
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=dest.parent) as tmp:
        arc, out = Path(tmp) / name, Path(tmp) / "x"
        arc.write_bytes(_get(url))
        if name.endswith(".zip"):
            with zipfile.ZipFile(arc) as z:
                z.extractall(out)
        else:
            with tarfile.open(arc) as t:
                t.extractall(out)
        kids = list(out.iterdir())
        top = kids[0] if len(kids) == 1 and kids[0].is_dir() else out
        (top / ".crane-version").write_text(name + "\n")
        old = dest.with_name(dest.name + ".old")
        shutil.rmtree(old, ignore_errors=True)
        if dest.exists():
            dest.rename(old)
        top.rename(dest)
        shutil.rmtree(old, ignore_errors=True)
    return True


def ensure_release(repo: str, pattern: str, dest: Path, tag: str = "latest") -> tuple[str, str]:
    """(asset, "installed"|"current") for the latest matching release asset in dest; offline, what dest has."""
    try:
        name, url = latest_asset(repo, pattern, tag)
    except OSError:
        stamp = dest / ".crane-version"
        if not stamp.is_file():
            raise
        return stamp.read_text().strip(), "kept (GitHub unreachable)"
    return name, "installed" if install_archive(name, url, dest) else "current"


def _run(cmd, **kw) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=kw.pop("timeout", 60), **kw)
    except (OSError, subprocess.SubprocessError):
        return ""
    return r.stdout.strip() if r.returncode == 0 else ""


def _outside(exe: str | None, tools: Path) -> bool:
    return bool(exe) and not Path(exe).resolve().is_relative_to(tools.resolve())


class Provisioner:
    def __init__(self, plan: dict, tools: Path, cfg_path: Path):
        self.plan, self.tools, self.cfg_path = plan, tools, cfg_path
        self.cfg = config_mod.load(cfg_path if cfg_path.exists() else None)
        self.path: list[str] = []
        self.env: dict[str, str] = {}
        self.have: set[str] = set()
        self.subst = {"python": Path(sys.executable).as_posix(), "repos": (self.cfg["data_dir"] / "repos").as_posix()}
        self.report: list[tuple[str, str]] = []

    def note(self, item: str, outcome: str) -> None:
        self.report.append((item, outcome))
        print(f"  {item:<14} {outcome}", flush=True)

    def add_path(self, d: Path) -> None:
        self.path.append(str(d))
        os.environ["PATH"] = str(d) + os.pathsep + os.environ.get("PATH", "")

    # ---- tools
    def tool_git(self, opt: dict) -> None:
        mingit = self.tools / "MinGit"
        if WIN and not _outside(shutil.which("git"), self.tools):
            name, how = ensure_release("git-for-windows/git", r"MinGit-[\d.]+-64-bit\.zip", mingit)
            self.add_path(mingit / "cmd")
            git = str(mingit / "cmd" / "git.exe")
            # MinGit's own etc/gitconfig: checkouts byte-identical to Linux ones (content hashes such as a
            # course generator's source_sha256 must match across hosts), and deep Unity paths
            for key, value in (("core.autocrlf", "false"), ("core.longpaths", "true")):
                subprocess.run([git, "config", "--system", key, value], capture_output=True)
            self.note("git", f"{how} {name[:-4]} at {mingit}")
        elif shutil.which("git"):
            self.note("git", f"system {_run(['git', '--version'])}")
        else:
            return self.note("git", "MISSING: owner step: install git (e.g. sudo apt install git)")
        self.have.add("git")

    def tool_ffmpeg(self, opt: dict) -> None:
        dest = self.tools / "ffmpeg"
        if _outside(shutil.which("ffmpeg"), self.tools):
            self.have.add("ffmpeg")
            return self.note("ffmpeg", f"system {shutil.which('ffmpeg')}")
        plat = {"Windows": "win64", "Linux": "linux64"}.get(platform.system())
        if plat and platform.machine().lower() in ("aarch64", "arm64"):
            plat = "linuxarm64" if plat == "linux64" else None
        if not plat:
            return self.note("ffmpeg", f"skipped: no static build for {platform.system()} {platform.machine()}")
        name, how = ensure_release("BtbN/FFmpeg-Builds", rf"ffmpeg-n\d+\.\d+-latest-{plat}-gpl-\d+\.\d+\.(zip|tar\.xz)", dest, tag="latest")
        self.add_path(dest / "bin")
        self.have.add("ffmpeg")
        self.note("ffmpeg", f"{how} {name.split('-latest')[0]} at {dest}")

    def tool_dotnet(self, opt: dict) -> None:
        want = [int(x) for x in str(opt.get("min", "0")).split(".")]
        ok = lambda v: v[0] == want[0] and v >= want
        sdks = lambda exe: [[int(x) for x in re.findall(r"\d+", l.split()[0])[:3]] for l in _run([exe, "--list-sdks"]).splitlines() if l.strip()]
        system = shutil.which("dotnet")
        if _outside(system, self.tools) and any(ok(v) for v in sdks(system)):
            self.have.add("dotnet")
            return self.note("dotnet", f"system {system}")
        dest = self.tools / "dotnet"
        script = "dotnet-install.ps1" if WIN else "dotnet-install.sh"
        with tempfile.TemporaryDirectory() as tmp:
            s = Path(tmp) / script
            s.write_bytes(_get(f"https://dot.net/v1/{script}"))
            cmd = (["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(s), "-Channel", opt["channel"], "-InstallDir", str(dest), "-NoPath"]
                   if WIN else ["bash", str(s), "--channel", opt["channel"], "--install-dir", str(dest), "--no-path"])
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
        exe = str(dest / ("dotnet.exe" if WIN else "dotnet"))
        got = [v for v in sdks(exe) if ok(v)]
        if r.returncode or not got:
            return self.note("dotnet", f"FAILED: {(r.stdout + r.stderr).strip()[-300:]}")
        self.add_path(dest)
        self.env.update({"DOTNET_ROOT": str(dest), "DOTNET_CLI_TELEMETRY_OPTOUT": "1", "DOTNET_NOLOGO": "1"})
        self.have.add("dotnet")
        self.note("dotnet", f"sdk {'.'.join(map(str, max(got)))} at {dest}")

    def vulkan(self) -> None:
        exe = shutil.which("vulkaninfo")
        out = _run([exe, "--summary"], timeout=30) if exe else ""
        gpus = sorted(set(re.findall(r"deviceName\s*=\s*(.+)", out)))
        if gpus:
            self.have.add("vulkan")
            return self.note("vulkan", "ok: " + "; ".join(g.strip() for g in gpus))
        loader = Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32" / "vulkan-1.dll" if WIN else None
        self.note("vulkan", "loader present, vulkaninfo found no device (session 0 or no GPU)" if loader and loader.exists()
                  else "none: GPU jobs need a Vulkan driver (owner step on this host)")

    # ---- Unity pieces (sources: a configured path or URL; none = skipped)
    def _fetch(self, src: str, dest: Path) -> Path:
        if re.match(r"https?://", src):
            name = src.rstrip("/").rsplit("/", 1)[-1]
            if name.endswith((".zip", ".tar.gz", ".tar.xz", ".tgz")):
                install_archive(name, src, dest)
                return dest
            dest.mkdir(parents=True, exist_ok=True)
            (dest / name).write_bytes(_get(src))
            return dest / name
        p = Path(os.path.expanduser(src))
        if p.is_file() and p.suffix == ".zip":
            install_archive(p.name, p.resolve().as_uri(), dest)
            return dest
        return p

    def unity_managed(self, src: str | None) -> None:
        dll = Path("UnityEngine") / "UnityEngine.CoreModule.dll"
        own = self.tools / "unity-managed"
        if src:
            got = self._fetch(src, own)
            if got.is_file() and got.name == dll.name and got.parent != own / "UnityEngine":
                (own / "UnityEngine").mkdir(parents=True, exist_ok=True)
                shutil.copy2(got, own / dll)
            elif got.is_dir() and got != own:
                hit = next(iter(sorted(got.rglob(dll.name))), None)
                if hit:
                    (own / "UnityEngine").mkdir(parents=True, exist_ok=True)
                    shutil.copy2(hit, own / dll)
        root = self.cfg["data_dir"] / "players"
        mono = next(iter(sorted(root.glob(f"*/*_Data/Managed/{dll.name}"))), None) if root.is_dir() else None
        if mono and not (own / dll).is_file():  # a Mono player carries the same assembly
            (own / "UnityEngine").mkdir(parents=True, exist_ok=True)
            shutil.copy2(mono, own / dll)
        candidates = [own] + [Path(p) for p in (os.environ.get("UnityManaged"),) if p]
        for hub in (Path.home() / "Unity/Hub/Editor", Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "Unity/Hub/Editor"):
            candidates += sorted(hub.glob("*/Editor/Data/Managed"), reverse=True) if hub.is_dir() else []
        hit = next((c for c in candidates if (c / dll).is_file()), None)
        if not hit:
            return self.note("unity_managed", "skipped: no UnityEngine.CoreModule.dll (give --unity-managed DIR|DLL|URL "
                                              "or install the Unity editor); fastsim_roboboat is not offered")
        self.env["UnityManaged"] = str(hit)
        self.have.add("unity_managed")
        self.note("unity_managed", str(hit))

    def player(self, src: str | None) -> None:
        root = self.cfg["data_dir"] / "players"
        if src:
            name = re.sub(r"\.(zip|tar\.gz|tar\.xz|tgz)$", "", Path(src.rstrip("/")).name)
            got = self._fetch(src, root / name)
            if got.is_dir() and not got.is_relative_to(root):
                shutil.copytree(got, root / name, dirs_exist_ok=True)
        exes = sorted(root.glob("*/CRANE.exe" if WIN else "*/*.x86_64")) if root.is_dir() else []
        if not exes:
            return self.note("player", "skipped: no source (give --player ZIP|DIR|URL; a player is over the "
                                       "coordinator's artifact limit, so none is published there)")
        self.subst["player"] = exes[0].as_posix()
        self.have.add("player")
        self.note("player", str(exes[0]))

    # ---- repos through the worker's git cache
    def repos(self) -> None:
        if "git" not in self.have:
            return self.note("repos", "skipped: no git")
        cache = GitCache(self.cfg["data_dir"] / "git")
        for name, r in (self.plan.get("repos") or {}).items():
            try:
                cache.refresh(name, r["url"])
                sha = cache.checkout(name, r["url"], r.get("ref", "main"), self.cfg["data_dir"] / "repos" / name,
                                     sparse=r.get("sparse"))
            except (RuntimeError, OSError) as e:
                self.note(f"repo {name}", f"FAILED: {e}")
                continue
            self.have.add(f"repo:{name}")
            self.note(f"repo {name}", f"{sha[:12]} at {self.subst['repos']}/{name}")

    # ---- backends added to worker.yaml
    def backends(self) -> None:
        present = set(self.cfg.get("backends") or {}) if self.cfg_path.exists() else set()
        add = {}
        for name, b in (self.plan.get("backends") or {}).items():
            missing = [r for r in b.get("requires", []) if r not in self.have]
            if name in present:
                self.note(f"backend {name}", "configured (kept)")
            elif missing:
                self.note(f"backend {name}", f"not offered: missing {', '.join(missing)}")
            else:
                add[name] = _fill(b["config"], self.subst)
        if not add:
            return
        block = "".join(f"  {k}:\n{_dump(v, 4)}" for k, v in add.items())
        text = self.cfg_path.read_text(encoding="utf-8-sig") if self.cfg_path.exists() else ""
        m = re.search(r"^backends:[ \t]*(#.*)?\n", text, re.M)
        if m:
            text = text[:m.end()] + block + text[m.end():]
        elif re.search(r"^backends:", text, re.M):
            return self.note("backends", "worker.yaml's backends: is not a block mapping; add these by hand:\n" + block)
        else:
            text = text.rstrip("\n") + "\nbackends:\n  dummy: {type: dummy}\n" + block
        self.cfg_path.write_text(text, encoding="utf-8")
        for k in add:
            self.note(f"backend {k}", "added to worker.yaml")

    def write(self) -> Path:
        f = self.cfg_path.parent / "tools.json"
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(json.dumps({"path": self.path, "env": self.env, "have": sorted(self.have)}, indent=1) + "\n")
        return f


def _fill(v, subst):
    if isinstance(v, dict):
        return {k: _fill(x, subst) for k, x in v.items()}
    if isinstance(v, list):
        return [_fill(x, subst) for x in v]
    return v.format_map(subst) if isinstance(v, str) else v


def _dump(d: dict, indent: int) -> str:
    pad, out = " " * indent, ""
    for k, v in d.items():
        if isinstance(v, dict):
            out += f"{pad}{k}:\n{_dump(v, indent + 2)}"
        else:
            out += f"{pad}{k}: {json.dumps(v)}\n"
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="crane-provision", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--plan", required=True)
    p.add_argument("--tools", required=True, help="where tools are installed (user-level)")
    p.add_argument("--config", default=str(config_mod.config_dir() / "worker.yaml"))
    p.add_argument("--player", help="a Unity player: zip, folder or URL")
    p.add_argument("--unity-managed", help="UnityEngine.CoreModule.dll: a folder holding it, the dll, or a URL")
    p.add_argument("--no-repos", action="store_true")
    a = p.parse_args(argv)
    plan = miniyaml.load_file(a.plan) or {}
    pv = Provisioner(plan, Path(os.path.expanduser(a.tools)), Path(a.config))
    print(f"provisioning {pv.cfg['name']} (tools in {pv.tools})", flush=True)
    for name, opt in (plan.get("tools") or {}).items():
        try:
            getattr(pv, f"tool_{name}")(opt or {})
        except Exception as e:  # one tool failing (network, a moved asset) never stops the rest
            pv.note(name, f"FAILED: {e}")
    pv.vulkan()
    pv.player(a.player)
    pv.unity_managed(a.unity_managed)
    if not a.no_repos:
        pv.repos()
    pv.backends()
    print(f"  wrote {pv.write()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
