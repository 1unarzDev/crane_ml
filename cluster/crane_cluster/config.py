"""Worker configuration (YAML) and capability detection."""
from __future__ import annotations

import ctypes
import os
import platform
import shutil
import socket
import subprocess
import sys
from pathlib import Path

from . import miniyaml

OS_NAMES = {"linux": "linux", "windows": "windows", "darwin": "darwin"}
TOOL_TAGS = ("git", "ffmpeg", "dotnet", "docker")


def config_dir() -> Path:
    if sys.platform == "win32":
        return Path(os.environ.get("APPDATA", Path.home())) / "crane-cluster"
    return Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "crane-cluster"


def data_dir() -> Path:
    if sys.platform == "win32":
        return Path(os.environ.get("LOCALAPPDATA", Path.home())) / "crane-cluster"
    return Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "crane-cluster"


DEFAULTS = {
    "coordinator": None,
    "name": None,
    "token_file": None,
    "data_dir": None,
    "poll_s": 30,
    "long_poll_s": 25,   # the lease call waits up to this long for a job (0: plain polling)
    "heartbeat_s": 60,
    "caps": {},
    "policy": {},
    "pause_when": None,          # shell command; exit 0 = pause (see idle.py)
    "pause_when_timeout_s": 20,
    "isolation": {"port_base": 20000, "port_stride": 100, "id_base": 1, "slot": 0},
    "slots": None,               # concurrent jobs per backend (see slots.py); none: one job at a time
    "backends": {"dummy": {"type": "dummy"}},
}


def load(path: str | Path | None) -> dict:
    cfg = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULTS.items()}
    if path:
        user = miniyaml.load_file(path) or {}
        for k, v in user.items():
            cfg[k] = {**cfg[k], **v} if isinstance(cfg.get(k), dict) and isinstance(v, dict) and k != "backends" else v
    env_url = os.environ.get("CRANE_COORDINATOR")
    if env_url:
        cfg["coordinator"] = env_url
    cfg["name"] = cfg["name"] or socket.gethostname()
    cfg["token_file"] = Path(os.path.expanduser(cfg["token_file"] or config_dir() / "token"))
    cfg["data_dir"] = Path(os.path.expanduser(cfg["data_dir"] or data_dir()))
    # `crane-worker off` creates it beside the config; the worker takes no job while it exists.
    cfg["off_file"] = Path(os.path.expanduser(cfg.get("off_file") or (Path(path).parent if path else config_dir()) / "off"))
    return cfg


def _ram_gb() -> float:
    try:
        if sys.platform.startswith("linux"):
            for line in Path("/proc/meminfo").read_text().splitlines():
                if line.startswith("MemTotal:"):
                    return round(int(line.split()[1]) / 1024**2, 1)
        if sys.platform == "darwin":
            return round(int(subprocess.run(["sysctl", "-n", "hw.memsize"], capture_output=True, text=True).stdout) / 1024**3, 1)
        if sys.platform == "win32":
            class MEMSTAT(ctypes.Structure):
                _fields_ = [("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong), ("ullTotalPhys", ctypes.c_ulonglong),
                            ("ullAvailPhys", ctypes.c_ulonglong), ("ullTotalPageFile", ctypes.c_ulonglong),
                            ("ullAvailPageFile", ctypes.c_ulonglong), ("ullTotalVirtual", ctypes.c_ulonglong),
                            ("ullAvailVirtual", ctypes.c_ulonglong), ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]
            m = MEMSTAT()
            m.dwLength = ctypes.sizeof(m)
            ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))  # type: ignore[attr-defined]
            return round(m.ullTotalPhys / 1024**3, 1)
    except (OSError, ValueError):
        pass
    return 0.0


def mem_available_gb() -> float | None:
    """MemAvailable (Linux) or available physical memory (Windows) in GB; None where unknown."""
    try:
        if sys.platform.startswith("linux"):
            for line in Path("/proc/meminfo").read_text().splitlines():
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) / 1024**2
        if sys.platform == "win32":
            class MEMSTAT(ctypes.Structure):
                _fields_ = [("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong)] + \
                           [(n, ctypes.c_ulonglong) for n in ("ullTotalPhys", "ullAvailPhys", "ullTotalPageFile",
                            "ullAvailPageFile", "ullTotalVirtual", "ullAvailVirtual", "ullAvailExtendedVirtual")]
            m = MEMSTAT()
            m.dwLength = ctypes.sizeof(m)
            ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))  # type: ignore[attr-defined]
            return m.ullAvailPhys / 1024**3
    except (OSError, ValueError):
        pass
    return None


def _gpu() -> tuple[str | None, int, float]:
    if not shutil.which("nvidia-smi"):
        return None, 0, 0.0
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout.strip().splitlines()
    except (OSError, subprocess.SubprocessError):
        return None, 0, 0.0
    if not out:
        return None, 0, 0.0
    name, mem = [x.strip() for x in out[0].split(",", 1)]
    return name, len(out), round(float(mem) / 1024, 1)


def _vulkan() -> bool:
    if not shutil.which("vulkaninfo"):
        return False
    try:
        return subprocess.run(["vulkaninfo", "--summary"], capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def detect_caps(cfg: dict, backends: list[str]) -> dict:
    """Detected capabilities with the YAML's ``caps`` overriding any field (e.g. vulkan: false on a shared box)."""
    gpu, n_gpu, vram = _gpu()
    caps = {
        "os": OS_NAMES.get(platform.system().lower(), platform.system().lower()),
        "arch": platform.machine().lower(),
        "cpu": os.cpu_count() or 1,
        "ram_gb": _ram_gb(),
        "gpu": gpu,
        "gpu_count": n_gpu,
        "vram_gb": vram,
        "vulkan": _vulkan(),
        "tags": [],
    }
    own = dict(cfg.get("caps") or {})
    caps.update(own)
    # Tags: the YAML's plus what the host offers (its os, a GPU, Vulkan, the tools on PATH), so a job can ask for them.
    found = [caps["os"]] + [t for t in ("gpu", "vulkan") if caps.get(t)] + [t for t in TOOL_TAGS if shutil.which(t)]
    caps["tags"] = list(dict.fromkeys(list(own.get("tags") or []) + found))
    caps["backends"] = backends
    return caps
