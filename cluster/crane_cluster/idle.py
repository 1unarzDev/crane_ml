"""Availability on shared machines: low priority, quiet hours, user activity.

Policy (worker YAML ``policy``):
  nice: 10                       # lower the daemon's (and its children's) CPU priority
  quiet_hours: ["09:00-17:00"]   # local times when no new job starts; a range may wrap midnight
  quiet_days: [mon, tue, ...]    # optional: quiet hours apply only on these days (default: every day)
  pause_on_activity: true        # no new job while the user is active; a running job is released
                                 # (Windows session 0, e.g. an S4U task: crane_cluster.idleprobe on the desktop)
  idle_seconds: 300              # how long without input counts as idle (Linux: keyboard/mouse
                                 # events from /dev/input when readable, e.g. the input group; else
                                 # xprintidle on X11; else logind's IdleHint)
  max_est_minutes / require_tags # matched by the coordinator

Top-level ``pause_when: <shell command>`` is the host's own pause rule: the
worker runs it between jobs and on every heartbeat, and exit 0 means pause (a
running job's lease is released without an attempt penalty, no new job is
leased) until the command exits nonzero. A command that hangs past
``pause_when_timeout_s`` (default 20) counts as pause.
"""
from __future__ import annotations

import ctypes
import datetime as dt
import glob
import json
import os
import re
import select
import shutil
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

from . import config

DAYS = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
_RANGE = re.compile(r"^(\d{1,2}):(\d{2})-(\d{1,2}):(\d{2})$")


def in_quiet_hours(policy: dict, now: dt.datetime | None = None) -> bool:
    now = now or dt.datetime.now()
    days = [d.lower()[:3] for d in policy.get("quiet_days") or DAYS]
    minute = now.hour * 60 + now.minute
    for spec in policy.get("quiet_hours") or []:
        m = _RANGE.match(spec.strip())
        if not m:
            raise ValueError(f"quiet_hours entry {spec!r} is not HH:MM-HH:MM")
        a = int(m[1]) * 60 + int(m[2])
        b = int(m[3]) * 60 + int(m[4])
        if a <= b:
            if a <= minute < b and DAYS[now.weekday()] in days:
                return True
        else:  # wraps midnight: the part after midnight belongs to the previous day's window
            if minute >= a and DAYS[now.weekday()] in days:
                return True
            if minute < b and DAYS[(now.weekday() - 1) % 7] in days:
                return True
    return False


def user_idle_seconds() -> float | None:
    """Seconds since the last keyboard/mouse input, or None where it can't be detected."""
    try:
        if sys.platform == "win32":
            if windows_session_id() == 0:  # a service-like worker (S4U task): the desktop's probe tells
                return probe_idle_seconds()
            class LASTINPUTINFO(ctypes.Structure):
                _fields_ = [("cbSize", ctypes.c_uint), ("dwTime", ctypes.c_uint)]
            info = LASTINPUTINFO()
            info.cbSize = ctypes.sizeof(info)
            if ctypes.windll.user32.GetLastInputInfo(ctypes.byref(info)):  # type: ignore[attr-defined]
                return (ctypes.windll.kernel32.GetTickCount() - info.dwTime) / 1000.0  # type: ignore[attr-defined]
            return None
        if sys.platform.startswith("linux"):
            seconds = _INPUT.idle_seconds()
            if seconds is not None:
                return seconds
        if sys.platform == "darwin":
            out = subprocess.run(["ioreg", "-c", "IOHIDSystem"], capture_output=True, text=True, timeout=5).stdout
            m = re.search(r'"HIDIdleTime" = (\d+)', out)
            return int(m[1]) / 1e9 if m else None
        if os.environ.get("DISPLAY") and shutil.which("xprintidle"):
            out = subprocess.run(["xprintidle"], capture_output=True, text=True, timeout=5).stdout.strip()
            return int(out) / 1000.0 if out.isdigit() else None
        return _logind_idle()
    except (OSError, subprocess.SubprocessError, ValueError):
        return None


def windows_session_id() -> int:
    """This process's Windows session; 0 is the services session, which sees no one's input."""
    sid = ctypes.c_ulong()
    ctypes.windll.kernel32.ProcessIdToSessionId(os.getpid(), ctypes.byref(sid))  # type: ignore[attr-defined]
    return sid.value


def probe_path() -> Path:
    return config.data_dir() / "input-idle.json"


def probe_idle_seconds(path: Path | None = None, now: float | None = None, stale_s: float = 120) -> float | None:
    """Idle seconds that ``crane_cluster.idleprobe`` reports from the interactive desktop, aged to now;
    None without a fresh report (nobody logged in, or no probe running)."""
    try:
        r = json.loads((path or probe_path()).read_text())
        age = (now if now is not None else time.time()) - float(r["at"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return None if age > stale_s or age < -stale_s else float(r["idle_s"]) + max(0.0, age)


def _logind_idle() -> float | None:
    """systemd-logind's IdleHint for graphical sessions (desktop environments set it)."""
    if not shutil.which("loginctl"):
        return None
    sessions = subprocess.run(["loginctl", "list-sessions", "--no-legend"], capture_output=True, text=True, timeout=5).stdout
    idle = []
    for line in sessions.splitlines():
        sid = line.split()[0] if line.split() else None
        if not sid:
            continue
        props = subprocess.run(["loginctl", "show-session", sid, "-p", "Type", "-p", "IdleHint", "-p", "IdleSinceHint"],
                               capture_output=True, text=True, timeout=5).stdout
        p = dict(x.split("=", 1) for x in props.splitlines() if "=" in x)
        if p.get("Type") not in ("x11", "wayland"):
            continue
        since = int(p.get("IdleSinceHint", "0") or 0)
        if p.get("IdleHint") != "yes":
            if not since:  # nothing on this desktop maintains the hint: unknown, not "always active"
                continue
            return 0.0
        idle.append(max(0.0, dt.datetime.now().timestamp() - since / 1e6) if since else 0.0)
    return min(idle) if idle else None


class _InputWatch:
    """Linux: the time of the last key, button, pointer or touch event on any readable
    /dev/input/event* device (read without grabbing; event contents are discarded).
    Started on first use; until then, and when no device is readable, idle time is unknown."""
    EVENT = struct.Struct("llHHi")
    USER_TYPES = {1, 2, 3}  # EV_KEY, EV_REL, EV_ABS

    def __init__(self):
        self.started = self.last = None
        self.readable = False
        self.lock = threading.Lock()

    def idle_seconds(self) -> float | None:
        with self.lock:
            if self.started is None:
                self.started = self.last = time.monotonic()
                self.readable = bool(self._open())
                if self.readable:
                    threading.Thread(target=self._watch, daemon=True).start()
        return time.monotonic() - self.last if self.readable else None

    def _open(self) -> dict:
        fds = {}
        for path in sorted(glob.glob("/dev/input/event*")):
            try:
                fds[os.open(path, os.O_RDONLY | os.O_NONBLOCK)] = path
            except OSError:
                pass
        return fds

    def _watch(self) -> None:
        fds, rescan = self._open(), time.monotonic() + 60
        while True:
            if time.monotonic() > rescan:  # hotplugged keyboards and mice
                for fd in fds:
                    os.close(fd)
                fds, rescan = self._open(), time.monotonic() + 60
            ready = select.select(list(fds), [], [], 5)[0] if fds else time.sleep(5) or []
            for fd in ready:
                try:
                    data = os.read(fd, self.EVENT.size * 64)
                except OSError:
                    continue
                for off in range(0, len(data) - self.EVENT.size + 1, self.EVENT.size):
                    if self.EVENT.unpack_from(data, off)[2] in self.USER_TYPES:
                        self.last = time.monotonic()
                        break


_INPUT = _InputWatch()


def user_active(policy: dict) -> bool:
    if not policy.get("pause_on_activity"):
        return False
    idle = user_idle_seconds()
    return idle is not None and idle < float(policy.get("idle_seconds", 300))


def lower_priority(policy: dict) -> None:
    """Applies the policy's nice level to this process; children inherit it."""
    nice = int(policy.get("nice", 10))
    if nice <= 0:
        return
    if hasattr(os, "nice"):
        try:
            os.nice(nice)
            if sys.platform.startswith("linux") and shutil.which("ionice"):
                subprocess.run(["ionice", "-c", "3", "-p", str(os.getpid())], check=False, capture_output=True)
        except OSError:
            pass
    elif sys.platform == "win32":
        BELOW_NORMAL = 0x00004000
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), BELOW_NORMAL)  # type: ignore[attr-defined]


def pause_requested(command: str | None, timeout_s: float = 20) -> bool:
    """True when the host's ``pause_when`` shell command exits 0 (or hangs past timeout_s)."""
    if not command:
        return False
    try:
        return subprocess.run(command, shell=True, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=timeout_s).returncode == 0
    except subprocess.TimeoutExpired:
        return True
    except OSError:
        return False


def unavailable_reason(policy: dict, now: dt.datetime | None = None, cfg: dict | None = None) -> str | None:
    """Why no new job should start right now, or None. ``cfg`` supplies the off flag and pause_when."""
    if cfg and cfg.get("off_file") and Path(cfg["off_file"]).exists():
        return "off"
    if cfg and pause_requested(cfg.get("pause_when"), float(cfg.get("pause_when_timeout_s", 20))):
        return "pause_when"
    if in_quiet_hours(policy, now):
        return "quiet hours"
    if user_active(policy):
        return "user active"
    return None
