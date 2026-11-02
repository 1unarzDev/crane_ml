"""Availability on shared machines: low priority, quiet hours, user activity.

Policy (worker YAML ``policy``):
  nice: 10                       # lower the daemon's (and its children's) CPU priority
  quiet_hours: ["09:00-17:00"]   # local times when no new job starts; a range may wrap midnight
  quiet_days: [mon, tue, ...]    # optional: quiet hours apply only on these days (default: every day)
  pause_on_activity: true        # no new job while the user is active; a running job is released
  idle_seconds: 300              # how long without input counts as idle
  max_est_minutes / require_tags # matched by the coordinator
"""
from __future__ import annotations

import ctypes
import datetime as dt
import os
import re
import shutil
import subprocess
import sys

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
            class LASTINPUTINFO(ctypes.Structure):
                _fields_ = [("cbSize", ctypes.c_uint), ("dwTime", ctypes.c_uint)]
            info = LASTINPUTINFO()
            info.cbSize = ctypes.sizeof(info)
            if ctypes.windll.user32.GetLastInputInfo(ctypes.byref(info)):  # type: ignore[attr-defined]
                return (ctypes.windll.kernel32.GetTickCount() - info.dwTime) / 1000.0  # type: ignore[attr-defined]
            return None
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
        if p.get("IdleHint") != "yes":
            return 0.0
        since = int(p.get("IdleSinceHint", "0") or 0)
        idle.append(max(0.0, dt.datetime.now().timestamp() - since / 1e6) if since else 0.0)
    return min(idle) if idle else None


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


def unavailable_reason(policy: dict, now: dt.datetime | None = None) -> str | None:
    """Why no new job should start right now, or None."""
    if in_quiet_hours(policy, now):
        return "quiet hours"
    if user_active(policy):
        return "user active"
    return None
