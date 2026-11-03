"""The GPU window: when a worker's gpu lane may lease GPU jobs on someone else's PC.

A Windows worker in session 0 (an S4U task) cannot render: Unity's D3D12 device fails there (887a0022) and
D3D11 hangs. So a Windows worker on a shared PC runs as ONE hidden task in the logged-on user's session
(install.ps1), with two lanes (config.lanes): the cpu lane (every other backend, paused on his activity as the
policy says) and the gpu lane, the backends named here. The PC's owner is using it, so the gpu lane leases only
inside a schedule AND while the PC is idle or locked, and gives the job back the moment he returns. Worker YAML::

    gpu_window:
      enabled: true                      # false: the lane reports its window but leases nothing
      backends: [unity, theater-render]  # the gpu lane; every other backend is the cpu lane
      hours:                             # local time, "DAYS HH:MM-HH:MM"; a range past midnight ends the next day
        - "mon-fri 01:00-08:00"          #   weekday nights
        - "fri,sat 23:00-11:00"          #   longer on weekends: Fri 23:00-Sat 11:00 and Sat 23:00-Sun 11:00
      idle_seconds: 600                  # no keyboard/mouse input this long counts as idle
      locked_idle_seconds: 60            # a locked (or disconnected) session: this long without input
      desktop: crane-gpu                 # players start on this hidden desktop (never shown); "" to not

DAYS: ``mon``..``sun``, ranges (``mon-fri``), lists (``fri,sat``), ``daily``. Leasing starts only when the
window is open and the session is idle (or locked); a running job is released (no attempt counted) and its
player killed within ``check_s`` seconds of any input, an unlock, or the window's end.
"""
from __future__ import annotations

import ctypes
import datetime as dt
import re
import sys
import time

DAYS = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
DEFAULTS = {
    "enabled": True,
    "backends": ["unity", "theater-render"],
    "hours": ["mon-fri 01:00-08:00", "fri,sat 23:00-11:00"],
    "idle_seconds": 600,
    "locked_idle_seconds": 60,
    "check_s": 1.0,
    "desktop": "crane-gpu",
}
_ENTRY = re.compile(r"^\s*(\S+)\s+(\d{1,2}):(\d{2})\s*-\s*(\d{1,2}):(\d{2})\s*$")


def options(cfg_section: dict | None) -> dict:
    return {**DEFAULTS, **(cfg_section or {})}


def _days(spec: str) -> set[int]:
    if spec.lower() in ("daily", "*", "all"):
        return set(range(7))
    out: set[int] = set()
    for part in spec.lower().split(","):
        a, _, b = part.partition("-")
        try:
            i, j = DAYS.index(a[:3]), DAYS.index((b or a)[:3])
        except ValueError:
            raise ValueError(f"gpu_window.hours: unknown day in {spec!r} (mon..sun, ranges, lists, daily)") from None
        k = i
        while True:
            out.add(k)
            if k == j:
                break
            k = (k + 1) % 7
    return out


def parse_hours(hours: list[str]) -> list[tuple[set[int], int, int]]:
    """[(weekdays, start minute, end minute)]; an end at or before the start is on the next day."""
    out = []
    for spec in hours or []:
        m = _ENTRY.match(str(spec))
        if not m:
            raise ValueError(f"gpu_window.hours entry {spec!r} is not 'DAYS HH:MM-HH:MM'")
        a, b = int(m[2]) * 60 + int(m[3]), int(m[4]) * 60 + int(m[5])
        if a >= 24 * 60 or b > 24 * 60:
            raise ValueError(f"gpu_window.hours entry {spec!r}: times run 00:00-24:00")
        out.append((_days(m[1]), a, b if b > a else b + 24 * 60))
    return out


def intervals(hours: list[str], now: dt.datetime) -> list[tuple[dt.datetime, dt.datetime]]:
    """The window's open intervals from a day before ``now`` to eight days after, merged and sorted."""
    rules = parse_hours(hours)
    base = now.replace(hour=0, minute=0, second=0, microsecond=0)
    spans = []
    for d in range(-1, 9):
        day = base + dt.timedelta(days=d)
        for days, a, b in rules:
            if day.weekday() in days:
                spans.append((day + dt.timedelta(minutes=a), day + dt.timedelta(minutes=b)))
    spans.sort()
    merged: list[list[dt.datetime]] = []
    for s, e in spans:
        if merged and s <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], e)
        else:
            merged.append([s, e])
    return [(s, e) for s, e in merged]


def window_at(hours: list[str], now: dt.datetime) -> tuple[bool, dt.datetime | None]:
    """(open, when): open now and until when, or closed and the next opening (None: never)."""
    for s, e in intervals(hours, now):
        if s <= now < e:
            return True, e
        if s > now:
            return False, s
    return False, None


# ---------------------------------------------------------------- the user's session (Windows)

def session_locked() -> bool | None:
    """True when this session is locked or not attached to a console (nobody sees it); None where unknown."""
    if sys.platform != "win32":
        return None
    try:
        wts = ctypes.windll.wtsapi32  # type: ignore[attr-defined]
        buf, size = ctypes.c_void_p(), ctypes.c_ulong()
        # WTSSessionInfoEx (25) of WTS_CURRENT_SESSION: WTSINFOEXW { DWORD Level; union (8-aligned: it holds
        # LARGE_INTEGERs) { SessionId, SessionState, SessionFlags, ... } }
        if not wts.WTSQuerySessionInformationW(None, ctypes.c_ulong(0xFFFFFFFF), 25, ctypes.byref(buf), ctypes.byref(size)):
            return None
        try:
            level, _pad, _sid, state, flags = (ctypes.c_long * 5).from_address(buf.value)
        finally:
            wts.WTSFreeMemory(buf)
        if level != 1:
            return None
        if state != 0:      # not WTSActive: disconnected (switched user, closed RDP), so no one is at it
            return True
        return {0: True, 1: False}.get(flags & 0xFFFFFFFF)   # WTS_SESSIONSTATE_LOCK 0, UNLOCK 1, else unknown
    except (OSError, AttributeError, ValueError):
        return None


def input_idle_seconds() -> float | None:
    """Seconds since the last keyboard/mouse input in this session (the worker runs in the user's session)."""
    from . import idle
    return idle.user_idle_seconds()


# ---------------------------------------------------------------- the gate

class Gate:
    """Decides, from the schedule, the idle time and the lock state, whether the gpu lane may lease now
    (``closed()``) and whether a running job must go back now (``interrupted()``). The probes are injectable
    so the logic is tested without a desktop."""

    def __init__(self, opts: dict, idle_fn=input_idle_seconds, locked_fn=session_locked,
                 clock=dt.datetime.now, mono=time.monotonic):
        self.opts = options(opts)
        parse_hours(self.opts["hours"])   # a bad entry fails at start, not at 01:00
        self.idle_fn, self.locked_fn, self.clock, self.mono = idle_fn, locked_fn, clock, mono
        self._armed: tuple[float, float, bool] | None = None   # (monotonic, idle, locked) of the last check

    def window(self, now: dt.datetime | None = None) -> tuple[bool, dt.datetime | None]:
        return window_at(self.opts["hours"], now or self.clock())

    def closed(self, now: dt.datetime | None = None) -> str | None:
        """Why no GPU job may start now, or None."""
        now = now or self.clock()
        if not self.opts.get("enabled", True):
            return "GPU window disabled"
        is_open, when = self.window(now)
        if not is_open:
            return f"outside the GPU window (next at {when:%a %H:%M})" if when else "no GPU window configured"
        idle, locked = self.idle_fn(), self.locked_fn()
        if locked and (idle is None or idle >= float(self.opts["locked_idle_seconds"])):
            return None
        if idle is None:
            return "GPU window open; user activity unknown"
        if idle >= float(self.opts["idle_seconds"]):
            return None
        return f"GPU window open; user active (idle {idle:.0f} s of {float(self.opts['idle_seconds']):.0f})"

    def arm(self) -> None:
        """Called when a job starts: later input is measured from here."""
        self._armed = (self.mono(), self.idle_fn() or 0.0, bool(self.locked_fn()))

    def interrupted(self, now: dt.datetime | None = None) -> str | None:
        """Why a running GPU job must be released now (input, unlock, the window closing), or None."""
        now = now or self.clock()
        if not self.opts.get("enabled", True):
            return "GPU window disabled"
        if not self.window(now)[0]:
            return "GPU window closed"
        t, idle, locked = self.mono(), self.idle_fn(), self.locked_fn()
        prev = self._armed
        self._armed = (t, idle if idle is not None else 0.0, bool(locked))
        if prev is None:
            return None
        t0, idle0, locked0 = prev
        if locked0 and locked is False:
            return "session unlocked"
        if idle is None:
            return None if locked else "user activity unknown"
        # Without input the idle time grows with the clock; any input resets it.
        if idle + 1.0 < idle0 + (t - t0):
            return "user input"
        return None

    def report(self, now: dt.datetime | None = None, waiting: str | None = None) -> dict:
        """The policy's ``window`` field: the feed shows the next opening ("GPU window: next at HH:MM")."""
        now = now or self.clock()
        is_open, when = self.window(now)
        out = {"enabled": bool(self.opts.get("enabled", True)), "open": is_open}
        if when:
            out["until" if is_open else "next_at"] = when.astimezone().isoformat(timespec="minutes")
        if waiting:
            out["waiting"] = waiting[:80]
        return out
