"""Windows: start a process on a hidden desktop of this session, so no window it makes can be seen or take focus.

The GPU runner works in the logged-on user's session (its players need the desktop session's GPU access).
A player runs with ``-batchmode`` (no window); ``popen`` additionally starts it on a desktop of the user's
window station that is created here and never switched to (``CreateDesktop`` + ``STARTUPINFO.lpDesktop``).
Windows only ever shows the input desktop, so a window, dialog or console the player or its children open
lands where nobody sees it and cannot take the foreground. ``windows_of`` lists the windows a process has on a
desktop, which ``crane-worker --gpu-window gpu-check`` uses to verify that.
"""
from __future__ import annotations

import ctypes
import subprocess
import sys
import time

GENERIC_ALL = 0x10000000
CREATE_UNICODE_ENVIRONMENT = 0x00000400
WAIT_TIMEOUT = 0x102
INFINITE = 0xFFFFFFFF

_desktops: dict[str, int] = {}   # name -> handle, kept open for the process's lifetime (a desktop dies with its last handle)


if sys.platform == "win32":
    from ctypes import wintypes

    class STARTUPINFOW(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("lpReserved", wintypes.LPWSTR), ("lpDesktop", wintypes.LPWSTR),
                    ("lpTitle", wintypes.LPWSTR), ("dwX", wintypes.DWORD), ("dwY", wintypes.DWORD),
                    ("dwXSize", wintypes.DWORD), ("dwYSize", wintypes.DWORD), ("dwXCountChars", wintypes.DWORD),
                    ("dwYCountChars", wintypes.DWORD), ("dwFillAttribute", wintypes.DWORD), ("dwFlags", wintypes.DWORD),
                    ("wShowWindow", wintypes.WORD), ("cbReserved2", wintypes.WORD), ("lpReserved2", ctypes.c_void_p),
                    ("hStdInput", wintypes.HANDLE), ("hStdOutput", wintypes.HANDLE), ("hStdError", wintypes.HANDLE)]

    class PROCESS_INFORMATION(ctypes.Structure):
        _fields_ = [("hProcess", wintypes.HANDLE), ("hThread", wintypes.HANDLE),
                    ("dwProcessId", wintypes.DWORD), ("dwThreadId", wintypes.DWORD)]

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _u32 = ctypes.WinDLL("user32", use_last_error=True)
    _u32.CreateDesktopW.restype = wintypes.HANDLE
    _u32.CreateDesktopW.argtypes = [wintypes.LPCWSTR, wintypes.LPCWSTR, ctypes.c_void_p, wintypes.DWORD,
                                    wintypes.DWORD, ctypes.c_void_p]
    _u32.OpenDesktopW.restype = wintypes.HANDLE
    _u32.OpenDesktopW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _u32.OpenInputDesktop.restype = wintypes.HANDLE
    _u32.OpenInputDesktop.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _ENUM = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    _u32.EnumDesktopWindows.argtypes = [wintypes.HANDLE, _ENUM, wintypes.LPARAM]
    _u32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    _u32.IsWindowVisible.argtypes = [wintypes.HWND]
    _u32.CloseDesktop.argtypes = [wintypes.HANDLE]
    _u32.GetForegroundWindow.restype = wintypes.HWND
    _k32.CreateProcessW.argtypes = [wintypes.LPCWSTR, wintypes.LPWSTR, ctypes.c_void_p, ctypes.c_void_p, wintypes.BOOL,
                                    wintypes.DWORD, ctypes.c_void_p, wintypes.LPCWSTR, ctypes.POINTER(STARTUPINFOW),
                                    ctypes.POINTER(PROCESS_INFORMATION)]
    _k32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    _k32.WaitForSingleObject.restype = wintypes.DWORD
    _k32.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    _k32.CloseHandle.argtypes = [wintypes.HANDLE]


def ensure_desktop(name: str) -> None:
    """Creates (or opens) the hidden desktop ``name`` in this process's window station; never switches to it."""
    if name in _desktops:
        return
    h = _u32.CreateDesktopW(name, None, None, 0, GENERIC_ALL, None)
    if not h:
        raise OSError(ctypes.get_last_error(), f"CreateDesktop {name} failed")
    _desktops[name] = h


def _env_block(env: dict) -> ctypes.Array:
    items = sorted(((str(k), str(v)) for k, v in env.items()), key=lambda kv: kv[0].upper())
    return ctypes.create_unicode_buffer("".join(f"{k}={v}\0" for k, v in items) + "\0")


class Process:
    """The subset of subprocess.Popen the backends use: pid, poll(), wait(timeout), returncode."""

    def __init__(self, argv, desktop: str, cwd=None, env=None, creationflags: int = 0):
        ensure_desktop(desktop)
        si = STARTUPINFOW()
        si.cb = ctypes.sizeof(si)
        si.lpDesktop = desktop
        pi = PROCESS_INFORMATION()
        cmd = ctypes.create_unicode_buffer(subprocess.list2cmdline([str(a) for a in argv]))
        block = _env_block(env) if env is not None else None
        flags = creationflags | (CREATE_UNICODE_ENVIRONMENT if block is not None else 0)
        if not _k32.CreateProcessW(None, cmd, None, None, False, flags, block, None if cwd is None else str(cwd),
                                   ctypes.byref(si), ctypes.byref(pi)):
            raise OSError(ctypes.get_last_error(), f"CreateProcess on desktop {desktop} failed: {argv[0]}")
        _k32.CloseHandle(pi.hThread)
        self._h, self.pid, self.returncode, self.args = pi.hProcess, pi.dwProcessId, None, list(argv)

    def poll(self):
        if self.returncode is None and _k32.WaitForSingleObject(self._h, 0) != WAIT_TIMEOUT:
            self._reap()
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is not None:
            return self.returncode
        ms = INFINITE if timeout is None else max(0, int(timeout * 1000))
        if _k32.WaitForSingleObject(self._h, ms) == WAIT_TIMEOUT:
            raise subprocess.TimeoutExpired(self.args, timeout)
        self._reap()
        return self.returncode

    def _reap(self):
        code = wintypes.DWORD()
        _k32.GetExitCodeProcess(self._h, ctypes.byref(code))
        self.returncode = code.value
        _k32.CloseHandle(self._h)
        self._h = None

    def kill(self):
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(self.pid)], capture_output=True, check=False,
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))


def popen(argv, desktop: str, cwd=None, env=None, creationflags: int = 0) -> Process:
    return Process(argv, desktop, cwd=cwd, env=env, creationflags=creationflags)


def windows_of(pids, desktop: str | None = None, visible_only: bool = True) -> list[int]:
    """Window handles owned by ``pids`` (None: anyone) on ``desktop`` (None: the input desktop, the one on screen)."""
    pids = None if pids is None else {int(p) for p in pids}
    h = _u32.OpenInputDesktop(0, False, 0x0040) if desktop is None else _u32.OpenDesktopW(desktop, 0, False, 0x0040)
    if not h:   # DESKTOP_ENUMERATE refused (e.g. the secure desktop while locked): report nothing, say so
        raise OSError(ctypes.get_last_error(), f"cannot open desktop {desktop or '(input)'}")
    found = []

    def cb(hwnd, _):
        pid = wintypes.DWORD()
        _u32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if (pids is None or pid.value in pids) and (not visible_only or _u32.IsWindowVisible(hwnd)):
            found.append(hwnd)
        return True
    try:
        _u32.EnumDesktopWindows(h, _ENUM(cb), 0)
    finally:
        _u32.CloseDesktop(h)
    return found


def tree(pid: int) -> set[int]:
    """``pid`` and its descendants (a venv's python.exe, or a player's crash handler, starts a child)."""
    class PE(ctypes.Structure):
        _fields_ = [("dwSize", wintypes.DWORD), ("cntUsage", wintypes.DWORD), ("th32ProcessID", wintypes.DWORD),
                    ("th32DefaultHeapID", ctypes.c_size_t), ("th32ModuleID", wintypes.DWORD), ("cntThreads", wintypes.DWORD),
                    ("th32ParentProcessID", wintypes.DWORD), ("pcPriClassBase", wintypes.LONG), ("dwFlags", wintypes.DWORD),
                    ("szExeFile", wintypes.WCHAR * 260)]
    _k32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
    snap = _k32.CreateToolhelp32Snapshot(0x2, 0)   # TH32CS_SNAPPROCESS
    parents: dict[int, int] = {}
    try:
        e = PE()
        e.dwSize = ctypes.sizeof(e)
        ok = _k32.Process32FirstW(snap, ctypes.byref(e))
        while ok:
            parents[e.th32ProcessID] = e.th32ParentProcessID
            ok = _k32.Process32NextW(snap, ctypes.byref(e))
    finally:
        _k32.CloseHandle(snap)
    out, grew = {int(pid)}, True
    while grew:
        grew = False
        for child, parent in parents.items():
            if parent in out and child not in out:
                out.add(child)
                grew = True
    return out


def foreground_pid() -> int | None:
    hwnd = _u32.GetForegroundWindow()
    if not hwnd:
        return None
    pid = wintypes.DWORD()
    _u32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    return pid.value


def watch(pids_fn, seconds: float, period: float = 0.1) -> dict:
    """For ``seconds``: every ``period``, the visible windows of ``pids_fn()`` on the input desktop and whether
    the foreground window belonged to one of them. ``{"samples", "visible", "foreground"}``."""
    out = {"samples": 0, "visible": 0, "foreground": 0, "errors": 0}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pids = set(pids_fn())
        try:
            out["visible"] += bool(windows_of(pids))
        except OSError:
            out["errors"] += 1
        out["foreground"] += foreground_pid() in pids
        out["samples"] += 1
        time.sleep(period)
    return out
