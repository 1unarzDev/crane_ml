"""Reports the interactive desktop's idle time for a worker that cannot see it.

A worker run as a background task on Windows (session 0) cannot read the logged-in user's input.
This probe runs in the user's desktop session (the installer registers it at logon, hidden) and
writes ``{"idle_s", "at"}`` to ``<data dir>/input-idle.json`` every ``--period`` seconds;
``idle.probe_idle_seconds`` reads it. ``python -m crane_cluster.idleprobe [--once]``.
"""
from __future__ import annotations

import argparse
import json
import os
import time

from . import idle


def write(path) -> float | None:
    s = idle.user_idle_seconds()
    if s is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"idle_s": round(s, 1), "at": time.time()}))
        os.replace(tmp, path)
    return s


def main(argv=None):
    p = argparse.ArgumentParser(prog="crane-idleprobe", description=__doc__)
    p.add_argument("--period", type=float, default=10.0)
    p.add_argument("--once", action="store_true")
    a = p.parse_args(argv)
    path = idle.probe_path()
    while True:
        write(path)
        if a.once:
            return
        time.sleep(a.period)


if __name__ == "__main__":
    main()
