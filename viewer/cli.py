"""python -m viewer {serve, ingest, shot}: the fast-tier web viewer (see viewer/README.md)."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def chrome() -> str | None:
    """A headless-only Chrome (Playwright's chrome-headless-shell): it cannot open a window."""
    env = os.environ.get('CRANE_HEADLESS_CHROME')
    if env:
        return env
    hits = sorted(Path.home().glob('.cache/ms-playwright/chromium_headless_shell-*/chrome-*/chrome-headless-shell'))
    return str(hits[-1]) if hits else shutil.which('chrome-headless-shell')


def shot(url, out, size='1600,1000', budget_ms=6000) -> int:
    exe = chrome()
    if not exe:
        print('no chrome-headless-shell (npx playwright install chromium-headless-shell, or set CRANE_HEADLESS_CHROME)', file=sys.stderr)
        return 2
    cmd = [exe, '--no-sandbox', '--hide-scrollbars', '--disable-gpu', '--use-gl=swiftshader', f'--window-size={size}',
           f'--virtual-time-budget={budget_ms}', f'--screenshot={Path(out).resolve()}', url]
    return subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120).returncode


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='python -m viewer', description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('serve', help='serve the viewer over local runs (crane-traj/1) and judge reports')
    s.add_argument('roots', nargs='*', default=['.'], help='dirs searched for *.ctraj, courses/*.json and judge report.json')
    s.add_argument('--host', default='127.0.0.1')
    s.add_argument('--port', type=int, default=8765)
    s.add_argument('--remote', help='a coordinator (e.g. https://crane.lunarz.dev) for runs not found locally')
    s.add_argument('--feed-only', action='store_true', help='hide the local index and judge routes (the dashboard mode)')
    i = sub.add_parser('ingest', help='run dirs (fastsim / mhseals_testing layout) -> crane-traj/1 under OUT')
    i.add_argument('src', nargs='+', help='a paired archive (has index.jsonl + pairs/), or run dirs')
    i.add_argument('--out', required=True)
    i.add_argument('--tier', default='fast', help='tier of plain run dirs')
    i.add_argument('--study', help='study name (default: the archive dir name)')
    i.add_argument('--arm')
    p = sub.add_parser('shot', help='headless screenshot of a viewer URL (no window)')
    p.add_argument('url')
    p.add_argument('out')
    p.add_argument('--size', default='1600,1000')
    p.add_argument('--budget-ms', type=int, default=6000)
    a = ap.parse_args(argv)
    if a.cmd == 'serve':
        from .server import serve
        httpd = serve(a.roots, a.host, a.port, a.remote, a.feed_only)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            httpd.server_close()
        return 0
    if a.cmd == 'ingest':
        from . import ingest
        first = Path(a.src[0]).expanduser()
        if len(a.src) == 1 and (first / 'index.jsonl').exists():
            n = ingest.ingest_pairs(first, a.out, a.study)
        else:
            n = ingest.ingest_dirs(a.src, a.out, a.tier, a.study or Path(a.out).name, a.arm)
        print(f'{n} runs -> {a.out}')
        return 0 if n else 1
    return shot(a.url, a.out, a.size, a.budget_ms)
