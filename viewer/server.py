"""``python -m viewer serve``: the viewer page plus a local copy of the coordinator's replay feed.

Routes (the feed ones answer in replay.ts's shapes, so the page is the same here and on the dashboard):
  GET /                         the viewer (web/), /ctraj.js is theater/web/ctraj.js
  GET /v1/viewer/index          local only: runs, studies, tier pairs, judge reports
  GET /v1/viewer/judge?runs=a,b local only: the judge's anchors, insights and pair records about these runs
  GET /v1/viewer/judge/<id>     local only: a judge report, its viewer feed and verdict
  GET /v1/feed/runs/<id>/traj   ?lod=1 for the overview LOD
  GET /v1/feed/courses/<sha>    the course reduced for 2-D
  GET /v1/feed/ghosts           ?run=ID | study=S&seed=N
  GET /v1/feed/fleet[.pack]     ?study=S
With ``remote``, a feed request this store cannot answer goes to that coordinator (read-only, no token).
``feed_only`` hides /v1/viewer/*, so the page behaves as it does on the dashboard.
"""

from __future__ import annotations

import json
import mimetypes
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .store import Store

WEB = Path(__file__).resolve().parent / 'web'
CODEC = Path(__file__).resolve().parents[1] / 'theater' / 'web' / 'ctraj.js'
CSP = ("default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; "
       "font-src https://fonts.gstatic.com; img-src 'self' data:; connect-src 'self'; base-uri 'none'")


def make_handler(store: Store, remote: str | None = None, feed_only: bool = False):
    class H(BaseHTTPRequestHandler):
        server_version = 'crane-viewer'

        def log_message(self, fmt, *args):  # quiet: one line per error only
            if args and str(args[1] if len(args) > 1 else '').startswith(('4', '5')):
                super().log_message(fmt, *args)

        def send(self, code, body: bytes, ctype='application/json; charset=utf-8', extra=None):
            self.send_response(code)
            self.send_header('content-type', ctype)
            self.send_header('content-length', str(len(body)))
            self.send_header('cache-control', 'no-store')
            self.send_header('x-content-type-options', 'nosniff')
            self.send_header('content-security-policy', CSP)
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)

        def json(self, obj, code=200):
            self.send(code, json.dumps(obj, separators=(',', ':')).encode())

        def proxy(self):
            if not remote:
                return self.json({'error': 'not found'}, 404)
            try:
                with urllib.request.urlopen(urllib.request.Request(remote.rstrip('/') + self.path, headers={'user-agent': 'crane-viewer'}), timeout=60) as r:
                    return self.send(r.status, r.read(), r.headers.get('content-type', 'application/octet-stream'))
            except urllib.error.HTTPError as e:
                return self.send(e.code, e.read(), e.headers.get('content-type', 'application/json'))
            except OSError as e:
                return self.json({'error': f'remote unreachable: {e}'}, 502)

        def static(self, rel):
            p = CODEC if rel == 'ctraj.js' else (WEB / (rel or 'index.html')).resolve()
            if p != CODEC and (WEB not in p.parents or not p.is_file()):
                return self.json({'error': 'not found'}, 404)
            ctype = mimetypes.guess_type(p.name)[0] or 'application/octet-stream'
            if ctype in ('application/javascript', 'text/javascript'):
                ctype = 'text/javascript; charset=utf-8'
            return self.send(200, p.read_bytes(), ctype)

        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):  # noqa: C901 - one flat router
            u = urllib.parse.urlsplit(self.path)
            q = {k: v[-1] for k, v in urllib.parse.parse_qs(u.query).items()}
            parts = [urllib.parse.unquote(x) for x in u.path.split('/') if x]
            try:
                if not parts or parts[0] != 'v1':
                    rel = '/'.join(parts)
                    if rel.startswith('viewer/'):
                        rel = rel[len('viewer/'):]  # the dashboard's mount path works here too
                    return self.static('' if rel == 'viewer' else rel)
                if parts[1] == 'viewer' and feed_only:
                    return self.json({'error': 'not found'}, 404)
                if parts[1:3] == ['viewer', 'index']:
                    if q.get('rescan'):
                        store.scan()
                    return self.json(store.index())
                if parts[1:3] == ['viewer', 'judge'] and len(parts) == 3:
                    return self.json(store.judge_for([x for x in q.get('runs', '').split(',') if x]))
                if parts[1:3] == ['viewer', 'judge'] and len(parts) >= 4:
                    r = store.report('/'.join(parts[3:]))
                    return self.json(r) if r else self.json({'error': 'no such report'}, 404)
                if parts[1] != 'feed':
                    return self.json({'error': 'not found'}, 404)
                if len(parts) >= 5 and parts[2] == 'runs' and parts[-1] == 'traj':
                    run = store.runs.get('/'.join(parts[3:-1]))
                    if not run:
                        return self.proxy()
                    lod = q.get('lod') == '1' and run.lod is not None
                    return self.send(200, (run.lod if lod else run.path).read_bytes(), 'application/octet-stream',
                                     {'x-crane-lod': '1' if lod else '0', 'etag': f'"{run.lod_sha if lod else run.sha}"'})
                if len(parts) == 4 and parts[2] == 'courses':
                    c = store.course(parts[3])
                    return self.json(c) if c else self.proxy()
                if parts[2:] == ['ghosts']:
                    seed = int(q['seed']) if q.get('seed') not in (None, '') else None
                    g = store.ghosts(q.get('run'), q.get('study'), seed, int(q.get('limit', 8)))
                    return self.json(g) if g and (g['runs'] or not remote) else self.proxy()
                if parts[2:] in (['fleet'], ['fleet.pack']):
                    study = q.get('study')
                    if not study or not any(r.header.get('study') == study for r in store.runs.values()):
                        return self.proxy()
                    kw = {'limit': int(q.get('limit', 2000))}
                    if parts[2] == 'fleet':
                        return self.json(store.fleet(study, **kw))
                    return self.send(200, store.fleet_pack(study, **kw), 'application/octet-stream')
                return self.proxy()
            except (ValueError, KeyError) as e:
                return self.json({'error': str(e)}, 400)

    return H


def serve(roots, host='127.0.0.1', port=8765, remote=None, feed_only=False, log=print):
    store = Store(roots)
    httpd = ThreadingHTTPServer((host, port), make_handler(store, remote, feed_only))
    log(f'crane viewer: {len(store.runs)} runs, {len(store.courses)} courses, {len(store.reports)} judge reports '
        f'-> http://{host}:{httpd.server_address[1]}/' + (f' (remote {remote})' if remote else ''))
    return httpd
