"""crane-archive command line.

    crane-archive init [--root DIR] [--node NAME]   create the store (refuses a non-empty directory)
    crane-archive run                               the daemon: pull, confirm, scrub, quota, heartbeat, serve
    crane-archive status                            space, objects by kind, the last loop results (local)
    crane-archive scrub [--gb N | --all]            rehash now; reports bad objects to the coordinator
    crane-archive put FILE [--kind K]               store a local file (rig and local runs); prints its sha256
    crane-archive get SHA [-o FILE]                 the original bytes
    crane-archive pin SHA [--off]                   exempt an object from eviction
    crane-archive move --to DIR                     copy, verify, and switch the config's root (drives arrived)
All take --config (default ~/.config/crane-cluster/archive.yaml).
"""
from __future__ import annotations

import argparse
import json
import re
import signal
import sys
import threading
from pathlib import Path

from .daemon import Coordinator, Daemon, load_config, read_token
from .server import AccessVerifier, make_server
from .store import Store, StoreError


def _store(cfg, create=False) -> Store:
    try:
        return Store(cfg['root'], node=str(cfg['node']), create=create)
    except StoreError as e:
        sys.exit(f'crane-archive: {e}')


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog='crane-archive', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config')
    sub = ap.add_subparsers(dest='cmd', required=True)
    i = sub.add_parser('init')
    i.add_argument('--root')
    i.add_argument('--node')
    sub.add_parser('run')
    sub.add_parser('status')
    s = sub.add_parser('scrub')
    s.add_argument('--gb', type=float, default=1.0)
    s.add_argument('--all', action='store_true')
    p = sub.add_parser('put')
    p.add_argument('file')
    p.add_argument('--kind', default='raw')
    g = sub.add_parser('get')
    g.add_argument('sha')
    g.add_argument('-o', '--out')
    pn = sub.add_parser('pin')
    pn.add_argument('sha')
    pn.add_argument('--off', action='store_true')
    m = sub.add_parser('move')
    m.add_argument('--to', required=True)
    a = ap.parse_args(argv)
    cfg = load_config(a.config)

    if a.cmd == 'init':
        if a.root:
            cfg['root'] = str(Path(a.root).expanduser())
        if a.node:
            cfg['node'] = a.node
        st = _store(cfg, create=True)
        print(f'archive root {st.root} (node {st.marker["node"]})')
        return
    store = _store(cfg)
    if a.cmd == 'run':
        coord = Coordinator(cfg['coordinator'], read_token(cfg))
        d = Daemon(cfg, store, coord, log=lambda msg: print(msg, file=sys.stderr, flush=True))
        acc = cfg.get('access') or {}
        verifier = AccessVerifier(acc['team_domain'], acc['aud']) if acc.get('team_domain') and acc.get('aud') else None

        def health():
            h = d.last_health
            return {'node': d.node, **{k: h.get(k) for k in ('state', 'objects', 'used', 'total')}, 'last': d.last}

        srv = make_server(store, str(cfg['listen']), verifier=verifier, health=health)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        signal.signal(signal.SIGTERM, lambda *_: d.stop.set())
        print(f'crane-archive {d.node}: root {store.root}, serving {cfg["listen"]}'
              f'{" (access checked)" if verifier else " (local requests only: no access config)"}', flush=True)
        d.run()
        srv.shutdown()
        return
    if a.cmd == 'status':
        d = Daemon(cfg, store, None)
        h = d.health()
        h.pop('last', None)
        print(json.dumps(h, indent=1, default=str))
        return
    if a.cmd == 'scrub':
        coord = None
        try:
            coord = Coordinator(cfg['coordinator'], read_token(cfg))
        except SystemExit:
            pass
        d = Daemon(cfg, store, coord)
        r = d.scrub(1 << 62 if a.all else int(a.gb * 1e9))
        print(json.dumps({k: (len(v) if k == 'bad' else v) for k, v in r.items()}))
        sys.exit(1 if r['bad'] else 0)
    if a.cmd == 'put':
        o = store.put_file(a.file, kind=a.kind, name=Path(a.file).name)
        print(o.sha256)
        return
    if a.cmd == 'get':
        data = store.read_bytes(a.sha)
        if a.out:
            Path(a.out).write_bytes(data)
        else:
            sys.stdout.buffer.write(data)
        return
    if a.cmd == 'pin':
        sys.exit(0 if store.pin(a.sha, not a.off) else f'crane-archive: no object {a.sha}')
    if a.cmd == 'move':
        new = store.move_to(a.to)
        path = Path(cfg['_path'])
        if path.is_file():
            text = path.read_text()
            line = f'root: {new.root}'
            text = re.sub(r'(?m)^root:.*$', line, text) if re.search(r'(?m)^root:', text) else f'{line}\n{text}'
            path.write_text(text)
            print(f'{path}: {line}; restart: systemctl --user restart crane-archive. The old root is kept; remove it when satisfied.')
        return


if __name__ == '__main__':
    main()
