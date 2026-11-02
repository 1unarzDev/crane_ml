#!/usr/bin/env python3
"""End-to-end on this machine: `wrangler dev` (local D1/R2 in a temp dir, nothing deployed), a crane
worker running fastsim toy jobs as a three-iteration study, then `crane-theater select` and `fetch`.

Checks: the worker stamps each run.ctraj and posts its run_meta; the .ctraj is a summary artifact;
select picks the per-iteration best and the failures with their trajectories' sha256; fetch
downloads and verifies them.

  python3 theater/tests/e2e_local.py [--keep]
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'cluster')]
from crane_cluster import config as config_mod, worker  # noqa: E402
from crane_cluster.api import Client  # noqa: E402
from theater import cli, traj  # noqa: E402

ADMIN = 'e2e-admin-token'
TOY_GENERATE = ['{python}', '-c', 'import sys; sys.path[:0] = [sys.argv[1], sys.argv[1] + "/../.."]; import toy; '
                '[toy.write_scenario(sys.argv[2], s) for s in range(int(sys.argv[3]), int(sys.argv[4]))]',
                '{base}', '{dir}', '{seeds_from}', '{seeds_to}']


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def main():
    tmp = Path(tempfile.mkdtemp(prefix='crane-theater-e2e-'))
    port, coord_dir = free_port(), ROOT / 'cluster' / 'coordinator'
    url = f'http://127.0.0.1:{port}'
    env = {**os.environ, 'WRANGLER_SEND_METRICS': 'false', 'CI': '1', 'NO_COLOR': '1'}
    state = tmp / 'state'
    proc = None
    try:
        subprocess.run(['npx', 'wrangler', 'd1', 'migrations', 'apply', 'crane-cluster', '--local', '--persist-to', str(state)],
                       cwd=coord_dir, env=env, check=True, capture_output=True)
        with open(tmp / 'wrangler.log', 'w') as log:
            proc = subprocess.Popen(['npx', 'wrangler', 'dev', '--local', '--ip', '127.0.0.1', '--port', str(port),
                                     '--persist-to', str(state), '--show-interactive-dev-session=false',
                                     '--var', f'ADMIN_TOKEN:{ADMIN}'], cwd=coord_dir, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
        adm = Client(url, ADMIN, retries=0)
        for _ in range(240):
            try:
                if adm.get('/v1/ping')['ok']:
                    break
            except Exception:
                time.sleep(0.5)
        else:
            raise AssertionError('wrangler dev did not come up')
        invite = adm.post('/v1/admin/invite', {'n': 1})['invites'][0]
        token = Client(url).post('/v1/join', {'invite': invite, 'name': 'e2e', 'caps': {'backends': ['fastsim']}})['token']
        (tmp / 'token').write_text(token)
        cfg = config_mod.load(None)
        cfg.update(coordinator=url, name='e2e', token_file=tmp / 'token', data_dir=tmp / 'data', off_file=tmp / 'off',
                   poll_s=0.2, heartbeat_s=1, policy={'nice': 0}, caps={'tags': [], 'gpu': None},
                   backends={'fastsim': {'type': 'cluster.backends.fastsim:ClusterBackend', 'root': str(ROOT)}})
        jobs = []
        for it, gain in enumerate((0.05, 0.15, 0.3)):
            spec = {'workdir': 'fastsim/tests', 'config': 'toy.yaml', 'metric': 'run.sim_s',
                    'overrides': {'scenarios': {'dir': str(tmp / 'scenarios'), 'generate': TOY_GENERATE},
                                  'vehicle': {'gain': gain}},
                    'theater': {'study': 'e2e-toy', 'iteration': it, 'insight': f'gain {gain}'}}
            jobs.append({'backend': 'fastsim', 'seeds': [1, 2, 3], 'spec': spec, 'est_seconds': 5, 'submitter': 'e2e',
                         'purpose': f'Theater end-to-end: toy fastsim iteration {it} with gain {gain} for the select manifest.'})
        ids = adm.post('/v1/admin/jobs', {'jobs': jobs})['job_ids']
        w = worker.Worker(cfg)
        for _ in ids:
            w.loop(once=True)
        done = [adm.get('/v1/jobs', status='done')['jobs']]
        assert len(done[0]) == 3, adm.get('/v1/jobs')
        a = SimpleNamespace(study='e2e-toy', hypothesis=None, iterations=None, top_k=1, best_per_iteration=False,
                            failures=True, failures_only=False, outcome=None, valid_only=False, sample=None, seed=0,
                            recorder=None, course=None, lowest_best=True, from_file=None, out=str(tmp / 'manifest.json'))
        coord = cli.Coordinator(url, ADMIN)
        doc = cli.cmd_select(a, coord)
        assert doc['counts']['matched'] == 9 and len(doc['iterations']) == 3, doc['counts']
        assert all(r['traj']['sha256'] and r['traj']['size'] for r in doc['runs']), doc['runs']
        assert [i['insights'] for i in doc['iterations']] == [['gain 0.05'], ['gain 0.15'], ['gain 0.3']]
        got = cli.cmd_fetch(SimpleNamespace(manifest=a.out, dir=str(tmp / 'trajs')), coord)
        assert len(got) == len(doc['runs'])
        h = traj.read_header(got[0])
        assert h['study'] == 'e2e-toy' and h['build']['recorder'] == 'fastsim' and 'vehicle.gain' in h['params'], h
        arts = adm.get('/v1/artifacts', job=ids[0])['artifacts']
        assert {a['retention'] for a in arts if a['name'] == 'run.ctraj'} == {'summary'}
        print(f'[e2e] ok: {doc["counts"]["selected"]} of 9 runs selected over 3 iterations, trajectories fetched and verified')
    finally:
        if proc:
            os.killpg(proc.pid, 15)
            proc.wait(timeout=20)
        if '--keep' not in sys.argv:
            shutil.rmtree(tmp, ignore_errors=True)
        else:
            print(f'[e2e] kept {tmp}')


if __name__ == '__main__':
    main()
