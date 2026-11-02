"""Serving the archive: a Cloudflare Tunnel, its DNS name and an Access application that admits only the
coordinator's service token. Idempotent: finds what exists, creates what is missing, never deletes.

    CLOUDFLARE_API_TOKEN=... python3 cloudflare_setup.py [--hostname crane-archive.lunarz.dev] [--zone lunarz.dev]

Writes (mode 600, never printed):
  ~/.config/crane-cluster/archive-tunnel.token   the tunnel's connector token (crane-archive-tunnel.service)
  ~/.config/crane-cluster/secrets.env            ARCHIVE_ACCESS_ID / ARCHIVE_ACCESS_SECRET (Worker secrets)
and sets ``access: {team_domain, aud}`` in ~/.config/crane-cluster/archive.yaml. The token needs Account
Cloudflare Tunnel edit, Access apps and policies edit, Access service tokens edit, and Zone DNS edit.
A hostname one level under the zone (crane-archive.lunarz.dev) is covered by the zone's universal
certificate; a deeper one (archive.crane.lunarz.dev) would need an advanced certificate.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

API = 'https://api.cloudflare.com/client/v4'
CFG = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'crane-cluster'


def call(method, path, body=None):
    tok = os.environ.get('CLOUDFLARE_API_TOKEN')
    if not tok:
        sys.exit('cloudflare_setup: CLOUDFLARE_API_TOKEN is not set')
    req = urllib.request.Request(API + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={'authorization': f'Bearer {tok}', 'content-type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            j = json.loads(r.read())
    except urllib.error.HTTPError as e:
        j = json.loads(e.read() or b'{}')
    if not j.get('success'):
        sys.exit(f'cloudflare_setup: {method} {path.split("?")[0]}: {j.get("errors")}')
    return j['result']


def write_secret(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    os.chmod(path, 0o600)


def set_env_lines(path: Path, values: dict) -> None:
    text = path.read_text() if path.is_file() else ''
    for k, v in values.items():
        line = f'{k}={v}'
        text = re.sub(rf'(?m)^{k}=.*$', line, text) if re.search(rf'(?m)^{k}=', text) else text + ('' if text.endswith('\n') or not text else '\n') + line + '\n'
    write_secret(path, text)


def env_value(path: Path, key: str):
    if not path.is_file():
        return None
    m = re.search(rf'(?m)^{key}=(.*)$', path.read_text())
    return m.group(1).strip() if m else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hostname', default='crane-archive.lunarz.dev')
    ap.add_argument('--zone', default='lunarz.dev')
    ap.add_argument('--tunnel', default='crane-archive')
    ap.add_argument('--service-token', default='crane-coordinator-archive')
    ap.add_argument('--origin', default='http://127.0.0.1:7420')
    a = ap.parse_args()

    account = os.environ.get('CLOUDFLARE_ACCOUNT_ID') or call('GET', '/accounts?per_page=5')[0]['id']
    zone = call('GET', f'/zones?name={a.zone}')[0]['id']
    acc = f'/accounts/{account}'

    tunnels = call('GET', f'{acc}/cfd_tunnel?is_deleted=false&name={a.tunnel}')
    tunnel = tunnels[0] if tunnels else call('POST', f'{acc}/cfd_tunnel', {'name': a.tunnel, 'config_src': 'cloudflare'})
    tid = tunnel['id']
    print(f'tunnel {a.tunnel}: {tid} ({"existing" if tunnels else "created"})')
    call('PUT', f'{acc}/cfd_tunnel/{tid}/configurations', {'config': {'ingress': [
        {'hostname': a.hostname, 'service': a.origin, 'originRequest': {'connectTimeout': 30}},
        {'service': 'http_status:404'}]}})
    write_secret(CFG / 'archive-tunnel.token', call('GET', f'{acc}/cfd_tunnel/{tid}/token') + '\n')
    print(f'tunnel ingress {a.hostname} -> {a.origin}; connector token in {CFG / "archive-tunnel.token"}')

    target = f'{tid}.cfargotunnel.com'
    recs = call('GET', f'/zones/{zone}/dns_records?name={a.hostname}')
    if not recs:
        call('POST', f'/zones/{zone}/dns_records', {'type': 'CNAME', 'name': a.hostname, 'content': target, 'proxied': True,
                                                    'comment': 'crane-archive tunnel'})
        print(f'dns {a.hostname} CNAME {target} (created)')
    elif recs[0]['content'] != target:
        sys.exit(f'cloudflare_setup: {a.hostname} already points at {recs[0]["content"]}; not changing it')
    else:
        print(f'dns {a.hostname} (existing)')

    secrets = CFG / 'secrets.env'
    toks = [t for t in call('GET', f'{acc}/access/service_tokens') if t['name'] == a.service_token]
    have_secret = env_value(secrets, 'ARCHIVE_ACCESS_SECRET') and env_value(secrets, 'ARCHIVE_ACCESS_ID')
    if toks and have_secret and env_value(secrets, 'ARCHIVE_ACCESS_ID') == toks[0]['client_id']:
        st = toks[0]
        print(f'service token {a.service_token} (existing; secret in {secrets})')
    else:
        st = (call('POST', f'{acc}/access/service_tokens/{toks[0]["id"]}/rotate') if toks else
              call('POST', f'{acc}/access/service_tokens', {'name': a.service_token, 'duration': '8760h'}))
        set_env_lines(secrets, {'ARCHIVE_ACCESS_ID': st['client_id'], 'ARCHIVE_ACCESS_SECRET': st['client_secret']})
        print(f'service token {a.service_token} ({"rotated" if toks else "created"}); id and secret in {secrets}')

    apps = [x for x in call('GET', f'{acc}/access/apps') if x.get('domain') == a.hostname]
    policy = {'name': 'coordinator service token', 'decision': 'non_identity',
              'include': [{'service_token': {'token_id': st['id']}}]}
    if apps:
        app = apps[0]
        print(f'access app {a.hostname} (existing)')
    else:
        app = call('POST', f'{acc}/access/apps', {'name': 'crane-archive', 'domain': a.hostname, 'type': 'self_hosted',
                                                 'session_duration': '24h', 'app_launcher_visible': False})
        print(f'access app {a.hostname} (created)')
    pols = call('GET', f'{acc}/access/apps/{app["id"]}/policies')
    if not any(p.get('name') == policy['name'] for p in pols):
        call('POST', f'{acc}/access/apps/{app["id"]}/policies', {**policy, 'precedence': 1})
        print('access policy: the coordinator service token only (created)')
    team = call('GET', f'{acc}/access/organizations')['auth_domain']

    cfg = CFG / 'archive.yaml'
    line = f'access: {{team_domain: {team}, aud: {app["aud"]}}}'
    if cfg.is_file():
        text = cfg.read_text()
        text = re.sub(r'(?m)^#? ?access:.*$', line, text) if re.search(r'(?m)^#? ?access:', text) else text + line + '\n'
        cfg.write_text(text)
        print(f'{cfg}: {line}')
    else:
        print(f'add to archive.yaml: {line}')
    print(f'Worker: ARCHIVE_URL=https://{a.hostname}; put ARCHIVE_ACCESS_ID and ARCHIVE_ACCESS_SECRET from {secrets} '
          'with wrangler secret put (README "Serving")')


if __name__ == '__main__':
    main()
