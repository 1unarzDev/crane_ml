#!/usr/bin/env python3
"""Rotate crane's pinned image tags on Docker Hub: keep the newest N KIND-<sha> pins, every KIND-vX.Y.Z and KIND-next.

    python3 docker/registry.py rotate --repository lunarzdev/crane --kind core --keep 20           # what it would do
    python3 docker/registry.py rotate --repository lunarzdev/crane --kind core --keep 20 --apply   # delete them

Deleting needs DOCKERHUB_USERNAME and DOCKERHUB_TOKEN (a token with delete scope). Stdlib only.
"""

import argparse
import json
import os
import re
import sys
import urllib.request

HUB = 'https://hub.docker.com/v2'


def pins_to_drop(tags, kind, keep):
    """The KIND-<40 hex> tags beyond the newest `keep` (by last_updated); release (KIND-v*) and KIND-next stay.
    tags: [{'name', 'last_updated'}]."""
    pin = re.compile(rf'^{re.escape(kind)}-[0-9a-f]{{40}}$')
    pins = sorted((t for t in tags if pin.match(t['name'])), key=lambda t: t.get('last_updated') or '', reverse=True)
    return [t['name'] for t in pins[keep:]]


def _request(url, method='GET', token=None, body=None):
    data = json.dumps(body).encode() if body is not None else None
    headers = {'Accept': 'application/json', **({'Content-Type': 'application/json'} if data else {}),
               **({'Authorization': f'Bearer {token}'} if token else {})}
    with urllib.request.urlopen(urllib.request.Request(url, data, headers, method=method), timeout=60) as r:
        payload = r.read()
    return json.loads(payload) if payload else None


def tags(repository):
    out, url = [], f'{HUB}/repositories/{repository}/tags?page_size=100'
    while url:
        page = _request(url)
        out += [{'name': t['name'], 'last_updated': t.get('last_updated')} for t in page.get('results', [])]
        url = page.get('next')
    return out


def rotate(repository, kind, keep, apply):
    drop = pins_to_drop(tags(repository), kind, keep)
    if not drop:
        print(f'rotate {repository} {kind}: nothing beyond the newest {keep} pins')
        return 0
    if not apply:
        print(f'rotate {repository} {kind}: would delete {len(drop)}: {" ".join(drop)} (--apply deletes)')
        return 0
    user, secret = os.environ.get('DOCKERHUB_USERNAME'), os.environ.get('DOCKERHUB_TOKEN')
    if not (user and secret):
        print('rotate: DOCKERHUB_USERNAME and DOCKERHUB_TOKEN are needed to delete', file=sys.stderr)
        return 2
    token = _request(f'{HUB}/users/login', 'POST', body={'username': user, 'password': secret})['token']
    for name in drop:
        _request(f'{HUB}/repositories/{repository}/tags/{name}/', 'DELETE', token)
        print(f'rotate {repository}: deleted {name}')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='command', required=True)
    r = sub.add_parser('rotate')
    r.add_argument('--repository', required=True)
    r.add_argument('--kind', required=True, choices=('core', 'gpu', 'sim'))
    r.add_argument('--keep', type=int, default=20)
    r.add_argument('--apply', action='store_true')
    args = ap.parse_args(argv)
    return rotate(args.repository, args.kind, max(1, args.keep), args.apply)


if __name__ == '__main__':
    sys.exit(main())
