#!/usr/bin/env python3
"""Rig hook (design §4): fail when a node not on truth_allowlist.yaml subscribes to /crane/truth/*.

Runs ``ros2 topic info -v`` per truth topic, so it needs a sourced ROS 2 environment and the
run's ROS_DOMAIN_ID. Exit 0 clean, 1 on a violation (the offending nodes are printed), 2 when
ROS 2 is unavailable or ``ros2`` exits non-zero. Subscribers are matched on namespace
plus node name. ``--info FILE`` judges saved ``ros2 topic info -v`` output instead (tests).
"""

import argparse
import fnmatch
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def read_allowlist(path):
    allow, topics, section = [], [], None
    for line in Path(path).read_text().splitlines():
        line = line.split('#', 1)[0].rstrip()
        if line.endswith(':') and not line.startswith(' '):
            section = line[:-1]
        elif line.strip().startswith('- ') and section in ('allow', 'topics'):
            (allow if section == 'allow' else topics).append(line.strip()[2:].strip())
    return allow, topics


def subscribers(info_text):
    """Fully qualified node names (namespace/name, no leading slash) of the subscription
    endpoints in ``ros2 topic info -v`` output, so ``/other/referee`` is not ``referee``."""
    nodes, name, namespace = [], None, '/'
    for line in info_text.splitlines():
        m = re.match(r'\s*Node name:\s*(\S+)', line)
        if m:
            name, namespace = m.group(1), '/'
        m = re.match(r'\s*Node namespace:\s*(\S+)', line)
        if m:
            namespace = m.group(1)
        if re.match(r'\s*Endpoint type:\s*SUBSCRIPTION', line) and name:
            nodes.append((namespace.rstrip('/') + '/' + name.lstrip('/')).lstrip('/'))
    return nodes


def violations(nodes, allow):
    return sorted({n for n in nodes if not any(fnmatch.fnmatch(n, pattern) for pattern in allow)})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--allowlist', default=str(HERE / 'truth_allowlist.yaml'))
    ap.add_argument('--info', help='saved ros2 topic info -v output to judge instead of a live graph')
    args = ap.parse_args(argv)
    allow, topics = read_allowlist(args.allowlist)
    bad = []
    for topic in topics:
        if args.info:
            text = Path(args.info).read_text()
        else:
            try:
                run = subprocess.run(['ros2', 'topic', 'info', '-v', topic], capture_output=True, text=True, timeout=20)
            except (OSError, subprocess.TimeoutExpired) as error:
                print(f'TRUTH_SUBSCRIBERS_UNAVAILABLE {error}')
                return 2
            if run.returncode != 0:
                detail = (run.stderr or run.stdout).strip().splitlines()
                print(f'TRUTH_SUBSCRIBERS_UNAVAILABLE topic={topic} ros2 exit {run.returncode}'
                      + (f': {detail[-1]}' if detail else ''))
                return 2
            text = run.stdout
        bad += [(topic, n) for n in violations(subscribers(text), allow)]
    for topic, node in bad:
        print(f'TRUTH_SUBSCRIBER_VIOLATION topic={topic} node={node}')
    if not bad:
        print('TRUTH_SUBSCRIBERS_OK')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
