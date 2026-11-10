#!/usr/bin/env python3
"""Prove that ``plant.SNAP_FRAC`` (default 1e-3) leaves the harness BIT-IDENTICAL to the committed plant.

Runs the same pipeline (probe generation -> twin fits -> twin missions and hidden missions) in three
subprocesses and compares raw array bytes:
  head      the committed plant.py (``git show HEAD:Tools/DecisionCal/plant.py``) shadowing the edited one;
  default   the edited plant.py, SNAP_FRAC untouched (1e-3);
  nosnap    the edited plant.py with SNAP_FRAC = 0.0 (must DIFFER, else the flag is not wired).

    nice -n 19 python3 -I Tools/DecisionCal/parity/bitwise_check.py
"""

import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
DC = HERE.parent
REPO = DC.parents[1]

CHILD = r'''
import sys, json, hashlib
sys.path[:0] = [p for p in {paths!r} if p]
import plant
if {snap!r} is not None:
    plant.SNAP_FRAC = {snap!r}
import numpy as np, hidden, probes, twin, mission
assert plant.__file__.startswith({expect!r}), plant.__file__
h = hashlib.sha256()
out = {{}}
for st in ('transient', 'timing'):
    row = hidden.sample_hidden(3, 5353, st).take([1])
    ev, _ = probes.generate(row, 99)
    for s in ev.segments:
        for k in sorted(vars(s)):
            v = getattr(s, k)
            if isinstance(v, np.ndarray):
                h.update(np.ascontiguousarray(v).tobytes())
    fits, _ = twin.fit_all(ev, 4, 7, ('rigid', 'rigid+lag'))
    tw = twin.to_spec(fits['rigid+lag'][0])
    m, _ = mission.run(tw.repeat(1).take([0, 1, 2, 3]), [0, 4, 8, 12], np.arange(4, dtype=np.uint64))
    mh, _ = mission.run(row.repeat(4), [0, 4, 8, 12], np.arange(4, dtype=np.uint64))
    for d in (m, mh):
        for k in sorted(d):
            h.update(np.ascontiguousarray(d[k]).tobytes())
    out[st] = {{'twin_J': m['J'].tolist(), 'hidden_J': mh['J'].tolist()}}
print(json.dumps({{'sha': h.hexdigest(), 'out': out}}))
'''


def run(paths, snap, expect):
    code = CHILD.format(paths=paths, snap=snap, expect=expect)
    r = subprocess.run([sys.executable, '-I', '-c', code], capture_output=True, text=True, cwd=str(DC))
    if r.returncode:
        raise SystemExit(r.stderr)
    return json.loads(r.stdout.strip().splitlines()[-1])


def main():
    head_src = subprocess.run(['git', '-C', str(REPO), 'show', 'HEAD:Tools/DecisionCal/plant.py'],
                              capture_output=True, text=True, check=True).stdout
    with tempfile.TemporaryDirectory() as td:
        Path(td, 'plant.py').write_text(head_src)
        head = run([td, str(DC)], None, td)
    default = run([str(DC)], None, str(DC))
    nosnap = run([str(DC)], 0.0, str(DC))
    res = {'head_sha': head['sha'], 'default_sha': default['sha'], 'nosnap_sha': nosnap['sha'],
           'identical_head_vs_default': head['sha'] == default['sha'],
           'nosnap_differs': nosnap['sha'] != default['sha'],
           'J': {'head': head['out'], 'default': default['out'], 'nosnap': nosnap['out']},
           'head_has_snapfrac': 'SNAP_FRAC' in head_src,
           'head_plant_sha256': hashlib.sha256(head_src.encode()).hexdigest()}
    print(json.dumps(res, indent=1))
    out = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'bitwise_check.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1))
    assert res['identical_head_vs_default'], 'default SNAP_FRAC is NOT bit-identical to HEAD'
    assert res['nosnap_differs'], 'SNAP_FRAC = 0 changed nothing: flag not wired'
    print('BITWISE PASS')


if __name__ == '__main__':
    main()
