#!/usr/bin/env bash
# Reproducibility harness for Tools/DecisionCal. See REPRODUCE.md.
#
#   Tools/DecisionCal/reproduce.sh --quick       tests + checksum/drift/summary-recompute (no simulation)
#   Tools/DecisionCal/reproduce.sh               ... plus a 4-boat smoke of every driver on a development seed
#   Tools/DecisionCal/reproduce.sh --no-tests    skip the pytest step (about 5 min on a loaded box)
#   Tools/DecisionCal/reproduce.sh --smoke-only  only the 4-boat smokes
#
# Env: DC_CACHE (stored runs, default ~/.cache/decisioncal), DC_NLC_ROOT (nl-constraints-bench checkout),
#      DC_SMOKE_OUT (scratch dir for smoke outputs, default a mktemp dir), DC_SMOKE_SEED (default 4242;
#      seeds 20261109-20261114 are the confirmatory seeds and are refused), DC_PROCS (default 4, max 4).
#      DC_MODS and DC_LIMIT (restrict the M1-M4 smoke to some modules / N boats; the dev-driver smokes still run).
# Everything is niced and uses at most DC_PROCS (<= 4) processes. Nothing under DC_CACHE is modified.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CACHE="${DC_CACHE:-$HOME/.cache/decisioncal}"
SEED="${DC_SMOKE_SEED:-4242}"
PROCS="${DC_PROCS:-4}"
RUN_TESTS=1; RUN_CHECK=1; RUN_SMOKE=1
for a in "$@"; do
  case "$a" in
    --quick) RUN_SMOKE=0 ;;
    --no-tests) RUN_TESTS=0 ;;
    --smoke-only) RUN_TESTS=0; RUN_CHECK=0 ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    *) echo "unknown flag $a" >&2; exit 2 ;;
  esac
done
if [ "$SEED" -ge 20261109 ] && [ "$SEED" -le 20261114 ]; then echo "refusing confirmatory seed $SEED" >&2; exit 2; fi
if [ "$PROCS" -gt 4 ]; then echo "at most 4 processes" >&2; exit 2; fi
cd "$ROOT" || exit 2
PY="nice -n 19 python3 -I -W ignore"
rc=0

echo "== environment"
python3 -I - <<'EOF'
import platform, sys, numpy
print('python', sys.version.split()[0], '| numpy', numpy.__version__, '|', platform.platform(), '|', platform.machine())
try:
    print('cpu', [l.split(':', 1)[1].strip() for l in open('/proc/cpuinfo') if l.startswith('model name')][0])
except OSError:
    pass
try:
    import numpy.__config__ as c
    b = c.CONFIG['Build Dependencies']['blas']
    print('blas', b.get('name'), b.get('version'), '(a BLAS from a different vendor/thread count can change last digits)')
except Exception as e:
    print('blas: unknown', e)
EOF
echo "git HEAD: $(git rev-parse --short HEAD 2>/dev/null || echo none)  (manifests record crane_head 403e33e4abfc)"
V=$(sha256sum "${DC_NLC_ROOT:-$HOME/nl-constraints-bench}/nlc/sim/vessel.py" 2>/dev/null | cut -c1-64)
P=$(grep -o "NLC_VESSEL_SHA256 = '[0-9a-f]*'" "$HERE/_paths.py" | grep -o "[0-9a-f]\{64\}")
[ "$V" = "$P" ] && echo "nlc vessel.py pin: OK ($P)" || { echo "nlc vessel.py pin: MISMATCH (have '${V:-missing}', pinned $P; need nl-constraints-bench@fc4437a)"; rc=1; }
CH=Tools/Characterization/profiles/characterized.json
echo "twin.py input $CH sha256 $(sha256sum "$CH" 2>/dev/null | cut -c1-64) (expected 1832d1188eb7b851dd49c79d7a93992447bc0a0d02ef4423efafbfd853460ccb; NOT covered by any prereg hash file)"

if [ $RUN_TESTS = 1 ]; then
  echo "== tests (expected: 61 passed or more, ~4.6 min when 4 other procs are busy)"
  $PY -m pytest Tools/DecisionCal -q 2>&1 | tail -5 || rc=1
fi

if [ $RUN_CHECK = 1 ]; then
  echo "== prereg drift: files whose current sha256 differs from the hash file frozen at that run's start"
  for r in m0 m0b m1 m2 m3 m4; do
    f="$CACHE/$r-prereg.sha256"
    if [ ! -f "$f" ]; then echo "  $r: no hash file at $f"; continue; fi
    bad=$(sha256sum -c "$f" 2>&1 | grep -v ': OK$' | grep -v WARNING | sed 's#Tools/DecisionCal/##; s#: FAILED$##' | tr '\n' ' ')
    echo "  $r: ${bad:-no drift}"
  done
  echo "== stored runs: canonical row digest (rows sorted by id, wall-clock 'budget' removed; invariant to imap_unordered order) and summary recompute"
  DC_CACHE="$CACHE" $PY - <<'EOF'
import hashlib, json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path('Tools/DecisionCal').resolve())); sys.dont_write_bytecode = True
import record
C = Path(os.environ['DC_CACHE'])
RUNS = [('m0', 'm0-20261109'), ('m0v11', 'm0v11-20261109'), ('m0b', 'm0b-20261109'), ('m1', 'm1-20261110'),
        ('m2', 'm2-20261112'), ('m3', 'm3-20261113'), ('m4', 'm4-20261114')]
def strip(o):
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if k != 'budget'}
    if isinstance(o, list):
        return [strip(v) for v in o]
    return o
rows_of = {}
for tag, d in RUNS:
    p = C / d / 'plants.jsonl'
    if not p.exists():
        print(f'  {tag:6s} MISSING {p}'); continue
    rows = {}
    for l in p.read_text().splitlines():
        r = json.loads(l); rows[r['id']] = r
    rows_of[tag] = rows
    blob = json.dumps([strip(rows[k]) for k in sorted(rows)], sort_keys=True).encode()
    s = C / d / 'summary.json'
    print(f'  {tag:6s} rows {len(rows):4d}  canonical-rows sha256 {hashlib.sha256(blob).hexdigest()[:16]}  '
          f'summary.json sha256 {hashlib.sha256(s.read_bytes()).hexdigest()[:16]}')
import importlib
def check(tag, d, mod):
    if tag not in rows_of: return
    S = importlib.import_module(mod)
    rows = rows_of[tag]
    if mod == 'study_m1':
        order = [j['id'] for j in S.build_jobs(S.SEED, 6, 16, 512)]
        # the driver summarises rows in memory (insertion order = ARMS order); plants.jsonl is written with
        # sort_keys=True, so restore that order or score_m1.h2's rank tie-breaking changes (see REPRODUCE.md)
        for r in rows.values():
            for lib in r['libs'].values():
                a = lib['scores']['arms']; lib['scores']['arms'] = {k: a[k] for k in S.ARMS}
        fn = S.score_m1.summarize
    else:
        order = [f'{st}-{i:03d}' for st, n in S.N_PER.items() for i in range(n)]
        fn = S.summarize
    new = json.loads(json.dumps(record._clean(fn([rows[i] for i in order]))))
    old = json.loads((C / d / 'summary.json').read_text())
    new.pop('elapsed_s', None); old.pop('elapsed_s', None)
    print(f'  {tag:6s} summary recomputed from stored rows == stored summary.json: {new == old}')
for tag, d, mod in [('m1', 'm1-20261110', 'study_m1'), ('m2', 'm2-20261112', 'study_m2'),
                    ('m3', 'm3-20261113', 'study_m3'), ('m4', 'm4-20261114', 'study_m4')]:
    try:
        check(tag, d, mod)
    except Exception as e:
        print(f'  {tag:6s} recompute failed: {type(e).__name__}: {e}')
EOF
fi

if [ $RUN_SMOKE = 1 ]; then
  OUT="${DC_SMOKE_OUT:-$(mktemp -d /tmp/dc_smoke.XXXXXX)}"; mkdir -p "$OUT"
  echo "== 4-boat smokes, development seed $SEED, outputs in $OUT (not the stored runs)"
  # study_m1..m4 share study_m1.evaluate; M2-M4 patch its module globals when imported, as their drivers do.
  # n_prop is 64 for M1 so the sample-based fitter does not dominate (stored M1 used 512).
  for m in ${DC_MODS:-study_m1 study_m2 study_m3 study_m4}; do
    echo "-- $m"
    DC_MOD=$m DC_SEED=$SEED DC_PROCS=$PROCS DC_OUT="$OUT" DC_LIMIT="${DC_LIMIT:-4}" $PY - <<'EOF'
import hashlib, importlib, json, os, sys, time
from multiprocessing import get_context
from pathlib import Path
sys.path.insert(0, str(Path('Tools/DecisionCal').resolve())); sys.dont_write_bytecode = True
import hidden, record
mod, seed, procs, limit = os.environ['DC_MOD'], int(os.environ['DC_SEED']), int(os.environ['DC_PROCS']), int(os.environ['DC_LIMIT'])
S = importlib.import_module(mod)
import study_m1
jobs = []
if mod == 'study_m1':
    seen = {}
    for j in S.build_jobs(seed, 6, 16, 64, {k: 2 for k in S.N_PER}, 2, (0.0, 0.4)):
        seen.setdefault(j['stratum'], j)              # first boat of each stratum
    jobs = list(seen.values())
else:
    for st in list(S.N_PER)[:2]:
        spec = hidden.sample_hidden(2, seed, st)
        for i in range(2):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-{i:03d}', 'stratum': st, 'seed': seed, 'K': 6, 'M': 16, 'n_prop': 0})
jobs = jobs[:limit]
t = time.time()
with get_context('fork').Pool(procs) as pool:
    res = list(pool.imap(study_m1.evaluate, jobs))
def strip(o):
    if isinstance(o, dict): return {k: strip(v) for k, v in o.items() if k != 'budget'}
    if isinstance(o, list): return [strip(v) for v in o]
    return o
for r in res:
    r = json.loads(json.dumps(record._clean(r)))
    print(f'  {r["id"]:22s} row sha256 {hashlib.sha256(json.dumps(strip(r), sort_keys=True).encode()).hexdigest()[:16]}')
(Path(os.environ['DC_OUT']) / f'{mod}.jsonl').write_text('\n'.join(json.dumps(record._clean(r), sort_keys=True) for r in res) + '\n')
print(f'  {len(res)} boats in {time.time() - t:.0f}s')
EOF
    [ $? = 0 ] || rc=1
  done
  echo "-- study.py (M0b driver, 1 boat per stratum, default twin.ARMS)"
  $PY Tools/DecisionCal/study.py --out "$OUT/m0b" --per-stratum 1 --seed "$SEED" --procs "$PROCS" 2>&1 | tail -6 || rc=1
  echo "-- study_oracle_twin.py"
  $PY Tools/DecisionCal/study_oracle_twin.py --out "$OUT/oracle" --seed "$SEED" --n 2 --procs 1 2>&1 | tail -4 || rc=1
  echo "-- study_probe_rate.py"
  $PY Tools/DecisionCal/study_probe_rate.py --out "$OUT/probe_rate" --seed "$SEED" --limit 4 --procs "$PROCS" 2>&1 | tail -4 || rc=1
  echo "-- study_rules.py"
  $PY Tools/DecisionCal/study_rules.py --out "$OUT/rules" --seed "$SEED" --n-per 2 --max-boats 4 --procs "$PROCS" 2>&1 | tail -4 || rc=1
  echo "-- baseline_selector.py smoke (6 training + 4 test boats)"
  $PY Tools/DecisionCal/baseline_selector.py smoke --out "$OUT/baseline" --procs 1 2>&1 | tail -4 || rc=1
  echo "smoke outputs left in $OUT (delete when done)"
fi
echo "== done (rc=$rc)"
exit $rc
