"""The one CPU budget: what this process may use, not what the host has.

    from crane.resources import cpus, pids_budget
    workers = min(cpus(), len(jobs))

``cpus()`` is ``$CRANE_CPUS`` when set, else the least of the cgroup quota (v2 ``cpu.max``, v1
``cpu.cfs_quota_us``), the scheduler affinity and ``os.cpu_count()``. A container on a 128-core host with a
24-core quota gets 24. ``pids_budget()`` is the cgroup's thread/process limit (``pids.max``) or None.
Stdlib only.
"""

import os
from pathlib import Path

CGROUP = Path('/sys/fs/cgroup')


def _read(path):
    try:
        return Path(path).read_text().split()
    except OSError:
        return []


def _quota_v2(root):
    parts = _read(root / 'cpu.max')
    try:
        return max(1, int(int(parts[0]) / int(parts[1]))) if parts[0] != 'max' else None
    except (IndexError, ValueError, ZeroDivisionError):
        return None


def _quota_v1(root):
    for d in (root / 'cpu', root):
        quota, period = _read(d / 'cpu.cfs_quota_us'), _read(d / 'cpu.cfs_period_us')
        try:
            if quota and period and int(quota[0]) > 0 and int(period[0]) > 0:
                return max(1, int(int(quota[0]) / int(period[0])))
        except ValueError:
            pass
    return None


def cpus():
    """The CPU budget, at least 1."""
    env = os.environ.get('CRANE_CPUS', '').strip()
    if env.isdigit() and int(env) >= 1:
        return int(env)
    found = [os.cpu_count() or 1, _quota_v2(CGROUP), _quota_v1(CGROUP)]
    if hasattr(os, 'sched_getaffinity'):
        found.append(len(os.sched_getaffinity(0)))
    return max(1, min(n for n in found if n))


def pids_budget():
    """The cgroup's ``pids.max`` (it counts threads), or None when unlimited or unknown."""
    parts = _read(CGROUP / 'pids.max')
    try:
        return int(parts[0])
    except (IndexError, ValueError):
        return None


def prep_processes(jobs, gpu=False, threads_per_child=1):
    """Processes for a host-side prep pool: processes x threads <= cpus() (<= 8 beside a GPU), at most
    ``jobs``, and well under the pid budget (a quarter of it, a child holding a few threads)."""
    n = min(cpus(), 8) if gpu else cpus()
    budget = pids_budget()
    if budget:
        n = min(n, max(1, budget // (4 * (threads_per_child + 3))))
    return max(1, min(n, jobs))


def single_thread_env():
    """Env settings that keep a child's numba/BLAS/OpenMP pools at one thread."""
    return {k: '1' for k in ('NUMBA_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS')}
