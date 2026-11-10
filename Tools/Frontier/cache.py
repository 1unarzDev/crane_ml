# SPDX-License-Identifier: GPL-3.0-or-later
"""Content-addressed result cache for the frontier studies (stdlib only).

    from cache import get_or_run
    res = get_or_run("d4", fn, config, seed, fidelity, code_files=[__file__, "sim.py"])

Key = sha256(code_hash(code_files) | canonical JSON config | seed | fidelity). A hit returns the stored result without
calling `fn(config, seed, fidelity)`. Editing any listed code file changes the key, so stale results are never served.

Store: append-only JSONL shards `<root>/<study>/shard_<xx>.jsonl` (xx = first two hex digits of the key). Writers take
an exclusive `flock` on the shard, re-check for the key under the lock (first writer wins), append one line, fsync.
Readers tolerate a truncated final line. The root is `$FRONTIER_CACHE` or `~/.cache/frontier` (never /tmp).

Held-out rule: `split == "heldout"` requires `prereg=<path>` whose sha256 appears, on a line that also names the file,
in Docs/DroneVisuals/frontier/SEEDS.md (override with `seeds_md=` or `$FRONTIER_SEEDS_MD`). Otherwise HeldoutRefused.
The check runs before any lookup or run, so held-out results are never read or produced without a frozen prereg.
"""
import fcntl
import hashlib
import json
import math
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SEEDS_MD = os.path.normpath(os.path.join(HERE, "..", "..", "Docs", "DroneVisuals", "frontier", "SEEDS.md"))


class HeldoutRefused(RuntimeError):
    pass


def cache_root():
    return os.environ.get("FRONTIER_CACHE") or os.path.join(os.path.expanduser("~"), ".cache", "frontier")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def code_hash(files):
    """Hash of file contents plus basenames, order-independent. Moving the checkout does not change it."""
    parts = sorted((os.path.basename(p), sha256_file(p)) for p in files)
    h = hashlib.sha256()
    for name, d in parts:
        h.update(name.encode() + b"\0" + d.encode() + b"\n")
    return h.hexdigest()


def _default(o):
    if hasattr(o, "item") and callable(o.item):  # numpy scalar
        return o.item()
    if hasattr(o, "tolist"):
        return o.tolist()
    if isinstance(o, (set, frozenset)):
        return sorted(o)
    raise TypeError("not JSON serialisable: %r" % type(o))


def canonical_json(obj):
    """Sorted keys, no whitespace, NaN/Inf refused (they do not round-trip)."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), default=_default, allow_nan=False)


def make_key(code_files, config, seed, fidelity):
    h = hashlib.sha256()
    for part in (code_hash(code_files), canonical_json(config), str(int(seed)), str(fidelity)):
        h.update(part.encode() + b"\x1f")
    return h.hexdigest()


def check_prereg(prereg, seeds_md=None):
    """Raise HeldoutRefused unless sha256(prereg) is recorded with its file name in SEEDS.md."""
    if not prereg:
        raise HeldoutRefused("held-out split requires prereg=<frozen prereg file>")
    if not os.path.isfile(prereg):
        raise HeldoutRefused("prereg file not found: %s" % prereg)
    ledger = seeds_md or os.environ.get("FRONTIER_SEEDS_MD") or DEFAULT_SEEDS_MD
    if not os.path.isfile(ledger):
        raise HeldoutRefused("seed ledger not found: %s" % ledger)
    digest, name = sha256_file(prereg), os.path.basename(prereg)
    for line in open(ledger, encoding="utf-8"):
        if digest in line and name in line:
            return digest
    raise HeldoutRefused("sha256 of %s (%s...) is not recorded in %s" % (name, digest[:12], ledger))


def shard_path(study, key, root=None):
    d = os.path.join(root or cache_root(), study)
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, "shard_%s.jsonl" % key[:2])


def _scan(path, key):
    """Return the first record with `key` in the shard, or None. Bad (truncated) lines are skipped."""
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        for raw in f:
            if key.encode() not in raw:
                continue
            try:
                rec = json.loads(raw)
            except ValueError:
                continue
            if rec.get("key") == key:
                return rec
    return None


def lookup(study, key, root=None):
    return _scan(shard_path(study, key, root), key)


def _append_once(path, key, rec):
    """Append `rec` unless `key` is already present (checked under the exclusive lock). Returns (stored, written)."""
    line = (canonical_json(rec) + "\n").encode()
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            existing = _scan(path, key)
            if existing is not None:
                return existing, False
            size = os.fstat(fd).st_size
            if size:  # a crashed writer may have left a partial line: terminate it so ours starts clean
                with open(path, "rb") as f:
                    f.seek(size - 1)
                    if f.read(1) != b"\n":
                        os.write(fd, b"\n")
            os.write(fd, line)
            os.fsync(fd)
            return rec, True
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


def get_or_run(study, fn, config, seed, fidelity, code_files, split="dev", prereg=None, seeds_md=None, root=None,
               cost_hook=None):
    """Return the cached result for (code, config, seed, fidelity) or run `fn(config, seed, fidelity)` and store it.

    The result must be JSON-serialisable. Returns the result object (use `get_record` for provenance fields).
    """
    return get_record(study, fn, config, seed, fidelity, code_files, split, prereg, seeds_md, root, cost_hook)["result"]


def get_record(study, fn, config, seed, fidelity, code_files, split="dev", prereg=None, seeds_md=None, root=None,
               cost_hook=None):
    if split not in ("dev", "train", "val", "test", "heldout"):
        raise ValueError("unknown split %r" % split)
    if split == "heldout":
        check_prereg(prereg, seeds_md)
    key = make_key(code_files, config, seed, fidelity)
    path = shard_path(study, key, root)
    hit = _scan(path, key)
    if hit is not None:
        if hit.get("split") != split:
            raise ValueError("key %s... cached under split %r, requested %r" % (key[:12], hit.get("split"), split))
        hit = dict(hit, cache_hit=True)
        return hit
    t0, c0 = time.time(), time.process_time()
    result = fn(config, seed, fidelity)
    wall, cpu = time.time() - t0, time.process_time() - c0
    rec = {"key": key, "study": study, "split": split, "seed": int(seed), "fidelity": fidelity,
           "config": json.loads(canonical_json(config)), "code_hash": code_hash(code_files),
           "result": json.loads(canonical_json(result)), "wall_s": round(wall, 6), "cpu_s": round(cpu, 6),
           "ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "prereg_sha": sha256_file(prereg) if prereg else None}
    if cost_hook:
        cost_hook(rec)
    stored, written = _append_once(path, key, rec)
    return dict(stored, cache_hit=not written)


def iter_study(study, root=None):
    """Yield every record of a study (all shards)."""
    d = os.path.join(root or cache_root(), study)
    if not os.path.isdir(d):
        return
    for name in sorted(os.listdir(d)):
        if name.startswith("shard_") and name.endswith(".jsonl"):
            with open(os.path.join(d, name), "rb") as f:
                for raw in f:
                    try:
                        yield json.loads(raw)
                    except ValueError:
                        continue
