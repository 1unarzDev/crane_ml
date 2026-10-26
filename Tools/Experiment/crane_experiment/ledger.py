"""Append-only, hash-chained JSONL ledger. Evidence ids are `ev:<sha12>`."""
import json
import os

from .canon import canon, sha256_hex
from .errors import LedgerCorrupt

GENESIS = "0" * 64


def entry_hash(prev: str, seq: int, etype: str, data) -> str:
    return sha256_hex(prev + canon({"seq": seq, "type": etype, "data": data}))


def evidence_id(h: str) -> str:
    return "ev:" + h[:12]


def read_verified(path):
    """Read and verify the whole chain; raises LedgerCorrupt on any break."""
    entries = []
    prev = GENESIS
    if not os.path.exists(path):
        return entries
    with open(path, "r", encoding="utf-8") as fh:
        for n, line in enumerate(fh):
            line = line.rstrip("\n")
            if not line:
                raise LedgerCorrupt(f"blank line at {n}")
            try:
                e = json.loads(line)
            except ValueError as exc:
                raise LedgerCorrupt(f"unparseable line {n}: {exc}")
            if e.get("seq") != n or e.get("prev") != prev:
                raise LedgerCorrupt(f"chain break at seq {n}")
            if entry_hash(prev, n, e["type"], e["data"]) != e.get("hash"):
                raise LedgerCorrupt(f"hash mismatch at seq {n}")
            prev = e["hash"]
            entries.append(e)
    return entries


class Ledger:
    def __init__(self, path, fsync=True):
        self.path = path
        self.fsync = fsync
        self.entries = read_verified(path)

    @property
    def head(self):
        return self.entries[-1]["hash"] if self.entries else GENESIS

    def append(self, etype, data):
        seq = len(self.entries)
        # round-trip through canon so stored data is exactly what is hashed
        data = json.loads(canon(data))
        h = entry_hash(self.head, seq, etype, data)
        e = {"seq": seq, "type": etype, "data": data, "prev": self.head, "hash": h}
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(canon(e) + "\n")
            fh.flush()
            if self.fsync:
                os.fsync(fh.fileno())
        self.entries.append(e)
        return e

    def verify(self):
        return len(read_verified(self.path))

    def evidence_ids(self):
        return {evidence_id(e["hash"]) for e in self.entries}
