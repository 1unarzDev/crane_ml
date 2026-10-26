"""Canonical JSON and content hashing for crane.* v2 documents."""
from __future__ import annotations

import hashlib
import json
from typing import Any

HASH_FIELD = "contentHash"


def canonical_json(doc: Any) -> str:
    return json.dumps(doc, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def content_hash(doc: dict) -> str:
    """sha256 of the canonical form of *doc* without its own contentHash."""
    body = {k: v for k, v in doc.items() if k != HASH_FIELD}
    return hashlib.sha256(canonical_json(body).encode("utf-8")).hexdigest()


def seal(doc: dict) -> dict:
    out = {k: v for k, v in doc.items() if k != HASH_FIELD}
    out[HASH_FIELD] = content_hash(out)
    return out


def write_sealed(doc: dict, path) -> str:
    from pathlib import Path
    sealed = seal(doc)
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(sealed, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    return sealed[HASH_FIELD]
