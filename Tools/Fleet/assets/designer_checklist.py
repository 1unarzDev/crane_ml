#!/usr/bin/env python3
"""List every provisional field: profile values + generator defaults (the complete designer question list).

    python designer_checklist.py [--profile P] [--defaults D] [--markdown]
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def walk(node, path=""):
    if isinstance(node, dict):
        if "value" in node and "provenance" in node:
            yield path, node
            return
        for k, v in node.items():
            yield from walk(v, f"{path}.{k}" if path else k)


def collect(profile_path, defaults_path):
    prof = json.load(open(profile_path))
    dfl = json.load(open(defaults_path))["groups"]
    rows = []
    for p, n in walk(prof):
        if n["provenance"] == "provisional":
            rows.append(("profile", p, n["value"], n.get("note", "")))
    for gname, items in dfl.items():
        if gname == "tessellation":
            continue
        for k, v in items.items():
            if k.startswith("_"):
                continue
            rows.append(("generator_default", f"{gname}.{k}", v["value"], v.get("question", "")))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default=os.path.join(REPO, "Tools", "Fleet", "spec", "fleet_profile_v0.json"))
    ap.add_argument("--defaults", default=os.path.join(HERE, "generator_defaults.json"))
    ap.add_argument("--markdown", action="store_true")
    a = ap.parse_args()
    rows = collect(a.profile, a.defaults)
    for src, key, val, note in rows:
        print(f"| {src} | `{key}` | {json.dumps(val)} | {note} |" if a.markdown else f"{src:18s} {key:55s} {json.dumps(val)[:50]:50s} {note}")
    print(f"# {len(rows)} provisional fields")


if __name__ == "__main__":
    main()
