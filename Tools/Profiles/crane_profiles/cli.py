"""CLI.  `validate <file>` prints a JSON report and exits 0 (ok) / 1 (rule violations) / 2 (unreadable)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import canonical, validator

TOOLS = Path(__file__).resolve().parents[2]
SOURCES = {"underwater": TOOLS / "Underwater" / "profiles", "aerial": TOOLS / "Aerial" / "profiles",
           "terrain": TOOLS / "Terrain" / "profiles"}


def migrate_all(out_dir: Path) -> list:
    from .migrations import aerial, terrain, underwater
    mods = {"underwater": underwater, "aerial": aerial, "terrain": terrain}
    written = []
    for domain, src in SOURCES.items():
        for f in sorted(src.glob("*.json")):
            v2 = mods[domain].migrate(json.loads(f.read_text(encoding="utf-8")))
            target = out_dir / f"{domain}__{v2['id']}.json"
            canonical.write_sealed(v2, target)
            written.append(str(target))
    return written


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="crane_profiles")
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate", help="validate a v2 profile or dataset manifest; emits JSON")
    v.add_argument("file")
    m = sub.add_parser("migrate-all", help="migrate the in-repo v1 profiles to v2 files")
    m.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "migrated"))
    args = ap.parse_args(argv)
    if args.cmd == "validate":
        rep = validator.validate_file(args.file)
        print(json.dumps({"file": args.file, **rep.to_dict()}, indent=2, sort_keys=True))
        if not rep.ok and any("cannot read" in e["message"] for e in rep.errors):
            return 2
        return 0 if rep.ok else 1
    print(json.dumps({"written": migrate_all(Path(args.out))}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
