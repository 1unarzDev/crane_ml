"""An ``astro bundle`` (tgz) read as the gate's inputs.

    b = bundle.open_bundle('bundle-173031.tgz', into=tmpdir)
    b.state, b.spec, b.results, b.log_lines, b.bags, b.sha256
    yaml, provenance, run, snapshot, notes = bundle.gate_inputs(b)

Layout: ``state/hardware*.yaml`` (the vessel state: characterization, mixer, channel map), ``state/tests.yaml``
(astro's tests file), ``<day>/results/<test>-<HHMMSS>.yaml``, ``<day>/log.txt`` and, with ``astro bundle --bags``,
``<day>/bags/<HHMMSS>[-label]/`` (rosbag2 MCAP).

What the gate gets, and from where:
  yaml        the state's characterization block (version 1 from astro's speed tests is mapped to 2 by
              ``gate.characterization_v2``; a block with the v2 keys and no version is taken as 2)
  provenance  ``crane.calibration/1``: per direction the speed result that measured it (fit id, session =
              the bundle, run = the result file, config = its state hash, software = its ``software``),
              the state's hardware hash (thruster_matrix, reversed, hardware_epoch) and verify hash
              (channel_map, pwm_channels); ``software: {repo: sha}`` from the results (the older
              ``mhseals_testing``/``mhseals_hardware`` keys read the same)
  run         the speed results' holds: command (a list, or astro's {axis, sign, level, efforts}), start,
              end, samples {t, vx, vy, wz[, yaw, t_sensor]}; ``settle`` is the test's ``baseline_s`` (the
              neutral the samples hold before ``start``), ``transient`` its ``settle_s`` (the measure
              window is the last ``measure_s``). astro's own ``settle`` key is a timestamp and is not read.
  snapshot    the bundled state
A state whose characterization ``source`` starts with ``sim`` is flagged sim_derived (SIM_DERIVED).
"""
from __future__ import annotations

import hashlib
import re
import tarfile
from dataclasses import dataclass, field
from pathlib import Path

import gate as G

LEGACY_SOFTWARE = ("mhseals_testing", "mhseals_hardware")
V2_KEYS = ("speed_curves", "wave_floor")


@dataclass
class Bundle:
    path: Path
    root: Path
    sha256: str
    state_path: Path | None
    state: dict
    spec: dict
    results: list = field(default_factory=list)      # [(path, doc)] oldest first
    log_lines: list = field(default_factory=list)    # [(day, 'HH:MM:SS', kind, message)]
    bags: list = field(default_factory=list)         # rosbag2 dirs (or loose .mcap files)

    @property
    def name(self):
        return self.path.name

    @property
    def sim_derived(self):
        char = G.characterization_block(self.state)
        return str(char.get("source", "")).lower().startswith("sim")

    def test(self, name):
        return ((self.spec or {}).get("tests") or {}).get(name) or {}


def _yaml(path):
    import yaml
    try:
        doc = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError):
        return None
    return doc


def unpack(path, into):
    with tarfile.open(path) as tar:
        tar.extractall(into, filter="data")
    return Path(into)


def open_bundle(path, into):
    """Unpack ``path`` into ``into`` (an empty dir) and read it. A directory is read in place."""
    path = Path(path)
    if path.is_dir():
        root, sha = path, ""
    else:
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        root = unpack(path, into)
    found = sorted((p for p in root.rglob("hardware*.yaml") if ".bak" not in p.name),
                   key=lambda p: (len(p.parts), p.name))
    state_path = found[0] if found else None
    state = (_yaml(state_path) or {}) if state_path else {}
    specs = sorted(root.rglob("tests.yaml"), key=lambda p: len(p.parts))
    spec = (_yaml(specs[0]) or {}) if specs else {}
    results = []
    for p in sorted(root.rglob("results/*.yaml"), key=lambda p: (p.parent.parent.name, p.name)):
        doc = _yaml(p)
        if isinstance(doc, dict) and doc.get("test"):
            results.append((p, doc))
    lines = []
    for p in sorted(root.rglob("log.txt")):
        for line in p.read_text(errors="replace").splitlines():
            m = re.match(r"^(\d\d:\d\d:\d\d)  (\S+) +(.*)$", line)
            if m:
                lines.append((p.parent.name, *m.groups()))
    bags = sorted({p.parent for p in root.rglob("*.mcap")})
    return Bundle(path, root, sha, state_path, state, spec, results, lines, bags)


def software(doc):
    """``{repo: sha}`` of a result: the generic ``software`` mapping (older keys included), a bare string,
    or ``commit``."""
    sw = doc.get("software")
    if isinstance(sw, dict):
        return {k: v for k, v in sw.items() if isinstance(v, str) and v}
    if isinstance(sw, str) and sw:
        return {"software": sw}
    return {"commit": doc["commit"]} if doc.get("commit") else {}


def latest_by_test(results):
    """{test: (path, doc)}: per test the applied result when one was, else the newest done one."""
    out = {}
    for path, doc in results:
        if doc.get("status", "done") != "done":
            continue
        have = out.get(doc["test"])
        if have is None or doc.get("applied") or not have[1].get("applied"):
            out[doc["test"]] = (path, doc)
    return out


def baseline_s(hold, test):
    """The neutral seconds before ``start``: astro's ``baseline: [b0, b1]`` window when present, else the tests
    file's ``baseline_s``. astro's older ``settle`` key (a timestamp) is never read."""
    window = hold.get("baseline")
    if isinstance(window, (list, tuple)) and len(window) == 2:
        return max(0.0, float(hold["start"]) - float(window[0]))
    return float(test.get("baseline_s", 2.0))


def gate_hold(hold, test):
    """A result hold in the gate's terms (see the module docstring); None without samples."""
    if not isinstance(hold, dict) or not hold.get("samples") or "command" not in hold:
        return None
    out = {"command": G.command_efforts(hold["command"]), "start": float(hold["start"]), "end": float(hold["end"]),
           "samples": hold["samples"], "settle": baseline_s(hold, test),
           "transient": float(test.get("settle_s", G.DEFAULT_SETTINGS["transient_s"]))}
    if hold.get("override"):
        out["override"] = True
    return out


def gate_inputs(b: Bundle):
    """(yaml, provenance, run, snapshot, notes) for ``gate.run_gate``."""
    notes = []
    char = dict(G.characterization_block(b.state))
    if "version" not in char and all(k in char for k in V2_KEYS):
        char["version"] = 2
    hw = {"hash": G.dependency_hash(b.state, G.DEP_INVALID), "epoch": b.state.get("hardware_epoch", 0),
          "verify": G.dependency_hash(b.state, G.DEP_VERIFY), "sim_derived": b.sim_derived}
    picked = latest_by_test(b.results)
    directions, holds, sw = {}, [], {}
    for test_name, (path, doc) in sorted(picked.items()):
        sw.update(software(doc))
        if doc.get("type") != "speed" and not str(test_name).startswith("speed-"):
            continue
        test = b.test(test_name)
        kept = [h for h in (gate_hold(x, test) for x in doc.get("holds") or []) if h]
        holds += kept
        if doc.get("holds") and not kept:
            notes.append(f"{path.name}: holds without samples")
        axis = test.get("axis") or test_name.split("-", 1)[-1]
        for sign in "+-":
            name = f"{axis}{sign}"
            if name in (char.get("speed_curves") or {}):
                directions[name] = {
                    "fit": {"id": path.stem, "time": f"{doc.get('date', '')} {doc.get('time', '')}".strip(),
                            "session": b.name, "run": str(path.relative_to(b.root)), "config": doc.get("state"),
                            **({"software": software(doc)} if software(doc) else {})},
                    "hardware": hw}
    provenance = {"schema": G.CALIBRATION, "directions": directions}
    if sw:
        provenance["software"] = sw
    run = {"procedure": "astro speed tests", "holds": holds,
           "settings": {"transient_s": G.DEFAULT_SETTINGS["transient_s"], "coast_s": 0.0,
                        "settle_s": G.DEFAULT_SETTINGS["settle_s"]}}
    still = picked.get("still")
    if still and (still[1].get("measured") or {}).get("conditions"):
        run["conditions"] = still[1]["measured"]["conditions"]
    return {"characterization": char}, provenance, run, b.state, notes


def import_profile(b: Bundle, out, report=None, base=None, log=print):
    """Run the gate on the bundle and write the profile (``out``), the gate report (``report``, default
    ``<out>.gate.json``) and the derived inputs (``<out>.inputs/``, so the import can be re-run and audited).
    Returns (profile, report, notes)."""
    import json
    import import_yaml
    out = Path(out).resolve()
    report = Path(report).resolve() if report else out.with_suffix(".gate.json")
    inputs = Path(str(out) + ".inputs")
    inputs.mkdir(parents=True, exist_ok=True)
    char, provenance, run, state, notes = gate_inputs(b)
    files = {}
    for name, doc in (("characterization", char), ("provenance", provenance), ("run", run), ("snapshot", state)):
        files[name] = inputs / f"{name}.json"
        files[name].write_text(json.dumps(doc, indent=2, default=str) + "\n")
    args = ["--yaml", str(files["characterization"]), "--provenance", str(files["provenance"]),
            "--run", str(files["run"]), "--config-snapshot", str(files["snapshot"]),
            "--out-profile", str(out), "--out-report", str(report)]
    if base:
        args += ["--base", str(base)]
    log(f"bundle {b.name}: state {b.state_path.name if b.state_path else 'none'}, "
        f"{len(provenance['directions'])} directions with a speed result, {len(run['holds'])} holds with samples")
    import_yaml.main(args, log=log)
    return json.loads(out.read_text()), json.loads(report.read_text()), notes
