"""Where crane's tools live in this checkout. Every path the verbs exec is named once, here."""

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # the crane_ml checkout
CONFIG = ROOT / 'config'
DOMAINS = CONFIG / 'domains'
SIM_UP = ROOT / 'Tools' / 'RoboBoat' / 'sim_up'
HEAVY = ROOT / 'bin' / 'heavy'
COURSE = ROOT / 'Tools' / 'RoboBoat'                    # generate.sh, validate.py, render.py
BENCH = ROOT / 'Tools' / 'Bench'                  # run_worker.sh, sweep_workers.sh (the player bench)
BUILDS = ROOT / 'Builds'
PLAYER_BUILD = BUILDS / 'CRANE-Worker'                  # what `crane build player` writes
PROJECT_VERSION = ROOT / 'ProjectSettings' / 'ProjectVersion.txt'
HOME = Path.home()
USER_CONFIG = HOME / '.config' / 'crane'
PLAYER_STORE = HOME / '.local' / 'share' / 'crane' / 'player'   # <commit12>/ per player, current and prev links
ASTRO_DOCK = Path(os.environ.get('ASTRO_DOCK', HOME / 'astro_dock'))


def research_cli():
    """The research-cluster CLI: RESEARCH_CLUSTER/bin/research, else ~/research-cluster/bin/research, else None."""
    for base in (os.environ.get('RESEARCH_CLUSTER'), HOME / 'research-cluster'):
        if base and (Path(base) / 'bin' / 'research').is_file():
            return Path(base) / 'bin' / 'research'
    return None


def player_candidates():
    """Players in the order sim_up tries them, then the installed ones (crane update, crane deploy): current, prev."""
    out = [Path(p) for p in [os.environ.get('CRANE_PLAYER')] if p]
    out += [PLAYER_BUILD / 'CRANE.x86_64', HOME / '.cache' / 'crane-phase1' / 'player' / 'CRANE.x86_64']
    out += [PLAYER_STORE / link / 'CRANE.x86_64' for link in ('current', 'prev')]
    return out


def fit_out(bundle):
    """Where `crane fit` writes by default: BUNDLE's dir/<stem>.fit/, else ~/.cache/crane/fit/<stem>/ when that is not writable."""
    bundle = Path(bundle).expanduser()
    stem = bundle.name.split('.')[0]
    beside = bundle.parent / f'{stem}.fit'
    probe = beside if beside.exists() else bundle.parent
    if os.access(probe, os.W_OK | os.X_OK) and (beside.is_dir() or not beside.exists()):
        return beside
    return Path.home() / '.cache' / 'crane' / 'fit' / stem
