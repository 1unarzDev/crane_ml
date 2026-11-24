"""Where crane's tools live in this checkout. Every path the verbs exec is named once, here."""

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # the crane_ml checkout
CONFIG = ROOT / 'config'
DOMAINS = CONFIG / 'domains'
SIM_UP = ROOT / 'Tools' / 'RoboBoat' / 'sim_up'
HEAVY = ROOT / 'bin' / 'heavy'
COURSE = ROOT / 'Tools' / 'RoboBoat'                    # generate.sh, validate.py, render.py
BENCH = ROOT / 'Tools' / 'Performance'                  # run_worker.sh, sweep_workers.sh (the player bench)
BUILDS = ROOT / 'Builds'
PLAYER_BUILD = BUILDS / 'CRANE-Worker'                  # what `crane build player` writes
PROJECT_VERSION = ROOT / 'ProjectSettings' / 'ProjectVersion.txt'
HOME = Path.home()
USER_CONFIG = HOME / '.config' / 'crane'
PLAYER_STORE = HOME / '.local' / 'share' / 'crane' / 'player'   # installed player tarballs, one dir per version
ASTRO_DOCK = Path(os.environ.get('ASTRO_DOCK', HOME / 'astro_dock'))


def research_cli():
    """The research-cluster CLI: RESEARCH_CLUSTER/bin/research, else ~/research-cluster/bin/research, else None."""
    for base in (os.environ.get('RESEARCH_CLUSTER'), HOME / 'research-cluster'):
        if base and (Path(base) / 'bin' / 'research').is_file():
            return Path(base) / 'bin' / 'research'
    return None


def player_candidates():
    """Players in the order sim_up tries them, then the installed ones (newest version last)."""
    out = [Path(p) for p in [os.environ.get('CRANE_PLAYER')] if p]
    out += [PLAYER_BUILD / 'CRANE.x86_64', HOME / '.cache' / 'crane-phase1' / 'player' / 'CRANE.x86_64']
    out += sorted(PLAYER_STORE.glob('*/CRANE.x86_64'))
    return out
