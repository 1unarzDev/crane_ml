"""Where up puts a sim, and whether one already running is the same: reuse it, start beside it, or refuse.

    decide(running, identity, caller, fresh=False, auto=False)  -> 'start' | 'reuse' | 'replace' | 'next' | SimError

``running`` is None (nothing runs in that instance) or {identity, owner, sim}; ``caller`` is 'person' (a terminal)
or 'script' (an agent, a test, a pipeline). A person's sim is never taken over by a script: the script's T1 world
goes to the next free instance, and its T2 world is refused (the one Unity player). CRANE_SIM_OWNER=person|script
names the caller when stdin cannot (astro dispatching from a terminal keeps the terminal).
"""

import getpass
import json
import os
import sys
import time

from crane.sim.config import SimError, invoked_as

RECIPE = 'recipe.json'
MAX_INSTANCES = 32


def caller():
    kind = os.environ.get('CRANE_SIM_OWNER')
    if kind in ('person', 'script'):
        return kind
    try:
        return 'person' if sys.stdin.isatty() else 'script'
    except (AttributeError, ValueError):
        return 'script'


def decide(running, identity, who, fresh=False, auto=False):
    if running is None:
        return 'start'
    owner, same = running.get('owner'), running.get('identity') == identity
    name = running.get('sim') or 'a sim'
    me = invoked_as()
    if auto:
        return 'reuse' if same and owner == who and not fresh else 'next'
    if owner == 'person' and who == 'script':
        return 'next'                     # never a person's world; T2 is refused at the next instance
    if fresh:
        return 'replace'
    if same:
        return 'reuse'
    if owner is None:
        raise SimError(f'already up; {me} down first (or {me} up --fresh)')
    raise SimError(f'already up with another sim ({name}, started by a {owner}); {me} up --fresh replaces it, '
                   f'--instance auto starts beside it (T1), {me} down stops it')


def read(run):
    try:
        return json.loads((run / RECIPE).read_text())
    except (OSError, ValueError):
        return {}


def write(run, recipe, who):
    run.mkdir(parents=True, exist_ok=True)
    doc = {**recipe.as_dict(), 'owner': who, 'user': getpass.getuser(), 'pid': os.getpid(),
           'tty': os.ttyname(0) if who == 'person' and os.isatty(0) else None,
           'started': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    (run / RECIPE).write_text(json.dumps(doc, indent=2) + '\n')


def running_doc(t2, run):
    """{identity, owner, sim} of the sim running in RUN, or None when nothing runs there."""
    from crane.sim import t1, world
    is_t1 = world._read(run / 'tier') == 'T1'
    container = (t2.get('ros') or {}).get('container')
    up = (t1.heartbeat(run) is not None) if is_t1 else world.alive(run)
    if not up and not world.ros_up(container):
        return None
    doc = read(run)
    return {'identity': doc.get('identity'), 'owner': doc.get('owner'), 'sim': doc.get('sim'), 'tier': doc.get('values', {}).get('tier')}
