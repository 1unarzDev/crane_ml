"""crane deploy HOST: bring a host up to this checkout over ssh, optionally with the T2 player, then crane doctor there.

    crane deploy HOST --dry-run          # probe the host and print the plan
    crane deploy HOST --player           # code, the .venv, then the player tarball (sha256 checked on the host)
    crane deploy local:/tmp/h --player   # the same into a dir standing in for the home (no ssh; tests)

On HOST it touches only ~/crane_ml, ~/.local/bin/crane (install.sh's link) and the player store
(~/.local/share/crane/player):
- code: REF (default HEAD, this checkout's commit). A git checkout there fetches that commit from this machine and
  fast-forwards (`git fetch ssh://THISHOST/REPO COMMIT && git merge --ff-only FETCH_HEAD`); without a checkout, its
  tracked files (no Unity project) go over by rsync into an empty ~/crane_ml or an earlier deploy's.
- venv: `install.sh --yes --no-clone --components LIST` there when it has no .venv or --components is given.
- player: rsync --partial into the store's .incoming/, then `bin/crane update --tarball` there checks the sha256,
  unpacks it and keeps current + prev (crane/update.py's installer, the one crane update uses).
- doctor: `bin/crane doctor --json` there, printed here.

A robot computer is refused before anything runs (robot_guard; config/deploy.yaml): robot code goes through astro.
"""

import fnmatch
import getpass
import ipaddress
import json
import os
import shlex
import shutil
import socket
import subprocess
import tempfile
from pathlib import Path

from crane import output, paths, update

STORE_REL = '.local/share/crane/player'          # paths.PLAYER_STORE, relative to the home
REPO_REL = 'crane_ml'
UNITY = ('Assets', 'Packages', 'ProjectSettings', 'UIElementsSchema')   # not sent to a host without a checkout
SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10']

PROBE = r'''
cd ~ || exit 3
echo "home=$HOME"
echo "free_kb=$(df -Pk . | awk 'NR==2 {print $4}')"
if [ -e crane_ml/.git ] && git -C crane_ml rev-parse --git-dir >/dev/null 2>&1; then
  echo checkout=git
  echo "head=$(git -C crane_ml rev-parse HEAD 2>/dev/null)"
  echo "dirty=$(git -C crane_ml status --porcelain --untracked-files=no 2>/dev/null | wc -l | tr -d ' ')"
elif [ -f crane_ml/.crane-deploy.json ] || grep -qs '^name = "crane"$' crane_ml/pyproject.toml; then
  echo checkout=files
  echo "head=$(sed -n 's/.*"commit": *"\([0-9a-f]*\)".*/\1/p' crane_ml/.crane-deploy.json 2>/dev/null)"
elif [ -n "$(ls -A crane_ml 2>/dev/null)" ]; then
  echo checkout=foreign
else
  echo checkout=none
fi
echo "venv=$([ -x crane_ml/.venv/bin/python ] && echo yes || echo no)"
echo "player=$(readlink .local/share/crane/player/current 2>/dev/null)"
echo "python=$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)"
for t in git rsync zstd; do echo "$t=$(command -v $t >/dev/null && echo yes || echo no)"; done
'''


ROBOT_REFUSAL = '{host} is a robot computer; robot code goes through `astro hot`'


def guard_config(path=None):
    from crane import miniyaml
    path = Path(path or paths.CONFIG / 'deploy.yaml')
    return (miniyaml.safe_load(path.read_text()) or {}).get('robot_guard') or {}


def _registry(root):
    """{host: (roles, addresses)} from every ROOT/*/devices.yaml; addresses resolve $KEY through the profile's
    network.env. Raises OSError or ValueError when the registry cannot be read."""
    from crane import miniyaml
    files = sorted(Path(root).expanduser().glob('*/devices.yaml'))
    if not files:
        raise OSError(f'no */devices.yaml under {root}')
    hosts = {}
    for f in files:
        env = {}
        if (f.parent / 'network.env').is_file():
            for line in (f.parent / 'network.env').read_text().splitlines():
                key, eq, value = line.partition('=')
                if eq and not key.lstrip().startswith('#'):
                    env[key.strip()] = value.strip().strip('"\'')
        for name, facts in ((miniyaml.safe_load(f.read_text()) or {}).get('hosts') or {}).items():
            facts = facts if isinstance(facts, dict) else {}
            roles, addrs = hosts.setdefault(str(name), (set(), set()))
            roles.update(str(r) for r in facts.get('roles') or ())
            address = str(facts.get('address') or '')
            addrs.add(env.get(address[1:], '') if address.startswith('$') else address)
    return hosts


def robot_guard(host, cfg=None, environ=None):
    """(refusal or None, warnings) for deploying to HOST. A host astro's registry lists is refused when it has a
    robot role and no allowed (operator) role; an unlisted host, or any host when the registry cannot be read, is
    refused when its name matches a robot pattern."""
    if host.startswith('local:'):
        return None, []
    cfg = guard_config() if cfg is None else cfg
    environ = os.environ if environ is None else environ
    name = host.rsplit('@', 1)[-1]
    try:
        ipaddress.ip_address(name)
        names = {name}
    except ValueError:
        names = {name, name.split('.', 1)[0]}
    root = environ.get('ASTRO_PROFILES') or cfg.get('registry') or '~/astro_dock/profiles'
    warnings, roles, listed = [], set(), False
    try:
        for entry, (entry_roles, addrs) in _registry(root).items():
            if names & ({entry} | addrs):
                listed, roles = True, roles | entry_roles
    except (OSError, ValueError) as error:
        warnings.append(f"cannot read astro's host registry ({root}: {error}); checking {name} against the robot "
                        'name patterns only')
    allowed, robot = set(cfg.get('allowed_roles') or ()), set(cfg.get('robot_roles') or ())
    if listed:
        refused = bool(roles & robot) and not roles & allowed
    else:
        refused = any(fnmatch.fnmatch(n, pat) for n in names for pat in cfg.get('robot_patterns') or ())
    return (ROBOT_REFUSAL.format(host=name) if refused else None), warnings


class DeployError(RuntimeError):
    def __init__(self, cause, fix=None, code=output.FAILED):
        super().__init__(cause)
        self.fix, self.code = fix, code


# ---- transports: run a script in the host's home, send a file or a dir there --------------------------------

class Ssh:
    def __init__(self, host):
        self.host, self.name = host, host

    def run(self, script, timeout=600):
        return subprocess.run(['ssh', *SSH_OPTS, self.host, 'bash -c ' + shlex.quote(script)], capture_output=True,
                              text=True, timeout=timeout)

    def send(self, src, rel):
        """rsync src to HOME/rel on the host (--partial: a cut transfer resumes)."""
        cmd = ['rsync', '-a', '--partial', '-e', 'ssh ' + ' '.join(SSH_OPTS), str(src), f'{self.host}:{rel}']
        return subprocess.run(cmd, capture_output=True, text=True)

    def fetch_url(self, source):
        return source or os.environ.get('CRANE_DEPLOY_FROM') or \
            f'ssh://{getpass.getuser()}@{socket.gethostname()}{paths.ROOT}'


class Local:
    """local:DIR: DIR stands in for the host's home (scripts run with HOME=DIR); no ssh."""

    def __init__(self, home):
        self.home = Path(home).expanduser().resolve()
        self.name = f'local:{self.home}'

    def run(self, script, timeout=600):
        return subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=timeout,
                              cwd=self.home if self.home.is_dir() else None, env={**os.environ, 'HOME': str(self.home)})

    def send(self, src, rel):
        dest = self.home / rel
        if shutil.which('rsync'):
            return subprocess.run(['rsync', '-a', '--partial', str(src), str(dest)], capture_output=True, text=True)
        src = Path(str(src).rstrip('/'))
        if src.is_dir():
            shutil.copytree(src, dest, dirs_exist_ok=True)
        else:
            shutil.copy2(src, dest / src.name if str(rel).endswith('/') else dest)
        return subprocess.CompletedProcess([], 0, '', '')

    def fetch_url(self, source):
        return source or str(paths.ROOT)


def transport(host):
    return Local(host[len('local:'):]) if host.startswith('local:') else Ssh(host)


def _checked(proc, what, fix=None):
    if proc.returncode:
        tail = (proc.stderr or proc.stdout or '').strip().splitlines()[-1:] or [f'exit {proc.returncode}']
        raise DeployError(f'{what}: {tail[0]}', fix)
    return proc


# ---- the steps -----------------------------------------------------------------------------------------------

def probe(t):
    try:
        proc = t.run(PROBE, timeout=60)
    except subprocess.TimeoutExpired:
        raise DeployError(f'{t.name} did not answer in 60 s', f'ssh {t.name} true') from None
    if proc.returncode:
        detail = (proc.stderr.strip().splitlines() or [f'exit {proc.returncode}'])[-1]
        fix = f'mkdir -p {t.home}' if isinstance(t, Local) else f'ssh {t.name} true   (keys, host name)'
        raise DeployError(f'cannot reach {t.name}: {detail}', fix)
    info = dict(line.split('=', 1) for line in proc.stdout.splitlines() if '=' in line)
    info['free_bytes'] = int(info.get('free_kb') or 0) * 1024
    return info


def resolve_ref(ref):
    proc = subprocess.run(['git', '-C', str(paths.ROOT), 'rev-parse', '--verify', '--quiet', f'{ref}^{{commit}}'],
                          capture_output=True, text=True)
    if proc.returncode:
        raise DeployError(f'no ref {ref!r} in {paths.ROOT}', 'crane deploy HOST --ref BRANCH', output.USAGE)
    return proc.stdout.strip()


def tarball_commit(tarball):
    m = update.TARBALL.search(Path(tarball).name)
    return m.group(1) if m else None


def find_tarball(path=None):
    """(tarball, sha256, release player dict or None): --tarball, else the newest Builds tarball with its .sha256,
    else the release player (downloaded into ~/.cache/crane/downloads)."""
    if path:
        tarball = Path(path).expanduser().resolve()
        side = tarball.with_name(tarball.name + '.sha256')
        if not tarball.is_file() or not side.is_file():
            raise DeployError(f'no {tarball} with its .sha256', 'crane build player --tarball', output.USAGE)
        return tarball, update.read_sha256(side), None
    built = sorted((p for p in paths.BUILDS.glob('crane-player-*-linux-x86_64.tar.*')
                    if not p.name.endswith('.sha256') and p.with_name(p.name + '.sha256').is_file()),
                   key=lambda p: p.stat().st_mtime)
    if built:
        return built[-1], update.read_sha256(built[-1].with_name(built[-1].name + '.sha256')), None
    try:
        release, url = update.fetch_release()
    except update.ReleaseError as error:
        raise DeployError(f'no player tarball in {paths.BUILDS} and {error}',
                          'crane build player --tarball, or --tarball PATH', output.USAGE) from None
    player = release.get('player') or {}
    if not player:
        raise DeployError(f'release {release["version"]} has no player', 'crane build player --tarball', output.USAGE)
    return None, player['sha256'], {**player, 'url': update.asset_url(url, player['name']),
                                    'version': release['version']}


def code_plan(t, info, ref, commit, source):
    if info.get('checkout') == 'git':
        url = t.fetch_url(source)
        script = (f'cd ~/{REPO_REL} && git fetch --quiet {shlex.quote(url)} {commit} && '
                  'git merge --ff-only --quiet FETCH_HEAD && git rev-parse HEAD')
        return 'git', f'git fetch {url} {commit[:12]} ({ref}); git merge --ff-only FETCH_HEAD', script
    return 'rsync', f'git archive {commit[:12]} (no {", ".join(UNITY)}) | rsync -a to ~/{REPO_REL}/', None


def sync_code(t, info, ref, commit, source):
    how, _, script = code_plan(t, info, ref, commit, source)
    if how == 'git':
        if info.get('head') == commit:
            return 'already there'
        proc = _checked(t.run(script), 'code', f'ssh {t.name} "cd ~/{REPO_REL} && git status"   (commit or stash '
                        'there; it must fast-forward)')
        head = proc.stdout.strip().splitlines()[-1:]
        if head and head[0] != commit:
            raise DeployError(f'code: {t.name} is at {head[0][:12]} after the merge, not {commit[:12]}',
                              f'crane deploy {t.name} --ref {commit}')
        return f'{(info.get("head") or "?")[:12]} -> {commit[:12]}'
    with tempfile.TemporaryDirectory(prefix='crane-deploy-', dir=_scratch()) as tmp:
        tree = Path(tmp) / REPO_REL
        tree.mkdir()
        spec = ['--', '.', *(f':(exclude){d}' for d in UNITY)]
        archive = subprocess.Popen(['git', '-C', str(paths.ROOT), 'archive', '--format=tar', commit, *spec],
                                   stdout=subprocess.PIPE)
        _checked(subprocess.run(['tar', '-x', '-C', str(tree)], stdin=archive.stdout, capture_output=True,
                                text=True), 'code: unpack the archive')
        archive.stdout.close()
        if archive.wait():
            raise DeployError('code: git archive failed', f'git -C {paths.ROOT} archive {commit[:12]}')
        (tree / '.crane-deploy.json').write_text(json.dumps({'commit': commit, 'ref': ref}) + '\n')
        _checked(t.run(f'mkdir -p ~/{REPO_REL}'), 'code: mkdir')
        _checked(t.send(f'{tree}/', f'{REPO_REL}/'), 'code: rsync')
    return f'{(info.get("head") or "none")[:12]} -> {commit[:12]} (files)'


def venv_plan(info, components, explicit, skip):
    """The install.sh argv run in ~/crane_ml on the host, or None: only when there is no .venv there, or
    --components was given (install.sh is idempotent: it keeps a 3.12 venv and reinstalls the extras)."""
    if skip or (info.get('venv') == 'yes' and not explicit):
        return None
    return ['./install.sh', '--yes', '--no-clone', '--components', components]


def make_venv(t, argv):
    script = f'cd ~/{REPO_REL} && ' + ' '.join(shlex.quote(a) for a in argv) + ' </dev/null'
    _checked(t.run(script, timeout=1800), 'venv', f'ssh {t.name} "cd ~/{REPO_REL} && {" ".join(argv)}"   (its '
             'output names the step)' if isinstance(t, Ssh) else f'cd {t.home}/{REPO_REL} && {" ".join(argv)}')
    return f'.venv with {argv[-1]}'


def _scratch():
    d = paths.HOME / '.cache' / 'crane' / 'deploy'
    d.mkdir(parents=True, exist_ok=True)
    return d


def send_player(t, tarball, sha256):
    incoming = f'{STORE_REL}/.incoming'
    _checked(t.run(f'mkdir -p ~/{incoming}'), 'player: mkdir')
    _checked(t.send(tarball, f'{incoming}/'), 'player: rsync', f'crane deploy {t.name} --player   (resumes)')
    remote = f'~/{incoming}/{shlex.quote(tarball.name)}'
    script = (f'cd ~/{REPO_REL} && bin/crane update --json --tarball {remote} --sha256 {shlex.quote(sha256)} '
              f'--store ~/{STORE_REL}; code=$?; rm -f {remote}; exit $code')
    proc = t.run(script)
    if proc.returncode:
        cause = (proc.stderr.strip().splitlines() or [f'exit {proc.returncode}'])[-1]
        raise DeployError(f'player on {t.name}: {cause.removeprefix("crane update: ")}',
                          f'crane deploy {t.name} --player   (sends it again)')
    return json.loads(proc.stdout)


def remote_doctor(t):
    proc = t.run(f'cd ~/{REPO_REL} && bin/crane doctor --json', timeout=120)
    try:
        return json.loads(proc.stdout), proc.returncode
    except ValueError:
        tail = (proc.stderr.strip().splitlines() or [f'exit {proc.returncode}'])[-1]
        raise DeployError(f'doctor on {t.name}: {tail}', f'ssh {t.name} ~/{REPO_REL}/bin/crane doctor') from None


def _row(out, step, detail):
    out.line(f'  {step:<7} {detail}')


# ---- crane deploy --------------------------------------------------------------------------------------------

def main(args):
    out = args.output
    t = transport(args.host)
    want_player = bool(args.player or args.tarball)
    doc = {'schema': 'crane.deploy/1', 'host': t.name, 'dry_run': out.dry_run, 'steps': []}
    refusal, warnings = robot_guard(args.host)
    for warning in warnings:
        print(f'crane deploy: warning: {warning}', file=out.stderr)
    if refusal:
        out.doc({**doc, 'ok': False, 'error': refusal})
        print(f'crane deploy: {refusal}', file=out.stderr)
        return output.USAGE
    try:
        commit = resolve_ref(args.ref)
        info = probe(t)
        doc['probe'] = info
        out.line(f'crane deploy {t.name}' + (' (dry run: nothing changes there)' if out.dry_run else ''))
        _row(out, 'probe', f'{update.gb(info["free_bytes"])} free, checkout {info.get("checkout")} at '
             f'{(info.get("head") or "none")[:12]}, player {info.get("player") or "none"}, python '
             f'{info.get("python") or "none"}')
        if not info.get('python'):
            raise DeployError(f'{t.name} has no python3', f'install python3 >= 3.10 on {t.name}', output.USAGE)
        if info.get('checkout') == 'foreign':
            raise DeployError(f'~/{REPO_REL} on {t.name} is not a crane checkout or an earlier deploy',
                              f'move it aside on {t.name} (mv ~/{REPO_REL} ~/{REPO_REL}.old), then deploy again',
                              output.USAGE)
        if info.get('checkout') != 'git' and info.get('rsync') != 'yes' and isinstance(t, Ssh):
            raise DeployError(f'{t.name} has no rsync', f'install rsync on {t.name}', output.USAGE)

        tarball = sha = rel_player = None
        need = update.HEADROOM
        if want_player:
            tarball, sha, rel_player = find_tarball(args.tarball)
            name = update.player_name(tarball or rel_player['name'], sha)
            built = tarball_commit(tarball or rel_player['name'])
            if tarball and not args.tarball and built and not commit.startswith(built):
                note = (f'{tarball.name} was built from {built[:12]}, the code is {commit[:12]} (--tarball PATH '
                        'picks one; crane build player --tarball makes a matching one)')
                doc.setdefault('warnings', []).append(note)
                _row(out, 'warn', note)
            size = tarball.stat().st_size if tarball else int(rel_player.get('bytes') or 0)
            if info.get('player') == name:
                want_player = False
                doc['steps'].append({'step': 'player', 'detail': f'{name} is current there'})
                _row(out, 'player', f'{name} is current there')
            else:
                need = update.player_need({'bytes': size, 'unpacked_bytes': (rel_player or {}).get('unpacked_bytes')})
            if want_player and tarball and tarball.name.endswith('.zst') and info.get('zstd') != 'yes':
                raise DeployError(f'{t.name} has no zstd for {tarball.name}', f'install zstd on {t.name}',
                                  output.USAGE)
        if info['free_bytes'] < need:
            raise DeployError(f'{update.gb(info["free_bytes"])} free on {t.name}, the deploy needs {update.gb(need)} '
                              f'({"the player and " if want_player else ""}{update.gb(update.HEADROOM)} headroom)',
                              f'free space on {t.name}, or deploy without --player', output.USAGE)

        how, shown, _ = code_plan(t, info, args.ref, commit, args.source)
        venv = venv_plan(info, args.components or 'core', bool(args.components), args.no_venv)
        plan = [('code', shown)]
        if venv:
            plan.append(('venv', f'~/{REPO_REL}/' + ' '.join(venv)[2:]))
        if want_player:
            label = tarball.name if tarball else f'{rel_player["name"]} (release {rel_player["version"]})'
            plan.append(('player', f'{label}: rsync --partial to ~/{STORE_REL}/.incoming/, sha256 {sha[:12]} '
                         'checked there, current + prev'))
        plan.append(('doctor', f'~/{REPO_REL}/bin/crane doctor --json'))
        if out.dry_run:
            doc['steps'] += [{'step': s, 'plan': d} for s, d in plan]
            if out.doc({**doc, 'ok': True}):
                return output.OK
            for step, detail in plan:
                _row(out, step, detail)
            out.next(f'crane deploy {args.host}' + (' --player' if args.player else '') +
                     (f' --ref {args.ref}' if args.ref != 'HEAD' else ''))
            return output.OK

        detail = sync_code(t, info, args.ref, commit, args.source)
        doc['steps'].append({'step': 'code', 'how': how, 'detail': detail})
        _row(out, 'code', f'{how}: {detail}')
        if venv:
            detail = make_venv(t, venv)
            doc['steps'].append({'step': 'venv', 'argv': venv, 'detail': detail})
            _row(out, 'venv', detail)
        if want_player:
            if tarball is None:
                tarball = paths.HOME / '.cache' / 'crane' / 'downloads' / rel_player['version'] / \
                    rel_player['name']
                update.download(rel_player['url'], tarball, sha)
            state = send_player(t, tarball, sha)
            detail = (f'{state["name"]} {"installed" if state["installed"] else "already there"} (current '
                      f'{state["current"]}, prev {state["prev"] or "none"})')
            doc['steps'].append({'step': 'player', 'detail': detail, 'store': state})
            _row(out, 'player', detail)
        report, code = remote_doctor(t)
        doc['doctor'] = report
    except (DeployError, update.ReleaseError, update.IntegrityError) as error:
        fix, code = getattr(error, 'fix', None), getattr(error, 'code', output.FAILED)
        out.doc({**doc, 'ok': False, 'error': str(error)})
        return out.fail(f'deploy {args.host}', str(error), fix, code)
    ok = code == 0
    if out.doc({**doc, 'ok': ok}):
        return output.OK if ok else output.FAILED
    _row(out, 'doctor', f'on {t.name}')
    out.table([(c['name'], c['state'], c['detail']) for c in report.get('checks', [])], indent=4)
    nxt = [f'crane deploy {args.host} --components core   (a .venv there: install.sh)'] \
        if info.get('venv') != 'yes' and args.no_venv else []
    nxt += [f'ssh {args.host} {c}' if isinstance(t, Ssh) else c for c in report.get('next', [])[:2]]
    out.next(*nxt)
    return output.OK if ok else output.FAILED
