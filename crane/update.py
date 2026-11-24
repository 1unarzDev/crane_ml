"""crane update: this checkout or this install from a release, and the player store crane deploy shares.

    crane update                       # a git checkout: git fetch + ff-only, then the release player
    crane update --version 0.3.0       # that release (a checkout fast-forwards to its tag v0.3.0)
    crane update --player-only         # only the player (install.sh --components ...,player)
    crane update --tarball T.tar.zst --sha256 HEX --store DIR    # install a local player tarball

A release is the assets of a GitHub Release (release.yml): release.json, the wheel, the sdist and the player
tarball. release.json: {schema: crane.release/1, version, tag, commit, unity_version,
player: {name, sha256, bytes, unpacked_bytes}, wheel: {name, sha256}, sdist: {name, sha256}}.

The player store (paths.PLAYER_STORE, ~/.local/share/crane/player) holds one dir per player, <commit12>/, with the
CRANE-Worker tree in it, plus `current` and `prev` symlinks; older players are pruned. Every tarball is checked by
sha256 before it is unpacked. Stdlib only.
"""

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import urllib.error
import urllib.request
from pathlib import Path

from crane import output, paths

RELEASES = 'https://github.com/1unarzDev/crane_ml/releases'
HEADROOM = 1 << 30                       # free space kept after a player is unpacked (1 GB)
TARBALL = re.compile(r'crane-player-([0-9a-f]{7,40})-linux-x86_64\.tar\.(zst|gz)$')
KEEP = ('current', 'prev')


class ReleaseError(RuntimeError):
    """No release, or an asset that cannot be fetched; the message names the URL."""


class IntegrityError(RuntimeError):
    """A file whose sha256 differs from the one it was published with."""


class DiskError(RuntimeError):
    """Not enough free space; the message gives the numbers."""


# ---- releases ------------------------------------------------------------------------------------------------

def releases_base():
    """CRANE_RELEASES (a URL or a local dir laid out like GitHub's), else the GitHub releases of the repo."""
    base = os.environ.get('CRANE_RELEASES', RELEASES).rstrip('/')
    return base if re.match(r'^[a-z]+://', base) else Path(base).expanduser().resolve().as_uri()


def release_url(version=None, base=None):
    """release.json's URL: the latest release, or v<version>'s."""
    base = base or releases_base()
    return f'{base}/latest/download/release.json' if not version else \
        f'{base}/download/v{str(version).lstrip("v")}/release.json'


def asset_url(release_json_url, name):
    return release_json_url.rsplit('/', 1)[0] + '/' + name


def fetch_release(version=None, timeout=30):
    """(release dict, its URL). ReleaseError when there is none."""
    url = release_url(version)
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            doc = json.loads(response.read().decode())
    except (urllib.error.URLError, OSError, ValueError) as error:
        raise ReleaseError(f'no release at {url} ({getattr(error, "reason", error)})') from None
    if not isinstance(doc, dict) or not doc.get('version'):
        raise ReleaseError(f'{url} is not a crane.release/1 document')
    return doc, url


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def read_sha256(path):
    """The digest in a sha256sum-format file (`HEX  NAME`)."""
    return Path(path).read_text().split()[0].strip().lower()


def download(url, dest, sha256, timeout=60):
    """url to dest (through dest.part), checked against sha256; a dest already there with that digest is kept."""
    dest = Path(dest)
    if dest.is_file() and sha256_file(dest) == sha256:
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_name(dest.name + '.part')
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response, open(part, 'wb') as sink:
            shutil.copyfileobj(response, sink, 1 << 20)
    except (urllib.error.URLError, OSError) as error:
        part.unlink(missing_ok=True)
        raise ReleaseError(f'cannot fetch {url} ({getattr(error, "reason", error)})') from None
    got = sha256_file(part)
    if got != sha256:
        part.unlink(missing_ok=True)
        raise IntegrityError(f'{dest.name} has sha256 {got[:12]}, the release says {sha256[:12]}')
    part.replace(dest)
    return dest


# ---- disk ----------------------------------------------------------------------------------------------------

def free_bytes(path):
    """Free bytes on the filesystem of path (its nearest existing parent)."""
    path = Path(path)
    while not path.exists() and path != path.parent:
        path = path.parent
    return shutil.disk_usage(path).free


def player_need(player, download=True):
    """Bytes a player install needs: the tarball (when it is downloaded here), it unpacked, and the headroom."""
    size = int(player.get('bytes') or 0)
    unpacked = int(player.get('unpacked_bytes') or 4 * size)
    return (size if download else 0) + unpacked + HEADROOM


def gb(n):
    return f'{n / 1e9:.1f} GB'


def check_disk(where, need, what='the player'):
    free = free_bytes(where)
    if free < need:
        raise DiskError(f'{gb(free)} free at {where}, {what} needs {gb(need)} (with {gb(HEADROOM)} headroom)')
    return free


# ---- the player store ----------------------------------------------------------------------------------------

def store_state(store=None):
    """{'store', 'current', 'prev', 'players'}: the names `current` and `prev` point at, and every player dir."""
    store = Path(store or paths.PLAYER_STORE)

    def target(link):
        p = store / link
        return os.readlink(p) if p.is_symlink() else None
    players = sorted(p.name for p in store.iterdir() if p.is_dir() and not p.is_symlink()
                     and not p.name.startswith('.')) if store.is_dir() else []
    return {'store': str(store), 'current': target('current'), 'prev': target('prev'), 'players': players}


def player_name(tarball, sha256):
    """The store dir name of a tarball: its commit12 (crane-player-<commit12>-linux-x86_64.tar.*), else sha256[:12]."""
    m = TARBALL.search(Path(tarball).name)
    return m.group(1)[:12] if m else sha256[:12]


def _members(tar, prefix='CRANE-Worker/'):
    """The tarball's members with the CRANE-Worker/ prefix stripped; anything absolute, outside or special is
    refused (a crafted tarball cannot write outside the player dir)."""
    seen = set()
    for m in tar:
        name = m.name[2:] if m.name.startswith('./') else m.name
        if name.rstrip('/') == prefix.rstrip('/'):
            continue
        if not name.startswith(prefix):
            raise IntegrityError(f'tarball member {m.name!r} is outside {prefix}')
        m.name = name[len(prefix):]
        if m.islnk():                    # a hard link names an earlier member, inside the player
            target = m.linkname[2:] if m.linkname.startswith('./') else m.linkname
            if not target.startswith(prefix) or target[len(prefix):] not in seen:
                raise IntegrityError(f'tarball hard link {m.name!r} names {m.linkname!r}, not an earlier member')
            m.linkname = target[len(prefix):]
        if (m.name.startswith('/') or '..' in Path(m.name).parts or not (m.isfile() or m.isdir() or m.issym()
                                                                            or m.islnk())):
            raise IntegrityError(f'tarball member {m.name!r} is not a plain file, dir or link inside the player')
        if (m.issym() or m.islnk()) and (m.linkname.startswith('/') or
                                         '..' in Path(m.name).parent.joinpath(m.linkname).parts):
            raise IntegrityError(f'tarball link {m.name!r} points outside the player')
        m.mode &= 0o755
        seen.add(m.name.rstrip('/'))
        yield m


def _unpack(tarball, dest):
    """Stream tarball (.tar.zst through zstd, else gz/plain) into dest."""
    dest.mkdir(parents=True)
    proc = None
    if str(tarball).endswith('.zst'):
        zstd = shutil.which('zstd')
        if not zstd:
            raise RuntimeError(f'{Path(tarball).name} is zstd: install zstd (apt install zstd)')
        proc = subprocess.Popen([zstd, '-dcq', str(tarball)], stdout=subprocess.PIPE)
        tar = tarfile.open(fileobj=proc.stdout, mode='r|')
    else:
        tar = tarfile.open(tarball, mode='r|*')
    kw = {'filter': 'fully_trusted'} if hasattr(tarfile, 'data_filter') else {}   # _members checked each one
    try:
        with tar:
            for m in _members(tar):
                tar.extract(m, dest, **kw)
    except (KeyError, tarfile.TarError, OSError) as error:
        raise IntegrityError(f'{Path(tarball).name} does not unpack ({error})') from None
    finally:
        if proc:
            proc.stdout.close()
            if proc.wait():
                raise RuntimeError(f'zstd -d failed on {Path(tarball).name} ({proc.returncode})')


def _point(store, link, name):
    tmp = store / f'.{link}.tmp'
    tmp.unlink(missing_ok=True)
    os.symlink(name, tmp)
    os.replace(tmp, store / link)


def install_player(tarball, sha256, store=None, name=None):
    """Check tarball's sha256, unpack it into STORE/<name>/, point `current` at it and `prev` at the old current,
    prune every other player. Returns store_state plus {'name', 'installed' (False: it was already there)}."""
    store = Path(store or paths.PLAYER_STORE)
    sha256 = sha256.strip().lower()
    got = sha256_file(tarball)
    if got != sha256:
        raise IntegrityError(f'{Path(tarball).name} has sha256 {got[:12]}, expected {sha256[:12]}')
    name = name or player_name(tarball, sha256)
    store.mkdir(parents=True, exist_ok=True)
    state = store_state(store)
    installed = False
    if not (store / name / 'CRANE.x86_64').is_file():
        tmp = store / f'.unpack-{name}'
        shutil.rmtree(tmp, ignore_errors=True)
        try:
            _unpack(tarball, tmp)
            if not (tmp / 'CRANE.x86_64').is_file():
                raise IntegrityError(f'{Path(tarball).name} has no CRANE-Worker/CRANE.x86_64')
            shutil.rmtree(store / name, ignore_errors=True)
            tmp.rename(store / name)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        (store / name / '.sha256').write_text(f'{sha256}  {Path(tarball).name}\n')
        installed = True
    if state['current'] != name:
        if state['current'] and (store / state['current']).is_dir():
            _point(store, 'prev', state['current'])
        _point(store, 'current', name)
    prune(store)
    return {**store_state(store), 'name': name, 'installed': installed}


def prune(store=None):
    """Remove every player dir that `current` and `prev` do not point at (and stale unpack dirs)."""
    store = Path(store or paths.PLAYER_STORE)
    state = store_state(store)
    keep = {state['current'], state['prev']}
    removed = []
    for p in store.iterdir():
        if p.is_symlink() or not p.is_dir() or p.name in keep or p.name == '.incoming':
            continue
        shutil.rmtree(p)
        removed.append(p.name)
    return removed


# ---- the code ------------------------------------------------------------------------------------------------

def _git(*a, check=True):
    return subprocess.run(['git', '-C', str(paths.ROOT), *a], capture_output=True, text=True, check=check)


def is_checkout(root=None):
    return (Path(root or paths.ROOT) / '.git').exists()


def code_plan(version, release):
    """[(label, argv)] for the code step: a checkout fast-forwards, an install pip-installs the wheel."""
    if is_checkout():
        target = f'v{str(version).lstrip("v")}' if version else '@{upstream}'
        fetch = ['git', '-C', str(paths.ROOT), 'fetch', '--quiet', '--tags']
        return [('fetch', fetch), ('ff-only', ['git', '-C', str(paths.ROOT), 'merge', '--ff-only', target])]
    if not release or not release.get('wheel'):
        return []
    from crane import __version__
    if release['version'] == __version__:
        return []
    uv = shutil.which('uv')
    pip = [uv, 'pip', 'install', '--python', sys.executable] if uv else [sys.executable, '-m', 'pip', 'install',
                                                                       '--quiet']
    return [('wheel', [*pip, '{wheel}'])]


# ---- crane update --------------------------------------------------------------------------------------------

def main(args):
    out = args.output
    store = Path(args.store).expanduser() if getattr(args, 'store', None) else paths.PLAYER_STORE
    if args.tarball:
        return _install_local(args, out, store)
    rows, doc = [], {'schema': 'crane.update/1', 'dry_run': out.dry_run, 'checkout': is_checkout(),
                     'store': str(store)}
    try:
        release, url = fetch_release(args.version)
    except ReleaseError as error:
        release, url = None, None
        if args.version or not is_checkout() or args.player_only:
            if args.player_only and not args.version:     # install.sh: no release yet is not an error there
                out.say(f'crane update: no release yet ({error}); the player comes with '
                        'crane build player --tarball, or set CRANE_PLAYER')
                return output.OK
            return out.fail('update', str(error), 'crane update --version X.Y.Z, or set CRANE_RELEASES')
        doc['release_error'] = str(error)
    doc['release'] = release
    downloads = paths.HOME / '.cache' / 'crane' / 'downloads' / (release['version'] if release else 'none')

    # 1. the code
    steps = [] if args.player_only else code_plan(args.version, release)
    wheel = None
    if any(label == 'wheel' for label, _ in steps):
        wheel = downloads / release['wheel']['name']
    for label, argv in steps:
        shown = [str(wheel) if a == '{wheel}' else a for a in argv]
        rows.append(('code', label, ' '.join(shown)))
        if out.dry_run:
            continue
        if label == 'wheel':
            try:
                download(asset_url(url, wheel.name), wheel, release['wheel']['sha256'])
            except (ReleaseError, IntegrityError) as error:
                return out.fail('update', str(error), 'crane update (again), or crane update --version X.Y.Z')
        proc = subprocess.run(shown, capture_output=True, text=True)
        if proc.returncode:
            detail = (proc.stderr or proc.stdout).strip().splitlines()[-1:] or [f'exit {proc.returncode}']
            fix = ('git -C ' + str(paths.ROOT) + ' status   (commit or stash, then crane update)') if label != 'wheel' \
                else 'pip install the wheel by hand: ' + ' '.join(shown)
            return out.fail('update', f'{label}: {detail[0]}', fix)
    if not steps and not args.player_only:
        rows.append(('code', 'ok', 'already at the release' if not is_checkout() else 'nothing to do'))
    if is_checkout() and steps and not out.dry_run:
        head = _git('rev-parse', 'HEAD', check=False).stdout.strip()
        rows.append(('code', 'at', head[:12]))
        if release and release.get('commit') and head != release['commit']:
            note = (f'HEAD {head[:12]} is not release {release["version"]}\'s commit {release["commit"][:12]}; the '
                    'player below is the release\'s')
            rows.append(('code', 'warn', note))
            doc['warnings'] = [note]

    # 2. the player
    player = (release or {}).get('player')
    state = store_state(store)
    if args.no_player:
        rows.append(('player', 'skipped', '--no-player'))
    elif not player:
        rows.append(('player', 'none', 'no release player yet: crane build player --tarball, or set CRANE_PLAYER'))
    else:
        name = player_name(player['name'], player['sha256'])
        if state['current'] == name and (store / name / 'CRANE.x86_64').is_file():
            rows.append(('player', 'ok', f'{store / name} is current'))
        else:
            need = player_need(player)
            try:
                check_disk(store.parent, need)
            except DiskError as error:
                out.doc({**doc, 'ok': False, 'error': str(error)})
                return out.fail('update', str(error), 'free space, or crane update --no-player', output.USAGE)
            tarball = downloads / player['name']
            rows.append(('player', 'fetch', f'{player["name"]} ({gb(player.get("bytes") or 0)}), sha256 '
                         f'{player["sha256"][:12]}'))
            rows.append(('player', 'install', f'{store / name} (current; prev {state["current"] or "none"})'))
            if not out.dry_run:
                try:
                    download(asset_url(url, player['name']), tarball, player['sha256'])
                    state = install_player(tarball, player['sha256'], store, name)
                except (ReleaseError, IntegrityError, RuntimeError) as error:
                    return out.fail('update', str(error), 'crane update (again); a mismatch twice: report the '
                                    'release')
                finally:
                    tarball.unlink(missing_ok=True)
    doc.update(ok=True, steps=[{'part': a, 'step': b, 'detail': c} for a, b, c in rows], player_store=state)
    if out.doc(doc):
        return output.OK
    out.line('crane update' + (' (dry run: nothing changed)' if out.dry_run else '') +
             (f': release {release["version"]} ({str(release.get("commit", ""))[:12]})' if release else
              ': no release (code only)'))
    out.table(rows)
    out.next('crane update' if out.dry_run else 'crane doctor')
    return output.OK


def _install_local(args, out, store):
    """--tarball T [--sha256 HEX]: install a local player tarball (its .sha256 beside it when no --sha256)."""
    tarball = Path(args.tarball).expanduser()
    sha = args.sha256 or (read_sha256(tarball.with_name(tarball.name + '.sha256'))
                          if tarball.with_name(tarball.name + '.sha256').is_file() else None)
    if not tarball.is_file():
        return out.usage('update', f'no tarball {tarball}', 'crane build player --tarball')
    if not sha:
        return out.usage('update', f'no sha256 for {tarball.name}', f'crane update --tarball {tarball} --sha256 HEX')
    if out.dry_run:
        out.say(f'crane update: would install {tarball.name} into {store}/{player_name(tarball, sha)}')
        return output.OK
    try:
        check_disk(store.parent, player_need({'bytes': tarball.stat().st_size}, download=False))
        state = install_player(tarball, sha, store)
    except DiskError as error:
        return out.fail('update', str(error), 'free space on that disk', output.USAGE)
    except (IntegrityError, RuntimeError) as error:
        return out.fail('update', str(error), 'fetch the tarball again (rsync --partial resumes it)')
    if out.doc({'schema': 'crane.update.player/1', 'ok': True, **state}):
        return output.OK
    out.say(f'crane update: player {state["name"]} {"installed" if state["installed"] else "already there"} in '
            f'{state["store"]} (current {state["current"]}, prev {state["prev"] or "none"})')
    return output.OK


# ---- release.json (release.yml writes it; tests build fake releases with it) --------------------------------

def unpacked_bytes(tarball):
    """The bytes a player tarball holds unpacked (the sum of its members' sizes)."""
    total, proc, manifest = 0, None, {}
    if str(tarball).endswith('.zst'):
        proc = subprocess.Popen([shutil.which('zstd') or 'zstd', '-dcq', str(tarball)], stdout=subprocess.PIPE)
        tar = tarfile.open(fileobj=proc.stdout, mode='r|')
    else:
        tar = tarfile.open(tarball, mode='r|*')
    with tar:
        for m in tar:
            total += m.size if m.isfile() else 0
            if m.name.endswith('crane-build-manifest.json'):
                manifest = json.load(tar.extractfile(m))
    if proc:
        proc.stdout.close()
        proc.wait()
    return total, manifest


def make_release(version, tag, commit, unity_version, player=None, wheel=None, sdist=None):
    """The crane.release/1 document for these files (paths; player optional)."""
    def entry(path):
        path = Path(path)
        return {'name': path.name, 'sha256': sha256_file(path), 'bytes': path.stat().st_size}
    doc = {'schema': 'crane.release/1', 'version': str(version).lstrip('v'), 'tag': tag, 'commit': commit,
           'unity_version': unity_version}
    if player:
        size, manifest = unpacked_bytes(player)
        doc['player'] = {**entry(player), 'unpacked_bytes': size, 'source_commit': manifest.get('sourceCommit'),
                         'unity_version': manifest.get('unityVersion')}
    for key, path in (('wheel', wheel), ('sdist', sdist)):
        if path:
            doc[key] = entry(path)
    return doc


def _release_main(argv):
    """python3 -m crane.update release-json --version V --tag T --commit C --unity U [--player P] [--wheel W]
    [--sdist S] --out release.json: checks the player was built from the commit and with that Unity."""
    import argparse
    ap = argparse.ArgumentParser(prog='python3 -m crane.update release-json')
    for name in ('--version', '--tag', '--commit', '--unity', '--out'):
        ap.add_argument(name, required=True)
    for name in ('--player', '--wheel', '--sdist'):
        ap.add_argument(name)
    a = ap.parse_args(argv)
    doc = make_release(a.version, a.tag, a.commit, a.unity, a.player, a.wheel, a.sdist)
    player = doc.get('player') or {}
    if player and player.get('source_commit') != a.commit:
        print(f'release-json: the player was built from {player.get("source_commit")}, not {a.commit}', file=sys.stderr)
        return 1
    if player and player.get('unity_version') and player['unity_version'] != a.unity:
        print(f'release-json: the player was built with Unity {player["unity_version"]}, the project pins {a.unity}',
              file=sys.stderr)
        return 1
    Path(a.out).write_text(json.dumps(doc, indent=1) + '\n')
    print(json.dumps(doc, indent=1))
    return 0


if __name__ == '__main__':
    if sys.argv[1:2] == ['release-json']:
        sys.exit(_release_main(sys.argv[2:]))
    print('usage: python3 -m crane.update release-json ... (crane update is the command)', file=sys.stderr)
    sys.exit(2)
