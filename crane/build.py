"""crane build player: the headless Linux player build, its manifest stamp and its release tarball."""

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
from datetime import datetime, timezone
from pathlib import Path

from crane import paths

EXECUTE_METHOD = 'CranePerformanceBuild.BuildLinuxWorker'
# Left out of the tarball: debug symbols and Burst's do-not-ship folder (the player runs without them).
LEAVE_OUT = re.compile(r'(_s\.debug$|_BurstDebugInformation_DoNotShip)')


def editor_path(project=paths.ROOT):
    """UNITY_EDITOR, else ~/Unity/Hub/Editor/<ProjectVersion.txt's m_EditorVersion>/Editor/Unity."""
    if os.environ.get('UNITY_EDITOR'):
        return Path(os.environ['UNITY_EDITOR'])
    text = (Path(project) / 'ProjectSettings' / 'ProjectVersion.txt').read_text()
    version = re.search(r'^m_EditorVersion:\s*(\S+)', text, re.M).group(1)
    return Path.home() / 'Unity' / 'Hub' / 'Editor' / version / 'Editor' / 'Unity'


def source_state(repo=paths.ROOT):
    """(commit, dirty) of the checkout; dirty means tracked files differ from HEAD."""
    def git(*a):
        return subprocess.run(['git', '-C', str(repo), *a], capture_output=True, text=True, check=True).stdout
    return git('rev-parse', 'HEAD').strip(), bool(git('status', '--porcelain', '--untracked-files=no').strip())


def unity_command(editor, log, commit, dirty, project=paths.ROOT):
    return [str(paths.HEAVY), 'unity', str(editor), '-batchmode', '-nographics', '-quit',
            '-projectPath', str(project), '-buildTarget', 'StandaloneLinux64', '-executeMethod', EXECUTE_METHOD,
            '-logFile', str(log), '--crane-source-commit', commit, '--crane-source-dirty', str(dirty).lower()]


def stamp_manifest(build_dir, commit, dirty):
    """Write sourceCommit and sourceDirty into build_dir/crane-build-manifest.json; returns the manifest."""
    path = Path(build_dir) / 'crane-build-manifest.json'
    manifest = json.loads(path.read_text())
    manifest.update(sourceCommit=commit, sourceDirty=bool(dirty))
    path.write_text(json.dumps(manifest, indent=4) + '\n')
    return manifest


def make_tarball(build_dir, out_dir, commit):
    """out_dir/crane-player-<commit12>-linux-x86_64.tar.zst (zstd when found, else .tar.gz) holding
    build_dir as CRANE-Worker/, and its .sha256 (sha256sum format). Returns the tarball's path."""
    build_dir, out_dir = Path(build_dir), Path(out_dir)
    zstd = shutil.which('zstd')
    out = out_dir / f'crane-player-{commit[:12]}-linux-x86_64.tar.{"zst" if zstd else "gz"}'
    tmp = out.with_name(out.name + '.part')

    def keep(info):
        return None if LEAVE_OUT.search(info.name) else info

    if zstd:
        with open(tmp, 'wb') as sink:
            proc = subprocess.Popen([zstd, '-q', '-T0', '-19', '-c'], stdin=subprocess.PIPE, stdout=sink)
            with tarfile.open(fileobj=proc.stdin, mode='w|') as tar:
                tar.add(build_dir, arcname='CRANE-Worker', filter=keep)
            proc.stdin.close()
            if proc.wait():
                raise RuntimeError(f'zstd failed ({proc.returncode})')
    else:
        with tarfile.open(tmp, 'w:gz') as tar:
            tar.add(build_dir, arcname='CRANE-Worker', filter=keep)
    tmp.replace(out)
    digest = hashlib.sha256()
    with open(out, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    out.with_name(out.name + '.sha256').write_text(f'{digest.hexdigest()}  {out.name}\n')
    return out


def main(args):
    builds = Path(args.builds).resolve() if args.builds else paths.BUILDS
    build_dir = builds / paths.PLAYER_BUILD.name
    commit, dirty = source_state()
    if not args.skip_unity:
        editor = Path(args.editor) if args.editor else editor_path()
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        log = builds / 'logs' / f'build-player-{stamp}.log'
        cmd = unity_command(editor, log, commit, dirty)
        print('crane build player: ' + ' '.join(cmd))
        if args.dry_run:
            return 0
        if not editor.is_file():
            print(f'crane build player: no Unity editor at {editor} (set UNITY_EDITOR or --editor)', file=sys.stderr)
            return 2
        log.parent.mkdir(parents=True, exist_ok=True)
        code = subprocess.call(cmd)
        if code:
            print(f'crane build player: Unity exited {code}; see {log}', file=sys.stderr)
            return code
    if not (build_dir / 'crane-build-manifest.json').is_file():
        print(f'crane build player: no {build_dir}/crane-build-manifest.json (build first)', file=sys.stderr)
        return 2
    stamp_manifest(build_dir, commit, dirty)
    print(f'crane build player: stamped {build_dir}/crane-build-manifest.json (sourceCommit {commit[:12]}, '
          f'sourceDirty {str(dirty).lower()})')
    if args.tarball:
        out = make_tarball(build_dir, builds, commit)
        print(f'crane build player: {out} ({out.stat().st_size / 1e6:.1f} MB) and {out.name}.sha256')
    return 0


# ---- crane build images, crane build wheel ------------------------------------------------------------------

DOCKERFILES = {'core': 'Dockerfile.core', 'gpu': 'Dockerfile.gpu', 'sim': 'Dockerfile.sim'}


def newest_tarball(builds=paths.BUILDS):
    """The newest crane-player-*.tar.* in builds that has its .sha256, or None."""
    found = sorted((p for p in Path(builds).glob('crane-player-*-linux-x86_64.tar.*')
                    if not p.name.endswith('.sha256') and p.with_name(p.name + '.sha256').is_file()),
                   key=lambda p: p.stat().st_mtime)
    return found[-1] if found else None


def image_commands(kinds, commit, repo='crane', player=None, context=None):
    """[(kind, tag, argv)] of the docker builds; sim builds on the core tag of the same commit and takes the player
    tarball's dir as the `player` build context (context: a dir holding only the tarball)."""
    out = []
    for kind in kinds:
        tag = f'{repo}:{kind}-{commit[:12]}'
        argv = ['docker', 'build', '-f', str(paths.ROOT / DOCKERFILES[kind]), '-t', tag,
                '--label', f'org.opencontainers.image.revision={commit}']
        if kind == 'sim':
            sha = (Path(player).with_name(Path(player).name + '.sha256').read_text().split()[0])
            argv += ['--build-context', f'player={context}', '--build-arg', f'CORE_IMAGE={repo}:core-{commit[:12]}',
                     '--build-arg', f'PLAYER_SHA256={sha}']
        out.append((kind, tag, [*argv, str(paths.ROOT)]))
    return out


def images_main(args):
    from crane import output
    out = args.output
    kinds = list(dict.fromkeys(args.kinds or ['core']))
    if 'sim' in kinds and 'core' not in kinds:
        kinds.insert(0, 'core')                   # sim builds on the core image of this commit
    commit, dirty = source_state()
    player = Path(args.player).expanduser().resolve() if args.player else newest_tarball()
    if 'sim' in kinds and (player is None or not player.with_name(player.name + '.sha256').is_file()):
        return out.usage('build images', 'sim needs a player tarball with its .sha256',
                         'crane build player --tarball, or crane build images sim --player PATH')
    if not shutil.which('docker') and not out.dry_run:
        return out.usage('build images', 'no docker on PATH', 'install docker')
    scratch = None
    if 'sim' in kinds:                            # the player context: a dir with only the tarball (a hard link)
        scratch = paths.HOME / '.cache' / 'crane' / 'build' / f'player-{commit[:12]}'
        if not out.dry_run:
            shutil.rmtree(scratch, ignore_errors=True)
            scratch.mkdir(parents=True)
            try:
                os.link(player, scratch / player.name)
            except OSError:
                shutil.copy2(player, scratch / player.name)
    rows = []
    try:
        for kind, tag, argv in image_commands(kinds, commit, args.repo, player, scratch):
            out.line('+ ' + ' '.join(argv))
            if out.dry_run:
                continue
            code = subprocess.call(argv)
            if code:
                return out.fail('build images', f'docker build of {kind} exited {code}',
                                f'docker build -f {DOCKERFILES[kind]} .   (its log is above)')
            size = subprocess.run(['docker', 'image', 'inspect', tag, '--format', '{{.Size}}'], capture_output=True,
                                  text=True).stdout.strip()
            rows.append((kind, tag, f'{int(size) / 1e6:.0f} MB unpacked' if size.isdigit() else ''))
    finally:
        if scratch:
            shutil.rmtree(scratch, ignore_errors=True)
    if dirty:
        out.line(f'note: the checkout has uncommitted changes; the tags name {commit[:12]}')
    out.table(rows)
    out.next(f'docker run --rm {rows[0][1]} crane doctor' if rows else 'crane build images ' + ' '.join(kinds))
    return output.OK


def wheel_main(args):
    import hashlib
    from crane import output
    out = args.output
    dest = Path(args.out).expanduser().resolve() if args.out else paths.HOME / '.cache' / 'crane' / 'dist'
    uv = shutil.which('uv')
    if uv:
        argv = [uv, 'build', '--quiet', '--out-dir', str(dest), str(paths.ROOT)]
    else:
        argv = [sys.executable, '-m', 'build', '--outdir', str(dest), str(paths.ROOT)]
    out.line('+ ' + ' '.join(argv))
    if out.dry_run:
        return output.OK
    proc = subprocess.run(argv, capture_output=True, text=True)
    if proc.returncode:
        tail = (proc.stderr or proc.stdout).strip().splitlines()[-1:] or [f'exit {proc.returncode}']
        return out.fail('build wheel', tail[0], 'install uv (or pip install build), then crane build wheel')
    from crane import __version__
    files = sorted(p for p in dest.glob(f'crane-{__version__}*') if p.suffix in ('.whl', '.gz'))
    rows = []
    for f in files:
        digest = hashlib.sha256(f.read_bytes()).hexdigest()
        f.with_name(f.name + '.sha256').write_text(f'{digest}  {f.name}\n')
        rows.append({'name': f.name, 'path': str(f), 'bytes': f.stat().st_size, 'sha256': digest})
    if out.doc({'schema': 'crane.build.wheel/1', 'version': __version__, 'files': rows}):
        return output.OK
    out.table([(r['name'], f'{r["bytes"] / 1e6:.1f} MB', r['sha256'][:12]) for r in rows])
    out.next(f'pip install {rows[0]["path"]}' if rows else 'crane build wheel')
    return output.OK
