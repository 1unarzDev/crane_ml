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
