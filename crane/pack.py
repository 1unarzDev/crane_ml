"""Scenario packs: a domain's scenario files for seeds A:B in one compressed archive, to carry to a host without
the generator (a rented box).

    crane env pack --domain boat --seeds 0:64 --out boat-courses.tar.zst     # on a machine with the generator
    crane env unpack boat-courses.tar.zst /tmp/courses                       # verifies every sha256
    CRANE_SCENARIOS=/tmp/courses crane run --domain boat --seeds 0:64       # found after the config's own dir

A pack is a tar of ``manifest.json`` (domain, generator identity and version, seeds, sha256 per file) and the
scenario files. It is zstd compressed when the zstd python module or binary is here, else gzip (``.tar.gz``).
A study spec lists packs under ``inputs:``; ``crane study run-cell`` unpacks them (``unpack_inputs``).
"""

import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

from crane import output, select
from crane.output import Output

SCHEMA = 'crane.pack/1'
MANIFEST = 'manifest.json'
PACKS = Path.home() / '.cache' / 'crane-packs'
INPUT_DIRS = ('CRANE_INPUTS_DIR', '/workspace/in')     # where a rented box finds the packs the job pushed
ZSTD, GZIP = b'\x28\xb5\x2f\xfd', b'\x1f\x8b'


class PackError(ValueError):
    """A pack that cannot be made, read or trusted; the message names the fix."""


def sha_of(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def content_sha(files):
    """The pack's identity: the sha256 of its (name, file sha256) pairs, whatever the compression."""
    return hashlib.sha256(json.dumps(sorted(files.items())).encode()).hexdigest()


# ---- compression ----------------------------------------------------------------------------------------------

def _zstd_module():
    try:
        from compression import zstd
        return zstd
    except ImportError:
        return None


def compressor():
    """'zstd' when the python module or the binary is here, else 'gzip'."""
    if _zstd_module() or shutil.which('zstd'):
        return 'zstd'
    try:
        import zstandard  # noqa: F401
        return 'zstd'
    except ImportError:
        return 'gzip'


def _compress(tar_path, dest, kind):
    if kind == 'gzip':
        with open(tar_path, 'rb') as src, open(dest, 'wb') as raw, \
                gzip.GzipFile(fileobj=raw, mode='wb', mtime=0) as gz:
            shutil.copyfileobj(src, gz)
    elif _zstd_module():
        with open(tar_path, 'rb') as src, _zstd_module().open(dest, 'wb') as z:
            shutil.copyfileobj(src, z)
    elif shutil.which('zstd'):
        subprocess.run(['zstd', '-q', '-f', '-o', str(dest), str(tar_path)], check=True)
    else:
        import zstandard
        with open(tar_path, 'rb') as src, open(dest, 'wb') as raw:
            zstandard.ZstdCompressor().copy_stream(src, raw)


def _decompress(pack, tar_path):
    with open(pack, 'rb') as f:
        magic = f.read(4)
    if magic[:2] == GZIP:
        with gzip.open(pack, 'rb') as src, open(tar_path, 'wb') as dst:
            shutil.copyfileobj(src, dst)
    elif magic == ZSTD:
        if _zstd_module():
            with _zstd_module().open(pack, 'rb') as src, open(tar_path, 'wb') as dst:
                shutil.copyfileobj(src, dst)
        elif shutil.which('zstd'):
            subprocess.run(['zstd', '-q', '-d', '-f', '-o', str(tar_path), str(pack)], check=True)
        else:
            try:
                import zstandard
            except ImportError:
                raise PackError(f'{pack} is zstd and this host has no zstd; fix: apt install zstd, or pip install '
                                'zstandard, or re-pack with gzip on a host without zstd') from None
            with open(pack, 'rb') as src, open(tar_path, 'wb') as dst:
                zstandard.ZstdDecompressor().copy_stream(src, dst)
    else:
        raise PackError(f'{pack} is not a crane pack (neither zstd nor gzip); fix: crane env pack --domain D --seeds A:B')


def _flat(name):
    if name != Path(name).name or name in ('', '.', '..'):
        raise PackError(f'pack file name {name!r} is not a plain file name')
    return name


# ---- make and open ----------------------------------------------------------------------------------------------

def make(files, manifest, out):
    """Write ``files`` ({name: path}) and the manifest into the archive ``out`` (the suffix follows the compression).
    Returns (the written path, the manifest dict)."""
    kind = compressor()
    manifest = {**manifest, 'schema': SCHEMA, 'compression': kind,
                'files': {n: sha_of(p) for n, p in sorted(files.items())}}
    manifest['sha256'] = content_sha(manifest['files'])
    out = Path(out).expanduser()
    if kind == 'gzip' and out.name.endswith('.zst'):
        out = out.with_name(out.name[:-4] + '.gz')
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tar_path = Path(tmp) / 'pack.tar'
        with tarfile.open(tar_path, 'w', format=tarfile.PAX_FORMAT) as tar:
            data = (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode()
            for name, body in [(MANIFEST, data)] + [(n, None) for n in sorted(files)]:
                info = tarfile.TarInfo(name)
                info.mtime, info.mode = 0, 0o644
                if body is None:
                    info.size = Path(files[name]).stat().st_size
                    with open(files[name], 'rb') as f:
                        tar.addfile(info, f)
                else:
                    info.size = len(body)
                    tar.addfile(info, io.BytesIO(body))
        part = out.with_name(out.name + '.part')
        _compress(tar_path, part, kind)
        os.replace(part, out)
    return out, manifest


def unpack(pack, dest):
    """Extract ``pack`` into DEST, every file checked against the manifest's sha256. Returns the manifest.
    PackError when a file is missing, altered or not listed."""
    pack, dest = Path(pack).expanduser(), Path(dest).expanduser()
    if not pack.is_file():
        raise PackError(f'no pack at {pack}; fix: crane env pack --domain D --seeds A:B --out {pack}')
    with tempfile.TemporaryDirectory() as tmp:
        tar_path = Path(tmp) / 'pack.tar'
        _decompress(pack, tar_path)
        stage = Path(tmp) / 'files'
        stage.mkdir()
        with tarfile.open(tar_path) as tar:
            for member in tar.getmembers():
                if not member.isfile():
                    raise PackError(f'{pack}: {member.name} is not a plain file')
                _flat(member.name)
                tar.extract(member, stage, filter='data')
        try:
            manifest = json.loads((stage / MANIFEST).read_text())
        except (OSError, ValueError):
            raise PackError(f'{pack} has no readable {MANIFEST}') from None
        listed = manifest.get('files') or {}
        for name, sha in listed.items():
            _flat(name)
            if not (stage / name).is_file():
                raise PackError(f'{pack}: {name} is listed in the manifest but missing')
            if sha_of(stage / name) != sha:
                raise PackError(f'{pack}: {name} does not match its sha256; fix: copy the pack again')
        extra = sorted({p.name for p in stage.iterdir()} - set(listed) - {MANIFEST})
        if extra:
            raise PackError(f'{pack}: {extra[0]} is not in the manifest')
        dest.mkdir(parents=True, exist_ok=True)
        for name in [*listed, MANIFEST]:
            shutil.copyfile(stage / name, dest / name)
    return manifest


# ---- a study's inputs on a box ----------------------------------------------------------------------------------

def find_input(path):
    """The pack a job names: its basename in $CRANE_INPUTS_DIR, then /workspace/in, then the path itself."""
    path = Path(path).expanduser()
    for d in INPUT_DIRS:
        d = os.environ.get(d) if not d.startswith('/') else d
        if d and (Path(d) / path.name).is_file():
            return Path(d) / path.name
    if path.is_file():
        return path
    raise PackError(f'input {path.name} is in no input dir ($CRANE_INPUTS_DIR, /workspace/in) and not at {path}; '
                    'fix: crane env pack ... on the machine with the generator, and list it under inputs: in the spec')


def unpack_inputs(inputs, out):
    """Unpack each pack of a study's ``inputs`` into OUT/inputs/<pack sha> (verified) and put those dirs on
    CRANE_SCENARIOS, where scenario sources look after their own dir. Returns the dirs."""
    dirs = []
    for pack in inputs:
        found = find_input(pack)
        dest = Path(out) / 'inputs' / sha_of(found)[:12]
        unpack(found, dest)
        dirs.append(str(dest))
    if dirs:
        have = [d for d in os.environ.get('CRANE_SCENARIOS', '').split(os.pathsep) if d]
        os.environ['CRANE_SCENARIOS'] = os.pathsep.join([*dirs, *[d for d in have if d not in dirs]])
    return dirs


# ---- the commands -------------------------------------------------------------------------------------------------

def pack_main(args):
    out = getattr(args, 'output', None) or Output.of(args)
    if not args.domain or not args.seeds:
        return out.usage('env pack', '--domain and --seeds are required',
                         'crane env pack --domain boat --seeds 0:64 --out boat-courses.tar.zst')
    from crane.run import tier_config
    from fastsim import config as fsconfig
    from fastsim.scenario import ScenarioMissing, ScenarioSource
    try:
        seeds = select.parse_seeds(args.seeds, args.domain)
        _, config, merged = tier_config(args.domain, None)
    except (select.SelectError, LookupError) as error:
        return out.usage('env pack', str(error).strip('"\''), 'crane env pack --domain DOMAIN --seeds A:B')
    cfg, base = fsconfig.load(config, merged or None)
    source = ScenarioSource(**{k: v for k, v in cfg['scenarios'].items() if k != 'pool'}, base=base)
    try:
        made = source.generate(seeds)
    except ScenarioMissing as error:
        print(str(error), file=sys.stderr)
        return output.USAGE
    except subprocess.CalledProcessError as error:
        return out.fail('env pack', 'the scenario generator failed: '
                        + (error.stderr or b'').decode(errors='replace').strip()[-300:])
    files = {source.path(s).name: source.path(s) for s in seeds}
    identity = source.load(seeds[0]).identity or {}
    manifest = {'domain': args.domain, 'kind': source.dir.name, 'seeds': args.seeds,
                'generator': {'identity': identity.get('generator', identity), 'version': identity.get('version')}}
    if args.out:
        dest = Path(args.out)
    else:
        sha = content_sha({n: sha_of(p) for n, p in sorted(files.items())})[:12]
        dest = PACKS / f"{args.domain}-{manifest['kind']}-{sha}.tar.{'zst' if compressor() == 'zstd' else 'gz'}"
    written, manifest = make(files, manifest, dest)
    doc = {'schema': 'crane.env.pack/1', 'pack': str(written), 'domain': args.domain, 'seeds': args.seeds,
           'files': len(files), 'generated': len(made), 'bytes': written.stat().st_size,
           'compression': manifest['compression'], 'sha256': manifest['sha256']}
    if not out.doc(doc):
        out.say(f"crane env pack: {written} ({len(files)} scenarios, {written.stat().st_size // 1024} KB, "
                f"{manifest['compression']}, {len(made)} generated here)")
        out.next(f'crane env unpack {written} DIR   # then CRANE_SCENARIOS=DIR on the host that lacks the generator')
    return output.OK


def unpack_main(args):
    out = getattr(args, 'output', None) or Output.of(args)
    try:
        manifest = unpack(args.pack, args.dir)
    except PackError as error:
        return out.fail('env unpack', str(error), None, output.FAILED)
    except subprocess.CalledProcessError as error:
        return out.fail('env unpack', f'{error.cmd[0]} failed ({error.returncode})')
    doc = {'schema': 'crane.env.unpack/1', 'dir': str(Path(args.dir).expanduser()), 'files': len(manifest['files']),
           'domain': manifest.get('domain'), 'seeds': manifest.get('seeds')}
    if not out.doc(doc):
        out.say(f"crane env unpack: {len(manifest['files'])} scenarios verified into {doc['dir']}")
        out.next(f'CRANE_SCENARIOS={doc["dir"]} crane run --domain {manifest.get("domain") or "DOMAIN"} --seeds '
                 f'{manifest.get("seeds") or "A:B"}')
    return output.OK
