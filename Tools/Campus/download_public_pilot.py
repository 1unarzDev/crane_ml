#!/usr/bin/env python3
"""Download and verify a pinned public pilot into an ignored local result store."""
import argparse
import json
from pathlib import Path
import tempfile
from urllib.request import urlopen

from heldout_calibration import sha256_file


def download(source, directory):
    directory.mkdir(parents=True, exist_ok=True)
    artifacts = []
    for artifact in source['artifacts']:
        name = artifact['name']
        if Path(name).name != name:
            raise ValueError('Artifact name must be a filename')
        target = directory/name
        expected = artifact['sha256']
        if target.exists():
            if sha256_file(target) != expected:
                raise ValueError(f'Existing {target} has a different hash; refusing to overwrite')
        else:
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=directory, delete=False) as stream:
                    temporary = Path(stream.name)
                    with urlopen(artifact['url'], timeout=30) as response:
                        size = 0
                        while block := response.read(1024*1024):
                            size += len(block)
                            if size > 80*1024*1024:
                                raise ValueError('Pinned pilot artifact exceeded 80 MiB limit')
                            stream.write(block)
                if sha256_file(temporary) != expected:
                    raise ValueError(f'{name}: downloaded source does not match pinned SHA-256')
                if 'bytes' in artifact and temporary.stat().st_size != artifact['bytes']:
                    raise ValueError(f'{name}: size differs from pinned source')
                temporary.replace(target)
            finally:
                if temporary is not None and temporary.exists():
                    temporary.unlink()
        artifacts.append(dict(path=str(target.resolve()), sha256=expected, bytes=target.stat().st_size))
    return dict(schema='crane-public-pilot-download-v1', id=source['id'],
                sourcePage=source['sourcePage'], license=source['license'],
                role=source['role'], artifacts=artifacts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='lilocbench-dynamics-0')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    catalog = json.loads(Path(__file__).with_name('public_pilot_sources.json').read_text())
    source = next((s for s in catalog['sources'] if s['id']==args.source), None)
    if source is None:
        parser.error('Unknown pinned pilot source')
    report = download(source, args.output)
    (args.output/'download-manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
