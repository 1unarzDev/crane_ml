"""Off-site bundles against rclone's ``local`` backend standing in for the drive (a remote defined by environment
variables, so no rclone.conf is read or written)."""
import json
import shutil
import threading
import urllib.error
import urllib.request

import pytest

from crane_archive import daemon as D
from crane_archive.offsite import Offsite, OffsiteError
from crane_archive.server import make_server
from crane_archive.store import Store

from test_archive import cfg_for, noise, text_blob

pytestmark = pytest.mark.skipif(shutil.which('rclone') is None, reason='rclone not installed')


@pytest.fixture
def remote(tmp_path, monkeypatch):
    monkeypatch.setenv('RCLONE_CONFIG_FAKEDRIVE_TYPE', 'local')
    monkeypatch.setenv('RCLONE_CONFIG', str(tmp_path / 'none.conf'))
    d = tmp_path / 'remote'
    d.mkdir()
    return d


@pytest.fixture
def store(tmp_path):
    return Store(tmp_path / 'arch', node='t', create=True)


def off_for(store, remote, **kw):
    cfg = cfg_for(store, offsite={'remote': 'fakedrive:', 'root': f'{remote}/crane-archive', 'bundle_bytes': '300K', 'bandwidth': '50M', **kw})
    return Offsite(cfg, store, log=lambda m: None)


def fill(store):
    items = {'a': (text_blob(5_000), 'summary'), 'b': (text_blob(400_000, 3) + noise(100_000, 4), 'raw'),
             'v': (noise(300_000), 'render_final'), 'c': (noise(1000, 9), 'ctraj'), 'h': (noise(250_000, 5), 'crane')}
    return {k: store.put_bytes(d, kind=kind, name=f'{k}.bin') for k, (d, kind) in items.items()}, {k: v[0] for k, v in items.items()}


def test_push_bundles_verifies_records_and_is_idempotent(store, remote):
    objs, _ = fill(store)
    off = off_for(store, remote)
    assert len(off.pending()) == 5 and off.status()['pending_objects'] == 5
    r = off.push()
    assert r['error'] is None and r['objects'] == 5 and r['bundles'] >= 2 and r['failed'] == 0
    names = sorted(p.name for p in (remote / 'crane-archive' / 'bundles').iterdir())
    assert len(names) == r['bundles'] and all(n.endswith(('.tar.zst', '.tar.xz')) for n in names)
    assert len(list((remote / 'crane-archive' / 'manifests').iterdir())) == r['bundles']
    assert off.pending() == [] and off.status()['behind'] is False
    assert {row[0] for row in store.db.execute('SELECT sha256 FROM copies')} == {o.sha256 for o in objs.values()}
    assert off.push()['bundles'] == 0
    assert list((store.root / 'tmp').iterdir()) == []


def test_restore_after_local_loss_and_via_the_origin(store, remote):
    objs, data = fill(store)
    off = off_for(store, remote)
    off.push()
    for k in ('b', 'c'):
        store.remove(objs[k], 'test')
    assert store.get(objs['b'].sha256) is None
    o = off.restore(objs['b'].sha256)
    assert store.read_bytes(o.sha256) == data['b']
    # the origin serves a missing object by restoring it
    srv = make_server(store, '127.0.0.1:0', restore=off.restore)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        url = f'http://127.0.0.1:{srv.server_address[1]}/o/'
        with urllib.request.urlopen(url + objs['c'].sha256) as r:
            assert r.read() == data['c']
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(url + '0' * 64)
        assert e.value.code == 404
    finally:
        srv.shutdown()
    assert off.restore('0' * 64) is None


def test_a_corrupt_remote_bundle_is_refused_and_a_failed_upload_is_not_recorded(store, remote, tmp_path):
    objs, _ = fill(store)
    off = off_for(store, remote, bundle_bytes='10G')
    off.push()
    bundle = next((remote / 'crane-archive' / 'bundles').iterdir())
    bundle.write_bytes(bundle.read_bytes()[:-1] + b'\0')
    store.remove(objs['a'], 'test')
    with pytest.raises(OffsiteError):
        off.restore(objs['a'].sha256)
    assert store.get(objs['a'].sha256) is None
    # a remote that cannot be written: nothing is recorded, the next push retries
    store2 = Store(tmp_path / 'arch2', node='t', create=True)
    store2.put_bytes(noise(2000, 1), kind='raw')
    (remote / 'ro').write_text('a file where the root should be a directory')
    bad = off_for(store2, remote, root=f'{remote}/ro/x')
    r = bad.push()
    assert r['error'] and r['bundles'] == 0 and store2.db.execute('SELECT COUNT(*) FROM copies').fetchone()[0] == 0


def test_reindex_from_manifests_and_mirror_policy(store, remote, tmp_path):
    objs, _ = fill(store)
    off = off_for(store, remote)
    old = __import__('time').time() - 23 * 3600
    store.db.execute('UPDATE objects SET first_seen = ?', (old,))
    assert off.due() and not off.status()['behind']
    store.db.execute('UPDATE objects SET first_seen = ?', (old - 3 * 3600,))
    assert off.status()['behind']
    d = D.Daemon(cfg_for(store, offsite=off.o), store, None, log=lambda m: None)
    assert d.health()['offsite']['pending_objects'] == 5
    off.push()
    fresh = Store(tmp_path / 'arch3', node='t', create=True)
    again = Offsite(cfg_for(fresh, offsite=off.o), fresh, log=lambda m: None)
    assert again.reindex() == off.status()['bundles']
    assert len(again.locations(objs['h'].sha256)) == 1
    assert again.restore(objs['h'].sha256).sha256 == objs['h'].sha256
