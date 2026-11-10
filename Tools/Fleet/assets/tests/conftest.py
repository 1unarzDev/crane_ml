import os
import sys
import shutil

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.dont_write_bytecode = True
BASE = os.path.expanduser("~/.cache/crane-fleet/assets_v0/_pytest")


def pytest_configure(config):
    # nothing may be written under /tmp: put pytest's temp tree in the cache directory
    if not config.option.basetemp:
        config.option.basetemp = BASE
    config.option.cacheprovider = False if hasattr(config.option, "cacheprovider") else None


@pytest.fixture(scope="session")
def gen_dir(tmp_path_factory):
    import gen_provisional
    d = str(tmp_path_factory.mktemp("assets"))
    gen_provisional.main(["--out", d])
    return d


@pytest.fixture()
def work(gen_dir, tmp_path):
    """A private writable copy of the generated asset set."""
    dst = str(tmp_path / "assets")
    shutil.copytree(gen_dir, dst)
    return dst
