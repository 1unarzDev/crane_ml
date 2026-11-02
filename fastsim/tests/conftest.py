import sys
from pathlib import Path

import pytest

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE.parents[1]), str(HERE)]  # the crane root (fastsim) and toy

from fastsim import config  # noqa: E402
import toy  # noqa: E402


@pytest.fixture
def toy_cfg(tmp_path):
    """(config, base, scenario folder) of the toy domain, scenarios under tmp_path."""
    folder = tmp_path / 'scenarios'
    cfg, base = config.load(HERE / 'toy.yaml', {'scenarios': {'dir': str(folder)}})
    return cfg, base, folder


@pytest.fixture
def toy_adapter():
    return toy.ToyAdapter(base=HERE, reach_m=1.5, time_limit_s=240)
