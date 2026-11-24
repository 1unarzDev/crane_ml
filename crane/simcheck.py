"""The old import path of `crane sim check`: this module is crane.sim.check (one module, two names)."""

import sys

from crane.sim import check

sys.modules[__name__] = check
