"""The controller library: 16 members of an L16 orthogonal array over six factors.

Every member is the boat's velocity law (``law.BatchedLaw``) behind a path-following guidance with
a speed profile. Factors (chosen to interact with actuator transients and timing, not with steady
speed alone):

* turn_alat  -- lateral acceleration allowed when entering the U-turn: 0.5 (carry speed) / 0.12
* brake      -- deceleration the speed profile assumes before stops: 0.6 (reverse-brake) / 0.2 (coast)
* cruise     -- transit speed: 1.2 / 0.8 m/s
* feedforward -- velocity-law feed-forward on (FF + PI) / off (PI only, kp x 1.5)
* latcomp    -- extrapolate the observed pose by 0.15 s / none
* dock       -- final 4 m: const 0.3 m/s, sqrt profile (a = 0.15), two-stage 0.4 -> 0.1, creep 0.08

Columns: b0..b3 a 2^4 full factorial; turn=b0, brake=b1, cruise=b2, ff=b3, latcomp=b0^b2,
dock=2(b0^b1)+(b2^b3). ``tests/test_library.py`` checks pairwise balance.
"""

import hashlib
import json
from dataclasses import asdict, dataclass

import numpy as np

DOCK = ('const', 'sqrt', 'two_stage', 'creep')


@dataclass(frozen=True)
class ControllerSpec:
    idx: int
    turn_alat: float
    brake: float
    cruise: float
    feedforward: bool
    latcomp: float
    dock: str

    def sha(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()[:12]

    def label(self):
        return (f'c{self.idx:02d}:{"wide" if self.turn_alat > 0.3 else "slow"}-turn/'
                f'{"rev" if self.brake > 0.4 else "coast"}-brake/v{self.cruise:.1f}/'
                f'{"ff" if self.feedforward else "pi"}/{"lc" if self.latcomp else "nolc"}/{self.dock}')


def _library():
    out = []
    for i in range(16):
        b = [(i >> k) & 1 for k in range(4)]
        out.append(ControllerSpec(
            idx=i, turn_alat=(0.5, 0.12)[b[0]], brake=(0.6, 0.2)[b[1]], cruise=(1.2, 0.8)[b[2]],
            feedforward=(True, False)[b[3]], latcomp=(0.15, 0.0)[b[0] ^ b[2]],
            dock=DOCK[2 * (b[0] ^ b[1]) + (b[2] ^ b[3])]))
    return tuple(out)


LIBRARY = _library()


def factor_matrix(lib=LIBRARY):
    """[16, 6] numeric factor levels (for balance checks and effect plots)."""
    return np.array([[c.turn_alat, c.brake, c.cruise, float(c.feedforward), c.latcomp, DOCK.index(c.dock)]
                     for c in lib])
