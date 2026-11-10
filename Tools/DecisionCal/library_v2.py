"""Controller library v2: 14 members that trade speed against robustness to lag and delay along graded axes.

Why: in library v1 (an L16 array of binary switches) the oracle on nearly every hidden boat was c08 or
c00 (identical except feed-forward) and the uncalibrated twin always picked c08, so "which calibration
changes the decision" collapsed to one binary choice with one fragile member (PI-only, 1.5x gain, which
fails under delay). v2 replaces the switches with graded levels. Members are hand-placed along an
aggressive -> conservative spectrum, deliberately NOT orthogonal, and cross the axes so that no member
is best everywhere:

* kp_scale, ki_scale in {0.5, 1.0, 1.5} -- velocity-law feedback gains (``mission.run`` multiplies the
  v1 base kp (0.8, 0.8, 0.6) and ki (0.6, 0.6, 0.4)). High gain is fast and crisp on a lag-free boat
  and oscillates under dead time / odometry latency; low gain is robust but tracks sluggishly and,
  with feed-forward mismatch, drifts. These are velocity-law gains (effort per m/s), unrelated to any
  force gain.
* cruise in {0.8 .. 1.3} m/s and turn_alat in {0.12, 0.25, 0.5} m/s^2 -- transit speed and the lateral
  acceleration allowed in the U-turn (turn-entry aggressiveness).
* brake in {0.2, 0.4, 0.6} m/s^2 -- deceleration the speed profile assumes before the slip. Optimistic
  braking overshoots when thrust reversal is slow (long spin-down / reversal hold).
* dock in {const, sqrt, two_stage, creep} -- final-4 m schedule (as v1).
* latcomp in {0, 0.1, 0.2} s -- pose extrapolation horizon; helps against latency, adds noise and
  overshoot on a crisp boat.
* feedforward: ON for 12 members (the realistic default), OFF (PI only) for two, at moderate gain.

Development protocol: the design was iterated at most three times against 24 development hidden boats
(``hidden.sample_hidden(6, 4242, stratum)``, K=2 repeats, common random numbers). Seed 4242 is a
development seed and must never be used for a scored run. Mission, scoring and plants are unchanged.
"""

from dataclasses import dataclass

import numpy as np

from library import DOCK as DOCK_V1, ControllerSpec

DOCK = DOCK_V1 + ('sqrt@0.08', 'sqrt@0.3', 'sqrt@0.6')   # 'sqrt@a': sqrt profile assuming deceleration a

__all__ = ['ControllerSpecV2', 'LIBRARY_V2', 'factor_matrix_v2']


@dataclass(frozen=True)
class ControllerSpecV2(ControllerSpec):
    kp_scale: float = 1.0
    ki_scale: float = 1.0

    def label(self):
        return (f'v2c{self.idx:02d}:kp{self.kp_scale:g}/ki{self.ki_scale:g}/v{self.cruise:g}/'
                f'turn{self.turn_alat:g}/brk{self.brake:g}/{self.dock}/lc{self.latcomp:g}/'
                f'{"ff" if self.feedforward else "pi"}')


# (kp, ki, cruise, turn_alat, brake, dock, latcomp, feedforward)
_TABLE = [
    (1.5, 1.0, 1.5, 0.50, 2.0, 'sqrt@0.6', 0.0, True),
    (1.5, 1.0, 1.5, 0.50, 1.0, 'sqrt@0.6', 0.1, True),
    (1.0, 1.0, 1.3, 0.50, 0.8, 'sqrt@0.3', 0.1, True),
    (1.0, 1.0, 1.2, 0.50, 0.6, 'sqrt',      0.1, True),
    (1.5, 1.5, 1.2, 0.25, 0.6, 'const',     0.2, True),
    (1.0, 1.0, 1.0, 0.25, 0.4, 'two_stage', 0.1, True),
    (1.0, 0.5, 1.0, 0.25, 0.4, 'sqrt',      0.2, True),
    (1.0, 0.5, 1.3, 0.50, 0.6, 'sqrt@0.3', 0.1, True),
    (1.0, 1.0, 1.0, 0.25, 0.4, 'sqrt',      0.1, False),
    (0.5, 0.5, 0.8, 0.12, 0.2, 'sqrt@0.08', 0.2, True),
    (1.0, 1.0, 0.8, 0.12, 0.2, 'two_stage', 0.0, True),
    (1.5, 0.5, 1.1, 0.25, 0.3, 'sqrt',      0.2, True),
    (1.5, 1.0, 0.8, 0.12, 0.2, 'two_stage', 0.1, False),
    (1.0, 0.5, 1.0, 0.12, 0.4, 'two_stage', 0.2, True),
    (1.5, 0.5, 1.2, 0.25, 0.4, 'sqrt',      0.2, True),
    (0.5, 1.0, 1.2, 0.50, 0.4, 'const',     0.2, True),
]


def _library():
    return tuple(ControllerSpecV2(idx=i, turn_alat=t, brake=b, cruise=v, feedforward=ff, latcomp=lc, dock=d,
                                  kp_scale=kp, ki_scale=ki)
                 for i, (kp, ki, v, t, b, d, lc, ff) in enumerate(_TABLE))


LIBRARY_V2 = _library()


def factor_matrix_v2(lib=LIBRARY_V2):
    """[C, 8] numeric levels: kp, ki, cruise, turn_alat, brake, dock index, latcomp, feedforward."""
    return np.array([[c.kp_scale, c.ki_scale, c.cruise, c.turn_alat, c.brake, DOCK.index(c.dock), c.latcomp,
                      float(c.feedforward)] for c in lib])
