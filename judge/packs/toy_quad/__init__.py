"""A planar quadrotor: the judge's second domain (a tiny sim, a Source, a Pack, a Replayer).

It exists to prove that a new robot needs no core change, and to give every insight generator a
seeded-fault test: tiers differ by plant (motor lag, drag, wind) and a 'decision fault' variant
skips a gate, so the judge must attribute the first to the plant and the second to a decision.

State (x, z, pitch, vx, vz, wpitch); two motors (left, right) with thrust commands in N. Channels:
pose.xz (x, z, pitch), twist.xz (vx, vz, wpitch, exact), actuator.cmd (left, right). Events:
gate.target (payload why=gate id), gate.passed, crash.
"""

from judge.packs.toy_quad.sim import PARAMS_HIGH, PARAMS_LOW, QuadParams, simulate
from judge.packs.toy_quad.pack import PACK, TASK, QuadReplayer, segments

__all__ = ['PACK', 'PARAMS_HIGH', 'PARAMS_LOW', 'QuadParams', 'QuadReplayer', 'TASK', 'segments', 'simulate']
