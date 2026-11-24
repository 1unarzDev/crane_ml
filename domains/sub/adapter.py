"""The sub domain: a public reference mission for a 6-DOF underwater vehicle in a generic pool.

    crane run --domain sub --tier T1 --engine batched --seeds 0:64

The vehicle starts at the course's start depth, then for each gate goes to a point in front of it, through it
at the gate's depth and heading to a point behind it, and finally holds station at the course's finish point. A
gate crossed inside its opening (less the vehicle's half width and half height) is passed, else missed;
the run succeeds when every gate is passed and the hold is done. The plant is the batched Fossen 6-DOF
kernel (engine/kernels/marine6.py) with the profile's thrusters; the mission is domains/sub/mission.py.
Only the batched engine runs it (one compiled episode per env, numba).
"""

from pathlib import Path

from fastsim.controller import DomainAdapter
from fastsim.identity import sha256

MISSION = 'pool_gates/1'
GAINS = ('kp_pos', 'vmax_u', 'vmax_v', 'vmax_w', 'kv', 'kp_att', 'kd_att', 'kp_yaw', 'kd_yaw', 'rmax',
         'turn_rad', 'face_m', 'wp_tol_m', 'hold_tol_m', 'hold_s', 'approach_m', 'exit_m', 'ki_pos', 'imax')


class SubAdapter(DomainAdapter):
    engines = ('batched',)          # crane run refuses --engine fastsim with the fix in the line
    backends = ('numba',)

    def __init__(self, base=None, tier='T1', profile=None, mission=None, **options):
        super().__init__(base, **options)
        if tier != 'T1':
            raise ValueError(f'sub: tier {tier!r}; this domain is T1 (dynamic) only')
        if not profile:
            raise ValueError('sub: adapter.options.profile (a vehicle profile JSON) is required')
        self.tier = tier
        self.profile = Path(base or '.') / Path(profile).expanduser()
        self.mission = dict(mission or {})
        missing = [k for k in GAINS if k not in self.mission]
        if missing:
            raise ValueError(f'sub: adapter.options.mission lacks {", ".join(missing)}')

    def identity(self):
        return {'domain': 'sub', 'tier': self.tier, 'mission': MISSION, 'gains': {k: self.mission[k] for k in GAINS},
                'profile_sha256': sha256(self.profile.read_bytes())}

    def controller(self, ctx):
        raise NotImplementedError('sub: the mission runs on the batched engine only (--engine batched)')

    def run_batched(self, cfg, base, seeds, out, variant, backend, dtype, record, provenance):
        from domains.sub import mission
        return mission.run_seeds(self, cfg, base, seeds, out, variant, backend, dtype, record, provenance)

    def metrics(self, run_dir):
        import json
        p = Path(run_dir) / 'metrics.json'
        return json.loads(p.read_text()).get('sub', {}) if p.is_file() else {}
