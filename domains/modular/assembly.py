"""Assemblies of modules: geometry from an assembly file, physics from a plant profile, packed for the kernel.

    asm = load('mecanum-pair')                       # domains/modular/assemblies/mecanum-pair.json, or a path
    prof = profile.load('example#plated')            # a plant card per plate state (domains/modular/profile.py)
    built = build(asm, prof)                         # wheels [{...}], body {...}, footprint, loads
    p = mecanum.pack(built['wheels'], built['body'], faults)

An assembly file (``crane.modular.assembly/1``) holds module types (size m, wheels in the module frame: kind
mecanum | omni | conventional | caster, x, y, yaw, hand, radius) and modules (id, type, x, y, yaw in the assembly
frame), plus the IMU's module and offset. The profile gives every physical number (friction, slip, motor, masses).
The footprint is each module's rectangle and their convex hull, in the assembly frame.

Normal loads are the rigid assembly's on its wheel springs (k_v per wheel): N_j = k_j (a + b y_j + c x_j) with
total weight and moments balanced; a negative load means it tips (``AssemblyError``).
"""

import json
import math
from pathlib import Path

import numpy as np

from engine.kernels import mecanum as K

HERE = Path(__file__).resolve().parent
ASSEMBLIES = HERE / 'assemblies'
SCHEMA = 'crane.modular.assembly/1'
KINDS = {'mecanum': K.MEC, 'omni': K.MEC, 'conventional': K.CONV, 'caster': K.BALL}
G0 = 9.81
CASTER_R = 0.03


class AssemblyError(ValueError):
    """An assembly that cannot be built; one line naming the fix."""


def load(ref):
    """An assembly document from a path or a name in domains/modular/assemblies."""
    if isinstance(ref, dict):
        return ref
    path = Path(str(ref)).expanduser()
    if not path.suffix:
        path = ASSEMBLIES / f'{ref}.json'
    if not path.is_file():
        known = ', '.join(sorted(p.stem for p in ASSEMBLIES.glob('*.json')))
        raise AssemblyError(f'no assembly {ref!r} ({path} is missing); assemblies: {known}')
    doc = json.loads(path.read_text())
    if doc.get('schema') != SCHEMA:
        raise AssemblyError(f'{path}: schema {doc.get("schema")!r}, want {SCHEMA}')
    return doc


def _rot(th, v):
    c, s = math.cos(th), math.sin(th)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def rect(cx, cy, yaw, lx, ly):
    h = [(lx / 2, ly / 2), (-lx / 2, ly / 2), (-lx / 2, -ly / 2), (lx / 2, -ly / 2)]
    return np.array([np.array([cx, cy]) + _rot(yaw, v) for v in h])


def hull(points):
    pts = sorted(set(map(tuple, np.round(np.asarray(points, float), 12))))
    if len(pts) < 3:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for q in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    for q in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], q) <= 0:
            up.pop()
        up.append(q)
    return np.array(lo[:-1] + up[:-1])


def static_loads(pos, k, mass, com):
    """Normal loads [n] of a rigid body of ``mass`` at ``com`` on vertical springs k [n] at pos [n, 2]."""
    x, y = pos[:, 0], pos[:, 1]
    rows = np.stack([k, k * y, k * x], axis=1)            # N_j = k_j (a + b y_j + c x_j)
    A = np.array([rows.sum(0), (rows * y[:, None]).sum(0), (rows * x[:, None]).sum(0)])
    rhs = mass * G0 * np.array([1.0, com[1], com[0]])
    try:
        a, b, c = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError:
        raise AssemblyError('the wheels are collinear: no vertical equilibrium; add a wheel or a caster off the '
                            'line') from None
    N = k * (a + b * y + c * x)
    if (N < -1e-9).any():
        raise AssemblyError(f'the assembly tips: wheel loads {np.round(N, 2).tolist()} N; move the COM (payload) '
                            'inside the wheel polygon')
    return np.maximum(N, 0.0)


def build(asm, prof, payload=None):
    """Kernel inputs of an assembly under a profile (a resolved dict from domains.modular.profile.resolve):
    {wheels: [...], body: {...}, footprint: {rects [m, 4, 2], hull [h, 2]}, keys: [(module, index)]}."""
    asm = load(asm)
    types, mods = asm.get('types') or {}, asm.get('modules') or []
    if not mods:
        raise AssemblyError(f'assembly {asm.get("name")!r} has no modules')
    wp, fl, sc, md = prof['wheel'], prof['floor'], prof['scale'], prof['module']
    masses, coms, Is, rects, wheels, keys, kv = [], [], [], [], [], [], []
    for m in mods:
        t = types.get(m['type'])
        if t is None:
            raise AssemblyError(f'module {m["id"]}: no type {m["type"]!r} in the assembly ({", ".join(types)})')
        lx, ly = t['size']
        mm = (md['mass_kg'].get(m['type']) if isinstance(md['mass_kg'], dict) else md['mass_kg'])
        if mm is None:
            raise AssemblyError(f'the profile has no module.mass_kg for type {m["type"]!r}; add it to the profile')
        mm = float(mm) * sc['mass']
        yaw = float(m.get('yaw', 0.0))
        c = np.array([m['x'], m['y']], float) + _rot(yaw, md.get('com', [0.0, 0.0]))
        masses.append(mm)
        coms.append(c)
        Is.append(mm * (lx * lx + ly * ly) / 12.0 * md['inertia_f'])
        rects.append(rect(m['x'], m['y'], yaw, lx, ly))
        for i, w in enumerate(t['wheels']):
            kind = w['kind']
            if kind not in KINDS:
                raise AssemblyError(f'module type {m["type"]}: wheel kind {kind!r}; kinds: {", ".join(KINDS)}')
            pos = np.array([m['x'], m['y']], float) + _rot(yaw, (w['x'], w['y']))
            yw = float(w.get('yaw', 0.0))
            e = _rot(yaw, (math.cos(yw), math.sin(yw)))
            rec = {'px': pos[0], 'py': pos[1], 'ex': e[0], 'ey': e[1], 'kind': KINDS[kind]}
            if kind == 'caster':
                rec.update(tx=e[0], ty=e[1], r=CASTER_R, r_fw=CASTER_R, mu_t=fl['mu'], mu_f=fl['mu'] * fl['kappa_lat'],
                           s_x0=1.0, alpha0=1.0, c_rr=prof['caster']['c_rr'], K=1.0, tau_stall=1.0, w_nl=1.0, b=0.0,
                           tau_c=0.0, tau_bd=0.0, phimax=0.0)
                kv.append(prof['caster']['k_v'])
            else:
                if kind == 'mecanum':
                    a = yw + int(w.get('hand', 1)) * math.pi / 4
                    d = _rot(math.radians(wp['roller_dev_deg']), (math.cos(a), math.sin(a)))
                    tt = _rot(yaw, d)
                    tt = tt / np.linalg.norm(tt)
                    mu_t, mu_f = fl['mu'] * wp['roller_mu_f'], wp['mu_rb']
                elif kind == 'omni':
                    tt, mu_t, mu_f = e, fl['mu'] * wp['roller_mu_f'], wp['mu_rb']
                else:
                    tt, mu_t, mu_f = e, fl['mu'], fl['mu'] * fl['kappa_lat']
                r0 = float(w.get('radius', t.get('wheel_radius', 0.0)))
                if r0 <= 0:
                    raise AssemblyError(f'module type {m["type"]} wheel {i}: no radius (give radius or wheel_radius)')
                rec.update(tx=tt[0], ty=tt[1], r=r0 * sc['radius'], r_fw=r0, mu_t=mu_t, mu_f=mu_f, s_x0=wp['s_x0'],
                           alpha0=wp['alpha0'], c_rr=wp['c_rr'], K=wp['K'] * sc['kp'],
                           tau_stall=wp['tau_stall_Nm'] * sc['stall'], w_nl=wp['w_nl_rad_s'] * sc['noload'],
                           b=wp['b_visc'], tau_c=wp['tau_c'], tau_bd=wp['tau_bd'], phimax=wp['phi_max_rad_s'])
                kv.append(wp['k_v'])
            wheels.append(rec)
            keys.append((m['id'], i))
    mass = float(sum(masses))
    mc = sum(mi * ci for mi, ci in zip(masses, coms))
    I_o = float(sum(Ii + mi * float(ci @ ci) for Ii, mi, ci in zip(Is, masses, coms)))
    if payload and float(payload.get('mass_kg', 0.0)) > 0:
        pm, pc = float(payload['mass_kg']), np.array(payload.get('at', [0.0, 0.0]), float)
        plx, ply = payload.get('size', [0.0, 0.0])
        mass, mc = mass + pm, mc + pm * pc
        I_o += pm * ((plx ** 2 + ply ** 2) / 12 + float(pc @ pc))
    com = mc / mass
    I_o *= sc['izz']
    pos = np.array([[w['px'], w['py']] for w in wheels])
    N = static_loads(pos, np.array(kv, float), mass, com)
    for w, n in zip(wheels, N):
        w['N'] = float(n)
    imu = asm.get('imu') or {}
    at = np.array([imu.get('x', 0.0), imu.get('y', 0.0)], float)
    if imu.get('module'):
        m = next((q for q in mods if q['id'] == imu['module']), None)
        if m is None:
            raise AssemblyError(f'imu.module {imu["module"]!r} is not a module of the assembly')
        at = np.array([m['x'], m['y']], float) + _rot(float(m.get('yaw', 0.0)), at)
    body = {'mass': mass, 'xc': float(com[0]), 'yc': float(com[1]), 'inertia': I_o, 'imu_x': float(at[0]),
            'imu_y': float(at[1]), 'cpr': float(wp['cpr'])}
    rects = np.array(rects)
    return {'wheels': wheels, 'body': body, 'keys': keys, 'name': asm.get('name'),
            'footprint': {'rects': rects, 'hull': hull(rects.reshape(-1, 2))}}


def footprint_world(poly, pose):
    """Footprint corners [E, F, 2] in the world from body-frame corners [E, F, 2] (or [F, 2]) and poses [E, 3]."""
    pose = np.atleast_2d(pose)
    poly = np.broadcast_to(poly, (pose.shape[0],) + np.shape(poly)[-2:])
    c, s = np.cos(pose[:, 2])[:, None], np.sin(pose[:, 2])[:, None]
    return np.stack([pose[:, 0:1] + c * poly[..., 0] - s * poly[..., 1],
                     pose[:, 1:2] + s * poly[..., 0] + c * poly[..., 1]], axis=-1)
