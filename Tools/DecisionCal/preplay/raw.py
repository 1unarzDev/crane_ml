"""PREPLAY raw reader: per-trial raw (un-gridded) streams from the 10-18 bags, read-only.
Times are bag log_time (recorder clock, epoch s) minus the pulse-onset log time t_on (so t=0 = onset message)."""
import sys, json, pickle, os
sys.path.insert(0, '/home/lunarz/worktrees/decisioncal')
import numpy as np
from pathlib import Path
from plantcard.marine import mcap_lite as M
from plantcard.marine import fieldtrials as F
D = F.DEFAULT_DIR
CACHE = Path(__file__).with_name('raw_cache.pkl')
TOP = ['/gps/fix', '/imu/raw', '/odom/local', '/thruster_pwm_node/test_pulses']


def read_raw(stem):
    jf = D / f'{stem}.json'
    doc = json.loads(jf.read_bytes())
    d = M.read(jf.with_suffix('.files') / 'bag' / 'bag_0.mcap', TOP)
    pw = [(t, M.uint16_array(x)) for t, x in d['/thruster_pwm_node/test_pulses']]
    on = [i for i, (_, v) in enumerate(pw) if any(a != 1500 for a in v)]
    t_on = pw[on[0]][0]
    after = [t for t, v in pw[on[-1]:] if all(a == 1500 for a in v)]
    t_off = after[0] if after else pw[on[-1]][0] + 0.05
    # last neutral message before onset (bounds the onset time: publisher period)
    prev = [t for t, v in pw[:on[0]]]
    gps = [(t, M.navsatfix(x)) for t, x in d['/gps/fix']]
    gps = [(t, g) for t, g in gps if g['status'] >= 0]
    lat0, lon0 = gps[0][1]['lat'], gps[0][1]['lon']
    P = np.array([[(g['lon'] - lon0) * np.pi / 180 * F.EARTH_R * np.cos(np.radians(lat0)),
                   (g['lat'] - lat0) * np.pi / 180 * F.EARTH_R] for _, g in gps])
    imu = [M.imu(x) for _, x in d['/imu/raw']]
    odo = [M.odometry(x) for _, x in d['/odom/local']]
    return dict(stem=stem, doc_settings=doc['settings'], t_on=t_on, t_off=t_off, t_prev=prev[-1] if prev else np.nan,
                pwm=pw[on[0]][1], pwm_t=np.array([t for t, _ in pw]) - t_on,
                tg=np.array([t for t, _ in gps]) - t_on, P=P, gcov=np.array([g['cov'][0] for _, g in gps]),
                ti=np.array([t for t, _ in d['/imu/raw']]) - t_on, wz=np.array([i['w'][2] for i in imu]),
                a=np.array([i['a'] for i in imu]), ti_stamp=np.array([i['stamp'] for i in imu]) - t_on,
                to=np.array([t for t, _ in d['/odom/local']]) - t_on, yaw=np.unwrap([o['yaw'] for o in odo]),
                tg_stamp=np.array([g['stamp'] for _, g in gps]) - t_on)


def load_all():
    if CACHE.exists():
        return pickle.loads(CACHE.read_bytes())
    ev = F.load()
    out = {s.name: read_raw(s.name) for s in ev.segments}
    for s in ev.segments:
        out[s.name]['session'] = s.session
        out[s.name]['labels'] = s.labels
    CACHE.write_bytes(pickle.dumps(out))
    return out
