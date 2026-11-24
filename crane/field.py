"""Field data as crane.run/1: an astro bundle's bags (or any rosbag2 MCAP dir) to one run record per bag.

    crane field import BUNDLE.tgz|BAG_DIR|FILE.mcap --out RUN_DIR [--odom /odom/local] [--events /events]
                                                    [--frame odom=enu] [--hz 20]

A run is ``kind: field`` (no tier). What goes where:

- ``traj.ctraj``: planar ``boat.x boat.y boat.yaw`` from the odometry topic, resampled at ``--hz`` on its
  header stamps (yaw from the quaternion, unwrapped before resampling).
- ``frame``: from the odometry ``header.frame_id`` and ``child_frame_id``. ROS REP 103/105 frames (``odom``,
  ``map``, ``world``, ...) are ENU with an FLU body; a ``*_ned`` frame is NED with an FRD body. Any other
  frame id is refused unless ``--frame ID=enu|ned`` says what it is. The origin is the frame id.
- ``clock``: header stamps on the wall clock (unix seconds, within an hour of the bag's receive times) give
  ``source: wall``. Stamps on another clock (a simulator's) give ``source: sim`` with ``time_multiplier``
  the fitted sim seconds per wall second. ``timing.t_src`` names the stamps used (``header.stamp``).
- ``events.jsonl``: astro's ``/events`` lines (std_msgs/String) and the bundle's ``log.txt`` lines, each
  ``{t, kind: log, log_kind, text, source}`` with ``t`` on the record clock. A ``log.txt`` line also on
  ``/events`` is kept once. ``log.txt`` times are local: their offset comes from the lines matched on
  ``/events``; with none matched, this machine's time zone is assumed (``tz_assumed`` in the provenance).
- ``metrics.json``: duration, samples, odometry rate, path length, event count.

MCAP is read with ``plantcard/marine/mcap_lite.py`` (stdlib, uncompressed chunks) and, for compressed
chunks, the ``mcap`` package of the ``[field]`` extra.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

FRAMES = {'odom': 'enu', 'map': 'enu', 'world': 'enu', 'earth': 'enu', 'odom_enu': 'enu', 'map_enu': 'enu',
          'enu': 'enu', 'odom_ned': 'ned', 'map_ned': 'ned', 'ned': 'ned', 'local_ned': 'ned'}
BODY = {'enu': 'flu', 'ned': 'frd'}
WALL_EPOCH_MIN = 1e9          # 2001-09: a stamp past this is unix time
WALL_SKEW_MAX_S = 3600.0


class FieldError(ValueError):
    pass


# ---- MCAP ---------------------------------------------------------------------------------------------

def mcap_files(where):
    """The .mcap files of a bag dir (rosbag2 splits in order), a loose file, or every bag under a dir."""
    p = Path(where)
    if p.is_file():
        return [p]
    files = sorted(p.glob('*.mcap'), key=lambda f: (len(f.name), f.name))
    return files or sorted(p.rglob('*.mcap'))


def read_mcap(path, topics):
    """{topic: [(log_time_s, cdr bytes)]}: mcap_lite, else the ``mcap`` package for compressed chunks."""
    from plantcard.marine import mcap_lite
    try:
        return mcap_lite.read(path, topics)
    except ValueError as error:
        if 'compressed' not in str(error):
            raise
    try:
        from mcap.reader import make_reader
    except ImportError:
        raise FieldError(f'{path}: compressed MCAP chunks need the [field] extra (pip install mcap)') from None
    out = {t: [] for t in topics}
    with open(path, 'rb') as f:
        for _, channel, message in make_reader(f).iter_messages(topics=list(topics)):
            out[channel.topic].append((message.log_time * 1e-9, bytes(message.data)))
    return out


def read_bag(where, topics):
    out = {t: [] for t in topics}
    files = mcap_files(where)
    if not files:
        raise FieldError(f'{where}: no .mcap files')
    for f in files:
        for topic, rows in read_mcap(f, topics).items():
            out[topic] += rows
    for rows in out.values():
        rows.sort(key=lambda r: r[0])
    return out, files


def string_msg(data):
    """std_msgs/String from CDR."""
    import struct
    n = struct.unpack_from('<I', data, 4)[0]
    return data[8:8 + n].rstrip(b'\0').decode('utf-8', 'replace')


def odometry(rows):
    """Arrays of an odometry stream: t_log, t_hdr, x, y, yaw, vx, vy, wz, and frame ids (first message)."""
    from plantcard.marine import mcap_lite
    if not rows:
        raise FieldError('no odometry messages')
    dec = [(t, mcap_lite.odometry(d)) for t, d in rows]
    a = {k: np.array([o[k] for _, o in dec], float) for k in ('x', 'y', 'yaw', 'vx', 'vy', 'wz')}
    a['t_log'] = np.array([t for t, _ in dec], float)
    a['t_hdr'] = np.array([o['stamp'] for _, o in dec], float)
    order = np.argsort(a['t_hdr'], kind='stable')
    a = {k: v[order] for k, v in a.items()}
    a['frame'], a['child'] = dec[0][1]['frame'], dec[0][1]['child']
    return a


def pwm(rows):
    """(t_log, [n, k] microseconds) of a UInt16MultiArray stream (physical outputs in order)."""
    from plantcard.marine import mcap_lite
    return (np.array([t for t, _ in rows], float),
            np.array([mcap_lite.uint16_array(d) for _, d in rows], float).reshape(len(rows), -1))


# ---- clocks and frames ----------------------------------------------------------------------------------

def clock_of(t_log, t_hdr):
    """(clock, to_record(t_log) -> s from clock t0, t_src). Header stamps are the time base; receive times
    map onto them (a constant latency on a wall clock, a linear fit on another clock)."""
    t0 = float(t_hdr[0])
    skew = float(np.median(t_log - t_hdr))
    if np.median(t_hdr) > WALL_EPOCH_MIN and abs(skew) < WALL_SKEW_MAX_S:
        clock = {'source': 'wall', 't0': t0, 'time_multiplier': 1.0}
        return clock, (lambda t: np.asarray(t, float) - skew - t0), {'latency_s': skew}
    if len(t_log) < 2 or np.ptp(t_log) <= 0:
        raise FieldError('header stamps are not on the wall clock and the bag is too short to map them')
    b, a = np.polyfit(t_log, t_hdr, 1)
    clock = {'source': 'sim', 't0': t0, 'time_multiplier': float(b)}
    fit = {'sim_per_wall': float(b), 'sim_at_wall0': float(a), 'wall_t0': float((t0 - a) / b)}
    return clock, (lambda t: a + b * np.asarray(t, float) - t0), fit


def frame_of(frame_id, child, overrides=None):
    key = (frame_id or '').strip('/')
    world = (overrides or {}).get(key) or FRAMES.get(key)
    if world not in ('enu', 'ned'):
        raise FieldError(f'odometry frame {frame_id!r} is not a known frame ({", ".join(sorted(FRAMES))}); '
                         f'say what it is with --frame {key or "ID"}=enu|ned')
    body = 'frd' if (child or '').endswith('_frd') else 'flu' if (child or '').endswith('_flu') else BODY[world]
    return {'world': world, 'origin': key, 'body': body}


# ---- events ---------------------------------------------------------------------------------------------

def _seconds_of_day(stamp):
    h, m, s = map(int, stamp.split(':'))
    return 3600 * h + 60 * m + s


def events(event_rows, log_lines, to_record):
    """``/events`` messages and log.txt lines (day, HH:MM:SS, kind, message) as record events, and the
    log.txt time-zone note."""
    out, seen, offsets = [], {}, []
    for t, data in event_rows:
        text = string_msg(data)
        parts = text.split(None, 2)
        kind = parts[1] if len(parts) > 1 else ''
        out.append({'t': float(to_record(t)), 'kind': 'log', 'log_kind': kind, 'text': text, 'source': '/events'})
        seen[text.strip()] = t
    for day, hms, kind, msg in log_lines:
        line = f'{hms}  {kind:<7} {msg}'.strip()
        try:
            base = _dt.datetime.fromisoformat(day).replace(tzinfo=_dt.timezone.utc).timestamp()
        except ValueError:
            continue
        naive = base + _seconds_of_day(hms)
        if line in seen:
            offsets.append(seen[line] - naive)
    if offsets:
        tz = float(np.median(offsets))
        note = {'tz_from_events_s': round(tz, 1), 'matched': len(offsets)}
    else:
        tz = -float(_dt.datetime.now().astimezone().utcoffset().total_seconds())
        note = {'tz_assumed': 'this machine', 'tz_offset_s': tz}
    for day, hms, kind, msg in log_lines:
        line = f'{hms}  {kind:<7} {msg}'.strip()
        if line in seen:
            continue
        try:
            base = _dt.datetime.fromisoformat(day).replace(tzinfo=_dt.timezone.utc).timestamp()
        except ValueError:
            continue
        wall = base + _seconds_of_day(hms) + tz
        out.append({'t': float(to_record(wall)), 'kind': 'log', 'log_kind': kind, 'text': line,
                    'source': 'log.txt'})
    out.sort(key=lambda e: e['t'])
    return out, note


# ---- one bag -> one run -------------------------------------------------------------------------------

def import_bag(bag, out_dir, *, odom_topic='/odom/local', events_topic='/events', log_lines=(), hz=20.0,
               frames=None, domain='boat', provenance=None, run_id=None):
    """Write ``out_dir`` as a crane.run/1 field record of one bag; returns the RunRecord."""
    from crane import record
    from theater import traj as ctraj
    streams, files = read_bag(bag, [odom_topic, events_topic])
    o = odometry(streams[odom_topic])
    frame = frame_of(o['frame'], o['child'], frames)
    clock, to_record, fit = clock_of(o['t_log'], o['t_hdr'])
    t = o['t_hdr'] - clock['t0']
    if len(t) < 2 or t[-1] <= 0:
        raise FieldError(f'{bag}: fewer than two odometry stamps')
    grid = np.arange(0.0, t[-1] + 1e-9, 1.0 / hz)
    yaw = np.unwrap(o['yaw'])
    cols = {'boat.x': np.interp(grid, t, o['x']).tolist(), 'boat.y': np.interp(grid, t, o['y']).tolist(),
            'boat.yaw': ((np.interp(grid, t, yaw) + math.pi) % (2 * math.pi) - math.pi).tolist()}
    evs, tz = events(streams[events_topic], log_lines, to_record)
    sha = hashlib.sha256()
    for f in files:
        sha.update(hashlib.sha256(f.read_bytes()).digest())
    header = {'pose_dims': 'planar', 'robot': {'model': domain, 'agents': ['boat']},
              'frame': {'type': frame['world'], 'origin': frame['origin'], 'body': frame['body']}}
    data = ctraj.encode(header, cols, [], hz=hz, t0_s=0.0)
    step = np.hypot(np.diff(o['x']), np.diff(o['y']))
    metrics = {'duration_s': round(float(t[-1]), 3), 'samples': int(len(t)),
               'odom_rate_hz': round(float((len(t) - 1) / t[-1]), 2), 'path_m': round(float(step.sum()), 3),
               'events': len(evs)}
    meta = record.meta(run_id or f'field-{Path(bag).name}-{sha.hexdigest()[:8]}', domain=domain, tier=None,
                       kind='field', frame=frame, clock=clock, dtype='float64', backend='field',
                       identity={'bags_sha256': sha.hexdigest(), 'files': [f.name for f in files]},
                       timing={'t_src': 'header.stamp', **fit},
                       provenance={**(provenance or {}), 'bag': str(bag), 'odom_topic': odom_topic,
                                   'events_topic': events_topic, 'events_clock': tz,
                                   'frame_assumption': f"{o['frame']!r} -> {frame['world']}/{frame['body']} "
                                                       '(REP 103/105 unless --frame said otherwise)'})
    return record.write_run(out_dir, meta, traj=data, events=evs, metrics=metrics)


def import_any(source, out, **kw):
    """A bundle (.tgz: every bag in it, log.txt lines as events), a bag dir or an .mcap file. Returns the
    RunRecords; a bundle or a dir of several bags gives OUT/<bag name>/ each."""
    src = Path(source)
    if src.is_file() and src.suffix in ('.tgz', '.gz', '.tar'):
        import tarfile
        with tempfile.TemporaryDirectory(prefix='crane-field-') as tmp:
            with tarfile.open(src) as tar:
                tar.extractall(tmp, filter='data')
            from crane import fit
            b = fit.open_bundle(src, tmp)
            if not b.bags:
                raise FieldError(f'{src}: the bundle has no bags (astro bundle --bags)')
            prov = {'bundle': src.name, 'bundle_sha256': b.sha256}
            return _each(b.bags, out, log_lines=b.log_lines, provenance=prov, **kw)
    bags = sorted({f.parent for f in mcap_files(src)}) if src.is_dir() else [src]
    return _each(bags, out, **kw)


def _each(bags, out, **kw):
    out = Path(out)
    if len(bags) == 1:
        return [import_bag(bags[0], out, **kw)]
    return [import_bag(b, out / Path(b).name, **kw) for b in bags]


def main(args):
    frames = {}
    for pair in args.frame or []:
        key, eq, world = pair.partition('=')
        if not eq or world not in ('enu', 'ned'):
            print(f'crane field: --frame wants ID=enu|ned, got {pair!r}', file=sys.stderr)
            return 2
        frames[key.strip('/')] = world
    try:
        runs = import_any(args.source, args.out, odom_topic=args.odom, events_topic=args.events, hz=args.hz,
                          frames=frames, domain=args.domain)
    except (FieldError, FileExistsError) as error:
        print(f'crane field: {error}', file=sys.stderr)
        return 2
    for r in runs:
        m, c = r.metrics, r.clock
        print(f'{r.path}: {m["duration_s"]:.1f} s, {m["samples"]} odometry samples at {m["odom_rate_hz"]} Hz, '
              f'{m["path_m"]:.1f} m, {m["events"]} events; frame {r.frame["world"]}/{r.frame["body"]} at '
              f'{r.frame["origin"]!r}, clock {c["source"]} (x{c["time_multiplier"]:.3g})')
    return 0

