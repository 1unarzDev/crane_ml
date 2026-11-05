"""Render a ghost-theater video: locally (a player on this machine) or on the cluster, then assemble and check.

    python3 -m theater.render script DIR [--seconds 15] [--fps 60]       # write the standard cut into DIR/manifest.json
    python3 -m theater.render local DIR --player CRANE.x86_64 --out OUT [--frames A:B] [--size WxH] [--display :1]
    python3 -m theater.render pack SELECT.json --trajs DIR --truth course_truth.json --world NAME --out DIR   # from select + fetch
    python3 -m theater.render submit DIR --purpose '...' [--chunk-s 10] [--worker NAME] [--render-id R]   # job id
    python3 -m theater.render assemble JOB --out OUT [--render-id R]   # chunks -> video.mp4, final_h265.mp4, gif, sheets
    python3 -m theater.render inspect OUT                # the render report's checks; exit 1 when one fails

DIR holds manifest.json, its pack and the world's inputs (e.g. course_truth.json). Run ``local`` under the host's
heavy-job guard. Cluster calls read the coordinator and tokens from ~/.config/crane-cluster (secrets.env:
ADMIN_TOKEN for submit, VIEWER_TOKEN or ADMIN_TOKEN for assemble); tokens are never printed.
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

from .worlds import WORLDS

CONFIG = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'crane-cluster'


# ---------------------------------------------------------------------- the standard cut

def default_script(manifest, seconds=15.0, fps=60, size=(1920, 1080)):
    """Establishing fleet coloured by iteration, the evolution (an evenly spaced subset of iterations when the
    cut is short), where runs went and failed, best vs previous best. Speeds follow the runs' typical length."""
    runs = manifest['runs']
    its = sorted({r['iteration'] for r in runs})
    done = [r['duration_s'] for r in runs if r.get('outcome') == 'finished' and r.get('duration_s')] or \
           [r.get('duration_s') or 60 for r in runs]
    typical = statistics.median(done)
    records, best = [], -math.inf
    for r in sorted(runs, key=lambda r: r['iteration']):
        if r.get('valid', True) and r['score'] > best:
            best = r['score']
            records.append(r)
    gold = records[-1]['duration_s'] if records else typical
    fleet, evo, heat, cmp = (round(seconds * k, 3) for k in (0.28, 0.27, 0.15, 0.30))
    shown = max(1, min(len(its), int(evo / 0.8)))
    pick = sorted({its[round(i * (len(its) - 1) / max(1, shown - 1))] for i in range(shown)}) if len(its) > 1 else its
    per = evo / len(pick)
    acts = [
        {'type': 'fleet', 'duration_s': fleet, 'speed': round(typical * 0.55 / fleet, 3), 'stagger_s': round(typical * 0.08, 2),
         'colour': 'iteration', 'ramp': 'viridis', 'trails': 'comet', 'tail_s': round(max(2.5, typical * 0.55 / fleet * 0.35), 2),
         'caption': f'{len(runs)} runs over {len(its)} iterations',
         'camera': {'shot': 'pack', 'height': 24, 'back': 26, 'deg_per_s': 5}},
        {'type': 'evolution', 'duration_s': evo, 'per_iteration_s': round(per, 4), 'iteration_list': pick,
         'speed': round(typical * 0.75 / per, 3), 'colour': 'progress', 'ramp': 'cividis', 'trails': 'comet',
         'tail_s': round(max(2.5, typical * 0.75 / per * 0.2), 2),
         'caption': 'evolution: each iteration’s fleet', 'camera': {'shot': 'top', 'pitch': 64}},
        {'type': 'heat', 'duration_s': heat, 'speed': round(typical * 1.2 / heat, 3), 'colour': 'outcome', 'trails': 'none',
         'heat': True, 'caption': 'where runs went, and where they failed', 'camera': {'shot': 'top', 'pitch': 80, 'push': 0.05}},
        {'type': 'compare', 'duration_s': cmp, 'speed': round(max(1.0, gold * 0.95 / cmp), 3), 'trails': 'top', 'trail_k': 8,
         'colour': 'progress', 'caption': 'best (gold) vs previous best (silver)',
         'camera': {'shot': 'follow', 'height': 6, 'back': 8, 'lead': 5}},
    ]
    return {'fps': fps, 'size': list(size), 'title': manifest.get('study') or 'crane study', 'acts': acts}


def frame_count(script):
    total, frames = 0.0, 0
    for a in script['acts']:
        frames = round((total + a['duration_s']) * script['fps'])
        total += a['duration_s']
    return frames


def cmd_script(a):
    d = Path(a.dir)
    m = json.loads((d / 'manifest.json').read_text())
    m['script'] = default_script(m, a.seconds, a.fps, tuple(int(x) for x in a.size.split('x')))
    (d / 'manifest.json').write_text(json.dumps(m, indent=1, sort_keys=True) + '\n')
    print(f"{d / 'manifest.json'}: {len(m['script']['acts'])} acts, {frame_count(m['script'])} frames")


# ---------------------------------------------------------------------- local

def player_argv(player, d, manifest, out, frames=None, extra=()):
    world = [str(x).replace('{dir}', str(d)) for x in (manifest.get('world') or {}).get('args') or []]
    argv = [str(player), '-batchmode', '-screen-fullscreen', '0', '-screen-width', '320', '-screen-height', '180',
            '-logFile', str(out / 'player.log'), '--crane-disable-ros', '--crane-disable-sitl'] + world + [
            '--crane-theater', str(d / 'manifest.json'), '--crane-theater-out', str(out)]
    if frames:
        argv += ['--crane-theater-frames', frames]
    return argv + list(extra)


def cmd_local(a):
    d, out = Path(a.dir).resolve(), Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    m = json.loads((d / 'manifest.json').read_text())
    extra = ['--crane-theater-ffmpeg', a.ffmpeg or shutil.which('ffmpeg') or 'none']
    for flag, v in (('--crane-theater-size', a.size), ('--crane-theater-fps', a.fps), ('--crane-theater-stills', a.stills),
                    ('--crane-theater-preset', a.preset)):
        if v:
            extra += [flag, str(v)]
    extra += a.args or []
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    if a.display:
        env['DISPLAY'] = a.display        # batchmode: a GPU context only, no window
    else:
        env['SDL_VIDEODRIVER'] = 'dummy'
    argv = player_argv(Path(a.player).resolve(), d, m, out, a.frames, extra)
    rc = subprocess.run(argv, cwd=str(Path(a.player).resolve().parent), env=env, stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, timeout=a.timeout).returncode
    report = out / 'render_report.json'
    print(f'player exit {rc}; report {report if report.is_file() else "missing"}')
    if report.is_file():
        contact_sheets(out / 'video.mp4', out, a.ffmpeg or shutil.which('ffmpeg'))
    return rc


# ---------------------------------------------------------------------- cluster

def secrets():
    env = {}
    p = CONFIG / 'secrets.env'
    if p.is_file():
        for line in p.read_text().splitlines():
            if '=' in line and not line.lstrip().startswith('#'):
                k, v = line.split('=', 1)
                env[k.strip().removeprefix('export ').strip()] = v.strip().strip('"\'')
    return env


def coordinator():
    for line in (CONFIG / 'worker.yaml').read_text().splitlines() if (CONFIG / 'worker.yaml').is_file() else []:
        if line.startswith('coordinator:'):
            return line.split(':', 1)[1].strip()
    return os.environ.get('CRANE_COORDINATOR', 'https://crane.lunarz.dev')


def http(method, path, token, data=None, body=None, headers=None):
    hdrs = {'authorization': f'Bearer {token}', 'user-agent': 'crane-theater', **(headers or {})}
    if body is not None:
        data = json.dumps(body).encode()
        hdrs['content-type'] = 'application/json'
    req = urllib.request.Request(coordinator().rstrip('/') + path, data=data, method=method, headers=hdrs)
    with urllib.request.urlopen(req, timeout=600) as r:
        raw = r.read()
        return json.loads(raw) if 'json' in r.headers.get('content-type', '') else raw


def input_files(d, m):
    names = ['manifest.json', m.get('pack', 'tracks.pack')]
    for v in (m.get('world') or {}).values():
        if isinstance(v, str) and (d / v).is_file() and v not in names:
            names.append(v)
    return names


def cmd_submit(a):
    d = Path(a.dir)
    m = json.loads((d / 'manifest.json').read_text())
    if 'script' not in m:
        raise SystemExit('manifest has no script: run `theater.render script DIR` first')
    token = secrets().get('ADMIN_TOKEN') or os.environ.get('CRANE_ADMIN_TOKEN')
    if not token:
        raise SystemExit('no ADMIN_TOKEN in ~/.config/crane-cluster/secrets.env')
    inputs = {}
    for name in input_files(d, m):
        data = (d / name).read_bytes()
        sha = hashlib.sha256(data).hexdigest()
        http('PUT', f'/v1/admin/inputs/{sha}?name={name}', token, data=data, headers={'content-type': 'application/octet-stream'})
        inputs[name] = sha
    s = m['script']
    total = frame_count(s)
    chunk = max(1, round(a.chunk_s * s['fps']))
    seeds = list(range(math.ceil(total / chunk)))
    tags = [a.tag] + ([a.worker] if a.worker else [])
    job = {'backend': 'theater-render', 'seeds': seeds, 'priority': a.priority, 'max_attempts': 3,
           'purpose': a.purpose, 'est_seconds': max(60.0, a.chunk_s * 12), 'submitter': a.submitter,
           **({'campaign': a.campaign} if a.campaign else {}),
           'spec': {'inputs': inputs, 'chunk_frames': chunk, 'total_frames': total, 'study': m.get('study'),
                    **({'stills': a.stills} if a.stills else {})},
           'needs': {'gpu': True, 'tags': tags, 'est_minutes': max(5, round(a.chunk_s * 2))}}
    r = http('POST', '/v1/admin/jobs', token, body={'jobs': [job]})
    jid = r['job_ids'][0]
    if a.render_id:   # a dashboard render request: it now has a job
        http('POST', f'/v1/admin/renders/{a.render_id}', token, body={'status': 'rendering', 'render_job_id': jid})
    print(json.dumps({'job_id': jid, 'chunks': len(seeds), 'frames': total, 'chunk_frames': chunk}))


def viewer_token():
    s = secrets()
    t = s.get('VIEWER_TOKEN') or s.get('ADMIN_TOKEN') or os.environ.get('CRANE_VIEWER_TOKEN')
    if not t:
        raise SystemExit('no VIEWER_TOKEN or ADMIN_TOKEN in ~/.config/crane-cluster/secrets.env')
    return t


def cmd_assemble(a):
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    token = viewer_token()
    arts = http('GET', f'/v1/artifacts?job={a.job}', token)['artifacts']
    chunks = {}
    for art in arts:
        run = art.get('run_id') or ''
        seed = int(run.rsplit('.s', 1)[1]) if '.s' in run else None
        if seed is None:
            continue
        attempt = int(run.split('.a')[-1].split('.')[0]) if '.a' in run else 0
        prev = chunks.get(seed, {}).get('_attempt', -1)
        if attempt < prev:
            continue
        if attempt > prev:
            chunks[seed] = {'_attempt': attempt}
        chunks[seed][art['name']] = art
    ffmpeg = a.ffmpeg or shutil.which('ffmpeg')
    parts, reports = [], []
    cdir = out / 'chunks'
    cdir.mkdir(exist_ok=True)
    for seed in sorted(chunks):
        c = chunks[seed]
        if 'video.mp4' not in c:
            raise SystemExit(f'chunk {seed} has no video.mp4 (job incomplete?)')
        for name in ('video.mp4', 'render_report.json', 'frames.jsonl', 'contact.png'):
            if name in c:
                data = http('GET', f"/v1/artifact/{c[name]['key']}", token)
                if isinstance(data, (dict, list)):
                    data = json.dumps(data).encode()
                if hashlib.sha256(data).hexdigest() != c[name]['sha256']:
                    raise SystemExit(f'chunk {seed} {name}: sha256 mismatch')
                (cdir / f'c{seed:04d}_{name}').write_bytes(data)
        parts.append(cdir / f'c{seed:04d}_video.mp4')
        reports.append(json.loads((cdir / f'c{seed:04d}_render_report.json').read_text()))
    lst = out / 'concat.txt'
    lst.write_text(''.join(f"file '{p.resolve()}'\n" for p in parts))
    run = lambda *argv: subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', *argv], check=True)
    run('-f', 'concat', '-safe', '0', '-i', str(lst), '-c', 'copy', '-movflags', '+faststart', str(out / 'video.mp4'))
    merged = finish(out, ffmpeg, reports, a.job)
    if a.render_id:   # answer the dashboard's render request with the video
        token = secrets().get('ADMIN_TOKEN')
        for name in ('final_h265.mp4', 'video.mp4'):
            p = out / name
            if p.is_file() and p.stat().st_size < 95 * 2 ** 20:
                http('PUT', f'/v1/admin/renders/{a.render_id}/video/{name}', token, data=p.read_bytes(),
                     headers={'content-type': 'application/octet-stream'})
                break
        if not merged['ok']:
            http('POST', f'/v1/admin/renders/{a.render_id}', token, body={'status': 'failed', 'note': 'render report not ok'})


def finish(out, ffmpeg, reports, job=None):
    run = lambda *argv: subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', *argv], check=True)
    try:
        run('-i', str(out / 'video.mp4'), '-c:v', 'libx265', '-crf', '24', '-preset', 'medium', '-tag:v', 'hvc1',
            '-pix_fmt', 'yuv420p', '-threads', '4', str(out / 'final_h265.mp4'))
    except subprocess.CalledProcessError:
        pass
    run('-i', str(out / 'video.mp4'), '-vf', 'fps=15,scale=854:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=192[p];[b][p]paletteuse=dither=sierra2_4a',
        str(out / 'preview.gif'))
    contact_sheets(out / 'video.mp4', out, ffmpeg)
    merged = merge_reports(reports)
    merged['job'] = job
    (out / 'render_report.json').write_text(json.dumps(merged, indent=1, sort_keys=True) + '\n')
    print(json.dumps({'video': str(out / 'video.mp4'), 'ok': merged['ok'], 'frames': merged['frames_written'],
                      'checks': merged['checks']}))
    return merged


def contact_sheets(video, out, ffmpeg, every_s=5):
    """Contact sheets: one frame every ``every_s`` seconds, 4 x 4 per sheet, 480 px tiles."""
    if not ffmpeg or not Path(video).is_file():
        return
    subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(video), '-vf',
                    f'fps=1/{every_s},scale=480:-1,tile=4x4', str(Path(out) / 'contact_%02d.png')], check=False)
    subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(video), '-vf',
                    'fps=1,scale=480:-1,tile=4x4', str(Path(out) / 'contact_1s_%02d.png')], check=False)


def merge_reports(reports):
    m = {'schema': 'crane-theater-report/1', 'chunks': len(reports), 'ok': all(r.get('ok') for r in reports),
         'frames_written': sum(r.get('frames_written', 0) for r in reports), 'errors': sum(r.get('errors', 0) for r in reports),
         'checks': {}, 'acts': {}, 'hud_samples': [], 'devices': sorted({f"{r.get('device')} ({r.get('gfx')})" for r in reports}),
         'error_lines': [line for r in reports for line in r.get('error_lines', [])][:40]}
    for r in reports:
        for k, v in (r.get('checks') or {}).items():
            m['checks'][k] = m['checks'].get(k, 0) + v if k not in ('events', 'events_off_path') else v
        for k, v in (r.get('acts') or {}).items():
            a = m['acts'].setdefault(k, dict(v, frames=0))
            a['frames'] += v.get('frames', 0)
            a['ghosts_min'] = min(a.get('ghosts_min', 1 << 30), v.get('ghosts_min', 1 << 30))
            a['ghosts_max'] = max(a.get('ghosts_max', 0), v.get('ghosts_max', 0))
        m['hud_samples'] += r.get('hud_samples', [])
        for k in ('fps', 'size', 'total_frames', 'script', 'unity', 'quality', 'world'):
            m.setdefault(k, r.get(k))
    return m


def cmd_pack(a):
    from . import pack
    world = WORLDS[a.world]
    truth = json.loads(Path(a.truth).read_text())
    if a.tasks:
        truth = world.only_tasks(truth, a.tasks.split(','))
    doc = pack.from_select(a.select, a.trajs, world(truth), a.out, hz=a.hz)
    print(f"{len(doc['runs'])} runs -> {a.out} (skipped {len(doc['source']['skipped'])} without a trajectory)")


# ---------------------------------------------------------------------- inspect

CHECKS = {'nan_poses': 0, 'teleports': 0, 'iteration_counter_backwards': 0, 'hud_count_mismatch': 0, 'events_off_path': 0}


def inspect(report):
    """Problems a render report shows (empty: the automated checks pass)."""
    bad = []
    if not report.get('ok'):
        bad.append('render not ok (encoder failed or frames missing)')
    for k, limit in CHECKS.items():
        v = (report.get('checks') or {}).get(k, 0)
        if v > limit:
            bad.append(f'{k} = {v}')
    off = (report.get('checks') or {}).get('off_world_poses', 0)
    if off > 0.01 * max(1, report.get('frames_written', 1)):
        bad.append(f'off_world_poses = {off}')
    if report.get('errors'):
        bad.append(f"{report['errors']} errors logged: {report.get('error_lines', [])[:3]}")
    for k, a in (report.get('acts') or {}).items():
        if a.get('ghosts_max', 0) == 0 and a.get('type') != 'compare':
            bad.append(f'act {k} ({a.get("type")}) shows no ghosts')
    return bad


def cmd_inspect(a):
    r = json.loads((Path(a.out) / 'render_report.json').read_text())
    bad = inspect(r)
    print(json.dumps({'pass': not bad, 'problems': bad, 'checks': r.get('checks'), 'frames': r.get('frames_written')}, indent=1))
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('script'); p.add_argument('dir'); p.add_argument('--seconds', type=float, default=15)
    p.add_argument('--fps', type=int, default=60); p.add_argument('--size', default='1920x1080')
    p = sub.add_parser('local'); p.add_argument('dir'); p.add_argument('--player', required=True); p.add_argument('--out', required=True)
    p.add_argument('--frames'); p.add_argument('--size'); p.add_argument('--fps'); p.add_argument('--stills', type=int)
    p.add_argument('--preset'); p.add_argument('--ffmpeg'); p.add_argument('--display'); p.add_argument('--timeout', type=float, default=7200)
    p.add_argument('args', nargs='*')
    p = sub.add_parser('submit'); p.add_argument('dir'); p.add_argument('--chunk-s', type=float, default=10)
    p.add_argument('--tag', default='theater-render'); p.add_argument('--worker'); p.add_argument('--priority', type=int, default=0)
    p.add_argument('--stills', type=int); p.add_argument('--render-id', help='the dashboard render request this answers')
    p.add_argument('--purpose', required=True, help='1-2 sentences: what this render is for (public feed)')
    p.add_argument('--submitter', default='theater-render'); p.add_argument('--campaign')
    p = sub.add_parser('assemble'); p.add_argument('job'); p.add_argument('--out', required=True); p.add_argument('--ffmpeg')
    p.add_argument('--render-id', help='upload the final video to this dashboard render request')
    p = sub.add_parser('pack', help='a theater manifest and pack from `python3 -m theater select` and `fetch`')
    p.add_argument('select'); p.add_argument('--trajs', required=True); p.add_argument('--truth', required=True)
    p.add_argument('--world', required=True, choices=sorted(WORLDS), help='the world plug-in (theater/worlds.py)')
    p.add_argument('--tasks', help="keep only these tasks of the truth (ids, comma-separated)")
    p.add_argument('--hz', type=float, default=10); p.add_argument('--out', required=True)
    p = sub.add_parser('inspect'); p.add_argument('out')
    a = ap.parse_args(argv)
    return {'script': cmd_script, 'local': cmd_local, 'submit': cmd_submit, 'assemble': cmd_assemble, 'inspect': cmd_inspect,
            'pack': cmd_pack}[a.cmd](a) or 0


if __name__ == '__main__':
    sys.exit(main())
