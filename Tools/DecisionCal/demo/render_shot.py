#!/usr/bin/env python3
"""Render one shot script of a demo pack with the prebuilt player through Theater, under ``heavy unity``.

    python3 render_shot.py PACK SHOT [--out OUT] [--size 1920x1080] [--caption TEXT] [--no-heavy]

SHOT is the name of PACK/script_<SHOT>.json (make_pack.py writes stills, clip, compare). The command it runs
(also saved to OUT/command.txt) is::

    HEAVY_WHO=decisioncal-demo heavy unity python3 -m theater.render local PACK --player CRANE.x86_64 --out OUT \
        --size WxH --fps 30 --frames A:B [--stills 30] -- --crane-theater-script PACK/script_<SHOT>.json

Frozen shots (stills, compare): every act is 1 s (30 frames); frames 15..end are encoded and a PNG is kept every
30 frames, i.e. frame 15 of each act (Theater pre-rolls 2 s before frame A and discards them, so even the first
still has rendered history). The PNGs are copied to OUT/<SHOT>_<act>.png. The clip is the full script as
OUT/video.mp4.

Caption bar: a 64 px black bar is added BELOW the rendered frame with ffmpeg (``pad`` + ``drawtext``), never on
it: OUT/captioned/... The rendered pixels are untouched; nothing is painted into the scene.
"""

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]                                    # the worktree (theater/ package)
PLAYER = Path.home() / '.cache' / 'nlc-lockstep' / 'player-linux' / 'CRANE.x86_64'
HEAVY = Path.home() / 'astro_dock' / 'src' / 'mhseals_agents' / 'tools' / 'heavy'
FONT = '/usr/share/fonts/noto/NotoSans-Regular.ttf'
FFMPEG = shutil.which('ffmpeg') or '/usr/bin/ffmpeg'


def esc(text):
    return text.replace('\\', '\\\\').replace(':', '\\:').replace("'", "’").replace('%', '\\%')


def caption_filter(lines, height=84, size=24):
    """A black bar of ``height`` px padded BELOW the frame, one drawtext per line (never on the rendered pixels)."""
    f = [f"pad=iw:ih+{height}:0:0:black"]
    step = height / max(1, len(lines))
    for i, line in enumerate(lines):
        f.append(f"drawtext=fontfile={FONT}:text='{esc(line)}':fontcolor=white:fontsize={size}:"
                 f"x=24:y=h-{height}+{int(i * step + (step - size) / 2)}")
    return ','.join(f)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pack')
    ap.add_argument('shot')
    ap.add_argument('--out', default=None, help='default: PACK/render-<SHOT>')
    ap.add_argument('--size', default='1920x1080')
    ap.add_argument('--caption', default=None, help='caption bar text (default: built from the manifest)')
    ap.add_argument('--no-heavy', action='store_true', help='do not wrap in heavy (only if you hold a slot yourself)')
    ap.add_argument('--timeout', type=float, default=1800)
    ap.add_argument('--caption-only', action='store_true', help='re-caption an existing render (no Unity)')
    a = ap.parse_args(argv)
    pack = Path(a.pack).resolve()
    script_path = pack / f'script_{a.shot}.json'
    script = json.loads(script_path.read_text())
    man = json.loads((pack / 'manifest.json').read_text())
    out = Path(a.out).resolve() if a.out else pack / f'render-{a.shot}'
    if a.caption_only:
        return caption(a, pack, out, script, man)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    fps = int(script['fps'])
    per_act = [round(x['duration_s'] * fps) for x in script['acts']]
    total = sum(per_act)
    frozen = all(x.get('speed', 1) < 0.05 for x in script['acts'])
    cmd = [sys.executable, '-m', 'theater.render', 'local', str(pack), '--player', str(PLAYER), '--out', str(out),
           '--size', a.size, '--fps', str(fps)]
    if frozen:
        cmd += ['--frames', f'15:{total}', '--stills', str(fps)]
    else:
        cmd += ['--frames', f'0:{total}']
    cmd += ['--', '--crane-theater-script', str(script_path)]
    if not a.no_heavy:
        cmd = [sys.executable, str(HEAVY), 'unity'] + cmd
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    env.update({'HEAVY_WHO': os.environ.get('HEAVY_WHO', 'decisioncal-demo'), 'HEAVY_MAX_S': str(int(a.timeout)),
                'SDL_VIDEODRIVER': 'dummy'})
    (out / 'command.txt').write_text(f'cd {shlex.quote(str(ROOT))} && HEAVY_WHO={env["HEAVY_WHO"]} '
                                     + ' '.join(shlex.quote(c) for c in cmd) + '\n')
    print((out / 'command.txt').read_text().strip(), flush=True)
    t0 = time.time()
    rc = subprocess.run(cmd, cwd=str(ROOT), env=env, timeout=a.timeout + 600).returncode
    wall = time.time() - t0
    rep = out / 'render_report.json'
    report = json.loads(rep.read_text()) if rep.is_file() else {}
    meta = {'shot': a.shot, 'rc': rc, 'wall_s_incl_admission': wall, 'render_wall_s': report.get('wall_s'),
            'frames': report.get('frames'), 'errors': report.get('errors'), 'error_lines': report.get('error_lines'),
            'world': report.get('world'), 'device': report.get('device'), 'gfx': report.get('gfx'),
            'script_sha256': hashlib.sha256(script_path.read_bytes()).hexdigest(),
            'pack_sha256': hashlib.sha256((pack / 'tracks.pack').read_bytes()).hexdigest(),
            'truth_sha256': hashlib.sha256((pack / 'course_truth.json').read_bytes()).hexdigest(),
            'water': man.get('water'), 'pose_dims': man.get('pose_dims')}
    print(json.dumps({k: meta[k] for k in ('rc', 'wall_s_incl_admission', 'render_wall_s', 'frames', 'errors', 'world')}, indent=1))
    (out / 'render_meta.json').write_text(json.dumps(meta, indent=1))
    caption(a, pack, out, script, man)
    return rc


def caption(a, pack, out, script, man):
    meta = json.loads((out / 'render_meta.json').read_text())
    fps = int(script['fps'])
    per_act = [round(x['duration_s'] * fps) for x in script['acts']]
    frozen = all(x.get('speed', 1) < 0.05 for x in script['acts'])
    cap_dir = out / 'captioned'
    cap_dir.mkdir(exist_ok=True)
    ids = (man['decisioncal'].get('run_ids') or {})
    backends = '  vs  '.join(f"{v} ({'gold, Unity lockstep plant, player e27629b' if v.startswith('unity') else 'silver, NumPy plant'})"
                             for v in sorted(ids.values(), key=lambda v: not v.startswith('unity'))) if a.shot == 'compare' else \
        f"{man['decisioncal']['unity_run_id']}  |  Unity lockstep plant (player e27629b), external controller c08, noise seed 1"
    water = f"replayed in CRANE Theater from the run's own trajectory, HDRP water '{(man.get('water') or {}).get('mode')}'"
    rc = meta.get('rc', 0)
    outputs = []
    if frozen:
        stills = sorted((out / 'stills').glob('frame_*.png'))
        start = 0
        for i, n in enumerate(per_act):
            want = start + 15
            src = out / 'stills' / f'frame_{want:06d}.png'
            start += n
            if not src.is_file():
                continue
            dst = out / f'{a.shot}_{i}.png'
            if not dst.is_file():
                shutil.copyfile(src, dst)
            act_ = script['acts'][i]
            lines = [a.caption or backends, f"{water}  |  {act_.get('note') or act_['caption']}"]
            subprocess.run([FFMPEG, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(dst), '-vf',
                            caption_filter(lines), str(cap_dir / dst.name)], check=False)
            outputs.append(str(dst))
        meta['stills_found'] = [s.name for s in stills]
    elif (out / 'video.mp4').is_file():
        lines = [a.caption or backends, water]
        subprocess.run([FFMPEG, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(out / 'video.mp4'), '-vf',
                        caption_filter(lines), '-c:v', 'libx264', '-crf', '18', '-preset', 'medium',
                        '-pix_fmt', 'yuv420p', str(cap_dir / f'{a.shot}.mp4')], check=False)
        outputs.append(str(out / 'video.mp4'))
    meta['outputs'] = outputs
    (out / 'render_meta.json').write_text(json.dumps(meta, indent=1))
    print('\n'.join(outputs))
    return rc


if __name__ == '__main__':
    sys.exit(main())
