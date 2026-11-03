"""The ``theater-render`` backend: renders chunks of a ghost-theater video with the crane player.

A job is one video; each seed is one chunk (frames ``[seed * chunk, (seed + 1) * chunk)``), so chunks render
in parallel on any GPU worker and upload separately (each well under the 95 MB artifact cap);
``python3 -m theater.render assemble`` joins them. Inputs (the manifest, its track pack, the world's files
and an optional act script) are content-addressed blobs the submitter uploaded (``PUT /v1/admin/inputs/<sha>``);
the worker fetches them with its own token and checks each hash.

Worker YAML (``path`` puts this crane copy on sys.path)::

    theater-render:
      type: "cluster.backends.theater:ClusterBackend"
      path: ~/crane                    # this crane copy (cluster/ and theater/)
      player: ~/players/CRANE-Worker/CRANE.x86_64      # the worker player (has --crane-theater)
      ffmpeg: /usr/bin/ffmpeg          # or ffmpeg_url + ffmpeg_sha256 (a pinned static build, cached)
      display: null                    # Linux: an X display for the NVIDIA Vulkan device, if it needs one;
                                       # batchmode opens no window. Unset: DISPLAY is removed.
      args: []                         # extra player arguments (e.g. -force-vulkan)
      timeout_s: 3600                  # per chunk

Job spec: ``inputs`` {file name: sha256} (must include the manifest, default name ``manifest.json``),
``manifest`` (its name), ``script`` (a name among the inputs, optional), ``chunk_frames`` (required),
``fps``, ``size`` ("WxH"), ``crf`` (18), ``preset`` (medium), ``preroll_s`` (2), ``quality``, ``stills``.
Needs: ``{"gpu": true, "tags": ["theater-render"]}``; a worker gets the tag once its render spike passes.
Artifacts per chunk: ``video.mp4``, ``frames.jsonl``, ``render_report.json``, ``contact.png`` (summary) and
``player.log`` (raw).
"""

import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import tarfile
import time
import urllib.request
import zipfile
from pathlib import Path

from .gpu import player_gpu_args

DEFAULTS = {'timeout_s': 3600, 'crf': 18, 'preset': 'medium', 'preroll_s': 2, 'manifest': 'manifest.json'}


def window_visible(env):
    if sys.platform == 'win32':
        import ctypes
        sid = ctypes.c_ulong()
        ctypes.windll.kernel32.ProcessIdToSessionId(os.getpid(), ctypes.byref(sid))  # type: ignore[attr-defined]
        return sid.value != 0
    return bool(env.get('DISPLAY') or env.get('WAYLAND_DISPLAY'))


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def cached_ffmpeg(url, sha, cache):
    """A pinned ffmpeg build: download once (hash checked), unpack, return the ffmpeg binary."""
    root = Path(cache) / 'ffmpeg' / sha[:16]
    exe = 'ffmpeg.exe' if sys.platform == 'win32' else 'ffmpeg'
    found = list(root.rglob(exe)) if root.is_dir() else []
    if found:
        return found[0]
    root.mkdir(parents=True, exist_ok=True)
    archive = root / Path(url.split('?')[0]).name
    with urllib.request.urlopen(url, timeout=600) as r, open(archive, 'wb') as f:
        shutil.copyfileobj(r, f)
    if sha256_file(archive) != sha:
        archive.unlink()
        raise RuntimeError(f'theater: {url} does not match its pinned sha256')
    if archive.suffix == '.zip':
        with zipfile.ZipFile(archive) as z:
            z.extractall(root)
    elif '.tar' in archive.name:
        with tarfile.open(archive) as t:
            t.extractall(root, filter='data')
    else:
        archive.rename(root / exe)
    found = list(root.rglob(exe))
    if not found:
        raise RuntimeError(f'theater: no {exe} in {url}')
    if sys.platform != 'win32':
        found[0].chmod(0o755)
    return found[0]


class ClusterBackend:
    def __init__(self, name='theater-render', options=None):
        self.name, self.options, self.ctx, self.proc = name, dict(options or {}), None, None

    def update(self):
        pass

    # ------------------------------------------------------------------ job

    def _client(self):
        client = getattr(self.ctx, 'client', None)
        if client is not None:
            return client
        from crane_cluster import config
        from crane_cluster.api import Client
        cfg = config.load(self.options.get('worker_config') or (config.config_dir() / 'worker.yaml'))
        return Client(cfg['coordinator'], cfg['token_file'].read_text().strip())

    def fetch_inputs(self, dest):
        dest.mkdir(parents=True, exist_ok=True)
        inputs = self.job.get('inputs') or {}
        if self.job['manifest'] not in inputs:
            raise RuntimeError(f"theater: inputs have no {self.job['manifest']}")
        client = self._client()
        for name, sha in sorted(inputs.items()):
            if '/' in name or '\\' in name or name.startswith('.'):
                raise RuntimeError(f'theater: bad input name {name}')
            data = client.request('GET', f'/v1/inputs/{sha}')
            if isinstance(data, (dict, list)):
                data = json.dumps(data).encode()
            if hashlib.sha256(data).hexdigest() != sha:
                raise RuntimeError(f'theater: input {name} does not match {sha}')
            (dest / name).write_bytes(data)
        return dest

    def prepare(self, ctx):
        self.ctx = ctx
        self.job = {**DEFAULTS, **self.options, **ctx.spec}
        if not self.job.get('chunk_frames'):
            raise RuntimeError('theater: spec.chunk_frames is required')
        self.player = Path(os.path.expandvars(str(self.job.get('player') or ''))).expanduser()
        if not self.player.is_file():
            raise RuntimeError(f'theater: player {self.player} not found')
        self.env = {**os.environ, **getattr(ctx, 'env', {})}
        if sys.platform != 'win32':
            for k in ('DISPLAY', 'WAYLAND_DISPLAY'):
                self.env.pop(k, None)
            if self.job.get('display'):
                self.env['DISPLAY'] = str(self.job['display'])   # batchmode: a GPU context, no window
            else:
                self.env.setdefault('SDL_VIDEODRIVER', 'dummy')
        elif window_visible(self.env):
            # -batchmode never opens a window; a desktop session is still refused, as the unity backend does
            raise RuntimeError('theater: refusing to run the player in a desktop session (run the worker in session 0)')
        self.ffmpeg = self._ffmpeg()
        self.inputs = self.fetch_inputs(Path(ctx.work_dir) / 'input')
        self.manifest = json.loads((self.inputs / self.job['manifest']).read_text())

    def _ffmpeg(self):
        if self.job.get('ffmpeg'):
            p = Path(os.path.expandvars(str(self.job['ffmpeg']))).expanduser()
            if not p.is_file():
                raise RuntimeError(f'theater: ffmpeg {p} not found')
            return p
        if self.job.get('ffmpeg_url') and self.job.get('ffmpeg_sha256'):
            from crane_cluster import config
            return cached_ffmpeg(self.job['ffmpeg_url'], self.job['ffmpeg_sha256'], self.job.get('cache') or config.data_dir())
        found = shutil.which('ffmpeg')
        if not found:
            raise RuntimeError('theater: no ffmpeg (set ffmpeg: or ffmpeg_url + ffmpeg_sha256)')
        return Path(found)

    def frames(self, seed):
        n = int(self.job['chunk_frames'])
        return int(seed) * n, (int(seed) + 1) * n

    def argv(self, seed, run_dir):
        j, a = self.job, [str(self.player)]
        a0, a1 = self.frames(seed)
        world_args = [str(x).replace('{dir}', str(self.inputs)) for x in (self.manifest.get('world') or {}).get('args') or []]
        a += ['-batchmode', '-screen-fullscreen', '0', '-screen-width', '320', '-screen-height', '180',
              '-logFile', str(run_dir / 'player.log'),
              '--crane-disable-ros', '--crane-disable-sitl'] + world_args + [
              '--crane-theater', str(self.inputs / j['manifest']), '--crane-theater-out', str(run_dir),
              '--crane-theater-frames', f'{a0}:{a1}', '--crane-theater-ffmpeg', str(self.ffmpeg),
              '--crane-theater-crf', str(j['crf']), '--crane-theater-preset', str(j['preset']),
              '--crane-theater-preroll-s', str(j['preroll_s'])]
        if j.get('script'):
            a += ['--crane-theater-script', str(self.inputs / j['script'])]
        for key, flag in (('fps', '--crane-theater-fps'), ('size', '--crane-theater-size'), ('quality', '--crane-theater-quality'),
                          ('stills', '--crane-theater-stills'), ('seconds', '--crane-theater-seconds')):
            if j.get(key) is not None:
                a += [flag, str(j[key])]
        extra = [str(x) for x in self.options.get('args') or []] + [str(x) for x in self.ctx.spec.get('args') or []]
        return a + extra + player_gpu_args(extra)   # Windows with an iGPU and a dGPU: the dGPU (gpu.py)

    def run(self, seed):
        run_dir = Path(self.ctx.work_dir) / 'chunks' / f'c{int(seed):04d}'
        run_dir.mkdir(parents=True, exist_ok=True)
        kw = {'cwd': str(self.player.parent), 'env': self.env, 'stdout': subprocess.DEVNULL, 'stderr': subprocess.DEVNULL}
        if sys.platform == 'win32':
            si = subprocess.STARTUPINFO()
            si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            kw.update(startupinfo=si, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
                      | subprocess.BELOW_NORMAL_PRIORITY_CLASS)
        else:
            kw['start_new_session'] = True
        t0 = time.time()
        self.proc = subprocess.Popen(self.argv(seed, run_dir), **kw)
        try:
            rc = self.proc.wait(timeout=float(self.job['timeout_s']))
        except subprocess.TimeoutExpired:
            self.cancel()
            raise RuntimeError(f'chunk {seed}: player timed out after {self.job["timeout_s"]} s')
        finally:
            self.proc = None
        (run_dir / 'player_s.txt').write_text(f'{time.time() - t0:.3f}\n')
        report = run_dir / 'render_report.json'
        if rc != 0 or not report.is_file() or not (run_dir / 'video.mp4').is_file():
            tail = ''
            if (run_dir / 'player.log').is_file():
                tail = (run_dir / 'player.log').read_bytes()[-2000:].decode(errors='replace')
            raise RuntimeError(f'chunk {seed}: player exited {rc}: {tail}')
        self.contact_sheet(run_dir)
        return run_dir

    def contact_sheet(self, run_dir):
        """One frame every 2 s of the chunk, 480 px wide, 4 across."""
        fps = float(json.loads((run_dir / 'render_report.json').read_text()).get('fps') or 60)
        subprocess.run([str(self.ffmpeg), '-hide_banner', '-loglevel', 'error', '-y', '-i', str(run_dir / 'video.mp4'),
                        '-vf', f'select=not(mod(n\\,{max(1, round(fps * 2))})),scale=480:-1,tile=4x4', '-frames:v', '1',
                        '-update', '1', str(run_dir / 'contact.png')],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=300, check=False,
                       **({'creationflags': subprocess.CREATE_NO_WINDOW} if sys.platform == 'win32' else {}))

    def metrics(self, run_dir):
        r = json.loads((Path(run_dir) / 'render_report.json').read_text())
        m = {'ok': float(bool(r.get('ok'))), 'frames_written': float(r.get('frames_written', 0)), 'errors': float(r.get('errors', 0)),
             'render_ms_median': float(r.get('render_ms_median', 0)), 'render_ms_p95': float(r.get('render_ms_p95', 0)),
             'wall_s': float(r.get('wall_s', 0)), 'player_s': float((Path(run_dir) / 'player_s.txt').read_text()),
             'video_mb': (Path(run_dir) / 'video.mp4').stat().st_size / 2 ** 20}
        for k, v in (r.get('checks') or {}).items():
            if isinstance(v, (int, float)):
                m[f'checks.{k}'] = float(v)
        return m

    def artifacts(self, run_dir):
        run_dir = Path(run_dir)
        out = [(run_dir / n, n, 'summary') for n in ('video.mp4', 'frames.jsonl', 'render_report.json', 'contact.png')
               if (run_dir / n).is_file()]
        if (run_dir / 'player.log').is_file():
            out.append((run_dir / 'player.log', 'player.log', 'raw'))
        stills = run_dir / 'stills'
        if stills.is_dir():
            out += [(p, f'stills/{p.name}', 'raw') for p in sorted(stills.glob('*.png'))]
        return out

    def cancel(self):
        p = self.proc
        if not p or p.poll() is not None:
            return
        try:
            if sys.platform == 'win32':
                subprocess.run(['taskkill', '/T', '/F', '/PID', str(p.pid)], capture_output=True, check=False,
                               creationflags=subprocess.CREATE_NO_WINDOW)
            else:
                os.killpg(p.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass

    def finish(self):
        self.cancel()
