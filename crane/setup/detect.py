"""What this machine has, for crane setup: read-only probes, each bounded (a few seconds at most)."""

import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

from crane import paths
from crane.resources import cpus


def _run(argv, timeout=5):
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return proc.stdout.strip() if proc.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''


def os_name():
    try:
        for line in Path('/etc/os-release').read_text().splitlines():
            if line.startswith('PRETTY_NAME='):
                return line.split('=', 1)[1].strip('"')
    except OSError:
        pass
    return f'{platform.system()} {platform.release()}'


def memory_gb():
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemTotal:'):
                return int(line.split()[1]) / 1e6
    except (OSError, ValueError):
        pass
    return None


def gpu():
    """'NAME, driver X' of the first NVIDIA GPU, or ''."""
    if not shutil.which('nvidia-smi'):
        return ''
    first = _run(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader']).splitlines()[:1]
    if not first:
        return ''
    name, _, driver = first[0].partition(',')
    return f'{name.strip()}, driver {driver.strip()}'


def docker():
    if not shutil.which('docker'):
        return ''
    return _run(['docker', 'version', '--format', '{{.Server.Version}}']) or 'client only (no daemon)'


def unity_editor():
    try:
        from crane.build import editor_path
        editor = editor_path()
    except (OSError, AttributeError):
        return ''
    return str(editor) if editor.is_file() else ''


def display():
    return os.environ.get('WAYLAND_DISPLAY') and f'wayland {os.environ["WAYLAND_DISPLAY"]}' or \
        os.environ.get('DISPLAY') and f'x11 {os.environ["DISPLAY"]}' or ''


def machine():
    """{name: value} of everything setup's defaults read ('' when absent)."""
    free = shutil.disk_usage(paths.HOME if paths.HOME.exists() else '/').free
    mem = memory_gb()
    return {
        'os': os_name(),
        'python': f'{platform.python_version()} at {sys.executable}',
        'cores': os.cpu_count() or 1,
        'cpus': cpus(),
        'memory': f'{mem:.0f} GB' if mem else '',
        'disk': f'{free / 1e9:.0f} GB free at {paths.HOME}',
        'free_bytes': free,
        'gpu': gpu(),
        'docker': docker(),
        'astro_dock': str(paths.ASTRO_DOCK) if (paths.ASTRO_DOCK / 'profiles').is_dir() else '',
        'unity': unity_editor(),
        'display': display(),
        'checkout': str(paths.ROOT) if (paths.ROOT / '.git').exists() else '',
    }
