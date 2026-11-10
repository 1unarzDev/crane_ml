# SPDX-License-Identifier: GPL-3.0-or-later
"""End-to-end detector latency on the local GPU at onboard resolution, idle vs under an emulated navigation load.

Latency = numpy 1920x1080 RGB frame in host memory -> person boxes in host memory (H2D copy, tiling/letterbox,
RT-DETR-R18 forward, score filter, NMS, D2H). Configs: mode {full, tiled} x precision {fp32, fp16} x load {idle, nav}.

Navigation-load proxy (runs concurrently in separate processes):
  * CPU: NCPU processes, each voxelising a fresh 120k-point cloud at 0.10 m (floor, hash, np.unique) in a busy loop,
    the cost profile of a stereo/LiDAR occupancy update. NCPU = 4 (Orin Nano has 6 A78AE cores).
  * GPU: one process voxelising 300k points into a 256^3 occupancy grid (scatter, 3x3x3 max-pool dilation)
    at 20 Hz, the shape of a GPU ESDF/occupancy map update. It shares the GPU with the detector.

Usage: python -I latency.py DATA_DIR OUT_JSON [--n 500] [--ncpu 4]
"""
import argparse, json, multiprocessing as mp, os, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def cpu_load(stop, counter):
    rng = np.random.default_rng(os.getpid())
    while not stop.is_set():
        pts = rng.normal(0, 8, (120_000, 3)).astype(np.float32)
        v = np.floor(pts / 0.10).astype(np.int64)
        key = (v[:, 0] + 1024) * 4_194_304 + (v[:, 1] + 1024) * 2048 + (v[:, 2] + 1024)
        np.unique(key)
        with counter.get_lock():
            counter.value += 1


def gpu_load(stop, counter):
    import torch
    dev = torch.device('cuda'); G = 256
    grid = torch.zeros(G * G * G, device=dev)
    while not stop.is_set():
        t = time.perf_counter()
        pts = torch.randn(300_000, 3, device=dev) * 40 + G / 2
        idx = pts.long().clamp_(0, G - 1)
        flat = (idx[:, 0] * G + idx[:, 1]) * G + idx[:, 2]
        grid.zero_(); grid.index_add_(0, flat, torch.ones_like(flat, dtype=torch.float32))
        occ = torch.nn.functional.max_pool3d(grid.view(1, 1, G, G, G), 3, 1, 1)
        torch.cuda.synchronize(); _ = float(occ.sum())
        with counter.get_lock():
            counter.value += 1
        time.sleep(max(0.0, 0.05 - (time.perf_counter() - t)))


def gpu_util_sampler(stop, out):
    """Sample whole-GPU utilisation (all processes, incl. other sessions' Unity renders) every 0.5 s."""
    import subprocess
    while not stop.is_set():
        try:
            out.append(float(subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                                            capture_output=True, text=True, timeout=5).stdout.split()[0]))
        except Exception:
            pass
        stop.wait(0.5)


def forward_only(det, batch, n=200):
    """GPU time of the model forward alone (CUDA events), no pre/post-processing or host transfers."""
    import torch
    x = torch.rand(batch, 3, 640, 640, device=det.device)
    if det.half:
        x = x.half()
    with torch.inference_mode():
        for _ in range(20):
            det.model(pixel_values=x)
        ts = []
        for _ in range(n):
            a, b = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
            a.record(); det.model(pixel_values=x); b.record(); torch.cuda.synchronize()
            ts.append(a.elapsed_time(b) / 1e3)
    return ts


def pct(x):
    x = np.asarray(x) * 1e3
    return dict(n=len(x), mean=float(x.mean()), p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                p99=float(np.percentile(x, 99)), max=float(x.max()))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('data'); ap.add_argument('out')
    ap.add_argument('--n', type=int, default=500); ap.add_argument('--ncpu', type=int, default=4)
    a = ap.parse_args()
    import cv2, torch
    from detector import Detector
    files = sorted((Path(a.data) / 'images').glob('*.jpg'))[::max(1, len(list((Path(a.data) / 'images').glob('*.jpg'))) // 40)][:40]
    frames = [cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2RGB) for f in files]
    results = {}
    for half in (False, True):
        det = Detector(half=half)
        for load in ('idle', 'nav'):
            procs, stop = [], mp.Event()
            cnt_cpu, cnt_gpu = mp.Value('l', 0), mp.Value('l', 0)
            if load == 'nav':
                ctx = mp.get_context('spawn')
                stop = ctx.Event(); cnt_cpu, cnt_gpu = ctx.Value('l', 0), ctx.Value('l', 0)
                procs = [ctx.Process(target=cpu_load, args=(stop, cnt_cpu)) for _ in range(a.ncpu)]
                procs.append(ctx.Process(target=gpu_load, args=(stop, cnt_gpu)))
                for p in procs: p.start()
                time.sleep(8.0)  # let the GPU loader initialise CUDA
            for mode in ('full', 'tiled'):
                for i in range(30):
                    det.detect(frames[i % len(frames)], mode=mode)
                import threading
                ustop, util = threading.Event(), []
                th = threading.Thread(target=gpu_util_sampler, args=(ustop, util), daemon=True); th.start()
                c0, g0, T0 = cnt_cpu.value, cnt_gpu.value, time.perf_counter()
                lat = []
                for i in range(a.n):
                    t = time.perf_counter()
                    det.detect(frames[i % len(frames)], mode=mode)
                    lat.append(time.perf_counter() - t)
                dt = time.perf_counter() - T0
                ustop.set(); th.join()
                key = f"{mode}|{'fp16' if half else 'fp32'}|{load}"
                results[key] = pct(lat) | dict(nav_cpu_voxel_hz=(cnt_cpu.value - c0) / dt, nav_gpu_voxel_hz=(cnt_gpu.value - g0) / dt,
                                               gpu_util_mean=float(np.mean(util)) if util else float('nan'))
                fkey = f"forward_b{1 if mode == 'full' else 6}|{'fp16' if half else 'fp32'}|{load}"
                results[fkey] = pct(forward_only(det, 1 if mode == 'full' else 6))
                print(key, {k: round(v, 2) for k, v in results[key].items()}, flush=True)
                print(fkey, {k: round(v, 2) for k, v in results[fkey].items()}, flush=True)
            stop.set()
            for p in procs: p.join(10)
        del det; torch.cuda.empty_cache()
    meta = dict(gpu=torch.cuda.get_device_name(0), torch=torch.__version__, n=a.n, ncpu_load=a.ncpu,
                frame='1920x1080 RGB uint8 host', note='end-to-end host->boxes; PyTorch eager, no TensorRT; forward_b* = model forward only (CUDA events); the GPU is shared with other sessions (gpu_util_mean includes them)')
    json.dump(dict(meta=meta, results=results), open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
