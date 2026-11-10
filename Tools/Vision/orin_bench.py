# SPDX-License-Identifier: GPL-3.0-or-later
"""Ready-to-run latency benchmark for a real Jetson Orin Nano (not run here: no Jetson on this host).

Measures the same end-to-end path as latency.py (1920x1080 RGB host frame -> person boxes), in 3 configurations
{full 640, tiled 6x640 batch 6} x TensorRT FP16, idle vs under the CPU part of latency.py's navigation-load proxy
(voxelisation processes; run latency.py's gpu_load alongside if torch is installed on the Jetson). It writes p50/p95/p99 and the power mode in its JSON output.

On the Jetson (JetPack 6.2+, which provides TensorRT 10 and trtexec):
  sudo nvpmodel -m 2 && sudo jetson_clocks            # MAXN_SUPER on Orin Nano Super (mode ids: nvpmodel -q --verbose)
  python3 export_onnx_here.py                          # or on the desktop: see EXPORT below, copy rtdetr_r18_b{1,6}.onnx
  /usr/src/tensorrt/bin/trtexec --onnx=rtdetr_r18_b1.onnx --fp16 --saveEngine=rtdetr_r18_b1_fp16.engine
  /usr/src/tensorrt/bin/trtexec --onnx=rtdetr_r18_b6.onnx --fp16 --saveEngine=rtdetr_r18_b6_fp16.engine
  python3 orin_bench.py --frames DIR_OF_1080P_JPGS --out orin_latency.json --ncpu 3
Record `nvpmodel -q`, `jetson_release`, and the tegrastats log (tegrastats --interval 500 --logfile tegra.log &).

EXPORT (run on the desktop venv, needs onnx; static shapes keep TensorRT simple):
  python -I orin_bench.py --export --weights ~/.cache/research/vision-data/weights_rtdetr_r18vd --onnx-dir OUT
"""
import argparse, glob, json, multiprocessing as mp, os, subprocess, sys, time

import numpy as np


def export(weights, outdir):
    import torch
    from transformers import RTDetrForObjectDetection
    m = RTDetrForObjectDetection.from_pretrained(weights, use_safetensors=True).eval()

    class Wrap(torch.nn.Module):
        def __init__(self, m):
            super().__init__(); self.m = m

        def forward(self, x):
            o = self.m(pixel_values=x)
            return o.logits[..., 0].sigmoid(), o.pred_boxes
    os.makedirs(outdir, exist_ok=True)
    for b in (1, 6):
        torch.onnx.export(Wrap(m), torch.rand(b, 3, 640, 640), os.path.join(outdir, f'rtdetr_r18_b{b}.onnx'),
                          input_names=['images'], output_names=['person_scores', 'boxes'], opset_version=17)
        print('exported batch', b)


class TrtRunner:
    """Minimal TensorRT 10 runner with pycuda-free device buffers via cuda-python (cudart)."""

    def __init__(self, engine_path):
        import tensorrt as trt
        from cuda import cudart
        self.cudart = cudart
        rt = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        self.engine = rt.deserialize_cuda_engine(open(engine_path, 'rb').read())
        self.ctx = self.engine.create_execution_context()
        self.bufs, self.host = {}, {}
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(n))
            arr = np.empty(shape, dtype=np.float32)
            err, ptr = cudart.cudaMalloc(arr.nbytes)
            self.bufs[n], self.host[n] = ptr, arr
            self.ctx.set_tensor_address(n, ptr)
        _, self.stream = cudart.cudaStreamCreate()

    def __call__(self, x):
        cu = self.cudart
        cu.cudaMemcpyAsync(self.bufs['images'], x.ctypes.data, x.nbytes, cu.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
        self.ctx.execute_async_v3(self.stream)
        for n in ('person_scores', 'boxes'):
            h = self.host[n]
            cu.cudaMemcpyAsync(h.ctypes.data, self.bufs[n], h.nbytes, cu.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream)
        cu.cudaStreamSynchronize(self.stream)
        return self.host['person_scores'], self.host['boxes']


def preprocess(img, mode):
    import cv2
    if mode == 'full':
        s = 640 / max(img.shape[:2]); r = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        c = np.full((640, 640, 3), 127, np.uint8); c[:r.shape[0], :r.shape[1]] = r
        return (c.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
    tiles = [img[y:y + 640, x:x + 640] for y in (0, 440) for x in (0, 640, 1280)]
    return np.ascontiguousarray(np.stack(tiles).transpose(0, 3, 1, 2).astype(np.float32) / 255.0)


def postprocess(scores, boxes, tau=0.3):
    k = scores >= tau
    return boxes[k], scores[k]   # tile offsets + NMS cost is small next to the engine; add torchvision NMS if available


def cpu_load(stop):
    rng = np.random.default_rng(os.getpid())
    while not stop.is_set():
        p = rng.normal(0, 8, (120_000, 3)).astype(np.float32); v = np.floor(p / 0.1).astype(np.int64)
        np.unique((v[:, 0] + 1024) * 4_194_304 + (v[:, 1] + 1024) * 2048 + (v[:, 2] + 1024))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--export', action='store_true'); ap.add_argument('--weights'); ap.add_argument('--onnx-dir', default='.')
    ap.add_argument('--frames'); ap.add_argument('--out', default='orin_latency.json'); ap.add_argument('--n', type=int, default=500)
    ap.add_argument('--ncpu', type=int, default=3); ap.add_argument('--engine-dir', default='.')
    a = ap.parse_args()
    if a.export:
        return export(a.weights, a.onnx_dir)
    import cv2
    frames = [cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB) for f in sorted(glob.glob(os.path.join(a.frames, '*.jpg')))[:40]]
    frames = [cv2.resize(f, (1920, 1080)) if f.shape[:2] != (1080, 1920) else f for f in frames]
    eng = {'full': TrtRunner(os.path.join(a.engine_dir, 'rtdetr_r18_b1_fp16.engine')),
           'tiled': TrtRunner(os.path.join(a.engine_dir, 'rtdetr_r18_b6_fp16.engine'))}
    out = dict(meta=dict(nvpmodel=subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True).stdout.strip(),
                         n=a.n, ncpu_load=a.ncpu, frame='1920x1080 RGB uint8 host', engine='TensorRT FP16'), results={})
    for load in ('idle', 'nav'):
        stop = mp.Event(); procs = []
        if load == 'nav':
            procs = [mp.Process(target=cpu_load, args=(stop,)) for _ in range(a.ncpu)]
            for p in procs: p.start()
            time.sleep(3)
        for mode in ('full', 'tiled'):
            for i in range(30):
                postprocess(*eng[mode](preprocess(frames[i % len(frames)], mode)))
            lat = []
            for i in range(a.n):
                t = time.perf_counter(); postprocess(*eng[mode](preprocess(frames[i % len(frames)], mode))); lat.append(time.perf_counter() - t)
            x = np.array(lat) * 1e3
            out['results'][f'{mode}|fp16|{load}'] = dict(p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                                                        p99=float(np.percentile(x, 99)), mean=float(x.mean()))
            print(mode, load, out['results'][f'{mode}|fp16|{load}'], flush=True)
        stop.set()
        for p in procs: p.join(5)
    json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
