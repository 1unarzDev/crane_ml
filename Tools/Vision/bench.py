# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the onboard detector over the NOMAD location-split subset under controlled conditions; write raw records.

For every image x condition (mode, blur length) the detector's person boxes (score >= 0.05) are kept, and
every ground-truth person gets its best-matching score at two match criteria:
  iou50 : IoU >= 0.5 (COCO AP50 convention)
  loc   : IoU >= 0.25 OR detection centre inside the GT box (the SAR-relevant "alert lands on the person")
Outputs (OUT_DIR): gt.csv (one row per GT x condition), dets.npz (per image x condition detections + match flags),
run.json (versions, conditions, timing). Analysis is in analyze.py; fitting in fit_obs.py.

Synthetic motion blur: a normalised horizontal line kernel of length L px applied to the 1920x1080 frame
(L = f_px * v_rel * t_exp / range + f_px * yaw_rate * t_exp, see obs_model.blur_px).

Usage: python -I bench.py DATA_DIR ANN_DIR OUT_DIR [--blur 0,3,6,12,24] [--limit N]
"""
import argparse, csv, json, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detector import Detector  # noqa: E402

SCALE = 1920 / 5472.0


SKY = {'sunny': 'sunny', 'passing clouds': 'partcloud', 'scattered clouds': 'partcloud', 'partly sunny': 'partcloud',
       'thundershowers. sunny': 'partcloud', 'overcast': 'overcast', 'broken clouds': 'overcast',
       'mostly cloudy': 'overcast', 'fog': 'overcast', 'thunderstorms. partly sunny': 'overcast'}


def lighting(meta):
    """(sky class, sun class) from NOMAD metadata: weather descriptor and local hour (low sun before 10 or from 17)."""
    sky = SKY.get(meta['environmental']['weather']['descriptor'].strip().lower(), 'overcast')
    hour = int(meta['environmental']['time[HH]'])
    return sky, ('low_sun' if (hour < 10 or hour >= 17) else 'high_sun')


def blur_kernel(L):
    if L <= 1:
        return None
    k = np.zeros((1, int(L)), np.float32); k[:] = 1.0 / int(L)
    return k


def iou(a, b):
    """a: Nx4, b: Mx4 xyxy -> NxM."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0]); y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2]); y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    aa = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1]); bb = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / (aa[:, None] + bb[None, :] - inter + 1e-9)


def loc_match(gt, det):
    """NxM bool: IoU >= 0.25 or detection centre inside GT box."""
    if len(gt) == 0 or len(det) == 0:
        return np.zeros((len(gt), len(det)), bool)
    cx = 0.5 * (det[:, 0] + det[:, 2]); cy = 0.5 * (det[:, 1] + det[:, 3])
    inside = (cx[None] >= gt[:, None, 0]) & (cx[None] <= gt[:, None, 2]) & (cy[None] >= gt[:, None, 1]) & (cy[None] <= gt[:, None, 3])
    return inside | (iou(gt, det) >= 0.25)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data'); ap.add_argument('ann'); ap.add_argument('out')
    ap.add_argument('--blur', default='0,3,6,12,24'); ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--full-blur', default='0')
    a = ap.parse_args()
    data, ann, out = Path(a.data), Path(a.ann), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    manifest = json.load(open(data / 'manifest.json'))
    annots = {r['file_name']: r for r in json.load(open(ann / 'annotations.json'))}
    meta = {m['id']: m for m in json.load(open(ann / 'metadata.json'))}
    acts = {x['id']: x['labels'] for x in json.load(open(ann / 'activityLabels.json'))}
    rows = [m for m in manifest if (data / 'images' / m['file']).exists() and m['file'] in annots]
    if a.limit:
        rows = rows[:a.limit]
    conds = [('tiled', int(L)) for L in a.blur.split(',')] + [('full', int(L)) for L in a.full_blur.split(',')]
    det = Detector()
    gt_rows, det_store = [], {}
    t0 = time.time(); n_inf = 0
    for i, m in enumerate(rows):
        img = cv2.cvtColor(cv2.imread(str(data / 'images' / m['file'])), cv2.COLOR_BGR2RGB)
        an = annots[m['file']]
        gts = np.array([[b['bbox'][0], b['bbox'][1], b['bbox'][0] + b['bbox'][2], b['bbox'][1] + b['bbox'][3]]
                        for b in an['annotations']], float).reshape(-1, 4) * SCALE
        vis = [int(b['visibility']) for b in an['annotations']]
        mt = meta[m['actor']]; sky, sun = lighting(mt)
        dist = int(m['folder'].split('_a')[1])
        frame = int(m['file'].rsplit('_f', 1)[1][:4])
        act = 'unknown'
        for k, spans in acts.get(m['actor'], {}).get(str(dist), {}).items():
            if any(int(s) <= frame <= int(e) for s, e in spans):
                act = k
        lum = float(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).mean())
        for mode, L in conds:
            k = blur_kernel(L)
            im = cv2.filter2D(img, -1, k, borderType=cv2.BORDER_REFLECT) if k is not None else img
            boxes, scores = det.detect(im, mode=mode); n_inf += 1
            I = iou(gts, boxes); LM = loc_match(gts, boxes)
            key = f"{m['file']}|{mode}|{L}"
            det_store[key] = dict(boxes=boxes.astype(np.float32), scores=scores.astype(np.float32),
                                  tp50=(I >= 0.5).any(0) if len(gts) else np.zeros(len(scores), bool),
                                  tploc=LM.any(0) if len(gts) else np.zeros(len(scores), bool))
            for g in range(len(gts)):
                s50 = float(scores[I[g] >= 0.5].max()) if (I[g] >= 0.5).any() else 0.0
                sl = float(scores[LM[g]].max()) if LM[g].any() else 0.0
                # centre error of the best loc-matched detection (pixels, 1920 frame)
                if LM[g].any():
                    j = np.where(LM[g])[0][np.argmax(scores[LM[g]])]
                    ex = 0.5 * (boxes[j, 0] + boxes[j, 2]) - 0.5 * (gts[g, 0] + gts[g, 2])
                    ey = 0.5 * (boxes[j, 1] + boxes[j, 3]) - 0.5 * (gts[g, 1] + gts[g, 3])
                else:
                    ex = ey = float('nan')
                gt_rows.append(dict(file=m['file'], actor=m['actor'], location=m['location'], split=m['split'],
                                    folder=m['folder'], frame=frame, dist_m=dist, gt_idx=g, visibility=vis[g],
                                    w_px=gts[g, 2] - gts[g, 0], h_px=gts[g, 3] - gts[g, 1], sky=sky, sun=sun,
                                    hour=int(mt['environmental']['time[HH]']), lum=lum, activity=act,
                                    mode=mode, blur_px=L, score50=s50, scoreloc=sl, ex=ex, ey=ey,
                                    n_gt_img=len(gts)))
        if i % 50 == 0:
            print(f'{i}/{len(rows)} {time.time() - t0:.0f}s', flush=True)
    with open(out / 'gt.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(gt_rows[0])); w.writeheader(); w.writerows(gt_rows)
    np.savez_compressed(out / 'dets.npz', **{k + '|' + f: v[f] for k, v in det_store.items() for f in v})
    import torch, transformers
    json.dump(dict(n_images=len(rows), conditions=conds, inferences=n_inf, wall_s=time.time() - t0,
                   torch=torch.__version__, transformers=transformers.__version__,
                   model='PekingU/rtdetr_r18vd@ac77a11f (Apache-2.0)', dataset='NOMAD (CC BY 4.0) location-split subset',
                   gpu=torch.cuda.get_device_name(0)), open(out / 'run.json', 'w'), indent=1)
    print('DONE', len(rows), f'{time.time() - t0:.0f}s')


if __name__ == '__main__':
    main()
