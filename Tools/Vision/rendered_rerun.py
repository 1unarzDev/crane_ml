# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the onboard detector on CRANE-rendered onboard RGB (DemoSession output) and compare with obs_model.

Input: one or more demo output directories written by Crane.Theater.Aerial.DemoCapture (-demoProfile onboard|both):
  onboard_oakd_rgb_1080p30.mp4   1920x1080 OAK-D S2 RGB stream (onboard volume profile, no cinematic post)
  frames.jsonl                    one row per rendered frame: frame, shot, t, drone [x,y,z] (Unity m), cam
  optional per-row ground truth   person_box_onboard [x0,y0,x1,y1] (px, top-left origin), person_occluded (bool),
                                  person_visible_frac (0..1), person_range_m, cam_yaw_rate (rad/s)
                                  -- NOT yet emitted by DemoSession; see Docs/DroneVisuals/mission/vision_rerun_jobs.json
Frames without ground truth are scored only for false alerts (detections >= tau on a person-free frame count as
false alerts only when the row says person_box_onboard is null AND person_in_view is false; unknown rows are
reported as 'detections, GT unknown').

Usage: python -I rendered_rerun.py OUT_DIR DEMO_DIR [DEMO_DIR ...] [--tau 0.3] [--mode tiled]
"""
import argparse, json, math, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def read_frames(demo):
    rows = [json.loads(l) for l in open(demo / 'frames.jsonl') if l.strip()]
    import cv2
    cap = cv2.VideoCapture(str(demo / 'onboard_oakd_rgb_1080p30.mp4'))
    imgs = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        imgs.append(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
    return rows, imgs


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('demos', nargs='+')
    ap.add_argument('--tau', type=float, default=0.3); ap.add_argument('--mode', default='tiled')
    a = ap.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    from detector import Detector
    from bench import loc_match
    import obs_model as om
    det = Detector(); M = om.ObsModel()
    recs = []
    for d in map(Path, a.demos):
        rows, imgs = read_frames(d)
        n = min(len(rows), len(imgs))
        if len(rows) != len(imgs):
            print(f'WARN {d}: {len(rows)} rows vs {len(imgs)} video frames; using first {n}')
        for i in range(n):
            r = rows[i]; boxes, scores = det.detect(imgs[i], mode=a.mode)
            k = scores >= a.tau
            rec = dict(demo=d.name, frame=r.get('frame'), shot=r.get('shot'), n_det=int(k.sum()),
                       max_score=float(scores.max()) if len(scores) else 0.0,
                       boxes=[[round(float(v), 1) for v in b] + [round(float(sc), 3)] for b, sc in zip(boxes[k], scores[k])])
            gt = r.get('person_box_onboard')
            if gt is not None:
                g = np.array([gt], float)
                LM = loc_match(g, boxes[k]) if k.any() else np.zeros((1, 0), bool)
                h = gt[3] - gt[1]
                # blur from drone motion between frames (translation parallax at person range + yaw rate)
                speed = 0.0
                if i > 0 and 'drone' in r and 'drone' in rows[i - 1]:
                    dt = max(r['t'] - rows[i - 1]['t'], 1e-3)
                    speed = float(np.linalg.norm(np.subtract(r['drone'], rows[i - 1]['drone']))) / dt
                rng_m = r.get('person_range_m', float('nan'))
                b = om.blur_px(speed, rng_m, M.exposure_s, r.get('cam_yaw_rate', 0.0)) if math.isfinite(rng_m) else 0.0
                vis = r.get('person_visible_frac', 0.3 if r.get('person_occluded') else 1.0)
                rec.update(gt=True, h_px=h, visible=vis, blur_px=b, detected=bool(LM.any()),
                           n_fp=int(k.sum() - LM.any(0).sum()) if k.any() else 0, p_model=M.p_detect(h, vis, 45.0, b))
            else:
                rec.update(gt=bool(r.get('person_in_view') is False), detected=None,
                           n_fp=int(k.sum()) if r.get('person_in_view') is False else None)
            recs.append(rec)
    with open(out / 'rendered_records.jsonl', 'w') as f:
        for r in recs:
            f.write(json.dumps(r) + '\n')
    g = [r for r in recs if r.get('detected') is not None]
    summ = dict(frames=len(recs), frames_with_gt=len(g), tau=a.tau, mode=a.mode,
                detections_per_frame_all=float(np.mean([r['n_det'] for r in recs])) if recs else float('nan'),
                frames_with_any_detection=float(np.mean([r['n_det'] > 0 for r in recs])) if recs else float('nan'))
    if g:
        summ.update(observed_rate=float(np.mean([r['detected'] for r in g])), predicted_rate=float(np.mean([r['p_model'] for r in g])),
                    fp_per_frame=float(np.mean([r['n_fp'] for r in g])))
        bins = [(0, 16), (16, 32), (32, 64), (64, 128), (128, 1e9)]
        summ['by_height'] = {f'{lo}-{hi:g}': dict(n=len(s), obs=float(np.mean([r['detected'] for r in s])),
                                                  pred=float(np.mean([r['p_model'] for r in s])))
                             for lo, hi in bins if (s := [r for r in g if lo <= r['h_px'] < hi])}
    json.dump(summ, open(out / 'rendered_summary.json', 'w'), indent=1)
    print(json.dumps(summ, indent=1))


if __name__ == '__main__':
    main()
