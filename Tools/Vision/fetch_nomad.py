"""Fetch a stratified, location-split subset of NOMAD (CC BY 4.0) frames and downscale them to the onboard RGB size.

NOMAD: Russell Bernal, Scheirer, Cleland-Huang, WACV 2024. https://github.com/ArtRuss/NOMAD (CC BY 4.0).
The full release is 5.4K frames in 26 GB zips per ten actors; we fetch single files from the public Drive folder,
resize each to 1920x1080 (the CRANE OAK-D RGB stream size, RenderProfiles.OakD) and drop the original.

Split is by LOCATION (scene) so no actor, flight or site is shared between fit and held-out:
  fit      : Paintball, School, Farm, Golf Course
  held-out : Forest, Lake, School (Nature), Quarry, Field
Frames: per actor-distance folder, RUNS consecutive-annotation runs of RUN_LEN frames (gap <= 12 video frames),
spread over the routine, so repeat-view correlation can be measured.

Usage: python -I fetch_nomad.py ANN_DIR OUT_DIR [--per-loc N] [--jobs J]
"""
import argparse, json, re, sys, time, urllib.request, io, os, random
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT_IMAGES = '14oxtWhFOoRRHIKBYFMSU945cTeUrT2Zr'
FIT_LOCS = ['Paintball', 'School', 'Farm', 'Golf Course']
HELD_LOCS = ['Forest', 'Lake', 'School (Nature)', 'Quarry', 'Field']
RUNS, RUN_LEN, MAX_GAP = 3, 4, 12
OUT_W, OUT_H = 1920, 1080


def ls(folder_id):
    for attempt in range(4):
        try:
            h = urllib.request.urlopen(f'https://drive.google.com/embeddedfolderview?id={folder_id}', timeout=60).read().decode()
            return {m.group(2): m.group(1) for m in re.finditer(r'id="entry-([^"]+)".*?flip-entry-title">([^<]+)<', h, re.S)}
        except Exception as e:
            time.sleep(3 * (attempt + 1)); err = e
    raise err


def pick_frames(names):
    fr = sorted(int(re.search(r'_f(\d+)\.jpg', n).group(1)) for n in names)
    runs, cur = [], [fr[0]]
    for a, b in zip(fr, fr[1:]):
        if b - a <= MAX_GAP: cur.append(b)
        else: runs.append(cur); cur = [b]
    runs.append(cur)
    runs = [r for r in runs if len(r) >= RUN_LEN]
    if not runs: return []
    idx = sorted(set(int(round(i)) for i in [k * (len(runs) - 1) / max(1, RUNS - 1) for k in range(RUNS)]))
    out = []
    for i in idx:
        r = runs[i]; s = (len(r) - RUN_LEN) // 2
        out += r[s:s + RUN_LEN]
    return out


def fetch(fid, dst):
    from PIL import Image
    if dst.exists(): return 'cached'
    for attempt in range(4):
        try:
            data = urllib.request.urlopen(f'https://drive.usercontent.google.com/download?id={fid}&export=download', timeout=120).read()
            im = Image.open(io.BytesIO(data)); im.load()
            if im.size != (5472, 3078): return f'unexpected size {im.size}'
            im.convert('RGB').resize((OUT_W, OUT_H), Image.LANCZOS).save(dst.with_suffix('.tmp.jpg'), quality=95)
            os.replace(dst.with_suffix('.tmp.jpg'), dst)
            return 'ok'
        except Exception as e:
            time.sleep(4 * (attempt + 1)); err = e
    return f'fail {err}'


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('ann'); ap.add_argument('out')
    ap.add_argument('--per-loc', type=int, default=3); ap.add_argument('--jobs', type=int, default=6)
    a = ap.parse_args(); ann, out = Path(a.ann), Path(a.out); (out / 'images').mkdir(parents=True, exist_ok=True)
    meta = {m['id']: m for m in json.load(open(ann / 'metadata.json'))}
    rng = random.Random(20261109)
    chosen = []
    for split, locs in (('fit', FIT_LOCS), ('heldout', HELD_LOCS)):
        for loc in locs:
            ids = sorted(i for i, m in meta.items() if m['environmental']['location'] == loc)
            # prefer a mix of weather: sort cloudy first then sunny, pick spread
            rng.shuffle(ids)
            n = a.per_loc if split == 'fit' else max(a.per_loc, 2)
            chosen += [(split, loc, i) for i in sorted(ids[:n])]
    actors = ls(ROOT_IMAGES)
    jobs, manifest = [], []
    for split, loc, i in chosen:
        name = f'Actor{i:03d}'
        sub = ls(actors[name])
        for folder, fid in sorted(sub.items()):
            files = ls(fid)
            for f in pick_frames(list(files)):
                fn = f'{folder}_f{f:04d}.jpg'
                if fn not in files: continue
                jobs.append((files[fn], out / 'images' / fn))
                manifest.append({'file': fn, 'actor': i, 'location': loc, 'split': split, 'folder': folder})
        print(f'listed {name} {loc} {split}: {len(jobs)} frames so far', flush=True)
    json.dump(manifest, open(out / 'manifest.json', 'w'), indent=0)
    t0 = time.time(); done = 0
    with ThreadPoolExecutor(a.jobs) as ex:
        for r in ex.map(lambda j: fetch(*j), jobs):
            done += 1
            if r not in ('ok', 'cached'): print('WARN', r, flush=True)
            if done % 50 == 0: print(f'{done}/{len(jobs)} {time.time() - t0:.0f}s', flush=True)
    print('DONE', done, f'{time.time() - t0:.0f}s')


if __name__ == '__main__':
    main()
