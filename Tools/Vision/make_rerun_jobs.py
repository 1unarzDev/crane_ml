# SPDX-License-Identifier: GPL-3.0-or-later
"""Write the rendered-image rerun job spec: variant demo sequences (Tools/Vision/rerun_sequences/*.json) derived from
Assets/Drones/Demo/Sequences/forest_demo_v1.json, and a serial job list (Tools/Vision/rerun_jobs.json) of
DemoCapture invocations through the project's Unity wrapper. Rendering is NOT started here.

Key cases (each crosses the strata the NOMAD benchmark could not cover or that the mission depends on):
  hover_*      : hover at the stand-off facing the person; GoldenHour / Overcast / Dusk (lighting)
  corridor_v*  : corridor approach at 2.0 / 4.5 / 7.0 m/s (motion blur at under-canopy range)
  top_*        : over-canopy look-down at 30 m and 50 m (nadir/steep view, canopy-gap occlusion; NOMAD has none)
  seeds        : 4 layout seeds per case (person placement relative to trunks/understory = occlusion variety)
  empty        : the same flights with the person removed (person_offset far outside the stand): false alerts

Usage: python -I make_rerun_jobs.py [--repo ROOT]
"""
import argparse, copy, json
from pathlib import Path

WRAPPER = '~/.cache/research/fs-scripts/unity-drones'
METHOD = 'Crane.Theater.Aerial.DemoCapture.Run'


def shot(base, sid):
    for s in base['shots']:
        if s['id'] == sid:
            return copy.deepcopy(s)
    raise KeyError(sid)


def onboard_only(seq, shots, title):
    s = copy.deepcopy(seq); s['title'] = title; s['shots'] = shots
    for sh in s['shots']:
        sh['detection'] = False; sh.pop('title', None); sh.pop('lower_third', None)
        sh['transition'] = {'type': 'cut', 'duration_s': 0}; sh['stills'] = [0.5]
    return s


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--repo', default=str(Path(__file__).resolve().parents[2]))
    a = ap.parse_args(); repo = Path(a.repo)
    base = json.load(open(repo / 'Assets/Drones/Demo/Sequences/forest_demo_v1.json'))
    outdir = repo / 'Tools/Vision/rerun_sequences'; outdir.mkdir(parents=True, exist_ok=True)
    cases = {}
    for light in ('GoldenHour', 'Overcast', 'Dusk'):
        s = onboard_only(base, [dict(shot(base, 's5_orbit'), duration_s=8, time_scale=1.0)], f'vision rerun hover {light}')
        s['light_preset'] = light; cases[f'hover_{light.lower()}'] = s
    for v in (2.0, 4.5, 7.0):
        s = onboard_only(base, [dict(shot(base, 's3_chase'), duration_s=10)], f'vision rerun corridor {v} m/s')
        s['flight']['cruise_ms'] = v; cases[f'corridor_v{str(v).replace(".", "p")}'] = s
    for alt in (30, 50):
        s = onboard_only(base, [dict(shot(base, 's1_establishing'), duration_s=6)], f'vision rerun top {alt} m')
        s['flight']['top_altitude_m'] = alt; cases[f'top_{alt}m'] = s
    for k in list(cases):
        e = copy.deepcopy(cases[k]); e['person_offset'] = [0, -500, 0]; e['title'] += ' (no person)'
        cases[k + '_empty'] = e
    jobs = []
    for name, s in sorted(cases.items()):
        p = outdir / f'{name}.json'
        p.write_text(json.dumps(s, indent=1))
        for seed in (7, 11, 13, 17):
            out = f'/tmp/crane_vision_rerun/{name}_s{seed}'
            jobs.append(dict(id=f'{name}_s{seed}', sequence=str(p.relative_to(repo)), seed=seed, out=out,
                             cmd=f'{WRAPPER} --render {out}/editor.log -executeMethod {METHOD} '
                                 f'-demoSequence {p.relative_to(repo)} -demoSeed {seed} -demoProfile onboard -demoNoStills '
                                 f'-demoOut {out} -demoRunId {name}_s{seed} -demoCrf 14'))
    spec = dict(schema='crane-vision-rerun/1',
                purpose='Feed CRANE onboard renders (OAK-D S2 RGB, onboard volume profile) to the benchmarked detector and '
                        'compare observed detection with Tools/Vision/obs_model.py predictions per key case.',
                detector='RT-DETR-R18 tiled 6x640 (Tools/Vision/detector.py), tau 0.3, loc criterion',
                prerequisite=('DemoSession must emit per-frame onboard ground truth in frames.jsonl: person_box_onboard '
                              '[x0,y0,x1,y1] px top-left origin from DemoDirector.PersonBox(rgbCam) (null when off-screen), '
                              'person_in_view (bool), person_occluded (bool, existing raycast flag), person_visible_frac '
                              '(fraction of N sight rays from the camera to points on the person that are unblocked), '
                              'person_range_m (camera to person centre), cam_yaw_rate (rad/s). Owner: the demo/Theater agent '
                              '(Crane.Theater.Aerial asmdef); without it only false alerts on *_empty runs can be scored.'),
                run_rules='One job at a time through the unity-drones wrapper (project lock, one heavy slot). No retry loops.',
                score_cmd='~/.cache/research/vision-venv/bin/python -I Tools/Vision/rendered_rerun.py '
                          'Tools/Vision/results/rendered/<case> /tmp/crane_vision_rerun/<case>_s*',
                compression_note='Encode at crf 14 so H.264 artefacts do not dominate small-person appearance; a PNG '
                                 'frame dump (-demoFrames slices + stills) is the reference if crf 14 still differs.',
                jobs=jobs)
    (repo / 'Tools/Vision/rerun_jobs.json').write_text(json.dumps(spec, indent=1))
    print(f'{len(cases)} sequences, {len(jobs)} jobs')


if __name__ == '__main__':
    main()
