#!/usr/bin/env python3
"""Seeded task endpoints and tightly bounded cart placement; retain resolved manifest."""
import argparse, hashlib, json, math, pathlib, random, subprocess, sys
from check_topology import blocked,connectivity

ROOT = pathlib.Path(__file__).resolve().parents[2]

STATIONS={
    'receiving':((-12.72,-12.58),(5.1,5.8)),
    'spares':((-12.48,-12.32),(22.2,23.5)),
    'packing':((-11.45,-11.2),(10.2,10.8)),
    'inspection':((-14.2,-14.0),(22.4,23.3)),
}
TASKS={
    'dispatch_forward':('receiving','spares'),
    'return_to_receiving':('spares','receiving'),
    'maintenance_delivery':('packing','spares'),
    'maintenance_return':('spares','packing'),
    'inspection_return':('inspection','receiving'),
}

def resolve(seed, scenario, task='dispatch_forward', parking_variation=False):
    if seed<=0:raise ValueError('seed must be positive')
    if task=='auto':task=random.Random(seed ^ 0xCA7).choice(list(TASKS))
    if task not in TASKS:raise ValueError('Unknown workshop task: '+task)
    manifest = json.loads((ROOT / 'Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json').read_text())
    trial = next(s for s in manifest['scenarios'] if s['id'] == scenario)
    rng = random.Random(seed)
    trial['seed'] = seed
    start_station,goal_station=TASKS[task]
    sample=lambda station:[rng.uniform(*interval) for interval in STATIONS[station]]
    trial['start']=sample(start_station);trial['goal']=sample(goal_station)
    heading=(0 if task=='dispatch_forward' else math.degrees(math.atan2(trial['goal'][0]-trial['start'][0],trial['goal'][1]-trial['start'][1])))
    trial['startYaw'] = heading+rng.uniform(-4, 4)
    trial['goalYaw']=heading
    trial['routeName']=scenario+':'+task
    trial['approximateLengthMeters']=round(math.dist(trial['start'],trial['goal'])+2.5,1)
    trial['expectedChallenge']+=' Task: '+start_station+' -> '+goal_station+'.'
    trial['evidenceAnnotations']+=['task:'+task,'station:'+start_station,'station:'+goal_station]
    for obstacle in trial['obstacles']:
        if obstacle['id'] == 'annex-tote-cart':
            obstacle['center'][2] += rng.uniform(-.12, .12)
            if parking_variation:obstacle['center'][0]+=rng.uniform(-.04,.04)
        if parking_variation and obstacle['id']=='annex-tool-cart':
            obstacle['center'][0]+=rng.uniform(-.025,.025)
            obstacle['center'][2]+=rng.uniform(-.08,.08)
    boxes=manifest['boxes']+trial['obstacles']
    if any(blocked(*trial[key],boxes) for key in ['start','goal']):
        raise ValueError('Task endpoint overlaps authoritative footprint geometry')
    if math.dist(trial['start'],trial['goal'])<8:raise ValueError('Task must span at least eight meters')
    return manifest

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--scenario', default='warehouse_compact_switchbacks', choices=['warehouse_compact_switchbacks', 'warehouse_compact_no_path'])
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--task',choices=['auto',*TASKS],default='dispatch_forward')
    parser.add_argument('--parking-variation',action='store_true',help='Bounded trolley parking offsets; fixed equipment and passage widths remain authoritative.')
    args = parser.parse_args()
    if args.seed <= 0:
        parser.error('seed must be positive')
    manifest = resolve(args.seed, args.scenario,args.task,args.parking_variation)
    trial=next(s for s in manifest['scenarios'] if s['id']==args.scenario)
    topology=connectivity(manifest,trial)
    if not topology['passCheck'] or topology['startBlocked'] or topology['goalBlocked']:
        parser.error('Generated task does not match its expected geometry outcome')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + '\n')
    subprocess.run([sys.executable,str(ROOT/'Tools/Campus/validate_campus.py'),'--manifest',str(args.output)],check=True,capture_output=True)
    validation=dict(schema='crane-annex-generated-task-v1',seed=args.seed,routeName=trial['routeName'],parkingVariation=args.parking_variation,
                    manifestSha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),topology=topology,
                    boundary='Conservative planar connectivity is a prerequisite, not a Nav2 or contact success claim.')
    args.output.with_suffix('.validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    print(args.output.resolve())

if __name__ == '__main__':
    main()
