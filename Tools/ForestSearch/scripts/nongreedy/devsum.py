import csv, collections
d=collections.defaultdict(list)
for r in csv.DictReader(open('/tmp/fs_nongreedy/dev.csv')): d[(r['grid'],r['seed'],r['planner'])].append((int(r['detected']),float(r['sec'])))
for k,v in d.items(): print(k, len(v), round(sum(a for a,_ in v)/len(v),3), round(sum(b for _,b in v)/len(v),3))
