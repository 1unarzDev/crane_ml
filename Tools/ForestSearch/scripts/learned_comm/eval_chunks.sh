#!/bin/bash
# DEV evaluation in <=10 min chunks via fsrun-quick (recfeas driver has the long slot).
cd ~/worktrees/crane-forest-sar/Tools
PY=~/.cache/research/fs-venv/bin/python
D=ForestSearch/results/learned_comm_data
Q=~/.cache/research/fs-scripts/fsrun-quick
run() { # N seed0 nseeds
  out=$D/eval_N$1_s$2.csv.gz
  [ -f $out ] && return
  $Q $PY -m ForestSearch.learned_comm eval --models $D/models.pkl --out $out --Ns $1 --seed0 $2 --nseeds $3 --E 20
}
run 2 0 16; run 3 0 16
for s in 0 4 8 12; do run 5 $s 4; done
for s in 0 2 4 6 8 10 12 14; do run 10 $s 2; done
