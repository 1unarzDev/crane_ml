#!/bin/bash
cd ~/worktrees/crane-forest-sar/Tools
PY=~/.cache/research/fs-venv/bin/python
for N in 2 3 5 10; do
  E=40; [ $N -eq 10 ] && E=24
  $PY -m ForestSearch.learned_comm labels --Ns $N --split train --E $E --krec 2 --kclaim 2 --kbeacon 6 --out /tmp/lc/train_N$N.npz
  $PY -m ForestSearch.learned_comm labels --Ns $N --split val --E 10 --e0 1000 --krec 2 --kclaim 2 --kbeacon 6 --out /tmp/lc/val_N$N.npz
done
