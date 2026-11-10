#!/bin/bash
# One shard of the full pipeline (call under fsrun). Usage: run_all.sh SHARD NSHARDS SPLIT
cd "$(dirname "$0")"
PY=/home/lunarz/.cache/research/fs-venv/bin/python
S=$1; N=$2; SPLIT=${3:-val}
if [ "$S" = "0" ]; then $PY -m unittest discover -s tests -p 'test_*.py' > results/tests.txt 2>&1; fi
$PY experiments.py run --exp E1forced --split $SPLIT --layouts ${L1:-12} --shard $S --nshards $N
$PY experiments.py run --exp E1nat --split $SPLIT --layouts ${L2:-12} --shard $S --nshards $N
$PY experiments.py run --exp E3 --split $SPLIT --layouts ${L3:-12} --shard $S --nshards $N
$PY experiments.py run --exp P7 --split $SPLIT --layouts ${L4:-10} --shard $S --nshards $N
$PY experiments.py run --exp ABL --split $SPLIT --layouts ${L5:-8} --shard $S --nshards $N
