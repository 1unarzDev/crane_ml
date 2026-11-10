#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# One staged recfeas stage, fix then legacy, sequential, single core.  Usage (via fsrun): run_recfeas_stage.sh STAGE [PART_SUFFIX...]
# Outputs ~/.cache/frontier/crane/recfeas/<spec name>; costs appended to Tools/Frontier/costs.jsonl.
set -u
cd "$(dirname "$0")/../../.."
BIN=$HOME/.cache/frontier/crane/bin; O=$HOME/.cache/frontier/crane/recfeas; PY=$HOME/.cache/research/fs-venv/bin/python
stage=$1
mkdir -p $O
for f in Tools/CraneHarness/specs/recfeas_staged_s${stage}*.json Tools/CraneHarness/specs/recfeas_legacy_s${stage}*.json; do
  n=$(basename $f .json)
  [ -f $O/$n/results_shard0.jsonl ] && { echo "skip $n"; continue; }
  $PY Tools/Frontier/ops/timed.py $O/$n.time.json -- dotnet $BIN/CraneExperiment.dll run --spec Tools/CraneHarness/specs/$n.json --out $O/$n >$O/$n.log 2>&1
  cf=$($PY -c "import json;d=json.load(open('$O/$n.time.json'));print(d['cpu_s']/d['wall_s']);print(d,file=__import__('sys').stderr)")
  $PY Tools/Frontier/ops/recfeas.py costs $O/$n "$n" $cf
done
