#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Run selected stages of run_pending.sh without editing it:  run_pending_stages.sh "2 3"
# Extracts the header (build, vars) and the chosen stage blocks; outputs and binary go under ~/.cache/frontier/crane.
set -u
cd "$(dirname "$0")/../.."
ST=${1:?stages e.g. "2 3"}
export CRANE_HARNESS_OUT=${CRANE_HARNESS_OUT:-$HOME/.cache/frontier/crane/harness}
BINDIR=${CRANE_BIN:-$HOME/.cache/frontier/crane/bin}
mkdir -p "$(dirname "$CRANE_HARNESS_OUT")"
GEN="$CRANE_HARNESS_OUT.gen.sh"
awk -v st="$ST" '
  BEGIN{n=split(st,a," "); for(i=1;i<=n;i++) want[a[i]]=1; cur=0}
  /^echo "\[pending\] [0-9]\/5/ {match($0,/[0-9]\/5/); cur=substr($0,RSTART,1)}
  /^echo "\[pending\] done/ {cur=-1}
  cur==0 && /^cd "\$\(dirname/ {next}
  cur==0 {print}
  cur>0 && (cur in want) {print}
' Tools/CraneHarness/run_pending.sh \
 | sed -e "s#^BIN=.*#BIN=$BINDIR#" -e 's#Tools/CraneHarness/navpilot_bridge.py#Tools/CraneHarness/navpilot_bridge_timed.py#' > "$GEN"
echo "[stages] generated $GEN for stages: $ST"
exec bash "$GEN"
