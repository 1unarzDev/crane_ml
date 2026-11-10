#!/bin/bash
# Host rule (one Unity process in total on innovation):
#  1. pre-launch: wait until `heavy status` shows NO unity slot held by anyone (poll every 30 s), then probe;
#  2. launch CMD (which itself goes through `heavy unity`, HEAVY_WHO must be set);
#  3. post-grant: as soon as a slot shows $HEAVY_WHO, re-run `heavy status`; if two unity slots are held, kill CMD's
#     process group at once, wait a random 30-90 s and start again from 1.
#   HEAVY_WHO=... unity_gate.sh CMD...
H=~/astro_dock/src/mhseals_agents/tools/heavy
WHO=${HEAVY_WHO:?set HEAVY_WHO}
t0=$(date +%s); attempt=0
held() { python3 $H status 2>&1 | grep -E '^unity\.[0-9]+' | grep -vc ' free'; }
while true; do
  attempt=$((attempt + 1))
  while true; do
    if [ "$(held)" = 0 ] && python3 $H probe unity >/dev/null 2>&1; then break; fi
    sleep 30
  done
  echo "unity_gate: attempt $attempt: all unity slots free after $(( $(date +%s) - t0 )) s; launching: $*"
  setsid "$@" &
  pid=$!
  granted=0; clash=0
  while kill -0 $pid 2>/dev/null; do
    st=$(python3 $H status 2>&1)
    if [ $granted = 0 ] && echo "$st" | grep -E '^unity\.[0-9]+' | grep -q "$WHO"; then
      granted=1
      n=$(python3 $H status 2>&1 | grep -E '^unity\.[0-9]+' | grep -vc ' free')
      echo "unity_gate: granted; unity slots held now: $n"
      if [ "$n" -ge 2 ]; then clash=1; kill -TERM -- -$pid 2>/dev/null; sleep 3; kill -KILL -- -$pid 2>/dev/null; break; fi
    fi
    sleep 1
  done
  if [ $clash = 1 ]; then
    w=$((30 + RANDOM % 61)); echo "unity_gate: two unity slots held after the grant; stopped, retrying in $w s"; sleep $w; continue
  fi
  wait $pid; rc=$?
  echo "unity_gate: done rc=$rc after $(( $(date +%s) - t0 )) s ($attempt attempt(s))"
  exit $rc
done
