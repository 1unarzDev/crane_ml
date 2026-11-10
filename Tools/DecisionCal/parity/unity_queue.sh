#!/bin/sh
# Run the parity Unity launches one at a time.
# Host rule (2026-11-09 23:40): at most ONE Unity process on this host across all sessions. Before every launch
# `heavy status` must show EVERY unity slot free (held by anyone = wait), and `heavy probe unity` must exit 0;
# both are re-checked every 30 s until true. Right after the grant the driver re-reads `heavy status`; if two
# unity slots are held it stops our player at once and exits 75: the mission does not count, and the queue waits a
# random 30-90 s and retries the same job from the pre-launch check.
# Usage: sh Tools/DecisionCal/parity/unity_queue.sh JOBFILE   (one argument line per job: "openloop ARGS" | "closed ARGS")
# Every launch is appended to ~/.cache/decisioncal/parity/unity_launches.log (the budget ledger: <= 40, <= 60 s each).
H=$HOME/astro_dock/src/mhseals_agents/tools/heavy
D=$(dirname "$0")
LEDGER=$HOME/.cache/decisioncal/parity/unity_launches.log

all_unity_free() {
  s=$(python3 "$H" status 2>/dev/null) || return 1
  echo "$s" | grep -q '^unity\.' || return 1                 # cannot read the slots: do not launch
  ! echo "$s" | grep '^unity\.' | grep -vq ' free *$'
}

while IFS= read -r line; do
  [ -z "$line" ] && continue
  case "$line" in \#*) continue;; esac
  while :; do
    until all_unity_free && python3 "$H" probe unity >/dev/null 2>&1; do sleep 30; done
    kind=${line%% *}; args=${line#* }
    echo "$(date -Is) START $line" >> "$LEDGER"
    if [ "$kind" = openloop ]; then
      nice -n 10 python3 "$D/lockstep_openloop.py" $args
    else
      nice -n 10 python3 "$D/closedloop.py" unity $args
    fi
    rc=$?
    if [ "$rc" -eq 75 ]; then                 # raced another Unity: our player was stopped at once; requeue
      echo "$(date -Is) RACE_ABORT (not counted, requeued) $line" >> "$LEDGER"
      sleep $(( 30 + $(od -An -N2 -tu2 /dev/urandom | tr -d ' ') % 61 ))
      continue
    fi
    echo "$(date -Is) END rc=$rc $line" >> "$LEDGER"
    break
  done
done < "$1"
