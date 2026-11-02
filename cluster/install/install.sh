#!/usr/bin/env bash
# Installs a crane cluster worker for the current user (Linux: systemd --user service).
#   install.sh --coordinator URL --invite CODE [--name NAME] [--source DIR|GIT_URL]
#              [--config FILE] [--prefix DIR] [--memory-max SIZE] [--no-service]
# --memory-max caps the service and its jobs (systemd MemoryMax=, e.g. 6G; swap is capped at 0).
# The worker is stdlib-only Python (>= 3.9) plus git; docker is needed only for
# container backends. Re-running updates the code and keeps config and token.
# A host that must never run jobs opts out with an empty file /etc/crane-cluster/deny.
set -euo pipefail

coordinator= invite= name= source= service=1 memory_max=
prefix="${XDG_DATA_HOME:-$HOME/.local/share}/crane-cluster"
config="${XDG_CONFIG_HOME:-$HOME/.config}/crane-cluster/worker.yaml"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while [ $# -gt 0 ]; do
  case $1 in
    --coordinator) coordinator=$2; shift 2 ;;
    --invite) invite=$2; shift 2 ;;
    --name) name=$2; shift 2 ;;
    --source) source=$2; shift 2 ;;
    --config) config=$2; shift 2 ;;
    --prefix) prefix=$2; shift 2 ;;
    --memory-max) memory_max=$2; shift 2 ;;
    --no-service) service=0; shift ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "install.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done
[ -n "$coordinator" ] || { echo "install.sh: --coordinator URL is required" >&2; exit 2; }
if [ -e /etc/crane-cluster/deny ]; then echo "install.sh: this host opted out (/etc/crane-cluster/deny)" >&2; exit 1; fi

py=$(command -v python3 || true)
if [ -z "$py" ] || ! "$py" -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
  echo "install.sh: python3 >= 3.9 required" >&2; exit 1
fi
command -v git >/dev/null || { echo "install.sh: git required" >&2; exit 1; }

# 1. code: a copy of the crane_cluster package (no pip; the network is used only for a git source)
[ -n "$source" ] || source="$here/.."
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
if [ -d "$source/crane_cluster" ]; then
  pkg="$source/crane_cluster"
elif [ -d "$source/cluster/crane_cluster" ]; then
  pkg="$source/cluster/crane_cluster"
else
  GIT_TERMINAL_PROMPT=0 git clone -q --depth 1 "$source" "$tmp/src"
  pkg="$tmp/src/cluster/crane_cluster"
fi
lib="$prefix/lib"
mkdir -p "$lib" "$prefix/bin"
rm -rf "$lib/crane_cluster.new"
cp -R "$pkg" "$lib/crane_cluster.new"
find "$lib/crane_cluster.new" -name __pycache__ -prune -exec rm -rf {} +
rm -rf "$lib/crane_cluster"
mv "$lib/crane_cluster.new" "$lib/crane_cluster"
for tool in worker hyp; do
  printf '#!/bin/sh\nPYTHONPATH="%s" exec "%s" -m crane_cluster.%s "$@"\n' "$lib" "$py" "$tool" > "$prefix/bin/crane-$tool"
  chmod +x "$prefix/bin/crane-$tool"
done

# 2. config (kept if present)
if [ ! -f "$config" ]; then
  mkdir -p "$(dirname "$config")"
  {
    echo "coordinator: $coordinator"
    [ -z "$name" ] || echo "name: $name"
    printf 'policy:\n  nice: 10\nbackends:\n  dummy: {type: dummy}\n'
  } > "$config"
  echo "wrote $config (edit backends, caps and policy there)"
fi

# 3. join (skipped without an invite, e.g. when re-running to update)
if [ -n "$invite" ]; then
  "$prefix/bin/crane-worker" --config "$config" join --coordinator "$coordinator" --invite "$invite" ${name:+--name "$name"}
fi

# 4. service
[ "$service" = 1 ] || exit 0
if ! command -v systemctl >/dev/null || ! systemctl --user show-environment >/dev/null 2>&1; then
  echo "no systemd user session; run the worker with: $prefix/bin/crane-worker --config $config run"
  exit 0
fi
unit="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/crane-worker.service"
mkdir -p "$(dirname "$unit")"
cat > "$unit" <<UNIT
[Unit]
Description=crane cluster worker
After=network-online.target

[Service]
ExecStart=$prefix/bin/crane-worker --config $config run
Restart=always
RestartSec=30
Nice=10
IOSchedulingClass=idle
Environment=PYTHONUNBUFFERED=1
${memory_max:+MemoryMax=$memory_max
MemorySwapMax=0
}
[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
systemctl --user enable --now crane-worker.service
echo "started crane-worker.service; logs: journalctl --user -u crane-worker -f"
echo "to keep it running after logout: loginctl enable-linger $USER"
