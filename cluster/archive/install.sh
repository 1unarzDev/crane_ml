#!/usr/bin/env bash
# Installs crane-archive (the cluster's cold storage node) for the current user: the code under
# ~/.local/share/crane-cluster/lib, a config at ~/.config/crane-cluster/archive.yaml, the store root
# (a plain directory until the drives arrive), and systemd --user units for the daemon and its tunnel.
#   install.sh [--root DIR] [--node NAME] [--quota SIZE] [--min-free SIZE] [--coordinator URL]
#              [--token-file FILE] [--no-service]
# The archive token: an admin calls POST /v1/admin/archive_token {"name": NODE} and saves the token
# at --token-file (default ~/.config/crane-cluster/archive.token, mode 600).
# The tunnel (crane-archive-tunnel.service) starts once ~/.config/crane-cluster/archive-tunnel.token
# exists; cloudflare_setup.py writes it (README "Serving").
# Re-running updates the code and units and keeps config, token and store.
set -euo pipefail

root="$HOME/crane-archive" node="$(cat /etc/hostname 2>/dev/null || uname -n)" quota=20G min_free=25G
coordinator=https://crane.lunarz.dev service=1
cfgdir="${XDG_CONFIG_HOME:-$HOME/.config}/crane-cluster"
token_file="$cfgdir/archive.token"
prefix="${XDG_DATA_HOME:-$HOME/.local/share}/crane-cluster"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while [ $# -gt 0 ]; do
  case $1 in
    --root) root=$2; shift 2 ;;
    --node) node=$2; shift 2 ;;
    --quota) quota=$2; shift 2 ;;
    --min-free) min_free=$2; shift 2 ;;
    --coordinator) coordinator=$2; shift 2 ;;
    --token-file) token_file=$2; shift 2 ;;
    --no-service) service=0; shift ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "install.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done

py=$(command -v python3 || true)
if [ -z "$py" ] || ! "$py" -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
  echo "install.sh: python3 >= 3.9 required" >&2; exit 1
fi
"$py" -c 'import compression.zstd' 2>/dev/null || "$py" -c 'import zstandard' 2>/dev/null ||
  echo "install.sh: no zstd for $py (Python 3.14, or pip install zstandard); compressible objects use xz" >&2

# 1. code: crane_archive plus crane_cluster (its YAML reader), copied (no pip)
lib="$prefix/lib"
mkdir -p "$lib" "$prefix/bin"
for pkg in "$here/crane_archive" "$here/../crane_cluster"; do
  name=$(basename "$pkg")
  rm -rf "$lib/$name.new"
  cp -R "$pkg" "$lib/$name.new"
  find "$lib/$name.new" -name __pycache__ -prune -exec rm -rf {} +
  rm -rf "$lib/$name"
  mv "$lib/$name.new" "$lib/$name"
done
printf '#!/bin/sh\nPYTHONPATH="%s" exec "%s" -m crane_archive "$@"\n' "$lib" "$py" > "$prefix/bin/crane-archive"
chmod +x "$prefix/bin/crane-archive"

# 2. config (kept if present)
config="$cfgdir/archive.yaml"
mkdir -p "$cfgdir"
if [ ! -f "$config" ]; then
  cat > "$config" <<CFG
# crane-archive (cluster/archive/README.md). Space: quota_bytes caps the store; min_free_bytes keeps the
# disk's free space above this whatever the quota says.
coordinator: $coordinator
token_file: $token_file
root: $root
node: $node
quota_bytes: $quota
min_free_bytes: $min_free
listen: 127.0.0.1:7420
pull: {interval_s: 300, batch: 200, min_age_h: 24}
scrub: {gb_per_day: 200}
# Off-site copy on an rclone remote (README "Off-site"); remote '' turns it off. rclone holds the credentials.
# offsite: {remote: 'drive:', root: crane-archive, bundle_bytes: 1.5G, bandwidth: '', transfers: 4, chunk_size: 64M}
# access: {team_domain: TEAM.cloudflareaccess.com, aud: AUD_TAG}   (cloudflare_setup.py fills this)
CFG
  echo "wrote $config"
fi

# 3. the store (refuses a non-empty directory without the marker)
root_now=$(sed -n 's/^root: *//p' "$config" | head -1)
"$prefix/bin/crane-archive" --config "$config" status >/dev/null 2>&1 ||
  "$prefix/bin/crane-archive" --config "$config" init --root "${root_now:-$root}" --node "$node"

[ -f "$token_file" ] || echo "install.sh: no archive token at $token_file yet (admin: POST /v1/admin/archive_token)" >&2

# 4. services
[ "$service" = 1 ] || exit 0
if ! command -v systemctl >/dev/null || ! systemctl --user show-environment >/dev/null 2>&1; then
  echo "no systemd user session; run: $prefix/bin/crane-archive --config $config run"
  exit 0
fi
units="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
mkdir -p "$units"
cat > "$units/crane-archive.service" <<UNIT
[Unit]
Description=crane cluster archive node (cold storage)
After=network-online.target
# never write into an empty mountpoint: the store refuses a root without its marker file, and this waits for the mount
RequiresMountsFor=${root_now:-$root}

[Service]
ExecStart=$prefix/bin/crane-archive --config $config run
Restart=always
RestartSec=30
Nice=10
IOSchedulingClass=idle
MemoryMax=2G
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=default.target
UNIT
tunnel_token="$cfgdir/archive-tunnel.token"
cat > "$units/crane-archive-tunnel.service" <<UNIT
[Unit]
Description=Cloudflare Tunnel for crane-archive (origin 127.0.0.1:7420)
After=network-online.target crane-archive.service
ConditionPathExists=$tunnel_token

[Service]
ExecStart=$(command -v cloudflared || echo /usr/bin/cloudflared) tunnel --no-autoupdate run --token-file $tunnel_token
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
UNIT
cat > "$units/crane-archive-offsite.service" <<UNIT
[Unit]
Description=crane-archive off-site push (bundles to the rclone remote)
After=network-online.target
ConditionPathExists=$config

[Service]
Type=oneshot
ExecStart=$prefix/bin/crane-archive --config $config offsite push
Nice=15
IOSchedulingClass=idle
TimeoutStartSec=12h
UNIT
cat > "$units/crane-archive-offsite.timer" <<UNIT
[Unit]
Description=Daily crane-archive off-site push

[Timer]
OnCalendar=*-*-* 03:30:00
RandomizedDelaySec=15m
Persistent=true

[Install]
WantedBy=timers.target
UNIT
systemctl --user daemon-reload
grep -qE '^ *offsite:' "$config" && systemctl --user enable --now crane-archive-offsite.timer
if [ -f "$token_file" ]; then
  systemctl --user enable crane-archive.service
  systemctl --user restart crane-archive.service
  echo "started crane-archive.service; logs: journalctl --user -u crane-archive -f"
fi
if [ -f "$tunnel_token" ] && command -v cloudflared >/dev/null; then
  systemctl --user enable crane-archive-tunnel.service
  systemctl --user restart crane-archive-tunnel.service
  echo "started crane-archive-tunnel.service"
fi
echo "to keep them running after logout: loginctl enable-linger $USER"
