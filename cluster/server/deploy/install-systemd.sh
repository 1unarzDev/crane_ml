#!/usr/bin/env bash
# Installs the built server (dist/) as a systemd user service on this host. Run from cluster/server after
# `npm ci && npm run build`. The .env (secrets) is not touched if it exists.
set -euo pipefail
cd "$(dirname "$0")/.."
[ -f dist/main.mjs ] || { echo "build first: npm ci && npm run build" >&2; exit 1; }
node -e 'const [a]=process.versions.node.split(".");process.exit(+a>=24?0:1)' || { echo "node >= 24 required (node:sqlite)" >&2; exit 1; }
mkdir -p ~/crane-server/data ~/.config/systemd/user
rm -rf ~/crane-server/app.new && cp -r dist ~/crane-server/app.new
[ -d ~/crane-server/app ] && rm -rf ~/crane-server/app.prev && mv ~/crane-server/app ~/crane-server/app.prev
mv ~/crane-server/app.new ~/crane-server/app
[ -f ~/crane-server/.env ] || { umask 077; printf 'ADMIN_TOKEN=\nGITHUB_WEBHOOK_SECRET=\nARCHIVE_URL=\n' > ~/crane-server/.env; echo "fill in ~/crane-server/.env"; }
cp deploy/crane-server.service ~/.config/systemd/user/crane-server.service
systemctl --user daemon-reload
systemctl --user enable --now crane-server
systemctl --user restart crane-server
for _ in $(seq 20); do curl -sf http://127.0.0.1:8787/healthz >/dev/null && { echo "crane-server healthy"; exit 0; }; sleep 1; done
echo "crane-server did not answer /healthz; journalctl --user -u crane-server" >&2; exit 1
