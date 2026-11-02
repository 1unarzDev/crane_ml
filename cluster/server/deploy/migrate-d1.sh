#!/usr/bin/env bash
# Moves the coordinator's state from Cloudflare to the self-hosted server; run on innovation (wrangler is here).
#   deploy/migrate-d1.sh            export D1 (read-only on Cloudflare), ship it, import it on the host (the server
#                                   stops for the import; the old database is kept as coordinator.sqlite.pre-import)
#   deploy/migrate-d1.sh --pull-r2  copy every R2 object into the host's disk store (~3 objects/s, resumable:
#                                   objects already copied are skipped); the API token goes over ssh stdin only
# The export reads every D1 row once (about the size of the database) from the account's daily read quota.
set -euo pipefail
HOST=${CRANE_HOST:-creation}
HERE=$(cd "$(dirname "$0")/.." && pwd)
FISH=${FISH_SECRETS:-$HOME/.config/caelestia/.env.secrets}
token() { sed -n 's/^set -gx CLOUDFLARE_API_KEY "\(.*\)"$/\1/p' "$FISH"; }

if [ "${1:-}" = --pull-r2 ]; then
  T=$(token)
  A=$(curl -s -H "Authorization: Bearer $T" "https://api.cloudflare.com/client/v4/accounts?per_page=5" | python3 -c 'import json,sys; print(json.load(sys.stdin)["result"][0]["id"])')
  printf '%s\n%s\n' "$T" "$A" | ssh "$HOST" 'read -r T; read -r A; cd ~/crane-server &&
    CLOUDFLARE_API_TOKEN=$T CLOUDFLARE_ACCOUNT_ID=$A docker compose run --rm --no-deps -e CLOUDFLARE_API_TOKEN -e CLOUDFLARE_ACCOUNT_ID \
      coordinator node --disable-warning=ExperimentalWarning migrate.mjs pull-r2'
  exit 0
fi

OUT=$HOME/.cache/crane-server
mkdir -p "$OUT" && chmod 700 "$OUT"
DUMP=$OUT/d1-$(date -u +%Y%m%dT%H%M%S).sql
(cd "$HERE/../coordinator" && CLOUDFLARE_API_TOKEN=$(token) npx wrangler d1 export crane-cluster --remote --output "$DUMP")
echo "exported $(wc -c < "$DUMP") bytes to $DUMP"
scp -q "$DUMP" "$HOST:crane-server/import/d1.sql"
ssh "$HOST" 'cd ~/crane-server && docker compose stop coordinator &&
  docker compose run --rm --no-deps -T coordinator \
    node --disable-warning=ExperimentalWarning migrate.mjs import /dev/stdin --force < import/d1.sql &&
  rm -f import/d1.sql && docker compose up -d --no-build &&
  for i in $(seq 45); do curl -sf -o /dev/null http://127.0.0.1:8787/healthz && { echo imported and healthy; exit 0; }; sleep 2; done; exit 1'
