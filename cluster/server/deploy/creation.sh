#!/usr/bin/env bash
# Deploys crane-server (and, with --litellm, the LiteLLM proxy) to a host over ssh; run on innovation from the repo.
# It builds the image here (tests run in the build), ships it with docker save | docker load, writes ~/crane-server/.env
# on the host from ~/.config/crane-cluster/secrets.env (mode 600, values never printed) and starts compose. DNS and
# the tunnel are not touched: the server answers on the host's 127.0.0.1:8787 until the cutover (../README.md).
#   deploy/creation.sh                 build, ship, start; the previous image tag is kept for --rollback
#   deploy/creation.sh --rollback      restart on the previous image
#   deploy/creation.sh --litellm       (re)deploy the LiteLLM stack to ~/litellm on the host (keys from caelestia)
# Env: CRANE_HOST (creation), ARCHIVE_URL (https://crane-archive.lunarz.dev), HOT_CAP_GB (15: disk hot store cap).
set -euo pipefail
HOST=${CRANE_HOST:-creation}
HERE=$(cd "$(dirname "$0")/.." && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
SECRETS=${CRANE_SECRETS:-$HOME/.config/crane-cluster/secrets.env}
FISH=${FISH_SECRETS:-$HOME/.config/caelestia/.env.secrets}

health() {
  ssh "$HOST" 'for i in $(seq 45); do curl -sf -o /dev/null http://127.0.0.1:8787/healthz && curl -sf -o /dev/null http://127.0.0.1:8787/v1/ping && { echo "healthy on $(hostname):8787"; exit 0; }; sleep 2; done
    cd ~/crane-server && docker compose logs --tail 40 coordinator; exit 1'
}

if [ "${1:-}" = --litellm ]; then
  ssh "$HOST" 'mkdir -p ~/litellm && chmod 700 ~/litellm'
  scp -q "$HERE/litellm/config.yaml" "$HERE/litellm/compose.yaml" "$HOST:litellm/"
  # Keep the master key and the database password across redeploys; take the provider keys from the fish secrets.
  {
    for k in OPEN_ROUTER_API_KEY GROQ_API_KEY GOOGLE_AI_STUDIO_KEY; do printf '%s=%s\n' "$k" "$(sed -n "s/^set -gx $k \"\(.*\)\"$/\1/p" "$FISH")"; done
    echo "OLLAMA_INNOVATION_URL=${OLLAMA_INNOVATION_URL:-http://innovation.mesh.lunarz.dev:11434}"
    echo "OLLAMA_SIMWIN_URL=${OLLAMA_SIMWIN_URL:-http://simwin.mesh.lunarz.dev:11434}"
  } | ssh "$HOST" 'umask 077; cd ~/litellm; cat > .env.new
    for k in LITELLM_MASTER_KEY POSTGRES_PASSWORD; do
      v=$(sed -n "s/^$k=//p" .env 2>/dev/null); [ -n "$v" ] || v=$( [ $k = LITELLM_MASTER_KEY ] && echo sk- )$(openssl rand -hex 24); echo "$k=$v" >> .env.new
    done
    mv .env.new .env && docker compose up -d && for i in $(seq 60); do curl -sf -o /dev/null http://127.0.0.1:4000/health/liveliness && { echo "litellm up on $(hostname):4000"; exit 0; }; sleep 3; done; docker compose logs --tail 40 litellm; exit 1'
  exit 0
fi

if [ "${1:-}" = --rollback ]; then
  ssh "$HOST" 'cd ~/crane-server && prev=$(cat .image.prev) && sed -i "s#^CRANE_IMAGE=.*#CRANE_IMAGE=$prev#" .env && docker compose up -d && echo "rolled back to $prev"'
  health
  exit 0
fi

TAG=crane-server:$(git -C "$ROOT" rev-parse --short HEAD)
docker build -f "$ROOT/cluster/server/Dockerfile" -t "$TAG" "$ROOT"
ssh "$HOST" 'mkdir -p ~/crane-server/import && chmod 700 ~/crane-server ~/crane-server/import'
scp -q "$HERE/compose.yaml" "$HOST:crane-server/compose.yaml"
docker save "$TAG" | gzip -1 | ssh "$HOST" 'gunzip | docker load -q'
{
  grep -E '^(ADMIN_TOKEN|GITHUB_WEBHOOK_SECRET|ARCHIVE_ACCESS_ID|ARCHIVE_ACCESS_SECRET)=' "$SECRETS"
  echo "CRANE_IMAGE=$TAG"
  echo "ARCHIVE_URL=${ARCHIVE_URL:-https://crane-archive.lunarz.dev}"
  echo "R2_SOFT_CAP_GB=${HOT_CAP_GB:-15}"
  echo "BACKUP_HOUR_UTC=9"
} | ssh "$HOST" 'umask 077; cd ~/crane-server
  old=$(sed -n "s/^CRANE_IMAGE=//p" .env 2>/dev/null); cat > .env.new; [ -n "$old" ] && echo "$old" > .image.prev
  [ -f .env.extra ] && cat .env.extra >> .env.new; mv .env.new .env
  docker compose up -d --no-build'
health
