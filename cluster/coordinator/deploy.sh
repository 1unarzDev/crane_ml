#!/usr/bin/env bash
# Deploys the coordinator, then smoke-checks it: GET /v1/ping and /v1/feed must answer 200 within ~60 s,
# or the Worker rolls back to the deployment that was live before. The rollback runs only when the live
# deployment passed the same check before this deploy: a failure the deploy did not cause (D1's daily
# quota, say) is reported, not "fixed" by rolling back good code. Extra arguments go to `wrangler deploy`.
set -euo pipefail
cd "$(dirname "$0")"
BASE="${CRANE_URL:-https://crane.lunarz.dev}"

healthy() {
  local p code
  for p in /v1/ping "/v1/feed?recent=1"; do
    code=$(curl -s -o /dev/null -m 15 -w '%{http_code}' "$BASE$p") || code=000
    [ "$code" = 200 ] || { echo "smoke: $p -> HTTP $code" >&2; return 1; }
  done
}

# The deep view (/viewer/) is crane_ml's viewer/web plus the ctraj codec: copied in, never committed here.
rm -rf public/viewer
mkdir -p public/viewer
cp ../../viewer/web/* public/viewer/
cp ../../theater/web/ctraj.js public/viewer/ctraj.js

before=ok
healthy || before=failing
npx wrangler deploy "$@"
for _ in $(seq 12); do
  if healthy 2>/dev/null; then echo "smoke: /v1/ping and /v1/feed answer 200"; exit 0; fi
  sleep 5
done
healthy || true
if [ "$before" = ok ]; then
  npx wrangler rollback -y -m "deploy.sh: post-deploy smoke of /v1/ping and /v1/feed failed"
  echo "smoke: FAILED after deploy; rolled back to the previous deployment" >&2
else
  echo "smoke: FAILED, but $BASE was already failing before this deploy; not rolling back" >&2
fi
exit 1
