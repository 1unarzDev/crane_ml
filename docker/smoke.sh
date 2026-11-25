#!/usr/bin/env bash
# Smoke-test a crane image: docker/smoke.sh KIND IMAGE (KIND core, gpu or sim). Exit 1 on the first failure.
#
# Every image: crane importable with its backends, the CLI, compute's boot.sh tools, no ENTRYPOINT, the size limit
# (compressed, as gzip -1 of docker save). gpu: jax and numba import and compute (on CPU here). sim: the player is
# installed with its sha256, and boots the scene headless (-nographics) to CRANE_PLANT_APPLIED within 60 s; then
# the rendering probe (Vulkan) is reported, not required: on a GPU-less runner Unity refuses Mesa's llvmpipe.
set -euo pipefail
kind=${1:?KIND (core, gpu, sim)} image=${2:?IMAGE}
declare -A limit_mb=([core]=300 [gpu]=4000 [sim]=1500)
fail() { echo "smoke $kind: $*" >&2; exit 1; }
inside() { docker run --rm --network none "$image" "$@"; }

inside python -c 'import crane_backends.fastsim, fastsim, judge.stats' || fail "crane does not import"
inside crane --help >/dev/null || fail "crane --help failed"
inside sh -c 'for c in /usr/sbin/sshd ssh rsync git curl ps; do command -v "$c" >/dev/null || { echo "missing $c"; exit 1; }; done' \
  || fail "a boot.sh tool is missing"
test -z "$(docker inspect "$image" --format '{{join .Config.Entrypoint " "}}')" || fail "the image has an ENTRYPOINT"

case $kind in
  gpu)
    inside env JAX_PLATFORMS=cpu python -c 'import jax, jax.numpy as jnp, numba, engine; assert float(jnp.ones(3).sum()) == 3.0; print("jax", jax.__version__, "numba", numba.__version__)' \
      || fail "jax/numba/engine do not work" ;;
  sim)
    inside sh -c 'test -x "$CRANE_PLAYER" && test -f "$(dirname "$CRANE_PLAYER")/.sha256"' || fail "no player at \$CRANE_PLAYER"
    boot=$(inside sh -c 'timeout 60 xvfb-run -a "$CRANE_PLAYER" -batchmode -nographics -logFile - 2>&1 | grep -m1 -E "CRANE_PLANT_APPLIED|No supported renderers"' || true)
    case $boot in *CRANE_PLANT_APPLIED*) echo "player boots headless: ${boot:0:100}" ;; *) fail "the player did not reach CRANE_PLANT_APPLIED in 60 s (-nographics)" ;; esac
    probe=$(inside sh -c 'timeout 60 xvfb-run -a "$CRANE_PLAYER" -batchmode -logFile - 2>&1 | grep -m1 -E "No supported renderers|CRANE_PLANT_APPLIED"' || true)
    case $probe in
      *CRANE_PLANT_APPLIED*) echo "vulkan probe: renders" ;;
      *) echo "vulkan probe: no GPU renderer here (${probe:-no output}); sim stays off rented boxes until it renders there" ;;
    esac ;;
esac

mb=$(( $(docker save "$image" | gzip -1 | wc -c) / 1000000 ))
echo "$image: ${mb} MB compressed (limit ${limit_mb[$kind]} MB)"
test "$mb" -lt "${limit_mb[$kind]}" || fail "${mb} MB is over ${limit_mb[$kind]} MB"
echo "smoke $kind: ok"
