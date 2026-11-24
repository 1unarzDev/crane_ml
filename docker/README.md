# crane images

```
docker build -f Dockerfile.core -t crane:core-test .
docker run --rm crane:core-test python -c 'import crane_backends.fastsim, fastsim, judge.stats'
docker run --rm crane:core-test crane --help
```

| Image | File | Holds | Size |
|---|---|---|---|
| `lunarzdev/crane:core-<sha>` | `Dockerfile.core` | python:3.12-slim, crane core (numpy, pyyaml) editable at `/opt/crane_ml`, ssh, rsync, git, curl, procps, zstd | 116 MB compressed, 336 MB unpacked |
| `lunarzdev/crane:gpu-<sha>` | `Dockerfile.gpu` | the same plus jax[cuda12] and numba (CUDA 12 from pip wheels; the box's driver is enough) | 3.5 GB compressed (limit 4 GB) |
| `lunarzdev/crane:sim-<sha>` | `Dockerfile.sim` | core plus the release player at `/opt/crane-player/current` (`CRANE_PLAYER`), xvfb, Mesa Vulkan/GL | 0.38 GB compressed, 1.5 GB unpacked |

Tags are pinned to the commit (`KIND-<sha>`), never `:latest`. `.github/workflows/images.yml` builds, smoke-tests
(`docker/smoke.sh KIND IMAGE`), pushes `KIND-next`, promotes it to `KIND-<sha>` and rotates old pins
(`docker/registry.py`); CI's `image` job also builds and smoke-tests core on every PR. Pushes need the
`DOCKERHUB_USERNAME` and `DOCKERHUB_TOKEN` secrets and are skipped without them. Docs/Release.md has the flow.

## The boot.sh contract (rented boxes)
- crane is importable from `/opt/crane_ml` (`PYTHONPATH=/opt/crane_ml`, installed editable), with its deps preinstalled.
- openssh-server, openssh-client, rsync, git, curl and procps are on the image.
- No ENTRYPOINT; `CMD` is `crane --help`, which returns at once.
- The image runs as root, so boot.sh can start sshd. A `crane` user (uid 1000) owns `/opt/crane_ml` for jobs that drop
  privileges.
- `/opt/crane_ml` is a copy without `.git`: syncing code over it (rsync) changes what runs.

Without an image, boot.sh checks out the directories in `crane/runtime.sparse` instead.

## What stays out
`.dockerignore` leaves out the Unity project (Assets, Packages, ProjectSettings, Library), builds, logs, Docs, `.git`
and venvs. core and gpu have no player and no Unity. sim gets its player through a separate build context
(`--build-context player=DIR`), checked by sha256. Unity refuses Mesa's llvmpipe, so sim renders only with a Vulkan
GPU; rented boxes do not use it until a Vulkan probe passes there.
