# crane images

```
docker build -f Dockerfile.core -t crane:core-test .
docker run --rm crane:core-test python -c 'import crane_backends.fastsim, fastsim, judge.stats'
docker run --rm crane:core-test crane --help
```

| Image | File | Holds | Size |
|---|---|---|---|
| `lunarzdev/crane:core-<sha>` | `Dockerfile.core` | python:3.12-slim, crane core (numpy, pyyaml) editable at `/opt/crane_ml`, ssh, rsync, git, curl, procps, zstd | 116 MB compressed, 336 MB unpacked |

Tags are pinned to the commit (`core-<sha>`), never `:latest`. CI (`.github/workflows/ci.yml`, job `image`) builds and
smoke-tests the image on every PR and push, and pushes `core-<sha>` on main when the `DOCKERHUB_USERNAME` and
`DOCKERHUB_TOKEN` secrets exist.

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
and venvs. The image has no player and no Unity: T2 runs on the hosts that have them.
