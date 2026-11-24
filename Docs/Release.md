# Releases and images

```sh
# bump pyproject.toml and crane/__init__.py to 0.3.0, commit, then:
git tag v0.3.0 && git push origin v0.3.0
# Player (self-hosted Unity runner) -> Release (assets) -> Images (sim)
crane update --version 0.3.0
```

## The release flow
1. **`player.yml`.** It runs on the tag, on the self-hosted `crane-unity` runner, through `bin/heavy unity`. It
   runs the EditMode tests, then `crane build player --tarball`. The artifact is
   `crane-player-<sha>` (`crane-player-<commit12>-linux-x86_64.tar.zst` and its `.sha256`).
2. **`release.yml`.** It runs when that Player run succeeds, or by dispatch with a tag. It checks:
   - the tag equals `pyproject.toml`'s version and `crane.__version__`;
   - the player tarball matches its `.sha256`;
   - the player was built from the tag's commit with the Unity version in `ProjectVersion.txt`.

   It then publishes `crane-X.Y.Z-py3-none-any.whl`, `crane-X.Y.Z.tar.gz`, the player tarball, `SHA256SUMS` and
   `release.json`.
3. **`images.yml`.** release.yml calls it with `kinds: sim`, and it builds `sim` from the released player.

`release.json` (`python3 -m crane.update release-json` writes it):

```json
{"schema": "crane.release/1", "version": "0.3.0", "tag": "v0.3.0", "commit": "<sha>", "unity_version": "6000.5.10f1",
 "player": {"name": "crane-player-<commit12>-linux-x86_64.tar.zst", "sha256": "...", "bytes": 104762091,
            "unpacked_bytes": 394593690, "source_commit": "<sha>", "unity_version": "6000.5.10f1"},
 "wheel": {"name": "...", "sha256": "...", "bytes": 0}, "sdist": {"name": "...", "sha256": "...", "bytes": 0}}
```

install.sh, `crane update` and `crane deploy` read it from `releases/latest/download/release.json`, or from
`releases/download/vX.Y.Z/` with a version.

## Images
| Image | File | Holds | Size (compressed) |
|---|---|---|---|
| `lunarzdev/crane:core-<sha>` | `Dockerfile.core` | python:3.12-slim, crane core | 115 MB |
| `lunarzdev/crane:gpu-<sha>` | `Dockerfile.gpu` | core + jax[cuda12] 0.12.1 + numba 0.68.1 (CUDA from pip wheels) | 3.5 GB (limit 4 GB) |
| `lunarzdev/crane:sim-<sha>` | `Dockerfile.sim` | core + the release player + xvfb, Mesa Vulkan/GL | 0.38 GB (1.5 GB unpacked) |

Every image keeps compute's boot.sh contract (`docker/README.md`). `crane build images core gpu sim` builds them
locally as `crane:KIND-<commit12>`; sim takes `--player TARBALL`, by default the newest `Builds` tarball.

**images.yml** follows astro's flow:
1. Build the image and smoke-test it (`docker/smoke.sh KIND IMAGE`).
2. Push it as `KIND-next` and smoke-test the pulled image.
3. Promote it to `KIND-<sha>`, plus `KIND-vX.Y.Z` on a release.
4. Rotate the pins with `docker/registry.py`: keep the newest 20 `KIND-<sha>` and every `KIND-vX.Y.Z`.

There is never a `:latest`. Registry mutations share the `docker-hub-mutations` concurrency group. Without the
`DOCKERHUB_USERNAME` and `DOCKERHUB_TOKEN` secrets, images are still built and smoke-tested, and the registry steps
are skipped with one line.

**sim and rendering.** Unity refuses Mesa's llvmpipe (`No supported renderers found`), so sim renders only on a box
with a Vulkan GPU. With `-nographics` it boots the scene and applies the plant (`CRANE_PLANT_APPLIED`), which is
what the smoke test requires. The smoke test reports the rendering probe without requiring it. sim stays off
rented boxes until that probe renders there. T2 runs on the Unity hosts.
