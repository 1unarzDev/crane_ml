# Installing crane

```sh
./install.sh                                    # a terminal: core, then the wizard (crane setup) asks the rest
./install.sh --yes --components core,fast,player
crane setup --yes --components core,fast,player --domains boat
crane update                                    # later: the code and the player from the latest release
crane deploy HOST --player                      # another machine, over ssh
crane doctor
```

## install.sh
It installs from a checkout (or clones one into `--dir`, default `~/crane_ml`). It needs no sudo, is safe to rerun
and prints every step before running it; `--dry-run` only prints the steps.

- **The venv.** uv, a Python 3.12 venv at `<repo>/.venv`, crane installed editable with the chosen extras, and
  `~/.local/bin/crane`.
- **Components.**
  - `core` (numpy, pyyaml) is always installed.
  - The extras: `fast` (numba), `gpu` (jax[cuda12]), `view` (PySide6, pyqtgraph), `field` (mcap), `data`
    (pyarrow), `setup` (rich, questionary).
  - `player` is the release's T2 player (see below). With no release yet, install.sh says how to get one.
- **The wizard.** On a terminal, with neither `--yes` nor `--components`, install.sh installs core and `setup`,
  then hands over to `crane setup`.
- **Rented boxes.** They run `install.sh --yes --no-clone --components core` as root on python:3.12-slim.

## crane setup
The wizard runs in four steps:

1. **Detect.** The OS, cores, memory, disk, GPU and driver, docker, astro_dock, the Unity editor and the display.
2. **Choose.** Components and domains (`config/domains/*.yaml`), with their sizes and time estimates. The defaults
   follow the machine: `fast` on 4+ cores, `gpu` with an NVIDIA driver, `view` with a display, and `player` and
   `astro` with astro_dock.
3. **Install.** The extras go through `install.sh --yes --no-clone --components ...`, and the player through
   `crane update --player-only`.
4. **Check.** `astro` checks each domain's T2 host tooling, read-only: its checkout, its vessel profile, a sim
   backend of `bin/crane sim` and crane's help.yaml. setup never builds astro images or workspaces. It then saves
   the choices to `~/.config/crane/setup.json` and runs `crane doctor`.

Ways to run it:
- `--yes` asks nothing.
- `--dry-run` stops after the plan.
- `--json` gives the machine, the plan and the results as one document.
- With rich and questionary installed it uses them; without, it uses plain prompts.

## The player store
`~/.local/share/crane/player/<commit12>/` holds one dir per player, plus `current` and `prev` links. Any other
player is pruned on each install.

- **Who installs it.** `crane update`, `crane deploy --player`, install.sh's `player` and the sim image all
  install through `crane update --tarball`.
- **Checks.** Each tarball is checked by sha256 before it is unpacked. An install that would leave less than
  1 GB free refuses.
- **Who uses it.** `crane sim up` and `crane doctor` look for the player in this order: `CRANE_PLAYER`, then
  `Builds/CRANE-Worker`, then the phase-1 link, then `current`.

## crane update
- **In a git checkout:** `git fetch` then `git merge --ff-only @{u}` (or the tag `vX.Y.Z` with `--version`), then
  the release player.
- **In an install without a checkout:** the release wheel, installed into crane's venv, then the player.
- **Flags:**
  - `--player-only` or `--no-player` limits it to one part.
  - `--dry-run` prints the steps.
  - `CRANE_RELEASES` points at another release base: a URL, or a local dir laid out like GitHub's
    `latest/download/` and `download/vX.Y.Z/`.
- **Exit codes:** 1 when a file fails its sha256 check, 2 when the disk is short.

## crane deploy HOST
```sh
crane deploy HOST --dry-run              # probe: free disk, checkout, player, python; then the plan
crane deploy HOST --player                         # this checkout's commit, a core .venv, the player
crane deploy HOST --components core,fast --ref main
```
On HOST it touches only `~/crane_ml`, `~/.local/bin/crane` and the player store. It runs these steps:

1. **Code.** REF is `--ref` (default `HEAD`, this checkout's commit). A host with a git checkout fetches that commit
   from this machine and fast-forwards (`git fetch ssh://THISHOST/REPO COMMIT && git merge --ff-only FETCH_HEAD`;
   `--from URL` or `CRANE_DEPLOY_FROM` changes the source). A host without one gets REF's tracked files by rsync,
   without the Unity project, into an empty `~/crane_ml` or an earlier deploy's.
2. **Venv.** `install.sh --yes --no-clone --components LIST` runs there (uv, Python 3.12, `~/crane_ml/.venv`) when
   HOST has no `.venv` or `--components` is given. `--no-venv` skips it.
3. **Player.** The tarball goes over with `rsync --partial`, so a cut transfer resumes. HOST checks its sha256 and
   installs it with current + prev. The tarball is `--tarball`, else the newest `Builds/crane-player-*.tar.*` (a
   `warn` row when it was built from another commit than the code), else the release's.
4. **Doctor.** `bin/crane doctor --json` runs on HOST and its table is printed here.

It refuses (exit 2) when HOST has less free space than the player, unpacked, plus 1 GB, and when `~/crane_ml` there
is neither a crane checkout nor an earlier deploy. `local:DIR` deploys into
DIR as the home, with no ssh; the tests use it.
