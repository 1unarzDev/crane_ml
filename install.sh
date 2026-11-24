#!/usr/bin/env bash
# Install crane: the repo, uv, a Python 3.12 venv at <repo>/.venv with crane and its extras, and ~/.local/bin/crane.
#
#   ./install.sh                     # on a terminal: core and the wizard's extra, then `crane setup` asks the rest
#   ./install.sh [--yes] [--dir D] [--components core,fast,gpu,view,field,data,setup,player] [--domains boat,...]
#                [--no-clone] [--dry-run]
#   curl -fsSL https://raw.githubusercontent.com/1unarzDev/crane_ml/main/install.sh | bash -s -- --yes
#
# From a checkout it installs that checkout; otherwise it clones (or updates) the repo into --dir (default
# ~/crane_ml). Idempotent, never uses sudo, prints every step before it runs it. --dry-run prints them only.
# Components: core (numpy, pyyaml; always), fast (numba), gpu (jax CUDA 12), view (PySide6, pyqtgraph),
# field (mcap), data (pyarrow, for parquet), setup (rich, questionary), player (the release T2 player, sha256
# checked, into ~/.local/share/crane/player through `crane update --player-only`; without a release yet it says
# how to get one). A terminal run without --yes or --components installs core and setup, then hands over to
# `crane setup` (the wizard: components, domains, sizes, the player, crane doctor).
set -euo pipefail

REPO_URL=${CRANE_REPO_URL:-https://github.com/1unarzDev/crane_ml.git}
yes=0 dir='' components=core domains='' clone=1 dry=0 given=0 player=0
here=$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" 2>/dev/null && pwd || true)

say() { printf 'install: %s\n' "$*"; }
die() { printf 'install: %s\n' "$*" >&2; exit 2; }
run() { printf '+ %s\n' "$*"; [ "$dry" = 1 ] || "$@"; }
usage() { sed -n '2,/^set -euo/{/^set -euo/d;s/^# \{0,1\}//p}' "$0"; }

while [ $# -gt 0 ]; do
  case $1 in
    --yes|-y) yes=1; shift ;;
    --dir) dir=$2; shift 2 ;;
    --components) components=$2; given=1; shift 2 ;;
    --domains) domains=$2; shift 2 ;;
    --no-clone) clone=0; shift ;;
    --dry-run) dry=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option $1 (install.sh --help)" ;;
  esac
done

extras=()
IFS=, read -ra wanted <<<"$components"
for c in "${wanted[@]}"; do
  case $c in
    core|'') ;;
    fast|gpu|view|field|data|setup|all) extras+=("$c") ;;
    player) player=1 ;;
    *) die "unknown component $c (core, fast, gpu, view, field, data, setup, player)" ;;
  esac
done

# A terminal run that names no components: core plus the wizard's extra, then crane setup asks for the rest.
wizard=0
if [ "$yes" = 0 ] && [ "$given" = 0 ] && [ "$dry" = 0 ] && [ -r /dev/tty ] && [ -t 1 ]; then
  wizard=1 extras=(setup)
fi

# 1. the repo: this checkout, or --dir (cloned or updated)
if [ -z "$dir" ] && [ -n "$here" ] && [ -f "$here/pyproject.toml" ] && [ -d "$here/crane" ]; then
  repo=$here
  say "repo: this checkout, $repo"
else
  repo=${dir:-$HOME/crane_ml}
  if [ -d "$repo/.git" ]; then
    if [ "$clone" = 1 ]; then run git -C "$repo" pull --ff-only; else say "repo: $repo (--no-clone: not updated)"; fi
  elif [ "$clone" = 1 ]; then
    command -v git >/dev/null || die "git is needed to clone $REPO_URL"
    run git clone --depth 1 "$REPO_URL" "$repo"
  elif [ -f "$repo/pyproject.toml" ] && [ -d "$repo/crane" ]; then
    say "repo: $repo (--no-clone, not a git checkout)"
  else
    die "no crane checkout at $repo and --no-clone given"
  fi
fi

spec="$repo"
[ ${#extras[@]} -gt 0 ] && spec="$repo[$(IFS=,; echo "${extras[*]}")]"
say "plan: uv, $repo/.venv (Python 3.12), crane${extras:+ [${extras[*]}]}, ~/.local/bin/crane"
[ "$player" = 1 ] && say "plan: the release T2 player (crane update --player-only)"
[ "$wizard" = 1 ] && say "plan: then crane setup (the wizard) for the other components, the domains and the player"
[ -n "$domains" ] && [ "$wizard" = 0 ] && say "domains: $domains (crane setup --yes --domains $domains records them)"
if [ "$wizard" = 0 ] && [ "$yes" = 0 ] && [ "$dry" = 0 ] && [ -r /dev/tty ] && [ -t 1 ]; then
  read -r -p "install: go ahead? [Y/n] " answer </dev/tty || answer=n
  case $answer in ''|y|Y|yes) ;; *) die "stopped" ;; esac
fi

# 2. uv: on PATH, in ~/.local/bin, else its installer (curl or wget), else pip --user
bin="$HOME/.local/bin"
uv=$(command -v uv || true)
[ -z "$uv" ] && [ -x "$bin/uv" ] && uv="$bin/uv"
if [ -z "$uv" ]; then
  if command -v curl >/dev/null; then
    run sh -c "curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR='$bin' UV_NO_MODIFY_PATH=1 sh"
  elif command -v wget >/dev/null; then
    run sh -c "wget -qO- https://astral.sh/uv/install.sh | env UV_INSTALL_DIR='$bin' UV_NO_MODIFY_PATH=1 sh"
  else
    run python3 -m pip install --user --quiet --disable-pip-version-check --root-user-action=ignore uv
  fi
  uv="$bin/uv"
else
  say "uv: $uv"
fi

# 3. the venv and crane (editable, so the checkout is what runs, and the cluster finds its backends)
venv="$repo/.venv"
if [ -x "$venv/bin/python" ] && "$venv/bin/python" -c 'import sys; sys.exit(sys.version_info[:2] != (3, 12))' 2>/dev/null; then
  say "venv: $venv (Python 3.12)"
else
  run "$uv" venv --quiet --python 3.12 --clear "$venv"     # absent, or not 3.12
fi
run "$uv" pip install --quiet --python "$venv/bin/python" -e "$spec"

# 4. the command
run mkdir -p "$bin"
run ln -sfn "$venv/bin/crane" "$bin/crane"
[ "$dry" = 1 ] || "$bin/crane" --version
case ":$PATH:" in *":$bin:"*) ;; *) say "add $bin to PATH (e.g. in ~/.bashrc: export PATH=\"$bin:\$PATH\")" ;; esac

# 5. the T2 player from the release (crane/update.py: sha256 checked, current + prev in the player store)
if [ "$player" = 1 ]; then
  run "$venv/bin/python" -m crane update --player-only
fi

# 6. a terminal run without components: the wizard takes over
if [ "$wizard" = 1 ]; then
  say "handing over to crane setup"
  exec "$venv/bin/python" -m crane setup </dev/tty
fi
say "done. Next: crane doctor"
