# Modular-fleet README curator (brief for the periodic agent)

The user asked for this on 2026-11-10: "have an agent update the readme periodically including the modular robot cad, the run visualizations and findings, and modular robotics testing scenes."

## What you produce each cycle
1. **Media:** curate `Docs/Fleet/media/` on branch `research/fleet` (worktree `~/worktrees/crane-fleet`) into these sections:
   - `cad/`: the current modular robot design. Take renders and comparison sheets from `Tools/Fleet/assets/cad_v1/images/` in `/home/lunarz/crane_ml` once they exist, else from `~/.cache/crane-fleet/assets_v1/previews/`, falling back to `assets_v0/previews/`. The newest design version is current, and older versions are marked superseded.
   - `runs/`: CRANE run visualizations: stills, replay frames and clip posters from `~/.cache/crane-fleet/stills/`, `~/.cache/crane-fleet/requests/*/replays/*/frames/` and `~/.cache/crane-fleet/jobs/*/`. Choose frames from the newest completed batches. Each frame's caption gives the run id and seed, the library commit, and the verdict (delivered / Capacity / tipped / ...).
   - `findings/`: figures from the studies, made from existing result files only. Use the papers' figures in `Docs/ResearchOrchestration/papers/fleet_p1/figures/`. Plot small tables from the handoffs in `Docs/ResearchOrchestration/AGENT_HANDOFFS/{fleet,capplan}/*.md` and decisions in `Docs/ResearchOrchestration/DECISIONS.md`: CRANE success/Capacity/tip batches, the XM cause table, the E2 arms that remain valid, caplearn/recon pilot results when they exist. Use matplotlib from `~/.cache/fleet-ml-venv` (install it there if missing).
     - Every figure states n, intervals where available, the decision ID, and "simulation only".
     - Label exploratory results "exploratory".
     - Never plot a withdrawn or suspended claim, for example D74 (suspended by D83). Check DECISIONS.md for D-entries that supersede earlier ones.
   - `scenes/`: modular-robotics testing scenes. Include:
     - the CRANE warehouse scene (`Tools/Fleet/spec/scenarios/*.json`);
     - fleet-game and MAPF instances rendered as simple top-down plots, from `Tools/Fleet/game/` scenes or instances, the mapf/xm instances and `Tools/Fleet/capplan/manifests` example configurations;
     - the Gazebo modular benchmark layouts if images already exist. Read-only: `~/mrmn-main`. Never touch `/home/lunarz/modular_robot_morphology_aware_navigation`'s checkout.
2. **`Docs/Fleet/media/README.md`:** one section per folder, each with a caption and render critique for every frame. The critique says what is physically valid and what is not: kinematic carry, proxy collision shapes, unverified overlaps. Put a "Current frames" list at the top: the DecisionCal README agent uses it and skips anything flagged as not physically valid.
3. **`Docs/Fleet/media/README_SECTION.md`:** a ready-to-paste markdown block, about 2 rows × 3 images plus a 3–5 bullet "latest findings" list with decision IDs, for the main crane_ml README. Use the same relative image paths as `README.md` at the repo root uses: `Docs/Fleet/media/...`.
4. **`README.md` on `research/fleet`:** a section `## Modular fleet (CRANE)`, after the existing `## Screenshots` grid, that equals `README_SECTION.md`. Do not edit the `## Screenshots` grid: the DecisionCal session owns it and will send cherry-picks.

## Rules
- **Images:** JPEG only, at most 300 KB each, at most 40 MB in total under `Docs/Fleet/media/`. Delete superseded frames from earlier cycles; git history keeps them.
- **Commits:** commit only `Docs/Fleet/media/**`, `Docs/Fleet/README_CURATOR.md` and the README section on `research/fleet`, then `git push origin research/fleet`. That push is authorized. Never touch main, other branches or MHSeals repos.
- **Don't disturb the CRANE integration agent**, which also commits on `research/fleet`:
  - If its working tree has uncommitted changes in your paths, skip the cycle.
  - Run `git pull --rebase` before pushing.
  - Never use `git add -A`; add your paths only.
- **Compute:** no Unity, no Gazebo, no heavy compute. Plotting and image conversion only, with nice -n 15.
- **Honesty:** if nothing new exists since the last cycle, change nothing and say so.
- **Notify:** afterwards, send the DecisionCal session the commit id and the paths of the current frames and of `README_SECTION.md`. Its address is `uds:/run/user/1000/cc-socks/2823592.sock`, name plan-simulation-experiment-selection; if that address no longer resolves, find it with ListAgents. Ask it to include the section in main's README.
- **Final report:** under 200 words, covering what changed, the commit id, and what was skipped and why.
