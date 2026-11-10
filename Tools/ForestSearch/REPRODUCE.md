# Reproducing ForestSearch results, tables, figures and the PDF

All commands run from `Tools/` (the directory that contains the `ForestSearch/` package) unless stated.
Raw CSVs live in `PerformanceResults/forest_search/` (gitignored). Compact gzip copies of every per-episode file
that backs a table are committed in `ForestSearch/results/data/` with hashes in `results/data/DATA_MANIFEST.json`.
Set `FS_DATA=<dir>` to point scripts at another raw directory; `make_hypotheses.py` and `stats_sensitivity.py` fall back to
`results/data/*.csv.gz` automatically when the raw CSVs are absent.

## 1. Environment

| item | version used | notes |
|---|---|---|
| Python | 3.14.9 | simulation code is pure Python + numpy |
| numpy | 2.5.4 | only hard dependency for simulations, `make_hypotheses.py`, `stats_sensitivity.py` (no scipy) |
| matplotlib | needed for figures and `analyze_curve.py --fig` | the system python has none; use a venv (`~/.cache/research/fs-venv` was used for figures) |
| TinyTeX v2026.11 (TeX Live 2026) | for the PDF | `paper/build_pdflatex.sh`; extra tlmgr packages: newtx fontaxes kastrup xstring zref etoolbox xpatch carlisle placeins caption courier helvetic times psnfss tex-gyre. Figures use `text.usetex` (fig 3) so LaTeX must be on PATH when making figures. |
| tectonic | fallback only | `paper/build.sh`; not the submission artifact |

Run simulations under the bounded-CPU wrapper: prefix with `~/.cache/research/fs-scripts/fsrun` (3 slots, nice 19).
Seeds are fixed everywhere (instances by `grid_instance(seed=...)`, episode index = episode RNG seed), so simulation CSVs are bit-reproducible
for a given Python/numpy version. Verify with `results/data/DATA_MANIFEST.json` (sha256 of raw and compact files).

## 2. Preregistration timing (honest statement)

* `PREREG.md` file mtime: **2026-11-09 13:47:05 -0600**; `confirm_manifest.txt` (code hashes) 13:47:07; earliest confirmatory CSV 13:52:43;
  last 13:57:13. By mtime the freeze preceded the held-out data by ~5 minutes.
* `git log`: `PREREG.md` was first committed in `6158f00` at **2026-11-09 18:06:23 -0600**, about 4 h after the results existed (the whole ForestSearch
  directory entered git in that one commit). No commit history therefore shows the freeze before the data. The timing claim rests on file mtimes, which are
  forgeable and were not independently witnessed. The code hashes in `confirm_manifest.txt` match the committed `.py` files (checked by reviewer B).
* The PREREG text fixes B=2000 and `analyze_curve.py`, but not the bootstrap RNG seed (`make_hypotheses.py` documents the seed it reproduces with).
* H1-H5 definitions were written after the dev run (`curve_dev_*`, seeds 100-102); dev data informed the hypotheses and the 0.03 margin.

## 3. Table / figure to command map

Runtimes are for one niced core on this machine (indicative; the shared box is slower under load). "raw" = needs the simulator; "data" = runs from `results/data/` in seconds.

| paper item | file | command | time |
|---|---|---|---|
| Table 2 (verdicts, H1-H5) | `results/hypotheses.md` | `python3 -m ForestSearch.make_hypotheses` | ~3 min (8 files x bootstrap), data |
| Table 2 inputs: confirmatory simulation | `confirm_{5x5,6x6}_t{0,1}.csv` | `python3 -m ForestSearch.event_curve --grid 5x5 --travel 0 --seeds 200 201 202 203 204 205 206 207 --episodes 300 --names ORACLE IMPLICIT SECTOR INDEP FULL HYBRID HYBRID_P --out PerformanceResults/forest_search/confirm_5x5_t0.csv` (travel 1: `--travel 1`; horizon defaults to 2*w*h as in PREREG; the other three files likewise) | ~6 min per file |
| full CI tables (`confirm.md`, `confirm.png`) | `results/confirm.md` | `python3 -m ForestSearch.analyze_curve PerformanceResults/forest_search/confirm_*_t?.csv --md results/confirm.md --fig results/confirm.png` | ~10 min |
| Robustness of the verdicts (stats review) | `results/stats_sensitivity.md` | `python3 -m ForestSearch.stats_sensitivity` | ~2 min, data |
| Fig. 1 | `paper/figs/fig1_confirm.pdf` | `python3 -m ForestSearch.make_fig1 PerformanceResults/forest_search ForestSearch/paper/figs/fig1_confirm.pdf` (B=2000, as in the caption) | ~5 min, needs matplotlib |
| Table 4 (controls, `tab:controls`) | `results/controls.md`, `phantom.md`, `leasefix.md` | `python3 ForestSearch/scripts/controls_run.py`, `phantom_run.py`, `leasefix_run.py` (write to `results/_regen/`, diff against committed) | ~3 min each (measured via the dump below; CPU-slot queueing dominated) |
| per-episode data for Table 4 | `results/data/{controls,phantom,leasefix}_episodes.csv.gz` | `python3 ForestSearch/scripts/dump_episodes.py controls\|phantom\|leasefix` | ~2-4 min each |
| Table 5 (joint planner, `tab:joint`) row 1 | `results/joint_heldout.md` | simulate: `python3 ForestSearch/scripts/joint_heldout.py 200,201,202,203 PerformanceResults/forest_search/joint_heldout_a.csv` and `204,205,206,207 ... _b.csv`; aggregate: `python3 ForestSearch/scripts/joint_heldout_agg.py` | simulate ~2 min per seed; agg 1 min |
| Table 5 rows 2-4 | `results/joint_heldout_all.md` | simulate: `python3 ForestSearch/scripts/joint_cells_worker.py 200 205` (several workers may run in parallel; cells c1=5x5 t0 T16, c2=6x6 t0 T16, c3=6x6 t1); aggregate: `python3 ForestSearch/scripts/joint_cells_agg.py c1 c2 c3` | ~3 min per (cell, seed); agg 2 min |
| alt. single-cell drivers | - | `scripts/joint_heldout_cells.py` / `joint_heldout_cells2.py`: `N TRAVEL HORIZON OUT [first last]` (superseded by the worker above; kept because they produced the original per-cell files) | |
| Table 6 and Fig. 2 (agent loss, `tab:loss`) | `results/loss_baselines.md` | `python3 -m ForestSearch.loss_baselines --out ForestSearch/results/loss_baselines.md` (seeds 504-511, 200 eps, regimes R7a-d) then `python3 -m ForestSearch.make_fig2` | ~3 min (measured, 4 regimes x 8 seeds x 200 eps) |
| per-episode data for Table 6 | `results/data/loss_episodes.csv.gz` | `python3 ForestSearch/scripts/dump_episodes.py loss` | ~3 min (measured, 4 regimes x 8 seeds x 200 eps) |
| N-agent pilot paragraph | `results/nagent_pilot.md` | `python3 -m ForestSearch.nagent --out ForestSearch/results/nagent_pilot.md` | unmeasured (est. ~1 h) |
| Exact DP / provenance tables | `exact*.md`, `alternation.md` | `python3 -m ForestSearch.exact_provenance`, `python3 -m ForestSearch.exact_dec` | minutes |
| Silent joint fixes | `joint_silent.md` | `python3 -m ForestSearch.joint_silent heldout --out ...` | |
| Fig. 3 (schematic) | `paper/figs/fig3_mechanism.pdf` | `python3 -m ForestSearch.make_fig3` | seconds, needs LaTeX |
| tab:protocols, tab:evidence | static LaTeX | none (hand-written) | |
| PDF | `paper/main.pdf` | `cd ForestSearch/paper && ./build_pdflatex.sh` (pdfLaTeX + BibTeX, TinyTeX on PATH or `TINYTEX_BIN`) | ~1 min |

## 4. Verification of what is regenerated

* `make_hypotheses.py` was run on the raw CSVs with RNG seed 0 (the `analyze_curve.paired` default, one generator per file, cells in table order).
  The output was byte-identical to the committed `results/hypotheses.md`; that file is therefore regenerable. The one-line H1 decision-rule footer is built from the data. Re-running from `results/data/*.csv.gz` (raw dir absent) is also byte-identical.
* `joint_cells_agg.py` reproduces the numbers of `joint_heldout_all.md` exactly (seed 12345). `joint_heldout_agg.py` reproduces the row-1 point estimates
  exactly; the CI upper end of the "joint ORACLE - greedy IMPLICIT" contrast was +0.024 vs +0.023 in the paper (bootstrap noise, the original aggregation script was not saved).

## 5. Known non-reproducible or incomplete items

* The original aggregation script for `joint_heldout.md` was not saved (replaced by `joint_heldout_agg.py`).
* `results/beacon_robustness.md` does not exist; `beacon_robustness.py` has not been run to completion (reviewer B issue 3). Seeds 800-811 untouched.
* N-agent pilot: only the summary markdown exists; no per-episode CSV (re-run needed).
* `/tmp/jh2` held 15 per-seed files (seeds 200-204 x 3 cells) plus `extra/c2_205.csv` (a stray seed 205); these are now archived in `results/data/`.
  Seeds 205-207 of the joint cells were never run (compute cap), as disclosed in `joint_heldout_all.md`.
* Bootstrap-based numbers vary in the last digit with the RNG stream; only `hypotheses.md` and `joint_cells_agg.py` outputs are stream-pinned.
* The exact digits in `controls.md`, `phantom.md`, `leasefix.md` depend on the `rng` stream order inside the original driver; `scripts/*_run.py` keep that order.
* `curve_dev_*`, `sweep.csv` and `falsify` per-episode data are not in the compact set beyond `curve_dev_*`; `falsify.md` and `sweep_summary.md` come from summary-only output.
