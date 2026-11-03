# viewer: the fast-tier web viewer

One page with three views over crane-traj/1 runs (`theater/traj.py`) and judge reports
(`crane.judge.report/1`): the run in depth, two tiers on one clock, and a batch at a glance. It is the
build of the three prototypes chosen in D-20261102-23 (A as the base, B's tier alignment, C's batch views).
The page runs locally (`python -m viewer serve`) and on the dashboard at `crane.lunarz.dev/viewer/`, the
run page's "Deep view". It draws and aligns; scores, flags and judge claims come from the data, and the
two statistics it shows (Spearman, MMRV) are computed by the server.

| route | view | what it answers |
|---|---|---|
| `#/run/<id>[?vs=<id>&t=&align=]` | chart room | Why did this run do that, at this moment? The course, the speed-ramped track, the hull and its goal, the other tier as a ghost with residual whiskers, contacts and the judge's spans on the map; an eight-lane timeline (task, decisions, behaviour, judge, speed, \|Δxy\|, perception, real); an inspector with the decision in force, referee rows, reliability flags and the judge's insights (click to jump). |
| `#/tiers/<a>/<b>[?t=&align=]` | tier bench | Where and when do the tiers disagree, and is the alignment fair? Linked maps (hover one, see the matched point on the other), the clock-warp diagram (shared anchors, 5 s stretch ticks, scrub on either axis), residual strips, the judge's pair record (gap origin per segment, first differing decision, code mismatch), the G4 table and rank agreement over every pair. |
| `#/batch/<study>[?seed=&focus=]` | batch board | Which variant is better across courses, and where does each fail? The variant × scenario points matrix, per-variant stats, score ECDFs, the per-seed Δ against a baseline, the ghost race with a live leaderboard, a lap chart and the focused variant's small multiples. |

Alignment (`web/core.js`): **event** warps piecewise-linearly through anchors both runs share
(task_started, each gate cleared, task_finished only when both finished the same way; past the last
anchor only the offset carries); **progress** matches fraction of path length and hides residuals when the
paths differ by more than 1.5×; **relative** starts both clocks at task_started. The first-divergence marker
(|Δxy| over 1 m for 2 s) is geometry, not a verdict. Channels use the present / empty / absent / invalid
grammar from the run header (`channels`), and the real tier is drawn as absent until a source adapter exists.

## Use

```sh
python3 -m viewer ingest ~/.cache/fastsim-pairs --out DATA          # a paired archive (index.jsonl + pairs/)
python3 -m viewer ingest RUN_DIR... --out DATA --tier fast --study S # plain run dirs
python3 -m viewer serve DATA JUDGE_DIR...                            # http://127.0.0.1:8765/
python3 -m viewer serve DATA --remote https://crane.lunarz.dev       # runs not found locally come from the cluster
python3 -m viewer serve DATA --feed-only                             # the page as the dashboard sees it
python3 -m viewer shot 'http://127.0.0.1:8765/#/batch/S' out.png     # headless screenshot, never a window
python3 -m unittest viewer.tests.test_viewer
```

`serve` finds `*.ctraj` (with `run.lod.ctraj` beside them), `courses/<sha>.json` and judge `report.json`
(with `feed.json`, `verdict.txt`) under its roots. Run ids are the header's `run_id`; the judge's anchors
`[run_id, t0, t1, channels, label]` are on the run's own clock (`t0_s` is its first pose), and the page
shows every time from the run's task_started.

## Pieces

| path | role |
|---|---|
| `ingest.py` | run dirs (the fastsim / mhseals_testing layout) -> `run.ctraj` + LOD: pose at 10 Hz, task events, decisions, goals in the world frame, behaviour, referee rows, contacts, near misses, channel status |
| `store.py` | the local store in the coordinator's feed shapes, the run index (studies, tier pairs, rank agreement) and the judge's evidence per run |
| `server.py` | stdlib HTTP server: the page, `/ctraj.js` (`theater/web/ctraj.js`), `/v1/feed/*` as in research-cluster `coordinator/src/replay.ts`, and the local-only `/v1/viewer/index` and `/v1/viewer/judge` |
| `web/` | no build step: `app.js` (routes), `data.js` (sources, run model, judge evidence), `core.js` (sampling, alignment, palettes), `run.js`, `tiers.js`, `batch.js`, `viewer.css` |

On the dashboard the page reads the public feed only (no judge reports, headers cut to public keys), so
it says what it cannot show rather than guessing. research-cluster's `coordinator/deploy.sh` copies `web/` and the
codec into `public/viewer/` at deploy time.
