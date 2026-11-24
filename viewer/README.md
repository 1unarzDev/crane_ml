# viewer: the web replay board (`crane board`)

```sh
crane board runs/ JUDGE_DIR...                                      # http://127.0.0.1:8765/ (python3 -m viewer serve)
python3 -m viewer serve DATA --remote https://crane.lunarz.dev       # runs not found locally come from the cluster
python3 -m viewer serve DATA --feed-only                             # the page as the dashboard sees it
python3 -m viewer ingest runs/ x.ctraj --out DATA --study S          # a self-contained copy, as served, with LODs
python3 -m viewer shot 'http://127.0.0.1:8765/#/batch/S' out.png     # headless screenshot, never a window
python3 -m unittest viewer.tests.test_viewer
```

One page with three views over runs and judge reports (`crane.judge.report/1`). It runs locally and on the
dashboard at `crane.lunarz.dev/viewer/` (the run page's "Deep view"). It draws and aligns. Scores, flags and judge
claims come from the data; the two statistics it shows (Spearman, MMRV) are computed by the server.

| route | view | what it answers |
|---|---|---|
| `#/run/<id>[?vs=<id>&t=&align=]` | run | Why did this run do that, at this moment? The world, the speed-ramped track, the vehicle outline and its goal, constraint volumes, a second run as a ghost with residual whiskers, contacts and judge spans on the map; one timeline lane per event lane the runs carry (task, mission, decisions, goals, judge, log, ...), speed and \|Δxy\|; an inspector with the decision in force, referee rows and the judge's insights. |
| `#/tiers/<a>/<b>[?t=&align=]` | tier bench | Where and when do two tiers disagree, and is the alignment fair? Linked maps, the clock-warp diagram, residual strips, the judge's pair record, the G4 table and rank agreement over every pair. |
| `#/batch/<study>[?seed=&focus=]` | batch board | Which variant is better across scenarios, and where does each fail? The variant × scenario matrix, per-variant stats, score ECDFs, the per-seed Δ against a baseline, the ghost race, a lap chart and small multiples. |

## What it serves
`serve` indexes, under its roots:
- **crane.run/1 records** (`crane.run.json`, or `run.json` with that schema). A record is served as one
  crane-traj/1 file: its trajectory (an event-kind run gets an empty one), with the meta's id, tier, kind,
  domain, frame and clock in the header. The header also carries its constraint `layers` (meta or
  `layers.json`) and the domain's vehicle `outline`. Its `events.jsonl` is merged into the events, each with a
  `lane` (log lines on `log`).
- **bare `*.ctraj`** files not inside a record, with `run.lod.ctraj` beside them when present.
- course truths (`courses/<sha>.json`, or the scenario file a record keeps) and judge `report.json` (with
  `feed.json` and `verdict.txt`).

Run ids are the meta's `run_id`, else the header's, else the path under the root. A fastsim batch
`OUT/<variant>/s<seed>` is study OUT, arm `<variant>`. Judge anchors `[run_id, t0, t1, channels, label]` are
on the run's own clock.

Alignment (`web/core.js`):
- **event** warps piecewise-linearly through anchors both runs share;
- **progress** matches the fraction of path length;
- **relative** starts both clocks at `task_started`.

The first-divergence marker (\|Δxy\| over 1 m for 2 s) is geometry, not a verdict.

## Pieces

| path | role |
|---|---|
| `store.py` | the index: records, bare files, courses, judge reports; the coordinator's feed shapes (studies, tier pairs, rank agreement) |
| `server.py` | stdlib HTTP server: the page, `/ctraj.js` (`theater/web/ctraj.js`), `/v1/feed/*` as in research-cluster `coordinator/src/replay.ts`, and the local-only `/v1/viewer/index` and `/v1/viewer/judge` |
| `ingest.py` | runs under any roots -> `DATA/runs/<id>/run.ctraj` + LOD + `DATA/courses` |
| `web/` | no build step: `app.js` (routes), `data.js` (sources, run model), `core.js` (sampling, alignment, outlines, constraint shapes), `run.js`, `tiers.js`, `batch.js`, `viewer.css` |

On the dashboard the page reads the public feed only (no judge reports, headers cut to public keys), so it
says what it cannot show rather than guessing. research-cluster's `coordinator/deploy.sh` copies `web/*` and
the codec into `public/viewer/`, so `web/` stays flat (no subdirectories).
