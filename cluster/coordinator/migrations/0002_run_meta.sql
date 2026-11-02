-- crane cluster coordinator schema v2: run_meta, the index the ghost theater selects runs from.
-- One row per run that recorded a trajectory (crane-traj/1, a summary artifact kept forever).
-- study is any named series (a tuner session, a hypothesis, an RL generation); iteration orders it.
-- hypothesis_id and arm come from the run's job; everything else from the worker's run report.
CREATE TABLE run_meta (
  run_id         TEXT PRIMARY KEY REFERENCES runs(run_id),
  job_id         TEXT NOT NULL,
  study          TEXT NOT NULL,
  hypothesis_id  TEXT,
  arm            TEXT,
  iteration      INTEGER NOT NULL DEFAULT 0,
  generation     INTEGER NOT NULL DEFAULT 0,
  seed           INTEGER,
  params_json    TEXT NOT NULL DEFAULT '{}',
  params_sha256  TEXT,
  score          REAL,
  outcome        TEXT NOT NULL DEFAULT 'unknown'
                 CHECK (outcome IN ('finished','failed','timeout','invalid','unknown')),
  failure_reason TEXT,
  progress       REAL,
  valid          INTEGER,            -- 1, 0 or NULL (not reported)
  insight        TEXT,               -- one line from whoever ran the study, never generated here
  course_sha256  TEXT,
  course_family  TEXT,
  build_hash     TEXT,
  recorder       TEXT,               -- unity, fastsim, ...
  water_mode     TEXT,
  duration_s     REAL,
  traj_key       TEXT,               -- artifacts.key of the .ctraj (uploaded after this row)
  traj_sha256    TEXT,
  created        TEXT NOT NULL
);
CREATE INDEX run_meta_study_idx ON run_meta(study, iteration, score);
CREATE INDEX run_meta_hyp_idx   ON run_meta(hypothesis_id, iteration) WHERE hypothesis_id IS NOT NULL;
