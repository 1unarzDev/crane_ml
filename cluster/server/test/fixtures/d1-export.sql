PRAGMA defer_foreign_keys=TRUE;
CREATE TABLE IF NOT EXISTS "d1_migrations"(
		id         INTEGER PRIMARY KEY AUTOINCREMENT,
		name       TEXT UNIQUE,
		applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
);
INSERT INTO "d1_migrations" ("id","name","applied_at") VALUES(1,'0001_init.sql','2026-11-02 22:19:20');
INSERT INTO "d1_migrations" ("id","name","applied_at") VALUES(2,'0002_run_meta.sql','2026-11-02 22:19:21');
INSERT INTO "d1_migrations" ("id","name","applied_at") VALUES(3,'0003_purpose.sql','2026-11-02 22:19:21');
INSERT INTO "d1_migrations" ("id","name","applied_at") VALUES(4,'0004_archive.sql','2026-11-02 22:19:22');
INSERT INTO "d1_migrations" ("id","name","applied_at") VALUES(5,'0005_dashboard.sql','2026-11-02 22:19:23');
CREATE TABLE config (
  key        TEXT PRIMARY KEY,
  value      TEXT NOT NULL,
  updated_at TEXT NOT NULL
);
INSERT INTO "config" ("key","value","updated_at") VALUES('raw_retention_days','21','2026-11-01T00:00:00Z');
CREATE TABLE workers (
  id             TEXT PRIMARY KEY,
  token_sha256   TEXT NOT NULL UNIQUE,
  name           TEXT NOT NULL,
  caps_json      TEXT NOT NULL DEFAULT '{}',
  policy_json    TEXT NOT NULL DEFAULT '{}',
  client_version TEXT,
  state          TEXT,              -- last self-reported state: idle, running, paused, ...
  stats_json     TEXT,
  created        TEXT NOT NULL,
  last_seen      TEXT,
  disabled       INTEGER NOT NULL DEFAULT 0
);
INSERT INTO "workers" ("id","token_sha256","name","caps_json","policy_json","client_version","state","stats_json","created","last_seen","disabled") VALUES('w_1','aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa','innovation','{"backends":["dummy"]}','{}','0.1.0',NULL,NULL,'2026-11-01T00:00:00Z',NULL,0);
CREATE TABLE invites (
  code_sha256 TEXT PRIMARY KEY,
  created     TEXT NOT NULL,
  used_by     TEXT REFERENCES workers(id),
  used_at     TEXT
);
CREATE TABLE viewer_tokens (
  token_sha256 TEXT PRIMARY KEY,
  name         TEXT NOT NULL,
  created      TEXT NOT NULL
);
CREATE TABLE repos (
  name         TEXT PRIMARY KEY,    -- e.g. "owner/repo"
  url          TEXT,
  main_branch  TEXT NOT NULL DEFAULT 'main',
  main_version TEXT,
  updated_at   TEXT
);
INSERT INTO "repos" ("name","url","main_branch","main_version","updated_at") VALUES('crane_ml','https://github.com/x/crane_ml','main','abc123',NULL);
CREATE TABLE hypotheses (
  id            TEXT PRIMARY KEY,
  name          TEXT NOT NULL,
  claim         TEXT NOT NULL,
  metric        TEXT NOT NULL,
  direction     TEXT NOT NULL CHECK (direction IN ('increase','decrease')),
  repo          TEXT,               -- repos.name the refs below live in (NULL: no code checkout)
  baseline_ref  TEXT NOT NULL,
  variants_json TEXT NOT NULL,      -- [{name, ref, params?}]
  scenario      TEXT,               -- scenario family, passed through to the backend
  seeds_json    TEXT NOT NULL,      -- [int]
  budget_json   TEXT NOT NULL DEFAULT '{}',
  stages_json   TEXT NOT NULL,      -- [{name, backend, params?, needs?}]
  promotion_json TEXT NOT NULL DEFAULT '{}',
  stage_index   INTEGER NOT NULL DEFAULT 0,
  status        TEXT NOT NULL DEFAULT 'running'
                CHECK (status IN ('running','candidate','rejected','cancelled')),
  decisions_json TEXT NOT NULL DEFAULT '[]',
  submitted_by  TEXT,
  created       TEXT NOT NULL,
  updated       TEXT NOT NULL
, purpose TEXT NOT NULL DEFAULT '(legacy)', est_seconds REAL, campaign TEXT);
CREATE TABLE jobs (
  job_id        TEXT PRIMARY KEY,
  hypothesis_id TEXT REFERENCES hypotheses(id),
  stage_index   INTEGER,
  arm           TEXT,               -- 'baseline' or a variant name (hypothesis jobs)
  backend       TEXT NOT NULL,
  repos_json    TEXT NOT NULL DEFAULT '[]',   -- [{name, url, ref}]
  spec_json     TEXT NOT NULL DEFAULT '{}',   -- backend params, scenario, metric spec
  seeds_json    TEXT NOT NULL DEFAULT '[0]',
  needs_json    TEXT NOT NULL DEFAULT '{}',
  priority      INTEGER NOT NULL DEFAULT 100, -- lower = served first
  status        TEXT NOT NULL DEFAULT 'queued'
                CHECK (status IN ('queued','leased','done','failed','dead','cancelled')),
  attempt       INTEGER NOT NULL DEFAULT 0,
  max_attempts  INTEGER,                      -- NULL: the coordinator default
  lease_worker  TEXT,
  lease_expires INTEGER,
  leased_at_ms  INTEGER,
  last_error    TEXT,
  done_at       TEXT,
  done_by       TEXT,
  created       TEXT NOT NULL
, purpose TEXT NOT NULL DEFAULT '(legacy)', est_seconds REAL, submitter TEXT, campaign TEXT, judge TEXT, ended_at TEXT);
INSERT INTO "jobs" ("job_id","hypothesis_id","stage_index","arm","backend","repos_json","spec_json","seeds_json","needs_json","priority","status","attempt","max_attempts","lease_worker","lease_expires","leased_at_ms","last_error","done_at","done_by","created","purpose","est_seconds","submitter","campaign","judge","ended_at") VALUES('j_q',NULL,NULL,NULL,'dummy','{}','{}','[1,2]','{}',5,'queued',0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'2026-11-01T00:00:00Z','A fixture job that checks the import keeps queued work leasable.',30,'fixture',NULL,NULL,NULL);
INSERT INTO "jobs" ("job_id","hypothesis_id","stage_index","arm","backend","repos_json","spec_json","seeds_json","needs_json","priority","status","attempt","max_attempts","lease_worker","lease_expires","leased_at_ms","last_error","done_at","done_by","created","purpose","est_seconds","submitter","campaign","judge","ended_at") VALUES('j_d',NULL,NULL,NULL,'dummy','{}','{}','[1]','{}',0,'done',1,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'2026-11-01T00:00:00Z','A fixture job that checks the import keeps finished history, it''s quoted.',30,'fixture',NULL,NULL,NULL);
CREATE TABLE leases (
  job_id       TEXT NOT NULL,
  worker_id    TEXT NOT NULL,
  attempt      INTEGER NOT NULL,
  leased_at_ms INTEGER NOT NULL,
  outcome      TEXT,               -- done, failed, expired, released
  PRIMARY KEY (job_id, worker_id, attempt)
);
CREATE TABLE runs (
  run_id     TEXT PRIMARY KEY,
  job_id     TEXT NOT NULL REFERENCES jobs(job_id),
  worker_id  TEXT NOT NULL,
  attempt    INTEGER NOT NULL,
  seed       INTEGER NOT NULL,
  status     TEXT NOT NULL CHECK (status IN ('ok','failed')),
  wall_s     REAL,
  error      TEXT,
  created    TEXT NOT NULL,
  UNIQUE (job_id, attempt, seed)
);
INSERT INTO "runs" ("run_id","job_id","worker_id","attempt","seed","status","wall_s","error","created") VALUES('j_d.a1.s1','j_d','w_1',1,1,'ok',3.5,NULL,'2026-11-01T00:00:00Z');
CREATE TABLE metrics (
  run_id TEXT NOT NULL REFERENCES runs(run_id),
  name   TEXT NOT NULL,
  value  REAL NOT NULL,
  PRIMARY KEY (run_id, name)
);
INSERT INTO "metrics" ("run_id","name","value") VALUES('j_d.a1.s1','score',0.75);
CREATE TABLE artifacts (
  key        TEXT PRIMARY KEY,
  job_id     TEXT,
  run_id     TEXT,
  name       TEXT NOT NULL,
  retention  TEXT NOT NULL CHECK (retention IN ('summary','raw')),
  size       INTEGER NOT NULL,
  sha256     TEXT NOT NULL,
  pinned     INTEGER NOT NULL DEFAULT 0,
  expires_at TEXT,                 -- NULL: never (summary, or pinned)
  deleted_at TEXT,
  created    TEXT NOT NULL
, kind TEXT, archived_at TEXT, archive_bytes INTEGER, archive_codec TEXT, archive_node TEXT, r2_deleted_at TEXT, hot_at TEXT);
INSERT INTO "artifacts" ("key","job_id","run_id","name","retention","size","sha256","pinned","expires_at","deleted_at","created","kind","archived_at","archive_bytes","archive_codec","archive_node","r2_deleted_at","hot_at") VALUES('artifacts/j_d/j_d.a1.s1/run.json','j_d','j_d.a1.s1','run.json','summary',2,'bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb',0,NULL,NULL,'2026-11-01T00:00:00Z','summary',NULL,NULL,NULL,NULL,NULL,NULL);
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
CREATE TABLE archive_nodes (
  name        TEXT PRIMARY KEY,
  root_kind   TEXT,                 -- dir, zfs, btrfs, ...
  total_bytes INTEGER,
  free_bytes  INTEGER,
  used_bytes  INTEGER,
  objects     INTEGER,
  state       TEXT,                 -- ok, degraded, full
  health_json TEXT NOT NULL DEFAULT '{}',
  last_seen   TEXT NOT NULL
);
CREATE TABLE archive_tokens (
  token_sha256 TEXT PRIMARY KEY,
  name         TEXT NOT NULL,
  created      TEXT NOT NULL
);
CREATE TABLE operator_tokens (
  token_sha256 TEXT PRIMARY KEY,
  name         TEXT NOT NULL,
  created      TEXT NOT NULL
);
CREATE TABLE theater_renders (
  id            TEXT PRIMARY KEY,
  job_id        TEXT,
  study         TEXT,
  run_id        TEXT,
  purpose       TEXT NOT NULL,
  requested_by  TEXT NOT NULL,
  status        TEXT NOT NULL DEFAULT 'requested'
                CHECK (status IN ('requested','rendering','done','failed','cancelled')),
  render_job_id TEXT,
  note          TEXT,
  video_key     TEXT,
  video_type    TEXT,
  video_size    INTEGER,
  video_sha256  TEXT,
  created       TEXT NOT NULL,
  updated       TEXT NOT NULL
);
DELETE FROM sqlite_sequence;
INSERT INTO "sqlite_sequence" ("name","seq") VALUES('d1_migrations',5);
CREATE INDEX jobs_lease_idx  ON jobs(status, priority, created);
CREATE INDEX jobs_expiry_idx ON jobs(status, lease_expires);
CREATE INDEX jobs_hyp_idx    ON jobs(hypothesis_id, stage_index) WHERE hypothesis_id IS NOT NULL;
CREATE INDEX leases_worker_idx ON leases(worker_id, job_id);
CREATE INDEX runs_job_idx ON runs(job_id);
CREATE INDEX artifacts_job_idx    ON artifacts(job_id);
CREATE INDEX artifacts_expiry_idx ON artifacts(retention, pinned, expires_at) WHERE deleted_at IS NULL;
CREATE INDEX run_meta_study_idx ON run_meta(study, iteration, score);
CREATE INDEX run_meta_hyp_idx   ON run_meta(hypothesis_id, iteration) WHERE hypothesis_id IS NOT NULL;
CREATE INDEX runs_created_idx ON runs(created);
CREATE INDEX jobs_ended_idx ON jobs(ended_at) WHERE ended_at IS NOT NULL;
CREATE INDEX artifacts_sha_idx     ON artifacts(sha256);
CREATE INDEX artifacts_archive_idx ON artifacts(archived_at, created) WHERE deleted_at IS NULL;
CREATE INDEX artifacts_run_idx     ON artifacts(run_id);
CREATE INDEX theater_renders_job_idx   ON theater_renders(job_id) WHERE job_id IS NOT NULL;
CREATE INDEX theater_renders_study_idx ON theater_renders(study) WHERE study IS NOT NULL;
CREATE INDEX theater_renders_status_idx ON theater_renders(status, created);
CREATE INDEX run_meta_seed_idx   ON run_meta(study, seed);
CREATE INDEX run_meta_course_idx ON run_meta(course_sha256) WHERE course_sha256 IS NOT NULL;
CREATE TRIGGER jobs_lease_log AFTER UPDATE OF status ON jobs
WHEN NEW.status = 'leased' AND NEW.lease_worker IS NOT NULL
BEGIN
  INSERT OR IGNORE INTO leases(job_id, worker_id, attempt, leased_at_ms)
  VALUES (NEW.job_id, NEW.lease_worker, NEW.attempt, NEW.leased_at_ms);
END;
CREATE VIEW protected_artifacts AS
  WITH ranked AS (
    SELECT run_id,
           ROW_NUMBER() OVER (PARTITION BY study ORDER BY score DESC, run_id) AS study_rank,
           ROW_NUMBER() OVER (PARTITION BY study, iteration ORDER BY score DESC, run_id) AS iter_rank
      FROM run_meta WHERE score IS NOT NULL AND COALESCE(valid, 1) = 1
  )
  SELECT key, sha256 FROM artifacts WHERE pinned = 1
  UNION
  SELECT a.key, a.sha256 FROM artifacts a JOIN ranked r ON r.run_id = a.run_id
   WHERE r.study_rank = 1
      OR (r.iter_rank = 1 AND a.kind IN ('ctraj', 'lod', 'summary', 'render_final'));