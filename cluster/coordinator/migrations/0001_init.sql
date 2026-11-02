-- crane cluster coordinator schema v1.
-- *_at / created columns are ISO-8601 UTC strings (sortable); lease_expires and
-- leased_at_ms are epoch milliseconds because the lease logic does arithmetic on them.
-- JSON columns (*_json) hold objects/arrays validated by the API layer.

-- Key/value configuration: client release gate, retention overrides, ...
CREATE TABLE config (
  key        TEXT PRIMARY KEY,
  value      TEXT NOT NULL,
  updated_at TEXT NOT NULL
);

-- Workers. Capabilities are reported at join and refreshed on every lease call.
-- caps_json: {os, arch, cpu, ram_gb, gpu, gpu_count, vram_gb, vulkan, tags[], backends[]}
-- policy_json: the worker's availability policy as it reported it (quiet hours, idle-only, ...).
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

CREATE TABLE invites (
  code_sha256 TEXT PRIMARY KEY,
  created     TEXT NOT NULL,
  used_by     TEXT REFERENCES workers(id),
  used_at     TEXT
);

-- Read-only tokens (status views, the hypothesis CLI's reads).
CREATE TABLE viewer_tokens (
  token_sha256 TEXT PRIMARY KEY,
  name         TEXT NOT NULL,
  created      TEXT NOT NULL
);

-- Repositories known to the cluster. main_version is set by the GitHub webhook
-- (or by an admin) and tells workers which commit "main" means right now.
CREATE TABLE repos (
  name         TEXT PRIMARY KEY,    -- e.g. "owner/repo"
  url          TEXT,
  main_branch  TEXT NOT NULL DEFAULT 'main',
  main_version TEXT,
  updated_at   TEXT
);

-- Hypotheses: a claim tested through staged promotion. Stage index walks
-- stages_json; past the last stage the hypothesis becomes 'candidate'.
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
);

-- One job = one backend invocation over a list of seeds at exact repo refs.
-- needs_json: {os[], gpu, vulkan, min_cpu, min_ram_gb, min_vram_gb, tags[]}
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
);
CREATE INDEX jobs_lease_idx  ON jobs(status, priority, created);
CREATE INDEX jobs_expiry_idx ON jobs(status, lease_expires);
CREATE INDEX jobs_hyp_idx    ON jobs(hypothesis_id, stage_index) WHERE hypothesis_id IS NOT NULL;

-- Lease history: one row per (job, worker, attempt), written by trigger so it
-- is atomic with the lease UPDATE.
CREATE TABLE leases (
  job_id       TEXT NOT NULL,
  worker_id    TEXT NOT NULL,
  attempt      INTEGER NOT NULL,
  leased_at_ms INTEGER NOT NULL,
  outcome      TEXT,               -- done, failed, expired, released
  PRIMARY KEY (job_id, worker_id, attempt)
);
CREATE INDEX leases_worker_idx ON leases(worker_id, job_id);

CREATE TRIGGER jobs_lease_log AFTER UPDATE OF status ON jobs
WHEN NEW.status = 'leased' AND NEW.lease_worker IS NOT NULL
BEGIN
  INSERT OR IGNORE INTO leases(job_id, worker_id, attempt, leased_at_ms)
  VALUES (NEW.job_id, NEW.lease_worker, NEW.attempt, NEW.leased_at_ms);
END;

-- One run = one seed of one job attempt.
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
CREATE INDEX runs_job_idx ON runs(job_id);

CREATE TABLE metrics (
  run_id TEXT NOT NULL REFERENCES runs(run_id),
  name   TEXT NOT NULL,
  value  REAL NOT NULL,
  PRIMARY KEY (run_id, name)
);

-- R2 artifacts. retention: 'summary' kept forever, 'raw' until expires_at unless pinned.
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
);
CREATE INDEX artifacts_job_idx    ON artifacts(job_id);
CREATE INDEX artifacts_expiry_idx ON artifacts(retention, pinned, expires_at) WHERE deleted_at IS NULL;
