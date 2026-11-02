-- crane cluster coordinator schema v4: storage tiers (design-storage.md section 2).
-- An artifact lives in R2 (hot) for HOT_DAYS[kind], is copied to an archive node (cold, content-addressed by
-- sha256), and is deleted from R2 only once archived, past its hot window and not protected.
--   r2_deleted_at: gone from R2 only (the archive still has it); deleted_at keeps its meaning: gone everywhere.
--   hot_at: the hot clock's start when an archived item was copied back into R2 (else created).
ALTER TABLE artifacts ADD COLUMN kind TEXT;
ALTER TABLE artifacts ADD COLUMN archived_at TEXT;
ALTER TABLE artifacts ADD COLUMN archive_bytes INTEGER;
ALTER TABLE artifacts ADD COLUMN archive_codec TEXT;
ALTER TABLE artifacts ADD COLUMN archive_node TEXT;
ALTER TABLE artifacts ADD COLUMN r2_deleted_at TEXT;
ALTER TABLE artifacts ADD COLUMN hot_at TEXT;

UPDATE artifacts SET r2_deleted_at = deleted_at WHERE deleted_at IS NOT NULL;
UPDATE artifacts SET kind = CASE
    WHEN name LIKE '%.lod.ctraj' THEN 'lod'
    WHEN name LIKE '%.ctraj' THEN 'ctraj'
    WHEN name LIKE '%.crane' OR name LIKE '%.crane.zst' THEN 'crane'
    WHEN name LIKE '%.tar.zst' OR name LIKE '%.tar.gz' OR name LIKE '%.tar.xz' THEN 'bundle'
    WHEN name LIKE '%.gif' THEN 'render_final'
    WHEN (name LIKE '%.mp4' OR name LIKE '%.mkv' OR name LIKE '%.webm')
         AND (name LIKE '%chunk%' OR name LIKE '%master%' OR name LIKE '%h264%') THEN 'render_master'
    WHEN name LIKE '%.mp4' OR name LIKE '%.mkv' OR name LIKE '%.webm' THEN 'render_final'
    WHEN retention = 'summary' THEN 'summary'
    ELSE 'raw' END;

CREATE INDEX artifacts_sha_idx     ON artifacts(sha256);
CREATE INDEX artifacts_archive_idx ON artifacts(archived_at, created) WHERE deleted_at IS NULL;
CREATE INDEX artifacts_run_idx     ON artifacts(run_id);

-- Archive nodes (crane-archive daemons). Any host; the name is free text. health_json is the heartbeat.
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

-- Archive tokens: read artifacts and call /v1/archive/*, nothing else (POST /v1/admin/archive_token).
CREATE TABLE archive_tokens (
  token_sha256 TEXT PRIMARY KEY,
  name         TEXT NOT NULL,
  created      TEXT NOT NULL
);

-- Kept in R2 whatever their age (and archived too: two copies, two sites): pinned artifacts, every artifact of
-- each study's best run, and the trajectories, summaries and renders of each iteration's best run.
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
