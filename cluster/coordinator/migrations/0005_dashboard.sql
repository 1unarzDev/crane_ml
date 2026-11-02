-- crane cluster coordinator schema v5: the dashboard's replay lookups and theater render requests.
-- Operator tokens (POST /v1/admin/operator_token) can request a Unity theater render from the public page
-- and nothing else. A render request names a job or a study, says why (purpose, like every job) and moves
-- requested -> rendering -> done | failed | cancelled as whoever runs `theater.render` reports it; a done
-- render may carry a video in R2 (renders/<id>/<name>), served publicly with Range.
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
CREATE INDEX theater_renders_job_idx   ON theater_renders(job_id) WHERE job_id IS NOT NULL;
CREATE INDEX theater_renders_study_idx ON theater_renders(study) WHERE study IS NOT NULL;
CREATE INDEX theater_renders_status_idx ON theater_renders(status, created);
-- Ghosts: the same seed in the same study; courses by truth hash.
CREATE INDEX run_meta_seed_idx   ON run_meta(study, seed);
CREATE INDEX run_meta_course_idx ON run_meta(course_sha256) WHERE course_sha256 IS NOT NULL;
