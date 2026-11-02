-- The feed (and the Hub's history refresh) look run_meta up by job: without this index every refresh
-- scanned the whole table.
CREATE INDEX IF NOT EXISTS run_meta_job_idx ON run_meta(job_id);
