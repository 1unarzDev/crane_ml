-- crane cluster coordinator schema v3: every job says why it runs and how long a run should take.
-- purpose: 1-2 sentences by the submitting agent (what hypothesis, or what for); rows from before this
-- migration read "(legacy)". est_seconds: the submitter's estimate per run (per seed). submitter: the agent or
-- session label. campaign: an optional link to a campaign, boat test or study. judge: an optional one-line
-- verdict posted after the fact (POST /v1/admin/jobs/judge). The public feed (/v1/feed) shows all of them.
ALTER TABLE jobs ADD COLUMN purpose TEXT NOT NULL DEFAULT '(legacy)';
ALTER TABLE jobs ADD COLUMN est_seconds REAL;
ALTER TABLE jobs ADD COLUMN submitter TEXT;
ALTER TABLE jobs ADD COLUMN campaign TEXT;
ALTER TABLE jobs ADD COLUMN judge TEXT;
-- ended_at: when the job reached done, dead or cancelled (the feed's recent list orders by it).
ALTER TABLE jobs ADD COLUMN ended_at TEXT;
UPDATE jobs SET ended_at = done_at WHERE done_at IS NOT NULL;
UPDATE jobs SET ended_at = created WHERE ended_at IS NULL AND status IN ('dead','failed','cancelled');
-- The feed's duration model reads the most recent runs.
CREATE INDEX runs_created_idx ON runs(created);
CREATE INDEX jobs_ended_idx ON jobs(ended_at) WHERE ended_at IS NOT NULL;

-- A hypothesis's stage jobs inherit its purpose, campaign and estimate (a stage's est_seconds overrides it).
ALTER TABLE hypotheses ADD COLUMN purpose TEXT NOT NULL DEFAULT '(legacy)';
ALTER TABLE hypotheses ADD COLUMN est_seconds REAL;
ALTER TABLE hypotheses ADD COLUMN campaign TEXT;
