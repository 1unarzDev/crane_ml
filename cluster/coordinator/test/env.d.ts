import type { D1Migration } from "cloudflare:test";

declare global {
  namespace Cloudflare {
    interface Env {
      DB: D1Database;
      HUB: DurableObjectNamespace<import("../src/hub").Hub>;
      BUCKET: R2Bucket;
      ADMIN_TOKEN: string;
      GITHUB_WEBHOOK_SECRET: string;
      MIN_CLIENT_VERSION: string;
      LEASE_TTL_SECONDS: string;
      MAX_ATTEMPTS: string;
      MAX_LEASE_JOBS: string;
      RAW_RETENTION_DAYS: string;
      MAX_ARTIFACT_MB: string;
      TEST_MIGRATIONS: D1Migration[];
      GOLDEN_CTRAJ: string;
      GOLDEN_JSON: string;
    }
  }
}
export {};
