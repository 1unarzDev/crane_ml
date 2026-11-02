import { readFileSync } from "node:fs";
import path from "node:path";
import { cloudflareTest, readD1Migrations } from "@cloudflare/vitest-plugin";
import { defineConfig } from "vitest/config";

export default defineConfig(async () => {
  const migrations = await readD1Migrations(path.join(import.meta.dirname, "migrations"));
  // The crane-traj golden pair the Python writer makes (theater/tests/golden): public/traj.js is tested on it.
  const golden = path.join(import.meta.dirname, "../../theater/tests/golden");
  return {
    plugins: [
      cloudflareTest({
        wrangler: { configPath: "./wrangler.toml" },
        miniflare: {
          bindings: { ADMIN_TOKEN: "test-admin-token", GITHUB_WEBHOOK_SECRET: "test-hook-secret", FEED_CACHE_S: "0", ARCHIVE_URL: "", TEST_MIGRATIONS: migrations,
            GOLDEN_CTRAJ: readFileSync(path.join(golden, "small.ctraj")).toString("base64"),
            GOLDEN_JSON: readFileSync(path.join(golden, "small.json"), "utf8") },
        },
      }),
    ],
    test: {
      setupFiles: ["./test/apply-migrations.ts"],
      // Files run one at a time: tests share one database and some reset tables or set global
      // config (client gate, retention).
      fileParallelism: false,
    },
  };
});
