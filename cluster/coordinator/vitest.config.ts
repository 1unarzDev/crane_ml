import path from "node:path";
import { cloudflareTest, readD1Migrations } from "@cloudflare/vitest-plugin";
import { defineConfig } from "vitest/config";

export default defineConfig(async () => {
  const migrations = await readD1Migrations(path.join(import.meta.dirname, "migrations"));
  return {
    plugins: [
      cloudflareTest({
        wrangler: { configPath: "./wrangler.toml" },
        miniflare: {
          bindings: { ADMIN_TOKEN: "test-admin-token", GITHUB_WEBHOOK_SECRET: "test-hook-secret", TEST_MIGRATIONS: migrations },
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
