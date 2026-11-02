// Runs the coordinator's own suites (.build/core/test, made by scripts/prepare.mjs: main's tests plus the Hub's)
// on the self-hosted runtime, plus this package's tests. CRANE_TEST_HTTP=1 sends every suite request over HTTP.
import path from "node:path";
import { defineConfig } from "vitest/config";

const here = import.meta.dirname;
const http = process.env.CRANE_TEST_HTTP === "1";

export default defineConfig({
  resolve: {
    alias: {
      "cloudflare:test": path.join(here, "test/cf-test.ts"),
      "cloudflare:workers": path.join(here, "src/shims/workers.ts"),
    },
  },
  plugins: [
    {
      name: "crane-http-mode",
      enforce: "pre",
      resolveId(source, importer) {
        if (http && source === "../src/index" && importer?.endsWith(path.join(".build", "core", "test", "helpers.ts")))
          return path.join(here, "test/http-index.ts");
        return null;
      },
    },
  ],
  test: {
    include: [".build/core/test/**/*.test.ts", "test/**/*.test.ts"],
    setupFiles: [".build/core/test/apply-migrations.ts"],
    fileParallelism: false,
    // D1-only: SQLite does not report rows scanned, so D1's meta.rows_read sums cannot be matched here.
    testNamePattern: /^(?!.*counts rows read from each statement's meta)/,
    testTimeout: 30_000,
  },
});
