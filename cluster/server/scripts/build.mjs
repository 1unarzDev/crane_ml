// Bundles the server (dist/main.mjs) and the migration tool (dist/migrate.mjs) with esbuild, after prepare.mjs, and
// copies the migrations and the dashboard next to them. dist/ is all the runtime image needs (no node_modules).
import { execFileSync } from "node:child_process";
import { cpSync, rmSync } from "node:fs";
import path from "node:path";
import { build } from "esbuild";

const server = path.resolve(path.dirname(new URL(import.meta.url).pathname), "..");
execFileSync(process.execPath, [path.join(server, "scripts/prepare.mjs")], { stdio: "inherit" });
const dist = path.join(server, "dist");
rmSync(dist, { recursive: true, force: true });
await build({
  entryPoints: { main: path.join(server, "src/main.ts"), migrate: path.join(server, "src/migrate.ts") },
  outdir: dist,
  outExtension: { ".js": ".mjs" },
  bundle: true,
  platform: "node",
  target: "node24",
  format: "esm",
  sourcemap: true,
  alias: { "cloudflare:workers": path.join(server, "src/shims/workers.ts") },
  banner: { js: "import { createRequire as __cr } from 'node:module'; const require = __cr(import.meta.url);" },
  logLevel: "info",
});
cpSync(path.join(server, ".build/core/migrations"), path.join(dist, "migrations"), { recursive: true });
cpSync(path.join(server, ".build/core/public"), path.join(dist, "public"), { recursive: true });
