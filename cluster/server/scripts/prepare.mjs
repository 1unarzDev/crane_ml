// Builds .build/core: the Worker's source (../coordinator: src, test, migrations, public) with hub.patch applied,
// the in-memory Hub the self-hosted server runs (it was reverted from the Worker on 2026-11-02, so it lives here as
// a patch). A change on main that conflicts with the patch fails here, loudly, instead of drifting. Also writes
// .build/vars.json, the Worker's [vars] from wrangler.toml, so the server's defaults are the Worker's.
import { execFileSync } from "node:child_process";
import { cpSync, existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import path from "node:path";

const here = path.dirname(new URL(import.meta.url).pathname);
const server = path.resolve(here, "..");
const coord = process.env.CRANE_COORDINATOR_DIR ?? path.resolve(server, "../coordinator");
const out = path.join(server, ".build", "core");

rmSync(out, { recursive: true, force: true });
mkdirSync(out, { recursive: true });
for (const d of ["src", "test", "migrations", "public"]) cpSync(path.join(coord, d), path.join(out, d), { recursive: true });
execFileSync("patch", ["-s", "-p3", "--no-backup-if-mismatch", "-d", out, "-i", path.join(server, "hub.patch")], { stdio: "inherit" });
for (const f of ["src/hub.ts", "src/hubstub.ts", "src/budget.ts", "test/hub.test.ts"])
  if (!existsSync(path.join(out, f))) throw new Error(`hub.patch did not produce ${f}`);

// [vars] of wrangler.toml: key = "value" lines (comments after # ignored).
const vars = {};
let inVars = false;
for (const line of readFileSync(path.join(coord, "wrangler.toml"), "utf8").split("\n")) {
  const t = line.trim();
  if (t.startsWith("[")) inVars = t === "[vars]";
  else if (inVars) {
    const m = /^([A-Z0-9_]+)\s*=\s*"([^"]*)"/.exec(t);
    if (m) vars[m[1]] = m[2];
  }
}
writeFileSync(path.join(server, ".build", "vars.json"), JSON.stringify(vars, null, 2) + "\n");
console.log(`prepare: ${out} (hub.patch applied), ${Object.keys(vars).length} vars`);
