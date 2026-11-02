// The dashboard's static files (coordinator/public), served like Workers static assets with
// run_worker_first = ["/v1/*"]: any other path that names a file is answered here with the _headers rules,
// "/" and "/dir/" serve index.html, and a miss falls through to the API (its JSON 404).
import { createHash } from "node:crypto";
import { readFileSync, readdirSync, statSync } from "node:fs";
import path from "node:path";

const TYPES: Record<string, string> = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".ico": "image/x-icon",
  ".webp": "image/webp",
  ".txt": "text/plain; charset=utf-8",
  ".wasm": "application/wasm",
};

interface Rule {
  re: RegExp;
  headers: [string, string][];
}

/** Cloudflare's _headers: a path pattern line (* matches anything), then indented "Name: value" lines. */
export function parseHeaders(text: string): Rule[] {
  const rules: Rule[] = [];
  let cur: Rule | null = null;
  for (const line of text.split("\n")) {
    if (!line.trim() || line.trim().startsWith("#")) continue;
    if (!/^\s/.test(line)) {
      const pat = line.trim();
      cur = { re: new RegExp(`^${pat.replace(/[.+?^${}()|[\]\\]/g, "\\$&").replace(/\*/g, ".*")}$`), headers: [] };
      rules.push(cur);
    } else if (cur) {
      const i = line.indexOf(":");
      if (i > 0) cur.headers.push([line.slice(0, i).trim(), line.slice(i + 1).trim()]);
    }
  }
  return rules;
}

interface Asset {
  body: Buffer;
  type: string;
  etag: string;
}

/** Loads public/ into memory once (a few files, ~70 KB). */
export class Assets {
  private files = new Map<string, Asset>();
  private rules: Rule[] = [];

  constructor(readonly dir: string) {
    const walk = (d: string) => {
      for (const f of readdirSync(d)) {
        const p = path.join(d, f);
        if (statSync(p).isDirectory()) walk(p);
        else if (f !== "_headers" && f !== "_redirects") {
          const body = readFileSync(p);
          const rel = "/" + path.relative(dir, p).split(path.sep).join("/");
          this.files.set(rel, {
            body,
            type: TYPES[path.extname(f).toLowerCase()] ?? "application/octet-stream",
            etag: `"${createHash("sha256").update(body).digest("hex").slice(0, 32)}"`,
          });
        }
      }
    };
    walk(dir);
    try {
      this.rules = parseHeaders(readFileSync(path.join(dir, "_headers"), "utf8"));
    } catch {}
  }

  /** The asset's response, or null to let the API answer. */
  serve(req: Request): Response | null {
    if (req.method !== "GET" && req.method !== "HEAD") return null;
    const url = new URL(req.url);
    if (url.pathname.startsWith("/v1/")) return null;
    let p: string;
    try {
      p = decodeURIComponent(url.pathname);
    } catch {
      return null;
    }
    let a = this.files.get(p);
    if (!a && p.endsWith("/")) a = this.files.get(p + "index.html");
    if (!a && !path.extname(p)) a = this.files.get(p + ".html");
    if (!a) return null;
    const headers = new Headers({ "content-type": a.type, etag: a.etag, "cache-control": "public, max-age=0, must-revalidate" });
    for (const r of this.rules) if (r.re.test(url.pathname)) for (const [k, v] of r.headers) headers.set(k, v);
    if (req.headers.get("if-none-match") === a.etag) return new Response(null, { status: 304, headers });
    headers.set("content-length", String(a.body.byteLength));
    return new Response(req.method === "HEAD" ? null : new Uint8Array(a.body), { headers });
  }
}
