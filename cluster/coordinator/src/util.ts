export interface Env {
  DB: D1Database;
  BUCKET: R2Bucket;
  ADMIN_TOKEN: string;
  GITHUB_WEBHOOK_SECRET?: string;
  MIN_CLIENT_VERSION?: string;
  LEASE_TTL_SECONDS?: string;
  MAX_ATTEMPTS?: string;
  MAX_LEASE_JOBS?: string;
  RAW_RETENTION_DAYS?: string;
  MAX_ARTIFACT_MB?: string;
}

export class HttpError extends Error {
  constructor(
    public status: number,
    message: string,
    public extra: Record<string, unknown> = {},
  ) {
    super(message);
  }
}

export function json(data: unknown, status = 200, headers: Record<string, string> = {}): Response {
  return new Response(JSON.stringify(data), {
    status,
    headers: { "content-type": "application/json; charset=utf-8", ...headers },
  });
}

export function errorResponse(e: HttpError): Response {
  return json({ error: e.message, ...e.extra }, e.status);
}

export async function readJson<T = Record<string, unknown>>(req: Request): Promise<T> {
  let body: unknown;
  try {
    body = await req.json();
  } catch {
    throw new HttpError(400, "invalid JSON body");
  }
  if (!isObject(body)) throw new HttpError(400, "JSON body must be an object");
  return body as T;
}

export function isObject(v: unknown): v is Record<string, unknown> {
  return v !== null && typeof v === "object" && !Array.isArray(v);
}

export function isStringArray(v: unknown): v is string[] {
  return Array.isArray(v) && v.every((x) => typeof x === "string");
}

export function parseJson<T>(s: string | null | undefined, dflt: T): T {
  if (!s) return dflt;
  try {
    return JSON.parse(s) as T;
  } catch {
    return dflt;
  }
}

const HEX = "0123456789abcdef";
export function toHex(buf: ArrayBuffer | Uint8Array): string {
  const b = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  let s = "";
  for (let i = 0; i < b.length; i++) s += HEX[b[i] >> 4] + HEX[b[i] & 15];
  return s;
}

export async function sha256Hex(data: string | ArrayBuffer | Uint8Array): Promise<string> {
  const bytes = typeof data === "string" ? new TextEncoder().encode(data) : data;
  return toHex(await crypto.subtle.digest("SHA-256", bytes));
}

export async function hmacSha256Hex(secret: string, data: ArrayBuffer): Promise<string> {
  const key = await crypto.subtle.importKey("raw", new TextEncoder().encode(secret), { name: "HMAC", hash: "SHA-256" }, false, ["sign"]);
  return toHex(await crypto.subtle.sign("HMAC", key, data));
}

/** Compare two strings without an early exit on the first differing byte. */
export function constantTimeEqual(a: string, b: string): boolean {
  if (a.length !== b.length) return false;
  let d = 0;
  for (let i = 0; i < a.length; i++) d |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return d === 0;
}

export function randomToken(nBytes = 32): string {
  const b = new Uint8Array(nBytes);
  crypto.getRandomValues(b);
  let bin = "";
  for (const x of b) bin += String.fromCharCode(x);
  return btoa(bin).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

export function randomId(prefix: string, nBytes = 8): string {
  const b = new Uint8Array(nBytes);
  crypto.getRandomValues(b);
  return prefix + toHex(b);
}

export function nowIso(ms = Date.now()): string {
  return new Date(ms).toISOString();
}

export function intVar(v: string | undefined, dflt: number): number {
  const n = v === undefined ? NaN : parseInt(v, 10);
  return Number.isFinite(n) ? n : dflt;
}

/** Dotted numeric version compare ("0.10.0" > "0.9.3"); suffixes are ignored. Returns -1/0/1. */
export function compareVersions(a: string, b: string): number {
  const parse = (v: string) =>
    (v.trim().replace(/^v/i, "").match(/^\d+(?:\.\d+)*/)?.[0] ?? "0").split(".").map(Number);
  const pa = parse(a);
  const pb = parse(b);
  for (let i = 0; i < Math.max(pa.length, pb.length); i++) {
    const x = pa[i] ?? 0;
    const y = pb[i] ?? 0;
    if (x !== y) return x < y ? -1 : 1;
  }
  return 0;
}

export function bearer(req: Request): string | null {
  const h = req.headers.get("authorization");
  if (!h) return null;
  const m = /^Bearer\s+(\S+)\s*$/i.exec(h);
  return m ? m[1] : null;
}
