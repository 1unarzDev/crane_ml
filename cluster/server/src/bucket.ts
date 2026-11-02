// The R2Bucket subset the coordinator uses (get with Range, put, head, delete), over pluggable stores:
//   DiskBucket   local directory, the default hot store (sized by R2_SOFT_CAP_GB and the hot windows in archive.ts)
//   S3Bucket     any S3 API, e.g. R2 itself (https://<account>.r2.cloudflarestorage.com) with an R2 access key
//   CfApiBucket  read-only R2 through the Cloudflare API token (cutover: reads of objects not yet pulled)
//   ReadThrough  a primary plus a read-only fallback; a fallback hit is copied into the primary
// Cold storage stays the archive node (cluster/archive): archive.ts reads it on a miss, and it mirrors to Drive.
import { createHash, randomBytes } from "node:crypto";
import { createReadStream } from "node:fs";
import { mkdir, open, readFile, rename, rm, stat, writeFile } from "node:fs/promises";
import path from "node:path";
import { Readable } from "node:stream";
import { AwsClient } from "aws4fetch";

export interface PutOpts {
  customMetadata?: Record<string, string>;
  httpMetadata?: { contentType?: string; [k: string]: unknown } | Headers;
}
export type RangeOpt = { offset?: number; length?: number; suffix?: number } | Headers;
export interface GetOpts {
  range?: RangeOpt;
}
export type Body = ReadableStream | ArrayBuffer | ArrayBufferView | string | Blob | null;

export interface StoredMeta {
  key: string;
  size: number;
  etag: string;
  uploaded: string;
  customMetadata: Record<string, string>;
  httpMetadata: { contentType?: string };
}

export class BucketObject {
  readonly key: string;
  readonly size: number;
  readonly etag: string;
  readonly httpEtag: string;
  readonly uploaded: Date;
  readonly customMetadata: Record<string, string>;
  readonly httpMetadata: { contentType?: string };
  readonly version: string;
  readonly storageClass = "Standard";
  constructor(m: StoredMeta) {
    this.key = m.key;
    this.size = m.size;
    this.etag = m.etag;
    this.httpEtag = `"${m.etag}"`;
    this.uploaded = new Date(m.uploaded);
    this.customMetadata = m.customMetadata ?? {};
    this.httpMetadata = m.httpMetadata ?? {};
    this.version = m.etag;
  }
  writeHttpMetadata(h: Headers) {
    if (this.httpMetadata.contentType) h.set("content-type", this.httpMetadata.contentType);
  }
}

export class BucketObjectBody extends BucketObject {
  private used = false;
  constructor(
    m: StoredMeta,
    private stream: ReadableStream<Uint8Array>,
    readonly range?: { offset: number; length: number },
  ) {
    super(m);
  }
  get body(): ReadableStream<Uint8Array> {
    this.used = true;
    return this.stream;
  }
  get bodyUsed() {
    return this.used;
  }
  async arrayBuffer(): Promise<ArrayBuffer> {
    return new Response(this.body).arrayBuffer();
  }
  async bytes(): Promise<Uint8Array> {
    return new Uint8Array(await this.arrayBuffer());
  }
  async text(): Promise<string> {
    return new Response(this.body).text();
  }
  async json<T = unknown>(): Promise<T> {
    return JSON.parse(await this.text()) as T;
  }
  async blob(): Promise<Blob> {
    return new Response(this.body).blob();
  }
}

export interface Bucket {
  readonly kind: string;
  get(key: string, opts?: GetOpts): Promise<BucketObjectBody | null>;
  head(key: string): Promise<BucketObject | null>;
  put(key: string, body: Body, opts?: PutOpts): Promise<BucketObject>;
  delete(keys: string | string[]): Promise<void>;
}

async function toBytes(body: Body): Promise<Uint8Array> {
  if (body === null) return new Uint8Array();
  if (typeof body === "string") return new TextEncoder().encode(body);
  if (body instanceof ArrayBuffer) return new Uint8Array(body);
  if (ArrayBuffer.isView(body)) return new Uint8Array(body.buffer, body.byteOffset, body.byteLength);
  return new Uint8Array(await new Response(body as BodyInit).arrayBuffer());
}

function contentTypeOf(h: PutOpts["httpMetadata"]): string | undefined {
  if (!h) return undefined;
  if (h instanceof Headers) return h.get("content-type") ?? undefined;
  return typeof h.contentType === "string" ? h.contentType : undefined;
}

/** R2's range semantics: {offset, length}, {offset}, {suffix}, or a Range header (one bytes= range). */
export function resolveRange(r: RangeOpt | undefined, size: number): { offset: number; length: number } | undefined {
  if (!r) return undefined;
  let offset: number | undefined;
  let length: number | undefined;
  if (r instanceof Headers) {
    const m = /^bytes=(\d*)-(\d*)$/.exec((r.get("range") ?? "").trim());
    if (!m || (m[1] === "" && m[2] === "")) return undefined;
    if (m[1] === "") {
      const n = Math.min(Number(m[2]), size);
      return { offset: size - n, length: n };
    }
    offset = Number(m[1]);
    length = m[2] === "" ? undefined : Number(m[2]) - offset + 1;
  } else if (r.suffix !== undefined) {
    const n = Math.min(r.suffix, size);
    return { offset: size - n, length: n };
  } else {
    offset = r.offset ?? 0;
    length = r.length;
  }
  if (offset >= size) return { offset: size, length: 0 };
  const len = Math.max(0, Math.min(length ?? size - offset, size - offset));
  return { offset, length: len };
}

// ------------------------------------------------------------------ local disk

/** Objects under <root>/objects/ab/<sha256(key)> with a .json sidecar; writes are staged and renamed. */
export class DiskBucket implements Bucket {
  readonly kind = "disk";
  constructor(readonly root: string) {}

  private paths(key: string) {
    const h = createHash("sha256").update(key).digest("hex");
    const dir = path.join(this.root, "objects", h.slice(0, 2));
    return { dir, data: path.join(dir, h), meta: path.join(dir, `${h}.json`) };
  }

  private async meta(key: string): Promise<StoredMeta | null> {
    try {
      return JSON.parse(await readFile(this.paths(key).meta, "utf8")) as StoredMeta;
    } catch {
      return null;
    }
  }

  async head(key: string) {
    const m = await this.meta(key);
    return m ? new BucketObject(m) : null;
  }

  async get(key: string, opts?: GetOpts) {
    const m = await this.meta(key);
    if (!m) return null;
    const range = resolveRange(opts?.range, m.size);
    const p = this.paths(key).data;
    let stream: ReadableStream<Uint8Array>;
    if (range && range.length === 0) stream = new Response(new Uint8Array()).body!;
    else {
      const node = createReadStream(p, range ? { start: range.offset, end: range.offset + range.length - 1 } : {});
      stream = Readable.toWeb(node) as unknown as ReadableStream<Uint8Array>;
    }
    return new BucketObjectBody(m, stream, range);
  }

  async put(key: string, body: Body, opts: PutOpts = {}) {
    const bytes = await toBytes(body);
    const { dir, data, meta } = this.paths(key);
    await mkdir(dir, { recursive: true });
    const m: StoredMeta = {
      key,
      size: bytes.byteLength,
      etag: createHash("md5").update(bytes).digest("hex"),
      uploaded: new Date().toISOString(),
      customMetadata: { ...(opts.customMetadata ?? {}) },
      httpMetadata: contentTypeOf(opts.httpMetadata) ? { contentType: contentTypeOf(opts.httpMetadata) } : {},
    };
    const tmp = `${data}.${randomBytes(6).toString("hex")}.tmp`;
    const fh = await open(tmp, "w");
    try {
      await fh.writeFile(bytes);
      await fh.sync();
    } finally {
      await fh.close();
    }
    await rename(tmp, data);
    await writeFile(`${meta}.tmp`, JSON.stringify(m));
    await rename(`${meta}.tmp`, meta);
    return new BucketObject(m);
  }

  async delete(keys: string | string[]) {
    for (const k of Array.isArray(keys) ? keys : [keys]) {
      const { data, meta } = this.paths(k);
      await rm(meta, { force: true });
      await rm(data, { force: true });
    }
  }

  /** Bytes held (for /healthz). */
  async usage(): Promise<{ objects: number; bytes: number }> {
    const { readdir } = await import("node:fs/promises");
    let objects = 0;
    let bytes = 0;
    const base = path.join(this.root, "objects");
    let dirs: string[] = [];
    try {
      dirs = await readdir(base);
    } catch {
      return { objects, bytes };
    }
    for (const d of dirs)
      for (const f of await readdir(path.join(base, d))) {
        if (f.endsWith(".json") || f.endsWith(".tmp")) continue;
        objects++;
        bytes += (await stat(path.join(base, d, f))).size;
      }
    return { objects, bytes };
  }
}

// ------------------------------------------------------------------ S3 API (R2, MinIO, ...)

export interface S3Config {
  endpoint: string; // https://<account>.r2.cloudflarestorage.com
  bucket: string;
  accessKeyId: string;
  secretAccessKey: string;
  region?: string; // R2: auto
}

const enc = (key: string) => key.split("/").map(encodeURIComponent).join("/");

export class S3Bucket implements Bucket {
  readonly kind = "s3";
  private aws: AwsClient;
  constructor(private cfg: S3Config) {
    this.aws = new AwsClient({ accessKeyId: cfg.accessKeyId, secretAccessKey: cfg.secretAccessKey, service: "s3", region: cfg.region ?? "auto" });
  }
  private url(key: string) {
    return `${this.cfg.endpoint.replace(/\/$/, "")}/${this.cfg.bucket}/${enc(key)}`;
  }
  private metaOf(key: string, res: Response, size: number): StoredMeta {
    const custom: Record<string, string> = {};
    res.headers.forEach((v, k) => {
      if (k.startsWith("x-amz-meta-")) custom[k.slice(11)] = v;
    });
    const ct = res.headers.get("content-type");
    return {
      key,
      size,
      etag: (res.headers.get("etag") ?? "").replace(/"/g, ""),
      uploaded: new Date(res.headers.get("last-modified") ?? Date.now()).toISOString(),
      customMetadata: custom,
      httpMetadata: ct && ct !== "binary/octet-stream" && ct !== "application/octet-stream" ? { contentType: ct } : {},
    };
  }
  async head(key: string) {
    const res = await this.aws.fetch(this.url(key), { method: "HEAD" });
    if (res.status === 404) return null;
    if (!res.ok) throw new Error(`s3 head ${res.status}`);
    return new BucketObject(this.metaOf(key, res, Number(res.headers.get("content-length") ?? 0)));
  }
  async get(key: string, opts?: GetOpts) {
    let range: { offset: number; length: number } | undefined;
    const headers: Record<string, string> = {};
    if (opts?.range) {
      // The size is needed to resolve suffix and open ranges the way R2 reports them.
      const h = await this.head(key);
      if (!h) return null;
      range = resolveRange(opts.range, h.size);
      if (range && range.length > 0) headers.range = `bytes=${range.offset}-${range.offset + range.length - 1}`;
    }
    const res = await this.aws.fetch(this.url(key), { headers });
    if (res.status === 404) return null;
    if (!res.ok) throw new Error(`s3 get ${res.status}`);
    const cr = /\/(\d+)$/.exec(res.headers.get("content-range") ?? "");
    const size = cr ? Number(cr[1]) : Number(res.headers.get("content-length") ?? 0);
    const body = range && range.length === 0 ? new Response(new Uint8Array()).body! : res.body!;
    return new BucketObjectBody(this.metaOf(key, res, size), body, range);
  }
  async put(key: string, body: Body, opts: PutOpts = {}) {
    const bytes = await toBytes(body);
    const headers: Record<string, string> = {};
    for (const [k, v] of Object.entries(opts.customMetadata ?? {})) headers[`x-amz-meta-${k.toLowerCase()}`] = v;
    const ct = contentTypeOf(opts.httpMetadata);
    if (ct) headers["content-type"] = ct;
    const res = await this.aws.fetch(this.url(key), { method: "PUT", headers, body: bytes });
    if (!res.ok) throw new Error(`s3 put ${res.status}`);
    return new BucketObject({
      key,
      size: bytes.byteLength,
      etag: (res.headers.get("etag") ?? "").replace(/"/g, ""),
      uploaded: new Date().toISOString(),
      customMetadata: { ...(opts.customMetadata ?? {}) },
      httpMetadata: ct ? { contentType: ct } : {},
    });
  }
  async delete(keys: string | string[]) {
    for (const k of Array.isArray(keys) ? keys : [keys]) {
      const res = await this.aws.fetch(this.url(k), { method: "DELETE" });
      if (!res.ok && res.status !== 404) throw new Error(`s3 delete ${res.status}`);
    }
  }
}

// ------------------------------------------------------------------ R2 through the Cloudflare API (read-only)

export interface CfApiConfig {
  accountId: string;
  bucket: string;
  token: string;
  api?: string;
}

/** R2 objects through api.cloudflare.com with an API token (no S3 key needed). Read-only: writes are refused. */
export class CfApiBucket implements Bucket {
  readonly kind = "r2-api";
  constructor(private cfg: CfApiConfig) {}
  private base() {
    return `${this.cfg.api ?? "https://api.cloudflare.com/client/v4"}/accounts/${this.cfg.accountId}/r2/buckets/${this.cfg.bucket}`;
  }
  private auth() {
    return { authorization: `Bearer ${this.cfg.token}` };
  }
  /** One page of the listing: {objects: [{key, size, custom_metadata, http_metadata}], cursor}. */
  async list(cursor?: string, perPage = 1000, prefix?: string) {
    const u = new URL(`${this.base()}/objects`);
    u.searchParams.set("per_page", String(perPage));
    if (cursor) u.searchParams.set("cursor", cursor);
    if (prefix) u.searchParams.set("prefix", prefix);
    const res = await fetch(u, { headers: this.auth() });
    const d = (await res.json()) as {
      success: boolean;
      errors: unknown[];
      result: { key: string; size: number; etag: string; last_modified: string; custom_metadata?: Record<string, string>; http_metadata?: { contentType?: string } }[];
      result_info?: { cursor?: string; is_truncated?: boolean };
    };
    if (!d.success) throw new Error(`r2 list: ${JSON.stringify(d.errors).slice(0, 200)}`);
    return { objects: d.result, cursor: d.result_info?.is_truncated ? d.result_info.cursor : undefined };
  }
  async head(key: string) {
    const o = await this.get(key);
    if (!o) return null;
    await o.body.cancel();
    return o;
  }
  async get(key: string, opts?: GetOpts) {
    const res = await fetch(`${this.base()}/objects/${encodeURIComponent(key)}`, { headers: this.auth() });
    if (res.status === 404) return null;
    if (!res.ok) throw new Error(`r2 api get ${res.status}`);
    const bytes = new Uint8Array(await res.arrayBuffer());
    // The object GET carries no custom metadata (the sha256 the Worker stored): the listing has it.
    const listed = (await this.list(undefined, 1, key).catch(() => ({ objects: [] }))).objects.find((o) => o.key === key);
    const custom: Record<string, string> = { ...(listed?.custom_metadata ?? {}) };
    const ct = listed?.http_metadata?.contentType;
    const m: StoredMeta = {
      key,
      size: bytes.byteLength,
      etag: (res.headers.get("etag") ?? "").replace(/"/g, ""),
      uploaded: new Date(res.headers.get("last-modified") ?? Date.now()).toISOString(),
      customMetadata: custom,
      httpMetadata: ct ? { contentType: ct } : {},
    };
    const range = resolveRange(opts?.range, bytes.byteLength);
    const slice = range ? bytes.subarray(range.offset, range.offset + range.length) : bytes;
    return new BucketObjectBody(m, new Response(slice).body!, range);
  }
  async put(): Promise<BucketObject> {
    throw new Error("the Cloudflare API bucket is read-only");
  }
  async delete() {
    // Read-only fallback: deletes go to the primary only; R2 itself is cleaned up after the cutover.
  }
}

/** A primary store with a read-only fallback: a miss in the primary is read from the fallback and copied in. */
export class ReadThroughBucket implements Bucket {
  readonly kind: string;
  constructor(
    readonly primary: Bucket,
    readonly fallback: Bucket,
  ) {
    this.kind = `${primary.kind}+${fallback.kind}`;
  }
  async head(key: string) {
    return (await this.primary.head(key)) ?? (await this.fallback.head(key));
  }
  async get(key: string, opts?: GetOpts) {
    const hit = await this.primary.get(key, opts);
    if (hit) return hit;
    const far = await this.fallback.get(key);
    if (!far) return null;
    const bytes = await far.arrayBuffer();
    await this.primary.put(key, bytes, { customMetadata: far.customMetadata, httpMetadata: far.httpMetadata });
    return this.primary.get(key, opts);
  }
  put(key: string, body: Body, opts?: PutOpts) {
    return this.primary.put(key, body, opts);
  }
  async delete(keys: string | string[]) {
    await this.primary.delete(keys);
  }
}
