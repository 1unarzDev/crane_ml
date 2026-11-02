// crane-traj/1 (.ctraj) reader for browsers and Workers: the format theater/traj.py writes.
// No dependencies: raw deflate through DecompressionStream('deflate-raw') (every current browser,
// Workers, Node >= 18). ES module; see theater/web/README.md for the API and the fetch patterns.
//
//   import { decode, parseHead, decodeBlock, fleetUnpack } from "./ctraj.js";
//   const t = await decode(await (await fetch(url)).arrayBuffer());   // {header, columns, events, specs, ticks, hz}
//   t.columns["boat.x"][i]  // Float64Array, metres; angles in radians; sample i at t0_s + i / hz

export const MAGIC = "CTRJ";
export const VERSION = 1;

export class CtrajError extends Error {}

/** Thrown by parseHead when `bytes` stop before the index ends; `need` is the byte count to fetch. */
export class NeedMore extends CtrajError {
  constructor(need) {
    super(`need ${need} bytes`);
    this.need = need;
  }
}

const u8 = (b) => (b instanceof Uint8Array ? b : new Uint8Array(b));

async function inflateRawDefault(bytes) {
  const ds = new DecompressionStream("deflate-raw");
  const out = new Response(new Blob([bytes]).stream().pipeThrough(ds));
  return new Uint8Array(await out.arrayBuffer());
}

let CRC_TABLE = null;
export function crc32(bytes) {
  if (!CRC_TABLE) {
    CRC_TABLE = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      CRC_TABLE[n] = c >>> 0;
    }
  }
  let c = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

/**
 * The header, columns and block index from the start of a file. Works on a prefix: a viewer fetches
 * `Range: bytes=0-65535`, calls parseHead, and on NeedMore fetches `e.need` bytes. Each index entry
 * gives the block's absolute byte range (`offset`, `len`), so a client can Range-fetch one 30 s block.
 */
export function parseHead(buf) {
  const b = u8(buf);
  const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
  const need = (n) => {
    if (n > b.length) throw new NeedMore(n);
  };
  need(12);
  if (String.fromCharCode(b[0], b[1], b[2], b[3]) !== MAGIC) throw new CtrajError("not a crane-traj file");
  const version = dv.getUint16(4, true);
  if (version !== VERSION) throw new CtrajError(`crane-traj version ${version} is not supported (this reader: ${VERSION})`);
  const hlen = dv.getUint32(8, true);
  let p = 12;
  need(p + hlen + 2);
  const header = JSON.parse(new TextDecoder().decode(b.subarray(p, p + hlen)));
  p += hlen;
  const ncols = dv.getUint16(p, true);
  p += 2;
  const specs = [];
  for (let i = 0; i < ncols; i++) {
    need(p + 1);
    const nl = b[p];
    need(p + 1 + nl + 16);
    const name = new TextDecoder().decode(b.subarray(p + 1, p + 1 + nl));
    p += 1 + nl;
    specs.push({ name, scale: dv.getFloat64(p, true), offset: dv.getFloat64(p + 8, true) });
    p += 16;
  }
  need(p + 12);
  const ticks = dv.getUint32(p, true);
  const blockTicks = dv.getUint32(p + 4, true);
  const nblocks = dv.getUint32(p + 8, true);
  p += 12;
  need(p + 12 * nblocks);
  const index = [];
  let off = p + 12 * nblocks;
  for (let i = 0; i < nblocks; i++, p += 12) {
    const len = dv.getUint32(p + 8, true);
    index.push({ first: dv.getUint32(p, true), n: dv.getUint32(p + 4, true), offset: off, len });
    off += len;
  }
  return { version, header, specs, ticks, blockTicks, index, dataStart: p, eventsAt: off };
}

/** One block's columns (name -> Float64Array of n values) from its compressed bytes. */
export async function decodeBlock(blob, specs, n, { inflateRaw = inflateRawDefault } = {}) {
  const raw = await inflateRaw(u8(blob));
  let pos = 0;
  const out = {};
  for (const c of specs) {
    const vals = new Float64Array(n);
    let prev = 0;
    for (let i = 0; i < n; i++) {
      let z = 0;
      let mul = 1;
      for (;;) {
        if (pos >= raw.length) throw new CtrajError("block shorter than its index says");
        const byte = raw[pos++];
        z += (byte & 0x7f) * mul; // arithmetic, not bit ops: deltas may pass 32 bits
        mul *= 128;
        if (byte < 0x80) break;
      }
      prev += z % 2 === 1 ? -(z + 1) / 2 : z / 2;
      vals[i] = prev * c.scale + c.offset;
    }
    out[c.name] = vals;
  }
  return out;
}

/**
 * Decode a whole file. With tFrom/tTo (seconds, header time) only overlapping blocks are decoded and
 * `firstTick` says where the columns start. The crc is checked unless checkCrc is false.
 */
export async function decode(buf, { tFrom = null, tTo = null, checkCrc = true, inflateRaw = inflateRawDefault } = {}) {
  const b = u8(buf);
  if (b.length < 16) throw new CtrajError("not a crane-traj file");
  if (checkCrc) {
    const want = new DataView(b.buffer, b.byteOffset + b.length - 4, 4).getUint32(0, true);
    if (crc32(b.subarray(0, b.length - 4)) !== want) throw new CtrajError("crc mismatch (corrupt or truncated file)");
  }
  const h = parseHead(b);
  const hz = Number(h.header.hz);
  const t0 = Number(h.header.t0_s ?? 0);
  const picked = h.index.filter(
    (e) => !(tFrom !== null && t0 + (e.first + e.n - 1) / hz < tFrom) && !(tTo !== null && t0 + e.first / hz > tTo),
  );
  const total = picked.reduce((s, e) => s + e.n, 0);
  const columns = Object.fromEntries(h.specs.map((c) => [c.name, new Float64Array(total)]));
  let at = 0;
  for (const e of picked) {
    const cols = await decodeBlock(b.subarray(e.offset, e.offset + e.len), h.specs, e.n, { inflateRaw });
    for (const c of h.specs) columns[c.name].set(cols[c.name], at);
    at += e.n;
  }
  const events = await decodeEvents(b, h.eventsAt, { inflateRaw });
  const windowed = tFrom !== null || tTo !== null;
  if (!windowed && total !== h.ticks) throw new CtrajError("decoded column length differs from ticks");
  return {
    header: h.header,
    specs: h.specs,
    columns,
    events,
    ticks: h.ticks,
    hz,
    t0,
    firstTick: picked.length ? picked[0].first : 0,
    index: h.index,
  };
}

async function decodeEvents(b, at, { inflateRaw }) {
  if (at + 4 > b.length) throw new CtrajError("truncated file");
  const len = new DataView(b.buffer, b.byteOffset + at, 4).getUint32(0, true);
  const text = new TextDecoder().decode(await inflateRaw(b.subarray(at + 4, at + 4 + len)));
  return text
    .split("\n")
    .filter((l) => l.trim())
    .map((l) => JSON.parse(l));
}

/** A fleet pack (GET /v1/replay/fleet): `u32 len, bytes` repeated -> [Uint8Array]. */
export function fleetUnpack(buf) {
  const b = u8(buf);
  const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
  const out = [];
  let p = 0;
  while (p < b.length) {
    if (p + 4 > b.length) throw new CtrajError("truncated fleet pack");
    const n = dv.getUint32(p, true);
    if (p + 4 + n > b.length) throw new CtrajError("truncated fleet pack");
    out.push(b.subarray(p + 4, p + 4 + n));
    p += 4 + n;
  }
  return out;
}

/** Agents of a decoded file: header.robot.agents, else the prefixes of `*.x` columns. */
export function agents(t) {
  const a = t.header?.robot?.agents;
  if (Array.isArray(a) && a.length) return a;
  return [...new Set(Object.keys(t.columns).filter((n) => n.endsWith(".x")).map((n) => n.slice(0, -2)))].sort();
}

/** Planar speed (m/s) per sample from x, y and hz (writers drop the speed column; it is derived). */
export function speed(t, agent) {
  const x = t.columns[`${agent}.x`];
  const y = t.columns[`${agent}.y`];
  const out = new Float64Array(x ? x.length : 0);
  for (let i = 1; i < out.length; i++) out[i] = Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) * t.hz;
  if (out.length > 1) out[0] = out[1];
  return out;
}
