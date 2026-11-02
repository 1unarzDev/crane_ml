// traj.js: a reader for crane-traj/1 (.ctraj) for the browser and the Worker; the format is theater/traj.py's.
// decode(bytes) -> {header, hz, ticks, t0, columns: {name: Float64Array}, events}; withHeader(bytes, header)
// re-encodes only the header section (blocks are copied) and fixes the crc. Inflate is DecompressionStream
// ('deflate-raw'), the same raw deflate as Python's zlib(-15) and .NET's DeflateStream.

const td = new TextDecoder();
const te = new TextEncoder();

export class TrajError extends Error {}

async function inflate(bytes) {
  const s = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate-raw'));
  return new Uint8Array(await new Response(s).arrayBuffer());
}

let CRC;
export function crc32(b, end = b.length) {
  if (!CRC) {
    CRC = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      CRC[n] = c >>> 0;
    }
  }
  let c = 0xffffffff;
  for (let i = 0; i < end; i++) c = CRC[(c ^ b[i]) & 255] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

/** The file's sections: header, column specs, index, and where blocks and events start. */
export function parse(buf, checkCrc = true) {
  const b = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  const v = new DataView(b.buffer, b.byteOffset, b.byteLength);
  if (b.length < 16 || td.decode(b.subarray(0, 4)) !== 'CTRJ') throw new TrajError('not a crane-traj file');
  if (checkCrc && crc32(b, b.length - 4) !== v.getUint32(b.length - 4, true)) throw new TrajError('crc mismatch');
  const version = v.getUint16(4, true);
  if (version !== 1) throw new TrajError(`crane-traj version ${version} is not supported`);
  let p = 8;
  const need = (n) => { if (p + n > b.length - 4) throw new TrajError('truncated file'); };
  const hlen = v.getUint32(p, true); p += 4; need(hlen);
  const headerEnd = p + hlen;
  const header = JSON.parse(td.decode(b.subarray(p, headerEnd))); p = headerEnd;
  const ncols = v.getUint16(p, true); p += 2;
  const specs = [];
  for (let i = 0; i < ncols; i++) {
    const nl = b[p]; p += 1; need(nl + 16);
    const name = td.decode(b.subarray(p, p + nl)); p += nl;
    specs.push({ name, scale: v.getFloat64(p, true), offset: v.getFloat64(p + 8, true) }); p += 16;
  }
  need(12);
  const ticks = v.getUint32(p, true), blockTicks = v.getUint32(p + 4, true), nblocks = v.getUint32(p + 8, true); p += 12;
  need(nblocks * 12);
  const index = [];
  for (let i = 0; i < nblocks; i++, p += 12) index.push([v.getUint32(p, true), v.getUint32(p + 4, true), v.getUint32(p + 8, true)]);
  return { b, v, header, headerEnd, specs, ticks, blockTicks, index, blocksAt: p };
}

/** Decodes a whole file; columns are Float64Arrays of `ticks` values (q * scale + offset). */
export async function decode(buf) {
  const f = parse(buf);
  const { b, v, specs, ticks, index } = f;
  const columns = Object.fromEntries(specs.map((c) => [c.name, new Float64Array(ticks)]));
  let p = f.blocksAt;
  for (const [first, n, len] of index) {
    if (p + len > b.length - 4) throw new TrajError('truncated file');
    const raw = await inflate(b.subarray(p, p + len)); p += len;
    let q = 0;
    for (const c of specs) {
      const out = columns[c.name];
      let prev = 0;
      for (let i = 0; i < n; i++) {
        let z = 0, mul = 1, byte;
        do {
          if (q >= raw.length) throw new TrajError('block shorter than its index says');
          byte = raw[q++];
          z += (byte & 0x7f) * mul;
          mul *= 128;
        } while (byte >= 0x80);
        prev += z % 2 ? -(z + 1) / 2 : z / 2;
        out[first + i] = prev * c.scale + c.offset;
      }
    }
  }
  const elen = v.getUint32(p, true); p += 4;
  const events = td.decode(await inflate(b.subarray(p, p + elen))).split('\n').filter((l) => l.trim()).map((l) => JSON.parse(l));
  const hz = Number(f.header.hz);
  return { header: f.header, hz, ticks, t0: Number(f.header.t0_s || 0), columns, events };
}

/** The same file with another header (blocks and events copied as they are) and a new crc. */
export function withHeader(buf, header) {
  const f = parse(buf, false);
  const hj = te.encode(JSON.stringify(sortKeys(header)));
  const rest = f.b.subarray(f.headerEnd, f.b.length - 4);
  const out = new Uint8Array(12 + hj.length + rest.length + 4);
  out.set(f.b.subarray(0, 8));
  const v = new DataView(out.buffer);
  v.setUint32(8, hj.length, true);
  out.set(hj, 12);
  out.set(rest, 12 + hj.length);
  v.setUint32(out.length - 4, crc32(out, out.length - 4), true);
  return out;
}

function sortKeys(o) {
  if (Array.isArray(o)) return o.map(sortKeys);
  if (o && typeof o === 'object') return Object.fromEntries(Object.keys(o).sort().map((k) => [k, sortKeys(o[k])]));
  return o;
}

/** One agent's planar track {t, x, y, yaw, v} (v from x, y and hz when the file has no speed column). */
export function track(d, agent) {
  const a = agent || d.header.robot?.agents?.[0] || Object.keys(d.columns).find((k) => k.endsWith('.x'))?.slice(0, -2);
  const x = d.columns[`${a}.x`], y = d.columns[`${a}.y`];
  if (!x || !y) throw new TrajError(`no ${a}.x/.y columns`);
  const n = x.length, t = new Float64Array(n), yaw = d.columns[`${a}.yaw`] || new Float64Array(n);
  let v = d.columns[`${a}.speed`];
  for (let i = 0; i < n; i++) t[i] = i / d.hz;
  if (!v) {
    v = new Float64Array(n);
    for (let i = 1; i < n; i++) v[i] = Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) * d.hz;
    v[0] = v[1] || 0;
  }
  return { t, x, y, yaw, v, events: d.events, header: d.header };
}
