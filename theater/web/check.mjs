// Test helper (theater/tests/test_js_codec.py): decode files with ctraj.js and print what Python compares.
//   node theater/web/check.mjs FILE [FILE...] | node check.mjs --fleet PACK
import { readFileSync } from "node:fs";
import { decode, fleetUnpack, parseHead, speed, agents } from "./ctraj.js";

const args = process.argv.slice(2);
const files = args[0] === "--fleet" ? fleetUnpack(readFileSync(args[1])) : args.map((f) => readFileSync(f));
const out = [];
for (const bytes of files) {
  const t = await decode(bytes);
  let partial = null;
  try {
    parseHead(bytes.subarray(0, 20));
  } catch (e) {
    partial = e.need ?? String(e);
  }
  const win = t.ticks > 0 ? await decode(bytes, { tFrom: t.t0 + (t.ticks - 1) / t.hz }) : null;
  out.push({
    header: t.header,
    ticks: t.ticks,
    hz: t.hz,
    events: t.events,
    columns: Object.fromEntries(Object.entries(t.columns).map(([k, v]) => [k, Array.from(v)])),
    blocks: t.index.map((e) => [e.first, e.n, e.offset, e.len]),
    partial_need: partial,
    window_first_tick: win ? win.firstTick : null,
    agents: agents(t),
    speed0: agents(t).length && t.columns[`${agents(t)[0]}.y`] ? speed(t, agents(t)[0])[1] : null,
  });
}
let corrupt = "ok";
if (files.length) {
  const bad = new Uint8Array(files[0]);
  bad[20] ^= 0xff;
  try {
    await decode(bad);
  } catch (e) {
    corrupt = String(e.message);
  }
}
console.log(JSON.stringify({ files: out, corrupt }));
