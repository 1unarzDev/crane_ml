# theater/web: crane-traj/1 in the browser

`ctraj.js` reads the `.ctraj` files `theater/traj.py` and the Unity recorder write. It is one ES module
with no dependencies (raw deflate through `DecompressionStream('deflate-raw')`), for the run dashboard,
the Worker and any page that replays runs. `theater/tests/test_js_codec.py` checks it against the Python
reader through `check.mjs` (node >= 18).

| export | does |
|---|---|
| `decode(buf, {tFrom, tTo, checkCrc})` | whole file -> `{header, specs, columns: {name: Float64Array}, events, ticks, hz, t0, firstTick, index}` |
| `parseHead(buf)` | header, columns and block index from a prefix; throws `NeedMore(need)` when the prefix is short |
| `decodeBlock(blob, specs, n)` | one block's columns (for Range-fetched 30 s blocks) |
| `fleetUnpack(buf)` | a fleet pack (`u32 len, bytes` repeated) -> one `Uint8Array` per file |
| `agents(t)`, `speed(t, agent)` | agent names; planar speed derived from x, y and hz (writers drop the speed column) |

Values are SI: metres (ENU world frame), radians (body FLU yaw, pitch, roll). Sample `i` is at `t0 + i / hz`.

## Which file to fetch

- **Overview / fleet:** `run.lod.ctraj`, the same format at 2 Hz (x y yaw at 5 cm and 0.5 deg; 6-DOF runs add
  z, pitch, roll). About 2 to 7 KB per run. Its header holds only public-safe keys: `pose_dims`, `box`,
  `outcome`, `progress`, `valid`, `robot`, `frame`, `course.truth_sha256`, `lod {of_hz, hz, of_ticks}` and
  `parent_sha256` (the full file). Catmull-Rom between samples is within 2 cm of the full path.
- **Hero / close-up:** the full `run.ctraj` (20 Hz Unity, 10 Hz fastsim), whole, or block by block:
  `Range: bytes=0-65535` -> `parseHead` -> `Range: bytes=offset-(offset+len-1)` -> `decodeBlock`.

## Sizes (design-storage.md, measured)

| | 10 min Unity 6-DOF | fastsim 3.5 min planar |
|---|---|---|
| `run.ctraj` | 17-36 KB | about 3 KB |
| `run.lod.ctraj` | 4-7 KB | about 1.5 KB |
