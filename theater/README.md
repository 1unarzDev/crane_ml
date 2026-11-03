# Ghost theater

Plays thousands of recorded runs back as one deterministic video, in the full Unity renderer: a translucent
ghost fleet per iteration, comet tails and full trails for the best runs, gate and failure markers, a
path-density and failure-heat layer, gold (best) and silver (previous best) hero boats with the real robot
visuals, and a data HUD (iteration counter, run counts, best-score sparkline, what changed between the two
heroes). The look follows Trackmania AI-training videos; the design is
`~/.cache/research/sim2real/design-ghost-theater.md` (D-20261102-13, -14).

It is pose playback: nothing is re-simulated, so a render is reproducible whatever the simulator does.

## Pieces

| Path | What |
|---|---|
| `theater/pack.py` | `crane-theater-pack/1` (runs resampled to one rate, int16, crc) and the `crane-theater/1` manifest |
| `theater/worlds.py` | world plug-ins on the Python side (RoboBoat: course truth to centre line, bounds, player args) |
| `theater/synth.py` | synthetic fleets for tests and spikes |
| `theater/render.py` | `script` (the standard cut), `local`, `submit` (cluster), `assemble`, `inspect` |
| `Assets/Scripts/Theater/` | `Crane.Theater`: manifest and pack readers, acts, state machine and camera director, GPU ghosts (HDRP custom pass), HUD, ffmpeg capture |
| `Assets/Scripts/Theater/RoboBoat/` | `ITheaterWorld`/`ITheaterRobot` for RoboBoat (the only domain code) |
| `crane_backends/theater.py` | the `theater-render` job kind: one seed per chunk of frames |

A drone or another robot plugs in with its own `ITheaterWorld` (how the world is built, its frame, its surface)
and `ITheaterRobot` (a ghost mesh of a few hundred triangles, its full visuals for heroes) plus a Python world.

## Study layers (optional manifest keys)

A study can draw its own world geometry and numbers without domain code in the theater:

| key | what |
|---|---|
| `overlay` | `crane-theater-overlay/1` (`TheaterOverlay`): `shapes` in the trajectory frame (`disc` with fill, rim and a curtain; `ring`; `band` along a polyline; `wall`), each with `color`, `alpha`, `pulse_hz` and the `acts` / `iterations` it shows in; `halo` (a ring of `radius_by_iteration` around every ghost and hero); `events` (marker colour and size per event kind, `"none"` hides one) |
| `hud` | the figure-style HUD (`HudStyle`): `brand`, `subtitle`, `iteration_label`, `font` / `mono` (DejaVu Sans and DejaVu Sans Mono are carried in `Resources/Fonts`, others come from the OS), `palette` {ink, dim, accent, panel, panel_alpha}, `metrics` rows per iteration (and `all`), and `spark` (the study's own series with axes and point labels) |
| `look` | a grade over the scene: `exposure_ev`, `saturation`, `contrast`, `color_filter`, `vignette` |
| `ramps` | named colour ramps (`{"epoch": ["#cde2fb", ...]}`) usable as an act's `ramp` |

An act's `event_markers: "all"` marks every shown run's events (default: heroes and trail runs); `trails: "top"` with
`colour: "iteration"` colours the trails by iteration. Without these keys a render is unchanged.

## Determinism

1. State: frame f's state (every ghost, hero, marker, the camera and the HUD numbers) is a pure function of
   the manifest, the script and f (`TheaterTests.FrameStateIsAPureFunctionOfTheFrame`); `frames.jsonl` logs
   its digest per frame. Camera follow paths are smoothed once per act, from the act's start.
2. Pixels: every time source is the frame index (water `simulationTime` per frame, no motion blur, exposure
   without temporal adaptation, SMAA instead of TAA), heat accumulates in integers, a chunk renders 2 s of
   discarded pre-roll; `frames.jsonl` logs each frame's pixel sha256, which is stable per worker class.
3. Across GPUs and APIs frames are similar, not identical.

## Use

```sh
python3 -m theater.synth --truth course_truth.json --runs 500 --iterations 20 --out DIR   # or a real study's runs
python3 -m theater.render script DIR --seconds 15            # the standard cut into DIR/manifest.json
heavy unity python3 -m theater.render local DIR --player Builds/CRANE-Worker/CRANE.x86_64 --out OUT
python3 -m theater.render submit DIR --chunk-s 10            # cluster: theater-render job, prints its id
python3 -m theater.render assemble JOB --out OUT              # video.mp4, final_h265.mp4, preview.gif, contact sheets
python3 -m theater.render inspect OUT                         # automated checks of render_report.json
python3 -m unittest theater.tests.test_pack                   # pack, golden fixture, synth
```

The player flags (`--crane-theater MANIFEST --crane-theater-out DIR`, frames, fps, size, ffmpeg, preroll) are
documented on `Crane.Theater.TheaterBoot`. Renders run batchmode on the GPU and never open a window.
