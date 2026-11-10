# Drone visuals pipeline: import, test, capture

Verified on 2026-11-10 with Unity 6000.5.10f1, HDRP 17.5, RTX 4070 Ti SUPER, Linux.
Worktree `/home/lunarz/worktrees/crane-drones`, branch `research/drone-sim`.

## 0. Rules

Shared rules: `Docs/DroneVisuals/AGENT_RULES.md`.

- Run Unity only through `~/.cache/research/fs-scripts/unity-drones [--render] LOG [args]`. It holds a project lock (one Editor per
  project: a waiting call is waiting on the lock) and goes through `heavy unity`.
  - default: `-nographics` (import, compile, EditMode tests).
  - `--render`: `xvfb-run` + `-force-vulkan`, so HDRP renders. Use it for every capture, still or video.
- `~/.cache/research/fs-scripts/unity-drones-player LOG [args]` runs the built player through `heavy unity` (no project lock needed, a
  player does not open the project). Fallback only; prefer Editor capture (section 3b).
- Put runtime code under your own asmdef and Editor code under your own Editor asmdef. Never leave non-compiling code.
- Do not commit. Do not start ROS. Do not command hardware.

## 1. Import (headless, works)

```sh
~/.cache/research/fs-scripts/unity-drones /tmp/import.log -quit
```

- A cold import with `Library/` seeded from `/home/lunarz/crane_ml/Library` (`cp -a`, 4.4 GB, 15 s) took about 90 s,
  of which 76 s is the asset-pipeline refresh. Without the seed it will be far longer. `Library/` is git-ignored.
- Licensing: the log shows `[Licensing::Module] Error: Access token is unavailable; failed to update`. This is
  harmless here: the Editor still imports, runs tests and builds players (exit 0). Do not treat it as failure.
- Success markers: `Batchmode quit successfully invoked`, `Exiting batchmode successfully now!`.
- The import regenerates `.vscode/settings.json` (shows as modified). Do not commit that.

## 2. EditMode tests

```sh
~/.cache/research/fs-scripts/unity-drones /tmp/tests.log -runTests -testPlatform EditMode -testResults /tmp/editmode.xml
```

Add `-testFilter "Name or Namespace"` or `-assemblyNames "Crane.Theater.Tests"` to narrow. Results are NUnit XML:
`grep -o 'total="[0-9]*" passed="[0-9]*" failed="[0-9]*"' /tmp/editmode.xml | head -1`.
Test assemblies: `Assets/Tests/Editor` (RoboBoatEditorTests), `Assets/Tests/Course`, `Assets/Tests/Theater`,
`Assets/Tests/Physics`.

## 3. Capture path (how a Theater render is launched headless)

The Theater (`Assets/Scripts/Theater`, docs in `theater/README.md`) is **player-driven, not an -executeMethod**.
`TheaterBoot` installs itself with `[RuntimeInitializeOnLoadMethod(BeforeSceneLoad)]` when the command line has
`--crane-theater MANIFEST`. It then:

1. loads the plug-ins named in the manifest (`world.plugin`, `robot.plugin`),
2. calls `world.Prepare(manifest)`, builds a `TheaterState` from the manifest plus an acts script,
3. renders frame f of the script into a RenderTexture with its own camera (deterministic: SMAA, no motion blur,
   fixed exposure adaptation, time = f / fps), draws the HUD, pipes frames to `ffmpeg`, and quits.

Outputs in `--crane-theater-out DIR`: `video.mp4`, `frames.jsonl` (per-frame pixel sha256), `render_report.json`,
`stills/frame_NNNNNN.png` (with `--crane-theater-stills N`), `player.log` (via `-logFile`).

### 3a. Capture from the Editor (preferred, no player build)

`Assets/Scripts/Theater/Editor/TheaterEditorCapture.cs` (asmdef `Crane.Theater.Editor`, Editor only) opens the scene named by
`--crane-scene` (found by name with AssetDatabase, so no build-settings entry), enters play mode, lets `TheaterBoot` render, and
exits the Editor with the Theater's exit code when it logs `CRANE_THEATER_EXIT`. Scene, prefab and C# changes are picked up
by a normal compile; no 6 to 8 minute player build.

```sh
# 1. a manifest (synthetic fleet on the RoboBoat course; for real studies see theater/README.md)
python3 -m theater.synth --truth Tools/RoboBoat/tests/referee/debris_red_circled/course_truth.json \
   --tasks entry_gates,debris_clearance,speed_challenge --runs 60 --iterations 6 --seed 1 --out /tmp/fleet
# 2. render (13 s, 720p30, clean frames, a still every 43 frames)
D=/tmp/fleet; O=/tmp/ref; mkdir -p $O
~/.cache/research/fs-scripts/unity-drones --render $O/editor.log \
  -executeMethod Crane.Theater.EditorTools.TheaterEditorCapture.Run \
  --crane-disable-ros --crane-disable-sitl --crane-scene "Roboboat Course" --crane-course-replay $D/course_truth.json \
  --crane-theater $D/manifest.json --crane-theater-out $O \
  --crane-theater-script Docs/DroneVisuals/reference/roboboat_reference_script.json \
  --crane-theater-stills 43 --crane-theater-nohud --crane-theater-crf 24
```

Do not pass `-quit` (the Editor exits itself). Exit code 0 plus `errors=0` in `render_report.json` means success.

**Editor-capture status: written and compiled-by-design but NOT yet run end to end** (the run was queued behind EditMode tests when this
was written; its output goes to `/tmp/drones_theater/edcap/`, `editor.log` and `exit.txt`). If it fails (play-mode domain reload may drop
the command line state, or `Application.Quit` handling differs), use the verified player path in 3b. Verify before relying on it.

### 3b. Capture from a built player (verified, fallback)

Build once per C# change: `unity-drones LOG -quit -executeMethod CranePerformanceBuild.BuildLinuxWorker`
(6 to 8 min, 420 MB, `Builds/` git-ignored, marker `CRANE_BUILD_COMPLETE`). `CranePerformanceBuild.Scenes` holds only
`Assets/Scenes/Roboboat Course.unity`; add another scene with `--crane-extra-scene PATH`.

**The player needs a display: use `xvfb-run`.** `-nographics` with no display crashes it (SIGSEGV after "Vulkan renderer=...").
Do NOT pass `-nographics` to the player.

```sh
D=/tmp/fleet; O=/tmp/ref; mkdir -p $O
xvfb-run -a -s "-screen 0 1280x720x24" ~/.cache/research/fs-scripts/unity-drones-player $O/player.log \
  -screen-fullscreen 0 -screen-width 320 -screen-height 180 --crane-disable-ros --crane-disable-sitl \
  --crane-scene "Roboboat Course" --crane-course-replay $D/course_truth.json \
  --crane-theater $D/manifest.json --crane-theater-out $O \
  --crane-theater-script Docs/DroneVisuals/reference/roboboat_reference_script.json \
  --crane-theater-stills 43 --crane-theater-nohud --crane-theater-crf 24
```

Result (player path, measured): 390 frames in about 30 s wall (about 40 s with startup), 4.4 MB mp4. `CRANE_THEATER_EXIT code=0` and
`errors=0` in `player.log` mean success.

Player flags (all `--crane-theater-*`): `out DIR`, `script ACTS.json`, `frames A:B`, `seconds S`, `fps F`,
`size WxH`, `ffmpeg PATH|none`, `crf N`, `preset P`, `preroll-s S`, `quality N`, `stills N`, **`nohud`** (added in this
branch: skips the data HUD, for clean stills and video). `--crane-theater-frames A:B` renders a slice, e.g. `0:1` for one
frame to check a shot (add `--crane-theater-preroll-s 0`).
The first frame costs about 1.5 s of shader and cache warm-up. Stills are PNG at the script size
(use `size: [1600, 900]` or smaller).

Acts script (`Docs/DroneVisuals/reference/roboboat_reference_script.json`): each act has `duration_s`, `speed`
(run seconds per video second), `t0` (run time at act start), `hero: "best"` (shows the real robot visuals),
`ghost_alpha: 0`, `markers: false`, `trails`, `caption`, and `camera: {shot, ...}`. Shots, from `TheaterState.CameraPath`:

| shot | params (defaults) |
|---|---|
| `orbit`, `establish` (orbit with push-in) | `radius`, `height`, `angle`, `deg_per_s` (5), `fov` (50) |
| `top` | `pitch` (62), `margin`, `push` |
| `flyover` | `back` (18), `height` (14): flies along the course centre line |
| `chase`, `follow` | `back` (5/9), `height` (2.5/7), `side`, `lead` (4), `stiffness` (2): tracks the hero |
| `pack` | `bearing`, `deg_per_s`, `back`, `height`: tracks the fleet centroid |

The camera is clamped to at least 1 m above `World.SurfaceY`. The hero shows a gold trail line and ring (Theater overlay, not the
robot); hide it only by editing `TheaterRenderer` or using `hero: "none"` with ghosts. Reference media were recorded this way.

## 4. RoboBoat reference (the quality bar)

`Docs/DroneVisuals/reference/`:

| file | what |
|---|---|
| `roboboat_reference_13s_720p30.mp4` | 13 s, 4 acts: establish, chase, flyover, top |
| `roboboat_01_establish.png` .. `_02_` | wide high-angle over the water, shore and tents at frame edge |
| `roboboat_03_chase_start.png` .. `_05_chase_late.png` | chase cam behind and beside the hero boat, horizon with clouds |
| `roboboat_06_flyover.png`, `_07_flyover_late.png` | low flyover along the course, buoys receding |
| `roboboat_08_top.png` | top-down on the shore, gates and buoys |
| `roboboat_reference_script.json` | the acts script that produced them |

What the quality bar is: physically-lit HDR sunlit scene, a real HDRP water surface with tessellated wave detail
and sun glints, large specular reflection of sky and clouds in the water, teal absorption body colour, a bright blue sky with
soft cumulus (HDRI), neutral tonemapping with automatic exposure (no blown or crushed areas), soft long shadows,
saturated but not neon accent colours (red/green buoys), and no UI overlay. A drone scene must match this
lighting and water/sky quality on its own terrain (forest) without looking flatter or noisier.

## 5. HDRP settings that make RoboBoat look right

Source files: `Assets/Scenes/Roboboat Course.unity`, `Assets/Settings/*`, `ProjectSettings/QualitySettings.asset`.
Project is linear colour space (`m_ActiveColorSpace: 1`). Quality level 0 = "High Fidelity" (default, used in captures).

**Pipeline asset** (`Assets/Settings/HDRP High Fidelity.asset`): `supportWater 1`, `supportWaterExclusion 1`, `supportSSR 1`,
`supportSSAO 1`, `supportSSGI 1`, `supportVolumetrics 1`, `supportVolumetricClouds 1`, `supportDecals 1`, `supportDistortion 1`,
`supportScreenSpaceLensFlare 1`, `supportRayTracing 0`, `colorBufferFormat 48` (R16G16B16A16 HDR buffer), `msaaSampleCount 1`
(post-process AA instead), `supportedLitShaderMode 3` (forward and deferred). Balanced and Performant assets exist
(`HDRP Balanced.asset`, `HDRP Performant.asset`); `--crane-theater-quality N` switches level.

**Sun** (scene object `Sun`, directional HDRP Light): rotation Euler (50, -30, 0), colour (1, 0.957, 0.839) with
temperature 6570 K (`m_UseColorTemperature 1`), intensity 57099 lux (physical units), angular diameter 0.5 deg,
shadows type 2 (shadow-mapped, PCSS-capable), shadow fade distance 10000, volumetric dimmer 1.
Reuse a sun of about 50 deg elevation and 57 klux with the automatic exposure limits below.

**Global volume** (scene object `EnvironmentVolume`, global, priority 0, weight 1) uses
`Assets/Settings/HDRPDefaultResources/DefaultLookDevProfile.asset`, which overrides:
`AmbientOcclusion intensity 0.5, radius 1`; `HDShadowSettings maxShadowDistance 25, cascadeShadowSplitCount 2`
(note: the short 25 m shadow distance suits a small boat, raise it for a drone scene with tall trees);
`Bloom intensity 0.1`; `Tonemapping mode 1` (Neutral).

**Project default volume** (HDRP global settings, `Assets/Settings/HDRPDefaultResources/DefaultSettingsVolumeProfile.asset`) supplies the rest:
`HDRISky` (exposure 11, multiplier 1, a cloudy HDRI cubemap `8253d41e6e8b11a4cbe77a4f8f82934d`, upper-hemisphere only, no lock-sun),
`Exposure` automatic (mode 1, histogram metering, percentages 40..90, limits -1..14 EV, adaptation dark-to-light 3, light-to-dark 1),
`Bloom` (scatter 0.7, anamorphic 1, high-quality filtering), `ColorAdjustments` neutral, `WhiteBalance` neutral,
`MotionBlur` intensity 0.5, volumetric clouds disabled. `Assets/Settings/SkyandFogSettingsProfile.asset` (referenced by the
quality assets) adds `WaterRendering enable, ambientProbeDimmer 0.5` and `Exposure` limits -3.9..11.5. There is no fog override:
no distance haze beyond HDRP defaults; visibility comes from the HDRI horizon.

**Theater volume** (added at runtime by `TheaterBoot`, priority 10000, so it overrides the above): `Exposure.adaptationMode = Fixed`
(adaptation only, mode stays automatic) and `MotionBlur.intensity = 0`. Theater camera: near 0.2, far 5000, FOV 45 to 55,
SMAA High, no dynamic resolution, RenderTexture ARGB32 sRGB 24-bit depth, `Camera.Render()` manual.

**Water** (`Ocean` object, HDRP `WaterSurface`): `surfaceType 0` (ocean), `geometryType 3` (infinite),
`repetitionSize 500`, `largeWindSpeed 10`, `largeChaos 0.85`, `ripples 1`, `ripplesWindSpeed 2.74`, `ripplesChaos 0.8`
(ripples fade 50..200 m), `startSmoothness 0.95` / `endSmoothness 0.85` (fade 100..500 m), `tessellation 1`
(max factor 3, fade from 150 m), `refractionColor` (0.072, 0.241, 0.241), `maxRefractionDistance 0.5`, `absorptionDistance 1.5`,
`scatteringColor` (0, 0.220, 0.337), `ambientScattering 0.2`, `heightScattering 0.2`, `displacementScattering 0.1`,
`directLightTipScattering 0.6`, `directLightBodyScattering 0.5`, `foam 1` (res 512, tiling 0.15, amount 0.2),
`caustics 0`, `deformation 0`, `timeMultiplier 1` (the Theater pins it to 0 and sets `simulationTime` per frame for determinism).
This water is the largest single contributor to the look; the ground has nothing equivalent, so forest needs equally strong
material work (see below).

**Materials / objects**: the course uses HDRP/Lit materials with simple saturated albedo (red/green buoys, white tents,
pale sand shore) so the sky/water lighting does most of the work. The shore uses `Shore Fade Visual`.
The hero robot is a `Strip()`-ed clone of the scene boat (meshes and renderers only).

**Camera**: physical HDRP camera defaults; Theater camera controls FOV only.

### Reuse for the drone forest
- Start from the same Sun (50 deg elevation, 6570 K, ~57 klux), same HDRISky + automatic Exposure + Neutral tonemap + Bloom 0.1,
  AO 0.5. Raise `maxShadowDistance` (100 to 300 m) and cascade count (3 to 4) for trees.
- Add a **volumetric fog** override (`Fog` enabled, `enableVolumetricFog`, mean free path 200 to 500, anisotropy 0.6,
  sun-lit god-rays) for depth and forest feel; RoboBoat does not use fog, but a forest needs depth cues.
- Keep colour space linear, HDR buffer 16-bit, SMAA for capture.
- Terrain/foliage needs real HDRP/Lit (or SpeedTree/Foliage) materials with proper smoothness and normal maps; flat albedo
  will read as a downgrade next to the water.

## 6. Add a Theater world and robot plug-in for a drone in a forest scene

Interfaces: `Assets/Scripts/Theater/TheaterPlugins.cs` (`ITheaterWorld`, `ITheaterRobot`, `TheaterPlugins.Worlds/Robots`).
Model implementation: `Assets/Scripts/Theater/RoboBoat/RoboBoatTheater.cs`.

1. **Assembly.** New folder `Assets/Scripts/Theater/Drone/` with `Crane.Theater.Drone.asmdef` referencing `Crane.Theater`,
   `Unity.RenderPipelines.Core.Runtime`, `Unity.RenderPipelines.HighDefinition.Runtime` (copy the RoboBoat asmdef, drop RoboBoat refs
   unless needed). Keep the drone code out of `Crane.Theater` core ("the only domain code" rule).
2. **Register** in a `[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]` method:
   `TheaterPlugins.Worlds["droneforest"] = () => new DroneForestWorld(); TheaterPlugins.Robots["quad"] = () => new QuadTheaterRobot();`
3. **World** (`ITheaterWorld`):
   - `Prepare(manifest)`: yield until the scene is built (the forest scene is a normal Unity scene; wait a few frames, find a marker
     object), switch off everything live (physics, controllers, sensors; see `DisableLive`), set bodies kinematic, hide the live drone.
     Read `manifest.World` for `bounds` `[minX, minY, maxX, maxY]` and `centreline` (ENU m).
   - `ToUnity(x,y,z)`: ENU to Unity. RoboBoat uses `(-y, waterY + z, x)`; a forest scene authored with +z north uses the plane
     mapping `(x, z, y)` (`PlaneWorld`). Pick one, and make `UnityYaw` match.
   - `SurfaceY`: the terrain height (raycast or `Terrain.SampleHeight`). Used for planar seating, the 1 m camera floor and markers.
     A drone pose with z recorded is not seated (`Planar` is false when the pack has z), so cameras clamp against terrain.
   - `Bounds`, `CentreLine` (camera flyover path), `SetTime(t)` (drive every time-based thing from `t`: foliage wind, sky, clouds, never
     `Time.time`), `Describe`.
4. **Robot** (`ITheaterRobot`): `GhostMesh()` (a few hundred tris, +z forward, +y up; use `MeshKit.Box` to build an X-frame quad),
   `CreateHero(name)` (the drone prefab, or `Instantiate` of the scene drone with scripts, colliders and bodies stripped as in
   `RoboBoatTheaterRobot.Strip`), `HeroRotation` (body-frame offset), `Length`, `Surface` (`"none"` for a flying robot).
   The prop-blur or rotor animation must be a function of `SetTime`/pose speed, not wall time.
5. **Scene in the player.** Add the scene to the build (`--crane-extra-scene Assets/Drones/Environment/Forest.unity` when running
   `BuildLinuxWorker`, or extend `CranePerformanceBuild.Scenes`), then run with `--crane-scene "Forest"` (the scene name without
   path or extension). `CraneStartupRenderGuard` hides the startup scene while it loads. The Roboboat course bootstrap
   (`--crane-course-replay`) is not needed for a drone world: omit it.
6. **Manifest.** `theater/pack.py` `write_manifest(..., world={'plugin': 'droneforest', 'scene': 'Forest', 'bounds': [...],
   'centreline': [[x,y],...], 'args': ['--crane-scene', 'Forest']}, robot={'plugin': 'quad'}, ...)` plus tracks with x, y, z, yaw, pitch,
   roll channels at 10 Hz (pack.py header, CHANNELS). Add a world class to `theater/worlds.py` (`WORLDS['droneforest']`) so
   `theater.synth`/`render script` can build manifests, or hand-write the manifest JSON and use `theater/pack.write_pack`.
   Replay physics output (`Tools/Aerial/aerial_replay.py`, `export_parity_trajectories.py`) can be converted into the pack channels.
7. **Script.** Copy `reference/roboboat_reference_script.json`, change shots (`chase` `back`/`height` smaller for a drone, FOV 60 to 70),
   and capture with section 3b. Always render `--crane-theater-frames 0:1 --crane-theater-preroll-s 0` first to check framing, then the full clip.
8. **Test.** `Assets/Tests/Theater/TheaterTests.cs` shows the EditMode pattern for plug-ins and the state machine.

## 7. Pitfalls

- **One Editor per project.** Unity refuses two (`Multiple Unity instances cannot open the same project`); `unity-drones` now serializes
  with a project lock, so long waits are normal when others run tests or builds (EditMode tests took over 10 min here, mostly
  `CourseGeneratorTests`; use `-testFilter` / `-assemblyNames` to narrow).
- **Player crashes without a display** (SIGSEGV after the Vulkan renderer line, "Video subsystem has not been initialized"): wrap in `xvfb-run -a`.
- **Default `unity-drones` cannot render** (`-nographics`). Use `--render` for captures.
- **Player path only: rebuild after every C# or scene change** (6 to 8 min). Iterate framing with one-frame slices (`--crane-theater-frames 0:1 --crane-theater-preroll-s 0`).
- `Time.timeScale = 0` and `captureDeltaTime = 1/fps` during a Theater run; anything driven by `Time.time`, `Time.deltaTime`, or wall
  clock (particles, wind, animators) will freeze or jump. Drive it from the world's `SetTime`.
- Exposure: do not `Add<Exposure>(true)` (resets to Fixed EV 0 and gives a white frame). Override only `adaptationMode`.
- The ghost overlay (gold trail and ring) is part of the Theater hero and is visible in stills. Use `--crane-theater-nohud` for the HUD only.
- Stills are PNG at the render size, about 1.2 MB at 1280x720. Keep media in git small: PNG at most 1600 px, video at most 15 MB (crf 24 gave 4.4 MB for 13 s).
- `water.mode` in the manifest (`still` by default) freezes the ocean; set `"water": {"mode": "live"}` for moving waves.
- Build artefacts (`Library/`, `Builds/`, `Temp/`) are git-ignored; `.vscode/settings.json` is rewritten by Unity: do not commit it.
- `theater/render.py local` also launches a player; it does not wrap in `xvfb-run` by itself unless the host provides a display
  (`--display :1`). Prefer the explicit command in 3b.
