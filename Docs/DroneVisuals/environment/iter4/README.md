# Forest iteration 4: continuous time of day, seasons, defect check (2026-11-10)

Rendered in ONE Unity launch (`IterFourRender.RenderBatch`, built-in tree provider, seed 1337, 1080p). Every frame has a cinematic version (graded by time of day) and an onboard-sensor version (`sensor/`, no grade, legacy auto-exposure limits -1..14 EV100) from identical world state.
Log: `render_log.txt`. Tests: `selftest.txt` (18 of 18 pass). Manifest: Per-season manifests are not committed (about 24 MB each); their sha256s are in `manifests_sha256.txt` and the files are in `~/.cache/drones-env/iter4_png_archive/manifests/`. All frames here are JPEG q90; PNG originals are in the same archive.

## Results in numbers
- EditMode tests (run in-process in the same launch): **18 pass, 0 fail** (`selftest.txt`).
- `MANIFEST_COMPARE`: the default (summer) look against the committed seed-1337 manifest, ignoring the scene path: **IDENTICAL**.
- Build time about 130 s per forest. Whole batch 707 s.

## What is reference quality
**Nothing in this folder is reference quality yet.** The time-of-day sweep is the strongest part. Seasons did not take effect on near vegetation (see below).

## Time of day (`timeofday/`, 3x3 sheet `contact_sheet_cinematic.jpg`, `contact_sheet_sensor.jpg`)
Date is day 230 (mid-August), latitude 47.5. The sky, sun, fog and exposure grade change continuously and plausibly from 05:30 to 21:00.

| Frame | Review |
|---|---|
| 01 05:30 | Good. Low warm sun, orange sky through the canopy, long soft shadows. Usable. |
| 02 07:00 | Good. Clean morning light, no blown highlights. Best "normal day" frame. |
| 03 09:00 | Good. Crisp sun patches on the ground, rich greens, healthy shadow contrast. Best daytime frame. |
| 04 12:00 | Good. Neutral, not blown out, strong shadows (check hot spots: none seen). |
| 05 16:00 | Good. Sun glare and god rays through the canopy are convincing; slightly hazy. |
| 06 18:30 | Good. Warm golden low sun, copper bark glow. Cinematic grade works. |
| 07 19:30 | Blue hour reads well in tone (blue-violet, bright overcast gaps). DEFECT: 3-4 glaring white blobs in the canopy (see "Open defects"). |
| 08 21:00 | Too dark and wrong: almost black; the same bright blobs dominate. Not usable. |
| 00:00 | Black except the blobs. No stars, no moon (the moon is below the horizon that night). Not usable. |

Sensor versions: 12:00 is a sensible neutral frame. 19:30 is bright and washed-out because auto-exposure runs at its gain limit; the blobs are NOT present in sensor frames (cinematic-only, so related to the night exposure and bloom path).

## Seasons (`seasons/`, sheet `contact_sheet_cinematic.jpg`)
**Season kit not applied to near vegetation.** Autumn, spring, late autumn and winter-bare frames show green broadleaf crowns, green ferns and the same ground as summer, and winter is not bare. Per-build diagnostics show the generator never created any `Season_*` materials or `season_*` textures; the cause is still unknown (the code path reads correct, and the unit tests on `SeasonModel` pass). What DID apply (via `c.season`, not the kit): far forest and far-ground colour. In `cinematic/hero_autumn_golden_aerial.jpg` the far canopy is orange-brown and the near forest is green. A diagnostic log line (`seasonalKit=`) was added for the next launch.

## Cinematic hero shots (`cinematic/`)
- `hero_autumn_golden_corridor.jpg`: lovely light, golden sun through the trees. Not autumn: green foliage. Keep as a golden-hour hero only.
- `hero_autumn_golden_aerial.jpg`: honest status: near green, far orange-brown caps; far forest is a recognisable improvement in colour variety but looks like mushrooms.
- `hero_blue_hour_aerial.jpg`: good blue-hour sky and horizon haze, dark-blue canopy; some small glowing blobs.
- `hero_night_clearing.jpg`, `night_sky_stars_x1/x10.jpg`: black. No stars visible at either multiplier. Not usable.

## Iteration-3 defects (`defects/`)
- **Far terrain**: now lit and per-tree textured, no more flat band or the dark-triangle impostors, but the far forest reads as large cream/olive mushroom caps and cones, too bright in the sun and clearly low-poly. Better than a flat band; not reference. The black edge at the far terrain horizon is still there (aerial wide).
- **Speckled crowns from above**: reduced (no tint above 1.0, lower gloss); less white speckle in `aerial_wide_golden.jpg`. Not fully gone.
- **Person target** (`target1_golden_dof.jpg`): a red jacket and dark trousers are visible (colour variation works), but the body is still mostly hidden by ferns and sticks, and the limbs are untextured cylinders. Improved, not fixed.

## Open defects and questions
1. Seasons: kit not applied (above). Highest priority for the next launch.
2. Night: bright glaring blobs in cinematic frames only; black night with no stars. Candidates: bloom on a few high-value pixels at the -5.5 EV limit, or leaf translucency lit by the sky-fill light. `starMultiplier` brackets x1/x10 show no stars: the star cubemap or the PBS space emission path is not producing output, or sits far below the night exposure.
3. Dark night sensor frames: the sensor profile keeps the legacy limits (-1..14 EV100), so a moonlit forest (EV100 -3 to -5) is under-exposed. Decision: keep as is for now; this is an open sensor-profile question.
4. The defect frames were rendered at 16.51 h / 17.57 h, the old legacy hours; on day 230 the sun is at 24 / 14 degrees, not 15 / 4.5. Golden hour on this date is 17.44 h.
5. HDRP sun attenuation: assumed that the physically based sky attenuates the 128000 lux sun itself. Daytime and golden-hour frames look right, so the assumption holds.
