# CRANE fleet: simulator screenshots (as of 2026-11-10)

These frames were rendered by the CRANE Unity player: Unity 6000.5.10f1, Vulkan, Linux, warehouse scene `fleet/warehouse_a/v0`. They are converted to JPEG to keep the repo small; the full-resolution PNGs are in `~/.cache/crane-fleet/`. **Simulation only. Nothing here is measured on hardware, and every rover dimension is provisional (pre-CAD).**

## 0. CURRENT design: CAD v1 from the user's concept pictures (`cad_v1/`, 2026-11-10)
**Current frames for the CAD row:** `cad_v1/compare_scout.jpg`, `cad_v1/compare_lifter.jpg`, `cad_v1/compare_cargo.jpg`. Each puts the concept on the left and the v1 parametric CAD (build123d) on the right.
- **Shared chassis:** 342 × 260 × 85 mm, with 97 mm mecanum wheels (9 rollers, X pattern) and Pololu 25D 47:1 gearmotors.
- **Docks:** a reciprocal J-hook + pin dock on every face, plus a 30 mm AprilTag on a 45° bevel above it.
- **Role kits:**
  - scout: RPLIDAR C1, RealSense D435 and a proxy arm;
  - lifter: 0.21 m jaws and a counterweight, rated 2.0 kg at safety factor 1.5;
  - cargo: raised deck on standoffs.
- **User decisions (D87):** width 0.26 m, so the electronics look smaller than in the concept by design; the lifter carries the 2 kg box_small; bevel dock tags.
- **Caveats:** this is concept/datasheet level, not measured. The arm is a proxy. The CRANE scene still uses the v0 rovers in sections 1–4 until v1 is imported.
- **Sources:** code, URDFs and the design log are on branch `research/fleet-orchestration` under `Tools/Fleet/assets/cad_v1/`.
- **Superseded:** section 4 (`assets_v0/`) is the previous provisional design.

## 1. Closed-loop mission, seed 1 (`demo_seed1_v3/`, player at commit aae01b90, clean tree)
The fleet runs the autonomy library: EKF, capability-conditioned planner, docking controller, mission executive. Simulator truth issues the verdict (D77: 10/10 seeds delivered on this library). Run manifest: `demo_seed1_v3/run_manifest.json`.

| Frame | What it shows |
|---|---|
| `00_establish` | Wide view of the whole fleet at the start: scout (orange, sensor mast), lifter (blue), two cargo rovers (green). |
| `01_scout_finds_payload_onboard`, `01b_…_wide` | The scout finds the payload behind the shelf, from its onboard camera and from a wide view. |
| `02_lift` | The lifter grips box_long at one end and lifts it off the pallet. |
| `03a_wide_formation_capture` | The two cargo rovers line up and capture in the assembly zone. |
| `03b_dock_hooks_closing`, `03c_dock_latched`, `03d_dock_latched_shell` | Docking close-ups: magnet capture, then both hooks closing, then the verified latch. |
| `04a_wide_handoff`, `04b_payload_on_decks` | The lifter sets box_long down across both coupled decks. |
| `05_through_doorway` | The coupled pair carries the box through the 0.80 m doorway. |
| `06_scout_watches_onboard` | The scout's onboard view of the loaded pair at the doorway. |
| `07_done` | Rendered 0.3 s after the host's accepted verdict: the payload is in the delivery zone and the scout watches. |

**Render critique (honest):**
- **Lifter carry is kinematic.** In `02_lift` and on the way to `04a`, the payload was attached to the lifter rigidly, and box_long is 6 kg against a 5 kg rating. The physical lift model added since then shows this lift is impossible: the lift stalls at its force limit. An end-gripped box_long also tips even at 5 kg. **Do not read `02_lift`–`04a` as physical transport.** Deck support (`04b`–`07`) uses the physical payload with support probes.
- **Wheel contact.** The rendered rollers sit on the floor and nothing visibly floats. Until today, however, the wheel collision shapes were spheres 2.35 cm wider than the rendered wheel. That error caused a wheel-to-wheel contact in a later box_small run, and they are being replaced by true-width cylinders.
- **Hook engagement.** It is visible only in the cutaway asset sheets (`assets_v0/docked_*_sheet`). In-scene, the shell hides the hooks (`03b`–`03d`).
- **Doorway clearance.** In `05` the far end of the box appears to overlap the door jamb. It could be perspective or real interpenetration, and I have not verified which. Check it against the simulator's contact logs before using this frame.
- **Legibility.** The tags and role colours read well. The scene is dim, and the wide views are cluttered by the pallet stack in the foreground.

## 2. Over-capacity lift attempt, old library (`load_box_long6_oldlib/`, seed 1, t = 77–80 s)
These are replay frames from the batch where box_long (6 kg) is lifted with physical load. The lift stalls (58.9 N needed against a 49.05 N limit). On this older library the mission then ended in a Timeout or Geometry failure: 0/10 delivered, 0 false successes. On the new library the same attempt fails fast with an explicit `Capacity` failure and puts the payload back. That batch is still running, so there are no frames yet.

## 3. Scene, first qualitative run (`scene_v0/`, Nov 9)
Wide start, close start and assembly views from the first spawn of the four-rover fleet. These predate the autonomy library.

## 4. Provisional rover assets (`assets_v0/`)
Asset sheets for the scout, lifter and cargo rover, and docked pairs (cargo front to cargo rear, lifter to cargo, scout to cargo). The sheets include perspective, top, mating-plane close-up, and a cutaway showing the two hooks closed on their pins. All geometry is **PROVISIONAL** until CAD replaces it (`Tools/Fleet/spec/fleet_profile_v0.json`).

## Not here yet
- A within-rating physical-carry demo; the box_small batch is queued.
- A clip with physical load. The only clip so far is `~/.cache/crane-fleet/stills/autonomy_attempt1_dock_deadlock/clip_wide.mp4`, an early failed attempt kept as a failure record.
- Coupled-pair group control (M2), which the final reconfiguration demo needs.
