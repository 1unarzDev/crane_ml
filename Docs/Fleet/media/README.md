# CRANE modular fleet: media (curated 2026-11-10)

Curated by the periodic README agent (brief: `Docs/Fleet/README_CURATOR.md`). **Simulation only: nothing here is measured on hardware.** Unity frames come from the CRANE player (Unity 6000.5.10f1, Vulkan, Linux, scene `fleet/warehouse_a/v0`) and were converted to JPEG. The full-resolution PNGs are in `~/.cache/crane-fleet/`. Plots are made only from existing result files; each one states n, intervals, the decision ID and "simulation only".

## Current frames
`OK` means usable in a README. `NOT VALID` means a consumer such as the DecisionCal README agent must skip the frame.

| Frame | Status |
|---|---|
| `cad_v1/compare_scout.jpg`, `cad_v1/compare_lifter.jpg`, `cad_v1/compare_cargo.jpg` | OK: design renders at concept/datasheet level |
| `cad_v1/lifter_places_box_small_on_cargo.jpg` | OK: static CAD placement check, not a run |
| `runs/box_small_seed1_t104.7_perception.jpg` | OK: physical lift (failure frame) |
| `runs/box_long6kg_seed1_t68.8_capacity_crop.jpg` | OK: over-capacity refusal |
| `runs/box_small_seed5_t102.1_configuration.jpg` | **NOT VALID**: lifter-to-deck contact unverified |
| `runs/box_long5kg_seed1_t70.1_tipped_crop.jpg` | **NOT VALID** as tip evidence: payload not in frame |
| `findings/crane_load_batches.jpg`, `findings/robust_feasibility_info.jpg`, `findings/e2fix_valid_arms.jpg`, `findings/fleet_p1_summary.jpg` | OK |
| `findings/xm_causes.jpg` | OK, labelled EXPLORATORY |
| `scenes/*.jpg` (all 8) | OK: schematic or benchmark renders, not CRANE runs |
| `demo_seed1_v3/00_establish`, `01*`, `03b`–`03d`, `04b`, `06`, `07` | OK with caveats (section 5) |
| `demo_seed1_v3/02_lift`, `04a_wide_handoff` | **NOT VALID**: kinematic carry of 6 kg over a 5 kg rating |
| `demo_seed1_v3/05_through_doorway` | **NOT VALID** until checked: possible box-jamb interpenetration |
| `assets_v0/*` | SUPERSEDED design (pre-CAD) |

Ready-to-paste block for the main README: `Docs/Fleet/media/README_SECTION.md`.

## 1. CAD: current design is CAD v1 (`cad_v1/`)
- **`compare_scout.jpg`, `compare_lifter.jpg`, `compare_cargo.jpg`**: each puts the user's concept on the left and the v1 parametric CAD (build123d) on the right.
  - Shared chassis: 342 × 260 × 85 mm, with 97 mm mecanum wheels and Pololu 25D 47:1 gearmotors.
  - Docks: a reciprocal J-hook + pin dock on each face, and a 30 mm AprilTag on a 45° bevel.
  - Scout: RPLIDAR C1 and D435, with a proxy arm.
  - Lifter: jaws and counterweight. These sheets come from `Tools/Fleet/assets/cad_v1/images/` and are unchanged since iteration 7.
  - Cargo: raised deck.
  - User decisions: D87.
  - *Critique:* concept/datasheet level, not measured. The arm is a proxy. The electronics look smaller than in the concept by design (the width is 0.26 m). There is no contact or mass validation in these renders.
- **`lifter_places_box_small_on_cargo.jpg`** (D89/D90, iteration 7): the lifter, with its 170 mm stroke and 0.24 m jaws, sets box_small (2 kg) side-on onto a cargo deck.
  - *Critique:* this is a static CAD placement check, not a simulated run. The rating is 2.0 kg at safety factor 1.51, a margin under 1 %. End-on set-down interferes by 4–8 mm (D90), so only the side-on pose is valid. The CRANE scene still runs the v0 rovers until v1 is imported.
- **Superseded:** `assets_v0/` holds the provisional pre-CAD sheets (perspective, top, mating close-ups, hook cutaways). They are kept for reference only, and their geometry is not the current design.

## 2. Runs: CRANE physical-load batches, new library (`runs/`)
These are replay frames from the newest completed batches: `~/.cache/crane-fleet/requests/load4_*`. They use the composite-body load model and wheel cylinders. The verdict comes from simulator truth.

| Frame | Run id / seed | CRANE commit | Verdict |
|---|---|---|---|
| `box_small_seed1_t104.7_perception.jpg` | `load4_box_small_1`, seed 1, t = 104.7 s | `df67914c` | not delivered: first failure Perception (receiver cargo_1 not localised for the handoff, σ 1.6 m); payload still carried |
| `box_small_seed5_t102.1_configuration.jpg` | `load4_box_small_5`, seed 5, t = 102.1 s | `df67914c` | not delivered: first failure Configuration ("lifter_1 holds nothing"); payload on the decks, outside the delivery zone |
| `box_long6kg_seed1_t68.8_capacity_crop.jpg` | `load4_box_long6_1`, seed 1, t = 68.8 s | `c2d13662` | **Capacity**: the lift stalled at its force limit (rated 5.0 kg) and the attempt was refused; payload returned to the pallet |
| `box_long5kg_seed1_t70.1_tipped_crop.jpg` | `load4_box_long5_1`, seed 1, t = 70.1 s | `0dc05957` | **tipped**: end-gripped 5 kg; the library logs Geometry "gripper closed on nothing" |

*Critique:*
- **box_small seed 1.** The lift is physical (no kinematic attach), and the box rides in the jaws. The lifter is close to the cargo pair, but this frame does not show whether they touch. D86 attributes both box_small failures to the handoff carry, which treated the receivers as non-obstacles. That fix is upstream, and its Unity verification is pending.
- **box_small seed 5.** The shelf in the foreground hides the lifter. The lifter's jaws appear to meet cargo_2's deck edge, and the overlap is unverified. Do not use this frame as evidence of a clean set-down.
- **box_long 6 kg, crop.** This is the top-left quarter of the 960 × 540 frame, around the lifter. The box is back on the pallet after the refusal. Truth still labels the payload `unsupported_or_tilted`, because the pallet is not modelled as a support (D82, known gap).
- **box_long 5 kg, crop.** The payload is not visible in any of the four replay frames, so the tip verdict rests on the logs and statics (D86), not on this image. The lifter looks pitched, but the camera is far away. This frame is not physical evidence.
- **Missing:** none of the 8 box_small successes was rendered (replays exist only for failures). A rendered within-rating delivery is still missing.

## 3. Findings (`findings/`)
- **`crane_load_batches.jpg`** (D82, D86): first-outcome shares per batch.
  - box_small 2 kg: delivered 8/10 (Wilson 95 % 0.49–0.94).
  - box_long 6 kg: Capacity refusal 10/10.
  - box_long 5 kg end-gripped: tipped 3/3.
  - 0 false successes in all batches.
  - The kinematic-carry 10/10 (D77) is deliberately not plotted: it is not physical evidence.
- **`robust_feasibility_info.jpg`** (D84, which revises D72): in game core v6 with the action set held fixed, feasibility information raises servable delivery on the large-fleet family H4F: +0.108 [0.067, 0.154] at 4 jobs and +0.040 [0.022, 0.058] at 16. At 16 jobs it beats timing information by +0.028 [0.010, 0.046]. The bottom row shows where the D72 null came from: the all-bays action set costs −0.096 [−0.144, −0.051]. The effect is null on train, and score gains sit just below the noise floor.
- **`e2fix_valid_arms.jpg`** (E2_PREREG amendments 1–3, the D76 decisive test; validity per D83): held-out success with the executor clearance fix, for the 5 arms that do not use observations. n = 43 configurations × 2 episodes. oracle − nominal = −0.035 [−0.081, +0.012], so the D76 threshold is not met. **Belief arms are excluded:** D74 is suspended (D83), and the corrected belief re-run has no D-entry yet. Nothing from D74 is plotted.
- **`xm_causes.jpg`** (D81, **EXPLORATORY** pilot, 15–17 train instances):
  - At drift 3, idealising one subsystem alone removes this share of the 40 failures: tracking 60 %, localisation 60 %, timeouts 38 %, recovery 10 %. The shares overlap.
  - Invalid plans from docking constraints: UD-TAPF 14/15, Zhou-style team formation 10/15.
- **`fleet_p1_summary.jpg`**: the figure from the paper draft `papers/fleet_p1` (claim frozen in D28-fleet; baselines in D46-fleet). It shows what survives each of the three stress tests. It is a timing model and planner-level evidence, not executions; the paper caption states this.

## 4. Scenes: modular-robotics testing scenes (`scenes/`)
- **`warehouse_a_v0_topdown.jpg`**: the CRANE warehouse scene, drawn from `Tools/Fleet/spec/scenarios/warehouse_a_v0.json`: 10 × 7 m, a 0.80 m doorway, a shelf, a pallet stack, an elevated gantry and two overhead cameras. *Critique:* all geometry is provisional and the drawing is schematic.
- **`scenes_v1_index.jpg`**, **`scenes_v1_narrow_door.jpg`**: scenes v1 (D85, `Tools/Fleet/scenes/previews/` on research/fleet). The index shows 9 families as fastsim top-down previews, each with the plan it tests.
  - Families that discriminate in fastsim: narrow_door, long_payload_door, lateral_slot, blocked_faces, charging_contention, wheel_failure.
  - Weak families: aisle_traffic, friction_patches, clutter_observer.
  - *Critique:* these are fastsim plots, not CRANE renders. The CRANE loader changes are untested in Unity. In CRANE, door sills are solid boxes (untested) and floor patches are visual only (D86).
- **`mapf_key_ii_detour.jpg`**: a WoMAPF key instance, fixed topology against reshaping, solved exactly (makespan 17 vs 15). *Critique:* it is a hand-built illustration. On the shared capplan families, reconfiguration rescued 0 of 234 instances (D59).
- **`fleetgame_A_v1012_t60.jpg`**: a fleet-game core audit replay, two auctions on the same instance with the divergence step marked. *Critique:* it is a grid abstraction, not physics.
- **`capplan_configs.jpg`**: one example configuration each from train and held-out H2–H6 (`Tools/Fleet/capplan/manifests/`). *Critique:* the footprints are a uniform schematic, not per-type geometry. A red outline marks a degraded actuator.
- **`gazebo_blocked_a_overview.jpg`**, **`gazebo_mission_oblique_018_t65.jpg`**: Gazebo modular benchmark (Blocked-A layout), copied read-only from `~/mrmn-main/paper/iros2026_codesign/`. *Critique:* the Gazebo primitives are boxes and cylinders, and "pod 4 jammed" is the benchmark's own failure case. This is a different simulator from CRANE.

## 5. Older run: closed-loop mission, seed 1 (`demo_seed1_v3/`, player at aae01b90)
This is the only end-to-end CRANE mission with frames (D77: 10/10 delivered on that library). It was made before the physical load model. Run manifest: `demo_seed1_v3/run_manifest.json`. The frames run 00 → 07: establish, scout finds the payload, lift, formation capture, dock close-ups (03b–03d), handoff, payload on the decks, doorway, scout view, done.

*Critique:*
- **`02_lift`–`04a`: NOT physically valid.** The carry is kinematic: 6 kg box_long against a 5 kg rating. The physical model now refuses this lift (D82), and an end-gripped box_long tips even at 5 kg (D86).
- **`05`:** the box may interpenetrate the door jamb. This is unverified.
- **Wheel collision shapes.** At this time they were spheres 2.35 cm wider than the rendered wheels. They have since been replaced by cylinders.
- **Dock frames.** The shell hides the hooks in `03b`–`03d`.

## Removed this cycle (git history keeps them)
- `load_box_long6_oldlib/`: superseded by the new-library batch in section 2.
- `scene_v0/`: the first qualitative spawn, superseded by section 4.
