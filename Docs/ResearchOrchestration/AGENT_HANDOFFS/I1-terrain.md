# I1 — Ground-vehicle complex-terrain audit, sources, design and prototype

Author: I1 complex-terrain specialist. Date: 2026-10-21. Backlog item 12.
Repo: `crane_ml` @ `976ff1e` plus uncommitted user work. I only read the user work: no existing file was modified, stashed or reset. Unity was not launched, and no ROS graph or hardware was touched. I made no git writes or pushes, and did not touch any MHSeals repository.
New files: `Tools/Terrain/{terrain_reference.py, terrain_generator.py, robot_profiles.py, test_terrain_reference.py, test_terrain_generator.py, terrain_course_validator.py, test_terrain_course_validator.py, profiles/*.json}` and this handoff.
Revision 2 (2026-10-21) applies the corrections from `I1-verify-terrain.md` (claims 1 and 4 were PARTIAL) and decision D12.
Evidence labels follow CONTRACTS.md. Labels used here:
- **VERIFIED-STATIC**: read from the source or scene YAML.
- **CX**: a `python -I` counterexample on a transcription of the C# formula.
- **HYP**: hypothesis, not yet checked.

`python -I -m pytest Tools/Terrain -q` → **44 passed** (1.0 s; revision 1 had 29, Python 3.14.8, pytest 9.1.1).

## 1. Audit — how land contact is modeled today

### 1.1 Three distinct land plants

| Plant | Where / when | Contact model | Slope behavior | Label |
|---|---|---|---|---|
| **Legacy diff-drive** | `DifferentialDriveDynamics.cs:49-68`. TurtleBot3 Warehouse scene; used when `--crane-campus-model` is absent. | No wheels. A `VelocityChange` servo with a clamp of 10 m/s² per step. Lateral velocity is cancelled every step (ideal nonholonomic constraint). The chassis uses `TurtleBot3 Chassis Slip` (scene `TurtleBot3 Warehouse Validation.unity:5053-5064`): μs=μd=0, combine Minimum (2). So **all** tangential support is scripted. | Ignores μ entirely, because chassis friction is 0. The 10 m/s² clamp exceeds g·sinθ at every slope, so the servo cancels the velocity, **but it does not hold the robot still.** The servo runs before PhysX adds g·sinθ·dt, so the robot **creeps downhill at g·sinθ·dt**, independent of μ: 0.139 m/s at 45° with the project default dt=0.02 (`TimeManager.asset`), and 0.069 m/s at the campus dt=0.01. The same creep appears on cross slopes through the lateral cancel. **CX C1 (corrected):** at 45° with μ=0.6, Coulomb says the robot slides freely (tan45° > 0.6). The legacy plant instead gives a dt-dependent creep with no μ dependence. Verifier: PARTIAL→corrected. | VERIFIED-STATIC + CX |
| **Campus force branch** | `CraneCampusRobotModel.cs:175-196`. Active with `--crane-campus-model` but without `--crane-campus-contact-wheels` (`CRANE_CAMPUS_CONTACT_WHEELS=0`). | Scripted. Per-side surface found by raycast. Drive acceleration is clamped to `min(Acceleration·traction, μd·g, 2T/(r·m))`. This clamp ignores slope, so cosθ normal load and gravity demand are absent from the limit. Rolling resistance is Crr·g. Lateral is a velocity P-loop `-v_lat/0.1 s`. Yaw torque acts about **world** up. The wheel spheres use the zero-friction chassis material. | **CX C2** (rough concrete, A=0.6): the drive limit is 0.57 m/s². Max climb is **1.9°**. Max hold at zero command is 4.8°. The campus `loading-ramp` is 5°, so it can neither climb nor hold it, while the analytic reference holds it (tan5°=0.087 ≪ 0.85). **CX C3:** steady cross-slope creep = 0.1·g·sinφ = **0.17 m/s at 10°**, where Coulomb predicts static. The V5b wheel-scale sign defect (I1-truth-inventory) also lives here. | VERIFIED-STATIC + CX |
| **Campus contact branch** (default for `run_calibration.sh:19` and `run_campus_nav2_fixture.sh:75`) | `CraneCampusRobotModel.cs:157-173, 198-219` | Two wheel Rigidbodies on `HingeJoint`s. Each is a sphere, or a convex 64-gon with `--crane-campus-convex-tires`. Rubber is μs 0.9 / μd 0.8, combine Minimum with the surface. A PI torque servo gives ≤0.35 N·m with reaction on the chassis. The two casters are **frictionless** spheres. Rolling resistance is scripted as Crr·g along body forward (no cosθ; error 1−cosθ). **No slip-ratio or slip-angle curve:** PhysX Coulomb patch friction only. **No suspension:** the axle is rigid. Load transfer emerges from rigid contact. | Physically plausible for static and traction slope limits (gravity and friction both come from PhysX). Analytic predictions with the profile prior: traction climb **31.9°**, static hold 35.8°. The torque limit is not binding: 2·0.35/0.033 = 21 N > W = 15 N. **NOT_RUN:** no slope run exists. | VERIFIED-STATIC; slope behavior HYP |
| **Ackermann WheelCollider rover** | `AckermannRoverDynamics.cs`; scene `Land Vehicle Validation.unity` | PhysX `WheelCollider`. Suspension spring 18000 N/m, damper 2200, travel 0.2 m. Forward curve extremum (0.35, 1.0) → asymptote (0.8, 0.65), stiffness 1.35. Sideways (0.2, 1.0) → (0.5, 0.75), stiffness 1.6 (scene `:337-350`). Body 60 kg, 4 × 6 N·m motor torque, r = 0.25 m. Brake = rolling torque only. Unity "slip" is PhysX's internal slip, not SAE slip ratio (HYP; check against Unity docs). | Torque-limited max grade **9.4°** (CX: asin(24/0.25/588.6)). Under throttle the code applies **no** rolling term: rolling torque is applied as a brake only when throttle and brake are both 0 (`:135-136`). The 8.5° in revision 1 wrongly assumed Crr=0.015 under throttle. When coasting it creeps on any slope > 0.86° (asin Crr). Verifier: PARTIAL→corrected. No terrain in that scene. | VERIFIED-STATIC + CX |

### 1.2 Terrain that exists

There are no heightmaps, `TerrainCollider` or `TerrainData` anywhere in `Assets/Scripts` (grep).
- **Proving-ground catalogs v1–v8:** flat. The `Box` schema has no pitch (`CraneLandProvingGround.cs:51-59`). The v8 shared boxes are only `traversable-floor` and `boundary-wall`.
- **Campus geometry** (`industrial_logistics_campus_v1.json`):
  - one 5° `loading-ramp` (rough concrete), 3 m × 4 m;
  - eight test-pad bevels of 3.05–6.09°, 0.15 m long and 2 mm thick;
  - a 3 mm metal threshold, adjustable to ≤ 20 mm via `--crane-campus-threshold-height`.
- **Surface priors:** 8 surfaces, all tagged `engineering-prior-unmeasured`.

### 1.3 Gravity on slopes

- **IMU:** the campus IMU publishes specific force including gravity (`CraneCampusSensorSuite.cs:29`), so tilt is observable. The gyro sign defect V4 still applies.
- **Odometry:**
  - Wheel odometry is planar. `WheelPosition` integrates horizontally with constant y. On a slope θ it overstates horizontal travel by 1/cosθ, as real planar odometry also would.
  - `SlipProxy = |ω·r − v_body|/|ω·r|` is a driving slip ratio, but it is a diagnostic only.

### 1.4 Existing flat or near-flat land tests

These were collected statically; I did not re-run `Tools/Campus`:
- `Tools/Campus/test_drive_calibration_gates.py` (concrete→dirt transition, straight drive, arc, odometry disagreement);
- `test_surface_evidence.py`, `test_contact_evidence.py`;
- `test_ramp_evidence.py`, which only checks that the body ends up on the raised platform footprint and height, not the dynamics;
- `test_slalom_evidence.py`;
- `Tools/ReferenceEnvironments/test_land_proving_ground_manifest.py`;
- the `land_proving_ground_navigation_gates_v1-v3.json` gates.

There are no slope, tip, traction or step tests. There is no EditMode test of land physics.

### 1.5 Rover URDFs (`Assets/Robot Descriptions/rover/urdf/`)

- **Provenance (VERIFIED):** the files are byte-equivalent, modulo include paths, to `roverrobotics/roverrobotics_ros2@8f74c12` `roverrobotics_description/urdf/max.urdf`. That repo is **Apache-2.0** per the GitHub API.
- **Notice gap:** `THIRD_PARTY_NOTICES.md` has no Rover Robotics entry. Apache-2.0 is GPL-3.0-compatible, but §4 requires the notice. This is a **compliance gap** (it is not my file to edit).
- **Not used:** neither GUID (`bf9e1f18…`, `1bc7b5db…`) is referenced by any `.prefab`, `.unity` or `.asset`, so these robots are unused.
- **Non-physical contact values:** `<mu>5.0</mu>` inside `<link>`, and gazebo `mu1/mu2=200`.
- **Mass and geometry:** `max.urdf` has a 6.54 kg wheel mass versus a 6.80 kg chassis. Its `wheel_y_offset` 0.11 disagrees with the plugin `wheel_separation` 0.35 (HYP: the visual offset is not shown).
- **Conclusion:** they are not a credible terrain reference.

### 1.6 TurtleBot3 campus prior

The CG height comes out as 0.0198 m: COM y 0.009766 above a ground plane 0.010 m below the body origin. This gives an implausible 82° static lateral tip angle (fixture `test_turtlebot_prior_cg_height_implies_implausible_tip_angle`). HYP: the COM was set relative to the collider frame rather than the ground. It must be measured before any tip-over claim is made.

## 2. Source models (licenses checked at the primary source)

### 2.1 Robots

GitHub API `license.spdx_id`. Files were inspected in `/tmp/crane-terrain-src/<name>/`; none was executed.

| Robot | Repo @ commit | License (code + meshes) | Published coefficients (file:line) | Use |
|---|---|---|---|---|
| Clearpath Husky A200 | `husky/husky@41e15d2` (noetic-devel) | BSD-3-Clause; meshes are in the same repo | wheelbase 0.512, track 0.555, r 0.1651, width 0.1143 (`husky.urdf.xacro:155-161`). Chassis 46.034 kg, COM (−0.00065, −0.085, 0.062), full inertia (`:197-199`). Wheel 2.637 kg (`wheel.urdf.xacro:31`). Gazebo μ 1.0 (`:50`). **`wheel_separation_multiplier: 1.875`** (`husky_control/config/control.yaml:26`). | **Primary terrain profile** → `profiles/husky_a200.json` |
| Clearpath Jackal | `jackal/jackal@4ddf9b5` | BSD-3-Clause | wheelbase 0.262, track 0.37559, r 0.098 (`jackal.urdf.xacro:5-11`); 16.523 kg, COM z 0.067 (`:115-120`); wheel 0.477 kg (`:44`); μ 0.5 (`:55`); multiplier **1.5** (`control.yaml:25`) | Second skid-steer profile → `profiles/jackal.json` |
| AgileX Scout v2 | `agilexrobotics/scout_ros2@bf0110f` | Apache-2.0 | wheelbase 0.498, track 0.58306, r 0.16459, base 40 kg (`scout_v2.xacro:19-24,58`) | Candidate, heavier skid-steer |
| Leo Rover | `LeoRover/leo_common-ros2@e4975e1` | MIT | r 0.0625 (`leo_description/urdf/macros.xacro:529`). Revolute rocker joints (`:180-183`). Masses at `:16,39,74,107`. | **Best rough-terrain candidate**: passive rockers keep all four wheels in contact |
| F1TENTH | `f1tenth/f1tenth_gym@4fdb9c7` | MIT | μ 1.0489, C_Sf 4.718, C_Sr 5.4562, lf 0.15875, lr 0.17145, h 0.074, m 3.74, I 0.04712 (`gym/f110_gym/envs/f110_env.py:130`) | Ackermann single-track tire reference (flat only) |
| Unitree (legged) | `unitreerobotics/unitree_ros` | BSD-3-Clause | not extracted | **Not justified now**: it needs a separate contact or controller stack, and wheeled slopes are the gap |

### 2.2 Terramechanics

- **Project Chrono** `projectchrono/chrono@19257fe`, BSD-3-Clause (LICENSE verified).
  - The SCM API is `SetSoilParameters(Kphi, Kc, n, cohesion, friction°, Janosi J, K_elastic, damping)`.
  - LETE sand: Kphi 5301e3, Kc 102e3, n 0.793, c 1.3 kPa, φ 31.1°, J 0.012 m, K 4e8, R 3e4. Source: `src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp:227-235`, commented alternative.
  - Demo soft soil: 0.2e6 / 0 / 1.1 / 0 / 30 / 0.01 / 4e7 / 3e4 (`:216-224`).
  - Units are [Kc]=N/m^(n+1) and [Kphi]=N/m^(n+2) (Chrono forum, R. Serban, 2024; secondary).
- **Bekker / Wong:** J.Y. Wong, *Theory of Ground Vehicles*. Bevameter soil table (sand, clays, loam) is cited by Chrono docs as Table 12.10; I did not verify the table or page. Copyrighted book: cite values only.
- **Iagnemma & Dubowsky**, *Mobile Robots in Rough Terrain* (Springer STAR 12, 2004): wheel–soil parameter estimation. Not inspected.
- **SCM:** Krenn & Hirzinger, "SCM – a soil contact model for multi-body system simulations" (secondary listing only).

### 2.3 Validation datasets

| Dataset | Source | License (verified where) | Signals useful for terrain | Status |
|---|---|---|---|---|
| RELLIS-3D | `unmannedlab/RELLIS-3D@c17a118` README:259 | **CC BY-NC-SA 3.0**: non-commercial, cannot be vendored into GPL-3.0 | Warthog platform. Lidar, IMU, GPS/odometry, semantic terrain classes | External validation only, never redistributed |
| TartanDrive 1.0 / 2.0 | `castacks/tartan_drive`, `tartan_drive_2.0` | Code MIT; **data license not stated in README** (UNKNOWN) | ATV. Odometry 50 Hz, IMU 200 Hz, **wheel RPM 50 Hz, shock travel 50 Hz**, local heightmap 501² at 20 Hz, controls (README:55-63) | Best slope/traction signal set. Get the data license before use |
| ORFD | `chaytonmin/Off-Road-Freespace-Detection` | Code MIT; data license UNKNOWN | Freespace segmentation | Perception only, weak for dynamics |
| CaT (CAVS) | cavs.msstate.edu; IEEE Access 10:24759 (2022) | UNKNOWN. Dataset Ninja claims CC BY 4.0, but the official page shows none | Traversability labels by vehicle class | Perception only |
| Slope-traction measurements (slip vs grade) | — | — | — | **Gap:** no openly licensed wheeled-robot slip-vs-grade dataset was found. CRANE must capture its own (see §3.4) |

## 3. Design

### 3.1 Analytic fixtures (`Tools/Terrain/terrain_reference.py`, all tested)

| Fixture | Formula | Test |
|---|---|---|
| Static slope limit | tanθ ≤ f·μs (f = load fraction on braked or driven wheels) | `test_static_hold_limit_is_atan_mu_and_boundary_is_sharp` |
| Tip-over | lateral atan((B/2 ∓ y_cg)/h); longitudinal atan(x/h) | `test_lateral_loads_vanish_on_uphill_side_at_tip_angle…`, `test_longitudinal_load_transfer…` |
| Load transfer | N_f = [W(x_r·cosθ − h·sinθ) − m·a_x·h]/L; lateral analogue with centripetal a_y | the same two tests |
| Climb limits | traction: μ·N_driven(θ) = W(sinθ + Crr·cosθ); torque: T/r = W(sinθ + Crr·cosθ) | `test_front_drive_climb…`, `test_torque_limited_climb` |
| Stopping distance | v²/[2g(f·μ·cosθ ± sinθ)], ∞ downhill once f·μ·cosθ ≤ sinθ | `test_stopping_distance_up_and_down_slope` |
| Skid-steer slip power | Coulomb 4-wheel steady turn, nested bisection. **Closed form B_eff = B(1 + (L/B)²)** | `test_skid_steer_coulomb_track_multiplier…`: it predicts **1.851** for Husky versus Clearpath's published 1.875, and **1.487** for Jackal versus 1.5 (both within 2 %). This is corroboration, not calibration. **Assumptions:** the closed form is exact for viscous friction, and for Coulomb friction **only with equal wheel loads and a fore/aft, left/right symmetric layout**. Unequal axle loads, a CG offset (Husky's −0.085 m) or slope load transfer break it; the solver still accepts unequal loads, but then no closed form applies. |
| Nonholonomic lateral-slip feasibility | \|v·ω ± g·sinφ\| ≤ μ·g·cosφ | `test_nonholonomic_lateral_slip_feasibility` |
| Step / ledge | rigid wheel needs h < r. Driven wheel: h ≤ r(1 − 1/√(1+μ²)). Pushed wheel: h ≤ r(1 − cos·atan(P/N)). **Assumptions:** quasi-static, single wheel. The driven-wheel bound is **weight-only** (no chassis push, no torque coupling between axles, no load transfer); it is a conservative per-wheel bound, not a general 4WD result. The pushed bound assumes a frictionless edge and horizontal P. | `test_step_climb_limits` |
| Identifiability | odom/true = s/(1 − i), with i = (sinθ + Crr·cosθ)/(C·cosθ) | `test_wheel_scale_and_slip_are_confounded_on_one_slope…`: one slope admits (s=1.02, C=12) and (s=1.00, C≈6.46) exactly; two slopes (or flat + slope) recover s and C to 1e-9 |
| CRANE counterexamples | transcriptions of the plants in §1.1. The legacy plant is transcribed **step by step** (`legacy_diff_drive_slope_trace`: script VelocityChange → gravity → semi-implicit integration), and the clamp constant is regex-read from `DifferentialDriveDynamics.cs`. Ackermann: `ackermann_torque_climb`, with the torque read from `AckermannRoverDynamics.cs`. | `test_counterexample_*` (C1–C3). `test_counterexample_legacy_drive_creeps_downhill_mu_independent` predicts both the creep and the clamp-limited capture transient (from v0 = −1 m/s it takes ⌈(1 − g·sinθ·dt)/((a − g·sinθ)·dt)⌉ steps). **Mutant check:** `test_mutant_without_clamp_turns_legacy_prediction_red` removes the clamp (a = ∞) and asserts that the oracle fails. `test_ackermann_torque_cap_has_no_rolling_term_under_throttle` gives 9.39° (8.52° only with Crr, coast onset 0.86°). |

**Implication for the calibration protocol.** A wheel-scale fit (`leftWheelScale`/`rightWheelScale` in `CraneCampusCalibrationProfile`) taken on any graded surface is confounded with slip. The scale must be fitted on flat, low-load runs (CAL-01/02) and then **frozen** before slip stiffness is fitted on ramps.

### 3.2 Procedural environments (`Tools/Terrain/terrain_generator.py`)

The generator writes a `crane-land-terrain-course-v1` manifest with:
- the seed and parameters;
- a heightmap SHA-256;
- a 16-bit RAW heightmap compatible with Unity `TerrainData` (2ⁿ+1);
- surface patches, reusing the campus surface ids (test-enforced to match the campus manifest);
- `groundTruth`: analytic vs measured max/mean slope, rock list, curvature radius;
- `evidenceLabel: NOT_RUN`.

`default_catalog()` contains:
- ramps at 0/5/10/15/20/25/30/35°, with a 2 m flat approach;
- cross slopes at 5/10/15/20°;
- moguls with A = 5/10/20 cm at λ = 4 m (analytic max slope atan(2πA/λ) = 4.5°/8.9°/17.4°);
- seeded rock fields at 0.05/0.2/0.5 rocks/m² (Poisson count, r 3–15 cm, half-embedded spherical caps; the max step height is recorded);
- mixed friction: concrete→dirt→epoxy in series, which extends the existing CAL-05 concrete→dirt bevel, plus a split-μ course (left concrete, right epoxy) that induces yaw;
- `soil-lete-sand-ablation`: a 6 m Bekker patch marked `ablation: true` and `requiresSolver: SCM-class`. PhysX cannot represent sinkage, so no claim may be made from it without an SCM-class solver.

### 3.3 Integration (proposals; **no Unity assets were created**)

**D12 course validator (implemented):** `Tools/Terrain/terrain_course_validator.py`, tested by `test_terrain_course_validator.py`.
- It infers the plant from the Unity args:
  - `--crane-campus-model` absent → legacy diff-drive;
  - `--crane-campus-model` without contact wheels → force branch;
  - `--crane-campus-contact-wheels` → contact branch;
  - `--crane-campus-slope-traction` (proposed) → slope-aware.
- Course slope is max(analytic, measured max slope).
- Runs on the legacy diff-drive or the force branch with slope > 2° (plus a 1e-6° rounding tolerance) are labeled **`INVALID_FOR_SLOPE`** with `evidenceUsable: false`, whatever label was claimed. No C# refuses slopes.
- Rock and mogul courses count as sloped.
- The deformable course is flagged as an ablation.
- It fails closed on an unknown plant or label, a schema mismatch, missing slope ground truth, or a heightmap SHA-256 or size mismatch.
- CLI exit codes: 0 usable, 1 invalid-for-slope, 2 input error.


**New C# files:**
- `Assets/Scripts/Physics/Land/CraneLandTerrainCourse.cs`, opt-in `--crane-land-terrain-course <id>`:
  - loads the manifest and `.r16` from `Assets/Resources/ReferenceEnvironments/terrain/`;
  - builds `TerrainData` and a `TerrainCollider` at runtime, as the campus does;
  - verifies `heightmapSha256` and fails closed;
  - writes evaluator truth in the `CraneLandProvingGround.EvaluatorTruth` style.

  A `TerrainCollider` takes a single `PhysicsMaterial`, so mixed friction needs one tile per patch. Each tile carries a `CraneCampusSurface` so the existing raycast surface sampler keeps working.
- `Assets/Scripts/Physics/Land/CraneSlopeAwareTraction.cs`: a **new**, flag-gated (`--crane-campus-slope-traction`) per-wheel friction-circle law for the force branch. It uses N_i from the analytic load transfer, F ≤ μ·N_i, and handles gravity explicitly. The force branch in the user's WIP `CraneCampusRobotModel.cs` must not be edited silently. The owner wires the hook (one call in `Step`) once they accept the design. Keep the current branch as the A/B baseline.
- `Assets/Tests/Editor/TerrainSlopeFixtureTests.cs`: EditMode tests with `Physics.Simulate` in an isolated `PhysicsScene`:
  - (a) a box on an inclined plane slides iff tanθ > μ, checking that PhysX Coulomb behaves with the Minimum combine;
  - (b) the contact-wheel robot with zero command holds at θ < atan(f·μ) and slides above it;
  - (c) cross-slope creep is zero below atan(μ).

  Python oracle values come from `terrain_reference.py`.

**Navigation gates** — proposed `Tools/Terrain/land_terrain_gates_v1.json`, consumed by a future `Tools/Terrain/terrain_evidence.py` over `telemetry.jsonl`:
- `staticHold`: drift < 5 mm over 5 s at zero command, for θ < atan(μ_eff) − 2°;
- `slideOnset`: within ±2° of analytic;
- `climbLimit`: within ±2° of `max_climb_traction` and `max_climb_torque`, whichever is lower;
- `crossSlopeCreep`: < 1 cm/s below atan(μ_lat);
- `tipOver`: roll at wheel lift within ±3° of `lateral_tip_angle`, using a measured CG;
- `stepClimb`: success iff h ≤ the analytic bound ± 2 mm;
- `skidTrack`: B_eff/B on concrete within 10 % of 1 + (L/B)²;
- `odomScaleFrozen`: wheel scale is fitted only on flat runs.

These extend the proving ground: terrain layouts get `diagnosticMechanism: terrain-*` and a `studySplit`, so they sit alongside the existing v1–v8 catalog without changing it.

**Robot:** import Husky via the URDF importer into `Assets/Robot Descriptions/husky/` (BSD-3, add a notice) as the slope study platform. Keep the TurtleBot3 for the campus.

### 3.4 Physical capture needed

On flat ground, 5°, 10° and 15° ramps of one surface, plus one cross slope:
- wheel encoder and IMU;
- ground-truth pose (mocap or total station);
- a pull test for μ;
- a measured CG (tilt-table or two-axle scale).

This is the minimum that makes slip stiffness and scale identifiable (§3.1).

## 4. Risks

- The force-branch and legacy counterexamples are transcriptions of the formulas, and the creep is derived, not run. A live PhysX run could differ through solver details (e.g. residual contact friction). Confirm them with the proposed EditMode tests before claiming LIVE_SIM_VALIDATED.
- Agreement on the Coulomb track multiplier could be coincidental. The Clearpath value may have been tuned on carpet or grass with unequal loads. Treat it as corroboration, not calibration.
- Husky's −0.085 m lateral COM looks like a description artifact (HYP). Tip-over gates must use a measured CG.
- TartanDrive, ORFD and CaT data licenses are UNKNOWN, and RELLIS-3D is NC. Do not vendor any of them.
- The PhysX `TerrainCollider` has one material per tile and no deformation. The soil course is an ablation only.

## 5. Next actions (owner decisions marked ★)

1. D12 (decided): `CraneSlopeAwareTraction.cs` is to be a NEW flagged component. The force branch and the legacy plant stay unchanged in code; their runs on courses > 2° are labeled INVALID_FOR_SLOPE by `terrain_course_validator.py` (done). Next: implement `CraneSlopeAwareTraction.cs` and its flag.
2. Add `TerrainSlopeFixtureTests.cs` (EditMode) and run it in Unity.
3. Add `CraneLandTerrainCourse.cs` plus `ramp-*`/`cross-slope-*` resources, then run the contact branch over the ramp sweep and compare against `max_climb_traction`.
4. ★ Add the Rover Robotics Apache-2.0 notice, or delete the unused rover URDFs.
5. Measure the TurtleBot3 CG height and replace the 0.0198 m prior.
