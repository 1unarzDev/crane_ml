# I1 — Underwater domain specialist: audit, sources, test design, reference model

- Repo: `/home/lunarz/crane_ml` @ `976ff1e` plus uncommitted user work. No existing file was modified. New files only: `Tools/Underwater/*` and this handoff.
- Method: static reading of C#, URDF, prefab and scene YAML (script GUID → `m_Script`, prefab-instance `propertyPath` overrides), `python -I` arithmetic, and web verification of licences and coefficients at primary sources (GitHub API and publisher pages). Unity was **not** launched, no ROS graph was started, nothing was cloned or executed from a third party.
- Labels follow `CONTRACTS.md`. Today nothing underwater is LIVE_SIM_VALIDATED or PHYSICALLY_CALIBRATED. Per the coordinator, **the current Sub hydrodynamics is not evidence-grade.** `Tools/Underwater/fossen6dof.py` is the analytic oracle for a future flag-gated replacement.

## 0. Summary

| # | Finding | Label |
|---|---|---|
| U1 | The only underwater vehicle (`Sub.prefab` in `Robosub Pool.unity`) has **no added mass, no Coriolis, no body-frame damping matrix, no current and no tether**. Its hydrodynamics is surface-patch Archimedes buoyancy, plus `GeneralDynamics` (V3 defects, live), plus PhysX damping 10/10. | IMPLEMENTED (defective) |
| U2 | The base_link inertia is **356 / 399 / 250 kg·m²** for an 11.5 kg, 0.30 × 0.22 × 0.21 m body. A solid box gives 0.085 / 0.127 / 0.131, so the stored value is about 3000× too large. Rotational dynamics are frozen. | VERIFIED (YAML + arithmetic) |
| U3 | Sub is **top-heavy**: CB is at the box centroid (base origin), and CG sits 1.25 mm *above* it (vertical thrusters at +0.032 m), so BG ≈ −1.25 mm. It is statically unstable in roll and pitch. The absurd inertia (U2) and PhysX angular damping hide this. With realistic inertia the capsizing e-folding rate would be about 1.4 s⁻¹. | VERIFIED (arithmetic); live behaviour NOT_RUN |
| U4 | Net buoyancy is +3.5 N (B = 130.6 N from the 0.01336 m³ collision box, W = 127.1 N for 12.96 kg). That is plausible for a ROV, but the value is accidental. No profile states it. | VERIFIED (arithmetic) |
| U5 | `Constants.waterViscosity = 1.0016` is used as a **kinematic** viscosity in `Re = vL/ν` (`SubmersionLib.cs:333-334`, `Constants.cs:5`). Water is 1.0016 *mPa·s* dynamic, about 1.0e-6 m²/s kinematic. Re is therefore about 10⁶× too small, and the ITTC-57 `Cf` comes out on the wrong branch: it **rises with speed**, and it is singular at vL ≈ 100. In Robosub the Reynolds length is also Blastoise's 5.4 m (`hullZMin/Max` −2.5/2.9, `Robosub Pool.unity:149-150`) on a 0.30 m vehicle. | VERIFIED (new) |
| U6 | Sub thrusters run in torque mode (`SubmarineThrusterConfig.asset`, `controlMode: 0`) on free rotors (`m_JointFriction: 0`, damping 0, I = 0.001 kg·m²), and no propeller load torque is modelled. **Hypothesis:** any non-zero command spins up to the 200 rad/s clamp, so steady thrust is about ±40 N (−32 N reverse) whatever the command magnitude, i.e. bang-bang. | HYPOTHESIS (H-U1) |
| U7 | The 6-thruster layout (4 vectored horizontal, 2 vertical at x = 0) is rank 5: **pitch is unactuated**, and `FullOmniController` has no pitch channel. That is acceptable for a RoboSub-class vehicle, but it rules out 6-DOF sysid on this hull. | VERIFIED (`FullOmniController.cs:88-134`, test) |
| U8 | Sub has **no sensors**: no camera component, no IMU, no DVL, pressure, USBL or sonar. The URDF `<sensor>` tags are inert. The Robosub scene has HDRP underwater rendering (`underWater: 1`, absorption distance 15.1 m) but no ROS sensor publishing. | VERIFIED |
| U9 | Everything (Submersion patch, Buoyancy, Thruster depth, Current) queries the HDRP `WaterSurface`. Underwater physics is therefore rendering-coupled exactly like the boat. `Current` is surface-only (`largeCurrentSpeedValue`, horizontal, no depth profile), is absent from Robosub, and is really a per-face quadratic drag. | VERIFIED |
| U10 | The new reference model, the BlueROV2 Heavy profile and **49 invariant/fixture tests pass** (`python -I -m pytest Tools/Underwater -q`). | FIXTURE_VALIDATED (Python only) |

## 1. Audit: what exists and what is enabled

Script GUIDs are resolved against `Assets/Robot Descriptions/sub/Sub.prefab` and `Assets/Scenes/Robosub Pool.unity`.

| Piece | Where | Enabled? | Notes / defects |
|---|---|---|---|
| `sub.urdf` | `Assets/Robot Descriptions/sub/urdf/sub.urdf` | Imported into `Sub.prefab` (UrdfImporter components) | base_link mass 3 kg (URDF), overridden to 5 (prefab) and then **11.456 (scene)**. Inertia 249.7/356.2/399.4 kg·m² (URDF); the prefab stores 356/399/250 (axis-permuted). Thruster links 0.25 kg with inertia 4.6–5.9 kg·m² (also about 10³–10⁴× too large). Six thrusters: 4 horizontal at ±45° (z=0), 2 vertical at (0, ±0.159, +0.032). URDF `<sensor>` tags are inert. |
| `Sub.prefab` | — | Root of the Robosub scene instance | A variant of source prefab GUID `4c9cfd3e…`, which is in neither `Assets/` nor `Packages/`. The scene also references a missing prefab `f9c9782e…` named "boat". The collision audit (`PerformanceEngineering.md:596`) shows that the Sub instance loads, so this is UNKNOWN-risk, not breakage. PhysX `m_LinearDamping 10` and `m_AngularDamping 10` on base_link. |
| `Submersion` + `SubmersionLib` | `Physics/Processing/Submersion.cs`, `SubmersionLib.cs` | Sub base_link, enabled; `waterSurface` set by scene override (`Robosub Pool.unity:1374`) | Hull = `sub.obj` box, 0.216 × 0.206 × 0.301 m. The volume uses a signed vertical-prism method against an HDRP surface patch. That is correct for a closed hull and fully submerged, but its float32 prism volumes grow with depth, so cancellation error grows too (UNKNOWN magnitude; test at 10 m and 50 m). The triangle normal is the mean of the vertex normals. |
| `Buoyancy` | `Physics/Water/Statics/Buoyancy.cs` | Sub base_link, enabled | Vertical ρgV force at the submerged centroid. Restoring moment arises only implicitly through CB−CG. Fine in principle, but the geometry gives BG < 0 (U3). Only base_link is buoyant; thruster volumes are ignored. |
| `PatchVoxelizedBuoyancy` | `Physics/Water/Statics/PatchVoxelizedBuoyancy.cs` | Not referenced by Sub or Robosub | Rigidbody-only. It reads `Assets/Data/Voxels/*.json` in `Awake`. `sumOfPositions/numberOfPoints` is NaN when nothing is submerged. CB is an unweighted voxel mean (partially submerged voxels get weight 1). |
| `PlaneVoxelizedBuoyancy` | `.../PlaneVoxelizedBuoyancy.cs` | Not referenced | Rigidbody-only. `buoyancyText.text` throws an NRE if unassigned. NaN with zero submerged points. Test harness only. |
| `GeneralDynamics` | `Physics/Water/Dynamics/GeneralDynamics.cs` | **Added in-scene to Sub** (`Robosub Pool.unity:128-150`), both terms on, `debugPressureDrag: 1` | V3 (normals transformed twice, quadratic term constant), unused suction falloff, local-unit area (I1-defect-verification "New findings"). **U5:** wrong viscosity units and Blastoise's 5.4 m hull length. |
| `FossenDynamics` | `.../FossenDynamics.cs` | **Not** on Sub, not in Robosub | Its defects V1/V2 would matter more underwater: M_A is comparable to m in heave (18.7 vs 13.5 kg for BlueROV2 Heavy), so the legacy sign would give a/a_true = (m+M_A)/(m−M_A) = 2.8 in surge, and in heave (M_A > m) the explicit recurrence diverges. |
| `Current` | `.../Current.cs` | Not on Sub | Surface HDRP current only (`largeCurrentSpeedValue` = 0 in Robosub). Horizontal (`force.y = 0`), no depth profile. Quadratic drag on every face's full area, so it double-counts as hull drag. |
| `Thruster` + `SubmarineThrusterConfig` | `Actuators/Motors/Thruster.cs`, `Assets/Config/SubmarineThrusterConfig.asset` | 6 in-scene components | thrustK −0.001, ω_max 200 → 40 N forward / 32 N reverse. Thrust is scaled by HDRP depth. No reaction torque (D16), latent D18, torque mode on a free rotor (U6/H-U1). |
| `FullOmniController` | `Controllers/FullOmniController.cs:88-134` | In-scene | Keyboard only. Mixes linearly in torque command. No pitch channel. No ROS command path for the Sub. |
| Sensors | — | none | No camera, IMU, DVL, pressure, USBL or sonar on Sub. The scene has `ROSClock` and `MAVROSConnection` only. HDRP underwater fog: absorption 15.1 m, scattering colour (0, 0.27, 0.23). |

**6-DOF effect checklist (Sub today → reference model):**

| Effect | Sub today | `fossen6dof.py` |
|---|---|---|
| Rigid-body M_RB with CG offset, C_RB | PhysX, with wrong inertia (U2) | yes (parallel axis, skew-symmetric C) |
| Added mass M_A (full 6×6 symmetric) and C_A Munk | **absent** | yes (−M_A ν̇_r; skew C_A(ν_r)) |
| Linear and quadratic damping D(ν_r) | GeneralDynamics (defective) + PhysX 10/10 | yes (diagonal quadratic, full linear) |
| Restoring g(η) with r_g/r_b | implicit via Buoyancy at CB; BG < 0 | yes (Fossen eq. 4.6) |
| Current (irrotational, constant in {n}) | absent | yes, with the ν̇_c term |
| Thruster dynamics | free-rotor spin-up (H-U1) | first-order lag + saturation; von Benzon eq. 19 recorded |
| Allocation | linear mixer, rank 5 | T matrix, pinv, direction-preserving saturation |
| Tether | absent | taut-only spring-damper fixture (cable model deferred) |

## 2. Source models (licences verified at primary sources on 2026-10-20/21)

| Source | URL @ version | Licence → GPL-3.0 | Coefficients and where | Meshes |
|---|---|---|---|---|
| **von Benzon et al. 2022**, BlueROV2 Heavy | doi 10.3390/jmse10121898; AAU PDF `vbn.aau.dk/ws/portalfiles/portal/505520780/jmse_10_01898.pdf` | Paper CC-BY-4.0. We cite facts. **OK** | **Table A1**: m, ∇, I (CAD), r_b, added mass (Eidsvik empirical), linear and quadratic damping (*experimental*), thruster geometry, tether. Eq. 18 thrust curve, eq. 19 thruster TF. Table 4 maximum speeds and forces. | n/a |
| von Benzon code | github.com/ROV-Simulator/ROV-Simulator @ `928bcbf` (2025-01-10) | **No licence → not usable** (binaries) | — | — |
| Wu 2018 thesis (Flinders) | `flex.flinders.edu.au/.../ThesisWu2018.pdf` | No statement (UNVERIFIED). Cite facts only. | Table 4.2 Heavy thruster arms; p.41 angles; Tables 5.1–5.3 a-priori values (standard BlueROV2 numbers from Sandøy, not measured) | n/a |
| CentraleNantesROV/bluerov2 | @ `dd7111b` (2026-09-23) | Apache-2.0 → **OK (one-way)** | `bluerov2_description/urdf/hydrodynamics.xacro`: Wu added mass; Einarsson & Lipenitis 2020 damping. `bluerov2.xacro` mass 14.8. **Inertia = RexROV2/100, so suspect.** | no separate mesh licence (Apache by repo) |
| uuvsimulator/rexrov2 | @ `1546681` (archived 2020-06-15) | Apache-2.0 → **OK** | `rexrov2_description/urdf/rexrov2.gazebo.xacro` (Berg 2012 NTNU thesis pp. 28–31), inertia in `rexrov2_base.xacro` | `.dae`, repo licence |
| uuvsimulator/lauv_gazebo | @ `3529e02` (2019-04-26) | Apache-2.0 → **OK** | `lauv_description/urdf/gazebo.xacro` (volume 0.017551, CoB, 6×6 damping, added mass). The da Silva 2007 citation is UNVERIFIED in the file. | README placeholder, no mesh licence: treat as UNKNOWN |
| uuvsimulator/uuv_simulator | @ `bfb40cb` (archived) | Apache-2.0 (+ `3rd-party-licenses.txt`) → **OK** | Fossen `hydrodynamic_model` plugin (structure reference) | — |
| Field-Robotics-Lab/dave | @ `e54ea16` (2023-12-04) | Apache-2.0 → **OK** | RexROV description. No BlueROV2. | one texture folder has its own licence file (unread) |
| Stonefish / stonefish_ros2 | @ `b21eb8e` (2026-07-14) / `6646e7a` (2025-12-04) | GPL-3.0(-or-later) → **OK (same licence)** | No BlueROV2 scenario upstream. Girona500 `.scn`/hydro meshes. Third-party BlueROV2 scenarios (Apache-2.0) not read. | Girona500 data in repo |
| HoloOcean | PyPI metadata (repo gated) | Client MIT. **Engine/assets under the Unreal EULA → not usable for assets** | BlueROV2 Heavy agent (8 thrusters). No coefficients published. | Unreal EULA |
| bluerov_ros_playground; R0mB0r / HKUST bluerov2_heavy | — | **No licence → not usable** | — | — |
| Blue Robotics T200 data | `cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx` | No terms stated (UNVERIFIED). Cite, don't redistribute. | Thrust vs PWM/voltage | — |

**Decision:** `Tools/Underwater/profiles/bluerov2_heavy.json` uses **von Benzon Table A1** as primary. It is the only source whose damping is experimental on a Heavy, and its numbers are internally consistent: our model reproduces Table 4's maximum surge speed (0.72 m/s) and heave speed (0.71 m/s) within 3 % from the Table 4 forces, and 4 × F(1) × cos 45° = 86 N against the paper's 85 N. Wu 2018 is kept as `alternatePrior`, and the test `test_wu_alternate_prior_is_inconsistent_with_heavy_performance` shows that it predicts about 2 m/s surge, so it must not be used for a Heavy. Thruster geometry comes from Wu Table 4.2, because von Benzon's printed β₃ = 3π/4 is not a 45° direction (recorded as a CONFLICT). Mass, CG and thruster positive-command signs need hardware measurement. No mesh is adopted. CRANE should model the BlueROV2 Heavy frame from published dimensions (Table 5: 0.46 × 0.38 × 0.58 m) or a new CC0/GPL mesh. CentraleNantes Apache meshes are acceptable with NOTICE attribution if wanted.

## 3. Test environments and tests

### 3a. Analytic fixtures (implemented in Python; each becomes a Unity EditMode/PlayMode falsifier later)

| Fixture | Closed form | Python test |
|---|---|---|
| Neutral-buoyancy hover | ν̇ = 0 at rest when W = B and r_b is directly above r_g | `test_neutral_buoyancy_hover_is_an_equilibrium` |
| Net buoyancy | ẇ = −(B−W)/(m+Z_ẇ) | `test_net_buoyancy_initial_heave_acceleration_includes_added_mass` |
| Restoring moment | K = W·BG·sin φ, so GZ = BG sin φ; pure couple | `test_restoring_moment_has_righting_arm_bg_sin_phi` (1°–90°), `test_top_heavy_vehicle_has_negative_bg_and_capsizing_moment` |
| Roll period | T = 2π√((I_x+K_ṗ)/(W·BG)) | `test_small_angle_roll_period_matches_closed_form` (0.5 %), `test_heavy_roll_period_and_restoring` (3.67 s) |
| Added-mass step | u̇ = F/(m+X_u̇). The legacy sign gives ≥1.9× | `test_added_mass_step_response_uses_m_plus_ma_and_not_legacy_sign` |
| Damping step | u = F/X_u·(1−e^{−t X_u/(m+X_u̇)}) | `test_linear_damping_step_response_matches_exponential` (1e-6) |
| Energy invariant | D=0: E = ½νᵀMν + U is constant. D>0: ΔE = ∫ν_rᵀD(ν_r)ν_r dt and E is monotone | `test_kinetic_plus_potential_energy_is_conserved_without_damping` (1e-6 rel., coupled M_A, W≠B, CG≠CB), `test_damping_decay_matches_dissipated_power_and_is_monotone` (1e-4) |
| Skew C, Munk routing | νᵀCν = 0. The inventory counterexample gives X=0.8, Y=−3.0, N=−0.4 | `test_coriolis_is_skew_symmetric…`, `test_munk_counterexample_from_inventory…` |
| Current-relative velocity | drift → v_c; same acceleration under the Galilean shift | `test_vehicle_drifts_with_current…`, `test_damping_depends_only_on_relative_velocity` |
| Quadratic scaling | F ∝ v² (contrast with V3) | `test_quadratic_damping_scales_with_speed_squared` |
| Allocation 8 / 6 thrusters | rank 6 / rank 5. The unreachable direction is pitch coupled with surge as −z_t·X; saturation preserves direction | `test_eight_thruster…`, `test_six_thruster_layout_cannot_command_pitch`, `test_saturated_allocation_preserves_wrench_direction`, `test_heavy_layout_is_fully_actuated` |
| Thruster lag | 63 % at τ; clamp | `test_thruster_first_order_lag_reaches_63_percent_at_tau` |
| Tether | 0 when slack, k·Δl when taut | `test_tether_is_zero_when_slack_and_restoring_when_taut` |
| Identifiability failures | only m+M_A is observable; constant speed gives no M_A and no linear/quadratic split; hover gives only W−B; static roll gives only z_gW−z_bB | `test_mass_and_added_mass_cannot_be_separated…`, `test_constant_speed_data_cannot_identify…`, `test_static_hover_only_identifies…`; positive control `test_rich_surge_excitation_recovers…` (2 %) |
| Sensors | p = p_atm + ρg·depth (lever arm, ρ mismatch); Janus DVL LS (bottom vs water track, range loss, noise cov σ²(BᵀB)⁻¹); USBL; Beer–Lambert | `test_sensor_models.py` (7 tests) |

### 3b. Unity scenes (proposed; none created)

All three are graphics-free-capable. A new `CraneUnderwaterMedium` supplies a flat surface plane, density, current field and turbidity, so no `WaterSurface` is required. HDRP underwater rendering is an optional visual layer driven *from* the medium, never the source of physics state.

1. **`Assets/Scenes/Underwater/UW Tank.unity`**: a 4 × 2 × 1.5 m test tank with a rigid surface plane at y=0, still water and no current. Hosts EditMode-equivalent PlayMode fixtures: hover, release-from-roll, step and coast in each DOF, free decay, bollard pull (fixed-joint load cell). Pass = matches the `fossen6dof.py` trajectory within 1 % RMS on identical inputs and timestep.
2. **`Assets/Scenes/Underwater/UW Pool.unity`**: a 25 × 12.5 m pool, 2 m shallow end ramping to 4 m deep (known geometry, CSV-defined), with lane-line textures, ArUco/AprilTag panels at surveyed poses and a flat floor for DVL. Tests: DVL bottom-lock loss above max range and over the ramp, pressure ↔ depth, camera tag-pose RMSE vs ground truth, wall-proximity sonar returns.
3. **`Assets/Scenes/Underwater/UW Harbor.unity`**: a 200 × 100 m basin, 3–15 m bathymetry heightmap, pier pilings and a quay wall. A depth-sheared current (power-law profile with a 0.5 m/s surface cap, spatially varying and seeded) and a turbidity gradient (c from 0.1 to 1.5 m⁻¹ across the basin, plus a thermocline layer). Tests: station-keeping in current, USBL tracking from a quay transceiver, tether drag, visibility-degraded detection.

### 3c. Sensor emulation plan

| Sensor | Model | Ground truth / validation | ROS msg |
|---|---|---|---|
| Pressure/depth | p = p_atm + ρ g d at the port lever arm; σ, quantisation, 1st-order lag, ρ configurable (fresh/sea) | analytic (implemented) | `sensor_msgs/FluidPressure` + `geometry_msgs/PoseWithCovarianceStamped` (z only) |
| DVL | 4-beam Janus (22.5° tilt, 45° yaw offset). Per-beam raycast to colliders (bottom track) or to a water layer at a configurable range cell (water track). Per-beam σ; lock loss on range or < 3 beams; altitude = min slant·cos tilt; latency | analytic LS (implemented); pool ramp scene | `nav_msgs/Odometry`-style velocity with covariance plus raw beams (custom `crane_msgs/DvlBeams`; no ROS-standard msg) |
| IMU | Reuse `CraneROSNavigationState.ToRosAngularVector` (V4-correct axial conversion), body frame, specific force incl. gravity; bias random walk | analytic step/spin fixtures | `sensor_msgs/Imu` |
| USBL | range/bearing from a fixed transceiver, two-way acoustic latency d/c, elevation-dependent σ, multipath dropout near walls | analytic (implemented) | `geometry_msgs/PointStamped` + covariance |
| Imaging sonar (FLS, e.g. Oculus-class) | Batched raycasts over a fan (azimuth × elevation spread) binned by range, intensity ∝ cos(incidence)·reflectivity/r² with speckle (Rayleigh), TVG; polar image | wall at known range and angle in tank | `sensor_msgs/Image` (polar) + `marine_acoustic_msgs/ProjectedSonarImage` if available |
| Camera attenuation | Post-process on the existing async camera: I = J e^{−c d} + B∞(1−e^{−c d}) per RGB channel, using the depth buffer (only after H6 metric-depth is resolved) or geometric depth | colour chart at known distances in tank | existing `RosCameraAsync` topic, plus `camera_info` |

### 3d. Public datasets for BlueROV2 validation

| Dataset | Licence | Use |
|---|---|---|
| DFKI "ROV and tether tracking" (Zenodo 17360027, 2025; Sci. Data s41597-025-06347-0) | CC-BY-4.0 → usable | BlueROV2 MAVLink attitude, raw IMU, pressure, **thruster PWM/commands**, mocap pose, tether tension. Best candidate for held-out sysid of drag and thrust curve, and the tether model. The PWM claim comes from a search snippet; verify before preregistering. |
| CRB20/bluerov2-system-identification | UNVERIFIED | Logs thruster PWM. Do not use until a licence exists. |
| SubPipe (Zenodo 12666132) | CC-BY-4.0 | AUV visual-inertial and side-scan. Sensor-model validation only. |
| AQUALOC | UNVERIFIED | ROV camera/IMU/pressure. Local evaluation only, pending licence. |
| Mallios underwater caves (Zenodo 7828405) | CC-BY-NC-SA-4.0 → **not GPL-compatible for redistribution** | DVL + scanning sonar realism check, local only. Never vendor it into the repo. |

## 4. Prototype (created, NEW files only)

- `Tools/Underwater/fossen6dof.py`: NED/FRD, quaternion, RK4. M_RB (parallel axis), skew C_RB/C_A (Thm 3.2), D(ν_r), g(η), irrotational current with the ν̇_c term, thrusters (lag + saturation), allocation, taut tether, energy and power diagnostics. Parameter validation rejects negative-stored M_A (the V1 class of error), asymmetric M_A and non-dissipative D.
- `Tools/Underwater/identifiability.py`: regressors plus a rank/condition/null-space analysis.
- `Tools/Underwater/sensor_models.py`: pressure, Janus DVL, USBL, Beer–Lambert.
- `Tools/Underwater/vehicle_profile.py` + `profiles/bluerov2_heavy.json`: every coefficient is `{value, source, location}`. The loader rejects bare numbers, unknown sources and a `PHYSICALLY_CALIBRATED` claim. `evidenceLabel: UNKNOWN`.
- Tests: `test_fossen6dof.py` (32), `test_sensor_models.py` (7), `test_vehicle_profile.py` (10). `python -I -m pytest Tools/Underwater -q` → **49 passed** (≈16 s, Python 3.14.8, numpy 2.5.4).

## 5. Proposed Unity asset/scene plan (not created)

Rule: nothing below references `UnityEngine.Rendering.HighDefinition`, `WaterSurface`, `Submersion`, `Buoyancy`, `GeneralDynamics`, `FossenDynamics` or `Current`. A new asmdef enforces this at compile time.

| New file | Purpose |
|---|---|
| `Assets/Scripts/Underwater/CraneUnderwater.asmdef` | References `UtilsAssembly`, `Unity.Robotics.ROSTCPConnector(.Messages)` only. **No HDRP.** |
| `Assets/Scripts/Underwater/Dynamics/CraneFossenModel.cs` | Pure C# (no MonoBehaviour) port of `fossen6dof.py`: M, C(ν), D, g, current term, in FRD/NED doubles. Unit-testable. |
| `Assets/Scripts/Underwater/Dynamics/CraneUnderwaterBody.cs` | MonoBehaviour on a single **Rigidbody** (not an articulation tree). PhysX damping 0, `useGravity=false`, inertia set from the profile. Each FixedUpdate converts Unity↔FRD/NED with one explicit, tested frame map (`bodyFromUnity` rotation, no implicit "+Z = bow"). It applies the *whole* hydrodynamic + restoring wrench. Added mass is handled implicitly by solving ν̇ with M_RB+M_A and applying `F = M_RB ν̇_target` (no finite-difference feedback). Implements `ICraneEpisodeResettable`. |
| `Assets/Scripts/Underwater/Dynamics/CraneUnderwaterProfile.cs` | Loads `bluerov2_heavy.json` (same schema), records the profile SHA-256 in replay metadata, fails closed on unknown or uncited values. |
| `Assets/Scripts/Underwater/Environment/CraneUnderwaterMedium.cs` | Scene singleton: surface height (plane), ρ, depth-sheared current field, turbidity c(x,y,z). Optionally *drives* HDRP fog/absorption for visuals. |
| `Assets/Scripts/Underwater/Actuation/CraneUnderwaterThrusterBank.cs` | T matrix from profile, thrust curve eq. 18, dynamics eq. 19 (or first-order), saturation that preserves direction, reaction torque, stamped ROS wrench/PWM input through `CraneActionGate` (bounded policy). |
| `Assets/Scripts/Underwater/Actuation/CraneUnderwaterTether.cs` | Lumped-mass cable (N segments, Cn/Ct drag, from the Table A1 tether) to a surface anchor. |
| `Assets/Scripts/Underwater/Sensors/CranePressureSensor.cs`, `CraneDvl.cs`, `CraneUsbl.cs`, `CraneImagingSonar.cs`, `CraneUnderwaterCameraAttenuation.cs` | As in §3c. Stamp with `Clock.time` (avoids D12), journal acquisition. |
| `Assets/Robot Descriptions/bluerov2_heavy/bluerov2_heavy.urdf` + `BlueROV2Heavy.prefab` | Visual/collision primitives from Table 5 dimensions. Inertial from profile (not URDF-importer defaults). |
| `Assets/Scenes/Underwater/UW Tank.unity`, `UW Pool.unity`, `UW Harbor.unity` | §3b |
| `Assets/Tests/Editor/Underwater/CraneUnderwater.EditorTests.asmdef` + `CraneFossenModelTests.cs` | Golden vectors exported from `fossen6dof.py` (JSON under `Tools/Underwater/golden/`) compared to the C# model to 1e-9. Asserts the assembly has no HDRP reference (reflection on `GetReferencedAssemblies`). |
| `Assets/Tests/PlayMode/Underwater/…` | Tank fixtures vs the Python trajectories (1 % RMS). |
| CLI flag `--crane-underwater-profile <json>` | Opt-in. Robosub Pool and the existing Sub are untouched (the legacy path stays as the A/B baseline). |

## 6. Tests / falsifiers still to add (Unity side, NOT_RUN)

- A C#↔Python golden-vector match (C, D, g, ν̇) at 1e-9. Any frame or sign slip fails it.
- PlayMode: hover drift < 1 mm over 60 s; release-from-roll period ±1 %; step/coast ±1 % RMS.
- **H-U1 falsifier:** log Sub thrust vs `FullOmniController` command at 0.1/0.5/1.0. If steady thrust is about 40 N for all three, bang-bang is confirmed.
- **U3 falsifier:** set the Sub inertia to box values in an isolated test copy and observe capsizing. Expected e-folding ≈ 0.7 s.
- **Submersion depth precision:** fully submerged box at 1, 10 and 50 m, compare against V = 0.013360 m³ and CB = centroid.

## 7. Risks

- Literature coefficients carry large spread (Wu vs von Benzon: 8× in X_u|u|). Added mass is empirical (Eidsvik), so it is a prior, not a calibration.
- Thruster command signs and the T3/T4 direction conflict can silently mirror yaw. Measure them on hardware or from the ArduSub motor matrix before any comparison.
- Graphics-free underwater physics would be new to CRANE. CONTRACTS forbid aquatic claims without validation, so the medium must be validated separately in the tank scene before any training claim.
- The missing nested-prefab GUIDs in Sub/Robosub could break on a fresh clone (UNKNOWN).
- Datasets: NC-licensed data must never enter the repo. The DFKI PWM content is unverified.

## 8. Next actions

1. Owner decision: approve the new `CraneUnderwater` assembly and the opt-in flag (Sub/Robosub stay as the frozen legacy baseline).
2. Export golden vectors from `fossen6dof.py` and port `CraneFossenModel.cs` with its EditMode test first (no scene needed).
3. Build the UW Tank scene with `CraneUnderwaterBody` + the BlueROV2 Heavy profile, and run the §3a fixtures in PlayMode.
4. Verify the DFKI dataset contents and licence, then preregister a held-out drag/thrust identification (`PREREGISTRATION.md`).
5. Record U2/U3/U5/U6 in the inventory and the defect list, and correct `CRANE-Fidelity-BaselineAudit.md:13` (the Robosub "validated benchmarks" claim) via the owner.
