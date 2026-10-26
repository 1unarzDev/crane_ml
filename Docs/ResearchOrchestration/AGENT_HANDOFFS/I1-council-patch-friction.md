# I1 council: PhysX improvedPatchFriction (systems critic)

**Scope.** This council covers the project default `m_ImprovedPatchFriction: 0` (`ProjectSettings/DynamicsManager.asset:36`, `m_FrictionType: 0` = patch). The question is whether its ~2× effective-μ inflation reaches CRANE evidence, and how to handle it.

**Constraints followed.** The main checkout was not modified. The only commit is in worktree `crane-terrain`: `1a4059c` on `feature/terrain-slopes`, parent `0947e6b`. Nothing was pushed.

## 1. Reproduction (UNITY_EDITMODE_EXECUTED, Unity 6000.5.10f1 batchmode, isolated preview PhysicsScenes)

**Test.** `Assets/Tests/Editor/Terrain/PatchFrictionCouncilTests.cs`, `[Category("PatchFrictionCouncil")]`.

**Logs.**
- `/tmp/crane-council-pf/run1-unity.log`: run 1, which used the high-chassis rigs.
- `/tmp/crane-council-pf/unity.log` and `results.xml`: run 2, 5/5 passed.

**Method.** Every effective μ is μ_eff = F_onset/N, found by bisection on force or angle. Both bodies use the same material with combine Average.

**Box on ramp** (1×0.2×1 m, 5 kg, dt 0.02):

| μ | atan μ | onset, flag off | μ_eff off (ratio) | onset, flag on | μ_eff on |
|---|---|---|---|---|---|
| 0.2 | 11.31° | 21.95° | 0.403 (2.02×) | 11.55° | 0.204 |
| 0.4 | 21.80° | 38.56° | 0.797 (1.99×) | 21.97° | 0.403 |
| 0.6 | 30.96° | 48.96° | 1.149 (1.91×) | 31.00° | 0.601 |
| 0.8 | 38.66° | 56.53° | 1.513 (1.89×) | 38.55° | 0.797 |

This reproduces E36 independently. With the flag on, the onset error is ≤0.25°.

**Locked wheel, lateral push along the axle** (1.5 kg, rotation frozen, dt 0.01, r = 0.033 m, width 0.018 m), μ ∈ {0.2, 0.4, 0.6, 0.8}:

| Shape | Flag off | Flag on |
|---|---|---|
| Box face 0.018×0.066 m | **2.000×** | 1.000× |
| Sphere (campus default) | 1.000× | 1.000× |
| Capsule along the axle | 1.000× | 1.000× |
| 64-gon prism, facet down | 1.000× | 1.000× |
| 64-gon prism, edge down | 1.000× | 1.000× |
| Verbatim campus rounded convex tire (`--crane-campus-convex-tires`) | 1.000× | 1.000× |

**ArticulationBody 4-wheel rig** (frictionless root, revolute wheels, lateral force on the root):
- Sphere, capsule and prism: 0.99–1.00× in both settings.
- Campus rounded tire: 0.95–1.01× in both settings. It sits slightly *under* nominal at μ ≥ 0.6. This does not depend on the flag.
- Run 1 put the root COM 6 cm higher. There the rounded tire read 0.65–0.99× and the sphere read 1.06× at μ 0.8, in both settings. This is a rig/tipping sensitivity, not inflation.

**Footprint probe** (locked square box, μ 0.4):
- Flag off: a 10 mm square gives 1.00×, and every square from 18 mm to 200 mm gives 2.00×. The jump is a step, not a ramp.
- Flag on: 1.00× for all sizes.
- Mechanism (HYP): PhysX patch friction adds a second friction anchor once the patch spans more than an internal threshold, which here falls between 14 and 25 mm diagonal. With the flag off, each anchor is then clamped to μ·N_patch.
- The 18×3.2 mm prism facet stayed at 1×.
- The PhysX docs' `frictionOffsetThreshold` of 0.04 m does **not** match this threshold. Unity's scaling is unknown.

**HingeJoint rig: INCONCLUSIVE (rig defect), now `[Ignore]`.**
- Run 1 gave a μ-independent onset: 1.367, the tipping ratio. The capsule read 0.147 and the convex wheel slid at 2%.
- Run 2 never slid up to 2.4 g.
- None of this is friction evidence. The contact-level question is answered by the locked-wheel and articulation rigs instead.
- The `[Ignore]` attribute was added after run 2 and committed without a re-run. It changes only the attribute.

**Not run:** longitudinal traction while rolling, and drive limits under torque. These are expected to be 1× for point contacts, because the contact type is the same. That expectation is HYP.

**Conclusion.** The inflation is a *patch* effect, and only appears for flat face contacts wider than about 2 cm. The campus wheel contacts (spheres, and the 18 mm convex tire) are **not** inflated in either setting.

## 2. Runtime settability

- `UnityEngine.Physics.improvedPatchFriction` has a public static get/set (UnityEngine.PhysicsModule.xml:3248).
- Probe `RuntimeToggle_AppliesToAlreadyCreatedScene` showed:
  - At 30° with μ 0.4: off-fresh moves 0.001 m and on-fresh moves 6.8 m.
  - A scene created with the flag off and then switched on **slides** (6.8 m).
  - A scene created with the flag on and then switched off **holds**.
- So the flag is read live by every PhysicsScene when it simulates. It is **process-global**. It is not captured per scene, and there is no per-PhysicsScene override.
- In practice it can be set **per run/process** from a profile at bootstrap, before the first step. It cannot be set per scene inside one process.
- After the test runs, `DynamicsManager.asset` in the worktree was not dirtied, because the tests restore the flag in TearDown.

## 3. Impact on existing evidence

No calibration fits μ from simulation. The calibration fits only mass, lag, acceleration, torque, wheel scale and LiDAR noise (`Tools/Campus/calibration_profile.py:24-32`). The campus surface μ values are `engineering-prior-unmeasured` (`Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json:45-131`), and rubber is 0.9/0.8 Min (`CraneCampusRobotModel.cs:61`). Nothing has absorbed the inflation, and nothing needs un-absorbing.

| Evidence | Friction path | Exposed to the 2×? |
|---|---|---|
| Concrete→dirt CAL-05, commit 8b9787c | Geometry only (bevels and start pose). No μ was changed. Runs on the contact branch. | No: the wheel contact is 1× |
| Campus contact-wheel branch (default for `run_calibration.sh`, CAL-01/02/04/05/08, slalom 976ff1e/a4275c7) | Sphere or 18 mm convex wheel on PhysX. Casters and chassis μ = 0. | No (measured 1.00×). Its friction remains *prior*, not calibrated. |
| Campus force branch | Scripted μ from a raycast lookup. Wheel spheres μ = 0. | No |
| TurtleBot3 warehouse (`DifferentialDriveDynamics`) | No wheel colliders. Chassis μ = 0, Min (`CranePerformanceBuild.cs:89-93`). | No: 2×0 = 0 |
| Jackal chassis | μ = 0 (`CraneReferenceEnvironmentImport.cs:164-168`) | No |
| RoboBoat (ArticulationBody hull, BoxColliders, null material = 0.6) vs marina boxes | Any contact is already a failure (`RoboBoatDockingEvaluator.cs:86-110,176`, contactCount == 0) | No claim depends on it. Post-contact rebound in a failed run would see 2× hull-box friction. |
| Terrain slope-v1 (I1-terrain PhysX ramp tests) | Box faces | Yes. This is already handled: the analytic test sets the flag on, and the default test is a tripwire that asserts the flag is off. |
| Underwater and aerial (new) | Sub or skid face contacts with the floor or ground are likely | Yes, if they contact flat faces wider than about 2 cm |

Side finding: `Assets/Editor/CraneCampusWheelDiagnostic.cs:23,28` labels the floor "normal-concrete" but uses the rough-concrete values .85/.7225. This is a labelling mismatch, not a patch-friction issue.

## 4. Recommendation: B′, a flagged per-process setting with provenance, with A as a gated follow-up

- **Reject C** (a documented μ correction factor). No single factor exists: it is 2.00× for flat patches of 18 mm and up, and 1.00× for points and narrow patches. A factor would be wrong for exactly the contacts that matter.
- **B′:**
  - A new opt-in `--crane-physics-patch-friction=improved` (or the profile field `physics.improvedPatchFriction`) sets the flag once at bootstrap, before the first `Simulate`.
  - The terrain slope-v1, underwater and aerial profiles default it to on.
  - The run header records the effective value as provenance, on every run including legacy (`physics.improvedPatchFriction=false`).
  - Fail closed if two profiles in one process request conflicting values, because the flag is global.
  - Legacy defaults and ProjectSettings stay unchanged, which satisfies CONTRACTS (no silent change to validated behaviour). EditMode tests must save and restore the flag (pattern in the council test).
- **A later (promotion gate).** The measured exposure of the legacy land and boat evidence is nil. The project default can therefore be flipped in a recorded DECISIONS entry after an A/B gate:
  - Re-run CAL-01/02/04/05/08 and slalom with the flag on, at least 5 repeats each.
  - Every gate must stay inside its run-to-run repeatability.
  - Boat docking contactCount must be unchanged.
  - Then update the I1-terrain tripwire test.
  - That makes the change explicit, not silent.

## 5. Risks and open items

- **Global-state leakage.** Any Editor code or test that sets the flag without restoring it could be saved into ProjectSettings by the Editor, which would be a silent default change. Recommend a CI check on `m_ImprovedPatchFriction` in `DynamicsManager.asset`.
- **The threshold mechanism is HYP.** Wider tires would cross it: Husky-class mesh wheels about 0.1 m wide, skid or track boxes, box feet, pallet pushes, and skids. Any new vehicle with flat contact faces wider than about 2 cm needs the flag on, or a per-geometry check.
- **Under-friction.** The rounded convex tire reads ~5% low at μ ≥ 0.6 in the articulation rig, and is rig-sensitive. This is separate from the flag and unresolved.
- **HingeJoint rig is unvalidated.** It is not evidence about joint dynamics. Rolling traction under torque is NOT_RUN.
- **Single author.** All of the above needs independent verification before any default flip.
