# RISK REGISTER

| ID | Risk | Mitigation |
|----|------|-----------|
| R1 | Overwriting large uncommitted user work in crane_ml / astro_dock / MHSeals checkouts | No edits to modified files without explicit coordination; new files only in I1 |
| R2 | Accidental write/push to MHSeals repos (local checkouts have MHSeals origins) | Read-only policy; agents told explicitly; no git push anywhere |
| R3 | Simulator-truth recovery mislabeled as real calibration | Evidence labels in CONTRACTS.md |
| R4 | Live hardware reachable from ROS domain | Isolated ROS_DOMAIN_ID, no hardware launch files |
| R5 | MHSeals packages are GPL-3.0 (mhseals_hardware has no LICENSE file, only the package.xml/setup.py declaration). crane_ml has no LICENSE at all. Any port into crane_nav/crane_hardware/crane_testing is a derivative work and must stay GPL-3.0 with attribution. | RESOLVED by D3: GPL-3.0 with per-file provenance headers |
| R6 | Live `reference` case fails, so expected-failure results are uninterpretable | Gate OPT-00: reference must pass before fault cases count; diagnose criterion vs sim defect |
| R7 | Literature prior suggests null result (sim fidelity ≠ better transfer); multi-fidelity BO may absorb the gap | Preregister; strongest baselines built first; report negative results |
| R8 | Real-trial requirement: P1–P6 need physical held-out trials CRANE does not yet have | Sim-only results labeled as such; physical data list in research memo |
| R9 | Correcting hydro un-tunes validated controller defaults | Council deciding between flag-gated profile and default change (I1-council-hydro.md) |
| R10 | Downstream consumers may compensate for the inverted /campus/imu gyro | Council searching consumers before the fix |
| R11 | `calibrated-*` result dirs could be read as physical calibration | Rename/label in docs (no deletes) |
| R12 | astro_dock devcontainer default ROS_DOMAIN_ID=42 is the boat's domain | crane_* sim launches refuse domains outside 200–228; isolated containers use --network none |
| R13 | Characterization procedures depend on Unity flags not on crane_ml main (--crane-calibration-faults, --crane-water-current, --crane-roboboat-contract, --crane-ros-thruster-pwm) | Mark those procedures NOT_RUN; merging branches is a later decision |
| R14 | Main-branch water runs (Fossen live: 1.69× surge, −450 N reset impulse) are not evidence-grade; main feed-forward 0.21/0.24 and 1.2 m/s top speed were measured on the defective stack | Label them; do not mix main- and branch-plant results |
| R15 | Loosening the 0.05 m/s cross-axis criterion just to pass `reference` | Only a derived tolerance (F1/F2) may replace it |
| R16 | hull_drag fault is invisible pre-fit (0.034 < tolerance) | Detect drag faults through speed-dependent metrics, not cross-axis |
| R17 | Reverse rows worsen after the decoupling fit (`decoupling_negative` null). This is a possible fitter defect in the upstream characterization code (read-only), which crane_testing would inherit | Reproduce in Tools/Adversarial before porting crane_testing |
| R18 | Rover URDFs appear to be Apache-2.0 Rover Robotics copies with no third-party notice | RESOLVED: notice appended to THIRD_PARTY_NOTICES.md (additive; user text untouched) + LICENSE-APACHE-2.0.txt (sha256 cfc7749b…) in the rover dir |
| R19 | BlueROV2 damping literature disagrees by up to 8× | Profile carries a prior range; calibration must abstain if data can't narrow it |
| R20 | Sub prefab / Robosub scene reference missing prefab GUIDs | Do not build on them; new scenes use new prefabs |
| R21 | The ArduPilot bridge would flip a vehicle; a user might connect a real autopilot | Legacy bridge refuses by default (D14); hardware never connected |
| R22 | Concurrent Unity batch instances exhaust RAM | ≤2 at once |
| R23 | Project-wide PhysX patch-friction inflation (~2×) may be absorbed into existing land calibration μ values and validations | RESOLVED D17: flat patches ≥2 cm only; wheels unaffected; new profiles set it at startup and record it in the header |
| R24 | The A1 frame defects may also affect the boat SITL bridge path (D sign, roll/pitch sign, world gyro) | Assess separately for the aquatic path; boat behavior unchanged (D16) |
| R25 | Wider tires, tracks, skids or thin strips (longest flat-patch extent ≳25 mm) would see 2× μ under the legacy setting; rolling traction under drive torque untested | Profiles with flat contacts must use D17 flag; add drive-torque test |
| R26 | No validated drive-torque traction test exists (two rigs failed self-calibration) | Build a calibrated articulation-wheel rig before any traction claim |
| R27 | MF-BO competence is marginal (p=0.039; wrong-basin replicates) and it is the P2 comparator, so a weak MF-BO would inflate P2 | Before any P2 run: more replicates and a second MF benchmark (e.g. multi-fidelity Hartmann/Park) |
