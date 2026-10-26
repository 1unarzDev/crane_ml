# CRANE Iteration 1 report (2026-10-21)

Evidence IDs (E#), decisions (D#) and risks (R#) refer to `EVIDENCE_INDEX.md`, `DECISIONS.md` and `RISK_REGISTER.md`.

**Bottom line:** nothing in CRANE is physically calibrated. No result here supports a real-world sim-to-real claim. This iteration established which code is correct, which is defective, which parameters are identifiable at all, and the tooling needed to test the research question honestly.

## 1. Verified defects vs hypotheses

### Verified independently (static review, numeric reproduction, and C# reproduction where noted)

**Water and underwater dynamics**
- **V1, Fossen added-mass sign is wrong.** Surge response is 1.69× too fast and sway 2.03× too fast. Reset leaves a −600 N impulse. Reproduced in C# Unity tests (E11, E15, E34). The magnitude and dt units are correct; only the sign is wrong.
- **V2, Fossen Coriolis routing and frames are wrong.** Coriolis terms land in the wrong DOF, the bow axis is wrong, and ω is taken in the world frame (E11, E34).
- **V3, GeneralDynamics is wrong.** The normal is transformed twice, and the "quadratic" drag term is constant. It is live in Roboboat Course and in Robosub Pool (E11, E15).
- **Water viscosity units are wrong.** The value is treated with the wrong units (Re below 100). The defect is shared with the surface boat's GeneralDynamics path on `main` (E30).
- **The Sub model has several errors** (E29):
  - inertia about 3000× too large
  - CG 1.26 mm above CB
  - hull length 5.4 m on a 0.30 m vehicle
  - pitch not actuated

**Land**
- **The campus IMU gyro sign is inverted.** This affects all campus runs (E11).
- **The calibration profile has gating and lag defects** (E11).
  - V5a and V5c are confirmed.
  - V5b is partial: it is off the default path.
- **Land slope models give wrong behaviour** (E22):
  - Force branch: climbs at most 1.90° and creeps on a cross-slope.
  - Legacy diff-drive: creeps downhill.
  - Ackermann: capped at 9.39°.

**Aerial**
- **The aerial ArduPilot bridge has sign and frame errors, a wrong servo order, and the wrong spin pattern** (E27, E32).
- **The aerial mixer clamps each motor separately**, so it delivers 58% of commanded roll at 0.95 collective (E32).
- **Aerial drag depends on heading** (E32).

**PhysX**
- **Unity Rigidbody applies no gyroscopic torque.** ArticulationBody does apply it (E48).
- **The PhysX patch-friction default (off) gives about 2× μ for flat patches of 25 mm or more.** Point-like contacts are unaffected (E41, E42).

### Hypotheses or not run

- Sub thrusters may behave bang-bang (E29f).
- Driven-wheel traction under torque is untested. Two test rigs failed their self-calibration (R26).
- Whether the A1 frame defects affect the boat's SITL bridge path (R24).
- Hypotheses H1–H10 in `AGENT_HANDOFFS/I1-truth-inventory.md`.

## 2. What was built, and where

### In `/home/lunarz/crane_ml` (all new, untracked files; none of your modified files were touched except as noted)

| Path | Contents | Evidence |
|---|---|---|
| `Tools/Adversarial/` | Adversarial calibration suite. It covers thruster sign and mount errors, speed-dependent drag, identifiability failures that MUST_ABSTAIN, 24 mutants, the R17 decoupling fitter and its coast-down protocol. | E39, E43, E52, E54 |
| `Tools/Underwater/` | Python 6-DOF oracle and BlueROV2 profile | E24, E29 |
| `Tools/Aerial/` | Python multirotor oracle and profiles | E27, E32 |
| `Tools/Terrain/` | Python terrain oracle, generators and slope validator | E21, E22, E26 |
| `Tools/Profiles/` | `crane.profile/v2` schema and its validator (R1–R12), plus 8 migrated profiles | E45 |
| `Tools/Experiment/` | Experiment API: ledger, budget, splits, abstention, claims linter, 8 optimizers with competence tests, fake worker | E56 |
| `Docs/ResearchOrchestration/` | State files, `PREREGISTRATION.md` (DRAFT), `RESEARCH_MEMO.md` (revision 2) | |
| `LICENSE`, rover Apache notice | GPL-3.0 licence and the Rover Robotics attribution (D3, E23) | |

Edits to tracked files: `Docs/RoboBoatNavigationBaseline.md:159` (a correction) and an appended section in `THIRD_PARTY_NOTICES.md`.

### Local branches and worktrees (none pushed)

| Branch @ commit | Contents | Evidence |
|---|---|---|
| `test/physics-invariants@c3705bf` | C# defect tests; 15 fail on legacy by design | E34 |
| `feature/underwater-6dof@498fd57` | CraneUnderwater assembly, UW Tank scene, pressure/DVL sensors | E31, E37, E44 |
| `feature/terrain-slopes@1a4059c` | Slope-traction component, terrain course and builder | E35, E41 |
| `feature/aerial-v2@4fb6950` | ArduPilot JSON v2 bridge (legacy refusal is aerial-only, D16) and rotor model v2 | E38, E40, E50 |
| `verify/patch-friction@4d8dc40` | Verification tests | E42 |
| `verify/physx-gyro@d989979` | Verification tests | E48 |
| `/home/lunarz/crane_ros@940a9ce` | New GPL-3.0 repo with no remote, containing `crane_hardware`, `crane_nav` and `crane_testing`. Covers the D24 certified runtime decoupler, launch domain guard (200–228), provenance and identity gates. | E53, E57 |

## 3. Commands (all offline)

```
cd /home/lunarz/crane_ml
python -I -m pytest Tools/Adversarial -q -m "not slow"   # 78 pass, 1 xfail (OOF-01, documented negative)
python -I -m pytest Tools/Adversarial/decoupling -q      # 66 pass (~4 min)
python -I -m pytest Tools/Underwater Tools/Aerial Tools/Terrain Tools/Profiles -q   # 314 pass
python -I -m pytest Tools/Experiment -q                  # 106 pass, 2 skip (~4 min)
bash /home/lunarz/crane_ros/scripts/check.sh             # docker --network none; 140/75/408 pass
~/Unity/Hub/Editor/6000.5.10f1/Editor/Unity -batchmode -nographics -projectPath <worktree> -runTests -testPlatform EditMode|PlayMode -testCategory <cat>
```

## 4. Tested vs not tested

**TESTED:**
- Python fixtures in every domain
- Unity EditMode and PlayMode tests in isolated worktrees
- ROS package builds and tests in a container with networking turned off

**NOT_RUN:**
- Any built player with the new code
- Any live Nav2 run
- Any ArduPilot or PX4 SITL
- The N ≥ 10 `reference` repeats needed to freeze the D10 tolerance
- Unity timing tests (E5)
- The characterization runner against Unity (its flags exist only on branches, R13)
- All physical hardware. Hardware was never contacted.

## 5. Blockers

1. **No physical data.** P1–P6 need held-out real sessions (`RESEARCH_MEMO.md` §7).
   - Roughly 59–92 configuration × site units for P1.
   - Roughly 200 accepted recommendations for P5.
2. **Unity flags are unmerged.** `--crane-calibration-faults`, `--crane-water-current`, `--crane-roboboat-contract` and `--crane-ros-thruster-pwm` exist only on `roboboat-thruster-pwm` and `test/characterization-faults` (R13). Merging is your call, because it touches your WIP areas.
3. **Campus fixes wait on your commit.** The IMU sign fix and the profile fixes touch `CraneCampusRobotModel.cs` and `CraneIndustrialCampus.cs`, which hold your uncommitted work.
4. **Evidence gaps.**
   - Marginal MF-BO competence (R27).
   - Driven-traction rig (R26).

## 6. Next steps, in order

1. Build a player from a merge of the feature worktrees, run `reference` N ≥ 10, and freeze the D10 tolerance (pre-decoupling only).
2. Add the `hull-am-v1` flagged hydro profile with gates G1–G3 (D8), using the PhysicsInvariants tests as its gate.
3. Connect the `crane_nav2` worker binding; freeze `PREREGISTRATION.md` first, choosing Spearman or Pearson for P1.
4. Run a pilot simulation-only study: a transient-timing model vs a steady-state model vs randomized delay, under equal budgets. This is not evidence for real transfer.
5. Collect physical data using the per-domain protocol in `RESEARCH_MEMO.md` §7.
6. Underwater: harbor scene with current and a tether port. Aerial: build a player and run the loopback SITL fixture. Terrain: wire the traction adapter onto a robot and build a calibrated drive-torque rig.
