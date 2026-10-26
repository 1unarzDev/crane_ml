# I1 — Independent verification of the truth-inventory top defects (V1–V5)

- Verifier: independent agent. Read-only on the repo apart from this file. Uncommitted user files (`CraneCampusRobotModel.cs`, `CraneCampusCalibrationProfile.cs`, and others) were read only.
- Target: `Docs/ResearchOrchestration/AGENT_HANDOFFS/I1-truth-inventory.md` §0 (V1–V5) and the matching §3a rows.
- Method: static reading of C#, YAML GUID/fileID resolution for prefabs and scenes, and the ROS-TCP-Connector `ROSGeometry` sources. Numeric reproduction is in `python -I /tmp/i1verify/check.py`, a scratch file outside the repo whose outputs are quoted below. Unity was not run.
- Fixed step: `ProjectSettings/TimeManager.asset` = 2822399/141120000 = 0.02 s. The campus overrides it to 0.01 s (`CraneIndustrialCampus.cs:84-86`).

## Verdict summary

| Item | Verdict | One-line reason |
|---|---|---|
| V1 added-mass sign + reset | **CONFIRMED** (one number corrected) | `+M_A·ν̇` with no negation (`FossenDynamics.cs:94,185-198`). Damping is negated at `:165`. The class does not implement `ICraneEpisodeResettable`. The reset impulse is −600 N, not −450 N, because surge rides on the `YdotV` channel (see V2). |
| V2 Coriolis channel / bow axis / world ω | **CONFIRMED** | `C[0..2,5]·r` goes to torque (`:173-179`). Rows 3-5 are never evaluated. FLU x = Unity local +Z, while the bow is local −X (`ROSOmniXCommand.cs:233-240`). ω is world-frame (`:110`). |
| V3 GeneralDynamics normals + v²/v² | **CONFIRMED**. Scope is wider than stated: also the Robosub Pool scene. | Normals are mesh-local (`SubmersionLib.cs:121,129,206`) and get `InverseTransformDirection` again (`GeneralDynamics.cs:100,134`). `:148` is `v²/v²`. Live on Blastoise in Roboboat Course and on Sub in Robosub Pool. |
| V4 IMU gyro sign | **CONFIRMED** (one enablement detail corrected) | `To<FLU>()` / `Flu()` use the polar `ConvertFromRUF`, not `ConvertAngularVelocityFromRUF`. Nothing downstream cancels it. OmniBoat's `Imu` is **disabled** (`m_Enabled: 0`), not enabled. |
| V5a profile ignored without `--crane-campus-model`, hash still recorded | **CONFIRMED** (only off the default path) | Both campus runners pass `--crane-campus-model` by default, so the problem appears only with `CRANE_CAMPUS_MODEL=0` or hand-built command lines. |
| V5b wheel scale on the wrong wheel | **PARTIAL** | The sign inversion in the force branch is real. The inventory says this hits "the default force branch", but both runners default to `--crane-campus-contact-wheels`, whose mapping is correct. |
| V5c lag applied twice | **CONFIRMED, and broader** | The force branch applies it twice (`:149-152` plus `:183,189`). The contact branch also reuses `ResponseSeconds` as the PI servo time constant (`:212-214`), so the lag is doubled on the **default** path too. |

---

## V1 — Added-mass sign and reset (`FossenDynamics.cs`)

**Code facts**
- `:82` computes `stateDot = (state − statePrev)/Time.deltaTime`, a backward difference of measured velocity. Inside FixedUpdate, `deltaTime` equals `fixedDeltaTime`.
- `:88,94` compute `F = … + AddedMassFactor * Fma`, where `Fma = Ma·ν̇` (`:185-198`) with no sign flip. Damping returns `(-Fd,-Td)` (`:165`) and Coriolis is not negated either.
- Fossen's model is (M_RB + M_A)ν̇ + C(ν)ν + D(ν)ν = τ, so the hydrodynamic force is **−M_A·ν̇**. The code applies **+M_A·ν̇_{k−1}**.
- Reset: `CraneEpisodeReset.Reset()` restores Rigidbody and ArticulationBody velocities and calls only `ICraneEpisodeResettable` implementers (`CraneEpisodeReset.cs:150-200`). `FossenDynamics` (`:8`) does not implement the interface, so `statePrev` keeps the pre-reset velocity. In-place reset is not a scene reload, so `Start()` is not re-run either. Also, `Start()` leaves `statePrev` at zero, so a body that spawns already moving gets a one-tick impulse.

**Numeric** (tree mass from `Blastoise.prefab` `m_Mass` sum = 23.468 kg, F = 10 N, dt = 0.02 s)

```
MA=6: code steady accel 0.5725 m/s², correct F/(M+MA)=0.3394, ratio 1.687
MA=8: code steady accel 0.6465 m/s², correct 0.3178, ratio 2.034
reset from 1.5 m/s bow speed: via XdotU (inventory) −450 N; via YdotV (actual surge channel) −600 N
```

**Correction to the inventory:** under its own V2 finding, bow-direction motion maps to `eta[1]`, which uses `YdotV = 8`. The one-tick reset impulse along the bow is therefore −600 N, not −450 N. A −450 N impulse needs 1.5 m/s of *sway*.

**Affected:** `Roboboat Course.unity`. Blastoise `base_link` FossenDynamics is `m_Enabled: 0` in the prefab (`Blastoise.prefab:1247`), and the scene override sets it to `1` (`Roboboat Course.unity:3685-3689`). It is also enabled in `OmniBoat.prefab:4287`, which no scene or prefab references. It does not appear in Robosub Pool. The reset part affects in-place-reset episodes in Roboboat Course.

## V2 — Coriolis routing, bow axis, world-frame ω

**Code facts**
- `CalculateCoriolisMatrix` (`:129-136`) fills only `C[0,5], C[1,5], C[5,0], C[5,1]`. `CalculateCoriolisForceTorque` (`:169-183`) loops `i<3` and sends `j≥3` terms to `Tc`, so `C[0,5]·r` and `C[1,5]·r` become roll and pitch *torque*, and the `C[5,·]` yaw row is never used. C is not skew-symmetric. `C[5,1]` uses `YdotV`, where `XdotU` would be expected. The result is not negated, so the would-be sway term also has the wrong sign.
- Bow: `RoboBoatRosFrame.BodyFromRos = Euler(0,−90,0)` with the doc comment "visible bow is physics-local −X" (`ROSOmniXCommand.cs:233-240`). The EditMode tests `RosSurgeAlignsWithVisibleBowInRoboboatScene` and `RosFluCommandMapsToMeasuredControllerAxes` assert the same. Fossen instead uses `transform.InverseTransformDirection(v).To<FLU>()` directly (`:109`), so `eta[0]` is Unity local +Z, which is ROS −sway, and `eta[1]` is −local x, which is ROS surge.
- ω: `-body.angularVelocity.To<FLU>()` (`:110`) has correct handedness (it equals `ConvertAngularVelocityFromRUF`) but uses the **world** frame. Torque goes back through world `AddTorque` (`:99,102`), so input and output frames agree with each other, but roll and pitch damping depend on heading. Yaw is unaffected while the hull is level.

**Numeric**

```
Fossen (a11=6, a22=8, u=1, v=0.2, r=0.5): X=+0.8 N, Y=−3.0 N, N=−0.4 N·m
code: Fc=(0,0,0), Tc(roll,pitch,yaw)=(0.8, 3.0, 0); skew-symmetric? False
ROS forward in Unity body-local = (−1,0,0) → Fossen eta = (0,1,0): surge rides on Yv=40/Yvv=35/YdotV=8
roll about body x, ψ=0: eta[3:6]=(0,1,0); ψ=90°: (1,0,0) → the same body roll is damped with Kp instead of Mq
```

The inventory's numbers reproduce exactly. **Affected:** same as V1.

## V3 — GeneralDynamics (`GeneralDynamics.cs:100,134,148`)

**Code facts**
- `submerged.data.normals[k]` is filled from `simplifiedHullMesh.normals`, which are mesh-local (`SubmersionLib.cs:86,121,129,206`). The buoyancy consumer correctly uses `TransformDirection` (`SubmersionLib.cs:245`). GeneralDynamics calls `t.InverseTransformDirection` on these normals (`:100,134`) and then combines them with world velocities. Under pure yaw ψ this produces R(−ψ)·n instead of R(ψ)·n, a 2ψ error.
- `:148`: `Cpd2 * ((viMag*viMag)/(viMag*viMag))` equals `Cpd2`. Both `Blastoise.prefab` and `Robosub Pool.unity` set `pressureDragVelocityRef: 1`.

**Numeric**

```
ψ=45°, n_local=(1,0,0): correct (0.707,0,−0.707), code (0.707,0,0.707), error 90.0°
quad coeff (Cpd2=30, vRef=1): v=0.01 → code 30 vs 3e-3; v=0.5 → 30 vs 7.5; v=2 → 30 vs 120
```

**Enablement** (the inventory only cited the prefab):
- `Blastoise.prefab:1214-1236`, base_link, `m_Enabled: 1`, `viscousResistActive: 1`, `pressureDragActive: 1`, with no scene override. It is therefore live in **Roboboat Course**.
- `Robosub Pool.unity:128-150`: added in-scene to the `Sub.prefab` instance, `m_Enabled: 1`, both terms on. It is therefore live in **Robosub Pool**, which is the first scene in `CranePerformanceBuild.cs:41-42`. The inventory does not mention this.
- Also present in `OmniBoat.prefab`, `AltWamVPrefab.prefab` and `NewWamVPrefab.prefab`. No scene uses these.

## V4 — IMU gyro sign (`CraneCampusSensorSuite.cs:27-30`, `Imu.cs:35`)

**Conversion chain checked**
- `Vector3.To<FLU>()` is `new Vector3<FLU>(v)`, which calls `ConvertFromRUF(v) = (z,−x,y)` (`ROSVector3.cs:27-29`, `CoordinateSpaces.cs:253-256,37`). The package's own angular helper is `ConvertAngularVelocityFromRUF = −ConvertFromRUF` (`CoordinateSpaces.cs:41`), and the quaternion map is `(z,−x,y,−w)` (`:39`). Unity is left-handed, so angular velocity is an axial vector and needs this extra negation to become REP-103 right-handed FLU.
- Campus: `Flu(v)=(v.z,−v.x,v.y)` is the polar map, applied to the body-local ω (`:27-28,30`). Orientation uses `body.rotation.To<FLU>()` (`:30`, correct). Wheel odometry in the same file publishes `−wheelAngular` / `−ω_y` (`:37`, correct).
- `Imu.cs:35`: `body.angularVelocity.To<FLU>()` is polar **and** world-frame.
- **Cancellation search:** no ROS-side consumer re-negates the gyro. `/campus/imu` is only recorded (`capture_ros.py:27`) and cadence-checked (`validate_run.py:25`). `imu/raw` has no consumer config in the repo. Nothing cancels the error.

**Numeric**

```
Unity +0.1 rad about +Y → published orientation ROS yaw = −0.1
campus gyro (z,−x,y) = (0,0,+0.1); wheel-odom z = −0.1; correct ConvertAngularVelocityFromRUF = (0,0,−0.1)
```

The gyro disagrees in sign with both the orientation derivative and wheel odometry, on all three axes because the map is a full negation.

**Affected:** every campus run. The suite is added unconditionally at runtime (`CraneIndustrialCampus.cs:136-138`). `Imu.cs` is disabled in Roboboat Course (stripped scene object `m_Enabled: 0`, `Roboboat Course.unity:398-405`; prefab `Blastoise.prefab:5744`) and in `OmniBoat.prefab:142` (the inventory says "enabled"; that is wrong). It is enabled only in `WamVPrefab.prefab:583`, which no scene references, so the Imu.cs defect is latent.

## V5 — Campus calibration profile

### (a) Profile ignored without `--crane-campus-model`: CONFIRMED (off the default path)
- `CraneIndustrialCampus.cs:78` always loads the profile. Plant fields (mass, torque, lag, accelerations, wheel scales) are consumed only in `CraneCampusRobotModel.Configure` (`:39-44,109`), which runs only under `--crane-campus-model` (`:133-135`). The `calibration-profile` event is written there too (`:115-116`). Only the LiDAR sub-block is applied unconditionally (`:144-152`).
- `CraneAuthoritativeReplay.RuntimeMetadataJson` (`:81-95`) hashes whatever path follows `--crane-campus-calibration-profile` and does not check that any campus or model consumed it. It would record the hash even in an aquatic scene.
- Scope: `run_campus_nav2_fixture.sh:72-75` (default `CRANE_CAMPUS_MODEL=1`) and `run_calibration.sh:20` both pass `--crane-campus-model`, so standard runs are fine. The defect bites in opt-out or manual runs.

### (b) Wheel scale on the wrong wheel: PARTIAL
- Force branch `:153-156`: `vl=(lin−ang·s/2)·leftScale`, `desiredYaw=(vr−vl)/s`. Unity +yaw is a right turn, which makes the *right* wheel slower, so `vl` is physically the right wheel. Contact branch `:161-166` maps it correctly, as its own comment at `:158-160` says.
- Numeric: s = 0.287 m, 0.2 m/s straight, leftScale = 1.02. Force branch yaw −0.01394 rad/s, contact branch +0.01394 rad/s. The inventory's numbers reproduce.
- Why only PARTIAL: the inventory says the fitted scale "transfers with the wrong sign to the **default** force branch". Both runners default to `--crane-campus-contact-wheels` (`run_calibration.sh:19`, `run_campus_nav2_fixture.sh:75`), so the default path uses the correct contact mapping. The bug applies only with `CRANE_CAMPUS_CONTACT_WHEELS=0`.

### (c) Lag applied twice: CONFIRMED, broader than stated
- Force branch: a first-order lag with `alpha=1−exp(−dt/τ)` plus a rate limit (`:149-152`), then a velocity servo `(desired−speed)/max(dt,τ)` (`:183`) and yaw `/max(dt,τ)` (`:189`). That is two cascaded τ stages. `Acceleration` is also used twice: as the reference rate limit (`:150`) and as the response clamp (`:182`).
- **Missed by the inventory:** the contact branch (the default) also passes the already-lagged reference to `DriveWheel`, which uses `tau=max(.05,ResponseSeconds)` as the PI servo time constant (`:212-214`, `raw = reflected·(2e/τ + ∫e/τ²)`). So τ is applied twice on the default path as well.
- Numeric (force branch, τ = 0.15, 0.2 m/s step, dt = 0.01): 63 % rise at **0.37 s**. Lag plus rate limit alone gives 0.22 s, the rate limit alone 0.21 s, and an ideal first-order lag 0.15 s. Part of the inventory's "single lag would be 0.15 s" gap therefore comes from the 0.6 m/s² rate limit, not from the duplicated τ. A clean single-τ comparison is 0.22 s vs 0.37 s.

## Defects in the same code regions that the inventory missed or misstated

1. **GeneralDynamics, unused suction falloff:** the suction branch (`:151`) uses `Mathf.Pow(cosThetai, fp)`, so `suctionDragFalloffPower` (`fd`) is never used. The pressure and suction coefficient sets are also swapped relative to the face orientation: the "pressure" coefficients apply to faces with `cosθ ≤ 0` (leeward). This is latent today because the prefab and scene values are equal (100/30, falloff 1/1). It becomes live once the coefficients are identified separately.
2. **GeneralDynamics, area in mesh-local units:** `Si` is computed from untransformed local vertices (`:136`). It is wrong if the submersion transform has a non-unit lossy scale. Not checked for Blastoise or Sub.
3. **FossenDynamics, first tick:** `statePrev` starts at zero (`:49`, `Start` sets only `state`), so a body spawned with velocity gets an `M_A·v0/dt` impulse.
4. **Imu.cs** publishes no `linear_acceleration` or covariances. Its orientation is the raw body transform, which for Blastoise is not rotated to the visible bow (−90° yaw offset compared with `RoboBoatRosFrame`). Latent, because the component is disabled in all scenes in use.
5. **Inventory corrections:** reset impulse −600 N, not −450 N (V1). OmniBoat `Imu` is disabled (V4). V3 also affects Robosub Pool. V5b does not hit the default path. V5c also hits the contact branch, and the 0.15 s baseline ignores the rate limit.
