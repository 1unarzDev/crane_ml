# I1 terrain independent verification

Verifier: independent, read-only. Date: 2026-10-21. `python -I -m pytest Tools/Terrain -q` gives 29 passed.
The 29 passes test internal consistency of the Python transcriptions only. They are not evidence about Unity.

## 1. Land drive-model claims (re-derived from C#)

`CraneCampusRobotModel.cs`: the uncommitted diff touches only `Configure` (calibration-profile override, mass).
`Step` and `DriveWheel` are byte-identical in the working tree and at HEAD, apart from a +13-line offset. I diffed them.
The line numbers below are working-tree numbers. At HEAD, subtract 13.
The calibration-profile override in the working tree can change Acceleration, torque and mass when a profile is loaded. It is not the default path.

**Chassis friction (verified from the scene, not from Python).** There are no `.physicMaterial` or `.physicsMaterial` assets in `Assets`.
- The material is built in code at `Assets/Editor/CranePerformanceBuild.cs:89-93`. It has dynamic and static friction 0 and combine Minimum.
- It is serialized at `Assets/Scenes/TurtleBot3 Warehouse Validation.unity:5059-5064`, with m_FrictionCombine: 2.
- The robot BoxCollider references it at `.unity:3064` (`m_Material: {fileID: 833585328}`).
- Campus surface materials also use Minimum (`CraneIndustrialCampus.cs:196-197`, manifest `frictionCombine`). The contact friction is therefore min(0, mu) = 0.
- In the force branch the wheel spheres use `chassis.sharedMaterial` (`CraneCampusRobotModel.cs:77`).
- The casters use a clone of the chassis material (`:99`).
- Chassis, wheel and caster friction is therefore zero in the force branch, and PhysX supplies no tangential support.

**Claim 1: the legacy diff-drive holds 45 deg at mu=0.6. PARTIAL.**
- `DifferentialDriveDynamics.cs:51-67` is a VelocityChange servo. It is clamped to 10 m/s^2 per step (`:15`, `:53-56`, `:64-67`).
- The chassis friction is 0 (see above), so mu is irrelevant. Gravity along a 45 deg slope is 6.94 m/s^2, which is below 10.
- The servo can cancel it, and the mu-independence is confirmed.
- It does not hold the robot still, though. The servo zeroes the velocity before the physics step. Gravity then adds g*sin(theta)*dt, and the position integrates with that velocity.
- The robot therefore creeps downhill at about 0.14 m/s at dt=0.02 (the fixed step is 141120000/2822399 steps, which equals 0.02 s in `TimeManager.asset`). The creep scales with dt.
- The Python test (`test_terrain_reference.py:163-167`) is a tautology, `g*sin(theta) <= 10`. It does not model this residual drift. The creep is my derivation and has not been run in Unity.
- "Holds" is true only in the sense that the slide speed is bounded by g*sin(theta)*dt and does not depend on mu. The core defect (ignores mu, exceeds the Coulomb limit) stands.

**Claim 2: the campus force branch climbs at most about 1.9 deg and creeps 0.17 m/s at 10 deg cross slope. CONFIRMED.**
- Limit (`:180-182`): `limit=min(Acceleration*traction, min(mu_d*g, 2T/(r*m)))`. It does not depend on slope.
- Rough concrete (manifest): mu_d 0.7225, traction 0.95, rolling resistance 0.025.
- The terms are 0.6*0.95=0.57, 7.09 and 13.65 (2*0.35/(0.033*1.5539)), so limit=0.57.
- `:183` clamps the response to the limit. `:186` then subtracts the rolling term, which adds 0.245 m/s^2 against motion.
- Climb: 0.57-0.245=0.325 = g*sin(theta), so theta = 1.90 deg. Hold at zero command, with the rolling term assisting while sliding down: 0.815/9.81 gives 4.77 deg.
- The 5 deg loading ramp therefore cannot be held (sin 5 deg is 0.0872, so 0.855 m/s^2 is needed against 0.815 available).
- Cross slope (`:190-192`): `a=clamp(-v_lat/max(dt,0.1), +-lateralLimit)`. The steady state is v=0.1*g*sin(10 deg)=0.170 m/s.
- lateralLimit is at least 2.4 on every surface, which exceeds the 1.70 needed, so the clamp never binds.
- `ForceMode.Acceleration` is mass-independent.

**Claim 3: the contact-wheel branch has no slip curve and frictionless casters. CONFIRMED.**
- Wheels are spheres, or a convex mesh, with PhysicsMaterial mu_s 0.9, mu_d 0.8, Minimum (`:51`, `:55-56`). PhysX Coulomb friction is the only slip model. `DriveWheel` (`:198-219`) is a torque servo with no slip ratio or slip angle.
- Casters (`:96-100`) use `chassis.material`, which is mu 0 / mu 0, so they are frictionless.
- Rolling resistance is scripted at `:169-170` along `transform.forward` with no cos(theta) factor. The wheel hinge has no suspension.
- Caveat: with Minimum combine, contact mu = min(0.9 or 0.8, surface mu). The surface value usually governs.

**Claim 4: the Ackermann WheelCollider rover is capped at about 8.5 deg. PARTIAL.**
- Values: `AckermannRoverDynamics.cs:22` and `Land Vehicle Validation.unity:1021` give 6 N*m on each of 4 wheels. The wheel radius is 0.25 (`:328`) and the body mass is 60 (`:1056`).
- Force limit: F = 24/0.25 = 96 N. W = 588.6 N. sin(theta) = 0.163, so theta = 9.39 deg with no rolling term.
- 8.5 deg appears only if Crr=0.015 is added: sin(theta) + 0.015*cos(theta) = 0.163 gives 8.5 deg.
- The code applies the rolling torque only when throttle is 0 and brake is 0 (`:135-136`). Under throttle it applies none. The torque cap is therefore about 9.4 deg, and 8.5 deg is conservative.
- The coast creep threshold is confirmed. Rolling torque of 0.552 N*m on each of 4 wheels is 8.83 N, which corresponds to sin(theta)=0.015, or 0.86 deg.
- The friction curve (extremum 1.0) does not bind first. The WheelCollider mass 3 is not body mass.

## 2. Analytic references against textbook formulas

No sign or convention error found. All checked by hand.
- **Slope limit:** tan(theta) <= mu, and f*mu with a load fraction f. Correct. For a fixed f the relation is exact, not first order.
- **Tip-over:** `lateral_tip_angle` is atan(lever/h). The lever is track/2 - cg_y if left is downhill, and + cg_y if right is downhill. This matches `lateral_side_loads`, where N_left = 0 gives tan(phi) = (half + cg_y)/h with left up.
- **Longitudinal tip:** atan(x_r/h). Correct.
- **Load transfer:** N_f L = W(x_r cos(theta) - h sin(theta)) - m a_x h, and the lateral analogue. Both have the right signs. Acceleration unloads the front, and a left turn unloads the left side.
- **Skid-steer:** B_eff = B(1+(L/B)^2) is exact for viscous friction. I derived it: omega = B*du/(L^2+B^2).
  - It also holds for Coulomb friction when all 4 wheels carry equal load and the layout is symmetric. All sliding magnitudes are then equal.
  - I confirmed this numerically for several L/B ratios and for mu=0.3 and mu=1.
  - It is not general: unequal axle loads or a CG offset break it. The docstring "closed form" should say so.
  - Husky 1.851 against Clearpath 1.875 and Jackal 1.487 against 1.5 are reproduced. This is corroboration, not calibration.
- **Step climb:**
  - Rigid wheel h < r. Correct.
  - Pushed frictionless edge: h = r(1 - cos(atan(P/N))). Correct.
  - Driven wheel: h <= r(1 - 1/sqrt(1+mu^2)). Correct only for the weight-only case (no chassis push or torque coupling, a single wheel). It is a conservative bound, not a general rover result.
- **Climb and stopping:** mu*N_driven(theta) = W(sin + Crr cos) is correct. Stopping distance uses v^2/2g(f*mu*cos +- sin), also correct.
- **Slip identifiability:** i = (omega*r - v)/(omega*r) and odometry/true = s/(1-i) are correct.
- **Minor:** `lateral_slip_feasible` takes the worst-case sign, which is conservative and fine.

## 3. Rover URDF provenance

- **Upstream:** `roverrobotics/roverrobotics_ros2`, `roverrobotics_description/urdf/`. HEAD is `8f74c12a92b4237e38ba5f4bd0ce297a9cdb9955` (committed 2026-10-19).
  - The GitHub API license is Apache-2.0, and `package.xml` says `<license>Apache-2.0</license>`.
  - Authors are Rover Robotics (info@roverrobotics.com) and Jack Rivera. The maintainer is Shashank Sharma.
  - I fetched into `/tmp/crane-verify-terrain/upstream/` and executed nothing.
- **Copy check:** I diffed max, mini, miti, mega, flipper and rover_4wd. Their only differences are the `$(find roverrobotics_description)` and `file://` path rewrites.
- **Not at upstream HEAD:** local also has max_65, max_100(_neo) and miti_65, which are not in upstream HEAD. They are likely older revisions. Local has 28 meshes against 27 upstream. The commit they came from is unknown.
- **Repo history:** the files arrived in `0b3c099` and `07231b9`.
- **Upstream notices:** upstream `LICENSE` is the stock Apache-2.0 text with the "[yyyy] [name of copyright owner]" template unfilled.
  - Upstream has **no NOTICE file** (404). The repo root also has none.
  - The only copyright attribution is the `package.xml` author line.
- **Shipped locally:** no LICENSE or NOTICE next to `Assets/Robot Descriptions/rover/`. The only LICENSE files in `Assets` are under `Assets/Resources/CampusRobot` and `CampusProps`. The URDF headers carry no copyright.
- **`THIRD_PARTY_NOTICES.md`:** no mention of Rover or Apache, in both the working tree and HEAD. The working-tree diff adds only the LILocBench section. The compliance gap is real and unresolved.
- **Drafted notice text (Apache-2.0 section 4):**

```
## Rover Robotics robot descriptions

Assets/Robot Descriptions/rover/ (URDF files and COLLADA meshes) is copied from
roverrobotics_description in https://github.com/roverrobotics/roverrobotics_ros2
(Apache-2.0; examined at commit 8f74c12a92b4237e38ba5f4bd0ce297a9cdb9955).
Copyright Rover Robotics (info@roverrobotics.com) and Jack Rivera, as stated in the
upstream package.xml.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use these
files except in compliance with the License. You may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, the files are distributed
on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

Modifications: xacro include and mesh paths were rewritten to relative paths.
Some URDF variants (max_65, max_100*, miti_65) are not present in the commit named above.
Upstream ships no NOTICE file, so there is nothing further to reproduce.
```

Also ship the Apache-2.0 text as `Assets/Robot Descriptions/rover/LICENSE`. Also mark which variants came from which upstream revision. Owner decision: add the notice or delete the unused rover URDFs.

## 4. License spot-checks at primary sources (GitHub API and raw files)

- Husky `husky/husky`: BSD-3-Clause. CONFIRMED.
- Jackal `jackal/jackal`: LICENSE reads "Copyright 2021 Clearpath Robotics Inc. Redistribution and use in source and binary forms...", which is BSD-style. Confirmed as BSD. The `clearpathrobotics/jackal` slug 404s, so I used `jackal/jackal`.
- Scout `agilexrobotics/scout_ros2`: Apache-2.0. CONFIRMED.
- Leo `LeoRover/leo_common-ros2`: MIT. CONFIRMED.
- F1TENTH `f1tenth/f1tenth_gym`: MIT. CONFIRMED.
- Chrono `projectchrono/chrono`: BSD-3-Clause. CONFIRMED.
- RELLIS-3D: GitHub API reports no SPDX license. README line 259-260 says "Creative Commons Attribution-NonCommercial-ShareAlike 3.0". CONFIRMED.
