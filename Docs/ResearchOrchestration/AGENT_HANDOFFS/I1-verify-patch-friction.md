# I1 independent verification: PhysX improvedPatchFriction

Worktree `/home/lunarz/worktrees/crane-friction-verify`, branch `verify/patch-friction` (from 1a4059c), one local commit, not pushed.
Tests: `Assets/Tests/Editor/Terrain/PatchFrictionVerifyTests.cs`, Category `PatchFrictionVerify`.
Logs: `/tmp/pfv/unity2.log` (main run, 4/4 passed), `/tmp/pfv/u3.log` (drive trace). Unity 6000.5.10f1 batchmode, preview PhysicsScenes.

## 1. Review of the council tests
- **Ramp onset is sound.** Bisection over 11 iterations gives about 0.04 deg resolution. The box settles for 3 s and "slid" means more than 5 cm travel. That is a bit coarse but safe, because above onset the box accelerates away. Rigidbody sleep cannot hide motion: a box that sleeps on a ramp below onset is correct, and above onset it never sleeps.
- **Locked-wheel test.** The wheel body is rotation-frozen (a locked wheel), pushed along its axle with constant force. Sliding means more than 5 mm of drift in 2 s. Each AddForce wakes the body. This is a valid lateral-friction measurement but not rolling. The "1.000x" for sphere, capsule and tire is real (see 2), but it is true by construction: the contact is a single point.
- **Not tested by the council:** rolling and drive torque.
- **Caveats.**
  - The wheel tests used contactOffset 0.001, while default is 0.01.
  - Only the Average combine mode was used.
  - The 1.000x is a ratio of bisection results, with a resolution of about 0.2%.

## 2. My independent method
A slowly increasing horizontal force on flat ground (0.08*expected*mg per second, a 1.5 mm displacement criterion, sleepThreshold=0, rotation frozen, mass 1.5 kg, default contactOffset). The quantity is F/(m*g) at breakaway, compared with expected = mu (Average) or mu^2 (Multiply). The baseline bias is +0.5..4% from the displacement criterion.

| Case | Box 0.5 m, flag off | Box 0.5 m, flag on | Box 0.01 m, sphere, capsule, mesh tire (both flags) |
|---|---|---|---|
| Average mu 0.3 | 2.02x | 1.03x | 1.02x |
| Average mu 0.6 | 2.01x | 1.02x | 1.00x |
| Multiply mu 0.3 (expected 0.09) | 2.04x | 1.04x | 1.04x |
| Multiply mu 0.6 (expected 0.36) | 2.02x | 1.02x | 1.02x |

The inflation is the same under Multiply, so it scales the friction limit, not mu itself.

**Footprint threshold.** Flag off, square box:
- 17.5 mm side: 1.02x.
- 18.0 mm side: 2.02x.
- Everything above 18 mm: 2.02x, up to 60 mm.
- Flag on: 1x throughout.

**Strips, flag off.** Strips 4, 10, 18 and 25 mm thick by 500 mm long give 2.02x whether pushed across the thin dimension or along the long one. So the trigger is **not** the minimum dimension. It is the longer extent or diagonal: a patch larger than about 18 mm side or 25 mm diagonal in some direction.

## 3. Verdicts
- (a) **CONFIRMED**: flat patches read 2.0x with the flag off (2.01..2.04 measured, 1.02..1.04 with the flag on).
- (b) **CONFIRMED** for sphere, capsule, convex mesh tire and a 10 mm box: 1.00..1.04x in both settings and both combines. The caveat is that a tire is a point contact only while a single facet is down. A flat wide tread would be 2x.
- (c) **CONFIRMED, with a refinement.** The step is between 17.5 and 18 mm square side (diagonal about 25 mm). Strips show it is the longest patch extent that matters, not the smallest, so the threshold is not a width. A 500 mm by 4 mm skid is inflated.
- (d) **CONFIRMED.** It is live and global. Mid-simulation, a 0.5 m box at mu 0.3 flipped off to on at F=0.45 mg slid at 0.451 mg (immediately). Flipped on to off at 0.2 mg, it broke away at 0.606 mg, the same as the off baseline (0.607). Fresh: off 0.607, on 0.308.
- (e) **NOT TESTED / not reproduced here.** I did not test the articulation rolling rig for this. The force-ramp breakaway of the locked convex tire is nominal (1.00x at mu 0.6, 1.02x at mu 0.3). Any ~5% shortfall is therefore a property of the articulation rig or load transfer, not of the contact, which agrees with the council's "rig-sensitive" description. I could not verify it either way.

## 4. Rolling traction under drive torque: INCONCLUSIVE
An articulation chassis with four revolute wheels, torque applied via `jointForce`, ramped torque:
- Calibration failed. At mu=3, a/g reached only 0.4 where 4T/(R m g) predicts about 3.4. Even at mu 0.6 the sphere vehicle reached about 3 m/s and then drove off the 40 m ground (`/tmp/pfv/u3.log`, y fell to -34).
- The rig is not valid, so no traction number is claimed. The test is committed as `[Ignore]`.
- **One suggestive observation, not evidence.** In the flawed rig, the flat BoxWheel peak differed with the flag at mu 0.6 (peak a/g 0.66 off vs 0.38 on). Sphere, capsule and mesh tire were identical to 4 digits between flags.
- Needed next: a validated rig (finite ground and a known torque-to-force calibration, with drive via xDrive and a forceLimit check). Rolling traction under torque remains NOT_RUN.

## 5. Anything new
- The inflation depends on the patch extent in either direction, so long thin skids are exposed as well.
- The council's 1x claim for the wheel contacts holds with explicit Multiply and default contactOffset.
