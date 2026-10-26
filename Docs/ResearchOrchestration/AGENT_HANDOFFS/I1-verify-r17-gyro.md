# I1 verify: R17 fitter and PhysX gyroscopic torque (independent verifier)

Date: 2026-10-21. I did not write either claim.

Evidence labels:
- Claim 1: **FIXTURE_VALIDATED** (adv_plant fixture only).
- Claim 2: **UNITY_EDITMODE_EXECUTED** (Unity 6000.5.10f1 batchmode, preview PhysicsScene).

Rules followed:
- The main repo was read only. I ran the R17 code from copies in `/tmp/crane-verify-r17/pkg/`. The council suite passes on those copies (31 passed, 117 s).
- My scripts and outputs are in `/tmp/crane-verify-r17/` (`case_*.py`, `algebra_check.py`, `out/*.txt`).
- The Unity test is in worktree `/home/lunarz/worktrees/crane-gyro-verify`, branch `verify/physx-gyro`, local commit d989979, not pushed. The file is `Assets/Tests/Editor/PhysXGyroVerifyTests.cs`, Category `PhysXGyroVerify`.
- Logs are `/tmp/crane-verify-gyro/unity_pgs.log` (PGS solver, the project default) and `unity_tgs.log` (TGS solver; `m_SolverType` was flipped temporarily and then reverted).

## Claim 1: R17 root cause and the new fitter

### 1.1 Root cause algebra: CONFIRMED, with one caveat

I checked it independently with `algebra_check.py`. It is a static model, v = D^-1 B K_sigma phi(M e) with linear drag, and uses none of the council's code.
- Surge rows with a reverse ratio that is the same on every thruster: kappa is **exactly n**. Measured 1.0000, 1.5000, 2.0000 and 3.0000 for n = 1, 1.5, 2, 3, on both signs, with a perturbed mount geometry.
- Caveat: if the reverse ratio differs per thruster (0.70 / 0.78 / 0.85 / 0.78), surge kappa is no longer n (0.40 / 0.72 at n = 2).
- What does hold for every row and every rho is **kappa(n) = n x kappa(n=1)**. The secant-versus-incremental factor is an exact n-times overcorrection relative to the linear-thrust fit. "kappa = n on surge rows" is the special case of uniform rho.
- The council's fixture `r17_proof.py` reproduces as stated (quad_lin surge 2.04 / 2.06).

### 1.2 Break attempts (quad_quad unless stated; "ratio" = new / identity cross score)

| case | result | verdict |
|---|---|---|
| (a) mixed commands (`case_a.txt`) | Yaw leak on the uncommanded axis falls 2.5-3x versus identity (quad_quad (0.3,-0.2,0): 0.030 -> 0.012 rad/s). That is still not zero: the sign-split C is not exact for mixed commands, as the council said. On **lin_lin_asym**, new makes commanded-axis superposition *worse*: surge 1.32 m/s against a single-axis prediction of 1.44 (-8%). Identity gives +0.7% and upstream 0%. In force space the uncommanded-axis error is identity 0.11-0.13 -> new 0.07. | Partial break, not covered by tests |
| (b) deadband / curve shape (`case_b.txt`) | db_edge 0.05 -> scheduled (correct). Command deadband 0.08 -> homogeneous, fine. Split exponent 2/1.5 -> scheduled; out of envelope it is 3.8-17x **worse** than identity, so callers must honour `in_envelope`, because `decoupler()` clamps and does not refuse. **Deadband 0.12** (larger than the 0.1 fit amplitude): it does not abstain, though the base holds are dead. It gives scheduled mode, only 0.5x at 0.15, and a false `non_actuator_coupling` flag. **RR-only deadband 0.08**: homogeneous; at 0.05 two rows are 1.17-1.19x worse (abs 0.013). | Break: no "dead operating point" gate; actuator deadband mislabelled |
| (c) asymmetric lever arms (`case_c.txt`) | When the design knows the geometry: no flags, rows 0.02-0.06x identity. When the geometry is wrong in the design ("strong"): **all 8 actuator flags fire** (strength 0.80-1.16, mount yaw +-7-9 deg) with an unexplained residual of 0.0025, so it is a confident misattribution. "Mild": RR strength reads 0.898 against a true 0.95, with no flag. | Decoupling robust; fault report misattributes geometry error |
| (d) noise near threshold (`case_d*.txt`, 10-20 seeds per level) | The gate is monotone and sharp, with a seed-dependent band: 0.022-0.025 gives 20/20 OK; 0.027 4/20 abstain; 0.028 10/20; 0.029 18/20; >= 0.03 all abstain. Flipping exists only within about +-10% of the threshold, and that is acceptable. **But OK fits just below the threshold are often worse than identity on low-leak rows.** That happened in 19/100 fine-sweep fits, and also at 0.01 noise (seed 7: surge-0.4 0.023 -> 0.024). At 0.02-0.025 the worst is 2.76x (surge-0.4 0.023 -> 0.063), with no flag. Mode also flips: homogeneous -> scheduled on 1/10 seeds at 0.01. | Break: precision gate does not imply "no worse than identity"; the per-row check that section 6.5 recommends is not in the fitter |
| (e) held-out speeds (`case_e.txt`, case_b knee) | For homogeneous plants extrapolation is exact from 0.03 to 1.0 (quad_quad at most 0.064x; quad_lin and lin_lin about 0). **Knee curve** (s\|s\| up to 0.3, then linear): the homogeneity test passes inside [0.1, 0.2], so it reports `valid_range (0, inf)`. Then 0.6 and 0.9 give only 0.33-0.50x, above the claimed 0.15x. | Break: homogeneity is tested locally but certified globally |
| (f) thruster fault vs hull coupling (`case_f.txt`) | FR mount yaw 8 / 15 deg -> `FR_mount_yaw` (6.0 / 11.5 deg). weak_rr -> `RR_strength` 0.598. **Hull couplings:** see the two rows below. | **Cannot tell fault from coupling, and does not say so** |
| (f) hull couplings, no flag | Nvv = +-8, Nuu = 3, Yuu = 4 -> **no flag at all**, though the column deviation (0.043-0.052) is *larger* than that of the flagged 8 deg mount fault (0.021). The actuator fit absorbs it just under tolerance (mount +-4.9 deg, strength 0.92 / 1.09). | |
| (f) hull couplings, misattributed | Linear Nv = 4 -> all 8 thruster flags (mount +-17.7 deg). Combined -> `FL_mount_yaw`, `FR_strength`. Council hull_drag -> `FL_strength`, `FR_mount_yaw`, `RR_strength`, `RL_mount_yaw`. | |

There is a discriminator the fitter ignores: actuator faults kept the null columns homogeneous, while every velocity-dependent hull coupling except drag_offset made them non-homogeneous (mode scheduled). drag_offset scales like the main drag, so it aliases exactly from steady holds; only a coast-down or transient test can separate it.

### 1.3 "Faults detected from the identified plant, not the residual": CONFIRMED

- `fault_report` uses raw-effort null columns against the design actuator, and never uses decoupled residuals.
- A residual-only gate does miss the faults. Max translation cross after decoupling at 0.4:

| fault | before | after | D10 0.087 |
|---|---|---|---|
| weak_rr | 0.353 | 0.0077 | passes |
| angled_fr 15 deg | 0.260 | 0.0063 | passes |

- Caveat 1: a main-axis gain gate would catch weak_rr (-7.5% to -9% speed on every axis versus the no-fault plant). It would catch angled_fr only weakly (-4% surge).
- Caveat 2: a gate on the *pre-decoupling* (identity) residual catches both.

## Claim 2: Rigidbody applies no gyroscopic torque. CONFIRMED

Setup for every variant:
- A torque-free body, mass 1, I = diag(1, 2, 3), set explicitly.
- Initial spin w0 = (0.02, 2.0, 0.02) rad/s (near the intermediate axis).
- Damping 0, sleep off, 20 s.

Reference: RK4 integration of Euler's equations.

| variant (dt = 0.005 s; 0.02 s similar) | flip? | first flip | min body w_y | max \|dL\|/\|L0\| world | max \|dE\|/E0 |
|---|---|---|---|---|---|
| reference (Euler equations) | yes | 5.46 s | -2.000 | 0.0000 | 0.0000 |
| **Rigidbody** (PGS and TGS; explicit tensor or collider-derived tensor) | **no** | none | +2.000 | **0.0141** | 0.0000 |
| Rigidbody + manual tau = -w x (I_w w) via AddTorque | yes | 5.475 s | -2.012 | 0.0117 (dt 0.02: 0.0485) | 0.0235 (dt 0.02: 0.099) |
| **ArticulationBody root** (free) | **yes** | 5.475 s | -2.012 | 0.0117 (dt 0.02: 0.0485) | 0.0235 (dt 0.02: 0.099) |
| control: Rigidbody near the major axis | no (correct) | none | n/a | 0.0149 | 0 |

Interpretation of the rows:
- Rigidbody keeps world-frame w constant: a steady spin, and kinetic energy is conserved.
- Rigidbody's world L is **not** conserved: it drifts 1.4%, which equals the precession of the off-axis 1% w component. That is the signature of a missing w x Iw term.
- ArticulationBody does include the gyroscopic term. It integrates it explicitly: its traces match the Rigidbody + manual-torque variant to 4 digits at t <= 10 s, then diverge slightly. It also gains energy (+2.4% at dt 0.005, +10% at dt 0.02).

**Settings check.**
- None of these exposes a gyroscopic option: `UnityEngine.PhysicsModule.dll` / `.xml`, `Rigidbody`, `ArticulationBody`, `Physics` and `PhysicsScene`. The only "gyro" strings in the module are `Input.gyro`.
- The ProjectSettings `DynamicsManager` has no such field.
- The solver type does not change the result: TGS is identical to PGS.
- The PhysX flag eENABLE_GYROSCOPIC_FORCES is not reachable from C#.

## Implications

- **R17.** The diagnosis is right, and the fitter is a real improvement in the regimes it was designed for. Do not adopt it as-is.
  - Add a per-row "no worse than identity" confirmation gate, and a minimum-excitation gate.
  - Certify homogeneity only over the tested range, and make `decoupler` refuse out-of-envelope commands instead of clamping them.
  - Make the fault report say "actuator or hull, ambiguous" when it cannot tell, use non-homogeneity as evidence of a hull coupling, and add coast-down tests to separate the two.
  - Recheck section 4's "fault detected / mislabelled" wording: some hull couplings go **undetected**.
- **Gyro.** Any CRANE body using Rigidbody with an anisotropic inertia (boat yaw-pitch-roll, the wheel and rotor spin of the aerial/rover models) lacks gyroscopic coupling: precession, Dzhanibekov, and roll-yaw coupling at high rates. The fixes are:
  - add `-w x (I_w w)` in FixedUpdate. Explicit, it needs a small dt or a semi-implicit form; it gains energy otherwise.
  - or use an ArticulationBody root, which is also explicit.
