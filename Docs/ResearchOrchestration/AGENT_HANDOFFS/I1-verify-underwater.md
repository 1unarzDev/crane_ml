# I1 independent verification: underwater

Verifier read-only. Scratch: /tmp/crane-verify-uw/. Paths are under /home/lunarz/crane_ml/Assets unless noted.

## 1. Sub / Robosub Pool claims

| Claim | Ruling | Evidence |
|---|---|---|
| (a) inertia 356/399/250 on 11.5 kg, ~3000x too large | CONFIRMED | `Robot Descriptions/sub/Sub.prefab:9729` `m_InertiaTensor {356.19846, 399.43127, 249.684}`, `m_ImplicitTensor 0` (:9741). Scene mass override 11.456 (`Scenes/Robosub Pool.unity` base_link target 3609751419727271082 `m_Mass`). Collider box 0.30081 x 0.21589 x 0.20572 (`urdf/sub.urdf:~86`). Box inertia m/12(...) = 0.0849 / 0.1268 / 0.1309. Ratios 2809 / 3052 / 2941. Thruster links add about +0.03 to 0.09, which does not change the order of magnitude. |
| (b) CG 1.25 mm above CB | CONFIRMED (value); consequence PARTIAL | Base COM is 0 (`Sub.prefab:9728`, implicit COM off). Vertical thrusters (0.25 kg x2) sit at +0.03229 (`sub.urdf:13,25`). Horizontal ones sit at z=0. z_g = 0.5*0.03229/12.96 = 1.26 mm (1.258 with props). CB is the centroid of a symmetric box mesh, with bounds +/-0.108/0.103/0.150 in `sub.obj`, so CB = origin (prism centroid at `SubmersionLib.cs:241-263`, `Buoyancy.cs:41-44`). BG = -1.26 mm. Caveats. (1) Unstable only when fully submerged. With +3.5 N net buoyancy it rises and floats, and the waterplane BM is about 19 mm for roll, so it is stable at the surface. (2) The "e-fold 1.4 /s" and the "0.7 s" falsifier ignore PhysX angular damping 10 (`Sub.prefab:9726`). With damping 10 and a real I, growth is about 0.2 /s (about 5 s) in roll, not 0.7 s. Thruster inertia also lowers sqrt(W*zg/I) to about 1.0 to 1.2 /s. The falsifier expectation should be corrected. |
| (c) waterViscosity as kinematic, Cf rises with speed | CONFIRMED | `Utils/Constants.cs:5` = 1.0016f (the value of dynamic mPa.s at 20 C). `SubmersionLib.cs:333-334` Re = vL/1.0016. Real nu is about 1.0e-6 m2/s, so Re is low by 1.0e6 / 1.0016. `:324-325` Cf = 0.075/(log10(Re)-2)^2 is the ITTC-57 form. With Re < 100 (always, until vL = 100) the denominator shrinks as v grows, so Cf rises and goes singular at Re=100. Example: v=0.5, L=5.4 gives Re=2.7. It is only used there (grep). |
| (d) hull length 5.4 m on 0.30 m vehicle | CONFIRMED | `Scenes/Robosub Pool.unity:149-150` hullZMin -2.5 / hullZMax 2.9 on the GeneralDynamics added to base_link (:128, target 6291754185679410842), L = |max-min| = 5.4 (`SubmersionLib.cs:322`). Mesh z extent is +/-0.15. |
| (e) net buoyancy +3.5 N; pitch unactuated | CONFIRMED | Constants rho=997, g=9.80665. V = 0.013360 m3 gives B = 130.62 N. Mass 11.456 + 6x0.25 + props = 12.96 kg gives W = 127.09 N. Net = +3.53 N. Pitch: horizontal thrusters (rpy 1.5708,0,+-0.785) lie in the z=0 plane with a horizontal axis, so they give no pitch moment. Verticals are at x=0, so they give none either. Rank is at most 5. `FullOmniController.cs:88-134` has only x,y,z,roll,yaw. |
| (f) thrusters bang-bang | SUPPORTED by code, not run (hypothesis stays open) | `SubmarineThrusterConfig.asset`: controlMode 0 (Torque), torqueK 1, maxTorque 1.3, thrustK -0.001, maxAngularVelocity 200. The Thruster sits on the propeller link (front_left_propeller_link). The prop is a revolute joint with `m_AngularDamping 0`, `m_JointFriction 0` (`Sub.prefab:~880`) and I about 0.001. The scene sets m_Twist=2 (free) on all 6 props. `MotorBase.ApplyTorqueControl` applies constant torque and there is no load model (`Thruster.cs` thrust = sign(w) w^2 k). So w ramps at tau/I (1300 rad/s^2 at full command) until the clamp. Commanded magnitude sets only the time to saturate, and w persists after command removal. Thrust at w=200 is 0.001*40000 = 40 N (reverse x0.8). The clamp relies on `body.maxAngularVelocity` for an ArticulationBody, which is untested. |

## 2. BlueROV2 Heavy profile vs von Benzon 2022
Fetched the AAU PDF to /tmp/crane-verify-uw/src/paper.pdf and read it as text only. Checked 15 or more values and all match: m=13.5, nabla=0.0134, Ix/Iy/Iz = 0.26/0.23/0.37 (CAD), r_b=(0,0,-0.01), r_g=0, g=9.82 (Table A1); X_u 13.7 / X_u|u| 141 / Y_v|v| 217 / Z_w 33 / Z_w|w| 190 / K_p|p| 1.19 / M_q 0.8 / M_q|q| 0.47 / N_r|r| 1.5; added mass 6.36 / 7.12 / 18.68 / 0.189 / 0.135 / 0.222 with origin [33] (Eidsvik); tether 0.043 / 0.0075 / 1.2 / 0.01 / 100 / 6.437e10; Table 4 (0.72 / 0.63 / 0.71 m/s; 85 / 85 / 120 N); Table 5 (0.46 x 0.38 x 0.58); eq. 18 coefficients (F(1) = 30.4 N, checked arithmetically); eq. 19 TF (poles -13.16 and -37.9 +/- 82.6j, zero -17.7, as quoted). The beta_i = {0, pi/2, 3pi/4, pi} conflict is real in the paper. The damping "Origin = Experimental" column is partly lost in the PDF text layout (Z_w|w| confirmed), which is minor.

## 3. Licences (GitHub API / raw LICENSE, 2026-10-21)
- Stonefish (patrykcieslak/stonefish): GPL-3.0, CONFIRMED.
- UUV Simulator: LICENSE text is Apache 2.0, package.xml says Apache-2.0 (the API reports NOASSERTION only because of 3rd-party text). CONFIRMED.
- CentraleNantesROV/bluerov2: Apache-2.0, CONFIRMED. rexrov2 also Apache-2.0.

## 4. Oracle identities
49/49 tests pass in a /tmp copy. Independent check (/tmp/crane-verify-uw/ident.py, random M_A, offset CG):
- M = M_RB + M_A. M is symmetric, min eigenvalue 0.88, positive definite.
- C is exactly skew (error 0). nu'C nu is about 4e-14, for both M_RB and M_A.
- D: nu'D(nu)nu > 0 in all samples (min 3.4). Note that the linear D alone has zero entries (Y_v=0) and is only PSD, so D is positive definite only through the quadratic terms. The validator enforces D+D' >= 0, not > 0. Minor.
- Restoring for W=B: g_K = +W*BG*sin(phi) to 1e-16 (0.2, 0.7, 1.4 rad), with zero force. The applied righting moment -g is therefore -W*BG*sin(phi). CONFIRMED.

Mutants (/tmp copies):
1. Buoyancy moment arm uses r_g instead of r_b: 11 failures.
2. C21 sign flipped (breaks skew): 4 failures (Munk, both energy tests).
3. Added mass subtracted in M_inv (legacy sign): 10 failures.
All killed.
