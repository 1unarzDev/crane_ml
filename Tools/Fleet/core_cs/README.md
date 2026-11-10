# Crane.Fleet.Docking: engine-independent docking connector core (C#)

Status: ASSUMED-PARAMETER MODEL. No rover or connector exists; every number comes from `Tools/Fleet/spec/fleet_profile_v0.json`
(all `provisional`) or from the single `DockingAssumptions` class. Not committed.

- Library `Crane.Fleet.Docking/` : `netstandard2.1`, C# 9, assembly name `Crane.Fleet.Core`, namespaces `Crane.Fleet.*`. No engine references.
- Tests `Crane.Fleet.Docking.Tests/` : xUnit on `net10.0` (installed runtime 10.0.14, SDK 10.0.403). Run: `TMPDIR=~/.cache/crane-fleet/tmp nice dotnet test Crane.Fleet.Docking.Tests`.
- Parity: `parity/gen_parity_vectors.py` (imports `docking_mc/model.py`), `parity/parity_vectors.json` (484 vectors), `parity/parity_differences.txt` (written by the test when `CRANE_PARITY_REPORT=<path>` is set).

## API sketch

```csharp
var profile  = ProfileLoader.LoadOrThrow(File.ReadAllText("fleet_profile_v0.json")); // DockingProfile; every Param has .Value and .Provenance
var assume   = new DockingAssumptions();                       // the ONLY place for non-profile constants; .Registry lists the measurement that replaces each
var model    = new ContactModel(profile, assume);              // guide, magnets, hook seating, load sharing (pure functions)
var machine  = new ConnectorMachine(model, seed: 42, id: "r0.front-r1.rear");
StepResult r = machine.Step(0.05, new ConnectorInputs {
    Pose = new FacePose(lateral, gap, yaw, height, pitch), ClosingSpeed = v,
    Command = DockingCommand.Approach /*None, Approach, EngageHooks, Verify, TestLoad, Release*/,
    HookA = HookDrive.Normal, HookB = HookDrive.Normal, Obstruction = false,
    ReleaseAuthorized = graph.CanRelease(id).Allowed, AppliedLoad = new LoadCase(tension, shear, moment) });
// r.State, r.Events (typed, timestamped DockingEvent), r.Action (BackOffAndReapproach, Abandon, ...), r.SensedA/B
DockingOutcome o = OutcomeClassifier.Classify(machine.Summary);
var graph = ModuleGraph.Branched(profile, assume);            // also Line(n), SideBySide(); AddPayload(...)
ReleaseDecision d = graph.CanRelease("c0_2");                  // {Allowed, Reason}
graph.Release("c0_1", force: true);                            // forced violation => UnsafeRelease event
graph.Poll();                                                  // UnsafeRelease if a load-bearing coupling lost verification
```

Conventions: lengths m, angles rad, forces N, time s; x forward, y left, yaw CCW. Hook A at +spacing/2 (left), hook B at -spacing/2.
Commands are edge-triggered (re-issue via `None` to repeat). Randomness is `DeterministicRng` (SplitMix64), identical on every runtime.
Pose input is the face-pair pose at the moment the gap first reaches the magnet pull-in range; the guide jam test is made at gap <= 0.

## State machine (rows of `ConnectorMachine.Table`; first row whose guard passes wins; no row = `IllegalTransition` event)

| From | Trigger [guard] | To | Notes |
|---|---|---|---|
| FREE, RELEASED | CmdApproach [retries left] | APPROACH | |
| OBSTRUCTED | CmdApproach [obstruction cleared, retries left] / CmdRelease | APPROACH / FREE | |
| APPROACH | Captured | MAGNET_CAPTURED | inside envelope, closing speed ok, no obstruction |
| APPROACH | Jam | FREE | failed attempt, action BackOffAndReapproach |
| APPROACH | ObstructionDetected (flag > debounce) | OBSTRUCTED | magnets never pull in while flagged |
| MAGNET_CAPTURED | HookASensed / HookBSensed | HOOK_A_ENGAGED / HOOK_B_ENGAGED | independent sensors |
| MAGNET_CAPTURED | HookTimeout (no hook) | RELEASE_REQUESTED | failed attempt, BackOffAndReapproach |
| HOOK_A/B_ENGAGED | other hook sensed | same state | both flags set; Verify needs both |
| HOOK_A/B_ENGAGED | HookSignalLost | MAGNET_CAPTURED / other hook state / same | by remaining flags |
| HOOK_A/B_ENGAGED | HookTimeout [not both sensed] | PARTIAL | failed attempt |
| PARTIAL | late hook [both sensed] / CmdRelease | HOOK_x_ENGAGED / RELEASE_REQUESTED | |
| HOOK_A/B_ENGAGED, VERIFICATION_LOST | VerifyPassed [both sensed, per-hook measured residual <= tol] | LATCHED_VERIFIED | |
| HOOK_A/B_ENGAGED, VERIFICATION_LOST | FalseEngagementDetected | FALSE_ENGAGEMENT | sensors say engaged, geometry says no |
| LATCHED_VERIFIED, LOAD_VALIDATED | LoadTestPassed | LOAD_VALIDATED | stiffness fraction >= 0.75 and no overload |
| LATCHED_VERIFIED, LOAD_VALIDATED | LoadTestSlip | FALSE_ENGAGEMENT | catches a hook that closed on nothing with good geometry |
| LATCHED_VERIFIED, LOAD_VALIDATED | VerificationLostDetected | VERIFICATION_LOST | hook signal drop, residual > 1.5 x tol, hook overload |
| LATCHED_VERIFIED, LOAD_VALIDATED, VERIFICATION_LOST | CmdRelease [ReleaseAuthorized] | RELEASE_REQUESTED | else GuardRejected + ReleaseRefused |
| MAGNET_CAPTURED, HOOK_x, PARTIAL, FALSE_ENGAGEMENT | CmdRelease | RELEASE_REQUESTED | unverified: no graph authorisation needed |
| RELEASE_REQUESTED | ReleaseComplete (both hooks open) / ReleaseTimeout | RELEASED / STUCK_HOOK | |
| STUCK_HOOK | CmdRelease [release retries left] | RELEASE_REQUESTED | |
| APPROACH | CmdRelease | FREE | abort |

Failed attempts (Jam, hook timeout, Partial, FalseEngagement, load slip) are counted; after `MaxRetries` a `RetriesExhausted` event and action `Abandon` are issued and `CmdApproach` is refused.

## Model (all in `ContactModel`; UNCALIBRATED)

- Capture envelope: lateral `hypot(y,h) <= g + r - ell*|sin(yaw)|` and `hypot(yaw,pitch) <= a` (same form as `model.py`). Outside: jam.
- Funnel: inside the envelope, |error| relaxes linearly to the dead-band over `FunnelDepth`; outside it does nothing.
- Settling: each component is clipped to a dead-band drawn at capture (lateral sd = profile `connector_compliance_lateral`). Residual = what the hooks see.
- Magnets: `F = hold * exp(-gap/lambda)` with `F(range) = 5 % of hold`, saturating at hold; lateral restoring `F_n * y / 20 mm` capped at `F_n`.
- Hook seating, per hook: lateral at hook = `y + side * (spacing/2) * sin(yaw)`; seats iff `hypot(lat_i, h) <= tol_pos` and `|yaw| <= tol_yaw`. Yaw therefore loads one hook and relieves the other: this is the physical route to one-hook engagement. Out of tolerance but within 3x: swings shut on nothing with probability 0.5 (sensor reads engaged), else jams short. Within tolerance: 2 % catch miss. Jammed-short hooks can still read engaged via a 2 % sensor false positive.
- Load sharing: statically determinate (tension `T/2 +- M/spacing`, shear `V/2`, compression borne by the face; unseated side = magnets only, tension cap hold/2, shear cap 0.3 x hold) and compliant (hook stiffness = rated load / compliance, slack = residual at each hook; the better-seated hook takes everything until `k * slack difference`, then `P/2 +- k*dslack/2`). Any hook above rated load slips (event `HookOverload`, signal drops, VERIFICATION_LOST).
- Module graph: support is verified for a payload iff its supporting decks are joined by LATCHED_VERIFIED/LOAD_VALIDATED couplings on the remaining graph, its COM is inside the convex hull of deck contact points (footprint corners inset 3 cm) by 2 cm, and every remaining coupling's estimated load (weight-share imbalance as shear) is within rated. Unrelated releases are refused too if the current graph already violates this (conservative reading of the invariant).

## Assumptions and what replaces them

`DockingAssumptions.Registry` lists these in code (name, default, measurement). Summary:

| Constant | Default | Replaced by |
|---|---|---|
| GuideFaceHalfLength (ell) | 0.03 m (py) | CAD guide face length |
| FunnelDepth | 0.02 m | CAD taper depth |
| StictionBandScale, HeightBandSd, YawBandSd | 1.0 x compliance, 0.5 mm, 0.3 deg (py) | bench settled-position repeatability (lateral, vertical, yaw) |
| MagnetEdgeForceFraction, MagnetLateralScale | 0.05, 20 mm | load-cell force-gap and lateral-offset curves |
| HookDeflectionAtRatedFactor | 1.0 | hook deflection at rated load (stiffness) |
| ClosedOnNothingWindowFactor / Probability | 3x / 0.5 | hook catch CAD and bench misaligned-close statistics |
| HookMissCatchProbability, SensorFalsePositiveProbability | 0.02, 0.02 | bench >= 500 cycles vs ground truth |
| MaxClosingSpeed | 0.15 m/s | impact test |
| HookTimeoutFactor, ReleaseTimeoutFactor | 2.0, 1.5 | actuation-time spread under load |
| PoseMeasureSd (lat/yaw/height) | 0.1 mm / 0.1 deg / 0.1 mm | verification sensor repeatability |
| VerificationLossFactor, LoadConsistencyMinFraction, TestPullTension | 1.5, 0.75, 20 N | latched-state noise; good-latch stiffness repeatability; policy |
| FaceFriction, JamPosition | 0.3, 0.6 | friction pull test; hook CAD |
| MaxRetries, BackoffGap, ObstructionDebounce | 3, 5 cm, 0.3 s | operations policy; rover positioning; sensor latency |
| DeckContactInset, SupportMargin | 3 cm, 2 cm | CAD contact patches; payload slide/tip tests |

Profile fields used: `face_frames`, `active_faces_default`, `guide_half_width`, `guide_acceptance_half_angle`, `magnet_pull_in_range`, `magnet_hold_force`,
`hook_count` (2 = two hooks per face, v0; 1 = reciprocal hook + pin per face, v1), `hook_lateral_spacing`, `hook_position_tolerance`, `hook_yaw_tolerance`, `hook_actuation_time`, `hook_rated_load`,
`connector_compliance_lateral` (read as lateral play in metres; also sets hook stiffness), `states`, `fault_states`, `release_invariant`, `chassis.body_size_lwh`, `chassis.base_mass`, `payloads`.
Provenance labels travel with every value (`DockingProfile.AllProvenance()`); a test asserts none of the distinctive profile numbers appear as literals in the sources.

## Reciprocal faces (fleet_profile_v1.json)

`DockingProfile.Mode` is selected by `docking.hook_count`: 2 = `TwoHooksPerFace` (v0, unchanged values), 1 = `Reciprocal` (v1). Each v1 face has one J-hook (face -y) and one catch pin (face +y); Rz(pi) maps each hook onto the partner's pin, so a mated connection still has two hooks, one per rover.

- Mapping: HOOK_A = the initiating rover's own hook, HOOK_B = the partner's hook. In the docking frame A sits at `-hook_lateral_spacing/2`, B at `+` (`HookASide = -1`, `ContactModel.SideOf`). Positive moment loads the +lateral hook (B here, A in v0).
- Sensing: each rover reads only its own hook. HOOK_B's state reaches the initiator through a connection-level report, modelled in `ConnectorMachine`: sampled each step, applied `DockingAssumptions.PartnerLinkLatency` (0.1 s, assumption) later; the partner's close command is delayed by the same latency. `ConnectorInputs.PartnerLinkUp = false` drops reports; after `PartnerObservationMaxAge` (0.5 s, assumption) HOOK_B reads not engaged (VERIFICATION_LOST) and `PartnerObservationFresh` is false, which also refuses release of a verified connection (fail-safe). In v0 mode none of this applies.
- Unchanged: the state set, LATCHED_VERIFIED needing both hooks plus pose residual within tolerance, and "release of a load-bearing connection only with support verified on the remaining graph" (graph authorisation plus, in v1, a fresh partner report). Between a partner hook failing and the report arriving (<= latency) the machine still shows LATCHED_VERIFIED; that window is modelled, not removed.
- Loader: provenance labels are checked against the profile's `provenance_levels` (`Provenance` now also has `Concept`, `Datasheet`); a label listed but unknown to the library is a named error. `docking.magnet` (diameter, thickness, face_y, face_z) and `hook_open_angle` are numeric (`DockingProfile.Magnet`, `HookOpenAngle`, malformed -> error with path); polarity text, `partner_acquisition`, `states_mapping_v1`, `partner_acquisition_proposal` go to `DockingProfile.Metadata` as text; `roles.*.active_faces` -> `RoleActiveFaces` (the lifter has no front dock; its front is a gripper).
- Not modelled: the hook release command is not delayed by the link; the hook open angle and magnet geometry are loaded but only the profile's pull-in range and hold force drive the contact model; per-rover (split) state machines are not built, one machine per connection sees both hooks with the delays above.

## Parity with `docking_mc/model.py`

`python3 -I parity/gen_parity_vectors.py parity/parity_vectors.json` writes 484 vectors: 288 terminal poses from real `simulate()` runs (4 floor/feedback/drive cells x 3 connector configs),
160 synthetic near the capture boundary, and 36 hand-built weak-centring cases (dead-bands huge, so residual = error) around the hook-spacing zone.
The Python dead-band draws are passed to C# explicitly, so the comparison is deterministic. Result (see `parity/parity_differences.txt`):

- Capture / miss: **484 of 484 agree**. Python codes 0 (missed) and 1 (captured, no hook): all agree exactly.
- Hook outcome: 464 agree, **20 differ**, all in one direction (C# stricter): Python both-hooks -> C# one hook: 10; Python both -> C# captured only: 1; Python one hook -> C# captured only: 9. C# never reports more hooks than Python.
- Why (and the test asserts every difference has one of these two causes):
  1. `model.py` checks the *centre* residual for "both hooks", so yaw never changes the verdict beyond its own tolerance. C# checks each hook at +-60 mm: a residual of 0.7 mm lateral with 0.2 deg yaw is 0.93 mm at one hook and 0.5 mm at the other, which together with height drops a 1 mm tolerance. This is the one-hook mechanism, absent from the Python model by construction (its dead-band keeps yaw at ~0.3 deg and lever arm ~0.3 mm).
  2. `model.py`'s one-hook rule ignores the yaw tolerance; C# requires `|yaw| <= tol_yaw` for any hook to seat.
- Where they occur: 15 of the 20 are the hand-built weak-centring vectors (residual = error, 9 of them at tol_y = 2 mm, e.g. 1.5 mm lateral with 1.4 deg yaw: Python says one hook, C# says neither because the yaw tolerance is exceeded and the far hook sees 2.9 mm); the other 5 (3 from real Monte Carlo poses, 2 synthetic) are all tol_y = 1 mm with sub-millimetre residuals, where the 60 mm lever arm of 0.2-0.6 deg of yaw tips one hook over tolerance.
  Under the nominal Python dead-band (0.5 mm / 0.3 deg) and tol_y = 2 mm there are no differences, matching the wave-3 handoff: that model leaves almost no residual to split. The C# model therefore does not claim more one-hook outcomes from the same nominal inputs; it makes them reachable once the settled residual is larger than about tol minus lever arm (weak centring, post-latch disturbance, a looser yaw band), which the tests exercise deterministically. Whether that happens on hardware depends on the measured settled-yaw repeatability.

## Unity drop-in

1. Create `Assets/Scripts/Fleet/Core/Crane.Fleet.Core.asmdef` with `"name": "Crane.Fleet.Core"`, `"rootNamespace": "Crane.Fleet"`, no references, `"noEngineReferences": true`, `"autoReferenced": false`.
2. Copy `Geometry.cs`, `Docking/*.cs`, `Graph/*.cs`, `Profile/DockingProfile.cs` and `Profile/ProfileLoader.cs` into it (the library is C# 9, no `Span`, no nullable annotations).
3. `ProfileLoader.cs` is the only file needing `System.Text.Json`. Either import `System.Text.Json.dll` (8.0.x, netstandard2.0 build) and its dependencies as plugins, or leave `ProfileLoader.cs` out and fill `DockingProfile` through its setters, then call `Validate()` (editor tooling can convert the JSON offline). Do not copy the test project.
4. Step the machine from `FixedUpdate` with the physics `dt`; keep one `ConnectorMachine` per coupling and set `Coupling.Machine` so the graph reads live states.

## Known gaps

No dynamics (poses are inputs), no friction in the hook seating, no combined shear+moment interaction beyond a couple on tension, the coupling load estimate in `ModuleGraph.CouplingLoad` is a placeholder, one coupling per face pair, hook count fixed at 2, commands are edge-triggered, hooks never re-seat after slipping without a release, and the Python models were not changed.
