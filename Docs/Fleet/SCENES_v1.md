# Fleet scenes v1: modular-robot environments and tasks, one spec for fastsim and CRANE (handoff, 2026-11-10)

Branch `research/fleet-scenes` (worktree `~/worktrees/crane-fleet-scenes`, based on `research/fleet`). Code, commands and
tests are in `Tools/Fleet/scenes/` (README there). Everything here is simulation only. The C# changes are **untested in
Unity**: this session ran no Unity process.

## 1. What exists

- **Spec v1.** `crane.fleet_scenario/v1` (`schema/scenario_v1.schema.json`) extends `crane.fleet_scenario/v0` with:
  - several axis-aligned walls, and doors with width, height and threshold height;
  - shelves and obstacles with a kind and a view-blocking flag;
  - floor patches with a friction class (`hard_tile`, `carpet`, `rubber`, and `polished`, which is new and not in the profile);
  - typed zones (staging, loading, assembly, delivery, charging, slot, parking, aisle) with capacity and required yaw;
  - landmark tags, and overhead cameras with a mode;
  - payload spawns with type, mass, an unknown-mass flag, a known/unknown pose, and an optional start on a unit;
  - the fleet start: units that point at capplan module graphs (`train_v1` items or train-pool compositions) with
    their config hash, state of charge, and timed faults.

  `crane.fleet_task/v1` (`schema/task_v1.schema.json`) holds **alternative plans**. Each plan is a task graph of steps
  (goto, pickup, deliver, dock, undock, charge, observe, handoff, wait) with `after` dependencies, conditional steps
  (`when: {fault: unit}`), `abort_on_fault`, a per-plan ego and a traffic policy. The task also has a deadline and
  outcome rules (`payload_in_zone`, `unit_in_zone`, `min_soc`, plus fail-on collision, deadline, stuck or a failed step).
  `validate.py` runs a built-in JSON Schema subset validator (jsonschema is not installed) and semantic checks:
  geometry, references, catalogue consistency, start overlaps, dependency order and the scenario hash. All 270 frozen
  scenes validate.
- **Unit catalogue** (`catalog/units_v1.json`). Built with `python3 -m scenes.catalog build`; `check` passes against
  capplan. It covers carrier, carrier_v3, diff_carrier, lifter and scout (the singles), plus pair_chain, pair_side and
  l3_carrier (compositions of train-pool types). Each entry carries the capplan descriptor: mobility class, lateral
  capability, vmax per direction, minimum width, payload set, tools, module rectangles and faces. It also carries health
  variants. For example, a carrier with `fl_dead_locked` becomes `differential_like` with vmax 0, `fl_weak` drops to
  0.29 m/s, and a diff_carrier with `fl_dead_free` becomes `fixed_icr`.
- **One lowering.** `common.lower()` turns walls, doors, lintels, thresholds and obstacles into boxes. The fastsim format
  adapter and the CRANE compiler both call it.

## 2. Families (each changes feasible actions, required capability or coalition, not look)

| family | stresses (capability / decision) | plans (fastsim expectation) | train rule | held-out rule |
|---|---|---|---|---|
| narrow_door | Two offset doors with a gap between them (a dogleg). A rigid unit longer than the gap cannot turn in it. A plain narrow door does **not** separate mecanum pairs: the capplan min width is 0.40 m for a single carrier and for a chain pair. **Reconfiguration is harmful.** | single ✓ / couple_chain ✗ | min(door, gap) ≥ 0.55, threshold ≤ 0.02 | tighter clearance (< 0.55) or threshold 0.03 |
| long_payload_door | Pre-assembled L of three carriers (min width 0.80 m) carrying box_long, one door narrower than 0.80 m. **Reconfiguration is beneficial**: undock one module and the chain passes with the payload. | keep ✗ (✓ if a wide far gate exists) / reconfigure ✓ | door ≥ 0.55, one door | door < 0.55 or two doors in series |
| lateral_slot | A shelf slot that admits the robot only sideways at yaw 0 (narrower than a rotation, shallower than the robot is long). | mecanum ✓ / diffdrive ✗ | north-wall slot, width ≥ 0.52 | east-wall slot or width < 0.52 |
| friction_patches | Floor patch class (capability identification). The deadline is set from hard-tile capability. | through / around (depends) | carpet or rubber | polished (an unseen floor class) |
| aisle_traffic | Single-lane aisles with opposing flows (a WoMAPF stand-in). | reserved (aisle reservations + priority yielding) / none | 2 aisles, ≤ 4 robots | 3 aisles or > 4 robots |
| charging_contention | Battery state and fewer bays than robots; robots arrive near one bay. | nearest ✗ / assigned ✓ | 2 bays, ≤ 3 robots | > 3 robots or ≠ 2 bays |
| blocked_faces | The docking approach corridor of the default face is blocked by pallets. | fixed_face ✗ / free_face ✓ / move_target ✓ | 1 blocked face | ≥ 2 blocked faces |
| wheel_failure | A wheel fault during the carry (dead_locked: immobile; weak: 0.29 m/s), with a spare carrier. | continue / reassign (handoff to the spare) | one fault, t ≤ 24 s | two faults or a late fault (t > 24 s) |
| clutter_observer | Clutter hides a payload whose pose is unknown. Overhead camera working, occluded (a banner box) or disabled. Mast camera 0.45 m vs deck camera 0.15 m. | observer (scout) / self_search (carrier) | overhead working or occluded, ≤ 6 clutter boxes | overhead disabled or > 6 clutter boxes |

**Splits (H7 control).** The split comes from `families.RULES` applied to the scene parameters, never from the seed. The
frozen manifest is `frozen/splits_v1.json`: 20 train and 10 held-out seeds per family (270 scenes), manifest hash
`7bdc86b6a600157a`, generator-code hash `897664d5bf82df9b`. `python3 -m scenes.splits check` passes all six checks:
- L1: rule disjointness;
- L2: no exact geometry-hash leakage;
- L3: no parameter leakage;
- L4: reproducibility;
- L5: code freeze;
- L6: every fleet uses `train_v1` or train-pool configurations only, verified against `capplan.configs.train_pool`.

Held-out environments never come with held-out configurations.

Previews: `Tools/Fleet/scenes/previews/<family>.jpg` (train seed 1, with the fastsim traces of the default plan) and
`index.jpg` (a 3 × 3 sheet). Each is at most 300 KB.

## 3. Transfer status

Legend: ✓ transfers; ≈ transfers with approximations; ✗ does not transfer. "Geometry" means walls, doors, zones,
starts and payloads, checked by `scenes.consistency` to 1 mm. All 38 (family, split, plan) checks on seed 1 pass.

| family | fastsim | CRANE geometry | CRANE mission (per plan) | main CRANE gaps |
|---|---|---|---|---|
| narrow_door | ✓ | ✓ (2 doors lowered to boxes) | ≈ single, couple_chain | door sills ignored (x_v1 only); deck pickup = lifter lift + handoff |
| long_payload_door | ✓ | ✓ | ≈ reconfigure (approximation), ✗ keep | a pre-coupled 3-module L cannot be spawned. The approximation: the chain's two modules dock at t0, the lifter loads box_long, the third module is omitted |
| lateral_slot | ✓ | ✓ | ≈ mecanum (diff_carrier left out, idle), ✗ diffdrive | non-profile module types (diff_carrier) only spawn in probe missions; the slot's required yaw is not judged |
| friction_patches | ✓ (assumed speed scales) | ✓ | ≈ through, around | floor patches are visual only until `FloorAt` drives the tractive plant; `polished` is not a profile floor (hard_tile used) |
| aisle_traffic | ✓ (stand-in coordination) | ✓ | ≈ train (≤ 4 robots, carrier 3/4 as lifter_1/scout_1), ✗ held-out (5–6 robots) | 4 profile rovers max; no aisle reservations; role substitution changes mass and sensors |
| charging_contention | ✓ | ✓ | ≈ train (2 robots), ✗ held-out (4–5) | no battery and no bay reservation (charge = goto bay + wait); at most 2 cargo rovers |
| blocked_faces | ✓ | ✓ | ≈ all three plans | deck pickup = lifter handoff |
| wheel_failure | ✓ | ✓ | ≈ continue; ≈ reassign without its conditional steps | `when: fault` steps and a deck-to-deck handoff have no CRANE equivalent, so CRANE's reassign mission equals continue. Faults go to the jobs `failures` string |
| clutter_observer | ✓ | ✓ | ≈ observer, self_search | the unknown pose becomes CRANE's explore task; per-camera modes are run options (occluded = a banner box, present) |

Fields CRANE ignores or cannot build yet:
- door thresholds;
- floor patches (the per-wheel floor is not looked up yet);
- multiple doors, and walls not of constant x. These are approximated: lowered into box obstacles, because Scenario.cs
  supports one doorway in a constant-x wall and BuildMap maps only the doorway wall;
- several payloads;
- required yaw in zones;
- charging and batteries;
- aisle reservations and traffic policy;
- conditional and abort-on-fault steps;
- deck-to-deck handoff;
- pre-assembled assemblies;
- non-profile module types in autonomy missions;
- more than 2 cargo rovers or 4 rovers in total;
- per-camera overhead modes;
- the arena floor class `polished`.

Per-scene and per-plan lists are in `Tools/Fleet/scenes/crane_out/transfer_summary.json`, and in each compiled
scenario's `x_v1.gaps` / `x_v1.plan_gaps`. Emitted: 18 scenarios (one train and one held-out per family), 30 missions,
and `jobs_scenes_v1.json` (30 jobs).

**fastsim results.** This is the official `python3 -m fastsim.batch ... --generate`: 38 configs, train seeds 1–5 and
held-out seeds 1–3. That is 152 runs, 2.7 s wall per run, single process, niced, with no errors. Run directories are in
`~/.cache/crane-fleet-scenes/fastsim_v1/`; the table below is also in `Tools/Fleet/scenes/results/`. Every run is
deterministic: equal identities give byte-equal tracks (tested).

| family | split | plan | n | success | median t (s, successes) | failure causes |
|---|---|---|---|---|---|---|
| aisle_traffic | heldout | none | 3 | 0/3 | - | collision x3 |
| aisle_traffic | heldout | reserved | 3 | 1/3 | 50.4 | collision x1, stuck x1 |
| aisle_traffic | train | none | 5 | 4/5 | 31.8 | collision x1 |
| aisle_traffic | train | reserved | 5 | 3/5 | 44.7 | stuck x1, collision x1 |
| blocked_faces | heldout | fixed_face | 3 | 0/3 | - | face_blocked x3 |
| blocked_faces | heldout | free_face | 3 | 3/3 | 68.7 | - |
| blocked_faces | heldout | move_target | 3 | 3/3 | 78.1 | - |
| blocked_faces | train | fixed_face | 5 | 0/5 | - | face_blocked x5 |
| blocked_faces | train | free_face | 5 | 5/5 | 68.9 | - |
| blocked_faces | train | move_target | 5 | 5/5 | 76.2 | - |
| charging_contention | heldout | assigned | 3 | 3/3 | 121.3 | - |
| charging_contention | heldout | nearest | 3 | 0/3 | - | deadline x3 |
| charging_contention | train | assigned | 5 | 5/5 | 73.3 | - |
| charging_contention | train | nearest | 5 | 0/5 | - | deadline x5 |
| clutter_observer | heldout | observer | 3 | 3/3 | 53.8 | - |
| clutter_observer | heldout | self_search | 3 | 2/3 | 59.5 | not_found x1 |
| clutter_observer | train | observer | 5 | 5/5 | 55.4 | - |
| clutter_observer | train | self_search | 5 | 5/5 | 54.0 | - |
| friction_patches | heldout | around | 3 | 1/3 | 44.2 | deadline x2 |
| friction_patches | heldout | through | 3 | 3/3 | 33.9 | - |
| friction_patches | train | around | 5 | 4/5 | 44.5 | deadline x1 |
| friction_patches | train | through | 5 | 5/5 | 33.5 | - |
| lateral_slot | heldout | diffdrive | 3 | 0/3 | - | no_path x3 |
| lateral_slot | heldout | mecanum | 3 | 3/3 | 41.3 | - |
| lateral_slot | train | diffdrive | 5 | 0/5 | - | no_path x5 |
| lateral_slot | train | mecanum | 5 | 5/5 | 45.0 | - |
| long_payload_door | heldout | keep | 3 | 0/3 | - | no_path x3 |
| long_payload_door | heldout | reconfigure | 3 | 3/3 | 57.9 | - |
| long_payload_door | train | keep | 5 | 0/5 | - | no_path x5 |
| long_payload_door | train | reconfigure | 5 | 5/5 | 52.0 | - |
| narrow_door | heldout | couple_chain | 3 | 0/3 | - | no_path x3 |
| narrow_door | heldout | single | 3 | 3/3 | 56.0 | - |
| narrow_door | train | couple_chain | 5 | 0/5 | - | no_path x5 |
| narrow_door | train | single | 5 | 5/5 | 56.2 | - |
| wheel_failure | heldout | continue | 3 | 2/3 | 42.0 | could not reach x1 |
| wheel_failure | heldout | reassign | 3 | 3/3 | 75.5 | - |
| wheel_failure | train | continue | 5 | 2/5 | 45.9 | could not reach x3 |
| wheel_failure | train | reassign | 5 | 5/5 | 66.5 | - |

Reading the table:
- **Results that match the design.**
  - The harmful/beneficial reconfiguration pair works: narrow_door single 8/8 vs couple_chain 0/8 (no lattice path),
    and long_payload_door reconfigure 8/8 vs keep 0/8 (no seed drew the far gate).
  - lateral_slot: mecanum 8/8, diffdrive 0/8.
  - blocked_faces: fixed_face 0/8, free_face and move_target 8/8.
  - charging_contention: nearest 0/8 (deadline), assigned 8/8.
  - wheel_failure train (dead_locked during the carry): continue 2/5, reassign 5/5. In held-out, with weak and late
    faults, continuing is faster than reassigning in 2 of 3 runs. That is a real decision boundary, not a bug.
- **Weak or flaky results** (honest):
  - aisle_traffic. With independent motion, train scenes pass 4/5. The stand-in coordination (priority yielding plus
    aisle reservations) is itself imperfect: 3/5 train, 1/3 held-out, with one deadlock and one collision. This is not
    a WoMAPF solver.
  - friction_patches. Under the tier's assumed speed scales, "through" always wins: carpet costs about 1 s over a
    2.1 m patch. In fastsim the family is only useful for identification; the real test is CRANE once `FloorAt` is wired.
  - clutter_observer. The observer only matters when the overhead camera is not working: held-out self_search failed
    once (not_found). In train, both plans tie because the overhead camera sees the payload.

**mf linkage** (`scenes.mf_adapter`; results in `Tools/Fleet/scenes/results/mf_levels_train_s1.jsonl`). The same scene
becomes an mf `TaskSpec` with:
- legs: the delivering unit's start, then the pickup, then door crossing points, then the zone;
- the narrowest door it crosses;
- the payload type and true mass;
- the handling counts;
- the floor.

The config is the delivering assembly, by catalogue key: a manifest item or `ConfigRef.from_graph`. The adapter evaluates
L0 and L1 through `mf.evaluate(..., cache=False)`, fastsim in-process, and L2 as a pending CRANE job request (or
unsupported, with the reason). On train seed 1, L0/L1 agree with fastsim on the door width rule: they reject keep (the
L, "door"). They **disagree**, by construction, where mf cannot see the stress:
- couple_chain is feasible at L0/L1, but the dogleg blocks it;
- diffdrive is feasible at L0/L1, but it cannot reach the slot pose;
- fixed_face is feasible at L1, but the face corridor is blocked;
- the wheel fault is invisible (mf health is static);
- `around` = `through`, because the route via-points are not in the legs.

L0 also rejects pair_chain with box_long ("payload") while L1 accepts it; that is an L0 rule question for the mf owner.
Each TaskSpec lists these blind spots in `source.unmodelled`, so mf.select can promote them to CRANE.

## 4. Commands

See `Tools/Fleet/scenes/README.md`. In short, from `Tools/Fleet`, niced, one process at a time:
`python3 -m scenes.validate`, `scenes.splits check`, `scenes.crane_compile --library`, `scenes.consistency`,
`scenes.mf_adapter --family F`, `scenes.render --all --runs DIR`, `scenes.summarize DIR`, and
`python3 -m pytest -q scenes/tests -p no:cacheprovider`. The fastsim batch runs from the crane root:
`python3 -m fastsim.batch --config Tools/Fleet/scenes/fastsim_domain/configs/<family>-<split>-<plan>.yaml --seeds 1:6 --out DIR --variant <family>-<split>-<plan> --generate`.

## 5. Fidelity notes (fastsim fleet tier; recorded in every run.json)

- **Kinematics.** Each unit is a fastsim `KinematicPlant` subclass (`FleetPlant`): rate limits, plus per-axis caps from
  the capplan descriptor (vmax fwd/back/left/right/yaw; a differential unit has no lateral motion).
  - The core GoalTracker always turns to face its direction of travel and corrects small offsets by turning in place.
    So the plant plugin tracks every leg itself: holonomic with the yaw held for omnidirectional units, and
    turn-drive-turn for the others. The core Navigator still owns goal status, the progress checker and the ego's
    goal events.
- **Fleet.** The ego (the task's `ego` unit) is the core's plant, so its `track.jsonl`, `run.ctraj` and contacts are
  the core's. The other units are stepped by the engine at the core dt with the same classes; every unit is in
  `fleet.json` and `fleet.ctraj`.
  - Coupled assemblies are rigid: they use the union of their module rectangles and the catalogue limits of the
    composition. There is no compliance, no hooks, no magnets, and no latch mechanics: a dock is a 20 s handling time
    at the mated pose (handling times are mf.levels.HANDLING_S).
  - Payloads ride kinematically (no tipping, no support dynamics). Support is the capplan payload set. Deck "pickup"
    is a kinematic load; in CRANE it is a lifter handoff.
- **Floors and faults.**
  - Floors: the profile's lateral efficiency and a mu-based acceleration scale. The speed scale per class (carpet 0.8,
    rubber 0.95, polished 0.75) is an ASSUMPTION of this tier with no source. Slip is multiplicative noise from
    slip_sigma.
  - Door thresholds are not modelled (they are traversable).
  - Wheel faults switch to the descriptor's health variant instantly (dead_locked: immobile; weak: 0.29 m/s). For
    assemblies the rule is conservative (immobile, or ×0.58).
  - Battery: linear drain per metre and per second.
- **Perception.** fastsim's `ConeCamera` per unit, plus a 3-D slab occlusion test against view-blocking boxes (a port of
  Scenario.cs Obb). Labels are truth: no detector or tag noise, no latency. Overhead cameras are view cones; `occluded`
  is a real banner box.
- **Planning and traffic.** The planner is a Manhattan C-space lattice (0.05 m cells, 4 headings, 0.025 m margin). It is
  not CRANE's planner. Traffic is priority yielding with a swept look-ahead and optional aisle reservations; it is not a
  WoMAPF solver.
- **Contacts.** Contacts never stop a plant (fastsim semantics). Any contact deeper than 0.01 m fails the run. The core's
  run.json contacts are the ego's single-module view with static payload markers; `fleet.json` is authoritative.

## 6. What the CRANE integration agent must build and test (owner of the Unity slot)

1. **Compile and run the EditMode tests.** The C# was written without Unity.
   - `Assets/Scripts/Fleet/Core/ScenarioV1.cs` (new): the `x_v1` parser, plus `FleetSpec.V1()` and `FleetSpec.FloorAt(x, y)` extensions.
   - `AutonomyMission.cs`: landmarks come from the v1 tags when present, else from the overlay.
   - `FleetWorld.cs`: threshold sills as solid boxes, and floor-patch plates (render only).
   - `Assets/Tests/Fleet/FleetScenesV1Tests.cs`: six tests over `Tools/Fleet/scenes/crane_out/scenarios`.

   Run `Tools/Fleet/fleet.sh test` (Crane.Fleet.Tests, 27 existing tests + 6 new).
2. **Run `Tools/Fleet/fleet.sh jobs Tools/Fleet/scenes/crane_out/jobs_scenes_v1.json`.** The v0 subset needs no C#.
   - Scenarios are passed by absolute path. `Path.Combine(specDir, absolute)` returns the absolute path; check this.
   - Pickups by carrier decks are mapped to a lifter lift followed by a handoff. The box_small grasp/hold offsets are
     provisional (only box_long's come from demo v2). Expect failures there first.
   - Compare the CRANE truth outcomes with the fastsim expectations in each job's `labels.expected_fastsim`.
3. **Wire `FleetSpec.FloorAt` into the tractive plant** (per-wheel floor at the contact point). Until then, floor
   patches are visual only in CRANE.
4. **Check that 0.01–0.03 m door sills are climbable** by the 0.0485 m wheels with 0.03 m clearance (narrow_door's
   held-out rule includes 0.03 m). If they are not, that is a CRANE finding, not a bug.
5. **Still missing in CRANE** (the gap list): see section 3.

## 7. Proposals for the fastsim core (shared with other domains; not edited here)

Everything below was done with plugins (the format adapter, `DomainAdapter`, and a `vehicle.ref` plant). These core
changes would remove workarounds:
1. **Per-run limits and overrides.** `DomainAdapter.limits(scenario)` and `overrides(scenario)` (today they take no
   arguments). Workaround: the format's `parse` stores the bundle in a module registry that `limits()` reads.
2. **Several plants per run.** For example `RunContext.extra_plants`, stepped, tracked, contact-checked and written to
   `track.jsonl` / `run.ctraj` by the core. Workaround: the controller steps its own plants and writes `fleet.ctraj`.
3. **A holonomic mode in GoalTracker** (translate with the yaw held) and a final approach that does not turn in place
   to fix centimetre offsets. Workaround: the plant overrides the tracker's target.
4. **Occlusion by boxes in ConeCamera.** Only labelled disks occlude today.
5. **Dynamic elements.** A carried payload moves; the core's element array is static, so phantom contacts can appear in
   run.json.
6. **Non-convex footprints** (a union of rectangles) in `footprint_clearance`.
7. **Overrides on the batch CLI.** `fastsim.batch` cannot override YAML keys (e.g. scenario fields), so there is one
   YAML per family, split and plan (`make_configs.py`).
