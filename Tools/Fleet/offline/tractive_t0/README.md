# tractive_t0

T0 checks for `Crane.Fleet.TractivePlant` (per-wheel tractive plant for N wheels of any kind; CRANE_MODULAR_PLAN item P1). They need
no Unity.

Run them with:

    dotnet run --project Tools/Fleet/offline/tractive_t0 -p:BaseOutputPath=$HOME/.cache/crane-fleet/tractive_t0/bin/ -p:BaseIntermediateOutputPath=$HOME/.cache/crane-fleet/tractive_t0/obj/

What the checks cover:

- **Kinematic identity.** Mecanum inverse and forward kinematics agree with `MecanumGeometry`.
- **Mecanum steady twists.** The steady twist tracks the command.
- **Differential pod.** The pod gets no lateral motion from a lateral command. Its casters give it a free spin.
- **Skid-steer.** A 2x2 array of parallel conventional wheels turns only partially, by scrub.
- **Dead wheels.** On a strafe, `dead_free` and `dead_locked` give different results (locked is worse).
- **Weak wheels.** A `weak(g)` wheel caps the speed.
- **Casters.** There is no force at rest, and only rolling drag when moving.
- **Load split.** The loads sum to the weight.
- **Determinism.** Results are bit-identical between runs.
- **Low friction.** The acceleration stays at or below mu g.

The parameter values are test-local. In simulation, every value comes from the overlay `plant.tractive`.
