# Shared rules for drone-sprint agents (orchestrator, 2026-11-09)

1. **Unity wrapper.** Use only `~/.cache/research/fs-scripts/unity-drones [--render] <log> ...`.
   - Default mode: `-nographics`, for imports, compiles and tests.
   - `--render` mode: `xvfb-run` + `-force-vulkan`, so HDRP renders. Use it for every capture, still or video.
   - The wrapper holds a project lock, so our agents never open this project twice. If you are waiting, it is the lock. Do not start Unity any other way.
2. **Never leave non-compiling code.** All agents share the project's assemblies, so one broken file blocks everyone's compile.
   - Each agent puts its runtime code under its own asmdef: Crane.Drones.Models, Crane.Drones.Environment, Crane.Theater.Aerial, Crane.Drones.Logging, Crane.Aerial.Physics.
   - Each agent puts its Editor code under its own Editor asmdef, so failures stay isolated.
   - Before you finish, run a compile (`-quit` import) and fix your errors.
3. **Render profiles.** Cinematic settings never touch onboard sensor settings: separate Volumes and separate cameras.
4. **No commits.** The orchestrator commits.
5. **One heavy slot.** Use at most ONE `heavy` Unity slot at a time across all our work; the other slot stays free for cluster jobs (cluster session request). The project lock guarantees this. Never bypass the wrapper with private loops or direct heavy calls.
6. **No retry loops, no background Unity scripts.** One invocation through the wrapper at a time, then wait. The orchestrator killed three bypasses on 2026-11-09; they deadlocked the project lock against heavy's slots.
7. **No full EditMode suite runs.** Course generator tests take more than 10 minutes. Always pass `-testFilter <your namespace>` or `-assemblyNames <your test asmdef>`.
