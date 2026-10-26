# coursegen: crane course truth outside Unity

One generator for every tier. `coursegen` compiles the generator's own C# files in
`Assets/Scripts/Course/Core` and `Assets/Scripts/Course/RoboBoat/Tasks` by reference (never a
copy). It reads the family assets in `Assets/Config/Course/Resources` the way Unity deserializes
them, and writes the same truth that `CourseCli.Generate` writes in the editor, byte for byte.
`UnityEngine.CoreModule.dll` is referenced only for `Vector3` and the ScriptableObject shells. It is
taken from the Unity install named in `ProjectSettings/ProjectVersion.txt`; override it with
`-p:UnityManaged=DIR`.

## Build and run (from the crane root)

    dotnet build Tools/RoboBoat/cli -c Release
    dotnet Tools/RoboBoat/cli/bin/Release/net10.0/coursegen.dll --seed 1 --out s1.json
    dotnet Tools/RoboBoat/cli/bin/Release/net10.0/coursegen.dll --seeds 1:201 --mode corner=hi --out DIR
    #   [--family roboboat_2026] [--mode iid|corner=lo|corner=hi|corner=mixed] [--set task.param=value]...
    #   --seed without --out writes the truth to stdout; --seeds writes DIR/<family>-s<N>.json (the editor's
#   CourseCli names) + failures.txt and prints one JSON summary line {family, seeds, failed, retried, ms, identity, out}
    #   --version   the generator version of this checkout (JSON)
    #   --sources   the compiled generator files and their sha256

Each run takes about 1 ms per seed. The options are the same as in Unity: `--seed` is
`--crane-course-seed`, `--mode` is `--crane-course-mode`, `--set` is `--crane-course-set`, and
`--family` is `--crane-course-family`. The rig passes them after `--` (or as a course JSON
`{family, seed, mode, set}` given with `-c`). Layout side and mirroring are drawn from the seed, so
they are not options. The binary records each source file's sha256 when it is built. If a generator
file changes after the build, it exits 3 with "stale build", so rebuild it then.

Exit codes: 0 ok, 2 bad arguments, 3 stale build or a seed that failed to generate (listed in
failures.txt).

## Identity (`generator.identity`, identity version 1)

Every truth from the generator carries this block, whether it came from Unity (editor or player)
or from coursegen. Schema `crane.course_truth/2` is unchanged. The block is additive, and old
readers ignore it.

    "identity": {
      "version": 1,
      "seed": 1,
      "options": {"family": "roboboat_2026", "mode": "iid", "set": {}},
      "generator": {"crane_commit": "<40 hex>", "crane_dirty": false, "source_sha256": "<64 hex>"},
      "truth_sha256": "<64 hex>",
      "pair_key": "<64 hex>"
    }

- `source_sha256` is the sha256 of the lines `"<path> <sha256 of file>\n"`, sorted by path, over
  every non-.meta file below `CourseIdentity.SourceDirs`. That covers the generator code and the
  family, task, catalog and constraint assets. Two truths with the same `source_sha256` were made
  by the same generator, whatever `crane_commit` says. `crane_commit` and `crane_dirty` are
  provenance; dirty means only the source set differs from HEAD.
- `truth_sha256` is the content hash, the same value as `generator.truth_sha256` (the goldens).
  It is the sha256 of the canonical compact JSON (insertion order, numbers rounded to 6 decimals)
  of the course-frame content, which is everything except the root keys `schema ... label_mode`
  and `generator.{crane_commit, truth_sha256, identity}`. Unity moves a session's truth to the
  world frame without changing this hash.
- `pair_key` is sha256(canonical `{"seed": N, "options": {...}, "source_sha256": "..."}`).
- A player records the version of the checkout it was built from: `CranePerformanceBuild` embeds
  it. A player built before this change, or by another build path, records `unknown` and cannot
  be paired.

Python: `course_truth.py identity TRUTH` prints the block and exits 1 if it is inconsistent.
`course_truth.check_identity(t)` recomputes the pair key and, for a course-frame truth, the
content hash. `course_truth.pair_key(seed, options, source_sha256)` gives the key for a run you
have not made yet.

## Pairing (fast tier and Unity)

1. Fast tier: `coursegen.dll --seed N [options] --out course_truth.json` from a crane checkout,
   then store the whole `identity` in the run record.
2. Unity: run the same seed and options on the rig. The session's `course_truth.json` carries the
   identity, and `player.log` has `CRANE_COURSE_READY ... sha256= pair_key= source_sha256=`.
3. Pair runs on an equal `pair_key`, which covers seed, options and generator source, then require
   an equal `truth_sha256`. A mismatch means a generator nondeterminism or a tampered file. Never
   pair on it.

## Tests

- `python3 -m unittest discover -s Tools/RoboBoat/tests -p 'test_cli.py'`: compiled files are
  exactly the generator in Assets/ (fails on a copy), plus identity, goldens, validate.py,
  stale-build refusal and tamper detection. With `CRANE_COURSE_CLI_DIR=DIR` it also writes the
  parity set: seeds 1-50 in each of the 4 modes, plus 10 pinned.
- Unity EditMode `CourseCliParityTests`, run with the same `CRANE_COURSE_CLI_DIR`, regenerates
  every truth in the editor and requires byte equality, identity included.
