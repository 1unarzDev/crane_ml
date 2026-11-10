# TreeGen trees in the Drone Forest

Status labels follow Docs/ResearchOrchestration/CONTRACTS.md.

## Pipeline
```sh
P=/home/lunarz/.cache/research/fs-venv/bin/python; F=/home/lunarz/.cache/research/fs-scripts/fsrun; W=~/.cache/research/treegen_work
$F $P -I Tools/TreeGen/treegen_cli.py atlas   --out Assets/Drones/TreeGen/Textures/treegen_leaf_atlas.png
$F $P -I Tools/TreeGen/treegen_cli.py library --out $W/lib_seed7 --species beech,spruce,birch,snag,fallen_log \
        --count 5 --seed 7 --age-classes 0.45,0.65,0.85,1,1 --no-obj
$F $P -I Tools/TreeGen/treegen_cli.py stand   --out $W/stand_seed7 --seed 7 --n 20 --size 30 --no-obj
$F $P -I Tools/TreeGen/validate.py --dir $W/stand_seed7 --wytham <trees_summary.csv> --json ... --plots ...
# Unity (one call at a time, through the wrapper)
unity-drones LOG -quit -executeMethod Crane.Drones.TreeGenImport.Editor.TreeGenImporter.ImportFromCommandLine \
    -treegenSrc $W/lib_seed7,$W/stand_seed7 -treegenName lib_seed7,stand_seed7
unity-drones LOG -runTests -testPlatform EditMode -assemblyNames Crane.Drones.TreeGen.Tests -testResults OUT.xml
# Comparison render (NOT RUN successfully yet; run at a quiet time, orchestrator's call):
unity-drones --render LOG -quit -executeMethod Crane.Drones.TreeGenImport.Editor.TreeGenCompare.Run \
    -treegenLibrary lib_seed7 -captureOut /abs/dir [-compareDensity 0.5] [-compareRestore 1]
$F $P -I Tools/TreeGen/side_by_side.py --left DIR/builtin --right DIR/treegen --out Docs/DroneVisuals/treegen \
    --pairs 01_corridor_start_golden.png:corridor_start,03_canopy_up_golden.png:canopy_up,06_corridor_mid_dusk.png:corridor_mid
```
To build a forest with TreeGen trees only: `-executeMethod Crane.Drones.TreeGenImport.Editor.TreeGenForestProvider.BuildFromCommandLine
-treegenLibrary lib_seed7` plus the usual BuildDroneForest arguments.

## TreeGen 0.2.0 changes (Tools/TreeGen)
- **Light-driven thickening.** Each season every live internode adds ring area `cambium_k * Q_sub`, where Q_sub is the
  bud light flowing through it (Pressler). The final radius is max(pipe, cambial). Topology is unchanged; the
  `test_thickening_preserves_topology` test checks this against the 0.1.0 goldens. `height_max` was lowered for beech
  (30 to 25 m) and birch (22 to 20 m), because Wytham's canopy tops out at 22 to 25 m.
- **Twig sprays.** Weber-Penn style terminal twigs on every leaf-bearing internode, with alternating leaf pairs and a
  terminal card. Conifers get planar needle sprays and birch gets pendulous twigs. The sprays are visual only.
- **Leaf atlas.** A procedural 2x2 RGBA atlas, `Assets/Drones/TreeGen/Textures/treegen_leaf_atlas.png`.
- **Library.** Beech, spruce and birch each have 5 variants (age classes 0.45, 0.65, 0.85, 1 and 1). Snag and
  fallen_log each have 5 variants. In total there are 25 trees.
- **Previews.** `previews_v02/`. **Validation:** `validation_{stand,lib}_seed7_v02.{json,png}`. The 0.1.0 baseline is
  `validation_stand_seed7.json`.

## Validation against Wytham Woods: DBH vs height percentile
The percentile is the generated tree's height among live Wytham trees within ±25% of its DBH.

| run | live trees | median percentile | trees in the 25th to 75th percentile |
|---|---|---|---|
| 0.1.0 stand, seed 7 (before) | 15 | 93.0 | 1/15 |
| 0.2.0 stand, seed 7 | 16 | 42.7 | 8/16 |
| 0.2.0 library, seed 7 | 15 | 61.9 | 10/15 |

Outliers in the 0.2.0 stand:
- Short, open-grown edge trees are too thick (beech #1 at the 6th percentile, suppressed spruce #16 at the 1st).
- The dominant spruce and birch stay slender (78th to 82nd percentile).

Wytham is a broadleaf stand, so spruce is compared against broadleaf allometry. [U]

## Unity integration
- **`Assets/Drones/TreeGen/Runtime`** (asmdef `Crane.Drones.TreeGen`):
  - `TreeGenTree`: metadata plus soft obstacles.
  - JSON DTOs.
- **`Assets/Drones/TreeGen/Editor`** (asmdef `Crane.Drones.TreeGen.Editor`):
  - `TreeGenGlb`: a minimal reader for TreeGen's own glb. It negates X, flips winding and flips V.
  - `TreeGenGlbImporter`: a ScriptedImporter for `*.tgglb`. It uses its own extension and never claims `.glb`.
  - `TreeGenImporter`: builds the prefabs.
    - LODGroup at 0.5 / 0.2 / 0.01.
    - Bark materials reuse the Poly Haven `bark_brown_02` and `pine_bark` textures in place.
    - The leaf material uses alpha clip, is double-sided and is translucent with its own diffusion profile.
    - CapsuleColliders on layer 9, set with `FromToRotation(up, axis)`.
    - It also builds a stand prefab that places trees at the manifest positions.
  - `TreeGenForestProvider`: the seam provider.
  - `TreeGenCompare`: builds and captures both forests (TreeGen and built-in) for comparison.
- **`Imported/`** is git-ignored (about 280 MB of meshes). Regenerate it with the import command above.
- **Seam change in `BuildDroneForest.cs`** (minimal, 2 statements):
  - A new `ExternalTreeInstantiated(GameObject, TreeResult)` hook, called after the default renderer and collider are
    added.
  - It replaces them with the TreeGen prefab and makes the scale uniform.
  - The placement code then reads the scale back, so the manifest's `scale_xz`/`scale_y`/`dbh_cm`/`height_m` and
    `flight_band_segments` match the geometry.
  - `ExternalTreeProvider` is registered from TreeGen code; no registration lives in BuildDroneForest.
- **Ground truth.**
  - `flight_band_segments` now come from TreeGen capsules and soft obstacles, through `TreeResult.segs`.
  - An appended `"treegen"` section lists every TreeGen tree with world-space capsules and soft obstacles
    `[ax,ay,az,bx,by,bz,r,dead]`.
  - The `TreeGenCompare` run then rebuilds the built-in forest, so the shared manifest, scene and `Generated/seed_1337`
    end up in the built-in state. The TreeGen manifest copy is kept at
    `Assets/Drones/TreeGen/Imported/forest_seed1337_manifest_treegen.json`.

## Comparison render: status and memory
- **2026-11-09, first attempt: stopped by the orchestrator.**
  - Settings: full density, 1627 trees, `DroneForestCapture.Run` (which reopens the scene), and an inline
    world-space manifest merge.
  - The process reached about 9 GB, so the orchestrator stopped it.
  - The merge alone wrote a 340 MB manifest (222,168 capsules, 4,728,727 soft obstacles).
  - TreeGen build time for the full forest was 186 s.
- **Changes since then:**
  - The manifest's treegen section references `colliders.json` plus a transform per tree, instead of inlining.
  - Flight-band segments are limited to radius >= 5 mm.
  - `TreeGenCompare` builds at `-compareDensity` 0.5 and strips capsule GameObjects (render only).
  - It renders the 3 rigs inside the built scene with no reopen, unloads assets between passes, and finishes
    with a full built-in rebuild (`-compareRestore 1`).
- **Outputs:** `DIR/{treegen,builtin}/0{1,3,6}_*.png` and `DIR/perf.json`, then `side_by_side.py` writes
  `unity_*.png` and `sbs_*.png` here.

## Known gaps
- **Stand mode is not used in the forest.** The seam is per tree and keeps BuildDroneForest's layout (trail,
  corridor and clearing exclusions). Stands import as prefabs (`Imported/stand_seed7/Prefabs/stand.prefab`) but are
  not placed.
- **Species mapping.**
  - Pine maps to spruce and Oak maps to birch.
  - Both snag classes use the beech-derived snag.
  - fallen_log variants are imported but not placed: forest logs are props outside the seam.
- **Saplings are scaled-down young library trees** (uniform scale of about 0.3 to 0.6).
- **Birch bark is a tinted brown texture.** There is no white-birch texture.
- **Translucency** depends on HDRP picking up the diffusion profile (the same method the forest's own foliage uses).
- **Mature beech** has about 1000 capsules and about 390k LOD0 triangles.
