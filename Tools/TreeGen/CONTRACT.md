# TreeGen output contract (schema_version 1)

Consumers (for example `Assets/Drones/Environment/**`) should read **only** what this file
describes. Evidence label for everything here: IMPLEMENTED, with FIXTURE_VALIDATED Python tests
(`Tools/TreeGen/tests`). No Unity import has been run (NOT_RUN).

## 1. Generating
```
P=/home/lunarz/.cache/research/fs-venv/bin/python
fsrun $P Tools/TreeGen/treegen_cli.py library --out OUT --species beech,spruce,birch,snag,fallen_log --count 2 --seed 7
fsrun $P Tools/TreeGen/treegen_cli.py stand   --out OUT --seed 7 --n 20 --size 30 [--layout cluster] [--config stand.json]
fsrun $P Tools/TreeGen/treegen_cli.py preview --out PNGDIR --stand OUT   (or --library OUT)
```
`--no-obj` skips the OBJ copies. Same seed and preset give byte-identical skeletons (golden hashes
in the tests). Presets live in `Tools/TreeGen/presets/*.json`. A stand config is a JSON object that
overrides any key of `treegen.forest.DEFAULT_STAND`.

## 2. Directory layout
```
OUT/
  stand_manifest.json  | library_manifest.json
  trees/tree_NNNN/                 (NNNN = tree_id, zero-padded to 4 digits)
    skeleton.json                  ground truth graph
    colliders.json                 capsules + soft obstacles + foliage volumes (never LOD'd)
    lod0.glb lod1.glb lod2.glb     glTF 2.0 binary; nodes "bark" and "leaves" (leaves may be absent)
    obj/lod{0,1,2}_bark.obj/.mtl   same geometry as the glb, one part per file (Unity imports natively)
    obj/lod{0,1,2}_leaves.obj/.mtl
```

## 3. Coordinate frames (read this first)
- **"unity" frame** = left-handed, Y up, metres. Every JSON coordinate uses it. Tree-local JSON
  (`skeleton.json`, `colliders.json`) is relative to the tree's root (stem base on the ground at the
  local origin).
- Stand manifest `position_unity` = the tree root in stand-local coordinates (stand corner at 0,
  ground at y=0, x in [0,width], z in [0,depth]). `yaw_deg` is always 0: trees were grown in place
  inside one shared light field, so don't rotate stand trees. Library trees may be rotated
  freely about Y.
- **Mesh files** (glb/obj) store `(-x_u, y_u, z_u)`, a right-handed rotation of the unity frame.
  Unity's OBJ importer and glTFast both negate X when importing, which gives the unity frame
  exactly. **Check once:** after importing, the renderer bounds must match `bounds_unity_local`
  in the manifest (a tree leans asymmetrically, so a mirror shows up as min.x/max.x swapped and
  negated). If an importer does not negate X, scale the mesh root by (-1,1,1).
- Internally TreeGen is right-handed Z-up: unity = (x, z, y). Python users can call
  `treegen.skeleton.to_unity` / `from_unity`.

## 4. stand_manifest.json / library_manifest.json
```
{ schema: "crane.treegen.stand" | "crane.treegen.library", schema_version: 1,
  generator, generator_version, forest_seed, frame, seasons, light, stand{...},
  species_params_hash{species: hash16}, timing_s{grow, export},
  trees: [ { tree_id, species, variant: "live"|"snag"|"fallen_log",
             position_unity:[x,y,z], yaw_deg:0, dir:"trees/tree_NNNN",
             files:{skeleton, colliders, lods:{"0":{glb, obj_bark, obj_leaves?}, "1":{..}, "2":{..}}},
             skeleton_hash, params_hash, bounds_unity_local:{min:[3], max:[3]},
             metrics:{height_m, dbh_m (NaN if <1.3 m or log), crown_base_m, crown_ratio, crown_radius_m,
                      nodes, dead_nodes, max_order, branch_angle_median_deg_by_order, n_branches_by_order},
             n_capsules, n_soft_obstacles, tri_counts:{"0":{bark_tris, leaf_tris},...}, start_season } ] }
```
`species` for variants is the preset name (`snag`, `fallen_log`). In stands, `species` is the
growth species and `variant` says whether it was converted afterwards. All paths are relative
to the manifest's directory.

## 5. skeleton.json (ground truth for LiDAR and collision labels)
```
{ schema:"crane.treegen.skeleton", schema_version:1, generator, generator_version, frame,
  species, variant?, forest_seed, tree_id, params_hash, seasons, start_season, n_pipe, r_tip,
  flare, flare_height, origin_unity, stand_position_unity?, metrics, bounds_unity,
  node_count, skeleton_hash,
  nodes: { pos:[x0,y0,z0,x1,...] (flat, unity frame, tree-local),
           parent:[int] (-1 root; parent[i] < i), radius:[m] (radius of internode parent->i, incl. trunk flare),
           order:[int] (0 stem), alive:[0|1] (0 = dead wood: stub/snag/log),
           is_main:[0|1] (continues parent's axis), birth:[season], dead_since:[season|-1] } }
```
Internode i is the cylinder/frustum from `pos[parent[i]]` (radius `radius[parent[i]]`) to
`pos[i]` (radius `radius[i]`). Branch chains are maximal runs of `is_main` links.

## 6. colliders.json
```
{ schema:"crane.treegen.colliders", schema_version:1, frame, tree_id, species, skeleton_hash,
  thresholds:{band_lo:1.0, band_hi:4.0, band_min_d:0.02, default_min_d:0.05, soft_min_d:0.004,
              merge_tol_frac:0.25, max_capsule_len:2.5},
  capsules:       [ {a:{x,y,z}, b:{x,y,z}, radius, center:{x,y,z}, axis:{x,y,z}, height, node_a, node_b, dead} ],
  soft_obstacles: [ same fields ],
  foliage_volumes:[ {center:{x,y,z}, radius} ] }
```
- **Hard rule:** every internode with diameter >= 2 cm that overlaps 1-4 m above the tree base,
  or >= 5 cm anywhere else, lies inside a capsule. Each capsule's radius is the maximum radius in
  its run. This is verified by `test_collider_coverage`.
- Unity CapsuleCollider: make a child GameObject at `center`, set
  `localRotation = Quaternion.FromToRotation(Vector3.up, axis)`, `direction = 1` (Y),
  `height` and `radius` as given (`height` already includes both hemispherical caps:
  `|b-a| + 2r`). The fields are JsonUtility-friendly (`{x,y,z}` objects, no nested arrays).
  Put colliders on a dedicated layer. Colliders come from the full skeleton and never change
  with LOD.
- `soft_obstacles`: thinner twigs (diameter 4 mm to the threshold), merged the same way. Use them
  for triggers, penalties or LiDAR labels, not rigid contact. `foliage_volumes`: leaf clump
  spheres (soft, for vision/LiDAR occlusion budgets). `dead` flags stub/snag wood.

## 7. Meshes
- glb: one scene, root node `tree` (extras: tree_id, lod, skeleton_hash) with children `bark`
  and `leaves`. Each is a separate mesh with one primitive: POSITION, NORMAL, TEXCOORD_0, and
  uint32 indices. Materials are flat PBR placeholders named `<species>_bark` / `<species>_leaf`.
  `extras.texture_hint` gives the intended texture slot. The leaf material is double-sided and
  alphaMode MASK.
- Bark UV: U wraps an integer number of times around each branch (`round(2πr/bark_tile_m)`),
  and V = arc length / `bark_tile_m` (0.4–0.6 m per preset). Use a tiling bark texture with
  repeat wrap.
- Leaf UV: a 2x2 atlas. Tile 0 (u 0–0.5, v 0–0.5) is a broadleaf, tile 1 a needle spray, tile 2 a
  small broadleaf (birch), and tile 3 a foliage clump (used by LOD2). The card's long axis runs
  along V, and the card hangs from the twig at v=0.
- LODs: LOD0 is full. LOD1 drops branches with radius < 1 cm and keeps 40% of the leaf cards at
  1.55x size. LOD2 keeps branches >= 3 cm plus crossed-quad foliage clumps. Suggested
  LODGroup screen heights are 0.5 / 0.2 / 0.05, with culling below 0.01. LOD3 impostors are
  not produced.
- Junctions: each lateral tube starts with a collar ring centred on its parent node, so the
  base is buried inside the parent tube. Tips have apex caps and the stem base has a fan.
  Tubes are closed but not boolean-unioned, so expect "watertight-ish" junctions.

## 8. Stability promises
- Field names and meanings above are stable within schema_version 1. New optional fields may
  appear; ignore unknown keys.
- `skeleton_hash` changes whenever geometry changes. Use it as a cache key for imported prefabs.
