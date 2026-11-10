# CRANE TreeGen: procedural tree and forest generator design

_Researched 2026-11-09. [V] means verified against a primary source or license file. [U] means unverified or a prior estimate. Only public docs, papers and license-compatible code were read. Nothing proprietary was consulted._

## 1. Decision
Write our own **self-organizing tree model (SOTM)** in numpy under GPL-3.0, in `Tools/TreeGen/`. It follows Pałubicki et al. 2009 (SIGGRAPH, "Self-organizing tree models for image synthesis"), which buds competing for light, Borchert–Honda resource allocation, and shedding. We add The Grove-style refinements: ray-traced or voxel shade, a node-level photosynthesis-to-cambium death rule, and beam bending. Weber–Penn (via `friggog/tree-gen`, GPL-3.0) is the fast fallback and the source for twig rules. No commercial tool can ship output into the repo: The Grove's Core EULA forbids distributing grown trees [V], and SpeedTree's EULA probably does too [U].

## 2. What the commercial tools do
**The Grove** (thegrove3d.com) [V]
- Closed Rust core (PyO3). Blender add-on is Apache-2.0 UI glue; outputs fall under the Core EULA ("may not sell or distribute ... 3D models of trees grown with The Grove Core").
- Its design principles are published:
  - Every branch order runs the same algorithm, which makes it a *grower* rather than a *modeler*.
  - Branch tips seek blue-sky light. Shade is ray-traced against foliage triangles; 2.1 added tip triangles and shifted the shade geometry to raise light contrast.
  - Each yearly cycle drops weak branches, adds buds, grows, thickens, then bends.
  - Parameters: bud Chance/Reduce/Fork/Regenerative, phyllotaxis, Angle; Turn Random/Up/To Light; Favor Ends/Bright (apical control, light bias).
  - Euler–Bernoulli beam bending; reaction-wood up-bend proportional to thickening rate.
  - 2.1 replaced the pipe rule with photosynthate-and-hormone thickening, and a node dies when its photosynthesis-to-cambium ratio drops too low.
- No papers are cited, but the model is plainly in the Pałubicki/Prusinkiewicz family.
- **Why it looks good:** emergent light competition (hollow, gappy crowns, natural self-pruning), age-correct thickening, sag, dead wood, and well-tuned species presets.

**SideFX Labs Tree Tools** [V]
- Tree Controller, Trunk Generator (1.1), Branch Generator (1.2), Branch Placer, Leaf Generator, Simple Leaf, Tree Hierarchy and Quick Basic Tree, all SOP HDAs.
- *Rule-based modelers*, not growers: per-level branch placement (count, Place/Alternate, angle, radius falloff, tropism, noise); atlas leaf cards with bend/gravity/light; meshes plus curves.
- They are published as expanded HDAs in `github.com/sideeffects/SideFXLabs` under `otls/tree_*.hda`.
- LICENSE.md is BSD-style (keep notice, no endorsement): GPL-compatible, so reading them is fine. They need Houdini, so we take ideas only.

## 3. Open-source code we can reuse
| Project | Licence [V] | Algorithm | Reuse |
|---|---|---|---|
| Blender Sapling (Blender extensions; old `blender-addons` repo archived) | GPL-2.0-or-later | Weber & Penn 1995 | Can port. Blender-bound (bpy). |
| `github.com/friggog/tree-gen` | GPL-3.0 | Weber–Penn, cleaner than Sapling, with species presets | **Port the parameter sets and twig/leaf rules** directly. |
| Infinigen `src/infinigen/assets/objects/trees/{tree.py,branch.py,treeconfigs.py,generate.py}` (`github.com/princeton-vl/infinigen`) | BSD-3-Clause | Recursive random-walk paths plus `space_colonization()` (Runions 2007) for crowns and roots. Radii depend on reverse depth; skinning uses Geometry Nodes | **Port `space_colonization`** and the skeleton bookkeeping (it is plain numpy). Keep the BSD notice. It uses the global `np.random`, so pass in a `Generator` instead. |
| Mtree / Modular Tree (`github.com/MaximeHerpin/modular_tree`) | Add-on GPL-3.0, C++ core MIT | Composable TrunkFunction/BranchFunction; `ManifoldMesher` gives watertight branch junctions | **Port the junction-meshing approach** (MIT). |
| ProcTree.js (`github.com/supereggbert/proctree.js`) | BSD-3-Clause (header, © 2012 Paul Brunt). No LICENSE file | Recursive clumped branching with twig cards | Reference only (low quality). |
| L-Py / OpenAlea (`github.com/openalea/lpy`) | CeCILL-C (conda/OpenAlea). FSF: **GPL-incompatible** | L-systems | **Do not vendor or port code.** Its papers and models are fine as references. |
| TreeQSM (`github.com/InverseTampere/TreeQSM`) | GPL-3.0-or-later (v2.4.0) | Cylinder QSMs from TLS (MATLAB) | We can use it to fit our own scans, and its cylinder format is our calibration input. |
| Tree-It (Evolved Software) | Closed freeware | n/a | Skip. |
| SOTM code | None suitable (blog has no code; `epabarker/tree-simulation` is 2D, licence unchecked) | | Implement from paper. |

## 4. Algorithm stack
One season per iteration, on a skeleton graph:
1. **Light:** shadow-propagation voxel grid (Pałubicki 2009): each bud adds shadow `a·b^-q` to a pyramid below it; `Q = max(C − s + a, 0)`; `V` points to the least-shaded voxel in a perception cone. Forests share one grid, giving inter-tree competition. Optional ray-traced sky view for final seasons.
2. **Space:** space-colonization markers as an alternative resource where crown-shape priors are strong (conifers).
3. **Resource:** accumulate `Q` basipetally and distribute `v` acropetally with Borchert–Honda: `v_m = v·λQ_m/(λQ_m+(1−λ)Q_l)`. λ sets apical control; the blog's examples are 0.46 spreading and 0.56 excurrent [U].
4. **Bud fates:** grow `⌊v⌋` metamers if ≥1, else dormant, dying after `T_dorm`; abort (`p_abort`); fork/flower terminal (sympodial); epicormic (`p_regen`). Phyllotaxis spiral 137.5°, decussate or whorled, parallel-transported frame.
5. **Direction:** `d = norm(w0·d_prev + wV·V + wg·g_dir(order) + wh·horiz + wn·noise)`. Gravitropism varies by order (orthotropic leader, plagiotropic laterals).
6. **Shedding:** cut when `Q_subtree/metamers < θ_shed` after a grace period, or at node death (`light/cambium_area < θ`); leave a dead stub (`p_stub`, `T_decay`). Stubs matter for UAVs.
7. **Radii:** pipe model `r_p^n = Σ r_i^n` from tip radius `r_0`; shed branches still count; n per species, about 1.8–2.5 [U]. Optional light-driven thickening calibrated to match the pipe-model equilibrium.
8. **Bending:** Euler–Bernoulli sag from loads (wood E per species), then reaction-wood up-bend.
9. **Twigs and leaves:** below `r_twig`, Weber–Penn twig sprays (from tree-gen); per-metamer leaf cards oriented between sun and crown normal.

## 5. Data structures
```
Skeleton (struct-of-arrays, numpy):
  pos[N,3] float64, parent[N] int32 (-1 root), radius[N], order[N] uint8,
  age[N], alive[N] bool, dead_since[N], light[N], frame[N,3,3]
  bud table: node, kind(terminal/axillary/latent), age, state
```
Branches are maximal chains; child lists derived as CSR. Metres; Y-up conversion at export.

## 6. Outputs (per tree id + seed)
- **Mesh:** tubes with Mtree-style manifold junctions; radial segments scale with radius; U = round(2πr/bark_tile) wraps, V = arc length/bark_tile (constant texel density); trunk flare. Export `.glb`.
- **Leaf cards:** instanced quads with an atlas index, as a separate submesh.
- **Skeleton JSON** (schema-versioned): nodes, radius, order, alive, plus generator version, params hash and seed. This is the ground truth for LiDAR and collision labels.
- **Colliders:** capsule chains for diameter ≥ `d_min` (2 cm in the 1–4 m band, 5 cm elsewhere [U]), collinear segments merged; thinner twigs go to a soft-obstacle list.
- **LOD:** LOD0 full; LOD1 drops r < 1 cm; LOD2 trunk + card clumps; LOD3 impostor. Colliders never LOD.

## 7. Determinism
- One `np.random.Generator(PCG64(seed))` per tree, created with `SeedSequence([forest_seed, tree_id, stream])`. Separate streams for growth, twigs and leaves mean changing the leaf count never shifts branch topology.
- No global RNG or set/dict-order dependence; buds processed in node-id order; fixed-order reductions; golden hash tests.

## 8. Species presets
JSON: phyllotaxis, λ, internode length, bud angle, tropism weights, `n_pipe`, θ_shed, dormancy, `p_regen`, E, leaf card, bark tile, target height/DBH/crown ratio. Start with Wytham species (sycamore, ash, oak, hazel, birch), Scots pine, spruce and a dead snag; seed angles/lengths from tree-gen presets.

## 9. Calibration against TLS
- **Data:** Wytham Woods TLS + TreeQSM (Zenodo 7307956, CC-BY-4.0 [V], about 1 ha UK broadleaf). FOR-species20K (Zenodo 13255198, GPL-3.0-or-later [V]) provides species clouds. Lau et al. 2018/2019 give tropical QSM-vs-harvest checks: 87% of branch length and 97% of volume recovered, with diameter bias around 8% for 20–60 cm branches [V].
- **Targets per species:** DBH–height curve, crown ratio and crown radius, branching angle by order, child/parent radius ratio, length ratio, path length vs radius, branch-diameter size distribution, and the order count.
- **Method:** CMA-ES/ABC on preset params, loss = summed Wasserstein distances; fit 2/3, hold out 1/3.
- **Caveat:** QSM cylinders below about 5–10 cm diameter are unreliable [U]. Fit thin twigs from the pipe-model exponent and from destructive or literature twig data, not from QSM counts.

## 10. Validation plan
1. Branch-diameter (log-binned), angle-by-order, radius-ratio and DBH–height distributions vs held-out QSMs (KS/Wasserstein, per-species thresholds).
2. Fitted pipe exponent on generated junctions matches preset.
3. Simulated CRANE LiDAR → TreeQSM on our trees recovers our skeleton within published QSM error.
4. Collider audit: every branch ≥ `d_min` in the 1–4 m band covered.
5. Visual critic vs reference photos.

## 11. Implementation plan
1. `Tools/TreeGen/skeleton.py`: SoA graph, CSR, JSON I/O, hashing tests.
2. `growth.py`: voxel shadow grid, Borchert–Honda, bud fates, tropisms, shedding, pipe radii. Pure numpy, headless, niced.
3. Port `space_colonization` (Infinigen, BSD notice) and the Weber–Penn twigs (tree-gen, GPL-3.0) with attribution in THIRD_PARTY_NOTICES.
4. `mesh.py` (tubes, junctions, UVs, cards, LOD) and `colliders.py`; export glb + JSON.
5. `forest.py`: seeded layout, shared light grid, per-tree streams.
6. `calibrate.py` and `validate.py` against Wytham QSMs.
7. Unity: a `TreeGenImporter` (ScriptedImporter for the JSON sidecar) builds a prefab with mesh LODGroup, HDRP bark/leaf materials, CapsuleCollider chains on a dedicated layer, and a ground-truth component.

Sources: thegrove3d.com (technical-overview, grow, releases/21, license); sidefx.com Labs docs; SideFXLabs LICENSE.md; algorithmicbotany.org/papers/selforg.sig2009.small.pdf; brashandplucky.com/2026/10/15/growing-trees.html; gnu.org license list; Lau et al. 2018.
