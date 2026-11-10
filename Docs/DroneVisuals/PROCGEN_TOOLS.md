# Procedural Forest Tools for CRANE (2026-11-09)

[V] = verified from a vendor page or search result. [U] = unverified or from memory. Re-check prices at checkout.

## Key finding
No tool gives seeded whole-forest generation, ground-truth export and thin-branch colliders. Build the reproducible core ourselves: a seeded Python layout generator (GPL-3.0) emitting JSON of stems, branches, deadwood, undergrowth and targets. Unity instantiates tree assets from that JSON, so ground truth comes from the layout. Third-party tools only supply meshes and materials. Build thin-branch colliders as capsule chains from our own branch skeletons, not from render meshes.

HDRP is in maintenance mode (no new features beyond Switch 2) [V]. It works, but avoid HDRP-only assets.

## Comparison

| Tool | Cost | GPL / research fit | Unity 6 / HDRP | Headless / determinism | Thin-branch colliders | Verdict |
|---|---|---|---|---|---|---|
| SpeedTree Modeler | Indie $19/mo (<$100k rev), Pro $899/yr [V, page may be stale] | Redistribution in a GPL repo probably barred [U] | Imports natively [U]. Sales were broken Jan-Apr 2026, fixed in 10.2.0 [V] | GUI only | Poor (LOD/billboards) | Demo visuals only |
| Houdini Indie + Engine | $299/yr, <$100k revenue [V]; Engine Indie free [V] | Own file format [V]; lab may need commercial license [U] | Plugin; Unity 6 [U] | Strong: HDAs headless via hython, seeded; node-locked [V] | Best: we emit skeletons | Best proc-gen engine, if eligible |
| Gaia Pro VS / GeNa Pro | ~$100-150 [U] | Asset Store EULA | Gaia: Unity 6, all pipelines [V] | Editor-driven | Terrain-tree only | Skip |
| MapMagic 2 | ~$100 [U] | EULA | Reviews report Unity 6 breakage [V] | Deterministic graph; headless [U] | None | Skip |
| Vegetation Studio Pro | Unavailable [V] | n/a | Deprecated [V] | n/a | n/a | Dead |
| Nature Renderer / Vegetation Engine | ~$60-100 each [U] | EULA | HDRP shaders; Unity 6 [U] | Rendering only | None | Optional wind/shading |
| Unity Terrain + built-in trees | Free | Fine | Works | Scriptable, seeded | Capsule only | Use for ground/slope |
| Blender Sapling | Free (GPL) | Fine | FBX import | `blender -b`, seeded | Mesh; skeleton exportable | Free tree source |
| The Grove 2.3 | Starter 99 EUR, Indie 199, Studio 799 [V] | Add-on Apache-2.0; models under Core license [V]; redistribution [U] | FBX | Blender 4.2+ Linux; scripting [U] | Real branch structure | Best realistic trees per cost |
| Botaniq | ~$100 [U] | Restricted [U] | [U] | Static | Mesh | Skip |
| Megascans/Megaplants (Fab) | Free since 2026-11-05; AI training allowed [V] | Fab Standard License; do not commit raw assets | Unity use [U]; HDRP conversion [U] | Static | Mesh | Fetch via script, don't ship |
| Poly Haven / ambientCG | Free CC0 [V] | Best; shippable | Yes | Static | n/a | Ground textures, HDRIs |
| Infinigen | Free, BSD-3 [V] | Best for research; GPL-compatible [U] | No Unity path. v1 exports FBX/OBJ; v2 has no documented FBX export (issue #511) [V] | Headless Blender, seeded [V]; slow [U] | Mesh only | Offline validation forests, not Unity-native |
| L-Py / TreeQSM | Free; GPL-ish [U] | Fine [U] | Export OBJ/skeleton | Python, deterministic | Native skeletons | Free skeleton source |
| TLS plots (Wytham Woods, FOR-species20K, SilviLaser 2021, TreeLearn) | Free [V] | Open on Zenodo/CEDA [V]; check each license [U] | n/a | Static | Real QSM branches | Layout and branch-statistics priors |

## Recommended stack

### Tier 1: free ($0)
- Our seeded layout generator, with parameters fitted to Wytham Woods and FOR-species20K.
- Blender Sapling or L-Py run headless, 5-10 variants per species, plus branch skeletons.
- Capsule-chain colliders from skeletons, down to about 1 cm.
- Unity Terrain for ground, with Poly Haven and ambientCG materials and HDRIs. Megascans foliage optional.
- Optional: Infinigen forests rendered offline for cross-checks.

### Tier 2: budget (about $530-830 in year one)
- The Grove Indie, 199 EUR, after confirming redistribution terms.
- Houdini Indie, $299/yr, if the lab qualifies. It adds headless seeded HDAs that output meshes, skeletons and colliders together.
- Optional SpeedTree Indie ($19/mo) and one foliage shader asset (~$100) after a Unity 6.5 HDRP check.

## What to download or buy
1. Free downloads: Blender 4.2+, Infinigen (github.com/princeton-vl/infinigen), L-Py (OpenAlea), Wytham Woods QSMs (Zenodo 7307956), FOR-species20K (Zenodo 13255197), Poly Haven and ambientCG assets.
2. Fab account (free Epic login) to claim Megascans and Megaplants. Read the Fab Standard License on redistribution.
3. The Grove: thegrove3d.com. Indie or Starter. Studio is needed only for the Houdini plugin.
4. Houdini Indie: sidefx.com account. Revenue and funding limits apply. Engine Indie is free and node-locked to the machine.
5. SpeedTree: unity.com/products/speedtree with a Unity ID. The SpeedTree account is separate; support@speedtree.com can issue temporary licenses [V].
6. Do not buy Gaia, GeNa, MapMagic, Botaniq or Vegetation Studio Pro.

## Open risks
- GPL redistribution of third-party assets is the main license risk. Ship scripts and seeds, not assets, unless CC0 or BSD.
- Raycast cost with tens of thousands of capsules per forest is untested [U].
- Pin Blender, Houdini and Unity versions and log all seeds for determinism.
