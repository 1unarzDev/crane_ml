# Vendor CAD used for the AOS 7 V5 model (retrieved 2026-11-10)

The repo is public (GPL-3.0). Only a file whose licence clearly allows redistribution is committed (or a derivative of it); everything
else stays in `~/.cache/research/vendor-cad/<part>/` (one new empty directory per part, never executed; inspected only by
`python -I tools/*.py <path>`). `fetch_vendor_cad.py` re-downloads and verifies the hashes. `build.py` runs without any of these files:
non-open parts are parametric, and the OAK-D uses a box if `vendor_derived/oak_d_s2_enclosure_stepframe.stl` is missing.

| part | what I found | file / URL | sha256 | licence | committed? |
|---|---|---|---|---|---|
| OAK-D S2 | enclosure STEP (CREO export, 12 solids, 97 x 29.5 x 23.1 mm) | https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-S2/DM9098S2_enclosure.stp (linked from github.com/luxonis/depthai-hardware, DM9098_OAK-D-S2/3D_Models) | bedf072b...85cdf | MIT (repo LICENSE; README: designs dual-licensed MIT or KiCad Libraries License) | **yes, as a decimated mesh** `vendor_derived/oak_d_s2_enclosure_stepframe.stl` (15.9k faces) + licence text; attribution in `Assets/Robot Descriptions/aos7v5/NOTICE.md` and `Assets/Drones/Models/LICENSES.md`. The 8.8 MB STEP itself is not committed |
| Cube Orange / Cube | `Cube.STEP.zip` (38.2 x 38.2 x 22.1 mm), docs.cubepilot.org "CubePilot product 3D model" | gitbook file link on https://docs.cubepilot.org/3d-printing/cubepilot-product-3d-model | a5a23d68...b8 | none stated (the licences page is a 404) | **no.** Used as a dimension source; geometry is parametric |
| Cube Mini carrier | `Mini Carrier Board.STEP.zip` (PCB 38.5 x 47.4 x 1.7, hole pattern, JST strip) | same page | cae41103...719 | none stated | **no.** Measured numbers in `params.py` |
| Unitree L2 | no CAD on the public pages; user manual p.8 has the installation drawing | https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf | 95a3e52c...6b1 | (c) Unitree | no; dimensions used as facts |
| Holybro Tekko32 F4 Metal 65 A ESC | spec sheet only (43 x 44, 30.5 x 30.5 M4, 15.8 g) | holybro shop spec V1.8 PDF (URL in `fetch_vendor_cad.py`) | a445f976...3c40 | (c) Holybro | no; facts only |
| AOS 7 V5 frame | **no public STEP/DXF**; manufacturer page: 250 g, stacks, patterns | https://www.aos-rc.com/designs/aos-7-v5 ; iFlight shop spec block | n/a | n/a | parametric from dimensions |
| Jetson Orin Nano Super dev kit | **needs an NVIDIA developer login** | https://developer.nvidia.com/embedded/downloads | n/a | NVIDIA terms | parametric; **user download**, see below |
| Avenger motor | no 2806.5 in BrotherHobby's catalogue; pages have no downloads; no thrust table in HTML | https://www.brotherhobby.com/c/avenger-series_0089 | n/a | n/a | F90 geometry stand-in |
| 7 in tri-blade prop (HQ 7040) | no public CAD (hqprop.com, T-Motor page checked) | n/a | n/a | n/a | parametric blade |
| pMDDL900 OEM | brochure only (7 g); no CAD link on the product page | https://www.microhardcorp.com/pMDDL900.php | n/a | n/a | parametric |
| FrSky RX | no CAD links on frsky-rc.com product/download pages | n/a | n/a | n/a | parametric |
| antennas | generic; nothing to download | n/a | n/a | n/a | parametric |

## Please download (cannot be fetched without a login or do not exist publicly)
1. **Jetson Orin Nano (Super) Developer Kit carrier board STEP**: https://developer.nvidia.com/embedded/downloads, NVIDIA developer
   login, "Jetson Orin Nano Developer Kit" Mechanical/3D model. Put it in `~/.cache/research/vendor-cad/jetson_orin_nano_devkit/`.
   It would replace my recalled 100 x 79 x 21 mm and 86 x 58 mm hole pattern. Do not commit it (NVIDIA terms).
2. **Exact motor model and datasheet** for the "Avenger" (see SPEC.md section 7) into `~/.cache/research/vendor-cad/avenger_motor/`.
3. Optional: AOS 7 V5 plate/arm DXF from AOS RC (`aos7_v5_frame/`); a real frame drawing would replace my plate outlines.

## Derived data committed
* `vendor_derived/oak_d_s2_enclosure_stepframe.stl`: `python -I tools/step_to_mesh.py DM9098S2_enclosure.stp out.stl 0.15 9000` (STEP axes: x width, y height, z depth, front = +z).
* Numbers measured from the CubePilot STEP (`tools/inspect_step.py`, `tools/step_holes.py`) are facts: Cube 38.2 x 38.2 x 22.1, connector zone 5.6 mm,
  carrier PCB 38.5 x 47.4 x 1.7 mm, mounting holes (r 1.74) at (-9.3, 8.3), (17.1, 8.3) and about (-9.4, -21.7), (17.4, -22.4) in STEP coordinates.
