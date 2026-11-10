# Assets/Drones/Models: licenses and provenance

No third-party meshes, textures or materials are used. Everything here is generated
deterministically by `Assets/Drones/Editor/BuildAOS7Quad.cs` (meshes, prefab) and
`Assets/Drones/Editor/DroneTextures.cs` (procedural PNGs in `Textures/`), and is
covered by the repository license (GPL-3.0). Nothing was downloaded.

## Dimension sources (facts only, no artwork copied)
- AOS 7 V5 frame: wheelbase 313 mm, frame 254 x 184 x 36 mm, 8 mm arms, 2 mm top plate, 2.5 mm bottom plate,
  19x19 motor pattern, 20x20 / 30x30 stack. https://shop.iflight.com/AOS-7-V5-Frame-Kit-Pro2311
  (RaceDayQuads, KiwiQuads, Phaser FPV list the same numbers). sqrt(254^2 + 184^2) = 313.6 mm, so the
  motor rectangle is 254 x 184 (stretched-X, not a square true-X).
- T-Motor F90 2806.5: 33.4 mm diameter x 34.7 mm height, 19x19 M3, 46.6 g.
  https://www.progressiverc.com/products/t-motor-f90-2806-5-brushless-motor
- Unitree 4D LiDAR L2: 75 x 75 x 65 mm, 230 g. https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf
- OAK-D S2: 97 x 29.5 x 22.9 mm, 75 mm stereo baseline.
  https://www.mouser.com/catalog/specsheets/Luxonis_4-13-2022_OAK-D-S2_AF%20&%20FF%20Datasheet.pdf
- Holybro Tekko32 F4 Metal 65 A 4-in-1 ESC: 43 x 44 mm, 30.5 x 30.5 mm M4 pattern.
  https://www.getfpv.com/holybro-tekko32-f4-metal-65a-4-in-1-esc.html
- Cube Orange case 38.25 x 38.25 x 22.3 mm (from memory, unverified); Mini carrier board outline, Jetson Orin Nano
  carrier outline (80 x 56 mm assumed for a compact third-party carrier), Microhard pMDDL900 module outline
  (32 x 30 mm) and the 6S pack size (108 x 36 x 38 mm) are approximations, not measured.
