#!/usr/bin/env python3
"""Regenerates the component table and the mass reconciliation inside SPEC.md (between the AUTO markers) from
params.py, cad_report.json and the existing CRANE profile.  python spec_table.py"""
import json, re
from pathlib import Path
import params as P

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
rep = json.loads((REPO / "Docs/DroneVisuals/model/cad_report.json").read_text())
old = json.loads((REPO / "Tools/Aerial/profiles/aos7_v5_f90_6s_v1.json").read_text())
L = rep["links"]

def g(n): return L[n]["mass_kg"] * 1000
def c(n): return ", ".join(f"{x*1000:.1f}" for x in L[n]["com_m"])

rows = [  # link(s), component, dims mm, dims status+source, mass status+source
    (["base_link"], "AOS 7 V5 frame kit: 4 arms, bottom/top/upper plates, 6 stack standoffs + hardware, 2 battery straps", "motor rect 254 x 184, arm 8, plates 2.5 / 2 / 2, H36, 19x19 motor pattern, 30.5 stack", "DATASHEET iFlight + aos-rc.com; plate outlines and hole positions ESTIMATE (no public CAD)", "250 g DATASHEET aos-rc.com ('including hardware'); iFlight shop lists 325 g (see section 2)"),
    (["payload_deck_link"], "Payload deck (2 mm carbon, z 50..52) + 5 alu standoffs (4 x 20 mm, 1 x 2 x 22 mm) + 4 nylon Jetson spacers (10 mm) + 4 nylon radio spacers (6 mm) + screws", "232 x 116 max, chamfered/tapered, lightening slots", "ESTIMATE (own design)", "volume x density: carbon 1.6, alu 2.7, nylon 1.14, steel 7.85 g/cc"),
    (["motor_0", "motor_1", "motor_2", "motor_3"], "Motor stator/base/cable (x4); F90 2806.5 KV1300 stand-in for the user's 'Avenger'", "33.4 dia x 34.7 high incl. bell", "DATASHEET T-Motor/ProgressiveRC", "46.6 g incl. cable DATASHEET (stator/bell split ESTIMATE)"),
    (["rotor_0", "rotor_1", "rotor_2", "rotor_3"], "Bell (20 g) + HQ 7040 tri-blade prop 9 g (x4), spin about +z", "177.8 mm dia; blade shape ESTIMATE", "DATASHEET diameter", "bell split ESTIMATE; prop 9 g ESTIMATE"),
    (["esc_link"], "Holybro Tekko32 F4 Metal 65 A 4-in-1 ESC", "43 x 44 x 7 (+ 2 caps, XT60); 30.5 x 30.5 M4 holes", "DATASHEET Holybro spec V1.8; height ESTIMATE", "15.8 g DATASHEET"),
    (["cube_carrier_link"], "Cube Orange + Mini carrier + 4 x 5 mm alu spacers", "case 38.25 x 38.25 x 22.3; carrier 38.5 x 47.4 x 1.7; holes 26.4 x 30 (approx.)", "case DATASHEET (PX4 docs); carrier and holes measured from the CubePilot STEP (not redistributed)", "Cube 40 g ESTIMATE; carrier 15 g ESTIMATE; spacers volume x 2.7"),
    (["l2_link"], "Unitree L2 LiDAR (upright, base on the deck)", "75 x 75 x 65, base 24, scan centre 44.5, 4 x M3 on a 51 mm circle", "DATASHEET Unitree manual p.8", "230 g DATASHEET"),
    (["jetson_link"], "Jetson Orin Nano Super dev kit (carrier long axis across the vehicle, ports aft)", "100 x 79 x 21, holes 86 x 58", "NVIDIA, recalled from memory: VERIFY (CAD needs a login)", "176 g ESTIMATE (dev kit class)"),
    (["usb_eth_link"], "Cable Matters USB-to-2.5GbE adapter + hook-and-loop bands", "60 x 22 x 13", "ESTIMATE", "25 g ESTIMATE + 0.5 g bands"),
    (["radio_link"], "Microhard pMDDL900 OEM + carrier + 2 antennas", "module 32 x 30 x 6; carrier 35 x 50 x 8; antennas 140 x 8 dia", "ESTIMATE", "module 7 g DATASHEET; carrier 15 g, antennas 2 x 10 g ESTIMATE"),
    (["rx_link"], "FrSky receiver (model unknown)", "16 x 11 x 5", "ESTIMATE", "5 g ESTIMATE"),
    (["oak_d_link"], "OAK-D S2 enclosure (real Luxonis STEP, MIT) + lens bezels", "97 x 29.5 x 23.1, baseline 75", "DATASHEET Luxonis STEP / datasheet", "91 g DATASHEET"),
    (["oak_bracket_link"], "Gold aluminium OAK-D bracket (top strap + 2 cheeks) + 4 x M3 screws", "3 mm plate", "ESTIMATE", "15 g ESTIMATE"),
    (["battery_link"], "Tattu 6S 22.2 V 3500 mAh + 3 mm foam pad (strapped under the bottom plate)", "140 x 45 x 40", "ESTIMATE (vendor page not fetchable)", "520 g + 4 g pad ESTIMATE"),
    (["harness_link"], "XT60 + power leads, 12 V buck for L2, 4 motor-lead bundles", "bundles", "ESTIMATE", "55 g ESTIMATE"),
]
t = ["| component | link(s) | dims (mm) | dims source | mass g | CoM x, y, z (mm, FLU from frame centre) | mass source |", "|---|---|---|---|---|---|---|"]
for links, name, dims, ds, ms in rows:
    mass = sum(g(n) for n in links)
    com = c(links[0]) if len(links) == 1 else "4 x see URDF (rotor hubs z = +40, motors z = +11.5)"
    t.append(f"| {name} | {', '.join(links) if len(links) == 1 else links[0][:-1] + '0..3'} | {dims} | {ds} | {mass:.1f} | {com} | {ms} |")
t.append(f"| **total** | | | | **{rep['mass_kg']*1000:.1f}** | **{', '.join(f'{x*1000:.1f}' for x in rep['com_m'])}** | |")
table = "\n".join(t)

mb = {i["item"]: i["mass_kg"] * 1000 for i in old["massBudget"]}
def o(*k): return sum(mb[x] for x in k)
rec = [("frame kit (+ standoffs, straps)", o("frame_center_plates", "frame_arms"), g("base_link"), "vendor 250 g incl. hardware vs 170 g guessed"),
       ("payload deck + standoffs + spacers", 0, g("payload_deck_link"), "new: needed above the prop plane"),
       ("motors + props", o("motors", "props"), sum(g(f"motor_{i}") + g(f"rotor_{i}") for i in range(4)), "unchanged (186.4 + 36)"),
       ("ESC", o("esc_4in1"), g("esc_link"), "unchanged"),
       ("Cube + carrier + spacers", o("cube_orange_plus", "cube_mini_carrier"), g("cube_carrier_link"), "Cube 40 g (v1 guessed 75 g)"),
       ("Jetson", o("jetson_orin_nano_super"), g("jetson_link"), "dev kit class 176 g"),
       ("USB-Ethernet adapter", 0, g("usb_eth_link"), "not in the v1 budget"),
       ("radio + antennas", o("microhard_pmddl900_oem", "microhard_carrier", "radio_antennas"), g("radio_link"), "same"),
       ("RC receiver", o("frsky_receiver"), g("rx_link"), "same"),
       ("OAK-D S2 + bracket", o("oak_d_s2"), g("oak_d_link") + g("oak_bracket_link"), "+15 g bracket"),
       ("L2", o("unitree_l2"), g("l2_link"), "same"),
       ("battery + pad", o("battery_6s_lipo"), g("battery_link"), "3500 mAh pack vs assumed 1800 mAh / 290 g"),
       ("harness / regulator / misc", o("l2_regulator", "wiring_mounts_misc"), g("harness_link"), "v1 82 g bucket incl. GNSS; CAD 55 g, no GNSS modelled")]
r = ["| item | CRANE v1 profile g | CAD g | delta g | why |", "|---|---|---|---|---|"]
so, sn = 0, 0
for n, a, b, w in rec:
    r.append(f"| {n} | {a:.1f} | {b:.1f} | {b-a:+.1f} | {w} |"); so += a; sn += b
r.append(f"| **total** | **{so:.1f}** | **{sn:.1f}** | **{sn-so:+.1f}** | |")
recon = "\n".join(r)

spec = HERE / "SPEC.md"
s = spec.read_text()
s = re.sub(r"<!-- AUTO:table -->.*?<!-- /AUTO:table -->", lambda m: "<!-- AUTO:table -->\n" + table + "\n<!-- /AUTO:table -->", s, flags=re.S)
s = re.sub(r"<!-- AUTO:recon -->.*?<!-- /AUTO:recon -->", lambda m: "<!-- AUTO:recon -->\n" + recon + "\n<!-- /AUTO:recon -->", s, flags=re.S)
spec.write_text(s)
print(table); print(); print(recon)
