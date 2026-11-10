#!/usr/bin/env python3
"""Regenerates the component table and the mass reconciliation inside SPEC.md (between the AUTO markers) from
params.py, cad_report.json and the existing CRANE profile.  python spec_table.py"""
import json, re
from pathlib import Path
import params as P

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
rep = json.loads((REPO / "Docs/DroneVisuals/model/cad_report.json").read_text())
old = json.loads((REPO / "Tools/Aerial/profiles/aos7_v5_f90_6s.json").read_text())
L = rep["links"]

def g(n): return L[n]["mass_kg"] * 1000
def c(n): return ", ".join(f"{x*1000:.1f}" for x in L[n]["com_m"])

rows = [  # link(s), component, dims mm, dims status+source, mass status+source
    (["base_link"], "AOS 7 V5 frame kit (arms, 3 plates, hardware, straps) + 6 gold stack standoffs", "motor rect 254 x 184, arm 8, top 2, bottom 2.5, upper 2, H36", "DATASHEET iFlight AOS 7 V5 page; plate outlines ESTIMATE", "325 g kit DATASHEET (iFlight 'Weight: 325g'); standoffs 7.2 g ESTIMATE"),
    (["payload_deck_link"], "Payload deck (2 mm carbon, z = 48 mm) + 5 gold standoffs", "approx. 187 x 104 (+ tabs), 2 thick", "ESTIMATE (my addition, needed for prop clearance)", "ESTIMATE (volume x 1.6 g/cc x 0.6 cut-out factor)"),
    (["motor_0", "motor_1", "motor_2", "motor_3"], "F90 2806.5 KV1300 stator/base/cable (x4)", "33.4 dia x 34.7 high incl. bell", "DATASHEET T-Motor/ProgressiveRC", "46.6 g incl. cable DATASHEET (split stator/bell ESTIMATE)"),
    (["rotor_0", "rotor_1", "rotor_2", "rotor_3"], "Bell (20 g) + HQ 7040 tri-blade prop 9 g (x4), spin about +z", "177.8 mm dia; blade shape ESTIMATE", "DATASHEET diameter", "bell split ESTIMATE; prop 9 g ESTIMATE"),
    (["esc_link"], "Holybro Tekko32 F4 Metal 65 A 4-in-1 ESC", "43 x 44 x 7 (+ 2 caps, XT60)", "DATASHEET Holybro/GetFPV; height ESTIMATE", "15.8 g DATASHEET"),
    (["cube_carrier_link"], "Cube Orange + Mini carrier", "case 38.25 x 38.25 x 22.3; carrier 45 x 40 x 1.6", "case DATASHEET (earlier fetch); carrier ESTIMATE", "Cube 40 g ESTIMATE; carrier 25 g ESTIMATE"),
    (["l2_link"], "Unitree L2 LiDAR (upright)", "75 x 75 x 65", "DATASHEET Unitree manual", "230 g DATASHEET"),
    (["jetson_link"], "Jetson Orin Nano Super dev kit (heatsink + fan)", "100 x 79 x 21", "DATASHEET NVIDIA (from memory, re-check)", "176 g ESTIMATE (dev kit class)"),
    (["usb_eth_link"], "Cable Matters USB-to-2.5GbE adapter", "60 x 22 x 13", "ESTIMATE", "25 g ESTIMATE"),
    (["radio_link"], "Microhard pMDDL900 OEM + carrier + 2 antennas", "module 32 x 30 x 6; carrier 50 x 35 x 8; antennas 140 x 8 dia", "ESTIMATE", "module 7 g DATASHEET; carrier 15 g, antennas 2 x 10 g ESTIMATE"),
    (["rx_link"], "FrSky receiver (model unknown)", "16 x 11 x 5", "ESTIMATE", "5 g ESTIMATE"),
    (["oak_d_link"], "OAK-D S2 + gold aluminium bracket", "97 x 29.5 x 22.9 (W x H x D), baseline 75", "DATASHEET Luxonis; bracket ESTIMATE", "91 g DATASHEET; bracket 15 g ESTIMATE"),
    (["battery_link"], "Tattu 6S 22.2 V 3500 mAh (slung under bottom plate)", "140 x 45 x 40", "ESTIMATE (vendor page not fetchable)", "520 g ESTIMATE"),
    (["harness_link"], "XT60 + power leads, 12 V buck for L2, signal wiring", "bundles", "ESTIMATE", "55 g ESTIMATE"),
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
rec = [("frame + arms", o("frame_center_plates", "frame_arms"), g("base_link") - 7.2, "kit weighs 325 g (DATASHEET); CRANE assumed 170 g"),
       ("gold standoffs (stack 6 + deck 5)", 0, 7.2 + g("payload_deck_link") * 0 , "new"),
       ("payload deck plate", 0, g("payload_deck_link") - 5.5, "needed above the prop plane"),
       ("motors + props", o("motors", "props"), sum(g(f"motor_{i}") + g(f"rotor_{i}") for i in range(4)), "unchanged (186.4 + 36)"),
       ("ESC", o("esc_4in1"), g("esc_link"), "unchanged"),
       ("Cube + carrier", o("cube_orange_plus", "cube_mini_carrier"), g("cube_carrier_link"), "CRANE used a 70-80 g class guess; Cube Orange is lighter"),
       ("Jetson", o("jetson_orin_nano_super"), g("jetson_link"), "full dev kit incl. big carrier and heatsink"),
       ("USB-Ethernet adapter", 0, g("usb_eth_link"), "not in the CRANE budget"),
       ("radio + antennas", o("microhard_pmddl900_oem", "microhard_carrier", "radio_antennas"), g("radio_link"), "same"),
       ("RC receiver", o("frsky_receiver"), g("rx_link"), "same"),
       ("OAK-D S2 + bracket", o("oak_d_s2"), g("oak_d_link"), "+15 g gold bracket"),
       ("L2", o("unitree_l2"), g("l2_link"), "same"),
       ("battery", o("battery_6s_lipo"), g("battery_link"), "3500 mAh pack vs assumed 1800 mAh / 290 g"),
       ("harness / regulator / misc", o("l2_regulator", "wiring_mounts_misc"), g("harness_link"), "CRANE 82 g bucket incl. GNSS; CAD 55 g, no GNSS modelled")]
# the two standoff lines are folded into base/deck in CAD; present them honestly as part of those links
rec = [r for r in rec if r[0] not in ("gold standoffs (stack 6 + deck 5)", "payload deck plate")]
rec.insert(1, ("payload deck + 5 standoffs", 0, g("payload_deck_link"), "needed above the prop plane (see section 4)"))
rec[0] = ("frame kit + 6 stack standoffs", rec[0][1], g("base_link"), "kit is 325 g (DATASHEET); CRANE assumed 170 g")
r = ["| item | CRANE profile g | CAD g | delta g | why |", "|---|---|---|---|---|"]
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
