import json
import os
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_repro  # noqa: E402
import drone_log  # noqa: E402
import splits  # noqa: E402


def png(w, h, depth, ctype, raw_rows):
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    data = b"".join(b"\x00" + r for r in raw_rows)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, depth, ctype, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(data)) + chunk(b"IEND", b""))


def synth_run(d, seed=1, dx=0.0, with_end=True):
    """A tiny valid run: 1 kHz state for 0.4 s, IMU 250 Hz, one depth camera, one lidar."""
    os.makedirs(os.path.join(d, "camera", "dep"))
    os.makedirs(os.path.join(d, "lidar", "lidar"))
    clk = {"fixed_dt_ns": 5000000, "substeps": 5, "substep_dt_ns": 1000000, "substep_hz": 1000.0,
           "state_every_substeps": 1, "state_hz": 1000.0}
    n = 400
    man = {"schema": "crane-drone-log/1", "run_id": os.path.basename(d), "scene_seed": seed, "split": "dev",
           "wall_clock_start_utc": os.path.basename(d), "clock": clk, "imu": {"rate_hz": 250.0}}
    if with_end:
        man["end"] = {"end_tick": 80, "end_t_ns": n * 1000000, "counts": {"state": n}}
    json.dump(man, open(os.path.join(d, "manifest.json"), "w"))
    with open(os.path.join(d, "state.csv"), "w") as f:
        f.write("tick,sub,t_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,ax,ay,az,rpm0,cmd0,batt_v,batt_a,batt_soc\n")
        for i in range(1, n + 1):
            t = i * 1000000
            f.write("%d,%d,%d,%r,0,1.5,1,0,0,0,2,0,0,0,0,0,0,0,0,5000,0.5,14,10,0.9\n" % (i // 5, i % 5, t, 0.002 * i + dx))
    with open(os.path.join(d, "imu.csv"), "w") as f:
        f.write("t_ns,gx,gy,gz,ax,ay,az,noise_flags\n")
        for i in range(1, n // 4 + 1):
            f.write("%d,0,0,0,0,0,9.8,1\n" % (i * 4000000))
    with open(os.path.join(d, "camera", "dep.csv"), "w") as f:
        f.write("seq,acq_ns,complete_ns,width,height,kind,file\n")
        for s in range(3):
            rel = "camera/dep/%06d.png" % s
            open(os.path.join(d, rel), "wb").write(png(2, 1, 16, 0, [struct.pack(">HH", 1000 + s, 2000)]))
            f.write("%d,%d,%d,2,1,Depth,%s\n" % (s, 40000000 * (s + 1), 40000000 * (s + 1) + 10000000, rel))
    with open(os.path.join(d, "lidar", "lidar.csv"), "w") as f:
        f.write("seq,acq_ns,points,t_first_ns,t_last_ns,file\n")
        rel = "lidar/lidar/000000.bin"
        open(os.path.join(d, rel), "wb").write(b"".join(struct.pack("<ffffi", i, 0, 0, 1, i * 1000) for i in range(4)))
        f.write("0,100000000,4,100000000,100003000,%s\n" % rel)
    with open(os.path.join(d, "events.jsonl"), "w") as f:
        f.write(json.dumps({"t_ns": 0, "kind": "marker"}) + "\n")
        f.write(json.dumps({"t_ns": 200000000, "kind": "near_miss", "distance_m": 0.2}) + "\n")
    json.dump({"obstacles": [{"x": 1, "y": 0, "r": 0.3}]}, open(os.path.join(d, "obstacles.json"), "w"))


class SplitsTests(unittest.TestCase):
    def setUp(self):
        self.s = splits.load()

    def test_ranges_disjoint_and_heldout_present(self):
        self.assertEqual(splits.split_of_seed(self.s, 1337), "dev")
        self.assertEqual(splits.split_of_seed(self.s, 95000), "heldout")
        self.assertEqual(splits.split_of_seed(self.s, 123456), "unassigned")

    def test_overlap_detected(self):
        bad = {"seed_ranges": [{"split": "a", "start": 0, "end": 10}, {"split": "b", "start": 10, "end": 20}]}
        with self.assertRaises(ValueError):
            splits.validate(bad)

    def test_guard(self):
        with self.assertRaises(splits.HeldoutRefused):
            splits.enforce(self.s, 95000)
        with self.assertRaises(splits.HeldoutRefused):
            splits.enforce(self.s, 123456)
        with self.assertRaises(splits.HeldoutRefused):
            splits.enforce(self.s, 500, wind="storm_gust")
        with self.assertRaises(splits.HeldoutRefused):
            splits.enforce(self.s, 500, lighting="HeavyFog")
        self.assertEqual(splits.enforce(self.s, 95000, allow_heldout=True), "heldout")
        self.assertEqual(splits.enforce(self.s, 75000, "moderate_gusty", "Dusk"), "val")

    def test_argv_entry(self):
        with self.assertRaises(splits.HeldoutRefused):
            splits.enforce_argv(["--drone-seed", "99999"])
        with self.assertRaises(splits.HeldoutRefused):
            splits.enforce_argv([])
        self.assertEqual(splits.enforce_argv(["--drone-seed", "99999", "--allow-heldout"]), "heldout")

    def test_every_disturbance_value_has_one_split_or_dev_overlap(self):
        seen = {}
        for d in self.s["disturbance"]:
            for v in d["values"]:
                if d["split"] == "heldout":
                    self.assertNotIn(("x", v), seen)
                    for o in self.s["disturbance"]:
                        if o["family"] == d["family"] and o["split"] != "heldout":
                            self.assertNotIn(v, o["values"])


class LogTests(unittest.TestCase):
    def test_load_validate_align_summary(self):
        with tempfile.TemporaryDirectory() as t:
            d = os.path.join(t, "r"); os.makedirs(d); synth_run(d)
            log = drone_log.DroneLog(d)
            self.assertEqual(log.validate(), [])
            s = log.summary()
            self.assertAlmostEqual(s["state_hz"], 1000.0, delta=1)
            self.assertEqual(s["events"], {"marker": 1, "near_miss": 1})
            a = log.align([2500000], ("px",))
            self.assertAlmostEqual(a["px"][0], 0.005, places=9)
            pose = log.camera_poses("dep")
            self.assertEqual(len(pose["px"]), 3)
            mm, w, h = drone_log.depth_mm(os.path.join(d, "camera/dep/000001.png"))
            self.assertEqual((mm, w, h), ([1001, 2000], 2, 1))
            self.assertEqual(len(log.lidar_scan("lidar", 0)), 4)
            self.assertIn(log.plot_trajectory(os.path.join(t, "p.png")), (True, False))

    def test_validator_catches_problems(self):
        with tempfile.TemporaryDirectory() as t:
            d = os.path.join(t, "r"); os.makedirs(d); synth_run(d, with_end=False)
            p = os.path.join(d, "camera", "dep.csv")
            txt = open(p).read().replace("40000000,50000000", "40000000,30000000")
            open(p, "w").write(txt)
            problems = drone_log.DroneLog(d).validate()
            self.assertTrue(any("completes before acquisition" in x for x in problems))
            self.assertTrue(any("no 'end'" in x for x in problems))

    def test_check_repro(self):
        with tempfile.TemporaryDirectory() as t:
            a, b, c = (os.path.join(t, n) for n in "abc")
            for d, dx in ((a, 0.0), (b, 1e-9), (c, 1e-3)):
                os.makedirs(d); synth_run(d, dx=dx)
            self.assertEqual(check_repro.main([a, a]), 0)
            self.assertEqual(check_repro.main([a, b]), 0)
            self.assertEqual(check_repro.main([a, c]), 1)


if __name__ == "__main__":
    unittest.main()
