"""Reader, aligner, validator and quick plot for crane-drone-log/1 run directories (Docs/DroneVisuals/LOG_FORMAT.md).

Pure standard library at import time; numpy / matplotlib are optional and never imported at module load.
"""
import bisect
import csv
import json
import math
import os
import struct
import sys
import zlib

SCHEMA = "crane-drone-log/1"


def _num(x):
    return float("nan") if x == "nan" else float(x)


def _read_csv(path, int_cols=()):
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        cols = {h: [] for h in head}
        for row in r:
            for h, v in zip(head, row):
                cols[h].append(int(v) if h in int_cols else _num(v))
    return cols


class DroneLog:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        with open(os.path.join(self.path, "manifest.json")) as f:
            self.manifest = json.load(f)
        if self.manifest.get("schema") != SCHEMA:
            raise ValueError("unsupported schema %r" % self.manifest.get("schema"))
        self.state = _read_csv(os.path.join(self.path, "state.csv"), int_cols=("tick", "sub", "t_ns"))
        imu = os.path.join(self.path, "imu.csv")
        self.imu = _read_csv(imu, int_cols=("t_ns", "noise_flags")) if os.path.exists(imu) else {}
        self.events = []
        ev = os.path.join(self.path, "events.jsonl")
        if os.path.exists(ev):
            with open(ev) as f:
                self.events = [json.loads(line) for line in f if line.strip()]
        self.cameras = {}
        cdir = os.path.join(self.path, "camera")
        if os.path.isdir(cdir):
            for fn in sorted(os.listdir(cdir)):
                if fn.endswith(".csv"):
                    self.cameras[fn[:-4]] = _read_csv_str(os.path.join(cdir, fn))
        self.lidars = {}
        ldir = os.path.join(self.path, "lidar")
        if os.path.isdir(ldir):
            for fn in sorted(os.listdir(ldir)):
                if fn.endswith(".csv"):
                    self.lidars[fn[:-4]] = _read_csv_str(os.path.join(ldir, fn))
        self.obstacles = []
        op = os.path.join(self.path, "obstacles.json")
        if os.path.exists(op):
            with open(op) as f:
                self.obstacles = json.load(f).get("obstacles", [])

    # ---- streams
    def lidar_scan(self, sensor, seq):
        """Return list of (x, y, z, intensity, t_abs_ns) for one scan."""
        idx = self.lidars[sensor]
        acq = int(idx["acq_ns"][seq])
        data = open(os.path.join(self.path, idx["file"][seq]), "rb").read()
        return [(x, y, z, i, acq + t) for x, y, z, i, t in struct.iter_unpack("<ffffi", data)]

    def camera_frame(self, camera, seq):
        """Return (width, height, channels, bit_depth, bytes) of the decoded PNG."""
        return read_png(os.path.join(self.path, self.cameras[camera]["file"][seq]))

    def t_s(self):
        return [t * 1e-9 for t in self.state["t_ns"]]

    # ---- alignment
    def align(self, times_ns, columns=("px", "py", "pz", "vx", "vy", "vz")):
        """Linearly interpolate ground-truth state columns at arbitrary sim-clock times (ns).
        Times outside the state range are clamped to the ends. Returns {column: [values]}."""
        ts = self.state["t_ns"]
        out = {c: [] for c in columns}
        for t in times_ns:
            j = bisect.bisect_right(ts, t)
            if j <= 0:
                lo = hi = 0
            elif j >= len(ts):
                lo = hi = len(ts) - 1
            else:
                lo, hi = j - 1, j
            w = 0.0 if hi == lo else (t - ts[lo]) / float(ts[hi] - ts[lo])
            for c in columns:
                a, b = self.state[c][lo], self.state[c][hi]
                out[c].append(a + (b - a) * w)
        return out

    def camera_poses(self, camera):
        """Ground-truth pose at each frame's ACQUISITION time (not completion)."""
        return self.align(self.cameras[camera]["acq_ns"], ("px", "py", "pz", "qw", "qx", "qy", "qz"))

    # ---- summary
    def summary(self):
        m = self.manifest
        ts = self.state["t_ns"]
        dur = (ts[-1] - ts[0]) * 1e-9 if len(ts) > 1 else 0.0
        path_len = sum(math.dist((self.state["px"][i], self.state["py"][i], self.state["pz"][i]),
                                 (self.state["px"][i - 1], self.state["py"][i - 1], self.state["pz"][i - 1]))
                       for i in range(1, len(ts)))
        kinds = {}
        for e in self.events:
            kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
        nm = [e["distance_m"] for e in self.events if e["kind"] == "near_miss"]
        return {
            "run_id": m["run_id"], "seed": m["scene_seed"], "split": m.get("split"),
            "duration_s": dur, "state_rows": len(ts),
            "state_hz": (len(ts) - 1) / dur if dur > 0 else 0.0,
            "imu_rows": len(self.imu.get("t_ns", [])),
            "cameras": {k: len(v["acq_ns"]) for k, v in self.cameras.items()},
            "lidars": {k: len(v["acq_ns"]) for k, v in self.lidars.items()},
            "events": kinds, "min_near_miss_m": min(nm) if nm else None,
            "path_length_m": path_len,
            "final_soc": self.state["batt_soc"][-1] if ts else None,
        }

    # ---- validation
    def validate(self):
        """Return a list of problems (empty = valid)."""
        pr = []
        m = self.manifest
        clk = m["clock"]
        fixed, subs = clk["fixed_dt_ns"], clk["substeps"]
        sub_ns = fixed // subs
        s = self.state
        _strict(pr, "state.t_ns", s["t_ns"])
        for i, (k, sb, t) in enumerate(zip(s["tick"], s["sub"], s["t_ns"])):
            if t != k * fixed + sb * sub_ns:
                pr.append("state row %d: t_ns %d != tick*dt+sub*substep" % (i, t))
                break
        if clk["state_hz"] < 200:
            pr.append("manifest state_hz %.1f < 200" % clk["state_hz"])
        ts = s["t_ns"]
        if len(ts) > 1:
            hz = (len(ts) - 1) / ((ts[-1] - ts[0]) * 1e-9)
            if hz < 199.0:
                pr.append("measured state rate %.1f Hz < 200" % hz)
        for c in ("px", "qw", "rpm0", "cmd0", "batt_soc"):
            if c not in s:
                pr.append("state column %s missing" % c)
        for c, v in s.items():
            if c not in ("ax", "ay", "az", "batt_v", "batt_a", "batt_soc") and any(math.isnan(x) for x in v):
                pr.append("NaN in state column %s" % c)
                break
        qn = max((abs(math.sqrt(a * a + b * b + c * c + d * d) - 1) for a, b, c, d in
                  zip(s["qw"], s["qx"], s["qy"], s["qz"])), default=0)
        if qn > 1e-6:
            pr.append("quaternion norm off by %.2e" % qn)
        if self.imu.get("t_ns"):
            _strict(pr, "imu.t_ns", self.imu["t_ns"])
            if m["imu"]["rate_hz"] > 0:
                d = [b - a for a, b in zip(self.imu["t_ns"], self.imu["t_ns"][1:])]
                if d and (max(d) - min(d)) > 1:
                    pr.append("imu period not uniform")
            if self.imu["t_ns"][0] < ts[0] or self.imu["t_ns"][-1] > ts[-1]:
                pr.append("imu outside state time range")
        for name, idx in self.cameras.items():
            _strict(pr, "camera/%s.acq_ns" % name, idx["acq_ns"])
            for i, (a, c) in enumerate(zip(idx["acq_ns"], idx["complete_ns"])):
                if c < a:
                    pr.append("camera %s frame %d completes before acquisition" % (name, i))
                    break
            if idx["acq_ns"] and (idx["acq_ns"][0] < ts[0] or idx["acq_ns"][-1] > ts[-1]):
                pr.append("camera %s acquisition outside state time range" % name)
            for i in range(len(idx["acq_ns"])):
                if not os.path.exists(os.path.join(self.path, idx["file"][i])):
                    pr.append("camera %s missing file %s" % (name, idx["file"][i]))
                    break
            if idx["acq_ns"]:
                w, h, ch, bd, _ = self.camera_frame(name, 0)
                if (w, h) != (int(idx["width"][0]), int(idx["height"][0])):
                    pr.append("camera %s PNG size mismatch" % name)
                if idx["kind"][0] == "Depth" and bd != 16:
                    pr.append("depth %s is not 16-bit" % name)
        for name, idx in self.lidars.items():
            _strict(pr, "lidar/%s.acq_ns" % name, idx["acq_ns"])
            for i in range(len(idx["acq_ns"])):
                pts = self.lidar_scan(name, i)
                if len(pts) != int(idx["points"][i]):
                    pr.append("lidar %s scan %d point count mismatch" % (name, i))
                    break
                tt = [p[4] for p in pts]
                if any(b < a for a, b in zip(tt, tt[1:])):
                    pr.append("lidar %s scan %d per-point time not monotonic" % (name, i))
                    break
        te = [e["t_ns"] for e in self.events]
        if any(b < a for a, b in zip(te, te[1:])):
            pr.append("events not time ordered")
        end = m.get("end")
        if end is None:
            pr.append("manifest has no 'end' record (run did not close)")
        else:
            if end["counts"].get("state") != len(ts):
                pr.append("state count %s != manifest end count %s" % (len(ts), end["counts"].get("state")))
            if ts and ts[-1] > end["end_t_ns"]:
                pr.append("state beyond end_t_ns")
        return pr

    # ---- plot
    def plot_trajectory(self, out_png=None):
        """Top-down trajectory vs obstacles. Returns False (and prints a note) if matplotlib is missing."""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:  # ImportError, or a broken backend
            print("plot skipped (matplotlib unavailable: %s)" % e, file=sys.stderr)
            return False
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(self.state["px"], self.state["py"], "b-", lw=1, label="drone")
        ax.plot(self.state["px"][:1], self.state["py"][:1], "go")
        for o in self.obstacles:
            ax.add_patch(plt.Circle((o["x"], o["y"]), o.get("r", 0.3), color="saddlebrown", alpha=0.6))
        for e in self.events:
            if e["kind"] == "collision":
                i = bisect.bisect_left(self.state["t_ns"], e["t_ns"])
                i = min(i, len(self.state["px"]) - 1)
                ax.plot(self.state["px"][i], self.state["py"][i], "rx", ms=10)
        ax.set_aspect("equal")
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_title(self.manifest["run_id"])
        if out_png:
            fig.savefig(out_png, dpi=120)
        plt.close(fig)
        return True


def _read_csv_str(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        cols = {h: [] for h in head}
        for row in r:
            for h, v in zip(head, row):
                try:
                    cols[h].append(int(v))
                except ValueError:
                    cols[h].append(v)
    return cols


def _strict(problems, name, seq):
    for i in range(1, len(seq)):
        if seq[i] <= seq[i - 1]:
            problems.append("%s not strictly increasing at row %d" % (name, i))
            return


def read_png(path):
    """Minimal decoder for the non-interlaced, filter-0 PNGs the logger writes (8-bit RGB, 16-bit gray)."""
    b = open(path, "rb").read()
    if b[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG: " + path)
    pos, idat, hdr = 8, b"", None
    while pos < len(b):
        n = struct.unpack(">I", b[pos:pos + 4])[0]
        typ, data = b[pos + 4:pos + 8], b[pos + 8:pos + 8 + n]
        if zlib.crc32(typ + data) & 0xFFFFFFFF != struct.unpack(">I", b[pos + 8 + n:pos + 12 + n])[0]:
            raise ValueError("bad chunk CRC in " + path)
        if typ == b"IHDR":
            hdr = struct.unpack(">IIBB", data[:10])
        elif typ == b"IDAT":
            idat += data
        pos += 12 + n
    w, h, depth, ctype = hdr
    ch = {0: 1, 2: 3}[ctype]
    raw = zlib.decompress(idat)
    stride = w * ch * depth // 8
    rows = []
    for y in range(h):
        if raw[y * (stride + 1)] != 0:
            raise ValueError("unsupported PNG filter")
        rows.append(raw[y * (stride + 1) + 1:(y + 1) * (stride + 1)])
    return w, h, ch, depth, b"".join(rows)


def depth_mm(path):
    """Decode a 16-bit depth PNG to a flat list of millimetres (row-major)."""
    w, h, ch, bd, data = read_png(path)
    if bd != 16 or ch != 1:
        raise ValueError("not a 16-bit gray PNG")
    return list(struct.unpack(">%dH" % (w * h), data)), w, h


def main(argv):
    if len(argv) < 2:
        print("usage: drone_log.py RUN_DIR [--plot out.png]")
        return 2
    log = DroneLog(argv[1])
    print(json.dumps(log.summary(), indent=2))
    problems = log.validate()
    for p in problems:
        print("PROBLEM:", p)
    if "--plot" in argv:
        log.plot_trajectory(argv[argv.index("--plot") + 1])
    print("VALID" if not problems else "INVALID")
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
