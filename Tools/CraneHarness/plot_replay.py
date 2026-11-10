# SPDX-License-Identifier: GPL-3.0-or-later
"""Top-down plot of a crane-replay-bundle/1: truth obstacles in the flight band, map belief (occupied voxels mapped
into the world frame), truth and estimated trajectory, route, failure point and the decision at failure.

  python plot_replay.py RUN_DIR/replay_bundle.json [--out PNG]
"""
import argparse
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402


def plot(bundle, out):
    b = bundle
    fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=120)
    tr = b["truth_traj_20hz"]
    end = tr[-1] if tr else b["route"][0]
    half = 7.0
    vox = b.get("voxel_m", 0.2)
    for x, y, _z in b.get("belief_occupied_xy", []):
        ax.add_patch(Rectangle((x - vox / 2, y - vox / 2), vox, vox, color="tab:orange", alpha=0.55, lw=0))
    struck = (b.get("evidence") or {}).get("struck", "")
    sid = int(struck.split("/seg")[1]) if "/seg" in struck else -1
    for c in b.get("truth_capsules", []):
        a, bb, r = c["a"], c["b"], c["r"]
        col = "red" if c["id"] == sid else ("black" if r >= 0.03 else ("dimgray" if r >= 0.01 else "silver"))
        lw = max(0.6, 2 * r * 72 * 8.5 / (2 * half))   # true width in points at this zoom
        ax.plot([a[0], bb[0]], [a[1], bb[1]], color=col, lw=lw, solid_capstyle="round", alpha=0.9, zorder=3)
    rt = b["route"]
    ax.plot([p[0] for p in rt], [p[1] for p in rt], "--", color="tab:green", lw=1, label="mission route", zorder=2)
    ax.plot([p[0] for p in tr], [p[1] for p in tr], "-", color="tab:blue", lw=1.6, label="truth trajectory", zorder=4)
    e = b.get("local_error_end", [0, 0, 0])
    est = b.get("est_traj_local_20hz", [])
    if est:
        ax.plot([p[0] + e[0] for p in est], [p[1] + e[1] for p in est], ":", color="tab:purple", lw=1.2,
                label="estimate (local frame, shifted by end-tick error)", zorder=4)
    ev = b.get("evidence") or {}
    body = ev.get("body_radius_m", 0.268)
    ax.add_patch(plt.Circle((end[0], end[1]), body, fill=False, color="tab:blue", lw=1.5, zorder=5))
    dec = (b.get("decision_at_failure") or {}).get("last_plan") or {}
    if dec:
        h, vc = dec.get("heading", 0.0), dec.get("v_cmd", 0.0)
        L = 0.6 + 0.4 * vc
        ax.annotate("", xy=(end[0] + L * math.cos(h), end[1] + L * math.sin(h)), xytext=(end[0], end[1]),
                    arrowprops=dict(arrowstyle="->", color="tab:red", lw=1.8), zorder=6)
    ax.set_xlim(end[0] - half, end[0] + half)
    ax.set_ylim(end[1] - half, end[1] + half)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("x east [m]")
    ax.set_ylabel("y north [m]")
    title = "%s\n%s -> %s (replay reproduced: %s)" % (b["run_id"], b["outcome"], b.get("label") or "-", b.get("replay_reproduced"))
    ax.set_title(title, fontsize=9)
    txt = []
    if ev.get("t_contact_s") is not None:
        txt.append("contact t=%.2fs v=%.2f m/s, t_need=%.2fs, struck %s D=%.1f cm" % (ev["t_contact_s"], ev.get("impact_speed_mps", 0), ev.get("t_need_s", 0), struck, 100 * (ev.get("struck_diam_m") or 0)))
        txt.append("mapped by t_need: %s, at contact: %s, first view %.2fs, first detect %s" % (
            ev.get("mapped_by_need"), ev.get("mapped_at_contact"), ev.get("first_view_s") or float("nan"),
            "%.2fs" % ev["first_detect_s"] if ev.get("first_detect_s") is not None else "never"))
        txt.append("belief dist at contact %.2f m (body %.2f, r_infl %.2f), local err change %.2f m" % (
            ev.get("belief_dist_at_contact_m") or float("nan"), body, ev.get("r_infl_m") or float("nan"), ev.get("local_err_change_m") or 0))
    if dec:
        txt.append("last plan: v_cmd %.2f heading %.2f d_free %.2f stop_dist %.2f feasible %s/%s tau %.3f %s" % (
            dec.get("v_cmd", 0), dec.get("heading", 0), dec.get("d_free", 0), dec.get("stop_dist", 0), dec.get("feasible"), dec.get("candidates"), dec.get("tau_age", 0), dec.get("note", "")))
    txt.append("band z %.2f-%.2f m; orange: occupied belief voxels; black >= 3 cm, gray 1-3 cm, silver twigs; red: struck" % tuple(b.get("belief_band_m", [0, 0])))
    fig.text(0.01, 0.005, "\n".join(txt), fontsize=6.5, family="monospace", va="bottom")
    ax.legend(loc="upper right", fontsize=7)
    fig.subplots_adjust(bottom=0.17)
    fig.savefig(out)
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    with open(a.bundle) as f:
        b = json.load(f)
    out = a.out or os.path.join(os.path.dirname(a.bundle), "replay_topdown.png")
    print(plot(b, out))


if __name__ == "__main__":
    main()
