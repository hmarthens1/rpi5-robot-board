#!/usr/bin/env python3
"""Figures and numbers from one exploration run.

  python3 tools/analyze_run.py RUN.jsonl MAP.yaml [--poses POSES.csv] --out DIR

RUN.jsonl  the explorer's log (~/explore_logs on the robot)
MAP.yaml   the saved map (map_server format)
POSES.csv  optional full-rate SLAM poses (t, x, y, yaw), recorded on the PC

Writes DIR/run_map.png (map, path, goals), DIR/run_progress.png (explored area and
distance vs time), DIR/run_summary.json.
"""
import argparse
import json
import math
import os

import numpy as np


def load_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    parts, i = [], 0
    while len(parts) < 4:                      # P5, width, height, maxval (skip comments)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)


def load_map(yaml_path):
    meta = {}
    for line in open(yaml_path):
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    img = load_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"]))
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    return img, res, ox, oy


def path_length(xy, step=0.05):
    """Length of a pose track, counting only moves larger than `step` (removes SLAM jitter)."""
    total, last = 0.0, None
    for x, y in xy:
        if last is None:
            last = (x, y)
            continue
        d = math.hypot(x - last[0], y - last[1])
        if d >= step:
            total += d
            last = (x, y)
    return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("map")
    ap.add_argument("--poses")
    ap.add_argument("--out", default=".")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    ev = [json.loads(l) for l in open(a.run) if l.strip()]
    t0 = next(e["t"] for e in ev if e["event"] == "start")
    end = next((e for e in ev if e["event"] == "done"), ev[-1])
    samples = [e for e in ev if e["event"] == "sample" and e.get("explored_m2") is not None]
    sent = [e for e in ev if e["event"] == "goal_sent"]
    reached = [e for e in ev if e["event"] == "goal_reached" and e.get("kind") == "frontier"]
    explored = [e for e in ev if e["event"] == "frontier_explored"]
    failed = [e for e in ev if e["event"] == "goal_failed"]
    pauses = [e for e in ev if e["event"] == "pause"]
    home = next((e for e in ev if e["event"] == "return_home"), None)

    track = [tuple(e["pose"][:2]) for e in ev if e.get("pose")]
    poses = None
    if a.poses and os.path.exists(a.poses):
        poses = np.loadtxt(a.poses, delimiter=",", ndmin=2)
        poses = poses[(poses[:, 0] >= t0) & (poses[:, 0] <= end["t"] + 1)]
    # full-rate poses where recorded, the explorer's own 5 s samples before that
    timed = [(e["t"], tuple(e["pose"][:2])) for e in ev if e.get("pose") and e["t"] >= t0]
    if poses is not None and len(poses) > 10:
        path_xy = [xy for t, xy in timed if t < poses[0, 0]] + [tuple(p) for p in poses[:, 1:3]]
    else:
        path_xy = track
    begin = next((e for e in ev if e["event"] == "begin"), None)
    start_xy = tuple(begin["home"]) if begin and begin.get("home") else path_xy[0]

    img, res, ox, oy = load_map(a.map)
    free_m2 = float((img == 254).sum()) * res * res
    occ_m2 = float((img == 0).sum()) * res * res
    search_ms = [e["search_ms"] for e in sent if "search_ms" in e]
    summary = {
        "duration_min": round((end["t"] - t0) / 60.0, 2),
        "explored_m2_final": samples[-1]["explored_m2"] if samples else None,
        "map_free_m2": round(free_m2, 1),
        "map_occupied_m2": round(occ_m2, 1),
        "map_size_m": [round(img.shape[1] * res, 2), round(img.shape[0] * res, 2)],
        "distance_logged_m": round(end.get("distance_m", 0.0), 1),
        "path_length_m": round(path_length(path_xy), 1),
        "goals_sent": len(sent), "frontiers_reached": len(reached), "frontiers_seen_on_the_way": len(explored),
        "goals_failed": len(failed), "failures": [f.get("why") for f in failed],
        "operator_pauses": len(pauses),
        "returned_home": home is not None,
        "end_reason": end.get("why") or end["event"],
        "search_ms_mean": round(float(np.mean(search_ms)), 1) if search_ms else None,
        "search_ms_max": round(float(np.max(search_ms)), 1) if search_ms else None,
        "candidates_mean": round(float(np.mean([e["candidates"] for e in sent])), 1) if sent else None,
        "battery_v": [samples[0].get("battery_v"), samples[-1].get("battery_v")] if samples else None,
    }
    if samples and summary["duration_min"] > 0:
        summary["area_rate_m2_per_min"] = round(summary["explored_m2_final"] / summary["duration_min"], 2)
    with open(os.path.join(a.out, "run_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 9})

    # map + path + goals
    h, w = img.shape
    fig, ax = plt.subplots(figsize=(6.2, 6.2 * h / w if w else 6))
    shade = np.full(img.shape, 0.78)
    shade[img == 254] = 1.0
    shade[img == 0] = 0.0
    ax.imshow(shade, cmap="gray", vmin=0, vmax=1, extent=(ox, ox + w * res, oy, oy + h * res))
    if len(path_xy) > 1:
        px, py = zip(*path_xy)
        ax.plot(px, py, "-", color="#1f5fd6", lw=1.0, label="robot path (SLAM)")
        ax.plot(start_xy[0], start_xy[1], "s", color="#e6a700", ms=7, label="start", zorder=5)
    gx = [e["goal"][0] for e in sent]
    gy = [e["goal"][1] for e in sent]
    ax.plot(gx, gy, "x", color="#c2185b", ms=5, mew=1.2, label="frontier goals")
    if failed:
        ax.plot([e["goal"][0] for e in failed], [e["goal"][1] for e in failed], "o", mfc="none",
                color="#d32f2f", ms=8, label="failed goals")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="best", fontsize=7, framealpha=0.9)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "run_map.png"), dpi=200)
    plt.close(fig)

    # progress
    if samples:
        tt = [(e["t"] - t0) / 60.0 for e in samples]
        fig, ax1 = plt.subplots(figsize=(6.2, 2.6))
        ax1.plot(tt, [e["explored_m2"] for e in samples], color="#1f5fd6", label="mapped area")
        ax1.set_xlabel("time (min)")
        ax1.set_ylabel("mapped area (m$^2$)", color="#1f5fd6")
        ax2 = ax1.twinx()
        ax2.plot(tt, [e["frontiers"] for e in samples], color="#c2185b", lw=0.9, label="frontiers")
        ax2.set_ylabel("open frontiers", color="#c2185b")
        for e in sent:
            ax1.axvline((e["t"] - t0) / 60.0, color="k", alpha=0.08, lw=0.8)
        ax1.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "run_progress.png"), dpi=200)
        plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
