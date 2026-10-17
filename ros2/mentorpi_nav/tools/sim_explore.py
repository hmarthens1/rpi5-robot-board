#!/usr/bin/env python3
"""Offline check of the frontier explorer: a synthetic apartment, a ray-cast 360-degree
lidar (MS200-like: 450 rays, 8 m), an occupancy grid built from the rays, and a robot
that drives the geodesic path to each frontier goal chosen by frontiers.find_frontiers.
No SLAM error and no controller: this tests frontier choice and termination only.

  python3 tools/sim_explore.py [--out DIR] [--seed N]

Writes DIR/sim_explore.json (coverage vs distance, goals) and DIR/sim_explore.png.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mentorpi_nav.frontiers import Grid, find_frontiers, geodesic, nearest_true  # noqa: E402

RES = 0.05


def apartment():
    """True = wall. 10 m x 7 m: living room, hallway, two bedrooms, bathroom, kitchen island."""
    w, h = int(10 / RES), int(7 / RES)
    m = np.zeros((h, w), bool)

    def wall(x0, y0, x1, y1, t=0.1):
        r0, r1 = int(min(y0, y1) / RES), int((max(y0, y1) + t) / RES)
        c0, c1 = int(min(x0, x1) / RES), int((max(x0, x1) + t) / RES)
        m[r0:r1, c0:c1] = True

    def door(x0, y0, x1, y1):
        r0, r1 = int(min(y0, y1) / RES), int(max(y0, y1) / RES) + 2
        c0, c1 = int(min(x0, x1) / RES), int(max(x0, x1) / RES) + 2
        m[r0:r1, c0:c1] = False

    wall(0, 0, 10, 0); wall(0, 6.9, 10, 6.9); wall(0, 0, 0, 7); wall(9.9, 0, 9.9, 7)
    wall(5, 0, 5, 7)                          # living | bedrooms
    door(5, 1.0, 5, 1.85)                     # living -> hallway door
    wall(5, 3, 10, 3)                         # bedroom 1 | hallway
    door(6.0, 3, 6.85, 3)
    wall(5, 4.2, 10, 4.2)                     # hallway | bedroom 2 + bath
    door(6.0, 4.2, 6.85, 4.2)
    door(8.6, 4.2, 9.4, 4.2)
    wall(8.2, 4.2, 8.2, 7)                    # bedroom 2 | bathroom
    m[int(2.5 / RES):int(3.3 / RES), int(1.5 / RES):int(3.0 / RES)] = True   # kitchen island
    m[int(5.5 / RES):int(5.8 / RES), int(0.6 / RES):int(2.2 / RES)] = True   # sofa
    m[int(1.0 / RES):int(1.1 / RES), int(5.0 / RES):int(5.1 / RES)] = True   # doorframe stub
    wall(5, 3, 5.1, 4.2)
    door(5, 3.3, 5, 4.1)                      # hallway also opens to the living room
    return m


def scan_update(world, occ, x, y, rays=450, rng=8.0):
    """Ray-cast from (x, y); mark free along each ray and occupied at the hit (like a fresh scan)."""
    h, w = world.shape
    for a in np.linspace(0, 2 * math.pi, rays, endpoint=False):
        dx, dy = math.cos(a), math.sin(a)
        d = 0.0
        while d < rng:
            px, py = x + d * dx, y + d * dy
            c, r = int(px / RES), int(py / RES)
            if not (0 <= r < h and 0 <= c < w):
                break
            if world[r, c]:
                occ[r, c] = 100
                break
            if occ[r, c] != 100:
                occ[r, c] = 0
            d += RES * 0.5


def run(out, seed=0):
    rng = np.random.default_rng(seed)
    world = apartment()
    occ = np.full(world.shape, -1, np.int8)
    x, y = 1.0, 1.0
    free_truth = (~world).sum()
    log = {"samples": [], "goals": []}
    dist = 0.0
    path_xy = [(x, y)]
    t0 = time.time()
    for step in range(200):
        scan_update(world, occ, x, y)
        g = Grid(occ, RES, 0.0, 0.0)
        known_free = int((occ == 0).sum())
        log["samples"].append({"distance_m": round(dist, 2), "coverage": round(known_free / free_truth, 4)})
        t = time.perf_counter()
        fr, info = find_frontiers(g, (x, y), robot_radius=0.15, min_size_m=0.30, size_weight=0.5)
        ms = (time.perf_counter() - t) * 1000
        if not fr:
            break
        f = fr[0]
        log["goals"].append({"goal": [round(f.x, 2), round(f.y, 2)], "path_m": round(f.distance_m, 2),
                             "size_m": round(f.size_m, 2), "candidates": len(fr), "search_ms": round(ms, 1)})
        # drive the geodesic path toward the goal, re-scanning every 0.5 m (like the real loop)
        trav = info["traversable"]
        r0, c0 = info["offset"]
        start = nearest_true(trav, (int(y / RES) - r0, int(x / RES) - c0), 8)
        goal = (int(f.y / RES) - r0, int(f.x / RES) - c0)
        dmap = geodesic(trav, goal)
        cur = start
        walked = 0.0
        while cur != goal and walked < 0.5 + 1e-9:
            best = cur
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = cur[0] + dr, cur[1] + dc
                    if 0 <= rr < dmap.shape[0] and 0 <= cc < dmap.shape[1] and dmap[rr, cc] < dmap[best]:
                        best = (rr, cc)
            if best == cur:
                break
            walked += math.hypot(best[0] - cur[0], best[1] - cur[1]) * RES
            cur = best
        nx, ny = (cur[1] + c0 + 0.5) * RES, (cur[0] + r0 + 0.5) * RES
        dist += math.hypot(nx - x, ny - y) if walked == 0 else walked
        x, y = nx + rng.normal(0, 0.005), ny + rng.normal(0, 0.005)
        path_xy.append((x, y))
    known_free = int((occ == 0).sum())
    log["final"] = {"coverage": round(known_free / free_truth, 4), "distance_m": round(dist, 2),
                    "goals": len(log["goals"]), "steps": step + 1, "wall_s": round(time.time() - t0, 1),
                    "free_area_m2": round(free_truth * RES * RES, 1),
                    "mean_search_ms": round(float(np.mean([g["search_ms"] for g in log["goals"]])), 1)}
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "sim_explore.json"), "w") as f:
        json.dump(log, f, indent=1)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
        img = np.full(occ.shape, 0.55)
        img[occ == 0] = 1.0
        img[occ == 100] = 0.0
        ax[0].imshow(img, cmap="gray", origin="lower", extent=(0, occ.shape[1] * RES, 0, occ.shape[0] * RES))
        px, py = zip(*path_xy)
        ax[0].plot(px, py, "-", color="#1f5fd6", lw=1.5, label="robot path")
        ax[0].plot(px[0], py[0], "s", color="#e6a700", ms=8, label="start")
        ax[0].set_title("Simulated apartment: final map")
        ax[0].set_xlabel("x (m)")
        ax[0].set_ylabel("y (m)")
        ax[0].legend(loc="upper right", fontsize=8)
        d = [s["distance_m"] for s in log["samples"]]
        c = [100 * s["coverage"] for s in log["samples"]]
        ax[1].plot(d, c, color="#1f5fd6")
        ax[1].set_xlabel("distance driven (m)")
        ax[1].set_ylabel("free space mapped (%)")
        ax[1].set_ylim(0, 102)
        ax[1].grid(alpha=0.3)
        ax[1].set_title("Coverage vs. distance")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "sim_explore.png"), dpi=150)
    except ImportError:
        pass
    return log["final"]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    print(json.dumps(run(a.out, a.seed)))
