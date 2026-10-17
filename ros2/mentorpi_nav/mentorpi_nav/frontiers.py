"""Frontier detection on an occupancy grid (numpy + scipy, no ROS), so it can be tested offline.

A frontier cell is a free cell with an unknown 4-neighbour: the edge of what the lidar has
seen. Frontier cells are grouped into 8-connected clusters; a cluster is a place to drive
to. The robot can only drive where the clearance to the nearest occupied cell is at least
its radius, so each cluster gets a goal on that traversable set: the reachable cell closest
to the cluster's centroid among those within `approach_m` of the cluster. Clusters are
ranked by the geodesic distance from the robot (Dijkstra over the traversable cells),
minus a (capped) bonus for size, which is what a greedy nearest-frontier explorer
(Yamauchi 1997) does with a tie-break for the larger unexplored opening.

Grid convention is nav_msgs/OccupancyGrid: row-major, row 0 at the origin (lowest y),
-1 unknown, 0..100 occupancy probability.
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

FREE_MAX = 25       # <= this is free (hector writes 0)
OCC_MIN = 65        # >= this is occupied (hector writes 100)


@dataclass
class Grid:
    data: np.ndarray        # int8 (h, w)
    resolution: float
    origin_x: float
    origin_y: float

    @classmethod
    def from_msg(cls, msg):
        d = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return cls(d, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)

    def to_cell(self, x, y):
        return int(math.floor((y - self.origin_y) / self.resolution)), int(math.floor((x - self.origin_x) / self.resolution))

    def to_world(self, r, c):
        return self.origin_x + (c + 0.5) * self.resolution, self.origin_y + (r + 0.5) * self.resolution


@dataclass
class Frontier:
    x: float                # goal (traversable cell next to the frontier)
    y: float
    cx: float               # centroid of the frontier cells
    cy: float
    size_m: float           # length of the frontier (cells x resolution)
    distance_m: float       # geodesic distance from the robot to the goal
    score: float


def masks(grid):
    d = grid.data
    unknown = d < 0
    occupied = d >= OCC_MIN
    free = (d >= 0) & (d <= FREE_MAX)
    return free, occupied, unknown


def clearance_m(grid, occupied):
    """Distance from each cell to the nearest occupied cell, metres."""
    if not occupied.any():
        return np.full(occupied.shape, np.inf)
    return ndimage.distance_transform_edt(~occupied) * grid.resolution


def frontier_mask(free, unknown):
    u = np.zeros_like(unknown)
    u[1:, :] |= unknown[:-1, :]
    u[:-1, :] |= unknown[1:, :]
    u[:, 1:] |= unknown[:, :-1]
    u[:, :-1] |= unknown[:, 1:]
    return free & u


def geodesic(traversable, start, max_cells=400000):
    """Dijkstra (8-connected, cell units) from start over traversable. inf where unreached."""
    h, w = traversable.shape
    dist = np.full((h, w), np.inf)
    r0, c0 = start
    if not (0 <= r0 < h and 0 <= c0 < w) or not traversable[r0, c0]:
        return dist
    dist[r0, c0] = 0.0
    heap = [(0.0, r0, c0)]
    steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
    n = 0
    while heap and n < max_cells:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        n += 1
        for dr, dc, k in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and traversable[rr, cc]:
                nd = d + k
                if nd < dist[rr, cc]:
                    dist[rr, cc] = nd
                    heapq.heappush(heap, (nd, rr, cc))
    return dist


def nearest_true(mask, cell, radius):
    """Closest True cell of mask within radius cells of cell, or None."""
    r, c = cell
    h, w = mask.shape
    r0, r1, c0, c1 = max(r - radius, 0), min(r + radius + 1, h), max(c - radius, 0), min(c + radius + 1, w)
    sub = mask[r0:r1, c0:c1]
    if not sub.any():
        return None
    rr, cc = np.nonzero(sub)
    i = np.argmin((rr + r0 - r) ** 2 + (cc + c0 - c) ** 2)
    return int(rr[i] + r0), int(cc[i] + c0)


def find_frontiers(grid, robot_xy, robot_radius=0.15, min_size_m=0.25, approach_m=0.35,
                   size_weight=0.5, size_cap_m=1.5, blacklist=(), blacklist_radius=0.4, crop_margin=20):
    """Ranked list of reachable frontiers (best first) and a dict of debug arrays."""
    # work on the known part of the map only (a 1024^2 grid is mostly unknown)
    known = grid.data >= 0
    rows, cols = np.nonzero(known.any(axis=1))[0], np.nonzero(known.any(axis=0))[0]
    if rows.size == 0:
        return [], {}
    h, w = grid.data.shape
    r0, r1 = max(rows[0] - crop_margin, 0), min(rows[-1] + crop_margin + 1, h)
    c0, c1 = max(cols[0] - crop_margin, 0), min(cols[-1] + crop_margin + 1, w)
    sub = Grid(grid.data[r0:r1, c0:c1], grid.resolution,
               grid.origin_x + c0 * grid.resolution, grid.origin_y + r0 * grid.resolution)

    free, occupied, unknown = masks(sub)
    clear = clearance_m(sub, occupied)
    traversable = free & (clear >= robot_radius)
    start = nearest_true(traversable, sub.to_cell(*robot_xy), int(round(0.4 / sub.resolution)))
    if start is None:
        return [], {"reason": "robot not on traversable space"}
    dist = geodesic(traversable, start)

    fmask = frontier_mask(free, unknown)
    labels, n = ndimage.label(fmask, structure=np.ones((3, 3), bool))
    if n == 0:
        return [], {"traversable": traversable, "dist": dist}
    reached = np.isfinite(dist)
    approach = int(round(approach_m / sub.resolution))
    min_cells = max(1, int(round(min_size_m / sub.resolution)))
    out = []
    for i, sl in enumerate(ndimage.find_objects(labels), start=1):
        cells = labels[sl] == i
        size = int(cells.sum())
        if size < min_cells:
            continue
        rr, cc = np.nonzero(cells)
        rr, cc = rr + sl[0].start, cc + sl[1].start
        cr, cc_ = rr.mean(), cc.mean()
        # reachable cells within `approach` of any frontier cell of this cluster
        a0, a1 = max(sl[0].start - approach, 0), min(sl[0].stop + approach, reached.shape[0])
        b0, b1 = max(sl[1].start - approach, 0), min(sl[1].stop + approach, reached.shape[1])
        near = np.zeros((a1 - a0, b1 - b0), bool)
        near[rr - a0, cc - b0] = True
        near = ndimage.binary_dilation(near, iterations=approach) & reached[a0:a1, b0:b1]
        if not near.any():
            continue
        gr, gc = np.nonzero(near)
        gr, gc = gr + a0, gc + b0
        k = np.argmin((gr - cr) ** 2 + (gc - cc_) ** 2)
        goal = (int(gr[k]), int(gc[k]))
        gx, gy = sub.to_world(*goal)
        if any(math.hypot(gx - bx, gy - by) < blacklist_radius for bx, by in blacklist):
            continue
        d_m = dist[goal] * sub.resolution
        size_m = size * sub.resolution
        cx, cy = sub.to_world(cr, cc_)
        # the bonus is capped: the ragged edge of a long scan can join into one huge cluster
        out.append(Frontier(gx, gy, cx, cy, size_m, d_m, d_m - size_weight * min(size_m, size_cap_m)))
    out.sort(key=lambda f: f.score)
    return out, {"traversable": traversable, "dist": dist, "frontier": fmask, "offset": (r0, c0)}


def explored_area_m2(grid):
    return float((grid.data >= 0).sum()) * grid.resolution ** 2
