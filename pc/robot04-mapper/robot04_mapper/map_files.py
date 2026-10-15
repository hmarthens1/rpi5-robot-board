"""Save an OccupancyGrid in the map_server format (Nav2 can load it), plus a preview and notes.

  <name>.pgm   trinary image: 0 occupied, 254 free, 205 unknown (top row = highest y)
  <name>.yaml  image, resolution, origin, thresholds  (map_server / nav2_map_server)
  <name>.png   preview with the robot's path
  <name>.json  when, robot, size, path length, final pose

The grid is cropped to the explored area (plus a margin) and the origin moved to match.
"""
import json
import math
import os
import time

import numpy as np
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPen

from .map_view import known_bbox

MAPS_DIR = os.path.expanduser("~/Desktop/robot04_maps")


def save_map(grid, name, trail=(), pose=None, robot="robot04", folder=None):
    folder = folder or MAPS_DIR
    os.makedirs(folder, exist_ok=True)
    base = os.path.join(folder, name)
    bbox = known_bbox(grid, margin=10)
    if bbox is None:
        raise ValueError("the map is still empty")
    r0, r1, c0, c1 = bbox
    res = grid.info.resolution
    ox = grid.info.origin.position.x + c0 * res
    oy = grid.info.origin.position.y + r0 * res
    d = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)[r0:r1, c0:c1]

    pgm = np.full(d.shape, 205, np.uint8)
    pgm[(d >= 0) & (d < 25)] = 254
    pgm[d >= 65] = 0
    pgm = np.flipud(pgm)                       # image top = highest y
    h, w = pgm.shape
    with open(base + ".pgm", "wb") as f:
        f.write(b"P5\n# robot04 hector map\n%d %d\n255\n" % (w, h))
        f.write(pgm.tobytes())

    with open(base + ".yaml", "w") as f:
        f.write("image: %s.pgm\n" % name)
        f.write("mode: trinary\n")
        f.write("resolution: %.4f\n" % res)
        f.write("origin: [%.4f, %.4f, 0.0]\n" % (ox, oy))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    # preview: map scaled up, path in blue, final pose in green
    k = max(1, int(round(800 / max(w, h))))
    img = QImage(np.ascontiguousarray(pgm).data, w, h, w, QImage.Format_Grayscale8).copy()
    img = img.scaled(w * k, h * k).convertToFormat(QImage.Format_RGB32)
    p = QPainter(img)
    p.setRenderHint(QPainter.Antialiasing)

    def px(x, y):
        return QPointF((x - ox) / res * k, (h - (y - oy) / res) * k)

    if len(trail) > 1:
        p.setPen(QPen(QColor(40, 110, 255), max(2, k)))
        for a, b in zip(trail, trail[1:]):
            p.drawLine(px(*a), px(*b))
    if pose:
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(30, 180, 70))
        p.drawEllipse(px(pose[0], pose[1]), 3 * k, 3 * k)
    p.end()
    img.save(base + ".png")

    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(trail, trail[1:]))
    known = int((d >= 0).sum())
    meta = {"name": name, "robot": robot, "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
            "slam": "hector_mapping", "resolution_m": res, "size_px": [w, h],
            "size_m": [round(w * res, 2), round(h * res, 2)],
            "explored_m2": round(known * res * res, 1), "path_length_m": round(length, 2),
            "final_pose": [round(v, 3) for v in pose[:3]] if pose else None,
            "files": [name + e for e in (".pgm", ".yaml", ".png", ".json")]}
    with open(base + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return base, meta
