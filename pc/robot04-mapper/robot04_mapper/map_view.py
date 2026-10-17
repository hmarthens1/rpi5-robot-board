"""Map panel: the SLAM occupancy grid, the robot, its path and the live scan; while the
robot explores on its own, also Nav2's planned path (green), the goal (magenta) and the
frontiers the explorer knows about (cyan, size = frontier length).

Drawn in world metres with a y-up transform, so OccupancyGrid row 0 (the map origin)
lands at the bottom without flipping the data. Wheel = zoom, drag = pan, double-click =
follow the robot again.
"""
import math

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPolygonF, QTransform
from PyQt5.QtWidgets import QWidget

# occupancy value -> grey level (unknown / free / occupied)
UNKNOWN, FREE, OCCUPIED = 70, 235, 25


def grid_to_gray(grid):
    """OccupancyGrid -> uint8 array (rows = y from the origin up), map_server style colours."""
    d = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    g = np.full(d.shape, UNKNOWN, np.uint8)
    g[(d >= 0) & (d <= 25)] = FREE
    g[d >= 65] = OCCUPIED
    between = (d > 25) & (d < 65)
    g[between] = 128
    return g


def known_bbox(grid, margin=10):
    """(r0, r1, c0, c1) of the known cells plus a margin, or None if nothing is known yet."""
    d = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    rows = np.where((d >= 0).any(axis=1))[0]
    cols = np.where((d >= 0).any(axis=0))[0]
    if rows.size == 0:
        return None
    h, w = d.shape
    return (max(rows[0] - margin, 0), min(rows[-1] + margin + 1, h),
            max(cols[0] - margin, 0), min(cols[-1] + margin + 1, w))


class MapView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 420)
        self.scale = 60.0              # pixels per metre
        self.center = QPointF(0, 0)    # world point at the widget centre
        self.follow = True
        self.drag = None
        self.image = None              # (QImage, world QRectF)
        self.image_seq = -1
        self.pose = None               # (x, y, yaw)
        self.trail = []
        self.scan_pts = []
        self.plan = []                 # [(x, y)] Nav2 global path
        self.goal = None               # (x, y)
        self.frontiers = []            # [(x, y, size_m)]
        self.home = None
        self.message = "SLAM not running: press Start SLAM"

    # ------------------------------------------------------------------ data
    def set_map(self, grid, seq):
        if seq == self.image_seq:
            return
        self.image_seq = seq
        if grid is None:
            self.image = None
            return
        bbox = known_bbox(grid, margin=4)
        if bbox is None:
            self.image = None
            return
        r0, r1, c0, c1 = bbox
        g = np.ascontiguousarray(grid_to_gray(grid)[r0:r1, c0:c1])
        img = QImage(g.data, g.shape[1], g.shape[0], g.shape[1], QImage.Format_Grayscale8).copy()
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        self.image = (img, QRectF(ox + c0 * res, oy + r0 * res, (c1 - c0) * res, (r1 - r0) * res))

    def set_robot(self, pose, trail, scan):
        self.pose, self.trail = pose, trail
        self.scan_pts = []
        if pose is not None and scan is not None:
            x, y, yaw = pose
            a = scan.angle_min
            for r in scan.ranges[::2]:
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    self.scan_pts.append(QPointF(x + r * math.cos(yaw + a), y + r * math.sin(yaw + a)))
                a += 2 * scan.angle_increment
        if self.follow and pose is not None:
            self.center = QPointF(pose[0], pose[1])
        self.update()

    def set_explore(self, plan, goal, frontiers, home):
        self.plan, self.goal, self.frontiers, self.home = plan or [], goal, frontiers or [], home

    # ------------------------------------------------------------------ drawing
    def world_transform(self):
        t = QTransform()
        t.translate(self.width() / 2, self.height() / 2)
        t.scale(self.scale, -self.scale)
        t.translate(-self.center.x(), -self.center.y())
        return t

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(UNKNOWN, UNKNOWN, UNKNOWN))
        p.setRenderHint(QPainter.Antialiasing)
        p.setTransform(self.world_transform())
        if self.image:
            img, rect = self.image
            p.drawImage(rect, img)     # image row 0 -> rect.top() = lowest y = bottom of the screen
        self.draw_grid(p)
        if len(self.trail) > 1:
            p.setPen(self.cosmetic(QColor(60, 140, 255), 2))
            p.drawPolyline(QPolygonF([QPointF(x, y) for x, y in self.trail]))
        for fx, fy, size in self.frontiers:
            r = min(0.25, 0.05 + 0.1 * size)
            p.setPen(self.cosmetic(QColor(0, 200, 230), 2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(fx, fy), r, r)
        if len(self.plan) > 1:
            p.setPen(self.cosmetic(QColor(30, 200, 60), 3))
            p.drawPolyline(QPolygonF([QPointF(x, y) for x, y in self.plan]))
        if self.goal:
            gx, gy = self.goal
            p.setPen(self.cosmetic(QColor(230, 0, 200), 3))
            p.drawLine(QPointF(gx - 0.12, gy - 0.12), QPointF(gx + 0.12, gy + 0.12))
            p.drawLine(QPointF(gx - 0.12, gy + 0.12), QPointF(gx + 0.12, gy - 0.12))
        if self.home:
            p.setPen(self.cosmetic(QColor(255, 200, 0), 2))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(self.home[0] - 0.1, self.home[1] - 0.1, 0.2, 0.2))
        if self.scan_pts:
            p.setPen(self.cosmetic(QColor(230, 40, 40), 3))
            p.drawPoints(QPolygonF(self.scan_pts))
        if self.pose:
            x, y, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            pts = [(0.18, 0), (-0.10, 0.09), (-0.10, -0.09)]
            poly = QPolygonF([QPointF(x + px * c - py * s, y + px * s + py * c) for px, py in pts])
            p.setPen(self.cosmetic(QColor(0, 0, 0), 1))
            p.setBrush(QBrush(QColor(40, 200, 80)))
            p.drawPolygon(poly)
        p.resetTransform()
        p.setPen(QColor(255, 255, 255))
        self.draw_scale_bar(p)
        if self.message:
            p.drawText(self.rect().adjusted(10, 10, -10, -10), Qt.AlignTop | Qt.AlignLeft, self.message)
        p.drawText(self.rect().adjusted(10, 10, -10, -10), Qt.AlignBottom | Qt.AlignRight,
                   "wheel: zoom   drag: pan   double-click: follow robot" + ("  [following]" if self.follow else ""))

    @staticmethod
    def cosmetic(color, width):
        pen = QPen(color, width)
        pen.setCosmetic(True)
        return pen

    def draw_grid(self, p):
        """1 m grid lines around the view."""
        inv, _ = self.world_transform().inverted()
        tl, br = inv.map(QPointF(0, 0)), inv.map(QPointF(self.width(), self.height()))
        p.setPen(self.cosmetic(QColor(255, 255, 255, 30), 1))
        for gx in range(math.floor(min(tl.x(), br.x())), math.ceil(max(tl.x(), br.x())) + 1):
            p.drawLine(QPointF(gx, tl.y()), QPointF(gx, br.y()))
        for gy in range(math.floor(min(tl.y(), br.y())), math.ceil(max(tl.y(), br.y())) + 1):
            p.drawLine(QPointF(tl.x(), gy), QPointF(br.x(), gy))

    def draw_scale_bar(self, p):
        x0, y0 = 14, self.height() - 34
        p.drawLine(x0, y0, x0 + int(self.scale), y0)
        p.drawLine(x0, y0 - 4, x0, y0 + 4)
        p.drawLine(x0 + int(self.scale), y0 - 4, x0 + int(self.scale), y0 + 4)
        p.drawText(x0, y0 - 6, "1 m")

    # ------------------------------------------------------------------ mouse
    def wheelEvent(self, e):
        f = 1.2 if e.angleDelta().y() > 0 else 1 / 1.2
        self.scale = min(400.0, max(5.0, self.scale * f))
        self.update()

    def mousePressEvent(self, e):
        self.drag = (e.pos(), QPointF(self.center))

    def mouseMoveEvent(self, e):
        if self.drag:
            start, c = self.drag
            d = e.pos() - start
            self.follow = False
            self.center = QPointF(c.x() - d.x() / self.scale, c.y() + d.y() / self.scale)
            self.update()

    def mouseReleaseEvent(self, _):
        self.drag = None

    def mouseDoubleClickEvent(self, _):
        self.follow = True
        if self.pose:
            self.center = QPointF(self.pose[0], self.pose[1])
        self.update()
