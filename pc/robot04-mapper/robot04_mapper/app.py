"""robot04 mapper: drive the robot from the camera view, run Hector SLAM, save maps.

Keys (anywhere except the map-name box):  W/↑ forward  S/↓ back  A/← left  D/→ right
Space = stop.  Motion only while a key or button is held; the robot also stops by itself
0.5 s after the last command (robot_node cmd_vel_timeout) and refuses to drive into an
obstacle closer than 0.25 m (lidar).
"""
import math
import os
import signal
import subprocess
import time

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                             QMainWindow, QMessageBox, QPushButton, QSlider, QSplitter, QVBoxLayout, QWidget)

from .bridge import Bridge
from .map_files import MAPS_DIR, save_map
from .map_view import MapView

KEYS = {Qt.Key_W: "f", Qt.Key_Up: "f", Qt.Key_S: "b", Qt.Key_Down: "b",
        Qt.Key_A: "l", Qt.Key_Left: "l", Qt.Key_D: "r", Qt.Key_Right: "r"}
MIN_CLEARANCE = 0.25        # robot_node's min_clearance: closer than this it refuses to drive
FRONT_HALF_ANGLE = math.radians(25)


def clearance(scan):
    """(front, back) nearest range in m, the same sectors robot_node checks."""
    front = back = math.inf
    a = scan.angle_min
    for r in scan.ranges:
        if scan.range_min < r < scan.range_max:
            ang = math.atan2(math.sin(a), math.cos(a))
            if abs(ang) <= FRONT_HALF_ANGLE:
                front = min(front, r)
            elif abs(ang) >= math.pi - FRONT_HALF_ANGLE:
                back = min(back, r)
        a += scan.angle_increment
    return front, back


def no_focus(w):
    w.setFocusPolicy(Qt.NoFocus)
    return w


class MapperWindow(QMainWindow):
    def __init__(self, robot="robot04"):
        super().__init__()
        self.robot = robot
        self.setWindowTitle("%s mapper" % robot)
        self.bridge = Bridge(robot)
        self.held = set()             # teleop directions held: f b l r
        self.moving = False
        self.slam = None              # subprocess.Popen of the hector launch
        self.slam_log = None
        self.rviz = None
        self.saved_seq = None         # map_seq at the last save
        self.last_image_t = None
        self.build_ui()

        self.drive_timer = QTimer(self, timeout=self.drive_tick)
        self.drive_timer.start(100)               # 10 Hz cmd_vel while a key is held
        self.ui_timer = QTimer(self, timeout=self.refresh)
        self.ui_timer.start(50)
        QApplication.instance().installEventFilter(self)

    # ================================================================== layout
    def build_ui(self):
        self.video = QLabel("waiting for video…")
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(480, 300)
        self.video.setStyleSheet("background:#111; color:#aaa;")
        self.map_view = MapView()

        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self.video, 1)
        cam_row = QHBoxLayout()
        self.depth_box = no_focus(QCheckBox("depth view"))
        self.depth_box.toggled.connect(lambda on: self.bridge.set_image_mode("depth" if on else "rgb"))
        self.video_info = QLabel("")
        cam_row.addWidget(self.depth_box)
        cam_row.addStretch(1)
        cam_row.addWidget(self.video_info)
        lv.addLayout(cam_row)
        lv.addWidget(self.teleop_box())
        split.addWidget(left)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self.map_view, 1)
        row = QHBoxLayout()
        row.addWidget(self.slam_box(), 1)
        row.addWidget(self.save_box(), 1)
        rv.addLayout(row)
        split.addWidget(right)
        split.setSizes([620, 760])

        self.status = QLabel("")
        self.status.setStyleSheet("padding:4px; font-family:monospace;")
        root = QWidget()
        v = QVBoxLayout(root)
        v.addWidget(split, 1)
        v.addWidget(self.status)
        self.setCentralWidget(root)
        self.resize(1400, 860)

    def teleop_box(self):
        box = QGroupBox("Drive  (hold W A S D / arrows, Space = stop)")
        g = QGridLayout(box)
        big = QFont()
        big.setPointSize(16)

        def hold_button(text, d):
            b = no_focus(QPushButton(text))
            b.setFont(big)
            b.setMinimumSize(70, 48)
            b.pressed.connect(lambda: self.press(d))
            b.released.connect(lambda: self.release(d))
            return b

        g.addWidget(hold_button("▲", "f"), 0, 1)
        g.addWidget(hold_button("◀", "l"), 1, 0)
        g.addWidget(hold_button("▶", "r"), 1, 2)
        g.addWidget(hold_button("▼", "b"), 2, 1)
        stop = no_focus(QPushButton("STOP"))
        stop.setFont(big)
        stop.setMinimumSize(70, 48)
        stop.setStyleSheet("background:#c62828; color:white; font-weight:bold;")
        stop.clicked.connect(self.stop_robot)
        g.addWidget(stop, 1, 1)

        self.speed = self.slider(40)
        self.turn = self.slider(30)
        self.speed_label, self.turn_label = QLabel(), QLabel()
        g.addWidget(QLabel("speed"), 0, 3)
        g.addWidget(self.speed, 0, 4)
        g.addWidget(self.speed_label, 0, 5)
        g.addWidget(QLabel("turn"), 1, 3)
        g.addWidget(self.turn, 1, 4)
        g.addWidget(self.turn_label, 1, 5)
        hint = QLabel("For mapping: drive slowly, turn slowly (Hector loses\n"
                      "track on fast turns). % of the robot's max speed.")
        hint.setStyleSheet("color:#888;")
        g.addWidget(hint, 2, 3, 1, 3)
        g.setColumnStretch(4, 1)
        return box

    def slider(self, value):
        s = no_focus(QSlider(Qt.Horizontal))
        s.setRange(10, 100)
        s.setValue(value)
        return s

    def slam_box(self):
        box = QGroupBox("SLAM (Hector, on this PC)")
        v = QVBoxLayout(box)
        self.slam_button = no_focus(QPushButton("▶ Start SLAM"))
        self.slam_button.setMinimumHeight(40)
        self.slam_button.clicked.connect(self.toggle_slam)
        row = QHBoxLayout()
        reset = no_focus(QPushButton("New map (reset)"))
        reset.clicked.connect(self.reset_map)
        rviz = no_focus(QPushButton("Open RViz"))
        rviz.clicked.connect(self.open_rviz)
        row.addWidget(reset)
        row.addWidget(rviz)
        self.slam_state = QLabel("stopped")
        v.addWidget(self.slam_button)
        v.addLayout(row)
        v.addWidget(self.slam_state)
        return box

    def save_box(self):
        box = QGroupBox("Save map → " + MAPS_DIR.replace(os.path.expanduser("~"), "~"))
        v = QVBoxLayout(box)
        self.name_edit = QLineEdit(self.default_name())
        self.name_edit.returnPressed.connect(self.save)
        row = QHBoxLayout()
        save = no_focus(QPushButton("💾 Save map"))
        save.setMinimumHeight(34)
        save.clicked.connect(self.save)
        folder = no_focus(QPushButton("Open folder"))
        folder.clicked.connect(self.open_folder)
        row.addWidget(save)
        row.addWidget(folder)
        self.save_info = QLabel("pgm + yaml (Nav2 map_server), png preview, json notes")
        self.save_info.setStyleSheet("color:#888;")
        self.save_info.setWordWrap(True)
        v.addWidget(self.name_edit)
        v.addLayout(row)
        v.addWidget(self.save_info)
        return box

    @staticmethod
    def default_name():
        return time.strftime("lab_%Y-%m-%d_%H%M")

    # ================================================================== teleop
    def eventFilter(self, obj, e):
        if e.type() in (QEvent.KeyPress, QEvent.KeyRelease) and not isinstance(QApplication.focusWidget(), QLineEdit):
            if e.key() == Qt.Key_Space and e.type() == QEvent.KeyPress:
                self.stop_robot()
                return True
            d = KEYS.get(e.key())
            if d:
                if not e.isAutoRepeat():
                    (self.press if e.type() == QEvent.KeyPress else self.release)(d)
                return True
            if e.key() == Qt.Key_Escape:
                self.setFocus()
        elif e.type() == QEvent.ApplicationDeactivate and self.held:
            self.held.clear()                # window lost focus: never keep driving
            self.drive_tick()
        return super().eventFilter(obj, e)

    def press(self, d):
        self.held.add(d)
        self.drive_tick()

    def release(self, d):
        self.held.discard(d)
        self.drive_tick()

    def drive_tick(self):
        fwd = (("f" in self.held) - ("b" in self.held)) * self.speed.value() / 100.0
        turn = (("l" in self.held) - ("r" in self.held)) * self.turn.value() / 100.0
        if fwd or turn:
            self.bridge.drive(fwd, turn)
            self.moving = True
        elif self.moving:
            self.bridge.drive(0.0, 0.0)
            self.moving = False

    def stop_robot(self):
        self.held.clear()
        self.moving = False
        self.bridge.stop()

    # ================================================================== SLAM
    def slam_running(self):
        return self.slam is not None and self.slam.poll() is None

    def toggle_slam(self):
        try:
            if self.slam_running():
                self.stop_slam()
            else:
                self.start_slam()
        except Exception as e:      # never fail silently from a button
            QMessageBox.warning(self, "SLAM", "%s: %s" % (type(e).__name__, e))

    def start_slam(self):
        if self.bridge.slam_publishers():
            QMessageBox.warning(self, "SLAM already running",
                                "A hector_mapping node is already running (another terminal?).\n"
                                "Stop it first; two would fight over the map.")
            return
        os.makedirs(os.path.join(MAPS_DIR, "logs"), exist_ok=True)
        log_path = os.path.join(MAPS_DIR, "logs", time.strftime("slam_%Y-%m-%d_%H%M%S.log"))
        self.slam_log = open(log_path, "w")
        self.bridge.clear_trail()
        self.slam = subprocess.Popen(
            ["ros2", "launch", "robot04_slam", "hector.launch.py", "robot:=" + self.robot, "rviz:=false"],
            stdout=self.slam_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.slam_started = time.monotonic()
        self.saved_seq = None
        self.slam_button.setText("■ Stop SLAM")
        self.name_edit.setText(self.default_name())

    def stop_slam(self):
        if not self.slam_running():
            return
        try:
            os.killpg(self.slam.pid, signal.SIGINT)
            self.slam.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(self.slam.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.slam = None
        if self.slam_log:
            self.slam_log.close()
        self.slam_button.setText("▶ Start SLAM")

    def reset_map(self):
        if not self.slam_running():
            self.bridge.clear_trail()
            return
        if self.unsaved() and QMessageBox.question(
                self, "New map", "Throw away the current map and start a new one?") != QMessageBox.Yes:
            return
        self.bridge.reset_map()
        self.saved_seq = None
        self.name_edit.setText(self.default_name())

    def open_rviz(self):
        if self.rviz is not None and self.rviz.poll() is None:
            return
        cfg = subprocess.run(["ros2", "pkg", "prefix", "robot04_slam"], capture_output=True, text=True).stdout.strip()
        cfg = os.path.join(cfg, "share", "robot04_slam", "rviz", "hector.rviz")
        self.rviz = subprocess.Popen(["rviz2", "-d", cfg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     start_new_session=True)

    # ================================================================== saving
    def unsaved(self):
        s = self.bridge.snapshot()
        return s["map"] is not None and s["map_seq"] != self.saved_seq

    def save(self):
        s = self.bridge.snapshot()
        if s["map"] is None:
            QMessageBox.information(self, "Save map", "No map yet: start SLAM and drive around first.")
            return False
        name = "".join(c if c.isalnum() or c in "-_." else "_" for c in self.name_edit.text().strip()) or self.default_name()
        if os.path.exists(os.path.join(MAPS_DIR, name + ".yaml")) and QMessageBox.question(
                self, "Save map", "%s already exists. Overwrite?" % name) != QMessageBox.Yes:
            return False
        try:
            base, meta = save_map(s["map"], name, s["trail"], s["slam_pose"], self.robot)
        except Exception as e:
            QMessageBox.warning(self, "Save map", "Could not save: %s" % e)
            return False
        self.saved_seq = s["map_seq"]
        self.save_info.setText("saved %s  (%.1f × %.1f m, %.0f m² explored, path %.1f m)"
                               % (os.path.basename(base), meta["size_m"][0], meta["size_m"][1],
                                  meta["explored_m2"], meta["path_length_m"]))
        self.save_info.setStyleSheet("color:#2e7d32;")
        return True

    def open_folder(self):
        os.makedirs(MAPS_DIR, exist_ok=True)
        subprocess.Popen(["xdg-open", MAPS_DIR], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ================================================================== refresh
    def refresh(self):
        s = self.bridge.snapshot()
        now = time.monotonic()
        self.speed_label.setText("%d %%" % self.speed.value())
        self.turn_label.setText("%d %%" % self.turn.value())

        # video
        if s["image"] and s["image"][1] != self.last_image_t:
            self.last_image_t = s["image"][1]
            img = QImage.fromData(s["image"][0], "JPG")
            if not img.isNull():
                self.video.setPixmap(QPixmap.fromImage(img).scaled(self.video.size(), Qt.KeepAspectRatio,
                                                                   Qt.SmoothTransformation))
        if not s["image"] or now - s["image"][1] > 1.5:
            self.video.setText("no video (camera restarting? it recovers by itself in ~15 s)"
                               if s["image"] else "waiting for video…")
        self.video_info.setText("video %.0f fps" % self.bridge.rates["image"].hz())

        # SLAM state + map
        running = self.slam_running()
        if self.slam is not None and not running:          # launch exited by itself
            self.slam_state.setText("SLAM exited (code %s), see %s/logs" % (self.slam.returncode, MAPS_DIR))
            self.slam = None
            self.slam_button.setText("▶ Start SLAM")
        pose = s["slam_pose"][:3] if s["slam_pose"] and (now - s["slam_pose"][3] < 2.0 or not running) else None
        scan = s["scan"][0] if s["scan"] and now - s["scan"][1] < 1.0 else None
        self.map_view.set_map(s["map"], s["map_seq"])
        self.map_view.set_robot(pose, s["trail"], scan)
        if running:
            if s["map"] is None:
                self.map_view.message = "SLAM starting… waiting for the first map"
            else:
                self.map_view.message = "" if pose else "SLAM running, no pose (scans arriving?)"
            self.slam_state.setText("running %s" % ("" if s["map"] is not None else "(waiting for map)"))
        else:
            self.map_view.message = ("SLAM stopped: last map still shown, you can save it"
                                     if s["map"] is not None else "SLAM not running: press Start SLAM")
            if not self.slam_state.text().startswith("SLAM exited"):
                self.slam_state.setText("stopped")

        # status line
        parts = []
        if s["scan"] is None or now - s["scan"][1] > 2.0:
            parts.append("⚠ no lidar data from %s" % self.robot)
        else:
            parts.append("lidar %.0f Hz" % self.bridge.rates["scan"].hz())
            f, b = clearance(s["scan"][0])
            parts.append("front %s  back %s" % tuple(
                ("%.2f m%s" % (c, " BLOCKED" if c < MIN_CLEARANCE else "")) if math.isfinite(c) else "clear"
                for c in (f, b)))
        v = s["battery_v"]
        parts.append("battery %s" % ("%.2f V%s" % (v, "  LOW" if v < 7.0 else "") if v else "?"))
        if pose:
            parts.append("map pose x %.2f  y %.2f  θ %.0f°" % (pose[0], pose[1], math.degrees(pose[2])))
        parts.append("driving" if self.moving else "stopped")
        self.status.setText("   |   ".join(parts))

    # ================================================================== close
    def closeEvent(self, e):
        if self.unsaved():
            r = QMessageBox.question(self, "Quit", "Save the map before quitting?",
                                     QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
            if r == QMessageBox.Cancel or (r == QMessageBox.Save and not self.save()):
                e.ignore()
                return
        self.stop_robot()
        self.stop_slam()
        self.bridge.shutdown()
        e.accept()
