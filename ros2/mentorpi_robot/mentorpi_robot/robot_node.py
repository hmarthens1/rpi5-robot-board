#!/usr/bin/env python3
"""
robot_node - the fleet interface for a Hiwonder MentorPi T1 (tracked chassis,
Raspberry Pi 5, RRC Lite controller), run in namespace /<hostname>.

Same contract as the Pi 4 fleet robots (rpi4-robot-board, command_node/status_node),
so robot-fleet-gui and the voice agent drive it without changes:

  in   command         std_msgs/String  JSON {"id", "action", ...}
  out  command_result  std_msgs/String  JSON {"id", "action", "ok", "detail"}
  in   cmd_vel         geometry_msgs/Twist  linear.x forward, angular.z turn, -1..1 of max_speed
  out  battery         sensor_msgs/BatteryState      1 Hz
  out  system          diagnostic_msgs/DiagnosticStatus  1 Hz (same keys as status_node)

New on this robot:
  out  odom            nav_msgs/Odometry 20 Hz, from the track encoders, + TF odom -> base_footprint
  out  imu             sensor_msgs/Imu   (board IMU, QMI8658)
  in   scan            sensor_msgs/LaserScan  obstacle checks in the direction of travel

Actions: status, stop, drive {vx,vy,turn,duration}, nudge {direction,seconds,speed},
motor {motor,speed,duration}, led {led,on}, buzzer {seconds,times},
move {distance_m, speed}   closed loop on the encoders  (new)
turn {degrees, speed}      closed loop on the encoders  (new)

Safety (enforced here, whatever the sender remembered): speeds are fractions of
max_speed; timed motions stop after at most max_duration; move/turn stop after
move_timeout; stop always wins; motion into an obstacle closer than min_clearance
(lidar) is refused or stopped; cmd_vel stops cmd_vel_timeout after the last message;
and underneath, the RRC Lite firmware stops the motors 300 ms after the Pi goes quiet.
"""
import json
import math
import os
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .rrclite import RRCLite, PROFILES

G = 9.80665
RPS_LIMIT = {"JGB520": 1.5, "JGB37": 3.0, "JGA27": 6.0, "JGB528": 1.1}   # firmware's cap per profile
UNSUPPORTED = {"servo", "rgb", "sonar_light", "arm_pose", "arm_move", "gripper", "arm_servos",
               "arm_sequence", "arm_release", "get_servo_offsets", "set_servo_offsets"}


def clip(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(x)))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")
        p = self.declare_parameter
        self.max_speed = p("max_speed", 50.0).value                  # % of the motor's top speed
        self.max_duration = p("max_duration", 5.0).value             # s, timed motions
        self.move_timeout = p("move_timeout", 15.0).value            # s, move/turn
        self.max_move_m = p("max_move_m", 2.0).value
        self.cmd_vel_timeout = p("cmd_vel_timeout", 0.5).value
        self.min_clearance = p("min_clearance", 0.25).value          # m, lidar
        self.front_half_angle = math.radians(p("front_half_angle_deg", 25.0).value)
        self.profile = p("motor_profile", "JGB520").value
        self.port = p("port", "/dev/rrclite").value
        self.left_motor = p("left_motor", 1).value
        self.right_motor = p("right_motor", 2).value
        self.left_sign = p("left_sign", 1).value                     # +1/-1: motor rotation for "forward"
        self.right_sign = p("right_sign", -1).value
        self.wheel_radius = p("wheel_radius", 0.0325).value          # m, track drive sprocket (calibrate)
        self.track_width = p("track_width", 0.170).value             # m, effective (calibrate with turn)
        self.battery_low_v = p("battery_low_v", 7.0).value
        self.temp_warn_c = p("temp_warn_c", 75.0).value
        self.lidar_yaw = p("lidar_yaw_deg", 0.0).value               # where the lidar's 0 deg points
        self.host = socket.gethostname()
        # TF is global: prefix every frame with the robot's name (robot04/odom ...)
        self.prefix = p("frame_prefix", self.host + "/").value

        self.rps_max = RPS_LIMIT[self.profile] * self.max_speed / 100.0
        self.board = RRCLite(self.port, self.profile)
        self.cpt = PROFILES[self.profile][1]

        self.lock = threading.RLock()
        self.speeds = {}                   # motor -> rps currently commanded
        self.motion = None                 # dict: kind, until, desc, dir (+1 fwd/-1 back/0 turn) ...
        self.cmd_vel_last = 0.0
        self.motion_desc = "stopped"

        # odometry state (updated from the board's encoder report thread)
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_counts = None
        self.dist_total = 0.0              # |distance| along the track, for move
        self.yaw_total = 0.0               # integrated yaw, for turn
        self.board.on_motor_report = self.on_encoders

        self.scan = None                   # (stamp, front_min, back_min)
        self.result_pub = self.create_publisher(String, "command_result", 10)
        self.battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self.system_pub = self.create_publisher(DiagnosticStatus, "system", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.imu_pub = self.create_publisher(Imu, "imu", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, "command", self.on_command, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.control_tick)                    # 20 Hz: keepalive, limits, odom
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.02, self.publish_imu)
        self.ns = self.get_namespace().strip("/") or self.host
        self.get_logger().info(
            "ready: max_speed %.0f %% (%.2f rev/s), max_duration %.1f s, profile %s, tracks M%d/M%d"
            % (self.max_speed, self.rps_max, self.max_duration, self.profile, self.left_motor, self.right_motor))

    # ------------------------------------------------------------------ plumbing
    def reply(self, cid, action, ok, detail):
        self.result_pub.publish(String(data=json.dumps(
            {"id": cid, "action": action, "ok": ok, "detail": detail})))
        text = detail if isinstance(detail, str) else json.dumps(detail)
        if ok:     # separate calls: rclpy fixes the severity per call site
            self.get_logger().info("%s: %s" % (action, text))
        else:
            self.get_logger().warning("%s: %s" % (action, text))

    def on_command(self, msg):
        cid, action = None, "?"
        try:
            cmd = json.loads(msg.data)
            cid = cmd.get("id")
            action = cmd.get("action", "?")
            self.reply(cid, action, True, self.dispatch(action, cmd, cid))
        except Exception as e:                  # bad JSON, bad values, serial errors: report, never crash
            if action != "stop":
                self.stop_motors()
            self.reply(cid, action, False, str(e))

    def dispatch(self, action, cmd, cid):
        handlers = {"drive": self.do_drive, "nudge": self.do_nudge, "motor": self.do_motor,
                    "led": self.do_led, "buzzer": self.do_buzzer, "status": self.do_status,
                    "move": self.do_move, "turn": self.do_turn}
        if action == "stop":
            self.stop_motors()
            return "stopped"
        if action in UNSUPPORTED:
            raise ValueError("this robot (MentorPi T1) has no %s" % action.replace("_", " "))
        if action not in handlers:
            raise ValueError("unknown action %r" % action)
        return handlers[action](cmd) if action not in ("move", "turn") else handlers[action](cmd, cid)

    # ------------------------------------------------------------------ motors
    def track_speeds(self, fwd, turn):
        """fwd, turn: -1..1 fractions of max speed -> {motor: rps}. turn > 0 = counter-clockwise."""
        left = clip(fwd - turn)
        right = clip(fwd + turn)
        return {self.left_motor: self.left_sign * left * self.rps_max,
                self.right_motor: self.right_sign * right * self.rps_max}

    def apply(self, speeds):
        with self.lock:
            self.speeds = speeds
            self.board.set_speeds(speeds)

    def stop_motors(self):
        with self.lock:
            self.motion = None
            self.speeds = {}
            self.cmd_vel_last = 0.0
            self.motion_desc = "stopped"
        self.board.stop()

    def duration(self, cmd):
        d = float(cmd.get("duration", 1.0))
        if d <= 0:
            raise ValueError("duration must be > 0")
        return min(d, self.max_duration)

    def start_motion(self, speeds, seconds, desc, direction):
        self.require_clearance(direction)
        with self.lock:
            self.apply(speeds)
            self.motion = {"kind": "timed", "until": time.monotonic() + seconds, "dir": direction}
            self.motion_desc = desc
        return "%s for %.1f s" % (desc, seconds)

    # ------------------------------------------------------------------ safety: lidar
    def on_scan(self, msg):
        front, back = math.inf, math.inf
        a = msg.angle_min + math.radians(self.lidar_yaw)
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                ang = math.atan2(math.sin(a), math.cos(a))
                if abs(ang) <= self.front_half_angle:
                    front = min(front, r)
                elif abs(ang) >= math.pi - self.front_half_angle:
                    back = min(back, r)
            a += msg.angle_increment
        self.scan = (time.monotonic(), front, back)

    def clearance(self, direction):
        """Distance to the nearest obstacle in the direction of travel, None if unknown,
        math.inf if no lidar has ever been seen (then nothing is enforced, like the fleet's
        sonar-less robots)."""
        if direction == 0:
            return math.inf
        if self.scan is None:
            return math.inf
        stamp, front, back = self.scan
        if time.monotonic() - stamp > 1.0:
            return None
        return front if direction > 0 else back

    def require_clearance(self, direction):
        c = self.clearance(direction)
        side = "ahead" if direction > 0 else "behind"
        if c is None:
            raise RuntimeError("refused: lidar stopped reporting, path %s unknown" % side)
        if c < self.min_clearance:
            raise RuntimeError("refused: obstacle %.2f m %s (minimum %.2f m)" % (c, side, self.min_clearance))

    # ------------------------------------------------------------------ actions
    def do_drive(self, cmd):
        vx, vy, turn = clip(cmd.get("vx", 0.0)), clip(cmd.get("vy", 0.0)), clip(cmd.get("turn", 0.0))
        if vx != 0.0:
            raise ValueError("this robot can't slide sideways (tracked chassis)")
        desc = "drive vy=%+.2f turn=%+.2f" % (vy, turn)
        return self.start_motion(self.track_speeds(vy, turn), self.duration(cmd), desc,
                                 (vy > 0) - (vy < 0))

    def do_nudge(self, cmd):
        d = cmd.get("direction", "forward")
        dirs = {"forward": (1, 0), "back": (-1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}
        if d not in dirs:
            raise ValueError("direction must be forward/back/left/right (no sideways slide on tracks)")
        speed = clip(cmd.get("speed", 1.0), 0.0, 1.0)
        seconds = clip(cmd.get("seconds", 0.3), 0.05, 0.5)
        f, t = dirs[d]
        with self.lock:
            self.apply(self.track_speeds(f * speed, t * speed))
            self.motion = {"kind": "timed", "until": time.monotonic() + seconds, "dir": 0}
            self.motion_desc = "nudge " + d
        return "nudge %s for %.2f s" % (d, seconds)

    def do_motor(self, cmd):
        m = int(cmd["motor"])
        if m not in (1, 2, 3, 4):
            raise ValueError("motor must be 1..4")
        speed = clip(cmd.get("speed", 0.3))
        return self.start_motion({m: speed * self.rps_max}, self.duration(cmd), "motor %d speed %+.2f" % (m, speed), 0)

    def do_led(self, cmd):
        on = bool(cmd.get("on", True))
        self.board.led(on_s=1.0 if on else 0.0, off_s=0.0 if on else 1.0, times=0 if on else 1)
        return "LED %s" % ("on" if on else "off")

    def do_buzzer(self, cmd):
        seconds = clip(cmd.get("seconds", 0.2), 0.02, 2.0)
        times = int(clip(cmd.get("times", 1), 1, 5))
        self.board.beep(2400, seconds, 0.15, times)
        return "beep %d x %.2f s" % (times, seconds)

    def do_move(self, cmd, cid):
        dist = float(cmd["distance_m"]) if "distance_m" in cmd else float(cmd["distance_cm"]) / 100.0
        if not 0.01 <= abs(dist) <= self.max_move_m:
            raise ValueError("distance must be 0.01..%.1f m" % self.max_move_m)
        if not self.board.has_encoders:
            raise RuntimeError("no encoder report: flash the Lab 03 firmware for closed-loop moves")
        speed = clip(cmd.get("speed", 0.6), 0.15, 1.0)
        direction = 1 if dist > 0 else -1
        self.require_clearance(direction)
        with self.lock:
            self.motion = {"kind": "move", "goal": abs(dist), "start": self.dist_total, "dir": direction,
                           "speed": speed, "until": time.monotonic() + self.move_timeout, "id": cid}
            self.motion_desc = "move %+.2f m" % dist
            self.apply(self.track_speeds(direction * speed, 0.0))
        return "moving %+.2f m (encoders)" % dist

    def do_turn(self, cmd, cid):
        deg = float(cmd["degrees"])
        if not 1.0 <= abs(deg) <= 720.0:
            raise ValueError("degrees must be 1..720 (positive = counter-clockwise / left)")
        if not self.board.has_encoders:
            raise RuntimeError("no encoder report: flash the Lab 03 firmware for closed-loop turns")
        speed = clip(cmd.get("speed", 0.5), 0.15, 1.0)
        sign = 1 if deg > 0 else -1
        with self.lock:
            self.motion = {"kind": "turn", "goal": math.radians(abs(deg)), "start": self.yaw_total,
                           "sign": sign, "speed": speed, "dir": 0,
                           "until": time.monotonic() + self.move_timeout, "id": cid}
            self.motion_desc = "turn %+.0f deg" % deg
            self.apply(self.track_speeds(0.0, sign * speed))
        return "turning %+.0f deg (encoders)" % deg

    def do_status(self, cmd=None):
        c_front, c_back = (self.scan[1], self.scan[2]) if self.scan else (None, None)
        fin = lambda v: None if v is None or not math.isfinite(v) else round(v, 3)
        with self.lock:
            return {"battery_v": self.board.battery_v, "distance_m": fin(c_front), "distance_back_m": fin(c_back),
                    "motion": self.motion_desc,
                    "motor_speeds": {str(i + 1): round(m.rps, 3) for i, m in enumerate(self.board.motors)},
                    "odom": {"x": round(self.x, 3), "y": round(self.y, 3), "yaw_deg": round(math.degrees(self.yaw), 1)},
                    "max_speed": self.max_speed, "max_duration": self.max_duration, "drive": "tracked",
                    "min_clearance_m": self.min_clearance if self.scan else None,
                    "encoders": self.board.has_encoders, "lidar": self.scan is not None,
                    "capabilities": ["drive", "nudge", "motor", "led", "buzzer", "move", "turn", "stop", "status"]}

    # ------------------------------------------------------------------ cmd_vel
    def on_cmd_vel(self, msg):
        fwd, turn = clip(msg.linear.x), clip(msg.angular.z)
        direction = (fwd > 0) - (fwd < 0)
        try:
            self.require_clearance(direction)
        except RuntimeError:
            self.stop_motors()
            return
        with self.lock:
            self.apply(self.track_speeds(fwd, turn))
            self.motion = {"kind": "cmd_vel", "dir": direction}
            self.motion_desc = "cmd_vel"
            self.cmd_vel_last = time.monotonic()

    # ------------------------------------------------------------------ odometry
    def on_encoders(self, stamp, motors):
        counts = (motors[self.left_motor - 1].count, motors[self.right_motor - 1].count)
        with self.lock:
            if self.last_counts is None:
                self.last_counts, self.last_stamp = counts, stamp
                return
            dl = self.left_sign * (counts[0] - self.last_counts[0]) / self.cpt * 2 * math.pi * self.wheel_radius
            dr = self.right_sign * (counts[1] - self.last_counts[1]) / self.cpt * 2 * math.pi * self.wheel_radius
            dt = max(stamp - self.last_stamp, 1e-3)
            self.last_counts, self.last_stamp = counts, stamp
            ds, dth = (dl + dr) / 2, (dr - dl) / self.track_width
            self.x += ds * math.cos(self.yaw + dth / 2)
            self.y += ds * math.sin(self.yaw + dth / 2)
            self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))
            self.v, self.w = ds / dt, dth / dt
            self.dist_total += abs(ds)
            self.yaw_total += dth

    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        with self.lock:
            x, y, yaw, v, w = self.x, self.y, self.yaw, self.v, self.w
        q = yaw_quat(yaw)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, self.prefix + "odom", self.prefix + "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.odom_pub.publish(o)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, self.prefix + "odom", self.prefix + "base_footprint"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)

    # ------------------------------------------------------------------ 20 Hz loop
    def control_tick(self):
        if self.board.has_encoders:
            self.publish_odom()
        finished = None
        with self.lock:
            m = self.motion
            now = time.monotonic()
            if m is None:
                return
            if m["kind"] == "cmd_vel" and now - self.cmd_vel_last > self.cmd_vel_timeout:
                self.motion, finished = None, (None, "stop", "cmd_vel timeout")
            elif m["kind"] != "cmd_vel" and now >= m["until"]:
                what = "motion finished" if m["kind"] == "timed" else "%s timed out" % m["kind"]
                self.motion, finished = None, (m.get("id"), "stop" if m["kind"] == "timed" else m["kind"], what)
            elif m["kind"] == "move":
                done = self.dist_total - m["start"]
                left = m["goal"] - done
                if left <= 0.005:
                    self.motion, finished = None, (m["id"], "move", "moved %.3f m (encoders)" % (m["dir"] * done))
                else:
                    slow = clip(left / 0.15, 0.3, 1.0)            # slow down over the last 15 cm
                    self.apply(self.track_speeds(m["dir"] * m["speed"] * slow, 0.0))
            elif m["kind"] == "turn":
                done = abs(self.yaw_total - m["start"])
                left = m["goal"] - done
                if left <= math.radians(1.0):
                    self.motion, finished = None, (m["id"], "turn", "turned %+.1f deg (encoders)" % (m["sign"] * math.degrees(done)))
                else:
                    slow = clip(left / math.radians(25), 0.3, 1.0)
                    self.apply(self.track_speeds(0.0, m["sign"] * m["speed"] * slow))
            elif self.speeds:
                self.board.set_speeds(self.speeds)                # keepalive for the firmware watchdog
        if finished is None and m is not None and m.get("dir"):
            c = self.clearance(m["dir"])
            if c is None or c < self.min_clearance:
                finished = (m.get("id"), "stop", "stopped: obstacle %s" % ("unknown (lidar silent)" if c is None else "%.2f m %s" % (c, "ahead" if m["dir"] > 0 else "behind")))
        if finished:
            self.stop_motors()
            self.reply(*finished[:2], True, finished[2])

    # ------------------------------------------------------------------ status
    def publish_imu(self):
        d = self.board.imu
        if not d:
            return
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.prefix + "imu_link"
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (v * G for v in d[:3])
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (math.radians(v) for v in d[3:])
        msg.orientation_covariance[0] = -1.0                       # no orientation estimate
        self.imu_pub.publish(msg)

    def publish_status(self):
        v = self.board.battery_v
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = float(v) if v is not None else math.nan
        b.current = b.charge = b.capacity = b.design_capacity = b.percentage = math.nan
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        b.present = v is not None
        self.battery_pub.publish(b)

        temp = read_float("/sys/class/thermal/thermal_zone0/temp", 1000.0)
        flags = throttled()
        values = [("cpu_temp_c", fmt(temp, 1)), ("load_1min", fmt(os.getloadavg()[0], 2)),
                  ("cpus", str(os.cpu_count())), ("mem_used_percent", fmt(mem_used(), 0)),
                  ("disk_free_gb", fmt(disk_free_gb(), 1)), ("wifi_signal_dbm", fmt(wifi_dbm(), 0)),
                  ("ip", wlan0_ip()), ("battery_v", fmt(v, 2)),
                  ("throttled", "0x%x" % flags if flags is not None else "unknown"),
                  ("uptime_s", fmt(read_float("/proc/uptime"), 0)),
                  ("robot_type", "mentorpi_t1"), ("encoders", str(self.board.has_encoders).lower()),
                  ("lidar", str(self.scan is not None).lower())]
        problems = []
        if flags is not None and flags & 0x1:
            problems.append("under-voltage now")
        elif flags is not None and flags & 0x10000:
            problems.append("under-voltage since boot")
        if temp is not None and temp >= self.temp_warn_c:
            problems.append("CPU %.0f C" % temp)
        if v is None:
            problems.append("no battery reading (RRC Lite not answering)")
        elif v < self.battery_low_v:
            problems.append("battery low %.2f V" % v)
        s = DiagnosticStatus()
        s.name, s.hardware_id = self.host + "/system", self.host
        s.level = DiagnosticStatus.WARN if problems else DiagnosticStatus.OK
        s.message = ", ".join(problems) if problems else "OK"
        s.values = [KeyValue(key=k, value=val) for k, val in values]
        self.system_pub.publish(s)

    def destroy_node(self):
        try:
            self.board.close()
        finally:
            super().destroy_node()


# ---------------------------------------------------------------------- helpers
def fmt(v, digits):
    return "nan" if v is None or (isinstance(v, float) and math.isnan(v)) else ("%.*f" % (digits, v))


def read_float(path, div=1.0):
    try:
        with open(path) as f:
            return float(f.read().split()[0]) / div
    except (OSError, ValueError, IndexError):
        return None


def throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1).stdout
        return int(out.strip().split("=")[1], 16)
    except Exception:
        return None


def mem_used():
    try:
        info = dict(l.split(":", 1) for l in open("/proc/meminfo"))
        total, avail = int(info["MemTotal"].split()[0]), int(info["MemAvailable"].split()[0])
        return 100.0 * (total - avail) / total
    except Exception:
        return None


def disk_free_gb():
    st = os.statvfs("/")
    return st.f_bavail * st.f_frsize / 1e9


def wifi_dbm():
    try:
        for line in open("/proc/net/wireless"):
            if line.strip().startswith("wlan0:"):
                return float(line.split()[3].rstrip("."))
    except (OSError, ValueError, IndexError):
        pass
    return None


def wlan0_ip():
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show", "wlan0"], capture_output=True, text=True, timeout=1).stdout
        return out.split()[3].split("/")[0]
    except Exception:
        return "unknown"


def main():
    rclpy.init()
    node = RobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
