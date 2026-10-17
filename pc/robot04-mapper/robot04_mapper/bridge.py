"""ROS side of the mapper: one rclpy node spun in a background thread.

Callbacks only store the latest message (under a lock); the Qt side polls with a timer,
so no Qt object is touched from the ROS thread.

  in   /<robot>/vision/image/compressed  JPEG from camera_bridge (10 Hz while subscribed)
  in   /<robot>/scan                     MS200, 10 Hz
  in   /<robot>/battery, /<robot>/odom
  in   /map, /slam_out_pose              from hector_mapping (when SLAM runs, here or on the robot)
  in   /<robot>/explore/state            explorer JSON (state, goal, frontiers, ...), 1 Hz
  in   /plan                             Nav2's global path to the current goal
  out  /<robot>/cmd_vel                  linear.x / angular.z in -1..1 of the robot's max_speed
  out  /<robot>/command                  JSON {"id", "action": "stop"}
  out  /<robot>/vision/control           JSON {"image": "rgb"|"depth"}
  out  /syscommand                       "reset" -> hector starts a new map
  out  /<robot>/explore/control          JSON {"action": "start"|"pause"|"stop"|"home"|"save"}
"""
import json
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from std_msgs.msg import String

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rate:
    """Messages per second over the last 2 s."""

    def __init__(self):
        self.times = deque(maxlen=100)

    def tick(self):
        self.times.append(time.monotonic())

    def hz(self):
        now = time.monotonic()
        n = sum(1 for t in self.times if now - t < 2.0)
        return n / 2.0


class Bridge:
    def __init__(self, robot="robot04"):
        self.robot = robot
        self.lock = threading.Lock()
        self.image = None            # (jpeg bytes, monotonic time)
        self.scan = None             # (LaserScan, monotonic time)
        self.map = None              # OccupancyGrid
        self.map_seq = 0
        self.slam_pose = None        # (x, y, yaw, monotonic time)
        self.odom = None             # (x, y, yaw)
        self.battery_v = None
        self.trail = []              # SLAM poses, for the drawn path and the saved metadata
        self.explore = None          # (dict, monotonic time) from the robot's explorer
        self.plan = None             # [(x, y)] Nav2 global path
        self.rates = {"image": Rate(), "scan": Rate()}

        rclpy.init(args=None)
        self.node = rclpy.create_node("robot04_mapper")
        n, r = self.node, "/" + robot
        n.create_subscription(CompressedImage, r + "/vision/image/compressed", self.on_image, qos_profile_sensor_data)
        n.create_subscription(LaserScan, r + "/scan", self.on_scan, qos_profile_sensor_data)
        n.create_subscription(BatteryState, r + "/battery", self.on_battery, 10)
        n.create_subscription(Odometry, r + "/odom", self.on_odom, qos_profile_sensor_data)
        n.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)
        n.create_subscription(PoseStamped, "/slam_out_pose", self.on_pose, 10)
        n.create_subscription(String, r + "/explore/state", self.on_explore, 10)
        n.create_subscription(Path, "/plan", self.on_plan, 10)
        self.cmd_pub = n.create_publisher(Twist, r + "/cmd_vel", 10)
        self.command_pub = n.create_publisher(String, r + "/command", 10)
        self.vision_pub = n.create_publisher(String, r + "/vision/control", 10)
        self.sys_pub = n.create_publisher(String, "/syscommand", 10)
        self.explore_pub = n.create_publisher(String, r + "/explore/control", 10)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    # ---------------------------------------------------------------- callbacks (ROS thread)
    def on_image(self, msg):
        with self.lock:
            self.image = (bytes(msg.data), time.monotonic())
        self.rates["image"].tick()

    def on_scan(self, msg):
        with self.lock:
            self.scan = (msg, time.monotonic())
        self.rates["scan"].tick()

    def on_battery(self, msg):
        with self.lock:
            self.battery_v = msg.voltage if not math.isnan(msg.voltage) else None

    def on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_map(self, msg):
        with self.lock:
            self.map = msg
            self.map_seq += 1

    def on_pose(self, msg):
        p = msg.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        with self.lock:
            self.slam_pose = (x, y, yaw, time.monotonic())
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.05:
                self.trail.append((x, y))

    def on_explore(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.explore = (state, time.monotonic())

    def on_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses[::3]]
        with self.lock:
            self.plan = pts

    # ---------------------------------------------------------------- reads (Qt thread)
    def snapshot(self):
        with self.lock:
            return {"image": self.image, "scan": self.scan, "map": self.map, "map_seq": self.map_seq,
                    "slam_pose": self.slam_pose, "odom": self.odom, "battery_v": self.battery_v,
                    "trail": list(self.trail), "explore": self.explore, "plan": self.plan}

    def clear_trail(self):
        with self.lock:
            self.trail = []
            self.map = None
            self.slam_pose = None
            self.map_seq += 1

    def slam_publishers(self):
        """Publishers of /slam_out_pose: is a hector_mapping already running somewhere?
        (Asks per topic: listing node names fails when any participant on the network
        has an empty name, "empty node name returned by the RMW layer".)"""
        try:
            return self.node.count_publishers("/slam_out_pose")
        except Exception:
            return 0

    # ---------------------------------------------------------------- outputs
    def drive(self, fwd, turn):
        t = Twist()
        t.linear.x, t.angular.z = float(fwd), float(turn)
        self.cmd_pub.publish(t)

    def stop(self):
        self.drive(0.0, 0.0)
        self.command_pub.publish(String(data=json.dumps({"id": "mapper-%d" % int(time.time() * 1000), "action": "stop"})))

    def set_image_mode(self, mode):
        self.vision_pub.publish(String(data=json.dumps({"image": mode})))

    def explore_command(self, action, **kw):
        kw["action"] = action
        self.explore_pub.publish(String(data=json.dumps(kw)))

    def reset_map(self):
        self.sys_pub.publish(String(data="reset"))
        self.clear_trail()

    def shutdown(self):
        try:
            self.stop()
        except Exception:
            pass
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        rclpy.try_shutdown()
