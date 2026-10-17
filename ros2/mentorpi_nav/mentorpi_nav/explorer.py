#!/usr/bin/env python3
"""
explorer - autonomous frontier exploration on top of Hector SLAM and Nav2.

Every `replan_period` s the latest /map is searched for frontiers (frontiers.py); the best
reachable one becomes a NavigateToPose goal for Nav2, which plans around the mapped walls
and lets its controller and costmaps avoid whatever the lidar sees now. A goal ends when
Nav2 reaches it, when the frontier behind it has been seen (it is then no longer a
frontier and the next one is taken), when the robot makes no progress, or on a timeout;
goals that fail, and goals reached while their frontier is still there, are blacklisted. With no frontier left, the robot drives back to where it
started and the map is saved.

  in   /map                        nav_msgs/OccupancyGrid (hector_mapping)
  in   TF map -> <robot>/base_footprint
  in   battery                     sensor_msgs/BatteryState
  in   explore/control             std_msgs/String JSON {"action": "start"|"pause"|"stop"|"home"|"save",
                                                         "name": map name for save}
  out  explore/state               std_msgs/String JSON, 1 Hz (state, goal, frontiers, distance, area, ...)
  act  navigate_to_pose            nav2_msgs/action/NavigateToPose
  log  ~/explore_logs/<start time>.jsonl   one JSON object per event, and a sample every 5 s
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .frontiers import Grid, explored_area_m2, find_frontiers

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def save_grid(grid_msg, folder, name):
    """map_server trinary pgm + yaml, cropped to the known area (same format as the PC app)."""
    os.makedirs(folder, exist_ok=True)
    g = Grid.from_msg(grid_msg)
    known = g.data >= 0
    rows, cols = np.nonzero(known.any(axis=1))[0], np.nonzero(known.any(axis=0))[0]
    if rows.size == 0:
        raise ValueError("the map is still empty")
    m = 10
    r0, r1 = max(rows[0] - m, 0), min(rows[-1] + m + 1, g.data.shape[0])
    c0, c1 = max(cols[0] - m, 0), min(cols[-1] + m + 1, g.data.shape[1])
    d = g.data[r0:r1, c0:c1]
    pgm = np.full(d.shape, 205, np.uint8)
    pgm[(d >= 0) & (d < 25)] = 254
    pgm[d >= 65] = 0
    pgm = np.flipud(pgm)
    base = os.path.join(folder, name)
    with open(base + ".pgm", "wb") as f:
        f.write(b"P5\n# robot04 explorer map\n%d %d\n255\n" % (pgm.shape[1], pgm.shape[0]))
        f.write(pgm.tobytes())
    with open(base + ".yaml", "w") as f:
        f.write("image: %s.pgm\nmode: trinary\nresolution: %.4f\n" % (name, g.resolution))
        f.write("origin: [%.4f, %.4f, 0.0]\n" % (g.origin_x + c0 * g.resolution, g.origin_y + r0 * g.resolution))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return base


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")
        p = self.declare_parameter
        self.map_frame = p("map_frame", "map").value
        self.base_frame = p("base_frame", "robot04/base_footprint").value
        self.robot_radius = p("robot_radius", 0.15).value
        self.min_frontier_m = p("min_frontier_m", 0.30).value
        self.size_weight = p("size_weight", 0.5).value
        self.replan_period = p("replan_period", 3.0).value
        self.no_progress_s = p("no_progress_s", 30.0).value
        self.goal_speed = p("goal_speed", 0.05).value            # m/s, for the goal timeout
        self.max_minutes = p("max_minutes", 45.0).value
        self.battery_min_v = p("battery_min_v", 7.0).value
        self.return_home = p("return_home", True).value
        self.save_dir = os.path.expanduser(p("save_dir", "~/maps").value)
        self.log_dir = os.path.expanduser(p("log_dir", "~/explore_logs").value)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.map_msg = None
        self.battery_v = None
        self.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)
        self.create_subscription(BatteryState, "battery", self.on_battery, 10)
        self.create_subscription(String, "explore/control", self.on_control, 10)
        self.state_pub = self.create_publisher(String, "explore/state", 10)

        self.state = "idle"          # idle | exploring | paused | returning | done
        self.message = "ready"
        self.home = None
        self.goal = None             # dict: x, y, frontier (cx, cy), sent, timeout, kind
        self.goal_handle = None
        self.goal_seq = 0
        self.blacklist = []
        self.frontiers = []
        self.last_replan = 0.0
        self.empty_count = 0
        self.progress = (0.0, None)  # (time, pose) of the last progress
        self.pose = None
        self.distance = 0.0
        self.last_pose = None
        self.started = None
        self.finished = None         # time the run ended (elapsed stops there)
        self.stats = {"goals_sent": 0, "goals_reached": 0, "goals_failed": 0, "goals_explored": 0}
        self.log_file = None
        self.last_sample = 0.0
        self.create_timer(1.0, self.tick)
        self.get_logger().info("ready: send {\"action\": \"start\"} on explore/control")

    # ------------------------------------------------------------------ inputs
    def on_map(self, msg):
        self.map_msg = msg

    def on_battery(self, msg):
        self.battery_v = None if math.isnan(msg.voltage) else msg.voltage

    def on_control(self, msg):
        try:
            c = json.loads(msg.data)
            action = c.get("action")
        except (ValueError, AttributeError):
            return
        if action == "start":
            if self.state in ("idle", "done"):
                self.begin()
            self.state, self.message = "exploring", "exploring"
            self.empty_count = 0
            self.event("start")
        elif action == "pause":
            self.cancel("paused by operator")
            if self.state in ("exploring", "returning"):
                self.state = "paused"
            self.message = "paused (teleop has control)"
            self.event("pause")
        elif action == "stop":
            self.cancel("stopped by operator")
            self.state, self.message = "idle", "stopped"
            self.event("stop")
        elif action == "home":
            self.cancel("going home")
            self.go_home()
        elif action == "save":
            self.save(c.get("name"))

    # ------------------------------------------------------------------ helpers
    def begin(self):
        self.started, self.finished = time.time(), None
        self.distance, self.last_pose = 0.0, None
        self.blacklist = []
        self.stats = {k: 0 for k in self.stats}
        os.makedirs(self.log_dir, exist_ok=True)
        if self.log_file:
            self.log_file.close()
        self.log_file = open(os.path.join(self.log_dir, time.strftime("%Y-%m-%d_%H-%M-%S") + ".jsonl"), "a")
        pose = self.robot_pose()
        self.home = pose[:2] if pose else (0.0, 0.0)
        self.event("begin", home=self.home, params={
            "robot_radius": self.robot_radius, "min_frontier_m": self.min_frontier_m,
            "size_weight": self.size_weight, "replan_period": self.replan_period,
            "no_progress_s": self.no_progress_s})

    def event(self, what, **kw):
        if not self.log_file:
            return
        rec = {"t": round(time.time(), 2), "event": what, "state": self.state,
               "pose": [round(v, 3) for v in self.pose] if self.pose else None,
               "distance_m": round(self.distance, 2)}
        rec.update(kw)
        self.log_file.write(json.dumps(rec) + "\n")
        self.log_file.flush()

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def send_goal(self, x, y, kind, frontier=None):
        if not self.nav.server_is_ready():
            self.message = "waiting for Nav2 (navigate_to_pose)"
            return False
        yaw = math.atan2(y - self.pose[1], x - self.pose[0]) if self.pose else 0.0
        msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = self.map_frame, self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = yaw_quat(yaw)
        msg.pose = ps
        self.goal_seq += 1
        seq = self.goal_seq
        dist = math.hypot(x - self.pose[0], y - self.pose[1]) if self.pose else 1.0
        path_d = frontier.distance_m if frontier else dist
        self.goal = {"x": x, "y": y, "kind": kind, "seq": seq, "sent": time.time(),
                     "timeout": 40.0 + 2.5 * path_d / self.goal_speed,
                     "frontier": (frontier.cx, frontier.cy) if frontier else None}
        self.progress = (time.time(), self.pose)
        self.stats["goals_sent"] += 1
        fut = self.nav.send_goal_async(msg)
        fut.add_done_callback(lambda f, s=seq: self.guarded(self.on_goal_response, f, s))
        return True

    def guarded(self, fn, *args):
        try:
            fn(*args)
        except Exception as e:     # noqa: BLE001
            self.get_logger().error("%s failed: %r" % (fn.__name__, e))

    def on_goal_response(self, fut, seq):
        gh = fut.result()
        if seq != self.goal_seq:
            if gh.accepted:
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            self.goal_failed("rejected by Nav2")
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, s=seq: self.guarded(self.on_result, f, s))

    def on_result(self, fut, seq):
        if seq != self.goal_seq or self.goal is None:
            return
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.stats["goals_reached"] += 1
            self.event("goal_reached", goal=[round(self.goal["x"], 2), round(self.goal["y"], 2)],
                       kind=self.goal["kind"], seconds=round(time.time() - self.goal["sent"], 1))
            kind = self.goal["kind"]
            if kind == "frontier":
                # reached, but the frontier may still be there: a pocket the lidar can't see
                # into from anywhere reachable (under furniture). Don't come back for it.
                self.blacklist.append((self.goal["x"], self.goal["y"]))
                if self.goal["frontier"]:
                    self.blacklist.append(self.goal["frontier"])
            self.goal, self.goal_handle = None, None
            if kind == "home":
                self.finish("back at the start")
        elif status == GoalStatus.STATUS_CANCELED:
            pass                      # we cancelled it ourselves
        else:
            self.goal_failed("Nav2 aborted (status %d)" % status)

    def goal_failed(self, why):
        g = self.goal
        self.goal, self.goal_handle = None, None
        if g is None:
            return
        self.stats["goals_failed"] += 1
        self.event("goal_failed", goal=[round(g["x"], 2), round(g["y"], 2)], kind=g["kind"], why=why)
        self.get_logger().warning("goal (%.2f, %.2f) failed: %s" % (g["x"], g["y"], why))
        if g["kind"] == "frontier":
            self.blacklist.append((g["x"], g["y"]))
            if g["frontier"]:
                self.blacklist.append(g["frontier"])
        elif g["kind"] == "home":
            self.finish("could not drive back to the start (%s)" % why)

    def cancel(self, why):
        if self.goal is not None:
            self.event("goal_cancelled", why=why)
        self.goal_seq += 1            # results of the old goal are ignored from now on
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal, self.goal_handle = None, None

    def go_home(self):
        if self.home is None:
            self.state, self.message = "idle", "no start position recorded"
            return
        self.state, self.message = "returning", "driving back to the start"
        self.event("return_home")
        self.send_goal(self.home[0], self.home[1], "home")

    def finish(self, why):
        self.cancel(why)
        self.state, self.message = "done", "exploration finished: " + why
        self.finished = time.time()
        self.get_logger().info(self.message)
        self.save(None)
        self.event("done", why=why, stats=self.stats,
                   explored_m2=round(explored_area_m2(Grid.from_msg(self.map_msg)), 2) if self.map_msg else None,
                   minutes=round((time.time() - self.started) / 60.0, 2) if self.started else None)

    def save(self, name):
        if self.map_msg is None:
            self.message = "no map to save"
            return
        name = name or "explore_" + time.strftime("%Y-%m-%d_%H-%M-%S")
        try:
            base = save_grid(self.map_msg, self.save_dir, name)
            self.message = "map saved to %s.pgm/.yaml" % base
            self.event("saved", path=base)
        except Exception as e:     # noqa: BLE001 - report, don't die
            self.message = "save failed: %s" % e

    # ------------------------------------------------------------------ main loop (1 Hz)
    def tick(self):
        try:
            self.tick_inner()
        except Exception as e:     # noqa: BLE001 - a bug must not leave the robot driving blind
            self.get_logger().error("tick failed: %r" % e)
            self.message = "internal error: %r" % e

    def tick_inner(self):
        pose = self.robot_pose()
        if pose:
            # distance driven: count 3 cm steps only (SLAM pose jitter would add up), ignore jumps
            if self.last_pose is None or self.state not in ("exploring", "returning"):
                self.last_pose = pose
            else:
                step = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
                if step > 1.0:
                    self.last_pose = pose
                elif step > 0.03:
                    self.distance += step
                    self.last_pose = pose
        self.pose = pose
        now = time.time()

        if self.state in ("exploring", "returning") and self.started:
            if (now - self.started) / 60.0 > self.max_minutes:
                self.cancel("time limit")
                if self.state == "exploring" and self.return_home:
                    self.go_home()
                else:
                    self.finish("time limit")
            elif self.battery_v is not None and self.battery_v < self.battery_min_v and self.state == "exploring":
                self.cancel("battery low")
                self.message = "battery %.2f V: returning" % self.battery_v
                self.go_home()

        if self.state == "exploring" and self.pose and self.map_msg is not None:
            self.explore_step(now)
        elif self.state == "returning" and self.goal is None and self.home:
            self.send_goal(self.home[0], self.home[1], "home")

        if self.goal is not None and self.state in ("exploring", "returning"):
            self.watch_goal(now)

        if self.log_file and now - self.last_sample > 5.0 and self.state in ("exploring", "returning"):
            self.last_sample = now
            self.event("sample", explored_m2=round(explored_area_m2(Grid.from_msg(self.map_msg)), 2)
                       if self.map_msg else None, frontiers=len(self.frontiers), battery_v=self.battery_v)
        self.publish_state(now)

    def explore_step(self, now):
        if now - self.last_replan < self.replan_period and self.goal is not None:
            return
        self.last_replan = now
        t0 = time.monotonic()
        grid = Grid.from_msg(self.map_msg)
        self.frontiers, info = find_frontiers(
            grid, self.pose[:2], robot_radius=self.robot_radius, min_size_m=self.min_frontier_m,
            size_weight=self.size_weight, blacklist=self.blacklist)
        ms = (time.monotonic() - t0) * 1000.0

        if self.goal is not None:
            # keep driving while the frontier behind the goal is still there
            fc = self.goal["frontier"]
            alive = fc is None or any(math.hypot(f.cx - fc[0], f.cy - fc[1]) < 0.6 or
                                      math.hypot(f.x - self.goal["x"], f.y - self.goal["y"]) < 0.4
                                      for f in self.frontiers)
            if alive:
                return
            self.stats["goals_explored"] += 1
            self.event("frontier_explored", goal=[round(self.goal["x"], 2), round(self.goal["y"], 2)])
            self.cancel("frontier explored")

        if not self.frontiers:
            self.empty_count += 1
            self.message = "no reachable frontier (%d/3)%s" % (
                self.empty_count, ": " + info["reason"] if info.get("reason") else "")
            if self.empty_count >= 3:
                self.event("no_frontiers")
                if self.return_home:
                    self.go_home()
                else:
                    self.finish("map complete")
            return
        self.empty_count = 0
        f = self.frontiers[0]
        if self.send_goal(f.x, f.y, "frontier", f):
            self.message = "to frontier %.1f m away (%.1f m wide)" % (f.distance_m, f.size_m)
            self.event("goal_sent", goal=[round(f.x, 2), round(f.y, 2)], frontier=[round(f.cx, 2), round(f.cy, 2)],
                       size_m=round(f.size_m, 2), path_m=round(f.distance_m, 2), candidates=len(self.frontiers),
                       search_ms=round(ms, 1))

    def watch_goal(self, now):
        g = self.goal
        t_prog, p_prog = self.progress
        if p_prog is None or (self.pose and math.hypot(self.pose[0] - p_prog[0], self.pose[1] - p_prog[1]) > 0.15) \
                or (self.pose and abs(math.atan2(math.sin(self.pose[2] - p_prog[2]), math.cos(self.pose[2] - p_prog[2]))) > 0.5):
            self.progress = (now, self.pose)
        elif now - t_prog > self.no_progress_s:
            self.cancel("no progress")
            self.goal = g                 # so goal_failed() can blacklist it
            self.goal_failed("no progress for %.0f s" % self.no_progress_s)
            return
        if now - g["sent"] > g["timeout"]:
            self.cancel("timeout")
            self.goal = g
            self.goal_failed("timeout after %.0f s" % g["timeout"])

    def publish_state(self, now):
        s = {"state": self.state, "message": self.message,
             "pose": [round(v, 3) for v in self.pose] if self.pose else None,
             "home": [round(v, 2) for v in self.home] if self.home else None,
             "goal": [round(self.goal["x"], 2), round(self.goal["y"], 2)] if self.goal else None,
             "frontiers": [[round(f.cx, 2), round(f.cy, 2), round(f.size_m, 2)] for f in self.frontiers[:30]],
             "distance_m": round(self.distance, 2),
             "elapsed_s": round((self.finished or now) - self.started, 1) if self.started else 0.0,
             "battery_v": self.battery_v, "blacklist": len(self.blacklist), "nav2": self.nav.server_is_ready()}
        s.update(self.stats)
        self.state_pub.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
