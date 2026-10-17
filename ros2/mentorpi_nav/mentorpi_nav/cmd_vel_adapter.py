#!/usr/bin/env python3
"""
cmd_vel_adapter - Nav2's velocities (m/s, rad/s) to robot_node's cmd_vel (fractions of max speed).

robot_node takes linear.x and angular.z as -1..1 of max_speed and mixes them per track
(left = fwd - turn, right = fwd + turn). Measured on robot04 (carpet, Oct 2026):
fraction 1 = 0.153 m/s (0.3 -> 0.047 m/s, 0.6 -> 0.091 m/s), and a track does not move
below about 0.1 of max speed. So:

  fwd  = v / v_max                         v_max = 0.153 m/s
  turn = w * track_width / (2 v_max)       track_width = 0.105 m (effective, scan-matched)
  each track's fraction, when not zero, is raised to at least `deadband`, so that the
  small commands a controller sends near its goal still move the robot.

  in   nav/cmd_vel        geometry_msgs/Twist  from Nav2's velocity smoother
  in   explore/control    std_msgs/String JSON {"action": "pause"|"stop"|"start"|"resume"|"home"}
  out  cmd_vel            to robot_node

Pause and stop mute the adapter (one zero command, then nothing), so the operator's
teleop owns cmd_vel; start/resume/home unmute it. robot_node still refuses to drive
into anything closer than min_clearance, whatever is sent here.
"""
import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def clip(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__("cmd_vel_adapter")
        self.v_max = self.declare_parameter("v_max", 0.153).value
        self.track_width = self.declare_parameter("track_width", 0.105).value
        self.deadband = self.declare_parameter("deadband", 0.12).value
        self.muted = self.declare_parameter("start_muted", True).value
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Twist, "nav/cmd_vel", self.on_nav, 10)
        self.create_subscription(String, "explore/control", self.on_control, 10)
        self.get_logger().info("v_max %.3f m/s, track %.3f m, deadband %.2f, %s"
                               % (self.v_max, self.track_width, self.deadband, "muted" if self.muted else "live"))

    def to_fractions(self, v, w):
        fwd = v / self.v_max
        turn = w * self.track_width / (2.0 * self.v_max)
        tracks = []
        for x in (fwd - turn, fwd + turn):
            if abs(x) < 1e-3:
                x = 0.0
            elif abs(x) < self.deadband:
                x = math.copysign(self.deadband, x)
            tracks.append(x)
        # keep the ratio between the tracks when one saturates
        k = max(1.0, abs(tracks[0]), abs(tracks[1]))
        left, right = tracks[0] / k, tracks[1] / k
        return clip((left + right) / 2.0), clip((right - left) / 2.0)

    def on_nav(self, msg):
        if self.muted:
            return
        out = Twist()
        out.linear.x, out.angular.z = self.to_fractions(msg.linear.x, msg.angular.z)
        self.pub.publish(out)

    def on_control(self, msg):
        try:
            action = json.loads(msg.data).get("action")
        except (ValueError, AttributeError):
            return
        if action in ("pause", "stop"):
            if not self.muted:
                self.pub.publish(Twist())
            self.muted = True
        elif action in ("start", "resume", "home"):
            self.muted = False


def main():
    rclpy.init()
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
