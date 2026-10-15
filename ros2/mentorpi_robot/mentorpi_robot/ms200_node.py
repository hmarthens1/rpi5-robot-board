#!/usr/bin/env python3
"""
ms200_node - Oradar MS200 lidar -> sensor_msgs/LaserScan.

The MS200 streams 47-byte packets at 230400 baud (same layout as the LD06):

  0x54 0x2C | speed u16 deg/s | start angle u16 0.01 deg | 12 x (distance u16 mm, intensity u8)
            | end angle u16 0.01 deg | timestamp u16 ms | CRC-8 (poly 0x4D, init 0)

Angles grow clockwise seen from above; ROS wants counter-clockwise, so with
clockwise=true each angle is negated. One revolution (~10 Hz, ~450 points) is binned
into a fixed number of rays so angle_increment is constant, which scan matchers
such as hector_mapping need. A ray with no return is +inf.

  out scan   sensor_msgs/LaserScan   frame_id param (default "laser"), SensorDataQoS
"""
import math
import struct
import threading
import time

import rclpy
import rclpy.time
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PACKET_LEN = 47
POINTS = 12


def _crc_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = _crc_table()


def crc8(data):
    c = 0
    for b in data:
        c = CRC_TABLE[c ^ b]
    return c


def parse(packet):
    """-> (speed deg/s, [(angle deg clockwise, distance m, intensity)]) or None on a bad CRC."""
    if crc8(packet[:46]) != packet[46]:
        return None
    speed, start = struct.unpack_from("<HH", packet, 2)
    end = struct.unpack_from("<H", packet, 42)[0]
    span = (end - start) % 36000
    points = []
    for k in range(POINTS):
        dist, inten = struct.unpack_from("<HB", packet, 6 + 3 * k)
        angle = (start + span * k / (POINTS - 1)) / 100.0 % 360.0
        points.append((angle, dist / 1000.0, inten))
    return speed, points


class Ms200Node(Node):
    def __init__(self):
        super().__init__("ms200_node")
        self.port = self.declare_parameter("port", "/dev/lidar").value
        self.baud = self.declare_parameter("baud", 230400).value
        self.frame_id = self.declare_parameter("frame_id", "laser").value
        self.clockwise = self.declare_parameter("clockwise", True).value
        self.rays = self.declare_parameter("rays", 450).value
        self.range_min = self.declare_parameter("range_min", 0.10).value   # robot04 sees its own frame at 0.03-0.09 m
        self.range_max = self.declare_parameter("range_max", 12.0).value
        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.stats = {"scans": 0, "crc_errors": 0}
        self.running = True
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()
        self.create_timer(10.0, self.report)

    def report(self):
        self.get_logger().info("scans %d, crc errors %d" % (self.stats["scans"], self.stats["crc_errors"]))

    def reader(self):
        while self.running and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.5) as ser:
                    self.get_logger().info("reading %s at %d" % (self.port, self.baud))
                    self.read_loop(ser)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning("lidar port: %s, retrying in 2 s" % e)
                time.sleep(2.0)
            except Exception as e:
                if not self.running:
                    return
                self.get_logger().error("lidar reader: %r, restarting in 2 s" % e)
                time.sleep(2.0)

    def read_loop(self, ser):
        buf = bytearray()
        rev, last_angle, rev_start, speed = [], None, time.time(), 3600
        while self.running and rclpy.ok():
            chunk = ser.read(512)
            if not chunk:
                raise serial.SerialException("no data for 0.5 s")
            buf += chunk
            while len(buf) >= PACKET_LEN:
                i = buf.find(b"\x54\x2c")
                if i < 0:
                    del buf[:-1]
                    break
                if i:
                    del buf[:i]
                    continue
                if len(buf) < PACKET_LEN:
                    break
                result = parse(bytes(buf[:PACKET_LEN]))
                if result is None:
                    self.stats["crc_errors"] += 1
                    del buf[:1]
                    continue
                del buf[:PACKET_LEN]
                speed, points = result
                for angle, dist, inten in points:
                    if last_angle is not None and angle < last_angle - 180.0:   # wrapped past 0
                        now = time.time()
                        if len(rev) > self.rays // 2:
                            self.publish(rev, rev_start, now, speed)
                        rev, rev_start = [], now
                    rev.append((angle, dist, inten))
                    last_angle = angle

    def publish(self, rev, start, end, speed):
        n = self.rays
        step = 2.0 * math.pi / n
        ranges = [math.inf] * n
        intens = [0.0] * n
        for angle, dist, inten in rev:
            if not (self.range_min <= dist <= self.range_max):
                continue
            a = math.radians(-angle if self.clockwise else angle) % (2.0 * math.pi)
            j = int(round(a / step)) % n
            if dist < ranges[j]:
                ranges[j], intens[j] = dist, float(inten)
        msg = LaserScan()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rclpy.time.Time(seconds=start).to_msg()
        msg.angle_min = 0.0
        msg.angle_max = step * (n - 1)
        msg.angle_increment = step
        scan_time = 360.0 / speed if speed else end - start
        msg.scan_time = scan_time
        msg.time_increment = scan_time / n
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intens
        try:
            self.pub.publish(msg)
        except Exception:          # context shut down by Ctrl-C while the reader was mid-scan
            if rclpy.ok():
                raise
            self.running = False
            return
        self.stats["scans"] += 1


def main():
    rclpy.init()
    node = Ms200Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
