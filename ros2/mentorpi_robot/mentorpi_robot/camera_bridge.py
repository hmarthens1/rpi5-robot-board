#!/usr/bin/env python3
"""
camera_bridge - the Aurora 930 depth camera in the fleet's vision interface.

The Deptrum driver (vendor, not in this repo) runs in namespace /<robot>/camera and
publishes camera/rgb/image_raw (bgr8) and camera/depth/image_raw (16UC1, millimetres),
aligned to each other. This node turns them into what robot-fleet-gui's Vision tab
and the voice agent already use:

  out vision/image/compressed  sensor_msgs/CompressedImage  JPEG, 320 px wide, 3 Hz,
                               only while somebody subscribes (SensorDataQoS)
  out vision/state             std_msgs/String JSON, 2 Hz:
                               {"camera": "aurora930", "image": "rgb"|"depth",
                                "center_m": distance straight ahead (median of a centre patch),
                                "nearest_m": nearest valid depth in the lower 2/3 of the view,
                                "nearest_x": where it is, -1 (left) .. 1 (right)}
  in  vision/control           std_msgs/String JSON {"image": "rgb"|"depth"|"raw"}
"""
import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


def to_numpy(msg):
    if msg.encoding in ("bgr8", "rgb8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a if msg.encoding == "bgr8" else a[:, :, ::-1]
    if msg.encoding in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
    raise ValueError("unsupported encoding " + msg.encoding)


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        self.width = self.declare_parameter("width", 320).value
        self.rate = self.declare_parameter("rate", 3.0).value
        self.quality = self.declare_parameter("jpeg_quality", 70).value
        self.max_depth_mm = self.declare_parameter("max_depth_mm", 4000).value
        self.mode = "rgb"
        self.lock = threading.Lock()
        self.rgb = self.depth = None
        self.rgb_stamp = self.depth_stamp = 0.0
        self.img_pub = self.create_publisher(CompressedImage, "vision/image/compressed", qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, "vision/state", 10)
        self.create_subscription(Image, "camera/rgb/image_raw", self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(String, "vision/control", self.on_control, 10)
        self.create_timer(1.0 / self.rate, self.publish_image)
        self.create_timer(0.5, self.publish_state)

    def on_rgb(self, msg):
        with self.lock:
            self.rgb, self.rgb_stamp = msg, time.monotonic()

    def on_depth(self, msg):
        with self.lock:
            self.depth, self.depth_stamp = msg, time.monotonic()

    def on_control(self, msg):
        try:
            c = json.loads(msg.data)
        except ValueError:
            return
        img = c.get("image")
        if img in ("rgb", "raw", "annotated"):
            self.mode = "rgb"
        elif img in ("depth", "mask"):
            self.mode = "depth"

    def publish_image(self):
        if self.img_pub.get_subscription_count() == 0:
            return
        with self.lock:
            src = self.depth if self.mode == "depth" else self.rgb
        if src is None:
            return
        a = to_numpy(src)
        if self.mode == "depth":
            d = np.clip(a.astype(np.float32) / self.max_depth_mm, 0, 1)
            a = cv2.applyColorMap((255 - d * 255).astype(np.uint8), cv2.COLORMAP_JET)
            a[src_invalid(src)] = 0
        h = int(a.shape[0] * self.width / a.shape[1])
        small = cv2.resize(a, (self.width, h), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, int(self.quality)])
        if not ok:
            return
        m = CompressedImage()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "camera"
        m.format, m.data = "jpeg", jpg.tobytes()
        self.img_pub.publish(m)

    def publish_state(self):
        with self.lock:
            depth, stamp = self.depth, self.depth_stamp
        state = {"camera": "aurora930", "image": self.mode, "center_m": None, "nearest_m": None, "nearest_x": None,
                 "ok": depth is not None and time.monotonic() - stamp < 1.0}
        if state["ok"]:
            d = to_numpy(depth)
            h, w = d.shape
            patch = d[h * 2 // 5:h * 3 // 5, w * 2 // 5:w * 3 // 5]
            valid = patch[patch > 0]
            if valid.size > 20:
                state["center_m"] = round(float(np.median(valid)) / 1000.0, 3)
            low = d[h // 3:, :].astype(np.float32)
            low[low == 0] = np.inf
            col_min = low.min(axis=0)
            i = int(np.argmin(col_min))
            if np.isfinite(col_min[i]):
                state["nearest_m"] = round(float(col_min[i]) / 1000.0, 3)
                state["nearest_x"] = round(2.0 * i / (w - 1) - 1.0, 2)
        self.state_pub.publish(String(data=json.dumps(state)))


def src_invalid(msg):
    return to_numpy(msg) == 0


def main():
    rclpy.init()
    node = CameraBridge()
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
