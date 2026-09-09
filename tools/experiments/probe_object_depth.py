#!/usr/bin/env python3
"""What the detector actually sees in the object depth band.

Written after a 30mm cube produced no blob at all while a 50mm one produced
1596 pixels. The arithmetic said the cube's top face sits at depth 0.750, but
arithmetic had already been wrong once here, so this reads the depth image.
"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self):
        super().__init__("object_depth_probe")
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, "/my_cam/depth", self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        img = np.nan_to_num(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"),
            nan=10.0).astype(np.float64)
        self.done = True
        print("frame %dx%d  finite range %.4f .. %.4f"
              % (img.shape[1], img.shape[0], img.min(), img[img < 5].max()))
        print("\ndepth histogram over 0.70 .. 0.82 (5mm bins):")
        for lo in np.arange(0.70, 0.82, 0.005):
            n = int(((img >= lo) & (img < lo + 0.005)).sum())
            if n:
                print("  %.3f - %.3f : %6d px %s"
                      % (lo, lo + 0.005, n, "#" * min(40, n // 40)))
        for lo, hi, label in ((0.710, 0.750, "old band [split, 0.75)"),
                              (0.710, 0.770, "new band [split, 0.77)"),
                              (0.745, 0.755, "around 0.750")):
            print("%-26s %6d px" % (label, int(((img >= lo) & (img < hi)).sum())))


def main():
    rclpy.init()
    node = Probe()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
