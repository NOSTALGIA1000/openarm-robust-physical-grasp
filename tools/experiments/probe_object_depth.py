#!/usr/bin/env python3
"""What the detector actually sees in the object depth band, and how much it
jitters.

Written after a 30mm cube produced no blob at all while a 50mm one produced
1596 pixels. The arithmetic said the cube's top face sits at depth 0.750, but
arithmetic had already been wrong once here, so this reads the depth image.

With --frames N it also measures the per-pixel temporal standard deviation over
N frames of a static scene. That number is what decides whether the simulated
noise sweep covers the real sensor: MuJoCo renders exact depth, so every band
and percentile in the detector was tuned against noise that does not exist. If
the measured sigma falls inside the swept range the sweep's conclusions carry
over; if it does not, the sweep has to be extended before trusting them.
"""
import argparse

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self, frames=1, band=(0.60, 0.82)):
        super().__init__("object_depth_probe")
        self.bridge = CvBridge()
        self.done = False
        self.frames = frames
        self.band = band
        self.stack = []
        self.create_subscription(Image, "/my_cam/depth", self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        if self.frames > 1:
            img = np.nan_to_num(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"),
                nan=10.0).astype(np.float64)
            self.stack.append(img)
            if len(self.stack) < self.frames:
                return
            self.done = True
            self.report_jitter()
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


    def report_jitter(self):
        stack = np.stack(self.stack)
        lo, hi = self.band
        # Only where the scene is, and only where every frame agreed there was
        # something - a pixel that flickers between object and background is a
        # dropout, not jitter, and is reported separately.
        inband = (stack > lo) & (stack < hi)
        always = inband.all(axis=0)
        sometimes = inband.any(axis=0)
        sigma = stack.std(axis=0)
        vals = sigma[always]
        print("%d frames, band %.2f .. %.2f" % (len(self.stack), lo, hi))
        print("  stable pixels        %7d" % int(always.sum()))
        print("  intermittent pixels  %7d  (in band in some frames only)"
              % int(sometimes.sum() - always.sum()))
        if vals.size == 0:
            print("  no stable in-band pixels - check the band")
            return
        print("  per-pixel temporal sigma over stable pixels:")
        for q in (50, 90, 99):
            print("    p%-3d %7.4f m  (%.2f mm)"
                  % (q, np.percentile(vals, q), np.percentile(vals, q) * 1000))
        print("    max  %7.4f m  (%.2f mm)" % (vals.max(), vals.max() * 1000))
        print()
        print("  仿真扫描覆盖 0 / 1 / 2 / 5 mm。若 p90 落在其中，")
        print("  stage48_depthnoise 的结论可直接引用；若超出 5 mm，需补扫。")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=1,
                    help="frames to average for temporal jitter (default 1: "
                         "single-frame histogram instead)")
    ap.add_argument("--band", type=float, nargs=2, default=(0.60, 0.82),
                    metavar=("LO", "HI"),
                    help="depth band to measure jitter over")
    args = ap.parse_args()
    rclpy.init()
    node = Probe(frames=args.frames, band=tuple(args.band))
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
