"""Depth histogram captured when the arm is actually in frame.

The first attempt sampled a fixed frame index and caught the scene before the
arm rose over the table: only object (0.72-0.74) and table (0.78-0.80) were
present, which says nothing about whether the marker renders. This waits until
the near range is populated, i.e. the arm has entered the view.
"""
import rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Hist(Node):
    def __init__(self):
        super().__init__("depth_hist")
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, "/my_cam/depth", self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        img = np.nan_to_num(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"), nan=10.0)
        near = int(((img > 0.40) & (img < 0.72)).sum())
        if near < 800:            # arm not in frame yet
            return
        self.done = True
        print("=== arm in frame, near-range px=%d ===" % near, flush=True)
        for lo in np.arange(0.40, 0.76, 0.02):
            c = int(((img > lo) & (img <= lo + 0.02)).sum())
            if c:
                print("  %.2f-%.2f : %6d %s" % (lo, lo + 0.02, c,
                                                "#" * min(50, c // 30)), flush=True)
        print("=== round blobs anywhere in 0.42-0.70 ===", flush=True)
        for lo in np.arange(0.42, 0.70, 0.04):
            m = (((img > lo) & (img < lo + 0.06)) * 255).astype(np.uint8)
            cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in sorted(cs, key=cv2.contourArea, reverse=True)[:3]:
                a = cv2.contourArea(c)
                if a < 80:
                    continue
                per = cv2.arcLength(c, True)
                circ = 4 * np.pi * a / (per * per) if per else 0
                M = cv2.moments(c)
                if circ > 0.6:      # disc-like: candidate marker
                    print("  band %.2f-%.2f area=%.0f circ=%.2f centre=(%.0f,%.0f)"
                          % (lo, lo + 0.06, a, circ,
                             M["m10"] / M["m00"], M["m01"] / M["m00"]), flush=True)
        raise SystemExit


rclpy.init()
try:
    rclpy.spin(Hist())
except SystemExit:
    pass
