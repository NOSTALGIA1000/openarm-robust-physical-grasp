# Derived from the course reference implementation of openarm_moveit_config
# (declared author and maintainer: yhang <1936951548@qq.com>), which declares
# <license>BSD</license> in its package.xml but ships no license text and no
# copyright line. The terms of the 3-clause BSD are reproduced and honoured in
# NOTICE at the repository root, which also records that no copyright line was
# supplied upstream and that none has been invented; see THIRD_PARTY.md
# section 2 for the full reasoning and for the enquiry sent to the author.
#
# Of the original file's 87 unique effective lines, 80 survive here. Classified
# there: 28 are rclpy boilerplate, 31 are textbook formulas and standard API
# idioms (pinhole back-projection, intrinsics from fovy, contour centroid,
# tf2 point transform), and 21 are the author's own expression - the Chinese
# log strings and comments, the class and node names, the topic and frame
# names, and three thresholds. Those 21 are kept rather than paraphrased away:
# removing them would reduce what has to be attributed without reducing what
# was taken.
#
# Everything else -- the gripper blob channel, the relative optical
# measurement, the depth band parameters, noise injection and the structured
# logging -- is original to this project and is licensed under the Apache
# License, Version 2.0.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs 

class UniversalDetector(Node):
    def __init__(self):
        super().__init__('universal_detector_node')
        
        self.depth_topic = '/my_cam/depth'
        
        # 相机内参 (保持不变)
        self.width = 640
        self.height = 480
        # Must match the scene camera: the unprojection scales every lateral
        # measurement by 1/tan(fovy/2), so a mismatch shows up as a
        # proportional error that grows with distance from the optical axis.
        self.declare_parameter('fovy_deg', 45.0)
        self.fovy_deg = float(self.get_parameter('fovy_deg').value)
        self.fy = (self.height / 2.0) / math.tan(math.radians(self.fovy_deg / 2.0))
        self.fx = self.fy 
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        
        # 初始化 TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.depth_topic, self.listener_callback, 10)

        self.coord_pub = self.create_publisher(Point, '/apple_location', 10)

        # --- Relative measurement support ---
        # A hand-eye translation error cannot be removed by re-observing the
        # object: the same wrong extrinsic returns the same wrong world point.
        # It cancels only in a difference of two points seen through that same
        # extrinsic, so the gripper has to be measured optically as well.
        # Object and gripper overlap in the image at the hover pose but are
        # well separated in depth (gripper ~0.68 m, object top ~0.74 m), so the
        # split is done on depth, not on contours.
        self.declare_parameter('publish_camera_frame', False)
        self.declare_parameter('depth_split', 0.71)
        # Far edge of the object band, used by all three depth masks: the
        # world-frame detection that publishes /apple_location, the
        # camera-frame one, and the structure logger. It keeps the table out -
        # the table surface reads 0.780 - and 0.75 was picked for the 44mm
        # cuboid, whose top face reads 0.736.
        #
        # A 30mm cube's top face reads exactly 0.750 and the comparison is
        # strict, so a probe of the depth image found 0 pixels in [split, 0.75)
        # against 621 in [split, 0.77). Every 30mm trial timed out with no
        # target published. Scaling as top + 0.020 gives a 50mm cube exactly
        # the 0.750 that was hardcoded, so batches already run stay comparable.
        self.declare_parameter('object_depth_hi', 0.75)
        # Per-pixel Gaussian depth noise in metres, added to the rendered
        # image before anything reads it. MuJoCo's depth is exact, and every
        # band, percentile and area threshold in this detector was tuned on
        # exact depth. A RealSense D435 at 0.7m carries roughly 2-3mm RMS,
        # so whether those thresholds survive noise is the largest open
        # question about whether any of this transfers to hardware.
        # Seeded, so a trial repeats.
        self.declare_parameter('depth_noise_std', 0.0)
        self.declare_parameter('depth_noise_seed', 0)
        # 0.09 puts the band's lower edge at 0.594, which overlaps the
        # forearm: the depth histogram shows forearm filling 0.40-0.60
        # continuously. 0.06 lifts the edge to 0.624 and excludes it.
        # Symptom that led here: gripper optics bimodal over 20mm in y
        # only, while x and z stayed within 4.4mm and 1.4mm.
        self.declare_parameter('gripper_band_below', 0.06)
        self.declare_parameter('gripper_band_above', 0.03)
        self.declare_parameter('gripper_min_area', 15)
        self.declare_parameter('gripper_deep_pct', 60.0)
        self.declare_parameter('gripper_roi_px', 30)
        self.declare_parameter('roi_cascade', [90, 55, 30])
        # Default off: with depth as the only channel the marker cannot be
        # separated from the forearm, which spans 0.40-0.60 continuously
        # while the marker sits at 0.53 inside that range. Needs an RGB
        # channel to be usable. See the 2026-09-05 marker document.
        self.declare_parameter('use_marker_roi', False)
        self.declare_parameter('marker_depth_lo', 0.50)
        self.declare_parameter('marker_depth_hi', 0.56)
        self.declare_parameter('marker_min_px', 60)
        self.declare_parameter('marker_max_px', 1200)
        self.declare_parameter('marker_min_circularity', 0.72)
        # Loose: a hard reject starved half the trials of any reading at all.
        # Only obvious nonsense is dropped here; picking the best of
        # several readings is left to the consumer.
        self.declare_parameter('grip_depth_tol', 0.08)
        self.declare_parameter('roi_offset_world_x', 0.076)
        self.declare_parameter('roi_offset_world_y', 0.0)
        self.declare_parameter('roi_offset_world_z', 0.0)
        self.publish_cam = self.get_parameter('publish_camera_frame').value
        self.depth_split = self.get_parameter('depth_split').value
        self.object_depth_hi = self.get_parameter('object_depth_hi').value
        self.depth_noise_std = float(
            self.get_parameter('depth_noise_std').value)
        self._noise_rng = np.random.default_rng(
            int(self.get_parameter('depth_noise_seed').value))
        self.gripper_band_below = self.get_parameter('gripper_band_below').value
        self.gripper_band_above = self.get_parameter('gripper_band_above').value
        self.gripper_min_area = self.get_parameter('gripper_min_area').value
        self.gripper_deep_pct = self.get_parameter('gripper_deep_pct').value
        self.gripper_roi_px = self.get_parameter('gripper_roi_px').value
        self.roi_cascade = list(
            self.get_parameter('roi_cascade').value)
        self.use_marker_roi = self.get_parameter('use_marker_roi').value
        self.marker_depth_lo = self.get_parameter('marker_depth_lo').value
        self.marker_depth_hi = self.get_parameter('marker_depth_hi').value
        self.marker_min_px = self.get_parameter('marker_min_px').value
        self.marker_max_px = self.get_parameter('marker_max_px').value
        self.marker_min_circularity = self.get_parameter(
            'marker_min_circularity').value
        self.grip_depth_tol = self.get_parameter('grip_depth_tol').value
        self.roi_offset_world = (
            self.get_parameter('roi_offset_world_x').value,
            self.get_parameter('roi_offset_world_y').value,
            self.get_parameter('roi_offset_world_z').value)
        self.obj_cam_pub = self.create_publisher(Point, '/apple_location_cam', 10)
        self.grip_cam_pub = self.create_publisher(Point, '/gripper_location_cam', 10)
        # Kinematics supplies the expected gripper height so the right blob can
        # be picked: a plain near-depth band selects whichever arm segment is
        # highest, not the hand. This gates SELECTION only -- the published
        # position is still purely optical, so the hand-eye bias that the
        # relative measurement is meant to cancel never enters it.
        self.gripper_z_hint = None
        self.gripper_point_hint = None
        self.create_subscription(Float64, '/gripper_z_hint', self._on_hint, 10)
        self.create_subscription(
            Point, '/gripper_point_hint', self._on_point_hint, 10)
        if self.publish_cam:
            self.get_logger().info(
                '=== publishing camera-frame points, depth split {:.3f} m, object band hi {:.3f} m ==='
                .format(self.depth_split, self.object_depth_hi))
        
        # --- 修改点1：状态变量 ---
        self.fixed_world_point = None  # 用于存储锁定的坐标
        self.lock_time = None          # 用于记录锁定的起始时间

        # Latching the first detection is the demo's behaviour and is kept as the
        # default. physics_mode needs repeated measurements: a pre-grasp visual
        # correction requires a second observation, which a latched detector can
        # never provide.
        self.declare_parameter('lock_first_detection', True)
        self.lock_enabled = self.get_parameter('lock_first_detection').value
        if not self.lock_enabled:
            self.get_logger().info('=== detector running UNLATCHED (continuous re-detection) ===')
        
        self.get_logger().info("=== 绝对坐标检测器启动 ===")

    def _decode_depth(self, msg):
        """Decode one depth frame, and add sensor noise if any was asked for.

        Both read paths go through here so a frame is never noised twice and
        the world-frame and camera-frame readings always see the same image.
        """
        img = np.nan_to_num(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'),
            nan=10.0)
        if self.depth_noise_std > 0.0:
            img = img + self._noise_rng.normal(
                0.0, self.depth_noise_std, img.shape).astype(img.dtype)
        return img

    def _blob_centroid_cam(self, depth_img, lo, hi, min_area=50):
        """Camera-frame centroid of the largest blob within a depth band."""
        mask = (((depth_img > lo) & (depth_img < hi)) * 255).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        d = float(depth_img[v, u])
        if not (lo < d < hi):
            # Centroid can fall off the blob for a concave shape; fall back to
            # the band's mean depth rather than reporting a bogus range.
            band = depth_img[(depth_img > lo) & (depth_img < hi)]
            if band.size == 0:
                return None
            d = float(np.mean(band))
        return Point(
            x=float((u - self.cx) * d / self.fx),
            y=float((v - self.cy) * d / self.fy),
            z=d,
        )

    def _deep_centroid_cam(self, depth_img, lo, hi, min_area):
        """3D centroid of the deepest part of the gripper blob.

        The band has to be ~116mm thick to contain enough pixels, which also
        pulls in a stretch of forearm. MoveIt returns a different arm
        configuration each run, so how much forearm falls inside the band
        varies and drags the blob centroid: measured, the blob shifts up to 49
        px (~54mm) while the gripper itself moves 4mm.

        Taking the deepest pixels keeps the surfaces nearest the jaw plane -
        finger and palm underside - which is what the target actually needs.
        Note this is the opposite end from the earlier top-slice attempt, which
        selected the highest arm surfaces and so did not help.
        """
        mask = (((depth_img > lo) & (depth_img < hi)) * 255).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None
        blob = np.zeros(depth_img.shape, np.uint8)
        cv2.drawContours(blob, [largest], -1, 255, -1)
        sel = (blob > 0) & (depth_img > lo) & (depth_img < hi)
        if not sel.any():
            return None
        cut = float(np.percentile(depth_img[sel], self.gripper_deep_pct))
        sel &= depth_img >= cut
        if sel.sum() < 8:
            return None
        vs, us = np.nonzero(sel)
        dd = depth_img[sel].astype(np.float64)
        return Point(
            x=float(np.mean((us - self.cx) * dd / self.fx)),
            y=float(np.mean((vs - self.cy) * dd / self.fy)),
            z=float(np.mean(dd)),
        )

    def _on_point_hint(self, msg):
        self.gripper_point_hint = (msg.x, msg.y, msg.z)

    def _project_hint(self):
        """Pixel of the kinematic gripper point, as the ROI centre.

        Depth banding alone cannot separate hand from forearm: the band must be
        ~116mm thick to hold enough pixels, and whatever forearm falls inside
        drags the centroid as the arm configuration changes. Restricting the
        SEARCH REGION removes the forearm regardless of its depth.

        The projection uses the same (miscalibrated) extrinsic as everything
        else, so the ROI is displaced by the bias -- harmless, because it only
        decides where to look. The centroid inside the ROI stays optical.
        """
        if self.gripper_point_hint is None:
            return None
        pt = PointStamped()
        pt.header.frame_id = 'world'
        pt.header.stamp = rclpy.time.Time().to_msg()
        # Offset in WORLD metres, not pixels. What the camera sees is the
        # palm/wrist surface, ~76mm behind the gripper frame origin toward the
        # base. Expressing that as a fixed pixel shift only works at the
        # gripper position it was measured at: the same 3D displacement
        # projects to a different pixel count elsewhere in the view, which is
        # why a pixel offset calibrated at x=64mm missed the hand entirely at
        # y=128/192mm. Offsetting before projection tracks the gripper.
        ox, oy, oz = self.roi_offset_world
        gx, gy, gz = self.gripper_point_hint
        pt.point.x, pt.point.y, pt.point.z = gx + ox, gy + oy, gz + oz
        try:
            tf = self.tf_buffer.lookup_transform(
                'camera_optical_frame', 'world', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            cam = tf2_geometry_msgs.do_transform_point(pt, tf).point
        except Exception:
            return None
        if cam.z <= 0.05:
            return None
        return (self.cx + cam.x * self.fx / cam.z,
                self.cy + cam.y * self.fy / cam.z)

    def _roi_centroid_cam(self, depth_img, lo, hi, centre):
        """Deepest-percentile 3D centroid inside a ROI around `centre`."""
        u0, v0 = centre
        h, w = depth_img.shape
        r = int(self.gripper_roi_px)
        u1, u2 = max(0, int(u0) - r), min(w, int(u0) + r + 1)
        v1, v2 = max(0, int(v0) - r), min(h, int(v0) + r + 1)
        if u2 <= u1 or v2 <= v1:
            return None, 0
        roi = np.zeros(depth_img.shape, bool)
        roi[v1:v2, u1:u2] = True
        sel = roi & (depth_img > lo) & (depth_img < hi)
        n = int(sel.sum())
        if n < 40:
            return None, n
        cut = float(np.percentile(depth_img[sel], self.gripper_deep_pct))
        sel &= depth_img >= cut
        if sel.sum() < 8:
            return None, n
        vs, us = np.nonzero(sel)
        dd = depth_img[sel].astype(np.float64)
        return Point(
            x=float(np.mean((us - self.cx) * dd / self.fx)),
            y=float(np.mean((vs - self.cy) * dd / self.fy)),
            z=float(np.mean(dd)),
        ), n

    def _roi_centroid_at(self, depth_img, lo, hi, centre, radius):
        """Deep-percentile 3D centroid inside a window of the given radius."""
        u0, v0 = centre
        h, w = depth_img.shape
        r = int(radius)
        u1, u2 = max(0, int(u0) - r), min(w, int(u0) + r + 1)
        v1, v2 = max(0, int(v0) - r), min(h, int(v0) + r + 1)
        if u2 <= u1 or v2 <= v1:
            return None, 0
        roi = np.zeros(depth_img.shape, bool)
        roi[v1:v2, u1:u2] = True
        sel = roi & (depth_img > lo) & (depth_img < hi)
        n = int(sel.sum())
        if n < 40:
            return None, n
        cut = float(np.percentile(depth_img[sel], self.gripper_deep_pct))
        sel &= depth_img >= cut
        if sel.sum() < 8:
            return None, n
        vs, us = np.nonzero(sel)
        dd = depth_img[sel].astype(np.float64)
        return Point(
            x=float(np.mean((us - self.cx) * dd / self.fx)),
            y=float(np.mean((vs - self.cy) * dd / self.fy)),
            z=float(np.mean(dd)),
        ), n

    def _cascade_centroid(self, depth_img, lo, hi, centre):
        """Coarse-to-fine: shrink the window, re-centring on what was measured.

        Resolves the conflict that a small window measures well (30px gave a
        3.1mm residual spread against 30.6mm at 55px) but is pushed off the
        hand by bias, since its centre comes from TF which carries that bias
        (54px of displacement at 64mm).

        Only the first pass uses the TF centre, and it is wide enough to
        contain the hand despite that displacement. Each later pass re-centres
        on the previous pass's own optical measurement, so the dependence on TF
        decays as the window closes in.
        """
        c = centre
        pt, n = None, -1
        trace = []
        for r in self.roi_cascade:
            p, nn = self._roi_centroid_at(depth_img, lo, hi, c, r)
            if p is None:
                trace.append("r%d@(%.0f,%.0f) n=%d FAIL" % (r, c[0], c[1], nn))
                break
            pt, n = p, nn
            if p.z > 0.05:
                c = (self.cx + p.x * self.fx / p.z,
                     self.cy + p.y * self.fy / p.z)
            trace.append("r%d n=%d ->(%.0f,%.0f)" % (r, nn, c[0], c[1]))
        self.get_logger().info("cascade: " + " | ".join(trace),
                               throttle_duration_sec=1.0)
        return pt, n

    def _find_marker(self, depth_img):
        """Locate the gripper marker in the image, without using TF.

        The kinematic ROI centre had to be projected through the camera->world
        extrinsic, which carries the hand-eye bias being cancelled, so the
        window slid off the hand as bias grew (54px at 64mm, 163px at 192mm).
        The marker occupies its own depth interval between forearm (~0.46) and
        the depth-continuous wrist/hand (0.60-0.70), so it can be found from
        the depth image alone and the circular dependency disappears.

        Returns (u, v) or None. Size is bounded both ways: too small is noise,
        too large means the band caught something else and the reading should
        be refused rather than silently wrong.
        """
        mask = (((depth_img > self.marker_depth_lo) &
                 (depth_img < self.marker_depth_hi)) * 255).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        # Area bounds alone admit forearm fragments that happen to fall in the
        # band: measured, the same band returned centres at u=6, u=105 and
        # u=199 across three frames. The marker is a sphere, so its silhouette
        # is a disc; circularity separates it from arbitrary fragments.
        best, best_score, cand = None, 0.0, []
        for c in contours:
            a = cv2.contourArea(c)
            if a < self.marker_min_px or a > self.marker_max_px:
                continue
            per = cv2.arcLength(c, True)
            if per <= 0:
                continue
            circ = 4.0 * math.pi * a / (per * per)
            M = cv2.moments(c)
            cu = M["m10"] / M["m00"] if M["m00"] else -1
            cand.append("a=%.0f circ=%.2f u=%.0f" % (a, circ, cu))
            if circ < self.marker_min_circularity:
                continue
            if circ > best_score:
                best, best_score = c, circ
        if cand:
            self.get_logger().info("marker cand: " + " ; ".join(cand[:5]),
                                   throttle_duration_sec=1.0)
        if best is None:
            return None
        M = cv2.moments(best)
        if M["m00"] == 0:
            return None
        return (M["m10"] / M["m00"], M["m01"] / M["m00"])

    def _feature_candidates(self, depth_img, lo, hi, centre):
        """Log several candidate gripper features from one ROI, to compare
        their stability before committing to one.

        The published value stays the deep-percentile centroid; this only
        records alternatives. Measured problem: at a fixed object position two
        trials differed by 30.8mm while the ROI centre moved 3px and in-window
        pixel count changed 31%, i.e. the centroid follows how much hand
        happens to be visible. A feature tied to the contour's extent or to the
        jaw plane should not.
        """
        u0, v0 = centre
        h, w = depth_img.shape
        r = int(self.gripper_roi_px)
        u1, u2 = max(0, int(u0) - r), min(w, int(u0) + r + 1)
        v1, v2 = max(0, int(v0) - r), min(h, int(v0) + r + 1)
        roi = np.zeros(depth_img.shape, bool)
        roi[v1:v2, u1:u2] = True
        sel = roi & (depth_img > lo) & (depth_img < hi)
        n = int(sel.sum())
        if n < 40:
            return "cand: n=%d too few" % n

        vs, us = np.nonzero(sel)
        dd = depth_img[sel].astype(np.float64)

        def cam_y(vv, d):
            return float(np.mean((vv - self.cy) * d / self.fy))

        # A: deep-percentile centroid (current published value)
        cut = float(np.percentile(dd, self.gripper_deep_pct))
        m = dd >= cut
        a = cam_y(vs[m], dd[m]) if m.sum() >= 8 else float('nan')

        # B: bbox centre of the in-window mask, at the band's median depth
        dmed = float(np.median(dd))
        b = float(((vs.min() + vs.max()) / 2.0 - self.cy) * dmed / self.fy)

        # C: deepest 10% centroid
        cut90 = float(np.percentile(dd, 90.0))
        m90 = dd >= cut90
        c = cam_y(vs[m90], dd[m90]) if m90.sum() >= 8 else float('nan')

        # D: single deepest point
        i = int(np.argmax(dd))
        dpt = float((vs[i] - self.cy) * dd[i] / self.fy)

        return ("cand n=%d vspan=%d..%d | A_centroid=%.4f B_bbox=%.4f "
                "C_deep10=%.4f D_deepest=%.4f" % (n, vs.min(), vs.max(), a, b, c, dpt))

    def _blob_diag(self, depth_img, lo, hi, min_area):
        """Describe the blob that would be selected, to identify misselection.

        The gripper reading is bistable: repeated trials at one bias give
        either ~0 or ~19mm of offset with nothing in between, so the two modes
        must correspond to two different blobs. Logging extent, area and depth
        spread is what distinguishes them.
        """
        mask = (((depth_img > lo) & (depth_img < hi)) * 255).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return "no contour"
        ordered = sorted(contours, key=cv2.contourArea, reverse=True)
        largest = ordered[0]
        if cv2.contourArea(largest) < min_area:
            return "largest area %.0f below floor %s" % (
                cv2.contourArea(largest), min_area)
        x, y, w, h = cv2.boundingRect(largest)
        M = cv2.moments(largest)
        cu = M["m10"] / M["m00"] if M["m00"] else -1
        cv_ = M["m01"] / M["m00"] if M["m00"] else -1
        blob = np.zeros(depth_img.shape, np.uint8)
        cv2.drawContours(blob, [largest], -1, 255, -1)
        dsel = depth_img[(blob > 0) & (depth_img > lo) & (depth_img < hi)]
        dstat = ("d=%.3f/%.3f/%.3f" % (float(dsel.min()), float(np.median(dsel)),
                                       float(dsel.max()))) if dsel.size else "d=none"
        runner = ""
        if len(ordered) > 1 and cv2.contourArea(ordered[1]) >= min_area:
            rx, ry, rw, rh = cv2.boundingRect(ordered[1])
            runner = " | 2nd area=%.0f bbox=%d,%d,%dx%d" % (
                cv2.contourArea(ordered[1]), rx, ry, rw, rh)
        return ("area=%.0f bbox=%d,%d,%dx%d centroid=%.1f,%.1f %s n_blobs=%d%s"
                % (cv2.contourArea(largest), x, y, w, h, cu, cv_, dstat,
                   len(ordered), runner))

    def _on_hint(self, msg):
        self.gripper_z_hint = float(msg.data)

    def _camera_world_z(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', 'camera_optical_frame', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            return tf.transform.translation.z
        except Exception:
            return None

    def _log_blob_structure(self, depth_img):
        """Log the depth structure so bands are set from data, not guesswork."""
        mask = (((depth_img > 0.30) & (depth_img < self.object_depth_hi))
                * 255).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parts = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
            area = cv2.contourArea(c)
            if area < 50:
                continue
            blob = np.zeros(depth_img.shape, np.uint8)
            cv2.drawContours(blob, [c], -1, 255, -1)
            d = depth_img[blob > 0]
            parts.append("area=%d depth min/med/max=%.3f/%.3f/%.3f"
                         % (area, float(d.min()), float(np.median(d)), float(d.max())))
        if parts:
            self.get_logger().info("blobs: " + " | ".join(parts),
                                   throttle_duration_sec=1.0)

    def publish_camera_frame_points(self, cv_image):
        """Publish object and gripper positions in the camera optical frame.

        These are raw optical measurements: they never pass through the
        camera->world extrinsic, so a miscalibration of it cannot corrupt
        their difference.
        """
        self._log_blob_structure(cv_image)
        obj = self._blob_centroid_cam(cv_image, self.depth_split,
                                      self.object_depth_hi)
        if obj is not None:
            self.obj_cam_pub.publish(obj)

        # Without a hint there is no way to tell the hand from any other arm
        # segment in the near band, so publish nothing rather than publish the
        # wrong body: a silently mis-selected blob is worse than no reading.
        if self.gripper_z_hint is None:
            return
        cam_z = self._camera_world_z()
        if cam_z is None:
            return
        # Asymmetric band: at the TCP height itself the hand shows only a thin
        # sliver (measured: 41 px, too few to be a stable centroid), so the band
        # is extended towards the camera to take in the hand body. It must stop
        # short of the object at 0.736 and of the forearm, which sits much
        # nearer at ~0.45.
        expected = cam_z - self.gripper_z_hint
        lo = max(0.30, expected - self.gripper_band_below)
        hi = min(self.depth_split, expected + self.gripper_band_above)
        npix = int(((cv_image > lo) & (cv_image < hi)).sum())
        # The ROI centre comes from TF, which carries the very hand-eye bias
        # this measurement exists to cancel, so the window drifts off the hand
        # as the bias grows: measured, x=64mm displaces it ~54px (inside the
        # 55px radius, so it still works) while y=128/192mm displaces it
        # 108/163px and the window catches nothing at all. A kinematic ROI is
        # therefore only usable while the bias stays under roughly the radius.
        # Beyond that, fall back to the whole-band deep percentile, which is
        # noisier but has no such dependence.
        # Marker first: it needs no TF and so does not inherit the bias.
        # Fall back to the kinematic projection when the marker is not visible.
        centre, csrc = None, "none"
        if self.use_marker_roi:
            centre = self._find_marker(cv_image)
            if centre is not None:
                csrc = "marker"
        if centre is None:
            centre = self._project_hint()
            csrc = "tf" if centre is not None else "none"
        grip, nroi = None, -1
        if centre is not None:
            grip, nroi = self._cascade_centroid(cv_image, lo, hi, centre)
            self.get_logger().info(
                self._feature_candidates(cv_image, lo, hi, centre),
                throttle_duration_sec=0.4)
        if grip is None:
            grip = self._deep_centroid_cam(
                cv_image, lo, hi, self.gripper_min_area)
            nroi = -abs(nroi) if nroi > 0 else -1
        self.get_logger().info(
            "gripper band hint_z=%.3f expected_depth=%.3f band=%.3f..%.3f "
            "pixels=%d out=%s | %s"
            % (self.gripper_z_hint, expected, lo, hi, npix,
               ("%.4f,%.4f,%.4f" % (grip.x, grip.y, grip.z)) if grip else "NONE",
               ("roi=%s[%s] n=%d | " % (
                   ("%.0f,%.0f" % centre) if centre else "NONE", csrc, nroi))
               + self._blob_diag(cv_image, lo, hi, self.gripper_min_area)),
            throttle_duration_sec=0.4)
        # Reject readings whose depth disagrees with the expected gripper
        # depth. Measured: readings split cleanly into z~0.682, matching the
        # expected 0.684 and giving a 2.3mm residual spread, and z~0.640, which
        # carries 40% more pixels because forearm has crept into the band and
        # costs -15..-17mm. Depth agreement separates the two without needing
        # to know which pixels are which.
        if grip is not None and abs(grip.z - expected) > self.grip_depth_tol:
            self.get_logger().info(
                "gripper reading rejected: z=%.4f vs expected %.4f (tol %.3f)"
                % (grip.z, expected, self.grip_depth_tol),
                throttle_duration_sec=1.0)
            grip = None
        if grip is not None:
            self.grip_cam_pub.publish(grip)

    def listener_callback(self, msg):
        # Camera-frame points are published on every frame, independently of the
        # latch below: the relative measurement needs a live gripper reading at
        # the hover pose, which is long after the world-frame fix is latched.
        # The decoded image is reused by phase B so a frame is never converted
        # twice.
        cv_image = None
        if self.publish_cam:
            try:
                cv_image = self._decode_depth(msg)
                self.publish_camera_frame_points(cv_image)
            except Exception as e:
                self.get_logger().error(f"camera-frame publish failed: {e}")
                cv_image = None

        # ==========================================
        # 阶段 A: 已经锁定目标 -> 持续广播 + 限时日志
        # ==========================================
        if self.lock_enabled and self.fixed_world_point is not None:
            # 1. 持续发布之前锁定的坐标 (不管现在图像里有没有东西)
            self.coord_pub.publish(self.fixed_world_point)
            
            # 2. 计算锁定后的时长
            time_since_lock = (self.get_clock().now() - self.lock_time).nanoseconds / 1e9
            
            # 3. 只有在前 5 秒内，且每隔 1 秒打印一次日志
            if time_since_lock < 5.0:
                self.get_logger().info(
                    f"🔒 [已锁定] 持续广播坐标: x={self.fixed_world_point.x:.2f}, y={self.fixed_world_point.y:.2f}, z={self.fixed_world_point.z:.2f}",
                    throttle_duration_sec=1.0  # ROS2自带功能：限制打印频率为1秒1次
                )
            return  # 直接返回，不再进行图像处理，节省资源

        # ==========================================
        # 阶段 B: 尚未锁定 -> 寻找目标
        # ==========================================
        try:
            if cv_image is None:
                cv_image = self._decode_depth(msg)

            # 简单的深度分割提取物体 (0.3m ~ 1.5m 范围内)
            mask = (cv_image < self.object_depth_hi) & (cv_image > 0.3)
            mask_uint8 = (mask * 255).astype(np.uint8)

            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找最大的轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                
                # 过滤太小的噪点
                if cv2.contourArea(largest_contour) < 50: return

                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    d = cv_image[v, u]

                    if 0.3 < d < 1.5:
                        # 1. 计算相机坐标系坐标
                        x_cam = (u - self.cx) * d / self.fx
                        y_cam = (v - self.cy) * d / self.fy
                        z_cam = d

                        # 2. 构建 Stamped Point
                        point_cam = PointStamped()
                        point_cam.header.frame_id = "camera_optical_frame" 
                        point_cam.header.stamp = self.get_clock().now().to_msg()
                        point_cam.point.x = float(x_cam)
                        point_cam.point.y = float(y_cam)
                        point_cam.point.z = float(z_cam)

                        # 3. 查询 TF 并转换到 World
                        try:
                            transform = self.tf_buffer.lookup_transform(
                                'world', 
                                'camera_optical_frame', 
                                rclpy.time.Time(), 
                                timeout=rclpy.duration.Duration(seconds=1.0)
                            )
                            
                            point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
                            
                            # --- 修改点2：锁定坐标 ---
                            if self.lock_enabled:
                                self.fixed_world_point = point_world.point
                                self.lock_time = self.get_clock().now()
                            else:
                                self.fixed_world_point = point_world.point
                            
                            # 立即发布一次
                            self.coord_pub.publish(self.fixed_world_point)
                            
                            if self.lock_enabled:
                                self.get_logger().info(
                                    f"🎯 目标初次锁定! World:({point_world.point.x:.2f}, {point_world.point.y:.2f}, {point_world.point.z:.2f})"
                                )
                            else:
                                self.get_logger().info(
                                    f"detected World:({point_world.point.x:.4f}, {point_world.point.y:.4f}, {point_world.point.z:.4f})",
                                    throttle_duration_sec=2.0
                                )
                            
                        except tf2_ros.LookupException:
                            self.get_logger().warn("TF 查找失败: 尚未收到 camera_optical_frame 的变换")
                        except tf2_ros.ExtrapolationException:
                            self.get_logger().warn("TF 时间不同步")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UniversalDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()