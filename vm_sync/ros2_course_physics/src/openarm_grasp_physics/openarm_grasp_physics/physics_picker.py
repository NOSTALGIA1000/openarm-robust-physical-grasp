#!/usr/bin/env python3
"""physics_mode single-trial grasp state machine.

There is no software attachment and no object pose publisher here. The object
is a MuJoCo free body; if it moves with the gripper, contact moved it.

All waits are measured in simulation time taken from the telemetry stream,
because the VM real-time factor is well below 1.0 and wall-clock sleeps are
therefore not evidence windows.
"""

import dataclasses
import math
import random
import sys
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, Header

from openarm_grasp_msgs.msg import GraspRawState
from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.contact import summarize_contacts
from openarm_grasp_physics.evaluation import (
    FailureCategory,
    TrialObservations,
    classify_trial,
)
from openarm_grasp_physics.geometry import distance, elevation, relative_position
from openarm_grasp_physics.trial_record import Sample, TrialRecorder

GROUP = "left_arm"
EE_LINK = "openarm_left_hand_tcp"
GRIPPER_ACTION = "/left_hand_controller/gripper_cmd"
PROTECTION_ID = "grasp_protection_sphere"
PROTECTION_RADIUS = 0.06


class PhysicsPicker(Node):
    def __init__(self):
        super().__init__("physics_picker")

        self.declare_parameter("trial_id", "trial_000")
        self.declare_parameter("calib_bias_x", 0.0)
        self.declare_parameter("calib_bias_y", 0.0)
        self.declare_parameter("vision_noise_std", 0.001)
        self.declare_parameter("visual_correction", False)
        self.declare_parameter("use_detector_target", False)
        self.declare_parameter("detector_z_bias", 0.0275)
        self.declare_parameter("pregrasp_redetect", False)
        self.declare_parameter("relative_correction", False)
        self.declare_parameter("rel_offset_x", 0.0)
        self.declare_parameter("rel_offset_y", 0.0)
        self.declare_parameter("rehover_after_correction", True)
        self.declare_parameter("hover_yaw_tolerance", 0.01)
        self.declare_parameter(
            "output_dir", "/home/nostalgia/ros2_ws_course_repro/results/physics_grasp"
        )

        self.config = dataclasses.replace(
            GraspConfig(),
            calib_bias_x=self.get_parameter("calib_bias_x").value,
            calib_bias_y=self.get_parameter("calib_bias_y").value,
            vision_noise_std=self.get_parameter("vision_noise_std").value,
            visual_correction=self.get_parameter("visual_correction").value,
            use_detector_target=self.get_parameter("use_detector_target").value,
            detector_z_bias=self.get_parameter("detector_z_bias").value,
            pregrasp_redetect=self.get_parameter("pregrasp_redetect").value,
            relative_correction=self.get_parameter("relative_correction").value,
            rel_offset_x=self.get_parameter("rel_offset_x").value,
            rel_offset_y=self.get_parameter("rel_offset_y").value,
            rehover_after_correction=self.get_parameter(
                "rehover_after_correction").value,
            hover_yaw_tolerance=self.get_parameter("hover_yaw_tolerance").value,
        )
        self.trial_id = self.get_parameter("trial_id").value
        self.output_dir = self.get_parameter("output_dir").value
        self.recorder = TrialRecorder(self.output_dir, self.trial_id, self.config)

        self._lock = threading.Lock()
        self._latest = None            # last GraspRawState
        self._stage = "startup"
        self._baseline_z = None
        self._grasp_relative = None

        self._max_elevation = 0.0
        self._max_drift = 0.0
        self._metrics_frozen = False
        self._any_contact = False
        self._bilateral_confirmed = False
        self._contact_lost_during_lift = False
        self._hold_total = 0
        self._hold_bilateral = 0

        self._target = None
        self._target_latest = None
        self._target_seq = 0
        self._obj_cam = None
        self._grip_cam = None
        self._cam_seq = 0
        self.create_subscription(Point, "/apple_location", self._on_target, 10)
        self.create_subscription(Point, "/apple_location_cam", self._on_obj_cam, 10)
        self.create_subscription(
            Point, "/gripper_location_cam", self._on_grip_cam, 10)
        self.create_subscription(
            GraspRawState, "/grasp_raw_state", self._on_telemetry, 50
        )
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        # Expected gripper height, so the detector can tell the hand from the
        # rest of the arm in the depth image. Selection aid only; the detector's
        # published position stays optical.
        self.hint_pub = self.create_publisher(Float64, "/gripper_z_hint", 10)
        # Full gripper position, so the detector can project it and search
        # only a small region around it. Same principle as the z hint: this
        # picks WHERE to look, never what the measurement reads.
        self.point_hint_pub = self.create_publisher(
            Point, "/gripper_point_hint", 10)

        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

    # ---------------- callbacks ----------------

    def _on_target(self, msg):
        if abs(msg.x) < 0.01 and abs(msg.y) < 0.01:
            return
        with self._lock:
            if self._target is None:
                self._target = (msg.x, msg.y, msg.z)
            # The first fix stays pinned above so the open-loop target cannot
            # drift mid-trial. Pre-grasp re-detection reads the latest fix
            # instead, and uses the sequence counter to distinguish a genuinely
            # fresh observation from a stale one.
            self._target_latest = (msg.x, msg.y, msg.z)
            self._target_seq += 1

    def _on_obj_cam(self, msg):
        with self._lock:
            self._obj_cam = (msg.x, msg.y, msg.z)

    def _on_grip_cam(self, msg):
        with self._lock:
            self._grip_cam = (msg.x, msg.y, msg.z)
            self._cam_seq += 1

    def _wait_relative_pair(self, timeout_s):
        """Wait for a gripper reading paired with an object reading.

        The gripper is only visible once the arm is at the hover pose, so this
        gates on the gripper stream and takes the object reading alongside it.
        """
        with self._lock:
            seq0 = self._cam_seq
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                if self._cam_seq > seq0 and self._obj_cam and self._grip_cam:
                    return self._obj_cam, self._grip_cam
            time.sleep(0.05)
        return None, None

    def _wait_fresh_target(self, timeout_s):
        """Block until the detector publishes a new fix. None on timeout."""
        with self._lock:
            seq0 = self._target_seq
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                if self._target_seq > seq0:
                    return self._target_latest
            time.sleep(0.05)
        return None

    def _on_telemetry(self, msg):
        summary = summarize_contacts(
            msg.contact_geoms, msg.contact_normal_forces, self.config
        )
        object_xyz = (
            msg.object_pose.position.x,
            msg.object_pose.position.y,
            msg.object_pose.position.z,
        )
        gripper_xyz = (
            msg.gripper_pose.position.x,
            msg.gripper_pose.position.y,
            msg.gripper_pose.position.z,
        )
        gripper_quat = (
            msg.gripper_pose.orientation.w,
            msg.gripper_pose.orientation.x,
            msg.gripper_pose.orientation.y,
            msg.gripper_pose.orientation.z,
        )
        relative = relative_position(object_xyz, gripper_xyz, gripper_quat)

        with self._lock:
            self._latest = msg
            stage = self._stage

            height = (
                elevation(object_xyz[2], self._baseline_z)
                if self._baseline_z is not None
                else 0.0
            )
            drift = (
                distance(relative, self._grasp_relative)
                if self._grasp_relative is not None
                else 0.0
            )
            self._relative = relative

            if summary.left_contact or summary.right_contact:
                self._any_contact = True
            if not self._metrics_frozen:
                if height > self._max_elevation:
                    self._max_elevation = height
                if self._grasp_relative is not None and drift > self._max_drift:
                    self._max_drift = drift
            if stage == "lift" and self._bilateral_confirmed and not summary.bilateral:
                self._contact_lost_during_lift = True
            if stage == "hold":
                self._hold_total += 1
                if summary.bilateral:
                    self._hold_bilateral += 1

            self._summary = summary
            self.recorder.add_sample(
                Sample(
                    sim_time=msg.sim_time,
                    stage=stage,
                    left_contact=summary.left_contact,
                    right_contact=summary.right_contact,
                    left_force=summary.left_normal_force,
                    right_force=summary.right_normal_force,
                    object_x=object_xyz[0],
                    object_y=object_xyz[1],
                    object_z=object_xyz[2],
                    object_qw=msg.object_pose.orientation.w,
                    object_qx=msg.object_pose.orientation.x,
                    object_qy=msg.object_pose.orientation.y,
                    object_qz=msg.object_pose.orientation.z,
                    gripper_x=gripper_xyz[0],
                    gripper_y=gripper_xyz[1],
                    gripper_z=gripper_xyz[2],
                    elevation=height,
                    drift=drift,
                )
            )

    # ---------------- simulation-time helpers ----------------

    def sim_time(self):
        with self._lock:
            return self._latest.sim_time if self._latest is not None else None

    def wait_for_telemetry(self, wall_timeout=60.0):
        deadline = time.time() + wall_timeout
        while time.time() < deadline:
            if self.sim_time() is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_sim(self, duration_s, wall_timeout=300.0):
        start = self.sim_time()
        if start is None:
            return False
        deadline = time.time() + wall_timeout
        while time.time() < deadline:
            now = self.sim_time()
            if now is not None and now - start >= duration_s:
                return True
            time.sleep(0.01)
        return False

    def set_stage(self, stage):
        with self._lock:
            self._stage = stage
        now = self.sim_time()
        if now is not None:
            with self._lock:
                self.recorder.mark_stage(stage, now)
        self.get_logger().info("stage -> {}".format(stage))

    # ---------------- motion primitives ----------------

    def move_to(self, x, y, z, quat, tolerance=0.01, velocity_scale=0.05,
                acceleration_scale=0.05, yaw_tolerance=None):
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity_scale
        goal.request.max_acceleration_scaling_factor = acceleration_scale

        constraints = Constraints()
        constraints.name = "physics_pose_goal"

        position = PositionConstraint()
        position.header.frame_id = "world"
        position.link_name = EE_LINK
        position.weight = 1.0
        volume = BoundingVolume()
        volume.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[tolerance] * 3)
        )
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        volume.primitive_poses.append(pose.pose)
        position.constraint_region = volume
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = "world"
        orientation.link_name = EE_LINK
        (
            orientation.orientation.x,
            orientation.orientation.y,
            orientation.orientation.z,
            orientation.orientation.w,
        ) = quat
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        # Rotation about the tool axis barely affects a parallel jaw: the jaws
        # are symmetric and the object self-aligns under load (Stage 2). Keeping
        # it pinned to 0.01 rad costs reachable states for no benefit, and at
        # the workspace edge that is what makes IK return nothing at all.
        # x/y stay tight -- the tool must still point down.
        orientation.absolute_z_axis_tolerance = (
            0.01 if yaw_tolerance is None else yaw_tolerance)
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)

        goal.request.goal_constraints.append(constraints)

        future = self.move_client.send_goal_async(goal)
        if not self._spin_future(future, self.config.plan_timeout_s * 20):
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MoveIt rejected the goal")
            return False
        result_future = handle.get_result_async()
        if not self._spin_future(result_future, self.config.plan_timeout_s * 20):
            self.get_logger().error(
                "move_to timed out waiting for the result at ({:.4f}, {:.4f}, "
                "{:.4f})".format(x, y, z))
            return False
        code = result_future.result().result.error_code.val
        if code != 1:
            # MoveIt's "Completed trajectory execution with status SUCCEEDED"
            # comes from the execution manager and is not the action's verdict;
            # a run can execute fully and still return GOAL_TOLERANCE_VIOLATED
            # (-6) or CONTROL_FAILED (-4). Without the code the two are
            # indistinguishable in the logs.
            self.get_logger().error(
                "move_to failed with MoveIt error_code {} at ({:.4f}, {:.4f}, "
                "{:.4f}) tol={:.4f}".format(code, x, y, z, tolerance))
        return code == 1

    def send_gripper(self, position, max_effort):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        future = self.gripper_client.send_goal_async(goal)
        if not self._spin_future(future, self.config.gripper_action_timeout_s * 20):
            return False
        handle = future.result()
        if not handle.accepted:
            return False
        result_future = handle.get_result_async()
        # A stalled gripper is the expected outcome when squeezing an object,
        # so a timeout here is not by itself a command failure.
        self._spin_future(result_future, self.config.gripper_action_timeout_s * 20)
        return True

    def _spin_future(self, future, wall_timeout):
        deadline = time.time() + wall_timeout
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return False

    def remove_protection_sphere(self):
        obj = CollisionObject()
        obj.header = Header(frame_id="world", stamp=self.get_clock().now().to_msg())
        obj.id = PROTECTION_ID
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True
        self.scene_pub.publish(scene)

    # ---------------- trial ----------------

    def run_trial(self):
        planning_ok = True
        gripper_ok = True
        timed_out = False

        if not self.wait_for_telemetry():
            self.get_logger().error("no /grasp_raw_state; is physics_mode running?")
            return self._finish(False, False, True)

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.set_stage("settle")
        if not self.wait_sim(self.config.settle_duration_s):
            timed_out = True
        with self._lock:
            self._baseline_z = self._latest.object_pose.position.z
        self.get_logger().info("settled baseline z = {:.4f}".format(self._baseline_z))

        deadline = time.time() + self.config.detector_wait_s
        while self._target is None and time.time() < deadline:
            time.sleep(0.1)
        detector_seen = self._target is not None
        if not detector_seen:
            self.get_logger().warning(
                "no observation on /apple_location within {:.0f}s; proceeding on "
                "simulator ground truth (Stage 1 tests mechanics, not perception)"
                .format(self.config.detector_wait_s))

        # Stage 1 isolates grasp mechanics from perception. The detector's
        # observation is still required as a task trigger above, but the grasp
        # target itself uses simulator ground truth so that lateral alignment
        # error does not confound the contact result. Stage 2 deliberately
        # re-injects calibration and vision error on top of this baseline.
        with self._lock:
            true_x = self._latest.object_pose.position.x
            true_y = self._latest.object_pose.position.y
            true_z = self._latest.object_pose.position.z
        # true_x/true_y are rebound further down to mean "target before the
        # synthetic Stage 2 bias", so keep the simulator's actual object pose
        # under names that are never shadowed.
        gt_x, gt_y = true_x, true_y

        if self.config.use_detector_target:
            if not detector_seen:
                self.get_logger().error(
                    "use_detector_target is set but the detector never reported; "
                    "refusing to silently fall back to ground truth")
                return self._finish(True, gripper_ok, True, detector_ok=False)
            dx, dy, dz = self._target
            x, y = dx, dy
            obj_z = dz - self.config.detector_z_bias
            self.get_logger().info(
                "vision target x={:.4f} y={:.4f} z={:.4f} (detector z {:.4f} "
                "minus calibrated bias {:.4f}); ground truth was x={:.4f} "
                "y={:.4f} z={:.4f}, so residual error is x{:+.4f} y{:+.4f} z{:+.4f}"
                .format(x, y, obj_z, dz, self.config.detector_z_bias,
                        true_x, true_y, true_z,
                        x - true_x, y - true_y, obj_z - true_z))
        else:
            x, y, obj_z = true_x, true_y, true_z

        # Compensate the jaw-centre offset. The grasp orientation is a fixed
        # 180 deg rotation about y, i.e. diag(-1, 1, -1), which maps the TCP
        # local offset (0, +y, +z) to (0, +y, -z) in world, so the TCP target
        # is shifted by the negative of that.
        x = x - self.config.jaw_offset_x
        y = y - self.config.jaw_offset_y

        # Stage 2: the observation the picker acts on is the true pose seen
        # through a miscalibrated transform, plus sensor noise.
        rng = random.Random(hash(self.trial_id) & 0xffffffff)
        true_x, true_y = x, y
        x = true_x + self.config.calib_bias_x + rng.gauss(0.0, self.config.vision_noise_std)
        y = true_y + self.config.calib_bias_y + rng.gauss(0.0, self.config.vision_noise_std)
        self.get_logger().info(
            "observed target x={:.4f} y={:.4f} (bias {:+.4f},{:+.4f}, noise sd {:.4f})"
            .format(x, y, self.config.calib_bias_x, self.config.calib_bias_y,
                    self.config.vision_noise_std))
        self._true_xy = (true_x, true_y)
        self._rng = rng
        if detector_seen:
            self.get_logger().info(
                "ground-truth grasp target: x={:.4f} y={:.4f} (detector gave "
                "x={:.4f} y={:.4f})".format(x, y, self._target[0], self._target[1]))
        else:
            self.get_logger().info(
                "ground-truth grasp target: x={:.4f} y={:.4f} (no detector fix)"
                .format(x, y))

        quat = (0.0, 1.0, 0.0, 0.0)  # tool z pointing down
        hover_z = obj_z + self.config.hover_offset_m
        grasp_z = obj_z + self.config.jaw_offset_z

        self.set_stage("hover")
        if not self.move_to(x, y, hover_z, quat, tolerance=0.005,
                            yaw_tolerance=self.config.hover_yaw_tolerance):
            planning_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        if self.config.relative_correction:
            # Tell the detector where the hand is so it can pick the hand's blob
            # rather than a higher arm segment. This is a selection aid: the
            # measured position it publishes is still purely optical.
            hint = Float64()
            hint.data = float(hover_z)
            for _ in range(12):
                self.hint_pub.publish(hint)
                with self._lock:
                    gp = self._latest.gripper_pose.position
                    ph = Point(x=gp.x, y=gp.y, z=gp.z)
                self.point_hint_pub.publish(ph)
                time.sleep(0.1)

            obj_cam, grip_cam = self._wait_relative_pair(
                self.config.redetect_timeout_s)
            if obj_cam is None:
                self.get_logger().error(
                    "relative_correction is set but no paired object/gripper "
                    "observation arrived within {:.1f}s (needs "
                    "DETECTOR_CAM_FRAME=1)".format(self.config.redetect_timeout_s))
                return self._finish(planning_ok, gripper_ok, True,
                                    detector_ok=False)

            with self._lock:
                gx = self._latest.gripper_pose.position.x
                gy = self._latest.gripper_pose.position.y
                gz = self._latest.gripper_pose.position.z
                gq = self._latest.gripper_pose.orientation
            # Gripper orientation, to test whether the outlier readings coincide
            # with an unusual arm configuration.
            self.get_logger().info(
                "gripper pose z={:.4f} quat=({:.4f},{:.4f},{:.4f},{:.4f})"
                .format(gz, gq.x, gq.y, gq.z, gq.w))

            # camera_optical -> world is a 180 deg rotation about x, i.e.
            # diag(1, -1, -1); world -> camera_base_link carries no rotation.
            rel_x = obj_cam[0] - grip_cam[0]
            rel_y = -(obj_cam[1] - grip_cam[1])

            new_x = gx + rel_x - self.config.rel_offset_x - self.config.jaw_offset_x
            new_y = gy + rel_y - self.config.rel_offset_y - self.config.jaw_offset_y
            self.get_logger().info(
                "raw cam readings: obj=({:.4f},{:.4f},{:.4f}) grip=({:.4f},{:.4f},{:.4f})"
                .format(*obj_cam, *grip_cam))
            self.get_logger().info(
                "relative correction -> x={:.4f} y={:.4f} (was x={:.4f} y={:.4f}); "
                "gripper_kin x={:.4f} y={:.4f}, optical rel x{:+.4f} y{:+.4f}; "
                "ground truth x={:.4f} y={:.4f}, so residual error is "
                "x{:+.4f} y{:+.4f}".format(
                    new_x, new_y, x, y, gx, gy, rel_x, rel_y, gt_x, gt_y,
                    new_x - gt_x, new_y - gt_y))
            x, y = new_x, new_y

            # Re-hover over the corrected point before descending. Without this
            # the arm is still parked at the biased pose and the descent becomes
            # a diagonal sweep that drags the open jaws sideways at low
            # altitude, striking the object -- the unilateral_contact mode from
            # Stage 2. Correcting the target does not by itself correct the
            # path taken to it.
            if self.config.rehover_after_correction:
                if not self.move_to(x, y, hover_z, quat, tolerance=0.005):
                    planning_ok = False
                    return self._finish(planning_ok, gripper_ok, timed_out)

        if self.config.pregrasp_redetect:
            # Genuine second observation from the hover pose, as opposed to the
            # idealised cancellation below. Note what this can and cannot do:
            # the object is re-observed through the SAME miscalibrated
            # extrinsic, so a hand-eye translation error reappears identically.
            # Only a measurement that also sees the gripper optically cancels
            # common-mode bias.
            fresh = self._wait_fresh_target(self.config.redetect_timeout_s)
            if fresh is None:
                self.get_logger().error(
                    "pregrasp_redetect is set but no fresh observation arrived "
                    "within {:.1f}s; is the detector latched? (needs "
                    "lock_first_detection:=false)".format(
                        self.config.redetect_timeout_s))
                return self._finish(planning_ok, gripper_ok, True)
            rx, ry, _rz = fresh
            new_x = rx - self.config.jaw_offset_x
            new_y = ry - self.config.jaw_offset_y
            self.get_logger().info(
                "pre-grasp re-detection -> x={:.4f} y={:.4f} (was x={:.4f} "
                "y={:.4f}, moved x{:+.4f} y{:+.4f}); ground truth x={:.4f} "
                "y={:.4f}, so residual error is x{:+.4f} y{:+.4f}".format(
                    new_x, new_y, x, y, new_x - x, new_y - y,
                    gt_x, gt_y, new_x - gt_x, new_y - gt_y))
            x, y = new_x, new_y

        if self.config.visual_correction:
            # Re-measure relative to the gripper from the hover pose. A relative
            # measurement sees the object and the gripper through the same
            # miscalibrated transform, so the common-mode bias cancels and only
            # sensor noise remains.
            tx, ty = self._true_xy
            x = tx + self._rng.gauss(0.0, self.config.vision_noise_std)
            y = ty + self._rng.gauss(0.0, self.config.vision_noise_std)
            self.get_logger().info(
                "visual correction -> x={:.4f} y={:.4f} (bias cancelled)".format(x, y))

        self.set_stage("open")
        if not self.send_gripper(self.config.open_position, -1.0):
            gripper_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)
        self.wait_sim(0.5)

        self.remove_protection_sphere()
        self.wait_sim(0.5)

        self.set_stage("descend")
        descend_ok = True
        steps = max(1, self.config.descend_steps)
        for step in range(1, steps + 1):
            z_step = hover_z + (grasp_z - hover_z) * float(step) / steps
            if not self.move_to(x, y, z_step, quat, tolerance=0.003):
                descend_ok = False
                break
        if not descend_ok:
            planning_ok = False
            self.move_to(x, y, hover_z, quat, tolerance=0.02)
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("close")
        if not self.send_gripper(self.config.close_position, 20.0):
            gripper_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        if not self._confirm_bilateral_contact():
            self.set_stage("retreat")
            self.move_to(x, y, hover_z, quat, tolerance=0.02)
            return self._finish(planning_ok, gripper_ok, timed_out)

        with self._lock:
            self._grasp_relative = self._relative
            self._bilateral_confirmed = True

        self.set_stage("lift")
        lift_z = grasp_z + self.config.lift_height_m + 0.01
        # DIAGNOSTIC (coordinator-approved, one bounded test): 10x gentler
        # lift to test whether contact_lost_during_lift is acceleration-
        # driven (teleported-qpos arm path) or structural. Revert after.
        if not self.move_to(x, y, lift_z, quat, tolerance=0.005, velocity_scale=0.05, acceleration_scale=0.05):
            planning_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("hold")
        hold_completed = self.wait_sim(self.config.hold_duration_s)

        # Spec step 11 (classify) precedes step 12 (retreat), so the graded
        # metrics are frozen here; the retreat that follows must not change the
        # verdict it is reported alongside.
        with self._lock:
            self._metrics_frozen = True

        self.set_stage("retreat")
        self.move_to(x, y, hover_z, quat, tolerance=0.02)

        return self._finish(planning_ok, gripper_ok, timed_out, hold_completed)

    def _confirm_bilateral_contact(self):
        """Accept only bilateral contact sustained for the confirmation window."""
        start = self.sim_time()
        window_start = None
        # Wall-clock safety bound: if the simulator hangs or dies, sim_time()
        # freezes and the sim-time checks below never fire on their own.
        wall_deadline = time.time() + self.config.gripper_action_timeout_s * 20
        while True:
            if time.time() > wall_deadline:
                return False
            now = self.sim_time()
            if now is None:
                return False
            if now - start > self.config.contact_confirm_timeout_s:
                return False
            with self._lock:
                bilateral = getattr(self, "_summary", None) is not None and self._summary.bilateral
            if bilateral:
                if window_start is None:
                    window_start = now
                elif now - window_start >= self.config.confirm_window_s:
                    return True
            else:
                window_start = None
            time.sleep(0.01)

    def _finish(self, planning_ok, gripper_ok, timed_out, hold_completed=False,
                detector_ok=True):
        with self._lock:
            ratio = (
                self._hold_bilateral / self._hold_total if self._hold_total else 0.0
            )
            observations = TrialObservations(
                planning_ok=planning_ok,
                detector_ok=detector_ok,
                gripper_ok=gripper_ok,
                timed_out=timed_out,
                any_contact=self._any_contact,
                bilateral_confirmed=self._bilateral_confirmed,
                contact_lost_during_lift=self._contact_lost_during_lift,
                max_elevation_m=self._max_elevation,
                hold_completed=hold_completed,
                hold_bilateral_ratio=ratio,
                max_drift_m=self._max_drift,
            )

        category = classify_trial(observations, self.config)
        with self._lock:
            summary = self.recorder.finish(observations, category)
        self.get_logger().info(
            "trial {} -> {} (elevation {:.4f} m, drift {:.4f} m, hold ratio {:.2f})".format(
                self.trial_id,
                category.value,
                observations.max_elevation_m,
                observations.max_drift_m,
                observations.hold_bilateral_ratio,
            )
        )
        return summary


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsPicker()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        summary = node.run_trial()
        exit_code = 0 if summary and summary["success"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
