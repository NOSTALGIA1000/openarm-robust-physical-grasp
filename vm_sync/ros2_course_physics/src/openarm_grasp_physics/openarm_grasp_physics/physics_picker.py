#!/usr/bin/env python3
"""physics_mode single-trial grasp state machine.

There is no software attachment and no object pose publisher here. The object
is a MuJoCo free body; if it moves with the gripper, contact moved it.

All waits are measured in simulation time taken from the telemetry stream,
because the VM real-time factor is well below 1.0 and wall-clock sleeps are
therefore not evidence windows.
"""

import math
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
from std_msgs.msg import Header

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
        self.declare_parameter(
            "output_dir", "/home/nostalgia/ros2_ws_course_repro/results/physics_grasp"
        )

        self.config = GraspConfig()
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
        self._any_contact = False
        self._bilateral_confirmed = False
        self._contact_lost_during_lift = False
        self._hold_total = 0
        self._hold_bilateral = 0

        self._target = None
        self.create_subscription(Point, "/apple_location", self._on_target, 10)
        self.create_subscription(
            GraspRawState, "/grasp_raw_state", self._on_telemetry, 50
        )
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

    # ---------------- callbacks ----------------

    def _on_target(self, msg):
        if abs(msg.x) < 0.01 and abs(msg.y) < 0.01:
            return
        with self._lock:
            if self._target is None:
                self._target = (msg.x, msg.y, msg.z)

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
            self.recorder.mark_stage(stage, now)
        self.get_logger().info("stage -> {}".format(stage))

    # ---------------- motion primitives ----------------

    def move_to(self, x, y, z, quat, tolerance=0.01):
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.05
        goal.request.max_acceleration_scaling_factor = 0.05

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
        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05
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
            return False
        return result_future.result().result.error_code.val == 1

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

        deadline = time.time() + 60.0
        while self._target is None and time.time() < deadline:
            time.sleep(0.1)
        if self._target is None:
            self.get_logger().error("no target observation on /apple_location")
            return self._finish(False, gripper_ok, True)
        x, y, z_surface = self._target

        quat = (0.0, 1.0, 0.0, 0.0)  # tool z pointing down
        hover_z = z_surface + self.config.hover_offset_m
        grasp_z = z_surface + self.config.grasp_z_offset_m

        self.set_stage("hover")
        if not self.move_to(x, y, hover_z, quat, tolerance=0.005):
            planning_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("open")
        if not self.send_gripper(self.config.open_position, -1.0):
            gripper_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)
        self.wait_sim(0.5)

        self.remove_protection_sphere()
        self.wait_sim(0.5)

        self.set_stage("descend")
        if not self.move_to(x, y, grasp_z, quat, tolerance=0.01):
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
        if not self.move_to(x, y, lift_z, quat, tolerance=0.005):
            planning_ok = False
            return self._finish(planning_ok, gripper_ok, timed_out)

        self.set_stage("hold")
        hold_completed = self.wait_sim(self.config.hold_duration_s)

        self.set_stage("retreat")
        self.move_to(x, y, hover_z, quat, tolerance=0.02)

        return self._finish(planning_ok, gripper_ok, timed_out, hold_completed)

    def _confirm_bilateral_contact(self):
        """Accept only bilateral contact sustained for the confirmation window."""
        start = self.sim_time()
        window_start = None
        while True:
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

    def _finish(self, planning_ok, gripper_ok, timed_out, hold_completed=False):
        with self._lock:
            ratio = (
                self._hold_bilateral / self._hold_total if self._hold_total else 0.0
            )
            observations = TrialObservations(
                planning_ok=planning_ok,
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
