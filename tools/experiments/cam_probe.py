"""Record camera-frame object/gripper measurements alongside telemetry.

Used to calibrate the constant offset between the gripper's optically measured
blob centroid and the gripper frame reported by kinematics. That offset is
fixed geometry; unlike hand-eye error it does not grow with calibration bias,
so it can be subtracted once.
"""
import csv
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from openarm_grasp_msgs.msg import GraspRawState


class CamProbe(Node):
    def __init__(self, path):
        super().__init__("cam_probe")
        self.obj_cam = None
        self.grip_cam = None
        self.seen = set()
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow([
            "sim_time",
            "obj_cam_x", "obj_cam_y", "obj_cam_z",
            "grip_cam_x", "grip_cam_y", "grip_cam_z",
            "grip_world_x", "grip_world_y", "grip_world_z",
            "obj_world_x", "obj_world_y", "obj_world_z",
        ])
        # Flush immediately: a killed run should still show the node started,
        # rather than losing everything in an unflushed buffer.
        self.fh.flush()
        self.create_subscription(Point, "/apple_location_cam", self._obj, 10)
        self.create_subscription(Point, "/gripper_location_cam", self._grip, 10)
        self.create_subscription(GraspRawState, "/grasp_raw_state", self._tel, 50)
        self.get_logger().info("cam_probe recording to %s" % path)

    def _note(self, name):
        if name not in self.seen:
            self.seen.add(name)
            self.get_logger().info("first message on %s" % name)

    def _obj(self, m):
        self._note("/apple_location_cam")
        self.obj_cam = (m.x, m.y, m.z)

    def _grip(self, m):
        self._note("/gripper_location_cam")
        self.grip_cam = (m.x, m.y, m.z)

    def _tel(self, msg):
        self._note("/grasp_raw_state")
        g = msg.gripper_pose.position
        o = msg.object_pose.position
        blank = ("", "", "")
        obj = ["%.5f" % v for v in self.obj_cam] if self.obj_cam else list(blank)
        grip = ["%.5f" % v for v in self.grip_cam] if self.grip_cam else list(blank)
        self.w.writerow([
            "%.4f" % msg.sim_time, *obj, *grip,
            "%.5f" % g.x, "%.5f" % g.y, "%.5f" % g.z,
            "%.5f" % o.x, "%.5f" % o.y, "%.5f" % o.z,
        ])
        self.fh.flush()


def main():
    rclpy.init()
    node = CamProbe(sys.argv[1] if len(sys.argv) > 1 else "/tmp/cam_probe.csv")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
