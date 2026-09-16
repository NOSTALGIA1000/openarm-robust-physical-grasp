# Simulator ground-truth grasp telemetry. Raw and unclassified on purpose:
# contact classification and grasp evaluation live in openarm_grasp_physics
# so they stay unit-testable without a simulator.

# MuJoCo simulation time in seconds (mj_data->time), not wall clock.
float64 sim_time

# World pose of the body owning the tracked object geom.
geometry_msgs/Pose object_pose

# World pose of the gripper reference body.
#
# This is the gripper LINK ORIGIN, which is not where the jaws meet. The offset
# between the two depends on the whole arm pose, not just the commanded yaw, so
# the same gripper_pose minus object_pose means different things in different
# approach configurations - measured across batches it varies by 22mm, and
# between the open-loop and corrected pipelines at zero bias by 13.8mm. Use
# jaw_centre below for anything that compares offsets across configurations.
geometry_msgs/Pose gripper_pose

# Midpoint of the two finger collision geoms, in world coordinates: where the
# jaws will actually meet. Unlike gripper_pose this is a property of the jaws
# themselves, so an offset measured from it is comparable across approach
# poses, arms and pipelines.
#
# Zero when grasp_telemetry.left_finger_geom / right_finger_geom are unset or
# name geoms that are not in the model; jaw_centre_valid says which.
geometry_msgs/Point jaw_centre
bool jaw_centre_valid

# For every MuJoCo contact involving the tracked object geom, the name of the
# other geom, and the normal component of that contact force in newtons.
# The two arrays always have the same length.
string[] contact_geoms
float64[] contact_normal_forces
