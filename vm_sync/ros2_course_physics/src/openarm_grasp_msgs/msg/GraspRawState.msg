# Simulator ground-truth grasp telemetry. Raw and unclassified on purpose:
# contact classification and grasp evaluation live in openarm_grasp_physics
# so they stay unit-testable without a simulator.

# MuJoCo simulation time in seconds (mj_data->time), not wall clock.
float64 sim_time

# World pose of the body owning the tracked object geom.
geometry_msgs/Pose object_pose

# World pose of the gripper reference body.
geometry_msgs/Pose gripper_pose

# For every MuJoCo contact involving the tracked object geom, the name of the
# other geom, and the normal component of that contact force in newtons.
# The two arrays always have the same length.
string[] contact_geoms
float64[] contact_normal_forces
