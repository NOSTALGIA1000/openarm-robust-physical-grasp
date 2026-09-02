from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # physics_mode semantics: the SRDF declares no object or marker joints, so
    # the robot state monitor no longer reports missing cube_x/y/z joints.
    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_moveit_config")
        .robot_description_semantic(file_path="config/openarm_physics.srdf")
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"monitor_dynamics": False},
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([run_move_group_node])
